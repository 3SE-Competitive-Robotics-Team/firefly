// SPDX-License-Identifier: BSD-3-Clause

//! Offline assembly of the translated estimator, frontend and occupancy mapper.

mod image_codec;

use crate::{
    application::okvis2x_app_synchronous::{
        ApplicationFactory, EstimatorApplication, EstimatorInputs, MappingApplication,
    },
    config::{parameters::ViParameters, vi_parameters_reader::ViParametersReader},
    frontend::{
        frame_absolute_pose_sac_problem::OpenGvSolver,
        pipeline::{Frontend, RustBriskFeatureFactory},
    },
    mapping::{
        config_mapping::SubMapConfig,
        map_typedefs::SharedOccupancyMaps,
        object_mapping::OccupancyMapBackend,
        submapping_interface::{AlignCallback, SubmappingInterface},
    },
    output::trajectory_output::TopViewPainter,
    runtime::{
        threaded_slam::{
            AlignmentTerm, OccupancyMappingRuntime, ProcessingCore, SensorInputConfig, ThreadedSlam,
        },
        vi_interface::ViInterface,
    },
};
use firefly_error::{Error, ErrorKind, Result};
use firefly_supereight2::{data::DataConfig, map::MapConfig};
use jiff::Timestamp;
use nalgebra::{Vector2, Vector3};
use purecv::core::{DynamicMatrix, Matrix};
use std::{io, path::Path, sync::Arc};

pub use image_codec::RustImageCodec;

fn io_error(error: impl std::fmt::Display) -> io::Error {
    io::Error::other(error.to_string())
}

fn config_error(error: impl std::fmt::Display) -> Error {
    Error::new(ErrorKind::InvalidArgument, error.to_string())
        .with_operation("okvis2_x::configuration")
}

fn unsupported() -> io::Error {
    io::Error::new(
        io::ErrorKind::Unsupported,
        "the offline application has no native display backend",
    )
}

struct Configuration {
    parameters: ViParameters,
    submaps: SubMapConfig,
    map: MapConfig,
    data: DataConfig,
}

impl Configuration {
    fn parse(document: &toml::Value) -> Result<Self> {
        let text = toml::to_string(document).map_err(config_error)?;
        let mut reader = ViParametersReader::new();
        reader.read_config_str(&text, None).map_err(config_error)?;
        let parameters = reader
            .into_parameters()
            .ok_or_else(|| config_error("missing VI parameters"))?;
        if parameters.output.display.display_topview
            || parameters.output.display.display_matches
            || parameters.output.display.display_overhead
        {
            return Err(config_error(
                "disable display_topview, display_matches and display_overhead for offline operation",
            ));
        }
        if parameters.output.enable_submapping && parameters.lidar.is_some() {
            return Err(config_error(
                "the occupancy backend supports pinhole depth mapping; LiDAR integration needs a separate backend",
            ));
        }
        if parameters.frontend.use_cnn {
            return Err(config_error(
                "use_cnn requires a classification network backend",
            ));
        }
        let submaps = SubMapConfig::from_toml(&text).map_err(config_error)?;
        if submaps.sensor_measurement_downsampling == 0 || submaps.depth_image_res_downsampling == 0
        {
            return Err(config_error(
                "measurement and depth-image downsampling must be positive",
            ));
        }
        let mut map = MapConfig::default();
        map.read_toml(document)?;
        let mut data = DataConfig::default();
        data.read_toml(document)?;
        Ok(Self {
            parameters,
            submaps,
            map,
            data,
        })
    }
}

/// Shared occupancy handles connect the processing owner and integration worker.
pub struct OfflineApplicationFactory {
    config: Configuration,
    maps: SharedOccupancyMaps,
    painter: HeadlessPainter,
}

impl OfflineApplicationFactory {
    /// Loads and validates the combined VI/mapping TOML before starting workers.
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    pub fn load(path: &Path) -> Result<Self> {
        let mut config = None;
        let _: toml::Value = firefly_config::startup(
            path,
            firefly_observability::init,
            |document: &toml::Value| {
                config = Some(Configuration::parse(document)?);
                Ok(())
            },
        )?;
        Ok(Self {
            config: config
                .ok_or_else(|| config_error("configuration validation produced no parameters"))?,
            maps: SharedOccupancyMaps::default(),
            painter: HeadlessPainter,
        })
    }
}

impl ApplicationFactory for OfflineApplicationFactory {
    type MapConfig = MapConfig;
    type DataConfig = DataConfig;

    fn init_logging(&mut self, _executable: &Path) -> io::Result<()> {
        Ok(())
    }
    fn read_vi_parameters(&mut self, _path: &Path) -> io::Result<ViParameters> {
        Ok(self.config.parameters.clone())
    }
    fn read_mapping_config(
        &mut self,
        _path: &Path,
    ) -> io::Result<(SubMapConfig, MapConfig, DataConfig)> {
        Ok((
            self.config.submaps.clone(),
            self.config.map,
            self.config.data,
        ))
    }
    fn create_estimator(
        &mut self,
        parameters: &ViParameters,
        vocabulary_directory: &Path,
        submaps: &SubMapConfig,
    ) -> io::Result<Box<dyn EstimatorApplication>> {
        let parameters = parameters.clone();
        let submaps = submaps.clone();
        let input = SensorInputConfig::new(&parameters, &submaps);
        let directory = vocabulary_directory.to_path_buf();
        let maps = self.maps.clone();
        let estimator = ThreadedSlam::new(
            input,
            move || {
                let solver: OpenGvSolver = OpenGvSolver::default();
                let frontend = Frontend::new(
                    parameters.n_camera_system.num_cameras(),
                    directory,
                    solver,
                    RustBriskFeatureFactory,
                )?;
                Ok(ProcessingCore::new(
                    crate::ceres::vi_slam_backend::ViSlamBackend::default(),
                    frontend,
                    parameters,
                    submaps,
                    OccupancyMappingRuntime::new(maps),
                ))
            },
            None,
        )
        .map_err(|error| io_error(&error))?;
        Ok(Box::new(estimator))
    }
    fn create_mapping(
        &mut self,
        map: MapConfig,
        data: DataConfig,
        submaps: &SubMapConfig,
        parameters: &ViParameters,
    ) -> io::Result<Box<dyn MappingApplication>> {
        let mut backend = OccupancyMapBackend::new(map, data);
        backend.shared_maps = self.maps.clone();
        Ok(Box::new(SubmappingInterface::new(
            submaps.clone(),
            parameters,
            backend,
        )?))
    }
    fn image_codec(&self) -> Arc<dyn crate::datasets::dataset_reader::ImageCodec> {
        Arc::new(RustImageCodec)
    }
    fn top_view_painter(&self) -> &dyn TopViewPainter {
        &self.painter
    }
    fn show_image(&mut self, _name: &str, _image: &DynamicMatrix) -> io::Result<()> {
        Err(unsupported())
    }
    fn wait_key(&mut self, _milliseconds: i32) -> io::Result<()> {
        Err(unsupported())
    }
    fn print_timing(&self) {}
}

fn input_result(result: Result<bool>) -> bool {
    result.unwrap_or_else(|error| {
        log::error!("{error}");
        false
    })
}

impl EstimatorApplication for ThreadedSlam {
    fn input_callbacks(&self) -> EstimatorInputs {
        let input = self.sensor_input_handle();
        let (images, imu, gps, geodetic, lidar, depth) = (
            input.clone(),
            input.clone(),
            input.clone(),
            input.clone(),
            input.clone(),
            input,
        );
        EstimatorInputs {
            images: Arc::new(move |t, i, d| images.clone().add_images(t, i, d)),
            imu: Arc::new(move |t, a, w| imu.clone().add_imu_measurement(t, a, w)),
            gps: Some(Arc::new(move |t, p, s| {
                input_result(gps.add_gps_measurement(t, p, s))
            })),
            geodetic_gps: Some(Arc::new(move |t, lat, lon, alt, h, v| {
                input_result(geodetic.add_geodetic_gps_measurement(t, lat, lon, alt, h, v))
            })),
            lidar: Arc::new(move |t, r| lidar.clone().add_lidar_measurement(t, r)),
            depth: Arc::new(move |t, d, s| depth.clone().add_depth_measurement(t, d, s)),
        }
    }
    fn add_gps_measurement(&mut self, t: Timestamp, p: &Vector3<f64>, s: &Vector3<f64>) -> bool {
        input_result(ThreadedSlam::add_gps_measurement(self, t, p, s))
    }
    fn add_geodetic_gps_measurement(
        &mut self,
        t: Timestamp,
        lat: f64,
        lon: f64,
        alt: f64,
        h: f64,
        v: f64,
    ) -> bool {
        input_result(ThreadedSlam::add_geodetic_gps_measurement(
            self, t, lat, lon, alt, h, v,
        ))
    }
    fn process_frame(&mut self) -> io::Result<bool> {
        ThreadedSlam::process_frame(self).map_err(|error| io_error(&error))
    }
    fn set_final_trajectory_csv_file(&mut self, path: &Path) {
        ThreadedSlam::set_final_trajectory_csv_file(self, path, false);
    }
    fn set_map_csv_file(&mut self, path: &Path) {
        ThreadedSlam::set_map_csv_file(self, path);
    }
    fn write_final_trajectory_csv(&mut self) -> io::Result<()> {
        ThreadedSlam::write_final_trajectory_csv(self).map_err(|error| io_error(&error))
    }
    fn do_final_ba(&mut self) -> io::Result<()> {
        ThreadedSlam::do_final_ba(self).map_err(|error| io_error(&error))
    }
    fn save_map(&mut self) -> io::Result<()> {
        ThreadedSlam::save_map(self)
            .map(|_| ())
            .map_err(|error| io_error(&error))
    }
    fn alignment_callback(&self) -> AlignCallback {
        let input = self.sensor_input_handle();
        Arc::new(move |a, b, frame_a, frame_b, points, sigmas| {
            input
                .add_submap_alignment_constraints(AlignmentTerm {
                    submap_a: a,
                    submap_b: b,
                    frame_a_id: frame_a,
                    frame_b_id: frame_b,
                    points_b: points.to_vec(),
                    sensor_errors: sigmas.to_vec(),
                })
                .map_err(|error| io_error(&error))
        })
    }
    fn shutdown(&mut self) -> io::Result<()> {
        ThreadedSlam::shutdown(self).map_err(|error| io_error(&error))
    }
}

struct HeadlessPainter;
impl TopViewPainter for HeadlessPainter {
    fn line(
        &self,
        _: &mut Matrix<u8>,
        _: Vector2<f64>,
        _: Vector2<f64>,
        _: [f64; 3],
        _: i32,
    ) -> io::Result<()> {
        Err(unsupported())
    }
    fn circle(&self, _: &mut Matrix<u8>, _: Vector2<f64>, _: f64, _: [f64; 3]) -> io::Result<()> {
        Err(unsupported())
    }
    fn text(
        &self,
        _: &mut Matrix<u8>,
        _: &str,
        _: Vector2<f64>,
        _: f64,
        _: [f64; 3],
        _: i32,
    ) -> io::Result<()> {
        Err(unsupported())
    }
    fn diamond(&self, _: &mut Matrix<u8>, _: Vector2<f64>, _: [f64; 3], _: i32) -> io::Result<()> {
        Err(unsupported())
    }
}
pub mod okvis2x_app_network_synchronous;
pub mod okvis2x_app_synchronous;
