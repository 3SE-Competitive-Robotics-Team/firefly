/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Synchronous application orchestration with explicit estimator/map/UI factories.
use crate::{
    config::parameters::ViParameters,
    datasets::{dataset_reader::ImageCodec, x_dataset_reader::XDatasetReader},
    deep_learning::deep_learning_processor::DeepLearningProcessor,
    frames::frame_typedefs::StateId,
    mapping::{
        config_mapping::SubMapConfig,
        submapping_interface::{
            AlignCallback, DepthFrames, SubmappingInterface, SupereightBackend,
        },
    },
    output::trajectory_output::{TopViewPainter, TrajectoryOutput},
    runtime::vi_interface::{OptimisedGraphCallback, State, TrackingState, ViInterface},
    sensors::vi_sensor_base::{DatasetReaderBase, ViSensorBase},
};
use jiff::Timestamp;
use nalgebra::Vector3;
use purecv::core::{DynamicData, DynamicMatrix};
use std::{
    collections::BTreeMap,
    ffi::OsString,
    fs::File,
    io,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};
pub type EstimatorDepthCallback =
    Arc<dyn Fn(Timestamp, &DynamicMatrix, Option<&DynamicMatrix>) -> bool + Send + Sync>;
/// Input closures own queue handles independently of the main estimator host.
#[derive(Clone)]
pub struct EstimatorInputs {
    pub images: crate::sensors::vi_sensor_base::ImageCallback,
    pub imu: crate::sensors::vi_sensor_base::ImuCallback,
    pub gps: Option<crate::sensors::vi_sensor_base::GpsCallback>,
    pub geodetic_gps: Option<crate::sensors::vi_sensor_base::GeodeticGpsCallback>,
    pub lidar: crate::datasets::x_dataset_reader::LidarCallback,
    pub depth: EstimatorDepthCallback,
}
pub type StateInput = Arc<
    dyn Fn(State, TrackingState, Arc<BTreeMap<StateId, State>>) -> io::Result<bool> + Send + Sync,
>;
pub type MappingDepthInput = Arc<dyn Fn(&mut DepthFrames) -> io::Result<bool> + Send + Sync>;
pub type MappingLidarInput = Arc<dyn Fn(Timestamp, Vector3<f64>) -> io::Result<bool> + Send + Sync>;
#[derive(Clone)]
pub struct MappingInputs {
    pub state: StateInput,
    pub lidar: MappingLidarInput,
    pub depth: MappingDepthInput,
}
/// Operations supplied by a concrete estimator host in addition to the common VI interface.
pub trait EstimatorApplication: ViInterface + Send {
    fn input_callbacks(&self) -> EstimatorInputs;
    fn add_gps_measurement(&mut self, t: Timestamp, p: &Vector3<f64>, error: &Vector3<f64>)
    -> bool;
    fn add_geodetic_gps_measurement(
        &mut self,
        t: Timestamp,
        lat: f64,
        lon: f64,
        alt: f64,
        h: f64,
        v: f64,
    ) -> bool;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn process_frame(&mut self) -> io::Result<bool>;
    fn set_final_trajectory_csv_file(&mut self, path: &Path);
    fn set_map_csv_file(&mut self, path: &Path);
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn write_final_trajectory_csv(&mut self) -> io::Result<()>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn do_final_ba(&mut self) -> io::Result<()>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn save_map(&mut self) -> io::Result<()>;
    fn alignment_callback(&self) -> AlignCallback;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn shutdown(&mut self) -> io::Result<()>;
}
pub trait MappingApplication: Send {
    fn input_callbacks(&self) -> MappingInputs;
    fn set_blocking(&mut self, b: bool);
    fn set_t_bs(&mut self, t: crate::kinematics::transformation::Transformation);
    fn set_align_callback(&mut self, cb: AlignCallback);
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn state_update(
        &mut self,
        state: State,
        tracking: TrackingState,
        updated: Arc<BTreeMap<StateId, State>>,
    ) -> io::Result<bool>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn add_lidar(&mut self, t: Timestamp, ray: Vector3<f64>) -> io::Result<bool>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn add_depth(&mut self, frames: &mut DepthFrames) -> io::Result<bool>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn start(&mut self) -> io::Result<bool>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn finished_integrating(&mut self) -> io::Result<bool>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn set_finished(&mut self) -> io::Result<()>;
    fn publish_submap_top_view(&mut self) -> Option<DynamicMatrix>;
    fn print_associations(&self);
}
impl<B: SupereightBackend> MappingApplication for SubmappingInterface<B> {
    fn input_callbacks(&self) -> MappingInputs {
        let h = self.input_handle();
        let state = h.clone();
        let depth = h.clone();
        MappingInputs {
            state: Arc::new(move |s, t, u| state.state_update_callback(s, t, u)),
            depth: Arc::new(move |f| depth.add_depth_measurement(f)),
            lidar: Arc::new(move |t, r| h.add_lidar_measurement(t, r)),
        }
    }
    fn set_blocking(&mut self, b: bool) {
        SubmappingInterface::set_blocking(self, b);
    }
    fn set_t_bs(&mut self, t: crate::kinematics::transformation::Transformation) {
        SubmappingInterface::set_t_bs(self, t);
    }
    fn set_align_callback(&mut self, cb: AlignCallback) {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .align_callback = Some(cb);
    }
    fn state_update(
        &mut self,
        s: State,
        t: TrackingState,
        u: Arc<BTreeMap<StateId, State>>,
    ) -> io::Result<bool> {
        self.state_update_callback(s, t, u)
    }
    fn add_lidar(&mut self, t: Timestamp, r: Vector3<f64>) -> io::Result<bool> {
        self.add_lidar_measurement(t, r)
    }
    fn add_depth(&mut self, f: &mut DepthFrames) -> io::Result<bool> {
        self.add_depth_measurement(f)
    }
    fn start(&mut self) -> io::Result<bool> {
        SubmappingInterface::start(self)
    }
    fn finished_integrating(&mut self) -> io::Result<bool> {
        SubmappingInterface::finished_integrating(self)
    }
    fn set_finished(&mut self) -> io::Result<()> {
        SubmappingInterface::set_finished(self)
    }
    fn publish_submap_top_view(&mut self) -> Option<DynamicMatrix> {
        SubmappingInterface::publish_submap_top_view(self)
    }
    fn print_associations(&self) {
        for (a, b) in &self
            .state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .associations
        {
            log::info!("Submap {a} aligned to {b}");
        }
    }
}
pub type EstimatorHandle = Arc<Mutex<Box<dyn EstimatorApplication>>>;
pub type MappingHandle = Arc<Mutex<Box<dyn MappingApplication>>>;
/// Resource construction is explicit: callers supply the numerical estimator,
/// Supereight implementation, image decoder, neural models and native display.
pub trait ApplicationFactory {
    type MapConfig;
    type DataConfig;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn init_logging(&mut self, executable: &Path) -> io::Result<()>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn read_vi_parameters(&mut self, path: &Path) -> io::Result<ViParameters>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn read_mapping_config(
        &mut self,
        path: &Path,
    ) -> io::Result<(SubMapConfig, Self::MapConfig, Self::DataConfig)>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn create_estimator(
        &mut self,
        parameters: &ViParameters,
        vocabulary_directory: &Path,
        submaps: &SubMapConfig,
    ) -> io::Result<Box<dyn EstimatorApplication>>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn create_mapping(
        &mut self,
        map: Self::MapConfig,
        data: Self::DataConfig,
        submaps: &SubMapConfig,
        parameters: &ViParameters,
    ) -> io::Result<Box<dyn MappingApplication>>;
    fn image_codec(&self) -> Arc<dyn ImageCodec>;
    fn top_view_painter(&self) -> &dyn TopViewPainter;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn show_image(&mut self, name: &str, image: &DynamicMatrix) -> io::Result<()>;
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    fn wait_key(&mut self, milliseconds: i32) -> io::Result<()>;
    fn print_timing(&self);
}
#[derive(Clone, Debug)]
pub struct Arguments {
    pub executable: PathBuf,
    pub vi_config: PathBuf,
    pub mapping_config: PathBuf,
    pub dataset: PathBuf,
    pub output: Option<PathBuf>,
}
impl Arguments {
    /// # Errors
    /// Returns an error if command-line/configuration input is invalid, an input/output file
    /// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
    pub fn parse(args: &[OsString]) -> io::Result<Self> {
        if args.len() != 4 && args.len() != 5 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "usage: app config-okvis2 config-se2 input-directory [output-directory]",
            ));
        }
        Ok(Self {
            executable: args[0].clone().into(),
            vi_config: args[1].clone().into(),
            mapping_config: args[2].clone().into(),
            dataset: args[3].clone().into(),
            output: args.get(4).map(PathBuf::from),
        })
    }
}
pub(crate) type Failure = Arc<Mutex<Option<io::Error>>>;
pub(crate) fn latch(f: &Failure, r: io::Result<bool>) -> bool {
    match r {
        Ok(b) => b,
        Err(e) => {
            f.lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .get_or_insert(e);
            false
        }
    }
}
pub(crate) fn check_failure(f: &Failure) -> io::Result<()> {
    if let Some(e) = f
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .take()
    {
        Err(e)
    } else {
        Ok(())
    }
}
pub(crate) fn omp_warning() {
    if let Some(n) = std::env::var_os("OMP_NUM_THREADS") {
        log::info!("OMP_NUM_THREADS is set to: {}", n.to_string_lossy());
    } else {
        log::warn!(
            "OMP_NUM_THREADS not set; set it to match available Supereight integration threads"
        );
        thread::sleep(Duration::from_secs(3));
    }
}
pub(crate) fn output_mode(p: &ViParameters) -> String {
    let mut mode = if p.estimator.do_loop_closures {
        "slam"
    } else {
        "vio"
    }
    .to_string();
    if p.camera.online_calibration.do_extrinsics {
        mode.push_str("-calib");
    }
    mode
}
pub(crate) fn vocabulary_directory(executable: &Path) -> io::Result<PathBuf> {
    let directory = executable.parent().unwrap_or_else(|| Path::new(""));
    File::open(directory.join("small_voc.yml.gz"))?;
    Ok(directory.into())
}
pub(crate) fn configure_estimator_output(e: &EstimatorHandle, path: &Path, mode: &str) {
    let mut e = e.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    e.set_blocking(true);
    e.set_final_trajectory_csv_file(&path.join(format!("okvis2-{mode}-final_trajectory.csv")));
    e.set_map_csv_file(&path.join(format!("okvis2-{mode}-final_map.csv")));
}
pub(crate) fn connect_reader(
    reader: &mut XDatasetReader,
    inputs: &EstimatorInputs,
    parameters: &ViParameters,
) {
    reader.set_imu_callback(inputs.imu.clone());
    reader.set_images_callback(inputs.images.clone());
    if let Some(gps) = &parameters.gps {
        if gps.r#type == "cartesian" {
            if let Some(cb) = &inputs.gps {
                reader.set_gps_callback(cb.clone());
            }
        } else if matches!(gps.r#type.as_str(), "geodetic" | "geodetic-leica")
            && let Some(cb) = &inputs.geodetic_gps
        {
            reader.set_geodetic_gps_callback(cb.clone());
        }
    }
}
pub(crate) fn depth_callback(
    e: EstimatorInputs,
    m: MappingInputs,
    failure: Failure,
) -> crate::sensors::vi_sensor_base::DepthImageCallback {
    Arc::new(move |frames| {
        let mut added = false;
        for ms in frames.values() {
            for m in ms {
                if let Some(depth) = &m.measurement.depth_image {
                    added = (e.depth)(m.time_stamp, depth, m.measurement.sigma_image.as_deref());
                }
            }
        }
        let map = latch(&failure, (m.depth)(frames));
        added && map
    })
}
pub(crate) fn set_graph_callback(
    e: &EstimatorHandle,
    m: Option<MappingInputs>,
    writer: Arc<Mutex<TrajectoryOutput>>,
    processor: Option<Arc<dyn DeepLearningProcessor>>,
    failure: Failure,
) {
    let cb: OptimisedGraphCallback = Arc::new(move |state, tracking, updated| {
        let result = writer
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .process_state(state, tracking, updated.clone())
            .map(|()| true);
        latch(&failure, result);
        if let Some(m) = &m {
            latch(
                &failure,
                (m.state)(state.clone(), tracking.clone(), updated.clone()),
            );
        }
        if let Some(p) = &processor {
            latch(
                &failure,
                p.state_update_callback(state, tracking, updated)
                    .map_err(|e| io::Error::other(e.to_string())),
            );
        }
    });
    e.lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .set_optimised_graph_callback(cb);
}
pub(crate) fn display_top_view(
    factory: &mut impl ApplicationFactory,
    writer: &Arc<Mutex<TrajectoryOutput>>,
) -> io::Result<bool> {
    let top = writer
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .draw_top_view(factory.top_view_painter())?;
    if let Some(top) = top {
        factory.show_image(
            "OKVIS 2 Top View",
            &DynamicMatrix {
                data: DynamicData::U8(top),
            },
        )?;
        Ok(true)
    } else {
        Ok(false)
    }
}
pub(crate) fn wait_for_integrators(
    m: Option<&MappingHandle>,
    processor: Option<&Arc<dyn DeepLearningProcessor>>,
    failure: &Failure,
) -> io::Result<()> {
    loop {
        check_failure(failure)?;
        let processor_done = if let Some(p) = processor {
            p.finished_processing()
                .map_err(|e| io::Error::other(e.to_string()))?
        } else {
            true
        };
        let map_done = if let Some(m) = m {
            m.lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .finished_integrating()?
        } else {
            true
        };
        if processor_done && map_done {
            break;
        }
        thread::sleep(Duration::from_millis(100));
        log::info!("waiting for integrator to finish!");
    }
    Ok(())
}
pub(crate) fn finish_output(
    (e, m, processor): (
        &EstimatorHandle,
        Option<&MappingHandle>,
        Option<&Arc<dyn DeepLearningProcessor>>,
    ),
    parameters: &ViParameters,
    (save, mode, codec): (&Path, &str, &dyn ImageCodec),
    failure: &Failure,
    start: Instant,
) -> io::Result<()> {
    wait_for_integrators(m, processor, failure)?;
    e.lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .write_final_trajectory_csv()?;
    if parameters.estimator.do_final_ba {
        {
            let mut e = e.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
            e.do_final_ba()?;
            e.set_final_trajectory_csv_file(
                &save.join(format!("okvis2-{mode}-final-ba_trajectory.csv")),
            );
            e.write_final_trajectory_csv()?;
        }
        wait_for_integrators(m, processor, failure)?;
    }
    log::info!(
        "total processing time OKVIS only {} s",
        start.elapsed().as_secs_f64()
    );
    if let Some(m) = m {
        let mut m = m.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        m.set_finished()?;
        m.print_associations();
        if let Some(plot) = m.publish_submap_top_view()
            && plot.rows() > 0
            && plot.cols() > 0
        {
            codec.write(
                &PathBuf::from(format!("{}submaps.png", save.display())),
                &plot,
            )?;
        }
    }
    e.lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .save_map()?;
    log::info!("total processing time {} s", start.elapsed().as_secs_f64());
    Ok(())
}
/// Executes the source startup, callback wiring, processing and final-BA lifecycle.
/// # Errors
/// Returns an error if command-line/configuration input is invalid, an input/output file
/// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
pub fn run(args: &[OsString], factory: &mut impl ApplicationFactory) -> io::Result<()> {
    run_with_vocabulary(args, factory, None)
}
/// Uses an explicit vocabulary location, or the executable directory when absent.
/// # Errors
/// Returns an error if command-line/configuration input is invalid, an input/output file
/// cannot be accessed, or a sensor, estimator, mapper or processor operation fails.
pub fn run_with_vocabulary(
    args: &[OsString],
    factory: &mut impl ApplicationFactory,
    vocabulary: Option<&Path>,
) -> io::Result<()> {
    let a = Arguments::parse(args)?;
    factory.init_logging(&a.executable)?;
    omp_warning();
    let parameters = factory.read_vi_parameters(&a.vi_config)?;
    let (mut config, map_config, data_config) = factory.read_mapping_config(&a.mapping_config)?;
    let is_submapping = parameters.output.enable_submapping;
    let lidar = parameters.lidar.is_some();
    let rgb = (0..parameters.n_camera_system.num_cameras())
        .any(|i| parameters.n_camera_system.camera_type(i).is_colour);
    let codec = factory.image_codec();
    let mut reader = XDatasetReader::new(
        &a.dataset,
        0.0,
        &parameters,
        is_submapping && lidar,
        (false, is_submapping && !lidar, is_submapping && rgb),
        codec.clone(),
    )?;
    let voc = if let Some(directory) = vocabulary {
        File::open(directory.join("small_voc.yml.gz"))?;
        directory.to_path_buf()
    } else {
        vocabulary_directory(&a.executable)?
    };
    let save = a.output.as_ref().unwrap_or(&a.dataset).clone();
    if a.output.is_some() {
        config.results_directory = save.to_string_lossy().into_owned();
    }
    let estimator = Arc::new(Mutex::new(factory.create_estimator(
        &parameters,
        &voc,
        &config,
    )?));
    configure_estimator_output(&estimator, &save, &output_mode(&parameters));
    let mapping = if is_submapping {
        let mut map = factory.create_mapping(map_config, data_config, &config, &parameters)?;
        map.set_t_bs(parameters.imu.t_bs.clone());
        map.set_blocking(true);
        if config.factors.use_map2map_factors || config.factors.use_map2live_factors {
            map.set_align_callback(
                estimator
                    .lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner)
                    .alignment_callback(),
            );
        }
        Some(Arc::new(Mutex::new(map)))
    } else {
        None
    };
    let mode = output_mode(&parameters);
    let writer = Arc::new(Mutex::new(TrajectoryOutput::with_csv(
        &save.join(format!("okvis2-{mode}_trajectory.csv")),
        false,
        parameters.output.display.display_topview,
    )?));
    let failure = Arc::new(Mutex::new(None));
    set_graph_callback(
        &estimator,
        mapping.as_ref().map(|m| {
            m.lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .input_callbacks()
        }),
        writer.clone(),
        None,
        failure.clone(),
    );
    let estimator_inputs = estimator
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .input_callbacks();
    connect_reader(&mut reader, &estimator_inputs, &parameters);
    connect_mapping_reader(
        &mut reader,
        mapping.as_ref(),
        &estimator_inputs,
        &config,
        &failure,
        lidar,
    )?;
    let result = process_dataset(
        &mut reader,
        factory,
        &RunOutput {
            estimator: &estimator,
            mapping: mapping.as_ref(),
            processor: None,
            writer: &writer,
            parameters: &parameters,
            save: &save,
            mode: &mode,
            codec: codec.as_ref(),
            failure: &failure,
        },
    );
    shutdown_dataset(&mut reader, &estimator, mapping.as_ref(), &failure, result)
}
fn shutdown_dataset(
    reader: &mut XDatasetReader,
    estimator: &EstimatorHandle,
    mapping: Option<&MappingHandle>,
    failure: &Failure,
    result: io::Result<()>,
) -> io::Result<()> {
    let reader_stop = reader.stop_streaming();
    let map_stop = if let Some(m) = &mapping {
        m.lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .set_finished()
    } else {
        Ok(())
    };
    let estimator_stop = estimator
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .shutdown();
    result?;
    reader_stop?;
    map_stop?;
    estimator_stop?;
    check_failure(failure)
}

fn connect_mapping_reader(
    reader: &mut XDatasetReader,
    mapping: Option<&MappingHandle>,
    estimator_inputs: &EstimatorInputs,
    config: &SubMapConfig,
    failure: &Failure,
    lidar: bool,
) -> io::Result<()> {
    if let Some(m) = &mapping {
        let mapping_inputs = m
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .input_callbacks();
        if lidar {
            let (e, m, f) = (
                estimator_inputs.clone(),
                mapping_inputs.clone(),
                failure.clone(),
            );
            let live = config.factors.use_map2live_factors;
            reader.set_lidar_callback(Arc::new(move |t, ray| {
                let added = if live { (e.lidar)(t, ray) } else { true };
                let map = latch(&f, (m.lidar)(t, *ray));
                added && map
            }));
        } else {
            reader.set_depth_image_callback(depth_callback(
                estimator_inputs.clone(),
                mapping_inputs,
                failure.clone(),
            ));
        }
        if !m
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .start()?
        {
            return Err(io::Error::other("submapping failed to start"));
        }
    }
    Ok(())
}

fn process_dataset(
    reader: &mut XDatasetReader,
    factory: &mut impl ApplicationFactory,
    output: &RunOutput<'_>,
) -> io::Result<()> {
    let RunOutput {
        estimator,
        mapping,
        processor: _,
        writer,
        parameters,
        save,
        mode,
        codec,
        failure,
    } = *output;

    if !reader.start_streaming()? {
        return Err(io::Error::other("dataset streaming failed"));
    }
    let start = Instant::now();
    let mut progress = 0;
    loop {
        check_failure(failure)?;
        estimator
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .process_frame()?;
        let mut images = BTreeMap::new();
        estimator
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .display(&mut images);
        for (name, image) in &images {
            factory.show_image(name, image)?;
        }
        let mut display = display_top_view(factory, writer)? || !images.is_empty();
        if let Some(m) = &mapping
            && let Some(plot) = m
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .publish_submap_top_view()
            && plot.rows() > 0
            && plot.cols() > 0
        {
            factory.show_image("Top View Submaps", &plot)?;
            display = true;
        }
        if display {
            factory.wait_key(2)?;
        }
        if !reader.is_streaming() {
            log::info!("DatasetReader Finished!");
            finish_output(
                (estimator, mapping, None),
                parameters,
                (save, mode, codec),
                failure,
                start,
            )?;
            break;
        }
        let new = (reader.completion() * 100.0) as i32;
        if new > progress {
            factory.print_timing();
            progress = new;
            log::info!("Progress: {progress}%");
        }
    }
    Ok(())
}

pub(crate) struct RunOutput<'a> {
    pub estimator: &'a EstimatorHandle,
    pub mapping: Option<&'a MappingHandle>,
    pub processor: Option<&'a Arc<dyn DeepLearningProcessor>>,
    pub writer: &'a Arc<Mutex<TrajectoryOutput>>,
    pub parameters: &'a ViParameters,
    pub save: &'a Path,
    pub mode: &'a str,
    pub codec: &'a dyn ImageCodec,
    pub failure: &'a Failure,
}
