/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENSE file for details
 */
//! Submap configuration corresponding to `config_mapping.hpp` and its impl.
//! TOML supplies the `general` table using the source configuration keys.
use serde::Deserialize;
use std::{fmt, path::Path};

#[derive(Clone, Copy, Debug, Default, serde::Deserialize)]
#[serde(default)]
pub struct MappingFactorConfig {
    #[serde(rename = "use_map_to_map_factors")]
    pub use_map2map_factors: bool,
    #[serde(rename = "use_map_to_live_factors")]
    pub use_map2live_factors: bool,
}
#[derive(Clone, Debug, Deserialize)]
#[serde(default)]
pub struct SubMapConfig {
    pub results_directory: String,
    pub write_mesh_output: bool,
    pub sensor_measurement_downsampling: usize,
    #[serde(rename = "depth_image_resolution_downsampling")]
    pub depth_image_res_downsampling: usize,
    pub submap_kf_threshold: usize,
    pub submap_overlap_ratio: f32,
    pub submap_min_frames: usize,
    #[serde(flatten)]
    pub factors: MappingFactorConfig,

    #[serde(rename = "n_factors_per_state")]
    pub num_submap_factors: usize,
    pub voxel_grid_resolution: f32,
    pub sensor_error: f32,
    pub use_uncertainty: bool,
    pub depth_scaling_factor: f32,
    pub near_plane: f32,
    pub far_plane: f32,
}
impl Default for SubMapConfig {
    fn default() -> Self {
        Self {
            results_directory: "/home/".into(),
            write_mesh_output: false,
            sensor_measurement_downsampling: 1,
            depth_image_res_downsampling: 1,
            submap_kf_threshold: 5,
            submap_overlap_ratio: 0.4,
            submap_min_frames: 1,
            factors: crate::mapping::config_mapping::MappingFactorConfig {
                use_map2map_factors: false,
                use_map2live_factors: false,
            },

            num_submap_factors: 200,
            voxel_grid_resolution: 0.1,
            sensor_error: 0.01,
            use_uncertainty: false,
            depth_scaling_factor: 1.0,
            near_plane: 0.1,
            far_plane: 5.0,
        }
    }
}
impl SubMapConfig {
    /// # Errors
    /// Returns the TOML decoding error for malformed configuration fields or values.
    pub fn from_toml(source: &str) -> Result<Self, toml::de::Error> {
        #[derive(Deserialize, Default)]
        #[serde(default)]
        struct Document {
            general: SubMapConfig,
        }
        toml::from_str::<Document>(source).map(|document| document.general)
    }
}
impl fmt::Display for SubMapConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "results_directory: {}", self.results_directory)?;
        writeln!(f, "write_mesh_output: {}", self.write_mesh_output)?;
        writeln!(
            f,
            "sensor_measurement_downsampling: {}",
            self.sensor_measurement_downsampling
        )?;
        writeln!(
            f,
            "depth_image_res_downsampling: {}",
            self.depth_image_res_downsampling
        )?;
        writeln!(f, "submap_kf_threshold: {}", self.submap_kf_threshold)?;
        writeln!(f, "submap_overlap_ratio: {}", self.submap_overlap_ratio)?;
        writeln!(f, "submap_min_frames: {}", self.submap_min_frames)?;
        writeln!(
            f,
            "use_map_to_map_factors: {}",
            self.factors.use_map2map_factors
        )?;
        writeln!(
            f,
            "use_map_to_live_factors: {}",
            self.factors.use_map2live_factors
        )?;
        writeln!(f, "n_factors_per_state: {}", self.num_submap_factors)?;
        writeln!(f, "voxel_grid_resolution: {}", self.voxel_grid_resolution)?;
        writeln!(f, "sensor_error: {}", self.sensor_error)?;
        writeln!(f, "use_uncertainty: {}", self.use_uncertainty)?;
        writeln!(f, "depth_scaling_factor: {}", self.depth_scaling_factor)?;
        writeln!(f, "far_plane: {}", self.far_plane)?;
        writeln!(f, "near_plane: {}", self.near_plane)
    }
}
/// Map/data/sensor backends own their parameter parsing.
pub trait ReadConfig: Default + fmt::Display {
    type Error;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn read_config(&mut self, path: &Path) -> Result<(), Self::Error>;
}
#[derive(Clone, Debug, Default)]
pub struct Config<M, D, S> {
    pub map: M,
    pub data: D,
    pub sensor: S,
}
impl<M: ReadConfig, D: ReadConfig<Error = M::Error>, S: ReadConfig<Error = M::Error>>
    Config<M, D, S>
{
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn from_file(path: &Path) -> Result<Self, M::Error> {
        let mut config = Self::default();
        config.map.read_config(path)?;
        config.data.read_config(path)?;
        config.sensor.read_config(path)?;
        Ok(config)
    }
}
impl<M: fmt::Display, D: fmt::Display, S: fmt::Display> fmt::Display for Config<M, D, S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Data config -----------------------\n{}Map config ------------------------\n{}Sensor config ---------------------\n{}",
            self.data, self.map, self.sensor
        )
    }
}
#[derive(Clone, Debug, Default)]
pub struct OkvisSubmapsConfig<M, D, S> {
    pub general: SubMapConfig,
    pub se_config: Config<M, D, S>,
}
impl<M: fmt::Display, D: fmt::Display, S: fmt::Display> fmt::Display
    for OkvisSubmapsConfig<M, D, S>
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "General submap config -----------------------\n{}SE2 config -----------------------\n{}",
            self.general, self.se_config
        )
    }
}
impl<M, D, S> OkvisSubmapsConfig<M, D, S> {
    /// General configuration is decoded once by the application startup loader.
    pub fn new(general: SubMapConfig, se_config: Config<M, D, S>) -> Self {
        Self { general, se_config }
    }
}

fn read_se_document(path: &Path) -> std::io::Result<toml::Value> {
    let source = match std::fs::read_to_string(path) {
        Ok(source) => source,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            log::info!(
                "supereight configuration absent; using defaults: {}",
                path.display()
            );
            return Ok(toml::Value::Table(toml::map::Map::default()));
        }
        Err(error) => return Err(error),
    };
    toml::from_str(&source)
        .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidData, error))
}
macro_rules! impl_se_read_config {
    ($($ty:path),+ $(,)?) => {$ (
        impl ReadConfig for $ty {
            type Error = std::io::Error;
            fn read_config(&mut self, path: &Path) -> Result<(), Self::Error> {
                self.read_toml(&read_se_document(path)?)
                    .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidData, error.to_string()))
            }
        }
    )+};
}
impl_se_read_config!(
    firefly_supereight2::map::MapConfig,
    firefly_supereight2::data::DataConfig,
    firefly_supereight2::sensor::SensorConfig,
    firefly_supereight2::pinhole_camera::PinholeConfig,
);
pub type OccupancyPinholeConfig = OkvisSubmapsConfig<
    firefly_supereight2::map::MapConfig,
    firefly_supereight2::data::DataConfig,
    firefly_supereight2::pinhole_camera::PinholeConfig,
>;
