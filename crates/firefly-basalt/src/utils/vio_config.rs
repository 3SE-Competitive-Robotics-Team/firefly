// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
// Source: VladyslavUsenko/basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc,
// include/basalt/utils/vio_config.h and src/utils/vio_config.cpp.
// See docs/licenses/README.md.

use firefly_error::{Error, ErrorKind, Result};
use serde::{Deserialize, Serialize};
use std::{
    fs::File,
    io::{BufReader, BufWriter, Read, Write},
    path::Path,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LinearizationType {
    #[serde(rename = "ABS_QR")]
    AbsQr,
    #[serde(rename = "ABS_SC")]
    AbsSc,
    #[serde(rename = "REL_SC")]
    RelSc,
}

/// Basalt algorithm parameters with the upstream cereal field names.
/// All archive fields are required; constructing defaults is a separate operation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[expect(
    clippy::struct_excessive_bools,
    reason = "Preserve the upstream configuration fields and JSON schema"
)]
pub struct VioConfig {
    #[serde(rename = "config.optical_flow_type")]
    pub optical_flow_type: String,
    #[serde(rename = "config.optical_flow_detection_grid_size")]
    pub optical_flow_detection_grid_size: i32,
    #[serde(rename = "config.optical_flow_max_recovered_dist2")]
    #[serde(serialize_with = "serialize_float")]
    pub optical_flow_max_recovered_dist2: f32,
    #[serde(rename = "config.optical_flow_pattern")]
    pub optical_flow_pattern: i32,
    #[serde(rename = "config.optical_flow_max_iterations")]
    pub optical_flow_max_iterations: i32,
    #[serde(rename = "config.optical_flow_epipolar_error")]
    #[serde(serialize_with = "serialize_float")]
    pub optical_flow_epipolar_error: f32,
    #[serde(rename = "config.optical_flow_levels")]
    pub optical_flow_levels: i32,
    #[serde(rename = "config.optical_flow_skip_frames")]
    pub optical_flow_skip_frames: i32,
    #[serde(rename = "config.vio_linearization_type")]
    pub vio_linearization_type: LinearizationType,
    #[serde(rename = "config.vio_sqrt_marg")]
    pub vio_sqrt_marg: bool,
    #[serde(rename = "config.vio_max_states")]
    pub vio_max_states: i32,
    #[serde(rename = "config.vio_max_kfs")]
    pub vio_max_kfs: i32,
    #[serde(rename = "config.vio_min_frames_after_kf")]
    pub vio_min_frames_after_kf: i32,
    #[serde(rename = "config.vio_new_kf_keypoints_thresh")]
    #[serde(serialize_with = "serialize_float")]
    pub vio_new_kf_keypoints_thresh: f32,
    #[serde(rename = "config.vio_debug")]
    pub vio_debug: bool,
    #[serde(rename = "config.vio_extended_logging")]
    pub vio_extended_logging: bool,
    #[serde(rename = "config.vio_max_iterations")]
    pub vio_max_iterations: i32,
    #[serde(rename = "config.vio_obs_std_dev")]
    pub vio_obs_std_dev: f64,
    #[serde(rename = "config.vio_obs_huber_thresh")]
    pub vio_obs_huber_thresh: f64,
    #[serde(rename = "config.vio_min_triangulation_dist")]
    pub vio_min_triangulation_dist: f64,
    #[serde(rename = "config.vio_enforce_realtime")]
    pub vio_enforce_realtime: bool,
    #[serde(rename = "config.vio_use_lm")]
    pub vio_use_lm: bool,
    #[serde(rename = "config.vio_lm_lambda_initial")]
    pub vio_lm_lambda_initial: f64,
    #[serde(rename = "config.vio_lm_lambda_min")]
    pub vio_lm_lambda_min: f64,
    #[serde(rename = "config.vio_lm_lambda_max")]
    pub vio_lm_lambda_max: f64,
    #[serde(rename = "config.vio_scale_jacobian")]
    pub vio_scale_jacobian: bool,
    #[serde(rename = "config.vio_init_pose_weight")]
    pub vio_init_pose_weight: f64,
    #[serde(skip)]
    pub vio_init_yaw_weight: Option<f64>,
    #[serde(rename = "config.vio_init_ba_weight")]
    pub vio_init_ba_weight: f64,
    #[serde(rename = "config.vio_init_bg_weight")]
    pub vio_init_bg_weight: f64,
    #[serde(rename = "config.vio_marg_lost_landmarks")]
    pub vio_marg_lost_landmarks: bool,
    #[serde(rename = "config.vio_kf_marg_feature_ratio")]
    pub vio_kf_marg_feature_ratio: f64,
    #[serde(rename = "config.mapper_obs_std_dev")]
    pub mapper_obs_std_dev: f64,
    #[serde(rename = "config.mapper_obs_huber_thresh")]
    pub mapper_obs_huber_thresh: f64,
    #[serde(rename = "config.mapper_detection_num_points")]
    pub mapper_detection_num_points: i32,
    #[serde(rename = "config.mapper_num_frames_to_match")]
    pub mapper_num_frames_to_match: f64,
    #[serde(rename = "config.mapper_frames_to_match_threshold")]
    pub mapper_frames_to_match_threshold: f64,
    #[serde(rename = "config.mapper_min_matches")]
    pub mapper_min_matches: f64,
    #[serde(rename = "config.mapper_ransac_threshold")]
    pub mapper_ransac_threshold: f64,
    #[serde(rename = "config.mapper_min_track_length")]
    pub mapper_min_track_length: f64,
    #[serde(rename = "config.mapper_max_hamming_distance")]
    pub mapper_max_hamming_distance: f64,
    #[serde(rename = "config.mapper_second_best_test_ratio")]
    pub mapper_second_best_test_ratio: f64,
    #[serde(rename = "config.mapper_bow_num_bits")]
    pub mapper_bow_num_bits: i32,
    #[serde(rename = "config.mapper_min_triangulation_dist")]
    pub mapper_min_triangulation_dist: f64,
    #[serde(rename = "config.mapper_no_factor_weights")]
    pub mapper_no_factor_weights: bool,
    #[serde(rename = "config.mapper_use_factors")]
    pub mapper_use_factors: bool,
    #[serde(rename = "config.mapper_use_lm")]
    pub mapper_use_lm: bool,
    #[serde(rename = "config.mapper_lm_lambda_min")]
    pub mapper_lm_lambda_min: f64,
    #[serde(rename = "config.mapper_lm_lambda_max")]
    pub mapper_lm_lambda_max: f64,
}

impl Default for VioConfig {
    fn default() -> Self {
        Self {
            optical_flow_type: "frame_to_frame".to_owned(),
            optical_flow_detection_grid_size: 50,
            optical_flow_max_recovered_dist2: 0.09,
            optical_flow_pattern: 51,
            optical_flow_max_iterations: 5,
            optical_flow_epipolar_error: 0.005,
            optical_flow_levels: 3,
            optical_flow_skip_frames: 1,
            vio_linearization_type: LinearizationType::AbsQr,
            vio_sqrt_marg: true,
            vio_max_states: 3,
            vio_max_kfs: 7,
            vio_min_frames_after_kf: 5,
            vio_new_kf_keypoints_thresh: 0.7,
            vio_debug: false,
            vio_extended_logging: false,
            vio_max_iterations: 7,
            vio_obs_std_dev: 0.5,
            vio_obs_huber_thresh: 1.0,
            vio_min_triangulation_dist: 0.05,
            vio_enforce_realtime: false,
            vio_use_lm: false,
            vio_lm_lambda_initial: 1e-4,
            vio_lm_lambda_min: 1e-6,
            vio_lm_lambda_max: 1e2,
            vio_scale_jacobian: true,
            vio_init_pose_weight: 1e8,
            vio_init_yaw_weight: None,
            vio_init_ba_weight: 1e1,
            vio_init_bg_weight: 1e2,
            vio_marg_lost_landmarks: true,
            vio_kf_marg_feature_ratio: 0.1,
            mapper_obs_std_dev: 0.25,
            mapper_obs_huber_thresh: 1.5,
            mapper_detection_num_points: 800,
            mapper_num_frames_to_match: 30.0,
            mapper_frames_to_match_threshold: 0.04,
            mapper_min_matches: 20.0,
            mapper_ransac_threshold: 5e-5,
            mapper_min_track_length: 5.0,
            mapper_max_hamming_distance: 70.0,
            mapper_second_best_test_ratio: 1.2,
            mapper_bow_num_bits: 16,
            mapper_min_triangulation_dist: 0.07,
            mapper_no_factor_weights: false,
            mapper_use_factors: true,
            mapper_use_lm: false,
            mapper_lm_lambda_min: 1e-32,
            mapper_lm_lambda_max: 1e2,
        }
    }
}

#[derive(Serialize, Deserialize)]
struct Archive<T> {
    value0: T,
}

// Cereal promotes float fields to double before writing JSON numbers.
#[expect(
    clippy::trivially_copy_pass_by_ref,
    reason = "Serde serialize_with requires a borrowed field"
)]
fn serialize_float<S: serde::Serializer>(
    value: &f32,
    serializer: S,
) -> std::result::Result<S::Ok, S::Error> {
    serializer.serialize_f64(f64::from(*value))
}

impl VioConfig {
    /// Reads the upstream JSON archive, including its outer `value0` object.
    /// # Errors
    /// Returns an error for I/O failure, missing fields, malformed values or unknown enum names.
    pub fn from_reader(reader: impl Read) -> Result<Self> {
        let archive: Archive<Self> = serde_json::from_reader(reader).map_err(|error| {
            Error::new(
                ErrorKind::InvalidArgument,
                "Invalid Basalt configuration archive",
            )
            .with_operation("basalt.config.read")
            .with_source(error)
        })?;
        Ok(archive.value0)
    }

    /// Writes a JSON archive accepted by Basalt's cereal reader.
    /// # Errors
    /// Returns an error when serialization or writing fails.
    pub fn to_writer(&self, writer: impl Write) -> Result<()> {
        serde_json::to_writer_pretty(writer, &Archive { value0: self }).map_err(|error| {
            Error::new(
                ErrorKind::Internal,
                "Cannot write Basalt configuration archive",
            )
            .with_operation("basalt.config.write")
            .with_source(error)
        })
    }

    /// Loads all fields from an upstream JSON file.
    /// # Errors
    /// Returns an error for inaccessible files or invalid archives; leaves this value unchanged.
    pub fn load(&mut self, filename: impl AsRef<Path>) -> Result<()> {
        let filename = filename.as_ref();
        let file = File::open(filename)
            .map_err(|error| io_error(error, "basalt.config.load", filename))?;
        let config = Self::from_reader(BufReader::new(file)).map_err(|error| {
            error
                .with_operation("basalt.config.load")
                .with_context("path", filename.display())
        })?;
        *self = config;
        Ok(())
    }

    /// Saves all fields in the upstream JSON archive format.
    /// # Errors
    /// Returns an error if the file cannot be created, written, or flushed.
    pub fn save(&self, filename: impl AsRef<Path>) -> Result<()> {
        let filename = filename.as_ref();
        let file = File::create(filename)
            .map_err(|error| io_error(error, "basalt.config.save", filename))?;
        let mut writer = BufWriter::new(file);
        self.to_writer(&mut writer).map_err(|error| {
            error
                .with_operation("basalt.config.save")
                .with_context("path", filename.display())
        })?;
        writer
            .flush()
            .map_err(|error| io_error(error, "basalt.config.save", filename))
    }
}

fn io_error(error: std::io::Error, operation: &'static str, path: &Path) -> Error {
    let kind = if error.kind() == std::io::ErrorKind::NotFound {
        ErrorKind::NotFound
    } else {
        ErrorKind::Internal
    };
    Error::new(kind, "Basalt configuration file I/O failed")
        .with_operation(operation)
        .with_context("path", path.display())
        .with_source(error)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_cpp_archive_exactly() {
        let reference = include_str!("../../tests/reference/vio_config.json");
        assert_eq!(
            VioConfig::from_reader(reference.as_bytes()).unwrap(),
            VioConfig::default()
        );
        let mut bytes = Vec::new();
        VioConfig::default().to_writer(&mut bytes).unwrap();
        let actual: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        let expected: serde_json::Value = serde_json::from_str(reference).unwrap();
        assert_eq!(actual, expected);
    }

    #[test]
    fn archive_requires_fields_and_preserves_enum_names() {
        let mut bytes = Vec::new();
        VioConfig::default().to_writer(&mut bytes).unwrap();
        let mut json: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(json["value0"]["config.vio_linearization_type"], "ABS_QR");
        assert_eq!(
            VioConfig::from_reader(bytes.as_slice()).unwrap(),
            VioConfig::default()
        );
        for (name, expected) in [
            ("ABS_QR", LinearizationType::AbsQr),
            ("ABS_SC", LinearizationType::AbsSc),
            ("REL_SC", LinearizationType::RelSc),
        ] {
            json["value0"]["config.vio_linearization_type"] = name.into();
            assert_eq!(
                VioConfig::from_reader(json.to_string().as_bytes())
                    .unwrap()
                    .vio_linearization_type,
                expected
            );
        }
        json["value0"]["config.vio_linearization_type"] = "abs_qr".into();
        assert!(VioConfig::from_reader(json.to_string().as_bytes()).is_err());
        json["value0"]["config.vio_linearization_type"] = "ABS_QR".into();
        json["value0"]["unknown"] = 42.into();
        assert!(VioConfig::from_reader(json.to_string().as_bytes()).is_ok());
        json["value0"]
            .as_object_mut()
            .unwrap()
            .remove("config.vio_max_states");
        assert!(VioConfig::from_reader(json.to_string().as_bytes()).is_err());
        assert!(VioConfig::from_reader(b"{}".as_slice()).is_err());
    }
}
