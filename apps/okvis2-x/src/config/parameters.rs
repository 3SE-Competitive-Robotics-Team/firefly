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

//! Sensor calibration and estimator configuration corresponding to `Parameters.hpp`.

use crate::{
    cameras::n_camera_system::{CameraType, NCameraSystem},
    kinematics::transformation::Transformation,
};
use nalgebra::{DVector, Vector2, Vector3};
use std::collections::BTreeSet;

#[derive(Clone, Debug)]
pub struct CameraCalibration {
    /// Camera-to-IMU transformation.
    pub t_sc: Transformation,
    pub image_dimension: Vector2<i32>,
    pub distortion_coefficients: DVector<f64>,
    pub focal_length: Vector2<f64>,
    pub principal_point: Vector2<f64>,
    pub camera_model: String,
    /// EUCM alpha and beta.
    pub eucm_parameters: Vector2<f64>,
    pub distortion_type: String,
    pub camera_type: CameraType,
}

#[derive(Clone, Debug)]
pub struct OnlineCalibrationParameters {
    pub do_extrinsics: bool,
    pub do_extrinsics_final_ba: bool,
    /// Extrinsic translation prior standard deviation, metres.
    pub sigma_r: f64,
    /// Extrinsic orientation prior standard deviation, radians.
    pub sigma_alpha: f64,
    pub sigma_r_final_ba: f64,
    pub sigma_alpha_final_ba: f64,
}

#[derive(Clone, Debug)]
pub struct CameraParameters {
    /// Maximum stereo timestamp difference, seconds.
    pub timestamp_tolerance: f64,
    pub sync_cameras: BTreeSet<usize>,
    pub stereo_indices: Vec<usize>,
    /// Correct timestamp = camera timestamp - image delay, seconds.
    pub image_delay: f64,
    pub online_calibration: OnlineCalibrationParameters,
}

#[derive(Clone, Debug)]
pub struct ImuParameters {
    pub r#use: bool,
    /// Body-from-IMU sensor transformation `T_BS`.
    pub t_bs: Transformation,
    /// Accelerometer saturation, metres per second squared.
    pub a_max: f64,
    /// Gyroscope saturation, radians per second.
    pub g_max: f64,
    pub sigma_g_c: f64,
    pub sigma_bg: f64,
    pub sigma_a_c: f64,
    pub sigma_ba: f64,
    pub sigma_gw_c: f64,
    pub sigma_aw_c: f64,
    pub g0: Vector3<f64>,
    pub a0: Vector3<f64>,
    pub g: f64,
    pub s_a: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct FrontendParameters {
    pub detection_threshold: f64,
    pub absolute_threshold: f64,
    pub matching_threshold: f64,
    pub octaves: i32,
    pub max_num_keypoints: i32,
    pub keyframe_overlap: f64,
    pub use_cnn: bool,
    pub parallelise_detection: bool,
    pub num_matching_threads: i32,
}

#[derive(Clone, Debug)]
pub struct EstimatorParameters {
    pub num_keyframes: i32,
    pub num_loop_closure_frames: i32,
    pub num_imu_frames: i32,
    pub do_loop_closures: bool,
    pub do_final_ba: bool,
    pub enforce_realtime: bool,
    pub realtime_min_iterations: i32,
    pub realtime_max_iterations: i32,
    pub realtime_time_limit: f64,
    pub realtime_num_threads: i32,
    pub full_graph_iterations: i32,
    pub full_graph_num_threads: i32,
    pub p_dbow: f64,
    pub drift_percentage_heuristic: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DisplayOptions {
    pub display_topview: bool,
    pub display_matches: bool,
    pub display_overhead: bool,
}
#[derive(Clone, Debug)]
pub struct OutputParameters {
    pub display: DisplayOptions,

    pub enable_submapping: bool,
}

#[derive(Clone, Debug)]
pub struct GpsParameters {
    pub r#type: String,
    pub r_sa: Vector3<f64>,
    /// Maximum estimated initial yaw error, degrees.
    pub yaw_error_threshold: f64,
    pub robust_gps_init: bool,
}

impl Default for GpsParameters {
    fn default() -> Self {
        Self {
            r#type: "none".into(),
            r_sa: Vector3::zeros(),
            yaw_error_threshold: 0.0,
            robust_gps_init: false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct LidarParameters {
    /// LiDAR-to-IMU transformation.
    pub t_sl: Transformation,
    pub elevation_resolution_angle: f32,
    pub azimuth_resolution_angle: f32,
}

#[derive(Clone)]
pub struct ViParameters {
    pub n_camera_system: NCameraSystem,
    pub camera: CameraParameters,
    pub imu: ImuParameters,
    pub gps: Option<GpsParameters>,
    pub lidar: Option<LidarParameters>,
    pub frontend: FrontendParameters,
    pub estimator: EstimatorParameters,
    pub output: OutputParameters,
    pub rgb: Option<CameraCalibration>,
}
