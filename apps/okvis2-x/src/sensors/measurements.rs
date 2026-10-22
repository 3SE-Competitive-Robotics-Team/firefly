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

//! Timestamped sensor data corresponding to `Measurements.hpp`.

use jiff::Timestamp;
use nalgebra::{Matrix3, Vector3};
use purecv::{core::DynamicMatrix, features2d::KeyPoint};
use std::{
    collections::{BTreeMap, VecDeque},
    sync::Arc,
};

#[derive(Clone, Debug)]
pub struct Measurement<T> {
    pub time_stamp: Timestamp,
    pub measurement: T,
    pub sensor_id: i32,
}

impl<T> Measurement<T> {
    pub fn new(time_stamp: Timestamp, measurement: T, sensor_id: i32) -> Self {
        Self {
            time_stamp,
            measurement,
            sensor_id,
        }
    }
}

impl<T: Default> Default for Measurement<T> {
    fn default() -> Self {
        Self::new(Timestamp::UNIX_EPOCH, T::default(), -1)
    }
}

/// Construct with actual readings; an absent sample is `Option<Measurement<_>>`.
#[derive(Clone, Debug)]
pub struct ImuSensorReadings {
    pub gyroscopes: Vector3<f64>,
    pub accelerometers: Vector3<f64>,
}
impl ImuSensorReadings {
    #[must_use]
    pub fn new(gyroscopes: Vector3<f64>, accelerometers: Vector3<f64>) -> Self {
        Self {
            gyroscopes,
            accelerometers,
        }
    }
}

#[derive(Clone, Debug)]
pub struct LidarSensorReadings {
    pub ray_measurement: Vector3<f64>,
    pub intensity: u32,
}
impl LidarSensorReadings {
    #[must_use]
    pub fn new(ray_measurement: Vector3<f64>, intensity: u32) -> Self {
        Self {
            ray_measurement,
            intensity,
        }
    }
    #[must_use]
    pub fn from_ray(ray_measurement: Vector3<f64>) -> Self {
        Self::new(ray_measurement, 0)
    }
}

#[derive(Clone, Debug, Default)]
pub struct DeepLearningImageData {
    pub shape: Vec<i64>,
    pub data: Option<Arc<[f32]>>,
}

#[derive(Clone, Debug, Default)]
pub struct CameraData {
    pub image: Option<Arc<DynamicMatrix>>,
    pub depth_image: Option<Arc<DynamicMatrix>>,
    pub sigma_image: Option<Arc<DynamicMatrix>>,
    pub keypoints: Vec<KeyPoint>,
    pub deep_learning_data: BTreeMap<String, DeepLearningImageData>,
    /// C++ supplies no initial value for this flag.
    pub delivers_keypoints: Option<bool>,
}

pub type ImuMeasurement = Measurement<ImuSensorReadings>;
pub type ImuMeasurementDeque = VecDeque<ImuMeasurement>;
pub type LidarMeasurement = Measurement<LidarSensorReadings>;
pub type LidarMeasurementDeque = VecDeque<LidarMeasurement>;
pub type CameraMeasurement = Measurement<CameraData>;
pub type DepthMeasurementDeque = VecDeque<CameraMeasurement>;

#[derive(Clone, Debug, Default)]
pub struct KeypointData {
    pub keypoints: Vec<KeyPoint>,
    pub landmark_ids: Vec<u64>,
    pub descriptors: Option<Arc<DynamicMatrix>>,
}

#[derive(Clone, Debug, Default)]
pub struct FrameData {
    pub image: CameraData,
    pub keypoints: KeypointData,
}
pub type FrameDataPtr = Arc<FrameData>;

#[derive(Clone, Debug)]
pub struct GpsSensorReadings {
    pub position: Vector3<f64>,
    pub covariances: Matrix3<f64>,
    pub latitude: f64,
    /// Longitude; corresponds to the C++ field spelled `longitdue`.
    pub longitude: f64,
    pub height: f64,
    pub horizontal_accuracy: f64,
    pub vertical_accuracy: f64,
}

impl GpsSensorReadings {
    /// Cartesian constructor with the explicit invalid geodetic sentinels.
    #[must_use]
    pub fn from_cartesian(position: Vector3<f64>, covariances: Matrix3<f64>) -> Self {
        Self {
            position,
            covariances,
            latitude: -1.0,
            longitude: -1.0,
            height: -1.0,
            horizontal_accuracy: -1.0,
            vertical_accuracy: -1.0,
        }
    }

    /// The per-axis constructor retains the in-class geodetic defaults of zero.
    #[must_use]
    pub fn from_cartesian_sigmas(
        position: Vector3<f64>,
        sigma_x: f64,
        sigma_y: f64,
        sigma_z: f64,
    ) -> Self {
        Self {
            position,
            covariances: Matrix3::from_diagonal(&Vector3::new(
                sigma_x * sigma_x,
                sigma_y * sigma_y,
                sigma_z * sigma_z,
            )),
            latitude: 0.0,
            longitude: 0.0,
            height: 0.0,
            horizontal_accuracy: 0.0,
            vertical_accuracy: 0.0,
        }
    }

    #[must_use]
    pub fn from_geodetic(
        latitude: f64,
        longitude: f64,
        height: f64,
        horizontal_accuracy: f64,
        vertical_accuracy: f64,
    ) -> Self {
        Self {
            position: Vector3::zeros(),
            covariances: Matrix3::from_diagonal(&Vector3::new(
                horizontal_accuracy * horizontal_accuracy,
                horizontal_accuracy * horizontal_accuracy,
                vertical_accuracy * vertical_accuracy,
            )),
            latitude,
            longitude,
            height,
            horizontal_accuracy,
            vertical_accuracy,
        }
    }

    pub fn set_position(&mut self, x: f64, y: f64, z: f64) {
        self.position = Vector3::new(x, y, z);
    }
}
pub type GpsMeasurement = Measurement<GpsSensorReadings>;
pub type GpsMeasurementDeque = VecDeque<GpsMeasurement>;
