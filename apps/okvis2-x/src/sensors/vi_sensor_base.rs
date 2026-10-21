/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Callback registration and common VI sensor / dataset reader interfaces.
use crate::{runtime::vi_interface::ImageMap, sensors::measurements::CameraMeasurement};
use jiff::Timestamp;
use nalgebra::Vector3;
use std::{collections::BTreeMap, io, path::Path, sync::Arc};
pub type ImageCallback = Arc<dyn Fn(Timestamp, &ImageMap, &ImageMap) -> bool + Send + Sync>;
pub type ImuCallback = Arc<dyn Fn(Timestamp, &Vector3<f64>, &Vector3<f64>) -> bool + Send + Sync>;
pub type GpsCallback = Arc<dyn Fn(Timestamp, &Vector3<f64>, &Vector3<f64>) -> bool + Send + Sync>;
pub type GeodeticGpsCallback =
    Arc<dyn Fn(Timestamp, f64, f64, f64, f64, f64) -> bool + Send + Sync>;
pub type DepthImageCallback =
    Arc<dyn Fn(&mut BTreeMap<usize, Vec<CameraMeasurement>>) -> bool + Send + Sync>;
#[derive(Clone, Default)]
pub struct SensorCallbacks {
    pub images_callbacks: Vec<ImageCallback>,
    pub imu_callbacks: Vec<ImuCallback>,
    pub gps_callback: Option<GpsCallback>,
    pub geodetic_gps_callback: Option<GeodeticGpsCallback>,
    pub depth_callback: Option<DepthImageCallback>,
}
impl SensorCallbacks {
    pub fn set_images_callback(&mut self, callback: ImageCallback) {
        self.images_callbacks.push(callback);
    }
    pub fn set_imu_callback(&mut self, callback: ImuCallback) {
        self.imu_callbacks.push(callback);
    }
    pub fn set_gps_callback(&mut self, callback: GpsCallback) {
        self.gps_callback = Some(callback);
    }
    pub fn set_geodetic_gps_callback(&mut self, callback: GeodeticGpsCallback) {
        self.geodetic_gps_callback = Some(callback);
    }
    pub fn set_depth_image_callback(&mut self, callback: DepthImageCallback) {
        self.depth_callback = Some(callback);
    }
}
pub trait ViSensorBase {
    fn callbacks(&self) -> &SensorCallbacks;
    fn callbacks_mut(&mut self) -> &mut SensorCallbacks;
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    fn start_streaming(&mut self) -> io::Result<bool>;
    /// # Errors
    /// Returns an error if device/configuration or sensor data is invalid, a required stream
    /// is unavailable, or the SDK, callback or device operation fails.
    fn stop_streaming(&mut self) -> io::Result<bool>;
    fn is_streaming(&self) -> bool;
    fn set_images_callback(&mut self, callback: ImageCallback) {
        self.callbacks_mut().set_images_callback(callback);
    }
    fn set_imu_callback(&mut self, callback: ImuCallback) {
        self.callbacks_mut().set_imu_callback(callback);
    }
    fn set_gps_callback(&mut self, callback: GpsCallback) {
        self.callbacks_mut().set_gps_callback(callback);
    }
    fn set_geodetic_gps_callback(&mut self, callback: GeodeticGpsCallback) {
        self.callbacks_mut().set_geodetic_gps_callback(callback);
    }
    fn set_depth_image_callback(&mut self, callback: DepthImageCallback) {
        self.callbacks_mut().set_depth_image_callback(callback);
    }
}
pub const GNSS_LEAP_NANOSECONDS: u64 = 0;
pub trait DatasetReaderBase: ViSensorBase {
    fn set_dataset_path(&mut self, path: &Path) -> bool;
    fn set_starting_delay(&mut self, seconds: f64) -> bool;
    fn completion(&self) -> f64;
}

/// Queue with the snapshot's drop-oldest and blocking producer policies.
pub(crate) struct BoundedQueue<T> {
    capacity: usize,
    state: std::sync::Mutex<(std::collections::VecDeque<T>, bool)>,
    changed: std::sync::Condvar,
}
impl<T> BoundedQueue<T> {
    pub(crate) fn new(capacity: usize) -> Self {
        Self {
            capacity,
            state: std::sync::Mutex::new((std::collections::VecDeque::new(), false)),
            changed: std::sync::Condvar::new(),
        }
    }
    pub(crate) fn push_dropping(&self, value: T) -> bool {
        let mut state = self
            .state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let dropped = state.0.len() >= self.capacity;
        if dropped {
            state.0.pop_front();
        }
        state.0.push_back(value);
        self.changed.notify_all();
        dropped
    }
    pub(crate) fn push_blocking(&self, value: T) -> bool {
        let mut state = self
            .state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        while state.0.len() >= self.capacity && !state.1 {
            state = self
                .changed
                .wait(state)
                .unwrap_or_else(std::sync::PoisonError::into_inner);
        }
        if state.1 {
            return false;
        }
        state.0.push_back(value);
        self.changed.notify_all();
        true
    }
    pub(crate) fn try_pop(&self) -> Option<T> {
        let mut state = self
            .state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let value = state.0.pop_front();
        self.changed.notify_all();
        value
    }
    pub(crate) fn pop(&self) -> Option<T> {
        let mut state = self
            .state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        while state.0.is_empty() && !state.1 {
            state = self
                .changed
                .wait(state)
                .unwrap_or_else(std::sync::PoisonError::into_inner);
        }
        let value = state.0.pop_front();
        self.changed.notify_all();
        value
    }
    pub(crate) fn is_empty(&self) -> bool {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .0
            .is_empty()
    }
    pub(crate) fn shutdown(&self) {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .1 = true;
        self.changed.notify_all();
    }
}
