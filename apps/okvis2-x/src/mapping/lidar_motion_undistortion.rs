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
//! Per-ray IMU deskewing and point-cloud selection from `LidarMotionUndistortion.cpp`.
use super::{
    map_typedefs::SupereightMapType, submapping_utils::LidarMotionCompensation,
    voxel_grid_filter::voxel_down_sample_inplace,
};
use crate::{
    ceres::imu_error::IntegrationError,
    frames::frame_typedefs::{SpeedAndBias, StateId},
    kinematics::transformation::Transformation,
    sensors::measurements::{ImuMeasurementDeque, LidarMeasurementDeque},
};
use jiff::Timestamp;
use nalgebra::Vector3;
use rand::seq::SliceRandom;

/// Exact state fields read from `ViInterface::State` during deskewing.
pub trait LidarInitialState {
    fn id(&self) -> StateId;
    fn timestamp(&self) -> Timestamp;
    fn t_ws(&self) -> &Transformation;
    fn v_w(&self) -> Vector3<f64>;
    fn b_g(&self) -> Vector3<f64>;
    fn b_a(&self) -> Vector3<f64>;
}
#[derive(Clone, Debug)]
struct InitialState {
    id: StateId,
    timestamp: Timestamp,
    t_ws: Transformation,
    speed_bias: SpeedAndBias,
}
#[derive(Clone, Debug)]
pub struct LidarMotionUndistortion {
    initial_state: InitialState,
    t_ws_live: Transformation,
    t_sw_live: Transformation,
    t_sl: Transformation,
    lidar_scan: LidarMeasurementDeque,
    lidar_scan_post_processed: Vec<Vector3<f32>>,
    imu_measurements: ImuMeasurementDeque,
}
impl LidarMotionUndistortion {
    pub fn new(
        initial: &impl LidarInitialState,
        t_ws_live: Transformation,
        t_sl: Transformation,
        lidar_scan: LidarMeasurementDeque,
        imu_measurements: ImuMeasurementDeque,
    ) -> Self {
        let mut sb = SpeedAndBias::zeros();
        sb.fixed_rows_mut::<3>(0).copy_from(&initial.v_w());
        sb.fixed_rows_mut::<3>(3).copy_from(&initial.b_g());
        sb.fixed_rows_mut::<3>(6).copy_from(&initial.b_a());
        let initial_state = InitialState {
            id: initial.id(),
            timestamp: initial.timestamp(),
            t_ws: initial.t_ws().clone(),
            speed_bias: sb,
        };
        Self {
            initial_state,
            t_sw_live: t_ws_live.inverse(),
            t_ws_live,
            t_sl,
            lidar_scan,
            lidar_scan_post_processed: Vec::new(),
            imu_measurements,
        }
    }
    /// Appends processed points, matching the source's repeated-call behavior.
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn deskew(&mut self) -> Result<bool, IntegrationError> {
        let initial = &self.initial_state;
        if initial.id.value() == 0 || self.lidar_scan.is_empty() {
            return Ok(false);
        }
        let first = self
            .imu_measurements
            .front()
            .ok_or(IntegrationError::MissingMeasurements)?;
        let last = self
            .imu_measurements
            .back()
            .ok_or(IntegrationError::MissingMeasurements)?;
        if first.time_stamp > initial.timestamp {
            return Err(IntegrationError::StartNotCovered);
        }
        if last.time_stamp
            < self
                .lidar_scan
                .back()
                .ok_or(IntegrationError::MissingMeasurements)?
                .time_stamp
        {
            return Err(IntegrationError::EndNotCovered);
        }
        let mut propagator = LidarMotionCompensation::batched(initial.timestamp);
        let mut transform = self.t_sw_live.compose(&initial.t_ws).compose(&self.t_sl);
        self.lidar_scan_post_processed
            .reserve(self.lidar_scan.len());
        let mut index = 0;
        if self.lidar_scan[0].time_stamp == initial.timestamp {
            let p = transform.rotation_matrix() * self.lidar_scan[0].measurement.ray_measurement
                + transform.translation();
            self.lidar_scan_post_processed.push(p.cast());
            index = 1;
        }
        let mut previous = Timestamp::UNIX_EPOCH;
        while index < self.lidar_scan.len() {
            let measurement = &self.lidar_scan[index];
            if measurement.time_stamp < initial.timestamp {
                index += 1;
                continue;
            }
            if previous != measurement.time_stamp {
                propagator.append_to(
                    &self.imu_measurements,
                    &initial.t_ws,
                    &initial.speed_bias,
                    measurement.time_stamp,
                )?;
                let (pose, _, _) = propagator.get_state(&initial.t_ws, &initial.speed_bias);
                previous = measurement.time_stamp;
                transform = self.t_sw_live.compose(&pose).compose(&self.t_sl);
            }
            let p = transform.rotation_matrix() * measurement.measurement.ray_measurement
                + transform.translation();
            self.lidar_scan_post_processed.push(p.cast());
            index += 1;
        }
        Ok(true)
    }
    pub fn downsample(&mut self, num_output_points: usize, voxel_grid_resolution: f64) -> bool {
        if self.lidar_scan_post_processed.is_empty() {
            return false;
        }
        if !voxel_down_sample_inplace(&mut self.lidar_scan_post_processed, voxel_grid_resolution) {
            return false;
        }
        self.random_downsample(num_output_points);
        true
    }
    pub fn deskewed_downsampled_point_cloud(&mut self) -> &mut Vec<Vector3<f32>> {
        &mut self.lidar_scan_post_processed
    }
    pub fn random_downsample(&mut self, num_output_points: usize) {
        if self.lidar_scan_post_processed.len() > num_output_points {
            self.lidar_scan_post_processed.shuffle(&mut rand::rng());
            self.lidar_scan_post_processed.truncate(num_output_points);
        }
    }
    /// Keeps observed rays in order, regardless of the interpolated field's value.
    pub fn filter_observed(
        &mut self,
        map: &impl SupereightMapType,
        t_wm: &Transformation,
    ) -> usize {
        if self.lidar_scan_post_processed.is_empty() {
            return 0;
        }
        // Cast each input matrix before multiplication, as in the source.
        let transform =
            t_wm.inverse().matrix_3x4().cast::<f32>() * self.t_ws_live.matrix().cast::<f32>();
        let rotation = transform.fixed_columns::<3>(0);
        let translation = transform.column(3);
        self.lidar_scan_post_processed.retain(|point| {
            map.interp_field_safe(&(rotation * point + translation))
                .is_some()
        });
        self.lidar_scan_post_processed.len()
    }
}
