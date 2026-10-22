/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Bounded FIFO matching measurements to reconstructed trajectory states.
use crate::{
    ceres::imu_error::IntegrationError,
    runtime::vi_interface::{State, Trajectory},
    sensors::measurements::Measurement,
};
use jiff::Timestamp;
use std::collections::VecDeque;
pub struct QueuedTrajectory<T> {
    measurement_queue: VecDeque<Measurement<T>>,
}
impl<T> Default for QueuedTrajectory<T> {
    fn default() -> Self {
        Self {
            measurement_queue: VecDeque::new(),
        }
    }
}
impl<T> QueuedTrajectory<T> {
    pub fn enqueue(&mut self, measurement: Measurement<T>) {
        self.measurement_queue.push_front(measurement);
        while self.measurement_queue.len() > 20 {
            log::warn!("RGB Image queue overloaded, popping last element");
            self.measurement_queue.pop_back();
        }
    }
    pub fn enqueue_data(&mut self, data: T, timestamp: Timestamp) {
        self.enqueue(Measurement::new(timestamp, data, -1));
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    ///
    /// # Panics
    /// Panics if an internally checked trajectory/state entry is absent from its required queue or
    /// map.
    pub fn get_states(
        &mut self,
        trajectory: &mut Trajectory,
    ) -> Result<Vec<(Measurement<T>, State)>, IntegrationError> {
        let mut states = Vec::new();
        while let Some(measurement) = self.measurement_queue.back() {
            let Some(state) = trajectory.get_state(measurement.time_stamp)? else {
                break;
            };
            states.push((self.measurement_queue.pop_back().unwrap(), state));
        }
        Ok(states)
    }
}
