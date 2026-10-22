/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Frontend detection, association, initialization and IMU propagation contract.

use crate::{
    config::parameters::{ImuParameters, ViParameters},
    frames::{frame::KeyPoint, frame_typedefs::SpeedAndBias, multi_frame::MultiFrame},
    kinematics::transformation::Transformation,
    sensors::measurements::ImuMeasurementDeque,
};
use jiff::Timestamp;
use nalgebra::SMatrix;
use std::{cell::RefCell, rc::Rc};

/// The concrete SLAM backend is supplied as the frontend's estimator type.
pub trait ViFrontendInterface {
    type Estimator;
    fn detect_and_describe(
        &mut self,
        camera_index: usize,
        frame_out: Rc<RefCell<MultiFrame>>,
        t_wc: &Transformation,
        keypoints: Option<&[KeyPoint]>,
    ) -> bool;
    fn data_association_and_initialization(
        &mut self,
        estimator: &mut Self::Estimator,
        params: &ViParameters,
        frames_in_out: Rc<RefCell<MultiFrame>>,
        kf_prior: bool,
        as_keyframe: &mut bool,
    ) -> bool;

    fn propagation(
        &self,
        imu_measurements: &ImuMeasurementDeque,
        imu_params: &ImuParameters,
        t_ws_propagated: &mut Transformation,
        speed_and_biases: &mut SpeedAndBias,
        interval: (Timestamp, Timestamp),
        derivatives: (
            Option<&mut SMatrix<f64, 15, 15>>,
            Option<&mut SMatrix<f64, 15, 15>>,
        ),
    ) -> bool;
}
