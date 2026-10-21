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

//! Synchronous GPS factor corresponding to `GpsErrorSynchronous.cpp`.

use super::pose_error::square_root_information;
use super::pose_local_parameterization::PoseManifold;
use crate::{
    config::parameters::GpsParameters,
    kinematics::{
        operators::cross_mx,
        transformation::{Transformation, raw_rotation_matrix},
    },
};
use nalgebra::{Matrix3, Quaternion, SMatrix, SVector, Vector3};

#[derive(Clone, Debug)]
pub struct GpsErrorSynchronous {
    camera_id: u64,
    measurement: Vector3<f64>,
    gps_parameters: GpsParameters,
    information: Matrix3<f64>,
    square_root_information: Matrix3<f64>,
    covariance: Option<Matrix3<f64>>,
}
impl GpsErrorSynchronous {
    pub const NUM_RESIDUALS: usize = 3;
    #[must_use]
    pub fn new(
        camera_id: u64,
        measurement: Vector3<f64>,
        information: Matrix3<f64>,
        gps_parameters: GpsParameters,
    ) -> Self {
        let mut factor = Self {
            camera_id,
            measurement,
            gps_parameters,
            information,
            square_root_information: Matrix3::zeros(),
            covariance: None,
        };
        factor.set_information(information);
        factor
    }
    pub fn set_camera_id(&mut self, camera_id: u64) {
        self.camera_id = camera_id;
    }
    pub fn set_measurement(&mut self, measurement: Vector3<f64>) {
        self.measurement = measurement;
    }
    pub fn set_gps_parameters(&mut self, gps_parameters: GpsParameters) {
        self.gps_parameters = gps_parameters;
    }
    /// Uses Eigen-compatible partial LLT weighting; singular covariance is absent.
    pub fn set_information(&mut self, information: Matrix3<f64>) {
        self.information = information;
        self.covariance = information.try_inverse();
        self.square_root_information = square_root_information(information);
    }
    #[must_use]
    pub fn camera_id(&self) -> u64 {
        self.camera_id
    }
    #[must_use]
    pub fn measurement(&self) -> &Vector3<f64> {
        &self.measurement
    }
    #[must_use]
    pub fn gps_parameters(&self) -> &GpsParameters {
        &self.gps_parameters
    }
    #[must_use]
    pub fn information(&self) -> &Matrix3<f64> {
        &self.information
    }
    #[must_use]
    pub fn covariance(&self) -> Option<&Matrix3<f64>> {
        self.covariance.as_ref()
    }
    pub fn evaluate(
        &self,
        parameters: [&SVector<f64, 7>; 2],
        residuals: &mut Vector3<f64>,
        jacobians: [Option<&mut SMatrix<f64, 3, 7>>; 2],
    ) -> bool {
        self.evaluate_with_minimal_jacobians(parameters, residuals, jacobians, [None, None])
    }
    /// Quaternion coefficients are used without normalization in this factor.
    /// Minimal outputs are written only for requested ambient Jacobians.
    pub fn evaluate_with_minimal_jacobians(
        &self,
        parameters: [&SVector<f64, 7>; 2],
        residuals: &mut Vector3<f64>,
        jacobians: [Option<&mut SMatrix<f64, 3, 7>>; 2],
        jacobians_minimal: [Option<&mut SMatrix<f64, 3, 6>>; 2],
    ) -> bool {
        let t_ws_w = parameters[0].fixed_rows::<3>(0).into_owned();
        let c_ws = raw_rotation_matrix(&quaternion(parameters[0]));
        let t_gw_g = parameters[1].fixed_rows::<3>(0).into_owned();
        let c_gw = raw_rotation_matrix(&quaternion(parameters[1]));
        let r_sa = self.gps_parameters.r_sa;
        let error = self.measurement - (c_gw * (t_ws_w + c_ws * r_sa) + t_gw_g);
        *residuals = self.square_root_information * error;
        let [j0, j1] = jacobians;
        let [j0_minimal, j1_minimal] = jacobians_minimal;
        if let Some(j0) = j0 {
            let mut minimal = SMatrix::<f64, 3, 6>::zeros();
            minimal.fixed_view_mut::<3, 3>(0, 0).copy_from(&(-c_gw));
            minimal
                .fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&(c_gw * cross_mx(&(c_ws * r_sa))));
            *j0 = self.square_root_information
                * minimal
                * PoseManifold::minus_jacobian(parameters[0]);
            if let Some(j0_minimal) = j0_minimal {
                *j0_minimal = self.square_root_information * minimal;
            }
        }
        if let Some(j1) = j1 {
            let mut minimal = SMatrix::<f64, 3, 6>::zeros();
            minimal
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&(-Matrix3::identity()));
            minimal
                .fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&cross_mx(&(c_gw * (t_ws_w + c_ws * r_sa))));
            *j1 = self.square_root_information
                * minimal
                * PoseManifold::minus_jacobian(parameters[1]);
            if let Some(j1_minimal) = j1_minimal {
                *j1_minimal = self.square_root_information * minimal;
            }
        }
        true
    }
    /// Evaluates central differences at `1e-7`; the source routine always returns true.
    #[must_use]
    pub fn verify_jacobian_num_diff(
        &self,
        parameters: [&SVector<f64, 7>; 2],
        jacobians: Option<[Option<&SMatrix<f64, 3, 7>>; 2]>,
    ) -> bool {
        if let Some(jacobians) = jacobians {
            let t_ws = Transformation::new(
                parameters[0].fixed_rows::<3>(0).into_owned(),
                quaternion(parameters[0]),
            );
            let t_gw = Transformation::new(
                parameters[1].fixed_rows::<3>(0).into_owned(),
                quaternion(parameters[1]),
            );
            let weighted_error = |world: &Transformation, gps: &Transformation| {
                self.square_root_information
                    * (self.measurement
                        - (gps.rotation_matrix()
                            * (world.translation()
                                + world.rotation_matrix() * self.gps_parameters.r_sa)
                            + gps.translation()))
            };
            for block in 0..2 {
                if jacobians[block].is_some() {
                    let mut numerical = SMatrix::<f64, 3, 6>::zeros();
                    for i in 0..6 {
                        let mut delta = SVector::<f64, 6>::zeros();
                        delta[i] = 1e-7;
                        let base = if block == 0 { &t_ws } else { &t_gw };
                        let mut plus = base.clone();
                        plus.oplus(&delta);
                        delta[i] = -1e-7;
                        let mut minus = base.clone();
                        minus.oplus(&delta);
                        let (ep, em) = if block == 0 {
                            (weighted_error(&plus, &t_gw), weighted_error(&minus, &t_gw))
                        } else {
                            (weighted_error(&t_ws, &plus), weighted_error(&t_ws, &minus))
                        };
                        numerical.set_column(i, &((ep - em) / (2.0 * 1e-7)));
                    }
                    let _lifted = numerical * PoseManifold::minus_jacobian(parameters[block]);
                }
            }
        }
        true
    }
    #[must_use]
    pub const fn residual_dim(&self) -> usize {
        3
    }
    #[must_use]
    pub const fn parameter_blocks(&self) -> usize {
        2
    }
    #[must_use]
    pub fn parameter_block_dim(&self, block: usize) -> usize {
        [7, 7][block]
    }
    #[must_use]
    pub const fn type_info(&self) -> &'static str {
        "GpsErrorSynchronous"
    }
}
fn quaternion(parameters: &SVector<f64, 7>) -> Quaternion<f64> {
    Quaternion::new(parameters[6], parameters[3], parameters[4], parameters[5])
}
