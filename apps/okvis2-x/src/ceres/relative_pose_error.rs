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

//! Relative pose prior corresponding to `RelativePoseError.cpp`.

use super::pose_error::square_root_information;
use nalgebra::{Isometry3, Matrix3, Quaternion, SMatrix, SVector};

use super::pose_local_parameterization::PoseManifold;
use crate::kinematics::{
    operators::{cross_mx, oplus, plus},
    transformation::{Transformation, normalized_quaternion, quaternion_inverse},
};

/// Prior on `inverse(T_WA) * T_WB`, with translation followed by rotation residuals.
#[derive(Clone, Debug)]
pub struct RelativePoseError {
    t_ab: Isometry3<f64>,
    information: SMatrix<f64, 6, 6>,
    square_root_information: SMatrix<f64, 6, 6>,
    covariance: Option<SMatrix<f64, 6, 6>>,
}

impl RelativePoseError {
    pub const NUM_RESIDUALS: usize = 6;

    /// Constructs a prior; pass `Isometry3::identity()` for an identity measurement.
    ///
    /// The weighting follows Eigen's unchecked LLT path; singular covariance is absent.
    #[must_use]
    pub fn new(information: SMatrix<f64, 6, 6>, first_from_second: Isometry3<f64>) -> Self {
        let mut prior = Self {
            t_ab: first_from_second,
            information: SMatrix::zeros(),
            square_root_information: SMatrix::zeros(),
            covariance: None,
        };
        prior.set_information(information);
        prior
    }

    /// Both variances must be positive and finite.
    #[must_use]
    pub fn from_variances(
        translation_variance: f64,
        rotation_variance: f64,
        first_from_second: Isometry3<f64>,
    ) -> Self {
        let mut information = SMatrix::<f64, 6, 6>::zeros();
        information
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(Matrix3::identity() * 1.0 / translation_variance));
        information
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(Matrix3::identity() * 1.0 / rotation_variance));
        Self::new(information, first_from_second)
    }

    /// Sets symmetric positive-definite information and derived weights.
    ///
    /// Singular covariance is absent; nonpositive pivots retain the partial LLT factor.
    pub fn set_information(&mut self, information: SMatrix<f64, 6, 6>) {
        self.information = information;
        self.covariance = information.try_inverse();
        self.square_root_information = square_root_information(information);
    }

    #[must_use]
    pub fn information(&self) -> &SMatrix<f64, 6, 6> {
        &self.information
    }
    #[must_use]
    pub fn covariance(&self) -> Option<&SMatrix<f64, 6, 6>> {
        self.covariance.as_ref()
    }
    #[must_use]
    pub fn t_ab(&self) -> &Isometry3<f64> {
        &self.t_ab
    }

    pub fn evaluate(
        &self,
        parameters: [&SVector<f64, 7>; 2],
        residuals: &mut SVector<f64, 6>,
        jacobians: [Option<&mut SMatrix<f64, 6, 7>>; 2],
    ) -> bool {
        self.evaluate_with_minimal_jacobians(parameters, residuals, jacobians, [None, None])
    }

    /// Each pose is `[tx, ty, tz, qx, qy, qz, qw]`.
    ///
    /// A minimal Jacobian is written only if its corresponding ambient Jacobian
    /// is requested, matching the C++ evaluation branches.
    pub fn evaluate_with_minimal_jacobians(
        &self,
        parameters: [&SVector<f64, 7>; 2],
        residuals: &mut SVector<f64, 6>,
        jacobians: [Option<&mut SMatrix<f64, 6, 7>>; 2],
        jacobians_minimal: [Option<&mut SMatrix<f64, 6, 6>>; 2],
    ) -> bool {
        let t_wa = pose_from_parameters(parameters[0]);
        let world_from_body = pose_from_parameters(parameters[1]);
        let t_aw = t_wa.inverse();
        let first_from_second = t_aw.compose(&world_from_body);
        let mut error = SVector::<f64, 6>::zeros();
        error
            .fixed_rows_mut::<3>(0)
            .copy_from(&(self.t_ab.translation.vector - first_from_second.translation()));
        let rotation_error =
            *self.t_ab.rotation.quaternion() * quaternion_inverse(&first_from_second.quaternion());
        error
            .fixed_rows_mut::<3>(3)
            .copy_from(&(2.0 * rotation_error.coords.fixed_rows::<3>(0)));
        *residuals = self.square_root_information * error;

        let body_from_world = world_from_body.inverse();
        let [jacobian_a, jacobian_b] = jacobians;
        let [jacobian_minimal_a, jacobian_minimal_b] = jacobians_minimal;
        if let Some(jacobian_a) = jacobian_a {
            let mut j_minimal = SMatrix::<f64, 6, 6>::identity();
            let c_aw = t_aw.rotation_matrix();
            j_minimal.fixed_view_mut::<3, 3>(0, 0).copy_from(&c_aw);
            j_minimal.fixed_view_mut::<3, 3>(0, 3).copy_from(
                &(-c_aw * cross_mx(&(world_from_body.translation() - t_wa.translation()))),
            );
            let q = *self.t_ab.rotation.quaternion() * body_from_world.quaternion();
            let rotation_jacobian = plus(&q) * oplus(&t_wa.quaternion());
            j_minimal
                .fixed_view_mut::<3, 3>(3, 3)
                .copy_from(&rotation_jacobian.fixed_view::<3, 3>(0, 0));
            let j_minimal = self.square_root_information * j_minimal;
            *jacobian_a = j_minimal * PoseManifold::minus_jacobian(parameters[0]);
            if let Some(jacobian_minimal_a) = jacobian_minimal_a {
                *jacobian_minimal_a = j_minimal;
            }
        }
        if let Some(jacobian_b) = jacobian_b {
            let mut j_minimal = SMatrix::<f64, 6, 6>::identity();
            let c_aw = t_aw.rotation_matrix();
            j_minimal.fixed_view_mut::<3, 3>(0, 0).copy_from(&(-c_aw));
            let q = *self.t_ab.rotation.quaternion() * body_from_world.quaternion();
            let rotation_jacobian = plus(&q) * oplus(&t_wa.quaternion());
            j_minimal
                .fixed_view_mut::<3, 3>(3, 3)
                .copy_from(&(-rotation_jacobian.fixed_view::<3, 3>(0, 0)));
            let j_minimal = self.square_root_information * j_minimal;
            *jacobian_b = j_minimal * PoseManifold::minus_jacobian(parameters[1]);
            if let Some(jacobian_minimal_b) = jacobian_minimal_b {
                *jacobian_minimal_b = j_minimal;
            }
        }
        true
    }

    #[must_use]
    pub const fn residual_dim(&self) -> usize {
        Self::NUM_RESIDUALS
    }
    #[must_use]
    pub const fn parameter_blocks(&self) -> usize {
        2
    }
    #[must_use]
    pub fn parameter_block_dim(&self, parameter_block_id: usize) -> usize {
        [7, 7][parameter_block_id]
    }
    #[must_use]
    pub const fn type_info(&self) -> &'static str {
        "RelativePoseError"
    }
}

fn pose_from_parameters(parameters: &SVector<f64, 7>) -> Transformation {
    let q = normalized_quaternion(Quaternion::new(
        parameters[6],
        parameters[3],
        parameters[4],
        parameters[5],
    ));
    Transformation::new(parameters.fixed_rows::<3>(0).into_owned(), q)
}
