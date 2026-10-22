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

//! Landmark prior corresponding to `HomogeneousPointError.cpp`.

use super::pose_error::square_root_information;
use nalgebra::{Matrix3, SMatrix, Vector3, Vector4};

use super::homogeneous_point_local_parameterization::HomogeneousPointManifold;

/// Prior on a homogeneous landmark with fixed fourth coordinate.
#[derive(Clone, Debug)]
pub struct HomogeneousPointError {
    measurement: Vector4<f64>,
    information: Matrix3<f64>,
    square_root_information: Matrix3<f64>,
    covariance: Option<Matrix3<f64>>,
}

impl HomogeneousPointError {
    pub const NUM_RESIDUALS: usize = 3;

    /// Constructs a prior from a symmetric positive-definite information matrix.
    ///
    /// Singular covariance is absent; nonpositive pivots retain the partial LLT factor.
    #[must_use]
    pub fn new(measurement: Vector4<f64>, information: Matrix3<f64>) -> Self {
        let mut prior = Self {
            measurement,
            information: Matrix3::zeros(),
            square_root_information: Matrix3::zeros(),
            covariance: None,
        };
        prior.set_information(information);
        prior
    }

    /// Constructs an isotropic prior; `variance` must be positive and finite.
    #[must_use]
    pub fn from_variance(measurement: Vector4<f64>, variance: f64) -> Self {
        Self::new(measurement, Matrix3::identity() * 1.0 / variance)
    }

    pub fn set_measurement(&mut self, measurement: Vector4<f64>) {
        self.measurement = measurement;
    }

    /// Sets a symmetric positive-definite information matrix and its derived weights.
    ///
    /// Singular covariance is absent; nonpositive pivots retain the partial LLT factor.
    pub fn set_information(&mut self, information: Matrix3<f64>) {
        self.information = information;
        self.covariance = information.try_inverse();
        self.square_root_information = square_root_information(information);
    }

    #[must_use]
    pub fn measurement(&self) -> &Vector4<f64> {
        &self.measurement
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
        estimate: &Vector4<f64>,
        residuals: &mut Vector3<f64>,
        jacobian: Option<&mut SMatrix<f64, 3, 4>>,
    ) -> bool {
        self.evaluate_with_minimal_jacobians(estimate, residuals, jacobian, None)
    }

    /// Weights `estimate - measurement` in the first three coordinates.
    ///
    /// Estimate and measurement must have equal fourth coordinates. As in C++,
    /// the minimal Jacobian is written only when the ambient Jacobian is requested.
    pub fn evaluate_with_minimal_jacobians(
        &self,
        estimate: &Vector4<f64>,
        residuals: &mut Vector3<f64>,
        jacobian: Option<&mut SMatrix<f64, 3, 4>>,
        jacobian_minimal: Option<&mut Matrix3<f64>>,
    ) -> bool {
        let error = HomogeneousPointManifold::minus(estimate, &self.measurement);
        *residuals = self.square_root_information * error;

        if let Some(jacobian) = jacobian {
            let j_lift = HomogeneousPointManifold::minus_jacobian(estimate);
            let j_plus = HomogeneousPointManifold::plus_jacobian(estimate);
            let j_minimal = j_lift * j_plus;
            let j_minimal = self.square_root_information * j_minimal;
            *jacobian = j_minimal * j_lift;

            if let Some(jacobian_minimal) = jacobian_minimal {
                *jacobian_minimal = j_minimal;
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
        1
    }

    /// Returns the single parameter block's size; panics for a nonzero index.
    #[must_use]
    pub fn parameter_block_dim(&self, parameter_block_id: usize) -> usize {
        [4][parameter_block_id]
    }

    #[must_use]
    pub const fn type_info(&self) -> &'static str {
        "HomogeneousPointError"
    }
}
