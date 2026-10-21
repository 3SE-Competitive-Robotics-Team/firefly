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

//! Speed and IMU-bias prior corresponding to `SpeedAndBiasError.cpp`.

use super::pose_error::square_root_information;
use nalgebra::{SMatrix, SVector};

/// Prior on `[velocity, gyroscope bias, accelerometer bias]`, three entries each.
#[derive(Clone, Debug)]
pub struct SpeedAndBiasError {
    measurement: SVector<f64, 9>,
    information: SMatrix<f64, 9, 9>,
    square_root_information: SMatrix<f64, 9, 9>,
    covariance: Option<SMatrix<f64, 9, 9>>,
}

impl SpeedAndBiasError {
    pub const NUM_RESIDUALS: usize = 9;

    /// Constructs a prior from a symmetric positive-definite information matrix.
    ///
    /// Singular covariance is absent; nonpositive pivots retain the partial LLT factor.
    #[must_use]
    pub fn new(measurement: SVector<f64, 9>, information: SMatrix<f64, 9, 9>) -> Self {
        let mut prior = Self {
            measurement,
            information: SMatrix::zeros(),
            square_root_information: SMatrix::zeros(),
            covariance: None,
        };
        prior.set_information(information);
        prior
    }

    /// Constructs a block-diagonal prior; all variances must be positive and finite.
    #[must_use]
    pub fn from_variances(
        measurement: SVector<f64, 9>,
        speed_variance: f64,
        gyr_bias_variance: f64,
        acc_bias_variance: f64,
    ) -> Self {
        let mut information = SMatrix::<f64, 9, 9>::zeros();
        information
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(SMatrix::<f64, 3, 3>::identity() * 1.0 / speed_variance));
        information
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(SMatrix::<f64, 3, 3>::identity() * 1.0 / gyr_bias_variance));
        information
            .fixed_view_mut::<3, 3>(6, 6)
            .copy_from(&(SMatrix::<f64, 3, 3>::identity() * 1.0 / acc_bias_variance));
        Self::new(measurement, information)
    }

    pub fn set_measurement(&mut self, measurement: SVector<f64, 9>) {
        self.measurement = measurement;
    }

    /// Sets a symmetric positive-definite information matrix and its derived weights.
    ///
    /// Singular covariance is absent; nonpositive pivots retain the partial LLT factor.
    pub fn set_information(&mut self, information: SMatrix<f64, 9, 9>) {
        self.information = information;
        self.covariance = information.try_inverse();
        self.square_root_information = square_root_information(information);
    }

    #[must_use]
    pub fn measurement(&self) -> &SVector<f64, 9> {
        &self.measurement
    }

    #[must_use]
    pub fn information(&self) -> &SMatrix<f64, 9, 9> {
        &self.information
    }

    #[must_use]
    pub fn covariance(&self) -> Option<&SMatrix<f64, 9, 9>> {
        self.covariance.as_ref()
    }

    pub fn evaluate(
        &self,
        estimate: &SVector<f64, 9>,
        residuals: &mut SVector<f64, 9>,
        jacobian: Option<&mut SMatrix<f64, 9, 9>>,
    ) -> bool {
        self.evaluate_with_minimal_jacobians(estimate, residuals, jacobian, None)
    }

    /// Weights `measurement - estimate`; both Jacobians are minus the upper Cholesky factor.
    pub fn evaluate_with_minimal_jacobians(
        &self,
        estimate: &SVector<f64, 9>,
        residuals: &mut SVector<f64, 9>,
        jacobian: Option<&mut SMatrix<f64, 9, 9>>,
        jacobian_minimal: Option<&mut SMatrix<f64, 9, 9>>,
    ) -> bool {
        let error = self.measurement - estimate;
        *residuals = self.square_root_information * error;

        if let Some(jacobian) = jacobian {
            *jacobian = -self.square_root_information * SMatrix::<f64, 9, 9>::identity();
        }
        if let Some(jacobian_minimal) = jacobian_minimal {
            *jacobian_minimal = -self.square_root_information * SMatrix::<f64, 9, 9>::identity();
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
        [9][parameter_block_id]
    }

    #[must_use]
    pub const fn type_info(&self) -> &'static str {
        "SpeedAndBiasError"
    }
}
