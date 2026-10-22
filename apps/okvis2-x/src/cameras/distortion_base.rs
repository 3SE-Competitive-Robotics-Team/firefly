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

//! Distortion-model interface corresponding to `DistortionBase.hpp`.

use nalgebra::{DVector, Matrix2, Matrix2xX, Vector2};

/// Maps normalized image coordinates and exposes point and parameter Jacobians.
pub trait DistortionBase: Send + Sync {
    fn set_parameters(&mut self, parameters: &DVector<f64>) -> bool;
    fn get_parameters(&self, parameters: &mut DVector<f64>) -> bool;
    fn type_name(&self) -> &'static str;
    fn num_distortion_intrinsics(&self) -> usize;

    fn distort(&self, point_undistorted: &Vector2<f64>, point_distorted: &mut Vector2<f64>)
    -> bool;

    fn distort_with_jacobians(
        &self,
        point_undistorted: &Vector2<f64>,
        point_distorted: &mut Vector2<f64>,
        point_jacobian: &mut Matrix2<f64>,
        parameter_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> bool;

    fn distort_with_external_parameters(
        &self,
        point_undistorted: &Vector2<f64>,
        parameters: &DVector<f64>,
        point_distorted: &mut Vector2<f64>,
        point_jacobian: Option<&mut Matrix2<f64>>,
        parameter_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> bool;

    fn undistort(
        &self,
        point_distorted: &Vector2<f64>,
        point_undistorted: &mut Vector2<f64>,
    ) -> bool;

    fn undistort_with_jacobian(
        &self,
        point_distorted: &Vector2<f64>,
        point_undistorted: &mut Vector2<f64>,
        point_jacobian: &mut Matrix2<f64>,
    ) -> bool;
}

/// Unchecked inverse with the same singular-matrix arithmetic as Eigen's fixed 2×2 inverse.
pub(crate) fn inverse_2x2(matrix: &Matrix2<f64>) -> Matrix2<f64> {
    Matrix2::new(
        matrix[(1, 1)],
        -matrix[(0, 1)],
        -matrix[(1, 0)],
        matrix[(0, 0)],
    ) * (1.0 / matrix.determinant())
}
