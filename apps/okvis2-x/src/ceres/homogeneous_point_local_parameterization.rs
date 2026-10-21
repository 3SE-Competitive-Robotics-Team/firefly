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

//! Homogeneous-point updates corresponding to `HomogeneousPointLocalParameterization.cpp`.

use nalgebra::{SMatrix, Vector3, Vector4};

/// Euclidean updates of `[x, y, z, c]` with a fixed homogeneous scale `c`.
#[derive(Clone, Copy, Debug, Default)]
pub struct HomogeneousPointManifold;

impl HomogeneousPointManifold {
    #[must_use]
    pub const fn ambient_size(&self) -> usize {
        4
    }

    #[must_use]
    pub const fn tangent_size(&self) -> usize {
        3
    }

    /// Adds a three-dimensional increment without changing the fourth coordinate.
    #[must_use]
    pub fn plus(x: &Vector4<f64>, delta: &Vector3<f64>) -> Vector4<f64> {
        x + Vector4::new(delta[0], delta[1], delta[2], 0.0)
    }

    /// Jacobian of `plus` with respect to the tangent increment at zero.
    #[must_use]
    pub fn plus_jacobian(_x: &Vector4<f64>) -> SMatrix<f64, 4, 3> {
        let mut jacobian = SMatrix::<f64, 4, 3>::zeros();
        jacobian
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&SMatrix::<f64, 3, 3>::identity());
        jacobian
    }

    /// Subtracts the first three coordinates; `y` and `x` must have equal scales.
    #[must_use]
    pub fn minus(y: &Vector4<f64>, x: &Vector4<f64>) -> Vector3<f64> {
        (y - x).fixed_rows::<3>(0).into_owned()
    }

    /// Jacobian of `minus(y, x)` with respect to `y` at `y = x`.
    #[must_use]
    pub fn minus_jacobian(_x: &Vector4<f64>) -> SMatrix<f64, 3, 4> {
        let mut jacobian = SMatrix::<f64, 3, 4>::zeros();
        jacobian
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&SMatrix::<f64, 3, 3>::identity());
        jacobian
    }
}
