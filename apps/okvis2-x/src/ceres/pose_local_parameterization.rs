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

//! Pose updates corresponding to `PoseLocalParameterization.cpp`.
//!
//! Ambient coordinates are `[tx, ty, tz, qx, qy, qz, qw]`; tangent coordinates
//! are translation followed by a left rotation increment in radians.

use nalgebra::{Quaternion, SMatrix, SVector, Vector4};

use crate::kinematics::{
    operators::oplus,
    transformation::{delta_q, normalized_quaternion, quaternion_inverse},
};

/// Six-degree-of-freedom pose manifold.
#[derive(Clone, Copy, Debug, Default)]
pub struct PoseManifold;

impl PoseManifold {
    #[must_use]
    pub const fn ambient_size(&self) -> usize {
        7
    }
    #[must_use]
    pub const fn tangent_size(&self) -> usize {
        6
    }

    #[must_use]
    pub fn plus(x: &SVector<f64, 7>, delta: &SVector<f64, 6>) -> SVector<f64, 7> {
        let rotation_delta = delta.fixed_rows::<3>(3).into_owned();
        let q = normalized_quaternion(Quaternion::new(x[6], x[3], x[4], x[5]));
        let q = normalized_quaternion(delta_q(&rotation_delta) * q);
        let mut result = *x;
        result
            .fixed_rows_mut::<3>(0)
            .copy_from(&(x.fixed_rows::<3>(0) + delta.fixed_rows::<3>(0)));
        result.fixed_rows_mut::<4>(3).copy_from(&q.coords);
        result
    }

    #[must_use]
    pub fn plus_jacobian(x: &SVector<f64, 7>) -> SMatrix<f64, 7, 6> {
        let mut jacobian = SMatrix::<f64, 7, 6>::zeros();
        jacobian.fixed_view_mut::<3, 3>(0, 0).fill_with_identity();
        let mut s = SMatrix::<f64, 4, 3>::zeros();
        s[(0, 0)] = 0.5;
        s[(1, 1)] = 0.5;
        s[(2, 2)] = 0.5;
        let q = Quaternion::new(x[6], x[3], x[4], x[5]);
        jacobian
            .fixed_view_mut::<4, 3>(3, 3)
            .copy_from(&(oplus(&q) * s));
        jacobian
    }

    #[must_use]
    pub fn minus(y: &SVector<f64, 7>, x: &SVector<f64, 7>) -> SVector<f64, 6> {
        let mut result = SVector::<f64, 6>::zeros();
        result
            .fixed_rows_mut::<3>(0)
            .copy_from(&(y.fixed_rows::<3>(0) - x.fixed_rows::<3>(0)));
        let q_y = Quaternion::new(y[6], y[3], y[4], y[5]);
        let q_x = Quaternion::new(x[6], x[3], x[4], x[5]);
        let delta_q = q_y * quaternion_inverse(&q_x);
        result
            .fixed_rows_mut::<3>(3)
            .copy_from(&(2.0 * delta_q.coords.fixed_rows::<3>(0)));
        result
    }

    /// Uses the conjugate without renormalizing, as in the C++ Jacobian.
    #[must_use]
    pub fn minus_jacobian(x: &SVector<f64, 7>) -> SMatrix<f64, 6, 7> {
        let q_inv = Quaternion::new(x[6], -x[3], -x[4], -x[5]);
        let mut jacobian = SMatrix::<f64, 6, 7>::zeros();
        jacobian.fixed_view_mut::<3, 3>(0, 0).fill_with_identity();
        let mut j_q_pinv = SMatrix::<f64, 3, 4>::zeros();
        j_q_pinv
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(nalgebra::Matrix3::identity() * 2.0));
        jacobian
            .fixed_view_mut::<3, 4>(3, 3)
            .copy_from(&(j_q_pinv * oplus(&q_inv)));
        jacobian
    }

    /// Compares the analytic Jacobian with central differences at step `1e-9`.
    pub fn verify_jacobian_num_diff(
        &self,
        x: &SVector<f64, 7>,
        jacobian: &mut SMatrix<f64, 7, 6>,
        jacobian_num_diff: &mut SMatrix<f64, 7, 6>,
    ) -> bool {
        *jacobian = Self::plus_jacobian(x);
        let dx = 1e-9;
        for i in 0..6 {
            let mut delta = SVector::<f64, 6>::zeros();
            delta[i] = dx;
            let xp = Self::plus(x, &delta);
            delta[i] = -dx;
            let xm = Self::plus(x, &delta);
            jacobian_num_diff.set_column(i, &((xp - xm) / (2.0 * dx)));
        }
        let difference = *jacobian - *jacobian_num_diff;
        let is_approx = difference.norm_squared()
            <= 1e-12
                * jacobian
                    .norm_squared()
                    .min(jacobian_num_diff.norm_squared());
        if !is_approx {
            log::error!("Manifold Jacobian check failed\n{jacobian}\nvs\n{jacobian_num_diff}");
        }
        is_approx
    }
}

/// Translation and yaw pose manifold; the ambient representation remains seven-dimensional.
#[derive(Clone, Copy, Debug, Default)]
pub struct PoseManifold4d;

impl PoseManifold4d {
    #[must_use]
    pub const fn ambient_size(&self) -> usize {
        7
    }
    #[must_use]
    pub const fn tangent_size(&self) -> usize {
        4
    }

    #[must_use]
    pub fn plus(x: &SVector<f64, 7>, delta: &Vector4<f64>) -> SVector<f64, 7> {
        let mut full_delta = SVector::<f64, 6>::zeros();
        full_delta
            .fixed_rows_mut::<3>(0)
            .copy_from(&delta.fixed_rows::<3>(0));
        full_delta[5] = delta[3];
        let result = PoseManifold::plus(x, &full_delta);
        let q = Quaternion::new(result[6], result[3], result[4], result[5]);
        debug_assert!(q.norm() - 1.0 < 1e-15);
        result
    }

    #[must_use]
    pub fn plus_jacobian(x: &SVector<f64, 7>) -> SMatrix<f64, 7, 4> {
        let q = normalized_quaternion(Quaternion::new(x[6], x[3], x[4], x[5]));
        let mut normalized_x = *x;
        normalized_x.fixed_rows_mut::<4>(3).copy_from(&q.coords);
        let full = PoseManifold::plus_jacobian(&normalized_x);
        let mut jacobian = SMatrix::<f64, 7, 4>::zeros();
        jacobian
            .fixed_columns_mut::<3>(0)
            .copy_from(&full.fixed_columns::<3>(0));
        jacobian.set_column(3, &full.column(5));
        jacobian
    }

    #[must_use]
    pub fn minus(y: &SVector<f64, 7>, x: &SVector<f64, 7>) -> Vector4<f64> {
        let full = PoseManifold::minus(y, x);
        Vector4::new(full[0], full[1], full[2], full[5])
    }

    #[must_use]
    pub fn minus_jacobian(x: &SVector<f64, 7>) -> SMatrix<f64, 4, 7> {
        let full = PoseManifold::minus_jacobian(x);
        let mut jacobian = SMatrix::<f64, 4, 7>::zeros();
        jacobian
            .fixed_rows_mut::<3>(0)
            .copy_from(&full.fixed_rows::<3>(0));
        jacobian.row_mut(3).copy_from(&full.row(5));
        jacobian
    }
}
