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

//! Ray triangulation corresponding to `stereo_triangulation.cpp`.

use nalgebra::{Matrix2, Vector2, Vector3, Vector4};

/// Triangulates two rays in the same coordinate frame.
///
/// `p1` and `p2` are camera centers; `e1` and `e2` must be unit bearing vectors.
/// `sigma` is the ray angular uncertainty in radians. The returned homogeneous
/// point has its fourth coordinate set to one, including for parallel rays.
/// `is_valid` reports angular consistency; `is_parallel` reports weak parallax
/// or use of the distant-point fallback. A parallel result can be valid.
pub fn triangulate_fast(
    p1: &Vector3<f64>,
    e1: &Vector3<f64>,
    p2: &Vector3<f64>,
    e2: &Vector3<f64>,
    sigma: f64,
    is_valid: &mut bool,
    is_parallel: &mut bool,
) -> Vector4<f64> {
    *is_parallel = false;
    *is_valid = true;

    let t12 = p2 - p1;
    let b = Vector2::new(t12.dot(e1), t12.dot(e2));
    let a = Matrix2::new(e1.dot(e1), -e1.dot(e2), e1.dot(e2), -e2.dot(e2));
    let determinant = a.determinant();
    let invertible = determinant.abs() > 1.0e-12;

    if !invertible {
        *is_parallel = true;
        let m = p1 + 0.5 * t12;
        let midpoint = m + 40.0 * f64::max(0.01, t12.norm()) * (e1 + e2);

        if e1.dot(&normalized(midpoint - p1)) < (2.6 * sigma).cos() {
            *is_valid = false;
        }
        if e2.dot(&normalized(midpoint - p2)) < (2.6 * sigma).cos() {
            *is_valid = false;
        }

        return Vector4::new(midpoint[0], midpoint[1], midpoint[2], 1.0);
    }

    let a_inverse =
        Matrix2::new(a[(1, 1)], -a[(0, 1)], -a[(1, 0)], a[(0, 0)]) * (1.0 / determinant);
    let lambda = a_inverse * b;

    if lambda[0] < 0.01 || lambda[1] < 0.01 {
        *is_parallel = true;
        let m = p1 + 0.5 * t12;
        let midpoint = m + 40.0 * f64::max(0.01, t12.norm()) * (e1 + e2);

        if e1.dot(&normalized(midpoint - p1)) < (2.6 * sigma).cos() {
            *is_valid = false;
        }
        if e2.dot(&normalized(midpoint - p2)) < (2.6 * sigma).cos() {
            *is_valid = false;
        }

        return Vector4::new(midpoint[0], midpoint[1], midpoint[2], 1.0);
    }

    let xm = lambda[0] * e1 + p1;
    let xn = lambda[1] * e2 + p2;
    let midpoint = (xm + xn) / 2.0;

    if e1.dot(&normalized(midpoint - p1)) < (2.6 * sigma).cos() {
        *is_valid = false;
    }
    if e2.dot(&normalized(midpoint - p2)) < (2.6 * sigma).cos() {
        *is_valid = false;
    }
    if normalized(midpoint - p2).dot(&normalized(midpoint - p1)) > (6.0 * sigma).cos() {
        *is_parallel = true;
    }

    Vector4::new(midpoint[0], midpoint[1], midpoint[2], 1.0)
}

/// Matches Eigen `normalized()`: divide only when the squared norm is positive.
fn normalized(vector: Vector3<f64>) -> Vector3<f64> {
    let squared_norm = vector.norm_squared();
    if squared_norm > 0.0 {
        vector / squared_norm.sqrt()
    } else {
        vector
    }
}
