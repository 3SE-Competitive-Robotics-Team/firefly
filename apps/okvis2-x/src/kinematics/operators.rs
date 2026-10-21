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

//! Matrix operators corresponding to `operators.hpp`.

use nalgebra::{Matrix3, Matrix4, Quaternion, RealField, Vector3};

/// Returns the cross-product matrix of `[x, y, z]`.
pub fn cross_mx_xyz<T: RealField + Copy>(x: T, y: T, z: T) -> Matrix3<T> {
    Matrix3::new(T::zero(), -z, y, z, T::zero(), -x, -y, x, T::zero())
}

/// Returns `C` such that `C * u = v.cross(u)`.
pub fn cross_mx<T: RealField + Copy>(v: &Vector3<T>) -> Matrix3<T> {
    cross_mx_xyz(v[0], v[1], v[2])
}

/// Left quaternion multiplication matrix for coefficient order `[x, y, z, w]`.
#[must_use]
pub fn plus(quaternion: &Quaternion<f64>) -> Matrix4<f64> {
    let q = &quaternion.coords;
    Matrix4::new(
        q[3], -q[2], q[1], q[0], q[2], q[3], -q[0], q[1], -q[1], q[0], q[3], q[2], -q[0], -q[1],
        -q[2], q[3],
    )
}

/// Right quaternion multiplication matrix for coefficient order `[x, y, z, w]`.
#[must_use]
pub fn oplus(quaternion: &Quaternion<f64>) -> Matrix4<f64> {
    let q = &quaternion.coords;
    Matrix4::new(
        q[3], q[2], -q[1], q[0], -q[2], q[3], q[0], q[1], q[1], -q[0], q[3], q[2], -q[0], -q[1],
        -q[2], q[3],
    )
}
