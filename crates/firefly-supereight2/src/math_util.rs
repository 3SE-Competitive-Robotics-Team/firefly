/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Scalar, plane and rigid-motion formulas used by volumetric geometry.
use crate::image::invalid;
use firefly_error::Result;
use nalgebra::{Matrix3, Matrix4, SVector, Vector3};
pub const INVALID_NORMAL: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0);
#[must_use]
pub const fn is_power_of_two(value: i32) -> bool {
    value > 0 && (value & (value - 1)) == 0
}
#[must_use]
pub const fn log2_const(value: i32) -> i32 {
    if value < 2 {
        0
    } else {
        1 + log2_const(value / 2)
    }
}
/// # Errors
/// Returns an error when the logarithm is non-finite or the rounded exponent is outside the
/// supported shift range.
pub fn power_two_up(value: f32) -> Result<u32> {
    let power = value.log2().ceil();
    if !power.is_finite() || !(0.0..32.0).contains(&power) {
        return Err(invalid(
            "power-of-two conversion outside signed shift range",
        ));
    }
    Ok(1u32 << power as u32)
}
#[must_use]
pub fn fractional<const N: usize>(v: &SVector<f32, N>) -> SVector<f32, N> {
    v.map(|x| x - x.floor())
}
pub fn floor<const N: usize>(v: &SVector<f32, N>) -> SVector<f32, N> {
    v.map(f32::floor)
}
pub fn abs<const N: usize>(v: &SVector<f32, N>) -> SVector<f32, N> {
    v.map(f32::abs)
}
pub fn square<T: Copy + std::ops::Mul<Output = T>>(v: T) -> T {
    v * v
}
pub fn cube<T: Copy + std::ops::Mul<Output = T>>(v: T) -> T {
    v * v * v
}
pub fn in_range<T: PartialOrd>(v: &T, low: &T, high: &T) -> bool {
    v >= low && v <= high
}
#[must_use]
pub fn plane_normal(p1: &Vector3<f32>, p2: &Vector3<f32>, p3: &Vector3<f32>) -> Vector3<f32> {
    let cross = (p2 - p1).cross(&(p3 - p2));
    let norm = cross.norm();
    if norm > 0.0 { cross / norm } else { cross }
}
#[must_use]
pub fn hat(omega: &Vector3<f32>) -> Matrix3<f32> {
    Matrix3::new(
        0.0, -omega.z, omega.y, omega.z, 0.0, -omega.x, -omega.y, omega.x, 0.0,
    )
}
#[must_use]
pub fn exp_and_theta(omega: &Vector3<f32>) -> (Matrix3<f32>, f32) {
    let theta_sq = omega.dot(omega);
    let theta = theta_sq.sqrt();
    let half_theta = 0.5f32 * theta;
    let (imag, real) = if f64::from(theta) < 1e-10 {
        let theta_po4 = theta_sq * theta_sq;
        (
            0.5f32 - (1.0f32 / 48.0f32) * theta_sq + (1.0f32 / 3840.0f32) * theta_po4,
            1.0f32 - 0.5f32 * theta_sq + (1.0f32 / 384.0f32) * theta_po4,
        )
    } else {
        (half_theta.sin() / theta, half_theta.cos())
    };
    let mut q = nalgebra::Quaternion::new(real, imag * omega.x, imag * omega.y, imag * omega.z);
    let norm = q.norm();
    if norm > 0.0 {
        q /= norm;
    }
    let q = nalgebra::UnitQuaternion::new_unchecked(q);
    (q.to_rotation_matrix().into_inner(), theta)
}
#[must_use]
pub fn exp(a: &SVector<f32, 6>) -> Matrix4<f32> {
    let omega = Vector3::new(a[3], a[4], a[5]);
    let (so3, theta) = exp_and_theta(&omega);
    let omega_hat = hat(&omega);
    let omega_sq = omega_hat * omega_hat;
    let v = if f64::from(theta) < 1e-10 {
        so3
    } else {
        let theta_sq = theta * theta;
        Matrix3::identity()
            + ((1.0f32 - theta.cos()) / theta_sq) * omega_hat
            + ((theta - theta.sin()) / (theta_sq * theta)) * omega_sq
    };
    let mut se3 = Matrix4::identity();
    se3.fixed_view_mut::<3, 3>(0, 0).copy_from(&so3);
    se3.fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&(v * Vector3::new(a[0], a[1], a[2])));
    se3
}
