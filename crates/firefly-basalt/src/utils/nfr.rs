// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are in ../../LICENSE.
//! `utils/nfr.h` from Basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc.
//! Nonlinear factor residuals with world-additive translation and left rotation increments.
use firefly_basalt_headers::utils::sophus_utils::{
    hat, right_jacobian_inv_se3_decoupled, se3_logd,
};
use nalgebra::{Isometry3, SMatrix, SVector, UnitQuaternion, Vector2, Vector3};

#[must_use]
#[expect(
    clippy::similar_names,
    reason = "Source uses frame-indexed transform names"
)]
pub fn rel_pose_error(
    t_i_j: &Isometry3<f64>,
    t_w_i: &Isometry3<f64>,
    t_w_j: &Isometry3<f64>,
    d_i: Option<&mut SMatrix<f64, 6, 6>>,
    d_j: Option<&mut SMatrix<f64, 6, 6>>,
) -> SVector<f64, 6> {
    let t_j_i = t_w_j.inverse() * t_w_i;
    let residual = se3_logd(&(t_i_j * t_j_i));
    if d_i.is_some() || d_j.is_some() {
        let jacobian = right_jacobian_inv_se3_decoupled(&residual);
        let rotation = t_w_i.rotation.inverse().to_rotation_matrix().into_inner();
        let mut adjoint = SMatrix::<f64, 6, 6>::zeros();
        adjoint.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
        adjoint.fixed_view_mut::<3, 3>(3, 3).copy_from(&rotation);
        if let Some(output) = d_i {
            *output = jacobian * adjoint;
        }
        if let Some(output) = d_j {
            adjoint
                .fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&(hat(&t_j_i.inverse().translation.vector) * rotation));
            *output = -jacobian * adjoint;
        }
    }
    residual
}

#[must_use]
pub fn abs_position_error(
    pose: &Isometry3<f64>,
    position: &Vector3<f64>,
    derivative: Option<&mut SMatrix<f64, 3, 6>>,
) -> Vector3<f64> {
    if let Some(output) = derivative {
        output.fill(0.);
        output.fixed_view_mut::<3, 3>(0, 0).fill_with_identity();
    }
    pose.translation.vector - position
}

#[must_use]
pub fn yaw_error(
    pose: &Isometry3<f64>,
    yaw_direction_body: &Vector3<f64>,
    derivative: Option<&mut SMatrix<f64, 1, 6>>,
) -> f64 {
    let rotated = pose.rotation * yaw_direction_body;
    if let Some(output) = derivative {
        output.fill(0.);
        output[3] = -rotated.z;
        output[5] = rotated.x;
    }
    rotated.y
}

/// Gravity points along negative world Z.
#[must_use]
pub fn roll_pitch_error(
    pose: &Isometry3<f64>,
    measured_rotation: &UnitQuaternion<f64>,
    derivative: Option<&mut SMatrix<f64, 2, 6>>,
) -> Vector2<f64> {
    let rotation = (measured_rotation * pose.rotation.inverse())
        .to_rotation_matrix()
        .into_inner();
    if let Some(output) = derivative {
        output.fill(0.);
        output[(0, 3)] = -rotation[(0, 1)];
        output[(1, 3)] = -rotation[(1, 1)];
        output[(0, 4)] = rotation[(0, 0)];
        output[(1, 4)] = rotation[(1, 0)];
    }
    -(rotation * Vector3::z()).fixed_rows::<2>(0).into_owned()
}
