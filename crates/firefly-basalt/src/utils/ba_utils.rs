// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of original Basalt `0f3b2b52` `utils/ba_utils.h`.
use crate::utils::sophus_utils::hat;
use crate::utils::sophus_utils::so3_inverse;
use crate::utils::sophus_utils::so3_matrix;
use crate::utils::sophus_utils::so3_product;
use crate::vi_estimator::landmark_database::Keypoint;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::camera::stereographic_param::StereographicParam;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{Isometry3, Matrix4, SMatrix, Translation3, Vector2, Vector4};

fn inverse<S: BasaltScalar>(pose: &Isometry3<S>) -> Isometry3<S> {
    let rotation = so3_inverse(&pose.rotation);
    Isometry3::from_parts(
        Translation3::from(rotation * (-pose.translation.vector)),
        rotation,
    )
}
fn product<S: BasaltScalar>(left: &Isometry3<S>, right: &Isometry3<S>) -> Isometry3<S> {
    Isometry3::from_parts(
        Translation3::from(left.translation.vector + left.rotation * right.translation.vector),
        so3_product(&left.rotation, &right.rotation),
    )
}
fn adjoint<S: BasaltScalar>(pose: &Isometry3<S>) -> SMatrix<S, 6, 6> {
    let rotation = so3_matrix(&pose.rotation);
    let mut result = SMatrix::<S, 6, 6>::zeros();
    result.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
    result.fixed_view_mut::<3, 3>(3, 3).copy_from(&rotation);
    result
        .fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&(hat(&pose.translation.vector) * rotation));
    result
}

/// Host-to-target camera transform and derivatives of its left increment.
#[must_use]
pub fn compute_rel_pose<S: BasaltScalar>(
    world_host: &Isometry3<S>,
    host_camera: &Isometry3<S>,
    world_target: &Isometry3<S>,
    target_camera: &Isometry3<S>,
    host_jacobian: Option<&mut SMatrix<S, 6, 6>>,
    target_jacobian: Option<&mut SMatrix<S, 6, 6>>,
) -> Isometry3<S> {
    let camera_inverse = inverse(target_camera);
    let target_inverse = so3_inverse(&world_target.rotation);
    let relative = Isometry3::from_parts(
        Translation3::from(
            target_inverse * (world_host.translation.vector - world_target.translation.vector),
        ),
        so3_product(&target_inverse, &world_host.rotation),
    );
    let intermediate = product(&camera_inverse, &relative);
    let result = product(&intermediate, host_camera);
    for (output, world, transform, negative) in [
        (host_jacobian, world_host, &intermediate, false),
        (target_jacobian, world_target, &camera_inverse, true),
    ] {
        if let Some(output) = output {
            let rotation = so3_matrix(&so3_inverse(&world.rotation));
            let mut repeated = SMatrix::<S, 6, 6>::zeros();
            repeated.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
            repeated.fixed_view_mut::<3, 3>(3, 3).copy_from(&rotation);
            *output = if negative {
                -adjoint(transform) * repeated
            } else {
                adjoint(transform) * repeated
            };
        }
    }
    result
}

/// Reprojection residual and analytic derivatives in inverse-distance coordinates.
/// Invalid projections return before altering derivative outputs. `projection[3]`
/// is intentionally untouched, matching the original diagnostic output.
#[expect(
    clippy::too_many_arguments,
    reason = "Preserves the upstream optional Jacobian interface"
)]
pub fn linearize_point<S: BasaltScalar>(
    observation: &Vector2<S>,
    point: &Keypoint<S>,
    transform: &Matrix4<S>,
    camera: &GenericCamera<S>,
    residual: &mut Vector2<S>,
    pose_jacobian: Option<&mut SMatrix<S, 2, 6>>,
    point_jacobian: Option<&mut SMatrix<S, 2, 3>>,
    projection: Option<&mut Vector4<S>>,
) -> bool {
    let mut unprojection_jacobian = SMatrix::<S, 4, 2>::zeros();
    let mut host =
        StereographicParam::unproject(&point.direction, Some(&mut unprojection_jacobian));
    host[3] = point.inv_dist;
    let target = transform * host;
    let mut camera_jacobian = SMatrix::<S, 2, 4>::zeros();
    let valid = camera.project(&target, residual, Some(&mut camera_jacobian));
    if !valid || !residual.iter().all(nalgebra::ComplexField::is_finite) {
        return false;
    }
    if let Some(projection) = projection {
        projection.fixed_rows_mut::<2>(0).copy_from(residual);
        projection[2] = target[3] / target.fixed_rows::<3>(0).norm();
    }
    *residual -= observation;
    if let Some(output) = pose_jacobian {
        let mut derivative = SMatrix::<S, 4, 6>::zeros();
        derivative
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(SMatrix::<S, 3, 3>::identity() * point.inv_dist));
        derivative
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&-hat(&target.fixed_rows::<3>(0).into_owned()));
        *output = camera_jacobian * derivative;
    }
    if let Some(output) = point_jacobian {
        let mut derivative = SMatrix::<S, 4, 3>::zeros();
        derivative
            .fixed_view_mut::<3, 2>(0, 0)
            .copy_from(&(transform.fixed_view::<3, 4>(0, 0) * unprojection_jacobian));
        derivative.set_column(2, &transform.column(3));
        *output = camera_jacobian * derivative;
    }
    true
}
