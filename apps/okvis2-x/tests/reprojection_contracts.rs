// SPDX-License-Identifier: BSD-3-Clause

//! Reprojection contracts across world/body/camera frames, with independent analytic observations.
use firefly_okvis_x::{
    cameras::{no_distortion::NoDistortion, pinhole_camera::PinholeCamera},
    ceres::{
        pose_local_parameterization::PoseManifold,
        reprojection_error::{
            MinimalReprojectionJacobians, ReprojectionError, ReprojectionJacobians,
        },
    },
    kinematics::transformation::Transformation,
};
use nalgebra::{Matrix2, Rotation3, SMatrix, SVector, UnitQuaternion, Vector2, Vector3, Vector4};
use std::sync::Arc;

type Factor = ReprojectionError<PinholeCamera<NoDistortion>>;
struct Scene {
    factor: Factor,
    pose: SVector<f64, 7>,
    point: Vector4<f64>,
    extrinsics: SVector<f64, 7>,
}
fn scene() -> Scene {
    let body_rotation = Rotation3::from_euler_angles(0.2, -0.3, 0.4);
    let camera_rotation = Rotation3::from_euler_angles(-0.1, 0.25, -0.15);
    let body_position = Vector3::new(0.5, -0.2, 0.3);
    let camera_offset = Vector3::new(0.1, 0.05, -0.04);
    let camera_point = Vector3::new(0.2, -0.3, 4.0);
    // Construct a visible world point with library rotations, independent of the app's transforms.
    let point = body_rotation * (camera_rotation * camera_point + camera_offset) + body_position;
    let camera = Arc::new(PinholeCamera::new(
        (640, 480),
        (320.0, 300.0, 319.5, 239.5),
        NoDistortion,
        0,
    ));
    let pixel = Vector2::new(320.0 * 0.2 / 4.0 + 319.5, 300.0 * -0.3 / 4.0 + 239.5);
    let factor = Factor::new(
        camera,
        0,
        pixel + Vector2::new(2.0, -3.0),
        Matrix2::new(4.0, 0.0, 0.0, 9.0),
    );
    Scene {
        factor,
        pose: *Transformation::new(
            body_position,
            UnitQuaternion::from_rotation_matrix(&body_rotation).into_inner(),
        )
        .coeffs(),
        extrinsics: *Transformation::new(
            camera_offset,
            UnitQuaternion::from_rotation_matrix(&camera_rotation).into_inner(),
        )
        .coeffs(),
        point: point.push(1.0),
    }
}
fn residual(
    s: &Scene,
    pose: &SVector<f64, 7>,
    point: &Vector4<f64>,
    extrinsics: &SVector<f64, 7>,
) -> Vector2<f64> {
    let mut value = Vector2::repeat(f64::NAN);
    assert!(
        s.factor
            .evaluate((pose, point, extrinsics), &mut value, None)
    );
    value
}

#[test]
fn residual_has_analytic_units_and_pose_point_extrinsic_jacobians_match_differences() {
    let s = scene();
    let mut value = Vector2::zeros();
    let mut ambient_pose = SMatrix::<f64, 2, 7>::zeros();
    let mut ambient_point = SMatrix::<f64, 2, 4>::zeros();
    let mut ambient_extrinsics = SMatrix::<f64, 2, 7>::zeros();
    let mut pose_jacobian = SMatrix::<f64, 2, 6>::zeros();
    let mut point_jacobian = SMatrix::<f64, 2, 3>::zeros();
    let mut extrinsic_jacobian = SMatrix::<f64, 2, 6>::zeros();
    assert!(s.factor.evaluate_with_minimal_jacobians(
        (&s.pose, &s.point, &s.extrinsics),
        &mut value,
        Some(ReprojectionJacobians {
            pose: Some(&mut ambient_pose),
            point: Some(&mut ambient_point),
            extrinsics: Some(&mut ambient_extrinsics)
        }),
        Some(MinimalReprojectionJacobians {
            pose: Some(&mut pose_jacobian),
            point: Some(&mut point_jacobian),
            extrinsics: Some(&mut extrinsic_jacobian)
        }),
    ));
    assert!((value - Vector2::new(4.0, -9.0)).norm() < 1e-11);
    assert!((value - residual(&s, &s.pose, &s.point, &s.extrinsics)).norm() < 1e-12);
    // The manifold's own differential is independently tested in numerical_contracts.
    for axis in 0..6 {
        let mut step = SVector::<f64, 6>::zeros();
        step[axis] = 1e-6;
        let pose_difference = (residual(
            &s,
            &PoseManifold::plus(&s.pose, &step),
            &s.point,
            &s.extrinsics,
        ) - residual(
            &s,
            &PoseManifold::plus(&s.pose, &-step),
            &s.point,
            &s.extrinsics,
        )) / 2e-6;
        let extrinsic_difference = (residual(
            &s,
            &s.pose,
            &s.point,
            &PoseManifold::plus(&s.extrinsics, &step),
        ) - residual(
            &s,
            &s.pose,
            &s.point,
            &PoseManifold::plus(&s.extrinsics, &-step),
        )) / 2e-6;
        assert!(
            (pose_difference - pose_jacobian.column(axis)).amax() < 3e-7,
            "pose axis={axis}"
        );
        assert!(
            (extrinsic_difference - extrinsic_jacobian.column(axis)).amax() < 3e-7,
            "extrinsic axis={axis}"
        );
    }
    for axis in 0..3 {
        let mut plus = s.point;
        let mut minus = s.point;
        plus[axis] += 1e-6;
        minus[axis] -= 1e-6;
        let numeric = (residual(&s, &s.pose, &plus, &s.extrinsics)
            - residual(&s, &s.pose, &minus, &s.extrinsics))
            / 2e-6;
        assert!(
            (numeric - point_jacobian.column(axis)).amax() < 3e-7,
            "point axis={axis}"
        );
    }
}

#[test]
fn changing_world_frame_or_homogeneous_scale_preserves_observation() {
    let s = scene();
    let original = residual(&s, &s.pose, &s.point, &s.extrinsics);
    let mut pose = Transformation::identity();
    assert!(pose.set_coeffs(&s.pose));
    let change = Transformation::new(
        Vector3::new(10.0, -5.0, 2.0),
        UnitQuaternion::from_euler_angles(0.3, -0.4, 0.5).into_inner(),
    );
    let changed_pose = change.compose(&pose);
    let changed_point = change.transform_homogeneous(&s.point);
    assert!(
        (residual(&s, changed_pose.coeffs(), &changed_point, &s.extrinsics) - original).norm()
            < 1e-11
    );
    for scale in [0.2, 3.0, -0.2, -3.0] {
        assert!(
            (residual(&s, &s.pose, &(s.point * scale), &s.extrinsics) - original).norm() < 1e-11
        );
    }
}

#[test]
fn points_at_infinity_are_invariant_to_camera_and_body_translation() {
    let s = scene();
    let direction = Vector4::new(0.2, -0.1, 1.0, 0.0);
    let original = residual(&s, &s.pose, &direction, &s.extrinsics);
    let mut body = s.pose;
    body.fixed_rows_mut::<3>(0).fill(1e6);
    let mut camera = s.extrinsics;
    camera.fixed_rows_mut::<3>(0).fill(-1e6);
    assert_eq!(residual(&s, &body, &direction, &camera), original);
    assert!(original.iter().all(|v| v.is_finite()));
}
