// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are indexed in docs/licenses/README.md.
//! Camera pose initialization from original Basalt calibration helpers.
use crate::calibration::Calibration;
use crate::camera::generic_camera::GenericCamera;
use crate::observation::{CalibCornerData, CalibCornerMap, CalibInitPoseData, CalibInitPoseMap};
use firefly_opengv::{
    absolute_pose::central_absolute_adapter::CentralAbsoluteAdapter,
    optimization_functor::RustNumericalBackend,
    sac::ransac::Ransac,
    sac_problems::absolute_pose::absolute_pose_sac_problem::{AbsolutePoseSacProblem, Algorithm},
};
use nalgebra::{Isometry3, Rotation3, Translation3, UnitQuaternion, Vector2, Vector4};
use rayon::prelude::*;

/// Initializes camera poses from calibrated target observations.
/// # Panics
/// Requires valid camera and corner indices.
#[must_use]
pub fn init_cam_poses(
    calibration: &Calibration<f64>,
    target: &[Vector4<f64>],
    corners: &CalibCornerMap,
) -> CalibInitPoseMap {
    corners
        .par_iter()
        .map(|(&id, data)| {
            (
                id,
                compute_initial_pose(&calibration.intrinsics[id.cam_id], target, data),
            )
        })
        .collect()
}

fn compute_initial_pose(
    camera: &GenericCamera<f64>,
    target: &[Vector4<f64>],
    data: &CalibCornerData,
) -> CalibInitPoseData {
    if data.corners.len() < 8 {
        return CalibInitPoseData::default();
    }
    let Some((pose, count)) =
        estimate_transformation(camera, &data.corners, &data.corner_ids, target)
    else {
        return CalibInitPoseData::default();
    };
    let mut result = CalibInitPoseData {
        t_a_c: pose,
        ..Default::default()
    };
    if count > 8 {
        result.reprojected_corners = project_target(camera, target, &pose);
        result.num_inliers = count;
    }
    result
}

/// Projects all target points through the inverse camera-to-target pose.
#[must_use]
pub fn project_target(
    camera: &GenericCamera<f64>,
    target: &[Vector4<f64>],
    pose: &Isometry3<f64>,
) -> Vec<Vector2<f64>> {
    let transform = pose.inverse().to_homogeneous();
    target
        .iter()
        .map(|point| {
            let mut pixel = Vector2::zeros();
            camera.project(&(transform * point), &mut pixel, None);
            pixel
        })
        .collect()
}

/// Estimates camera-to-target pose with the original Kneip central absolute-pose solver.
#[must_use]
pub fn estimate_transformation(
    camera: &GenericCamera<f64>,
    corners: &[Vector2<f64>],
    ids: &[i32],
    target: &[Vector4<f64>],
) -> Option<(Isometry3<f64>, usize)> {
    let (mut bearings, mut points) = (Vec::new(), Vec::new());
    for (pixel, &id) in corners.iter().zip(ids) {
        let mut bearing = Vector4::zeros();
        if !camera.unproject(pixel, &mut bearing, None) {
            continue;
        }
        bearings.push(bearing.fixed_rows::<3>(0).normalize());
        points.push(target[id as usize].fixed_rows::<3>(0).into_owned());
    }
    let mut adapter = CentralAbsoluteAdapter::new(&bearings, &points).ok()?;
    let backend = RustNumericalBackend;
    let problem = AbsolutePoseSacProblem::new(&mut adapter, Algorithm::Kneip, &backend, true);
    let threshold = 1. - (2.0_f64.sqrt() / camera.param()[0]).atan().cos();
    let mut ransac = Ransac::new(problem, 50, threshold, 0.99);
    if !ransac.compute_model(0) {
        return None;
    }
    let model = ransac.model_coefficients?;
    let pose = Isometry3::from_parts(
        Translation3::from(model.column(3).into_owned()),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
            model.fixed_view::<3, 3>(0, 0).into_owned(),
        )),
    );
    Some((pose, ransac.inliers.len()))
}

/// Computes the original summed reprojection error used for calibration selection.
#[must_use]
pub fn reprojection_error(
    camera: &GenericCamera<f64>,
    corners: &[Vector2<f64>],
    ids: &[i32],
    target: &[Vector4<f64>],
    pose: &Isometry3<f64>,
) -> (usize, f64) {
    let transform = pose.inverse().to_homogeneous();
    let mut error = 0.;
    for (pixel, &id) in corners.iter().zip(ids) {
        let mut prediction = Vector2::zeros();
        camera.project(&(transform * target[id as usize]), &mut prediction, None);
        error += (prediction - pixel).norm();
    }
    (corners.len(), error)
}
