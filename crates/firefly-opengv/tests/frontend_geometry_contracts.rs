// SPDX-License-Identifier: BSD-3-Clause
//! Known multi-camera scenes exercise the solvers selected by the OKVIS frontend.
use firefly_opengv::{
    absolute_pose::{methods as absolute, noncentral_absolute_adapter::NoncentralAbsoluteAdapter},
    optimization_functor::RustNumericalBackend,
    relative_pose::{central_relative_adapter::CentralRelativeAdapter, methods as relative},
    sac::ransac::Ransac,
    sac_problems::{
        absolute_pose::absolute_pose_sac_problem::{
            AbsolutePoseSacProblem, Algorithm as AbsoluteAlgorithm,
        },
        relative_pose::central_relative_pose_sac_problem::{
            Algorithm as RelativeAlgorithm, CentralRelativePoseSacProblem,
        },
    },
};
use nalgebra::{Matrix3, Rotation3, Vector3};

struct RigScene {
    points: Vec<Vector3<f64>>,
    bearings: Vec<Vector3<f64>>,
    cameras: Vec<usize>,
    offsets: [Vector3<f64>; 3],
    rotations: [Matrix3<f64>; 3],
    rotation: Matrix3<f64>,
    translation: Vector3<f64>,
}
fn scene() -> RigScene {
    let rotation = Rotation3::from_euler_angles(0.12, -0.17, 0.08).into_inner();
    let translation = Vector3::new(0.3, -0.2, 0.1);
    let offsets = [
        Vector3::new(-0.2, 0.0, 0.0),
        Vector3::new(0.2, 0.0, 0.0),
        Vector3::new(0.0, 0.15, 0.05),
    ];
    let rotations = [
        Matrix3::identity(),
        Rotation3::from_euler_angles(0.0, 0.08, 0.0).into_inner(),
        Rotation3::from_euler_angles(-0.05, 0.0, 0.0).into_inner(),
    ];
    let points: Vec<_> = (0..48)
        .map(|i| {
            Vector3::new(
                f64::from(i % 6) * 0.6 - 1.4,
                f64::from(i / 6) * 0.4 - 1.2,
                4.0 + f64::from(i % 7) * 0.35,
            )
        })
        .collect();
    let cameras: Vec<_> = (0..points.len()).map(|i| i % 3).collect();
    let bearings = points
        .iter()
        .zip(&cameras)
        .map(|(p, &c)| {
            (rotations[c].transpose() * (rotation.transpose() * (p - translation) - offsets[c]))
                .normalize()
        })
        .collect();
    RigScene {
        points,
        bearings,
        cameras,
        offsets,
        rotations,
        rotation,
        translation,
    }
}
fn adapter(s: &RigScene) -> NoncentralAbsoluteAdapter<'_> {
    NoncentralAbsoluteAdapter::new(&s.bearings, &s.cameras, &s.points, &s.offsets, &s.rotations)
        .unwrap()
}
fn bearing_error(scene: &RigScene, pose: &nalgebra::SMatrix<f64, 3, 4>) -> f64 {
    let rotation = pose.fixed_view::<3, 3>(0, 0);
    let translation = pose.column(3);
    scene
        .points
        .iter()
        .enumerate()
        .map(|(i, point)| {
            let camera = scene.cameras[i];
            let bearing = (scene.rotations[camera].transpose()
                * (rotation.transpose() * (point - translation) - scene.offsets[camera]))
                .normalize();
            (bearing - scene.bearings[i]).norm_squared()
        })
        .sum()
}
#[test]
fn gp3p_candidates_reproject_the_rig_and_refine_to_the_known_pose() {
    let scene = scene();
    let backend = RustNumericalBackend;
    let all_indices: Vec<_> = (0..scene.points.len()).collect();
    for indices in [[0, 13, 32], [7, 20, 45], [6, 25, 41]] {
        let candidates = absolute::gp3p(&adapter(&scene), &indices, &backend).unwrap();
        let candidate = candidates
            .iter()
            .min_by(|a, b| bearing_error(&scene, a).total_cmp(&bearing_error(&scene, b)))
            .unwrap();
        // A minimal polynomial root is a proposal; angular error bounds its geometric quality.
        assert!(
            bearing_error(&scene, candidate) < 1e-8,
            "indices={indices:?}"
        );
        let input = adapter(&scene).with_pose(
            candidate.column(3).into_owned(),
            candidate.fixed_view::<3, 3>(0, 0).into_owned(),
        );
        let refined = absolute::optimize_nonlinear(&input, &all_indices, &backend).unwrap();
        assert!(bearing_error(&scene, &refined) < 1e-9);
        assert!((refined.fixed_view::<3, 3>(0, 0) - scene.rotation).norm() < 1e-4);
        assert!((refined.column(3) - scene.translation).norm() < 1e-4);
    }
}
#[test]
fn gp3p_consensus_recovers_rig_pose_despite_planted_outliers() {
    let mut s = scene();
    for i in (0..48).step_by(6) {
        s.bearings[i] = Vector3::new(-0.4, 0.5, 1.0).normalize();
    }
    let mut input = adapter(&s);
    let backend = RustNumericalBackend;
    let problem = AbsolutePoseSacProblem::new(&mut input, AbsoluteAlgorithm::Gp3p, &backend, false);
    let mut consensus = Ransac::new(problem, 100, 1e-8, 0.999);
    assert!(consensus.compute_model(0));
    assert_eq!(
        consensus.inliers,
        (0..48).filter(|i| i % 6 != 0).collect::<Vec<_>>()
    );
    let pose = consensus.model_coefficients.unwrap();
    assert!((pose.fixed_view::<3, 3>(0, 0) - s.rotation).norm() < 1e-6);
    assert!((pose.column(3) - s.translation).norm() < 1e-6);
}
#[test]
fn stewenius_consensus_recovers_rotation_and_translation_direction() {
    let s = scene();
    let first: Vec<_> = s.points.iter().map(Vector3::normalize).collect();
    let mut second: Vec<_> = s
        .points
        .iter()
        .map(|p| (s.rotation.transpose() * (p - s.translation)).normalize())
        .collect();
    for i in (0..48).step_by(6) {
        second[i] = Vector3::new(-0.4, 0.5, 1.0).normalize();
    }
    let mut input = CentralRelativeAdapter::new(&first, &second).unwrap();
    let backend = RustNumericalBackend;
    let problem = CentralRelativePoseSacProblem::new(
        &mut input,
        RelativeAlgorithm::Stewenius,
        &backend,
        false,
    );
    let mut consensus = Ransac::new(problem, 150, 1e-8, 0.999);
    assert!(consensus.compute_model(0));
    assert_eq!(
        consensus.inliers,
        (0..48).filter(|i| i % 6 != 0).collect::<Vec<_>>()
    );
    let pose = consensus.model_coefficients.unwrap();
    assert!((pose.fixed_view::<3, 3>(0, 0) - s.rotation).norm() < 1e-6);
    assert!((pose.column(3).normalize() - s.translation.normalize()).norm() < 1e-6);
}
#[test]
fn nonlinear_absolute_and_relative_refinement_reduce_independent_bearing_error() {
    let s = scene();
    let backend = RustNumericalBackend;
    let indices: Vec<_> = (0..48).collect();
    let perturbed = Rotation3::from_euler_angles(0.015, -0.01, 0.02).into_inner() * s.rotation;
    let input = adapter(&s).with_pose(s.translation + Vector3::new(0.04, -0.03, 0.02), perturbed);
    let pose = absolute::optimize_nonlinear(&input, &indices, &backend).unwrap();
    let rotation = pose.fixed_view::<3, 3>(0, 0).into_owned();
    let translation = pose.column(3).into_owned();
    let error = bearing_error(&s, &pose);
    assert!(error < 1e-9, "bearing error={error}");
    assert!((rotation - s.rotation).norm() < 1e-4 && (translation - s.translation).norm() < 1e-4);
    let first: Vec<_> = s.points.iter().map(Vector3::normalize).collect();
    let second: Vec<_> = s
        .points
        .iter()
        .map(|p| (s.rotation.transpose() * (p - s.translation)).normalize())
        .collect();
    let mut input = CentralRelativeAdapter::new(&first, &second)
        .unwrap()
        .with_pose(s.translation.normalize(), perturbed);
    let pose = relative::optimize_nonlinear(&mut input, &indices, &backend).unwrap();
    assert!((pose.fixed_view::<3, 3>(0, 0) - s.rotation).norm() < 1e-4);
    assert!((pose.column(3).normalize() - s.translation.normalize()).norm() < 1e-4);
}
