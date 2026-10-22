// SPDX-License-Identifier: BSD-3-Clause

//! Known 3-D scenes, geometric transformations and actual consensus with deterministic sampling.
use firefly_opengv::{
    absolute_pose::{central_absolute_adapter::CentralAbsoluteAdapter, methods},
    optimization_functor::RustNumericalBackend,
    relative_pose::central_relative_adapter::CentralRelativeAdapter,
    sac::ransac::Ransac,
    sac_problems::absolute_pose::absolute_pose_sac_problem::{AbsolutePoseSacProblem, Algorithm},
    triangulation::methods::{triangulate, triangulate2},
};
use nalgebra::{Matrix3, Rotation3, Vector3};
use proptest::prelude::*;

struct StereoRays<'a, 'b>(&'a CentralRelativeAdapter<'b>);
impl firefly_opengv::triangulation::methods::RelativeAdapterBase for StereoRays<'_, '_> {
    fn t12(&self) -> Vector3<f64> {
        self.0.t12
    }
    fn r12(&self) -> Matrix3<f64> {
        self.0.r12
    }
    fn bearing_vector1(&self, index: usize) -> Vector3<f64> {
        self.0.bearing_vectors1[index]
    }
    fn bearing_vector2(&self, index: usize) -> Vector3<f64> {
        self.0.bearing_vectors2[index]
    }
}

struct Scene {
    points: Vec<Vector3<f64>>,
    rotation: Matrix3<f64>,
    translation: Vector3<f64>,
    bearings: Vec<Vector3<f64>>,
}
fn scene() -> Scene {
    let rotation = Rotation3::from_euler_angles(0.12, -0.17, 0.08).into_inner();
    let translation = Vector3::new(0.3, -0.2, 0.1);
    let points: Vec<_> = (0..40)
        .map(|i| {
            Vector3::new(
                f64::from(i % 5) * 0.7 - 1.3,
                f64::from(i / 5) * 0.4 - 1.4,
                4.0 + f64::from(i % 7) * 0.3,
            )
        })
        .collect();
    let bearings = points
        .iter()
        .map(|p| (rotation.transpose() * (p - translation)).normalize())
        .collect();
    Scene {
        points,
        rotation,
        translation,
        bearings,
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_failure_persistence(
        proptest::test_runner::FileFailurePersistence::WithSource("proptest-regressions")
    ))]
    #[test]
    fn stereo_recovers_scene_and_is_covariant_to_metric_scale(
        x in -1.0f64..1.0, y in -0.8f64..0.8, z in 2.0f64..12.0,
        baseline in 0.1f64..1.0, scale in 0.2f64..5.0, angle in -0.3f64..0.3
    ) {
        let point = Vector3::new(x,y,z);
        let rotation = Rotation3::from_euler_angles(angle,angle*0.7,-angle*0.3).into_inner();
        let translation = Vector3::new(baseline,baseline*0.2,-baseline*0.1);
        let first = [point.normalize()];
        let second = [(rotation.transpose()*(point-translation)).normalize()];
        let adapter = CentralRelativeAdapter::new(&first,&second).unwrap().with_pose(translation,rotation);
        let scaled = CentralRelativeAdapter::new(&first,&second).unwrap().with_pose(translation*scale,rotation);
        for (actual,scaled_actual) in [(triangulate(&StereoRays(&adapter),0),triangulate(&StereoRays(&scaled),0)),(triangulate2(&StereoRays(&adapter),0),triangulate2(&StereoRays(&scaled),0))] {
            // Conditioning is controlled by z <= 12 m, baseline >= 0.1 m.
            prop_assert!((actual-point).norm() < 1e-8, "actual={actual:?}, point={point:?}");
            prop_assert!((scaled_actual-point*scale).norm() < 5e-8);
            prop_assert!(actual.z>0.0);
        }
    }
}

#[test]
fn epnp_recovers_known_pose_and_all_bearing_reprojections() {
    let Scene {
        points,
        rotation,
        translation,
        bearings,
    } = scene();
    let adapter = CentralAbsoluteAdapter::new(&bearings, &points).unwrap();
    let indices: Vec<_> = (0..points.len()).collect();
    for indices in [indices.clone(), indices.into_iter().rev().collect()] {
        let estimate = methods::epnp(&adapter, &indices).unwrap();
        let r = estimate.fixed_view::<3, 3>(0, 0).into_owned();
        let t = estimate.column(3).into_owned();
        assert!((r - rotation).norm() < 1e-8);
        assert!((t - translation).norm() < 1e-8);
        assert!((r.transpose() * r - Matrix3::identity()).norm() < 1e-12);
        assert!((r.determinant() - 1.0).abs() < 1e-12);
        for (point, bearing) in points.iter().zip(&bearings) {
            assert!(((r.transpose() * (point - t)).normalize() - bearing).norm() < 1e-9);
        }
    }
}

#[test]
fn ransac_rejects_planted_outliers_and_returns_correct_metric_pose() {
    let Scene {
        points,
        rotation,
        translation,
        mut bearings,
    } = scene();
    for i in (0..points.len()).step_by(5) {
        bearings[i] = Vector3::new(-0.4, 0.5, 1.0).normalize();
    }
    let mut adapter = CentralAbsoluteAdapter::new(&bearings, &points).unwrap();
    let backend = RustNumericalBackend;
    let problem = AbsolutePoseSacProblem::new(&mut adapter, Algorithm::Epnp, &backend, false);
    let mut ransac = Ransac::new(problem, 100, 1e-8, 0.999);
    assert!(ransac.compute_model(0));
    let expected: Vec<_> = (0..points.len()).filter(|i| i % 5 != 0).collect();
    assert_eq!(ransac.inliers, expected);
    let model = ransac.model_coefficients.as_ref().unwrap();
    assert!((model.fixed_view::<3, 3>(0, 0) - rotation).norm() < 1e-7);
    assert!((model.column(3) - translation).norm() < 1e-7);
    assert!(ransac.iterations <= 101);
}

#[test]
fn incompatible_storage_and_invalid_solver_indices_fail_without_panicking() {
    assert!(CentralAbsoluteAdapter::new(&[Vector3::z()], &[]).is_err());
    assert!(CentralRelativeAdapter::new(&[Vector3::z()], &[]).is_err());
    let Scene {
        points, bearings, ..
    } = scene();
    let adapter = CentralAbsoluteAdapter::new(&bearings, &points).unwrap();
    assert!(methods::epnp(&adapter, &[0, 1, 2, 3, 4]).is_none());
    assert!(methods::epnp(&adapter, &[0, 1, 2, 3, 4, points.len()]).is_none());
    assert!(methods::p3p_kneip(&adapter, &[0, 1]).is_none());
    let mut adapter = CentralAbsoluteAdapter::new(&[], &[]).unwrap();
    let backend = RustNumericalBackend;
    let problem = AbsolutePoseSacProblem::new(&mut adapter, Algorithm::Epnp, &backend, false);
    let mut ransac = Ransac::new(problem, 10, 1e-8, 0.99);
    assert!(!ransac.compute_model(0));
    assert_eq!(ransac.inliers.len(), 0);
}

#[test]
fn parallel_rays_expose_undefined_depth_instead_of_a_fabricated_finite_point() {
    let first = [Vector3::z()];
    let second = first;
    let adapter = CentralRelativeAdapter::new(&first, &second)
        .unwrap()
        .with_pose(Vector3::x(), Matrix3::identity());
    let result = triangulate2(&StereoRays(&adapter), 0);
    assert!(result.iter().any(|v| !v.is_finite()));
}
