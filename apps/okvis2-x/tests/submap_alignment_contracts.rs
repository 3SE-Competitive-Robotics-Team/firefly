// SPDX-License-Identifier: BSD-3-Clause
//! Analytic occupancy planes isolate alignment geometry from voxel interpolation.
use firefly_okvis_x::{
    ceres::{
        pose_local_parameterization::PoseManifold,
        submap_icp_error::{OccupancySubmap, SubmapIcpError},
    },
    kinematics::transformation::Transformation,
};
use nalgebra::{SMatrix, SVector, UnitQuaternion, Vector3};
use std::sync::Arc;
#[derive(Clone)]
struct Plane {
    normal: Vector3<f32>,
    offset: f32,
    valid: bool,
}
impl OccupancySubmap for Plane {
    fn grad_field_mean_occup(&self, _: &Vector3<f32>) -> Option<Vector3<f32>> {
        self.valid.then_some(self.normal)
    }
    fn interp_field_mean_occup(&self, p: &Vector3<f32>) -> Option<f32> {
        self.valid.then(|| self.normal.dot(p) + self.offset)
    }
    fn log_odd_min(&self) -> f32 {
        -3.0
    }
    fn snapshot(&self) -> firefly_error::Result<Arc<dyn OccupancySubmap + Send + Sync>> {
        Ok(Arc::new(self.clone()))
    }
}
fn evaluate(
    map: &Plane,
    point: Vector3<f64>,
    a: &SVector<f64, 7>,
    b: &SVector<f64, 7>,
) -> (f64, SMatrix<f64, 1, 6>, SMatrix<f64, 1, 6>) {
    let factor = SubmapIcpError::new(map, point, 0.2);
    let mut r = f64::NAN;
    let mut ambient_a = SMatrix::<f64, 1, 7>::zeros();
    let mut ambient_b = ambient_a;
    let mut first = SMatrix::<f64, 1, 6>::zeros();
    let mut second = first;
    assert!(factor.evaluate_with_minimal_jacobians(
        [a, b],
        &mut r,
        Some([Some(&mut ambient_a), Some(&mut ambient_b)]),
        [Some(&mut first), Some(&mut second)]
    ));
    (r, first, second)
}
#[test]
fn plane_alignment_residual_and_two_pose_jacobians_match_geometry_and_differences() {
    let map = Plane {
        normal: Vector3::new(0.0, 0.0, 2.0),
        offset: -2.0,
        valid: true,
    };
    let point = Vector3::new(0.3, -0.2, 1.4);
    let first = Transformation::new(
        Vector3::new(0.1, 0.2, -0.1),
        UnitQuaternion::from_euler_angles(0.1, -0.2, 0.05).into_inner(),
    );
    let second = Transformation::new(
        Vector3::new(-0.2, 0.1, 0.3),
        UnitQuaternion::from_euler_angles(-0.1, 0.05, 0.2).into_inner(),
    );
    let (residual, j0, j1) = evaluate(&map, point, first.coeffs(), second.coeffs());
    let transformed = first
        .inverse()
        .compose(&second)
        .transform_homogeneous(&point.push(1.0));
    let expected = (transformed.z - 1.0) / (0.5f64.powi(2) + 0.2f64.powi(2)).sqrt();
    assert!((residual - expected).abs() < 3e-7);
    // Map queries are f32, so h=1e-3 radians/metres avoids differencing quantization noise.
    for (block, jacobian) in [j0, j1].iter().enumerate() {
        for axis in 0..6 {
            let mut step = SVector::<f64, 6>::zeros();
            step[axis] = 1e-3;
            let input = if block == 0 {
                first.coeffs()
            } else {
                second.coeffs()
            };
            let plus = PoseManifold::plus(input, &step);
            let minus = PoseManifold::plus(input, &-step);
            let cost = |changed| {
                if block == 0 {
                    evaluate(&map, point, changed, second.coeffs()).0
                } else {
                    evaluate(&map, point, first.coeffs(), changed).0
                }
            };
            assert!(
                ((cost(&plus) - cost(&minus)) / 2e-3 - jacobian[(0, axis)]).abs() < 3e-4,
                "block={block}, axis={axis}"
            );
        }
    }
}
#[test]
fn three_orthogonal_planes_recover_a_known_rigid_transform() {
    let truth = Transformation::new(
        Vector3::new(0.1, -0.08, 0.05),
        UnitQuaternion::from_euler_angles(0.03, -0.04, 0.02).into_inner(),
    );
    let anchor = Transformation::identity();
    let mut estimate = Transformation::identity();
    for _ in 0..8 {
        let mut hessian = SMatrix::<f64, 6, 6>::zeros();
        let mut gradient = SVector::<f64, 6>::zeros();
        for axis in 0..3 {
            for i in 0..9 {
                let mut target = Vector3::new(
                    f64::from(i % 3) * 0.3 - 0.3,
                    f64::from(i / 3) * 0.3 - 0.3,
                    0.2,
                );
                target[axis] = 1.0;
                let point = truth
                    .inverse()
                    .transform_homogeneous(&target.push(1.0))
                    .fixed_rows::<3>(0)
                    .into_owned();
                let map = Plane {
                    normal: Vector3::ith(axis, 1.0),
                    offset: -1.0,
                    valid: true,
                };
                let (residual, _, jacobian) =
                    evaluate(&map, point, anchor.coeffs(), estimate.coeffs());
                hessian += jacobian.transpose() * jacobian;
                gradient += jacobian.transpose() * residual;
            }
        }
        let step = hessian.cholesky().unwrap().solve(&-gradient);
        estimate.oplus(&step);
    }
    assert!((estimate.matrix() - truth.matrix()).amax() < 2e-6);
}
#[test]
fn unsupported_queries_and_residual_only_requests_preserve_documented_output_contracts() {
    let pose = Transformation::identity();
    let point = Vector3::new(0.0, 0.0, 2.0);
    let map = Plane {
        normal: Vector3::z(),
        offset: 0.0,
        valid: false,
    };
    let factor = SubmapIcpError::new(&map, point, 0.2);
    let mut residual = 99.0;
    let mut ambient = SMatrix::<f64, 1, 7>::repeat(17.0);
    let mut minimal = SMatrix::<f64, 1, 6>::repeat(19.0);
    assert!(factor.evaluate_with_minimal_jacobians(
        [pose.coeffs(), pose.coeffs()],
        &mut residual,
        Some([Some(&mut ambient), None]),
        [Some(&mut minimal), None]
    ));
    assert_eq!(residual, 0.0);
    assert_eq!(ambient, SMatrix::<f64, 1, 7>::zeros());
    assert_eq!(minimal, SMatrix::<f64, 1, 6>::repeat(19.0));
    let map = Plane { valid: true, ..map };
    let factor = SubmapIcpError::new(&map, point, 0.2);
    assert!(factor.evaluate([pose.coeffs(), pose.coeffs()], &mut residual, None));
    assert_eq!(residual, 0.0);
    let (_, j0, j1) = evaluate(
        &Plane {
            normal: Vector3::zeros(),
            offset: 1.0,
            valid: true,
        },
        point,
        pose.coeffs(),
        pose.coeffs(),
    );
    assert_eq!(j0, SMatrix::<f64, 1, 6>::zeros());
    assert_eq!(j1, SMatrix::<f64, 1, 6>::zeros());
}
