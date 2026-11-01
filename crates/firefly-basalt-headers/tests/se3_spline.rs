use firefly_basalt_headers::{
    calibration::calib_bias::{CalibAccelBias, CalibGyroBias},
    spline::{
        ceres_spline_helper::{CeresSplineHelper, SplineLieGroup},
        se3_spline::{AccelPosSo3Jacobian, PosePosSo3Jacobian, Se3Spline},
        so3_spline::Jacobian as So3Jacobian,
        spline_common::Jacobian as PositionJacobian,
    },
    utils::sophus_utils::so3_exp,
};
use nalgebra::{Isometry3, SMatrix, SVector, Translation3, UnitQuaternion, Vector3};
use serde_json::Value;
fn check<const R: usize, const C: usize>(actual: &SMatrix<f64, R, C>, expected: &Value) {
    for row in 0..R {
        for column in 0..C {
            let a = actual[(row, column)];
            let b = expected[row * C + column].as_f64().unwrap();
            assert!(
                (a - b).abs() <= 1e-10 * (1. + b.abs()),
                "[{row},{column}] {a} != {b}"
            );
        }
    }
}
fn run<const N: usize>(key: &str) {
    let fixture: Value = serde_json::from_str(include_str!("reference/se3_spline.json")).unwrap();
    let mut spline = Se3Spline::<f64, N>::new(20_000_000, 1_000_000_000);
    for i in 0..N + 3 {
        spline.knots_push_back(&Isometry3::from_parts(
            Translation3::from(Vector3::new(
                0.125 * i as f64,
                0.0625 * (i * i) as f64,
                -0.03125 * i as f64,
            )),
            so3_exp(&Vector3::new(
                0.025 * i as f64,
                -0.015 * (i * i) as f64,
                0.01 * (i % 3) as f64,
            )),
        ));
    }
    for case in fixture[key].as_array().unwrap() {
        let time = case["time"].as_i64().unwrap();
        let mut pose_j = PosePosSo3Jacobian::default();
        let mut gyro_j = So3Jacobian::default();
        let mut orientation_j = So3Jacobian::default();
        let mut accel_j = AccelPosSo3Jacobian::default();
        let mut position_j = PositionJacobian::default();
        let mut gyro_bias_j = SMatrix::zeros();
        let mut accel_bias_j = SMatrix::zeros();
        let mut gravity_j = SMatrix::zeros();
        let measurement = Vector3::new(0.1, -0.2, 0.3);
        let gravity = Vector3::new(0., 0., -9.81);
        check(
            &spline.pose(time, Some(&mut pose_j)).to_homogeneous(),
            &case["pose"],
        );
        check(&spline.d_pose_d_t(time), &case["time_jacobian"]);
        check(
            &spline.gyro_residual(
                time,
                &measurement,
                &CalibGyroBias::default(),
                Some(&mut gyro_j),
                Some(&mut gyro_bias_j),
            ),
            &case["gyro"],
        );
        check(
            &spline.accel_residual(
                time,
                &measurement,
                &CalibAccelBias::default(),
                &gravity,
                Some(&mut accel_j),
                Some(&mut accel_bias_j),
                Some(&mut gravity_j),
            ),
            &case["accel"],
        );
        check(
            &spline.position_residual(time, &measurement, Some(&mut position_j)),
            &case["position"],
        );
        check(
            &spline.orientation_residual(time, &so3_exp(&measurement), Some(&mut orientation_j)),
            &case["orientation"],
        );
        check(&gyro_bias_j, &case["gyro_bias_jacobian"]);
        check(&accel_bias_j, &case["accel_bias_jacobian"]);
        check(&gravity_j, &case["gravity_jacobian"]);
        for i in 0..N {
            check(&pose_j.d_val_d_knot[i], &case["pose_jacobian"][i]);
            check(&gyro_j.d_val_d_knot[i], &case["gyro_jacobian"][i]);
            check(&accel_j.d_val_d_knot[i], &case["accel_jacobian"][i]);
            check(
                &orientation_j.d_val_d_knot[i],
                &case["orientation_jacobian"][i],
            );
            let expected = case["position_jacobian"][i].as_f64().unwrap();
            assert!((position_j.d_val_d_knot[i] - expected).abs() <= 1e-10 * (1. + expected.abs()));
        }
    }
}
#[test]
fn official_se3_order4() {
    run::<4>("se3_4");
}
#[test]
fn official_se3_order5() {
    run::<5>("se3_5");
}
fn helper<G: SplineLieGroup<f64, D>, const D: usize>(
    key: &str,
    as_matrix: impl Fn(&G) -> Vec<f64>,
) {
    let fixture: Value = serde_json::from_str(include_str!("reference/se3_spline.json")).unwrap();
    let knots: [G; 4] = std::array::from_fn(|i| {
        G::exp(&SVector::from_fn(|k, _| 0.025 * i as f64 * (k + 1) as f64))
    });
    for case in fixture[key].as_array().unwrap() {
        let mut transform = knots[0].clone();
        let mut velocity = SVector::zeros();
        let mut acceleration = SVector::zeros();
        let mut jerk = SVector::zeros();
        CeresSplineHelper::<4>::evaluate_lie(
            &knots,
            case["u"].as_f64().unwrap(),
            50.,
            Some(&mut transform),
            Some(&mut velocity),
            Some(&mut acceleration),
            Some(&mut jerk),
        );
        for (a, b) in as_matrix(&transform)
            .iter()
            .zip(case["transform"].as_array().unwrap())
        {
            let b = b.as_f64().unwrap();
            assert!((a - b).abs() <= 1e-10 * (1. + b.abs()));
        }
        check(&velocity, &case["velocity"]);
        check(&acceleration, &case["acceleration"]);
        check(&jerk, &case["jerk"]);
    }
}
#[test]
fn original_ceres_so3() {
    helper::<UnitQuaternion<f64>, 3>("helper_so3_4", |rotation| {
        let matrix = rotation.to_rotation_matrix().into_inner();
        (0..3)
            .flat_map(|r| (0..3).map(move |c| matrix[(r, c)]))
            .collect()
    });
}
#[test]
fn original_ceres_se3() {
    helper::<Isometry3<f64>, 6>("helper_se3_4", |pose| {
        let matrix = pose.to_homogeneous();
        (0..4)
            .flat_map(|r| (0..4).map(move |c| matrix[(r, c)]))
            .collect()
    });
}

#[test]
fn original_ceres_euclidean() {
    let fixture: Value = serde_json::from_str(include_str!("reference/se3_spline.json")).unwrap();
    let knots: [Vector3<f64>; 4] = std::array::from_fn(|i| {
        Vector3::new(
            0.125 * i as f64,
            0.0625 * (i * i) as f64,
            -0.03125 * i as f64,
        )
    });
    for case in fixture["helper_rd_4"].as_array().unwrap() {
        let u = case["u"].as_f64().unwrap();
        let mut output = Vector3::zeros();
        CeresSplineHelper::<4>::evaluate::<f64, 3, 0>(&knots, u, 50., Some(&mut output));
        check(&output, &case["value"]);
        CeresSplineHelper::<4>::evaluate::<f64, 3, 1>(&knots, u, 50., Some(&mut output));
        check(&output, &case["velocity"]);
        CeresSplineHelper::<4>::evaluate::<f64, 3, 2>(&knots, u, 50., Some(&mut output));
        check(&output, &case["acceleration"]);
        CeresSplineHelper::<4>::evaluate::<f64, 3, 3>(&knots, u, 50., Some(&mut output));
        check(&output, &case["jerk"]);
    }
}

#[test]
fn coupled_se3_small_angle_and_pi_match_sophus() {
    let fixture: Value = serde_json::from_str(include_str!("reference/se3_spline.json")).unwrap();
    for case in fixture["group_ops"].as_array().unwrap() {
        let tangent = SVector::<f64, 6>::from_iterator(
            case["tangent"]
                .as_array()
                .unwrap()
                .iter()
                .map(|value| value.as_f64().unwrap()),
        );
        let pose = <Isometry3<f64> as SplineLieGroup<f64, 6>>::exp(&tangent);
        check(&pose.to_homogeneous(), &case["matrix"]);
        check(&SplineLieGroup::log(&pose), &case["log"]);
    }
}
