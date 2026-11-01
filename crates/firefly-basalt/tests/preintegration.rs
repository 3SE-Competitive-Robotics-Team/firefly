use firefly_basalt::imu::imu_types::ImuData;
use firefly_basalt::imu::imu_types::PoseVelState;
use firefly_basalt::imu::preintegration::IntegratedImuMeasurement;
use firefly_basalt::imu::preintegration::Matrix9;
use firefly_basalt::imu::preintegration::Matrix9x3;
use firefly_basalt::utils::sophus_utils::so3_exp;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{Isometry3, SMatrix, SVector, Translation3, Vector3};
use serde_json::Value;

fn vector<S: BasaltScalar>(values: &Value) -> Vector3<S> {
    Vector3::from_fn(|row, _| S::from_f64_lossy(values[row].as_f64().unwrap()))
}

fn compare<S: BasaltScalar, const R: usize, const C: usize>(
    name: &str,
    actual: &SMatrix<S, R, C>,
    expected: &Value,
    tolerance: f64,
) {
    for row in 0..R {
        for col in 0..C {
            let expected = expected[row * C + col].as_f64().unwrap();
            let actual = actual[(row, col)].as_f64();
            assert!(
                (actual - expected).abs() <= tolerance * expected.abs().max(1.0),
                "{name}[{row},{col}]: {actual} != {expected}"
            );
        }
    }
}

// Keep each reference scenario and all of its dependent outputs together.
#[allow(clippy::too_many_lines)]
fn reference<S: BasaltScalar>(fixtures: &Value, tolerance: f64) {
    let v = |x, y, z| {
        Vector3::new(
            S::from_f64_lossy(x),
            S::from_f64_lossy(y),
            S::from_f64_lossy(z),
        )
    };
    for fixture in fixtures.as_array().unwrap() {
        let gyro_bias = v(0.01, -0.02, 0.03);
        let accel_bias = v(0.1, -0.2, 0.3);
        let mut measurement = IntegratedImuMeasurement::new(1_000_000_000, gyro_bias, accel_bias);
        for sample in fixture["samples"].as_array().unwrap() {
            let data = ImuData {
                t_ns: sample["t_ns"].as_i64().unwrap(),
                accel: vector(&sample["accel"]),
                gyro: vector(&sample["gyro"]),
            };
            measurement.integrate(&data, &v(0.01, 0.02, 0.03), &v(0.001, 0.002, 0.003));
        }
        compare(
            "delta_pose",
            &measurement.delta_state().pose.t_w_i.to_homogeneous(),
            &fixture["delta_pose"],
            tolerance,
        );
        compare(
            "delta_velocity",
            &measurement.delta_state().vel_w_i,
            &fixture["delta_velocity"],
            tolerance,
        );
        compare("cov", measurement.cov(), &fixture["cov"], tolerance);
        compare(
            "dba",
            measurement.d_state_d_ba(),
            &fixture["dba"],
            tolerance,
        );
        compare(
            "dbg",
            measurement.d_state_d_bg(),
            &fixture["dbg"],
            tolerance,
        );
        compare(
            "sqrt_cov_inv",
            measurement.sqrt_cov_inv(),
            &fixture["sqrt_cov_inv"],
            tolerance,
        );
        compare(
            "cov_inv",
            &measurement.cov_inv(),
            &fixture["cov_inv"],
            tolerance,
        );
        let initial = PoseVelState::new(
            1_000_000_000,
            Isometry3::from_parts(
                Translation3::from(v(1.0, 2.0, -3.0)),
                so3_exp(&v(0.2, -0.1, 0.3)),
            ),
            v(0.1, -0.2, 0.3),
        );
        let gravity = v(0.0, 0.0, -9.81);
        let mut predicted = PoseVelState::new(777, Isometry3::identity(), Vector3::zeros());
        measurement.predict_state(&initial, &gravity, &mut predicted);
        assert_eq!(
            predicted.pose.t_ns,
            fixture["predicted_t_ns"].as_i64().unwrap()
        );
        compare(
            "predicted_pose",
            &predicted.pose.t_w_i.to_homogeneous(),
            &fixture["predicted_pose"],
            tolerance,
        );
        compare(
            "predicted_velocity",
            &predicted.vel_w_i,
            &fixture["predicted_velocity"],
            tolerance,
        );
        predicted.apply_inc(&SVector::<S, 9>::from_fn(|row, _| {
            S::from_f64_lossy(0.002 * (row as f64 - 4.0))
        }));
        let mut j0 = Matrix9::zeros();
        let mut j1 = Matrix9::zeros();
        let mut gyro_jacobian = Matrix9x3::zeros();
        let mut accel_jacobian = Matrix9x3::zeros();
        let residual = measurement.residual(
            &initial,
            &gravity,
            &predicted,
            &(gyro_bias + v(0.001, -0.002, 0.003)),
            &(accel_bias + v(-0.01, 0.02, -0.03)),
            Some(&mut j0),
            Some(&mut j1),
            Some(&mut gyro_jacobian),
            Some(&mut accel_jacobian),
        );
        compare("residual", &residual, &fixture["residual"], tolerance);
        compare("j0", &j0, &fixture["j0"], tolerance);
        compare("j1", &j1, &fixture["j1"], tolerance);
        compare("jbg", &gyro_jacobian, &fixture["jbg"], tolerance);
        compare("jba", &accel_jacobian, &fixture["jba"], tolerance);
    }
}

#[test]
fn official_cpp_preintegration_f32() {
    let fixtures: Value =
        serde_json::from_str(include_str!("reference/primitives/preintegration.json")).unwrap();
    reference::<f32>(&fixtures["f32"], 1e-5);
}

#[test]
fn official_cpp_preintegration_f64() {
    let fixtures: Value =
        serde_json::from_str(include_str!("reference/primitives/preintegration.json")).unwrap();
    reference::<f64>(&fixtures["f64"], 1e-10);
}

#[test]
fn zero_noise_and_covariance_cache_invalidation() {
    let mut measurement = IntegratedImuMeasurement::<f64>::default();
    assert_eq!(*measurement.sqrt_cov_inv(), Matrix9::zeros());
    let noise = Vector3::repeat(0.01);
    for index in 1..=3 {
        measurement.integrate(
            &ImuData {
                t_ns: index * 10_000_000,
                accel: Vector3::new(1.0, 2.0, 9.0),
                gyro: Vector3::new(0.1, 0.2, 0.3),
            },
            &noise,
            &noise,
        );
    }
    let first = *measurement.sqrt_cov_inv();
    assert_ne!(first, Matrix9::zeros());
    let identity =
        measurement.sqrt_cov_inv() * measurement.cov() * measurement.sqrt_cov_inv().transpose();
    assert!((identity - Matrix9::identity()).amax() < 1e-10);
    measurement.integrate(
        &ImuData {
            t_ns: 40_000_000,
            accel: Vector3::new(1.0, 2.0, 9.0),
            gyro: Vector3::new(0.1, 0.2, 0.3),
        },
        &noise,
        &noise,
    );
    assert_ne!(*measurement.sqrt_cov_inv(), first);
    let mut noiseless = IntegratedImuMeasurement::<f64>::default();
    noiseless.integrate(
        &ImuData {
            t_ns: 10_000_000,
            accel: Vector3::zeros(),
            gyro: Vector3::zeros(),
        },
        &Vector3::zeros(),
        &Vector3::zeros(),
    );
    assert_eq!(noiseless.cov_inv(), Matrix9::zeros());
}

#[test]
fn propagation_state_and_measurement_jacobians_match_finite_differences() {
    let current = PoseVelState::new(
        100_000_000,
        Isometry3::from_parts(
            Translation3::new(1.0, -2.0, 3.0),
            so3_exp(&Vector3::new(0.2, -0.1, 0.3)),
        ),
        Vector3::new(0.5, -0.2, 0.1),
    );
    let data = ImuData {
        t_ns: 115_000_000,
        accel: Vector3::new(0.2, -0.3, 9.8),
        gyro: Vector3::new(0.4, 0.1, -0.2),
    };
    let mut state_jacobian = Matrix9::zeros();
    let mut acceleration_jacobian = Matrix9x3::zeros();
    let mut angular_rate_jacobian = Matrix9x3::zeros();
    let predicted = IntegratedImuMeasurement::propagate_state(
        &current,
        &data,
        Some(&mut state_jacobian),
        Some(&mut acceleration_jacobian),
        Some(&mut angular_rate_jacobian),
    );
    for axis in 0..9 {
        let mut increment = SVector::<f64, 9>::zeros();
        increment[axis] = 1e-6;
        let mut plus = current.clone();
        let mut minus = current.clone();
        plus.apply_inc(&increment);
        minus.apply_inc(&(-increment));
        let plus = IntegratedImuMeasurement::propagate_state(&plus, &data, None, None, None);
        let minus = IntegratedImuMeasurement::propagate_state(&minus, &data, None, None, None);
        let derivative = (predicted.diff(&plus) - predicted.diff(&minus)) / 2e-6;
        assert!((derivative - state_jacobian.column(axis)).amax() < 1e-8);
    }
    for is_acceleration in [true, false] {
        for axis in 0..3 {
            let mut plus = data.clone();
            let mut minus = data.clone();
            if is_acceleration {
                plus.accel[axis] += 1e-6;
                minus.accel[axis] -= 1e-6;
            } else {
                plus.gyro[axis] += 1e-6;
                minus.gyro[axis] -= 1e-6;
            }
            let plus = IntegratedImuMeasurement::propagate_state(&current, &plus, None, None, None);
            let minus =
                IntegratedImuMeasurement::propagate_state(&current, &minus, None, None, None);
            let derivative = (predicted.diff(&plus) - predicted.diff(&minus)) / 2e-6;
            let expected = if is_acceleration {
                acceleration_jacobian.column(axis)
            } else {
                angular_rate_jacobian.column(axis)
            };
            assert!((derivative - expected).amax() < 1e-8);
        }
    }
}
