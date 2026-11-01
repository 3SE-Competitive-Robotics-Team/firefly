use firefly_basalt::imu::imu_types::ImuData;
use firefly_basalt::imu::imu_types::PoseVelBiasState;
use firefly_basalt::imu::imu_types::PoseVelState;
use firefly_basalt::utils::sophus_utils::left_jacobian_inv_so3;
use firefly_basalt::utils::sophus_utils::left_jacobian_so3;
use firefly_basalt::utils::sophus_utils::right_jacobian_inv_so3;
use firefly_basalt::utils::sophus_utils::right_jacobian_so3;
use firefly_basalt::utils::sophus_utils::so3_exp;
use firefly_basalt::utils::sophus_utils::so3_log;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{Isometry3, SMatrix, SVector, Translation3, Vector3};
use serde_json::Value;

fn compare<S: BasaltScalar, const R: usize, const C: usize>(
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
                "[{row},{col}] {actual} != {expected}"
            );
        }
    }
}

fn reference<S: BasaltScalar>(fixtures: &Value, tolerance: f64) {
    let vector = |x, y, z| {
        Vector3::new(
            S::from_f64_lossy(x),
            S::from_f64_lossy(y),
            S::from_f64_lossy(z),
        )
    };
    for fixture in fixtures.as_array().unwrap() {
        let initial = PoseVelBiasState::new(
            1_234_567_890_123,
            Isometry3::from_parts(
                Translation3::from(vector(1.0, -2.0, 3.0)),
                so3_exp(&vector(0.2, -0.1, 0.4)),
            ),
            vector(0.1, 0.2, -0.3),
            vector(0.001, -0.002, 0.003),
            vector(-0.04, 0.05, 0.06),
        );
        let increment = SVector::<S, 15>::from_fn(|row, _| {
            S::from_f64_lossy(fixture["increment"][row].as_f64().unwrap())
        });
        let mut updated = initial.clone();
        updated.apply_inc(&increment);
        assert_eq!(updated.state.pose.t_ns, initial.state.pose.t_ns);
        compare(
            &updated.state.pose.t_w_i.to_homogeneous(),
            &fixture["pose"],
            tolerance,
        );
        compare(&updated.state.vel_w_i, &fixture["velocity"], tolerance);
        compare(&updated.bias_gyro, &fixture["gyro"], tolerance);
        compare(&updated.bias_accel, &fixture["accel"], tolerance);
        compare(&initial.diff(&updated), &fixture["diff"], tolerance);
        let rotation = increment.fixed_rows::<3>(3).into_owned();
        compare(&right_jacobian_so3(&rotation), &fixture["jr"], tolerance);
        compare(&left_jacobian_so3(&rotation), &fixture["jl"], tolerance);
        compare(
            &right_jacobian_inv_so3(&rotation),
            &fixture["jri"],
            tolerance,
        );
        compare(
            &left_jacobian_inv_so3(&rotation),
            &fixture["jli"],
            tolerance,
        );
    }
}

#[test]
fn official_cpp_state_increments_and_rotation_jacobians() {
    let fixtures: Value =
        serde_json::from_str(include_str!("reference/primitives/imu_types.json")).unwrap();
    reference::<f32>(&fixtures["f32"], 3e-6);
    reference::<f64>(&fixtures["f64"], 3e-14);
}

#[test]
fn state_defaults_and_cast_preserve_timestamp_and_bias_order() {
    let state = PoseVelBiasState::<f64>::default();
    assert_eq!(state.state.pose.t_w_i, Isometry3::identity());
    assert_eq!(state.state.vel_w_i, Vector3::zeros());
    let mut updated = state.clone();
    let increment = SVector::<f64, 15>::from_fn(|i, _| i as f64 * 0.01);
    updated.apply_inc(&increment);
    assert!((state.diff(&updated) - increment).amax() < 1e-14);
    updated.state.pose.t_ns = i64::MAX;
    let cast = updated.cast::<f32>();
    assert_eq!(cast.state.pose.t_ns, i64::MAX);
    assert_eq!(cast.bias_gyro, Vector3::new(0.09, 0.10, 0.11));
    assert_eq!(cast.bias_accel, Vector3::new(0.12, 0.13, 0.14));
    let sample = ImuData::<f64> {
        t_ns: i64::MIN,
        accel: Vector3::new(1.0, 2.0, 3.0),
        gyro: Vector3::new(4.0, 5.0, 6.0),
    };
    assert_eq!(sample.cast::<f32>().t_ns, i64::MIN);
    assert_eq!(sample.cast::<f32>().gyro, Vector3::new(4.0, 5.0, 6.0));
    assert_eq!(
        PoseVelState::<f64>::default().diff(&PoseVelState::default()),
        SVector::<f64, 9>::zeros()
    );
}

#[test]
fn rotation_log_exp_and_jacobian_inverse_cover_zero_and_pi() {
    for phi in [
        Vector3::zeros(),
        Vector3::new(1e-12, -2e-12, 3e-12),
        Vector3::new(0.2, -0.4, 0.8),
        Vector3::new(std::f64::consts::PI, 0.0, 0.0),
    ] {
        assert!((so3_log(&so3_exp(&phi)) - phi).amax() < 1e-14);
        let product = right_jacobian_so3(&phi) * right_jacobian_inv_so3(&phi);
        assert!((product - SMatrix::<f64, 3, 3>::identity()).amax() < 1e-14);
    }
}
