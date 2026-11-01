use firefly_basalt::utils::imu_types::{PoseStateWithLin, PoseVelBiasStateWithLin};
use firefly_basalt_headers::{
    imu::imu_types::PoseVelBiasState, scalar::BasaltScalar, utils::sophus_utils::so3_exp,
};
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
                "{actual} != {expected}"
            );
        }
    }
}
fn check<S: BasaltScalar>(state: &PoseVelBiasStateWithLin<S>, fixture: &Value, tolerance: f64) {
    assert_eq!(
        state.is_linearized(),
        fixture["linearized"].as_bool().unwrap()
    );
    assert_eq!(state.t_ns(), fixture["t_ns"].as_i64().unwrap());
    let current = state.state();
    compare(
        &current.state.pose.t_w_i.to_homogeneous(),
        &fixture["pose"],
        tolerance,
    );
    compare(&current.state.vel_w_i, &fixture["velocity"], tolerance);
    compare(&current.bias_gyro, &fixture["gyro"], tolerance);
    compare(&current.bias_accel, &fixture["accel"], tolerance);
    compare(state.delta(), &fixture["delta"], tolerance);
    compare(
        &state.state_lin().state.pose.t_w_i.to_homogeneous(),
        &fixture["linearized_pose"],
        tolerance,
    );
    let pose = PoseStateWithLin::from_imu_state(state);
    compare(
        &pose.pose().to_homogeneous(),
        &fixture["pose_only"],
        tolerance,
    );
    compare(pose.delta(), &fixture["pose_delta"], tolerance);
}
fn reference<S: BasaltScalar>(fixtures: &Value, tolerance: f64) {
    let v = |x, y, z| {
        Vector3::new(
            S::from_f64_lossy(x),
            S::from_f64_lossy(y),
            S::from_f64_lossy(z),
        )
    };
    let mut state = PoseVelBiasStateWithLin::new(
        PoseVelBiasState::new(
            123,
            Isometry3::from_parts(
                Translation3::from(v(1.0, 2.0, 3.0)),
                so3_exp(&v(0.2, -0.3, 0.4)),
            ),
            v(0.1, -0.2, 0.3),
            Vector3::zeros(),
            Vector3::zeros(),
        ),
        false,
    );
    let increment =
        SVector::<S, 15>::from_fn(|row, _| S::from_f64_lossy((row as f64 - 7.0) * 0.01));
    check(&state, &fixtures[0], tolerance);
    state.apply_inc(&increment);
    check(&state, &fixtures[1], tolerance);
    state.set_lin_true();
    state.backup();
    state.apply_inc(&increment);
    state.apply_inc(&increment);
    check(&state, &fixtures[2], tolerance);
    state.restore();
    check(&state, &fixtures[3], tolerance);
    state.apply_inc(&increment);
    state.set_lin_false();
    check(&state, &fixtures[4], tolerance);
    state.apply_inc(&increment);
    check(&state, &fixtures[5], tolerance);
}

#[test]
fn official_cpp_linearization_and_backup_lifecycle() {
    let fixtures: Value = serde_json::from_str(include_str!("reference/imu_types.json")).unwrap();
    reference::<f32>(&fixtures["f32"], 3e-6);
    reference::<f64>(&fixtures["f64"], 3e-14);
}

#[test]
fn pose_only_accumulates_at_fixed_linearization_and_restores() {
    let mut pose = PoseStateWithLin::<f64>::new(i64::MAX, Isometry3::identity(), false);
    let increment = SVector::<f64, 6>::from_row_slice(&[1.0, 2.0, 3.0, 0.1, -0.2, 0.3]);
    pose.apply_inc(&increment);
    let linearization = *pose.pose();
    pose.set_lin_true();
    pose.backup();
    pose.apply_inc(&increment);
    pose.apply_inc(&increment);
    assert_eq!(pose.delta(), &(increment * 2.0));
    assert_eq!(pose.pose_lin(), &linearization);
    assert_eq!(pose.t_ns(), i64::MAX);
    pose.restore();
    assert_eq!(pose.pose(), &linearization);
    assert_eq!(pose.delta(), &SVector::<f64, 6>::zeros());
    assert_eq!(pose.cast::<f32>().t_ns(), i64::MAX);
}
