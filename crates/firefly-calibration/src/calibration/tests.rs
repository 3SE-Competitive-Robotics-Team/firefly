use super::*;
use crate::scalar::BasaltScalar;
use nalgebra::{Isometry3, SMatrix, Translation3, UnitQuaternion, Vector3};
use serde_json::Value;
fn compare<S: BasaltScalar, const R: usize, const C: usize>(
    actual: &SMatrix<S, R, C>,
    expected: &Value,
    tol: f64,
) {
    for row in 0..R {
        for col in 0..C {
            let a = actual[(row, col)].as_f64();
            let e = expected[row * C + col].as_f64().unwrap();
            assert!((a - e).abs() < tol, "({row},{col}) {a} != {e}");
        }
    }
}
fn run<S: BasaltScalar>(expected: &Value, tol: f64) -> Calibration<S> {
    let mut calibration = Calibration::<S>::default();
    for i in 0..9 {
        calibration.calib_accel_bias.param_mut()[i] =
            S::from_f64_lossy(0.001 * (f64::from(i32::try_from(i).unwrap()) - 4.0));
    }
    for i in 0..12 {
        calibration.calib_gyro_bias.param_mut()[i] =
            S::from_f64_lossy(0.002 * (f64::from(i32::try_from(i).unwrap()) - 5.0));
    }
    let raw = Vector3::new(
        S::from_f64_lossy(1.2),
        S::from_f64_lossy(-0.3),
        S::from_f64_lossy(9.81),
    );
    let (bias, scale) = calibration.calib_accel_bias.bias_and_scale();
    compare(&bias, &expected["accel_bias"], tol);
    compare(&scale, &expected["accel_scale"], tol);
    let (bias, scale) = calibration.calib_gyro_bias.bias_and_scale();
    compare(&bias, &expected["gyro_bias"], tol);
    compare(&scale, &expected["gyro_scale"], tol);
    compare(
        &calibration.calib_accel_bias.calibrated(&raw),
        &expected["accel"],
        tol,
    );
    compare(
        &calibration.calib_gyro_bias.calibrated(&raw),
        &expected["gyro"],
        tol,
    );
    compare(
        &calibration.dicrete_time_accel_noise_std(),
        &expected["accel_discrete"],
        tol,
    );
    compare(
        &calibration.dicrete_time_gyro_noise_std(),
        &expected["gyro_discrete"],
        tol,
    );
    calibration
}
#[test]
fn calibration_and_bias_match_cpp() {
    let reference: Value =
        serde_json::from_str(include_str!("../../tests/reference/calibration.json")).unwrap();
    run::<f32>(&reference["f32"], 1e-6);
    let mut calibration = run::<f64>(&reference["f64"], 1e-12);
    let raw = Vector3::new(1.2, -0.3, 9.81);
    compare(
        &calibration
            .calib_accel_bias
            .invert_calibration(&calibration.calib_accel_bias.calibrated(&raw)),
        &reference["f64"]["inverse_accel"],
        1e-12,
    );
    compare(
        &calibration
            .calib_gyro_bias
            .invert_calibration(&calibration.calib_gyro_bias.calibrated(&raw)),
        &reference["f64"]["inverse_gyro"],
        1e-12,
    );
    calibration.t_i_c.push(Isometry3::from_parts(
        Translation3::new(0.1, 0.2, 0.3),
        UnitQuaternion::from_scaled_axis(Vector3::new(0.2, -0.1, 0.3)),
    ));
    let cast = calibration.cast::<f32>();
    compare(
        &cast.t_i_c[0].to_homogeneous(),
        &reference["f64"]["cast_pose"],
        1e-6,
    );
    compare(
        cast.calib_accel_bias.param(),
        &reference["f64"]["cast_accel_bias"],
        1e-9,
    );
}
