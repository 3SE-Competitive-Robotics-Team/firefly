use firefly_basalt::imu::imu_types::ImuData;
use firefly_basalt::imu::imu_types::PoseVelBiasState;
use firefly_basalt::imu::preintegration::IntegratedImuMeasurement;
use firefly_basalt::utils::sophus_utils::so3_exp;
use firefly_basalt::{
    linearization::imu_block::ImuBlock,
    optimization::accumulator::DenseAccumulator,
    utils::imu_types::{AbsOrderMap, ImuLinData, PoseVelBiasStateWithLin},
};
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector, Isometry3, SVector, Translation3, Vector3};
use serde_json::Value;
use std::collections::BTreeMap;

fn compare<S: BasaltScalar>(name: &str, actual: &DMatrix<S>, expected: &Value, tolerance: f64) {
    for row in 0..actual.nrows() {
        for column in 0..actual.ncols() {
            let expected = expected[row * actual.ncols() + column].as_f64().unwrap();
            let actual = actual[(row, column)].as_f64();
            assert!(
                (actual - expected).abs() <= tolerance * expected.abs().max(1.0),
                "{name}[{row},{column}]: {actual} != {expected}"
            );
        }
    }
}
fn compare_vector<S: BasaltScalar>(
    name: &str,
    actual: &DVector<S>,
    expected: &Value,
    tolerance: f64,
) {
    compare(
        name,
        &DMatrix::from_column_slice(actual.len(), 1, actual.as_slice()),
        expected,
        tolerance,
    );
}

/// Dot-product forward bound: input perturbations plus gamma_(2n) rounding.
/// The factor 2n counts each multiplication and addition; it is not fitted to
/// observed errors. Cancellation can make a relative bound on the result invalid.
fn dot_bound<S: BasaltScalar>(left: &[(f64, f64)], right: &[(f64, f64)]) -> f64 {
    let unit_roundoff = S::default_epsilon().as_f64() / 2.0;
    let operations = (2 * left.len()) as f64;
    let gamma = operations * unit_roundoff / (1.0 - operations * unit_roundoff);
    let mut input_error = 0.0;
    let mut product_magnitude = 0.0;
    for (&(actual_left, expected_left), &(actual_right, expected_right)) in left.iter().zip(right) {
        input_error += (actual_left - expected_left).abs() * actual_right.abs()
            + expected_left.abs() * (actual_right - expected_right).abs();
        product_magnitude +=
            (actual_left * actual_right).abs() + (expected_left * expected_right).abs();
    }
    // Inflate only for the f64 arithmetic used to evaluate this positive bound.
    let bound_operations = (12 * left.len() + 8) as f64;
    let bound_gamma = bound_operations * f64::EPSILON / (1.0 - bound_operations * f64::EPSILON);
    (input_error + gamma * product_magnitude) * (1.0 + bound_gamma)
}

fn compare_normal_equations<S: BasaltScalar>(
    accumulator: &DenseAccumulator<S>,
    jacobian: &DMatrix<S>,
    residual: &DVector<S>,
    fixture: &Value,
) {
    let column = |col: usize| {
        (0..jacobian.nrows())
            .map(|row| {
                (
                    jacobian[(row, col)].as_f64(),
                    fixture["jacobian"][row * jacobian.ncols() + col]
                        .as_f64()
                        .unwrap(),
                )
            })
            .collect::<Vec<_>>()
    };
    let residuals: Vec<_> = residual
        .iter()
        .enumerate()
        .map(|(row, value)| (value.as_f64(), fixture["residual"][row].as_f64().unwrap()))
        .collect();
    for row in 0..jacobian.ncols() {
        let left = column(row);
        for col in 0..jacobian.ncols() {
            let expected = fixture["h"][row * jacobian.ncols() + col].as_f64().unwrap();
            let actual = accumulator.h()[(row, col)].as_f64();
            let bound = dot_bound::<S>(&left, &column(col));
            assert!(
                bound.is_finite() && (actual - expected).abs() <= bound,
                "H[{row},{col}]: {actual} != {expected}; IEEE forward bound {bound}"
            );
        }
        let expected = fixture["b"][row].as_f64().unwrap();
        let actual = accumulator.b()[row].as_f64();
        let bound = dot_bound::<S>(&left, &residuals);
        assert!(
            bound.is_finite() && (actual - expected).abs() <= bound,
            "b[{row}]: {actual} != {expected}; IEEE forward bound {bound}"
        );
    }
}

// Keep one complete source fixture scenario and all coupled outputs together.
#[allow(clippy::too_many_lines)]
fn reference<S: BasaltScalar>(fixture: &Value, tolerance: f64) {
    let v = |x, y, z| {
        Vector3::new(
            S::from_f64_lossy(x),
            S::from_f64_lossy(y),
            S::from_f64_lossy(z),
        )
    };
    let gyro_bias = v(0.01, -0.02, 0.03);
    let accel_bias = v(0.1, -0.2, 0.3);
    let mut measurement = IntegratedImuMeasurement::new(1_000_000_000, gyro_bias, accel_bias);
    for index in 1..=6 {
        let t = f64::from(index);
        measurement.integrate(
            &ImuData {
                t_ns: 1_000_000_000 + i64::from(index) * 5_000_000,
                accel: v(0.1 * t, -0.05 * t, 9.81 + 0.02 * t),
                gyro: v(0.2 + 0.01 * t, -0.1 + 0.02 * t, 0.3 - 0.01 * t),
            },
            &v(0.01, 0.01, 0.01),
            &v(0.001, 0.001, 0.001),
        );
    }
    let gravity = v(0.0, 0.0, -9.81);
    let gyro_weight = v(10.0, 20.0, 30.0);
    let accel_weight = v(2.0, 3.0, 4.0);
    let initial = PoseVelBiasState::new(
        1_000_000_000,
        Isometry3::from_parts(
            Translation3::from(v(1.0, 2.0, -3.0)),
            so3_exp(&v(0.2, -0.1, 0.3)),
        ),
        v(0.1, -0.2, 0.3),
        gyro_bias,
        accel_bias,
    );
    let mut next = initial.clone();
    next.state.pose.t_ns = 1_030_000_000;
    measurement.predict_state(&initial.state, &gravity, &mut next.state);
    let mut states = BTreeMap::from([
        (
            initial.state.pose.t_ns,
            PoseVelBiasStateWithLin::new(initial, false),
        ),
        (
            next.state.pose.t_ns,
            PoseVelBiasStateWithLin::new(next, false),
        ),
    ]);
    let increment = SVector::<S, 15>::from_fn(|i, _| S::from_f64_lossy((i as f64 - 7.0) * 0.0001));
    for (timestamp, state) in &mut states {
        state.set_lin_true();
        state.apply_inc(
            &(if *timestamp == 1_000_000_000 {
                increment
            } else {
                -increment
            }),
        );
    }
    let order = AbsOrderMap {
        abs_order_map: BTreeMap::from([(1_000_000_000, (5, 15)), (1_030_000_000, (20, 15))]),
        items: 2,
        total_size: 37,
    };
    let data = ImuLinData {
        g: &gravity,
        gyro_bias_weight_sqrt: &gyro_weight,
        accel_bias_weight_sqrt: &accel_weight,
        imu_meas: BTreeMap::new(),
    };
    let mut block = ImuBlock::new(&measurement, &data, &order);
    let error = block.linearize_imu(&states).as_f64();
    let expected = fixture["error"].as_f64().unwrap();
    assert!(
        (error - expected).abs() <= tolerance * expected.abs().max(1.0),
        "error {error} != {expected}"
    );
    let mut jacobian = DMatrix::zeros(17, 37);
    let mut residual = DVector::zeros(17);
    block.add_dense_q2jp_q2r(&mut jacobian, &mut residual, 1);
    compare("jacobian", &jacobian, &fixture["jacobian"], tolerance);
    compare_vector("residual", &residual, &fixture["residual"], tolerance);
    let mut accumulator = DenseAccumulator::default();
    accumulator.reset(37);
    block.add_dense_h_b(&mut accumulator);
    compare_normal_equations(&accumulator, &jacobian, &residual, fixture);
    let mut diagonal = DVector::zeros(37);
    block.add_jp_diag2(&mut diagonal);
    compare_vector("diagonal", &diagonal, &fixture["diagonal"], tolerance);
    let scaling = DVector::from_fn(37, |i, _| {
        S::one() / (S::one() + S::from_f64_lossy(0.03) * S::from_usize(i).unwrap())
    });
    let increment = DVector::from_fn(37, |i, _| S::from_f64_lossy((i as f64 - 18.0) * 0.00001));
    block.scale_jp_cols(&scaling);
    let mut scaled = DMatrix::zeros(17, 37);
    let mut scaled_residual = DVector::zeros(17);
    block.add_dense_q2jp_q2r(&mut scaled, &mut scaled_residual, 1);
    compare("scaled", &scaled, &fixture["scaled"], tolerance);
    compare_vector(
        "scaled_residual",
        &scaled_residual,
        &fixture["scaled_residual"],
        tolerance,
    );
    let mut decrease = S::from_f64_lossy(7.0);
    block.back_substitute(&increment, &mut decrease);
    let actual = decrease.as_f64();
    let expected = fixture["decrease"].as_f64().unwrap();
    assert!(
        (actual - expected).abs() <= tolerance * expected.abs().max(1.0),
        "decrease {actual} != {expected}"
    );
}

#[test]
fn official_cpp_imu_block_f32() {
    let fixtures: Value = serde_json::from_str(include_str!("reference/imu_block.json")).unwrap();
    reference::<f32>(&fixtures["f32"], 1e-5);
}
#[test]
fn official_cpp_imu_block_f64() {
    let fixtures: Value = serde_json::from_str(include_str!("reference/imu_block.json")).unwrap();
    reference::<f64>(&fixtures["f64"], 1e-10);
}
