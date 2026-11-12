use firefly_basalt::{
    linearization::{
        landmark_block::{Options, RelPoseLin},
        landmark_block_abs_dynamic::LandmarkBlockAbsDynamic,
    },
    utils::imu_types::AbsOrderMap,
    vi_estimator::landmark_database::Keypoint,
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::observation::TimeCamId;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector, Matrix4, SMatrix, Vector2, Vector4};
use serde_json::Value;
use std::collections::HashMap;
fn number(value: &Value) -> f64 {
    value
        .as_f64()
        .unwrap_or_else(|| match value.as_str().unwrap() {
            "NaN" => f64::NAN,
            "Infinity" => f64::INFINITY,
            "-Infinity" => f64::NEG_INFINITY,
            _ => panic!("invalid scalar"),
        })
}
fn check<S: BasaltScalar>(actual: &DMatrix<S>, expected: &Value, tolerance: f64, label: &str) {
    assert_eq!(actual.nrows() as u64, expected["rows"].as_u64().unwrap());
    assert_eq!(actual.ncols() as u64, expected["cols"].as_u64().unwrap());
    let values = expected["values"].as_array().unwrap();
    for row in 0..actual.nrows() {
        for col in 0..actual.ncols() {
            let a = actual[(row, col)].as_f64();
            let b = number(&values[row * actual.ncols() + col]);
            assert!(
                a == b || a.is_nan() && b.is_nan() || (a - b).abs() <= tolerance * (1. + b.abs()),
                "{label} [{row},{col}]: {a} != {b}"
            );
        }
    }
}
#[expect(
    clippy::too_many_lines,
    reason = "Ordered independent reference checks span a complete landmark block lifecycle"
)]
fn run<S: BasaltScalar>(text: &str, tolerance: f64) {
    let data: Value = serde_json::from_str(text).unwrap();
    let s = S::from_f64_lossy;
    for case in data.as_array().unwrap() {
        let mode = case["mode"].as_str().unwrap();
        let mut point = Keypoint::new(
            Vector2::new(s(0.15), s(-0.2)),
            s(number(&case["inverse_distance"])),
            TimeCamId {
                frame_id: 10,
                cam_id: 0,
            },
        );
        let mut relative = HashMap::new();
        for obs in case["observations"].as_array().unwrap() {
            let target = TimeCamId {
                frame_id: obs["frame"].as_i64().unwrap(),
                cam_id: obs["camera"].as_u64().unwrap() as usize,
            };
            let pixel = &obs["pixel"]["values"];
            point.obs.insert(
                target,
                Vector2::new(s(number(&pixel[0])), s(number(&pixel[1]))),
            );
            relative.insert(
                (point.host_kf_id, target),
                RelPoseLin {
                    t_t_h: Matrix4::from_row_iterator(
                        obs["transform"]["values"]
                            .as_array()
                            .unwrap()
                            .iter()
                            .map(|v| s(number(v))),
                    ),
                    d_rel_d_h: SMatrix::identity(),
                    d_rel_d_t: -SMatrix::<S, 6, 6>::identity(),
                },
            );
        }
        let mut order = AbsOrderMap::default();
        order.abs_order_map.insert(10, (0, 6));
        order.abs_order_map.insert(20, (6, 6));
        order.abs_order_map.insert(30, (12, 6));
        order.total_size = 18;
        order.items = 3;
        let mut camera = GenericCamera::from_name("pinhole");
        camera.set_from_init(Vector4::new(s(320.), s(315.), s(320.), s(240.)));
        let calibration = Calibration {
            intrinsics: vec![camera.clone(), camera],
            ..Calibration::default()
        };
        let options = Options {
            use_householder: case["householder"].as_bool().unwrap(),
            huber_parameter: if mode == "huber" { s(2.) } else { S::zero() },
            obs_std_dev: s(1.5),
            ..Options::default()
        };
        let mut block = LandmarkBlockAbsDynamic::<S, 6>::new(1, &point, &relative, &order, options);
        let error = block.linearize_landmark(&point, &calibration);
        assert!(
            (error.as_f64() - number(&case["error"])).abs()
                <= tolerance * (1. + number(&case["error"]).abs())
        );
        for stage in ["linearized", "scaled", "qr", "pose_scaled"] {
            match stage {
                "scaled" => block.scale_landmark_columns(),
                "qr" => block.perform_qr(),
                "pose_scaled" => block.scale_pose_columns(&DVector::from_fn(18, |i, _| {
                    S::one() + s(i as f64) * s(0.015_625)
                })),
                _ => {}
            }
            check(
                block.storage(),
                &case[stage],
                tolerance,
                &format!("{mode} {stage}"),
            );
        }
        for (index, damping) in [0.25, 2., 0., 0.125].into_iter().enumerate() {
            block.set_landmark_damping(s(damping));
            check(
                block.storage(),
                &case["damped"][index],
                tolerance,
                &format!("{mode} householder {} damped {index}", case["householder"]),
            );
        }
        let mut hessian = DMatrix::zeros(18, 18);
        let mut rhs = DVector::zeros(18);
        block.add_dense_h_b(&mut hessian, &mut rhs);
        check_normal_equations(
            block.storage(),
            &case["damped"][3],
            &hessian,
            &rhs,
            &case["h"],
            &case["b"],
        );
        check_sparse_accumulation(&point, &relative, &calibration, options);
        let increment = DVector::from_fn(18, |i, _| s(i as f64 - 8.) * s(0.000_125));
        let mut decrease = s(0.125);
        block.back_substitute(&mut point, &increment, &mut decrease);
        check(
            block.storage(),
            &case["undamped"],
            tolerance,
            &format!("{mode} undamped"),
        );
        check(
            &DMatrix::from_column_slice(2, 1, point.direction.as_slice()),
            &case["direction"],
            tolerance,
            &format!("{mode} direction"),
        );
        assert!(
            (point.inv_dist.as_f64() - number(&case["updated_inverse_distance"])).abs()
                <= tolerance * (1. + number(&case["updated_inverse_distance"]).abs())
        );
        assert!(
            (decrease.as_f64() - number(&case["model_decrease"])).abs()
                <= tolerance * (1. + number(&case["model_decrease"]).abs())
        );
    }
}
// Exercise mixed pose/state ordering, unobserved poses and accumulation into
// nonzero outputs. The independent dense product includes every padded column.
fn check_sparse_accumulation<S: BasaltScalar>(
    point: &Keypoint<S>,
    relative: &HashMap<(TimeCamId, TimeCamId), RelPoseLin<S>>,
    calibration: &Calibration<S>,
    options: Options<S>,
) {
    let mut order = AbsOrderMap::default();
    order
        .abs_order_map
        .extend([(10, (15, 15)), (20, (30, 15)), (30, (0, 15)), (40, (45, 6))]);
    order.total_size = 51;
    order.items = 4;
    let mut block = LandmarkBlockAbsDynamic::<S, 6>::new(1, point, relative, &order, options);
    block.linearize_landmark(point, calibration);
    block.perform_qr();
    let mut jacobian = DMatrix::zeros(block.num_q2_rows(), order.total_size);
    let mut residual = DVector::zeros(block.num_q2_rows());
    block.get_dense_q2jp_q2r(&mut jacobian, &mut residual, 0);
    let initial_h =
        |row: usize, col: usize| S::from_f64_lossy(0.25 + row as f64 * 0.003 - col as f64 * 0.007);
    let mut h = DMatrix::from_fn(51, 51, initial_h);
    let mut b = DVector::repeat(51, S::from_f64_lossy(-0.5));
    block.add_dense_h_b(&mut h, &mut b);
    block.add_dense_h_b(&mut h, &mut b);
    let operations = (2 * jacobian.nrows() + 4) as f64;
    let unit_roundoff = S::default_epsilon().as_f64() / 2.;
    let gamma = operations * unit_roundoff / (1. - operations * unit_roundoff);
    for row in 0..51 {
        for col in 0..=51 {
            let (initial, actual) = if col == 51 {
                (-0.5_f64, b[row])
            } else {
                (initial_h(row, col).as_f64(), h[(row, col)])
            };
            let mut expected = initial;
            let mut magnitude = initial.abs();
            for k in 0..jacobian.nrows() {
                let right = if col == 51 {
                    residual[k]
                } else {
                    jacobian[(k, col)]
                };
                let product = jacobian[(k, row)].as_f64() * right.as_f64() * 2.;
                expected += product;
                magnitude += product.abs();
            }
            // Bound both the accumulation and the independent f64 oracle.
            let bound = 2. * gamma * magnitude;
            assert!(
                (actual.as_f64() - expected).abs() <= bound,
                "sparse accumulation [{row},{col}]: {} != {expected}, bound {bound}",
                actual.as_f64()
            );
        }
    }
}
// The componentwise dot-product forward-error theorem, with explicit input
// perturbations. A relative tolerance on H alone is invalid under cancellation:
// even the C++ oracle's exact exported-J dot differs by 4.03e-4 at H[4,16].
// J and residuals themselves retain the fixed oracle thresholds above.
fn check_normal_equations<S: BasaltScalar>(
    storage: &DMatrix<S>,
    reference: &Value,
    hessian: &DMatrix<S>,
    rhs: &DVector<S>,
    expected_h: &Value,
    expected_b: &Value,
) {
    let rows = storage.nrows() - 3;
    let columns = storage.ncols();
    let values = reference["values"].as_array().unwrap();
    let unit_roundoff = if std::mem::size_of::<S>() == 4 {
        f64::from(f32::EPSILON) / 2.
    } else {
        f64::EPSILON / 2.
    };
    let operations = 2. * rows as f64;
    let gamma = operations * unit_roundoff / (1. - operations * unit_roundoff);
    for first in 0..hessian.nrows() {
        for second in 0..=hessian.ncols() {
            let other = if second == hessian.ncols() {
                columns - 1
            } else {
                second
            };
            let mut input_error = 0.;
            let mut product_magnitude = 0.;
            for row in 3..storage.nrows() {
                let a = storage[(row, first)].as_f64();
                let b = storage[(row, other)].as_f64();
                let x = number(&values[row * columns + first]);
                let y = number(&values[row * columns + other]);
                input_error += (a - x).abs() * b.abs() + x.abs() * (b - y).abs();
                product_magnitude += (a * b).abs() + (x * y).abs();
            }
            // One final f64 rounding allowance accounts for computing this bound.
            let bound =
                (input_error + gamma * product_magnitude) * (1. + operations * f64::EPSILON);
            let (actual, expected) = if second == hessian.ncols() {
                (rhs[first].as_f64(), number(&expected_b["values"][first]))
            } else {
                (
                    hessian[(first, second)].as_f64(),
                    number(&expected_h["values"][first * hessian.ncols() + second]),
                )
            };
            assert!(
                (actual - expected).abs() <= bound,
                "normal equations [{first},{second}]: {actual} != {expected}; proved bound {bound}"
            );
        }
    }
}
#[test]
fn original_landmark_block_f32() {
    run::<f32>(
        include_str!("reference/landmark_block_abs_dynamic_f32.json"),
        1e-5,
    );
}
#[test]
fn original_landmark_block_f64() {
    run::<f64>(
        include_str!("reference/landmark_block_abs_dynamic_f64.json"),
        1e-10,
    );
}
