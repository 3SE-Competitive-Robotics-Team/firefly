use firefly_basalt_headers::{
    scalar::BasaltScalar,
    spline::so3_spline::{Jacobian, So3Spline},
    utils::sophus_utils::so3_exp,
};
use nalgebra::{SMatrix, Vector3};
use serde_json::Value;
fn check<S: BasaltScalar, const R: usize, const C: usize>(
    actual: &SMatrix<S, R, C>,
    expected: &Value,
) {
    let tolerance = if std::mem::size_of::<S>() == 4 {
        1e-5
    } else {
        1e-10
    };
    for row in 0..R {
        for column in 0..C {
            let a = actual[(row, column)].as_f64();
            let b = expected[row * C + column].as_f64().unwrap();
            assert!(
                (a - b).abs() <= tolerance * (1. + b.abs()),
                "[{row},{column}] {a} != {b}"
            );
        }
    }
}
fn run<S: BasaltScalar, const N: usize>(key: &str) {
    let fixture: Value = serde_json::from_str(include_str!("reference/so3_spline.json")).unwrap();
    let mut spline = So3Spline::<S, N>::new(20_000_000, 1_000_000_000);
    for i in 0..N + 3 {
        let scalar = S::from_f64_lossy;
        spline.knots_push_back(so3_exp(&Vector3::new(
            scalar(0.025 * i as f64),
            scalar(-0.015 * (i * i) as f64),
            scalar(0.01 * (i % 3) as f64),
        )));
    }
    for case in fixture[key].as_array().unwrap() {
        let time = case["time"].as_i64().unwrap();
        let mut jacobian = Jacobian::default();
        check(
            &spline
                .evaluate(time, Some(&mut jacobian))
                .to_rotation_matrix()
                .into_inner(),
            &case["rotation"],
        );
        for i in 0..N {
            check(&jacobian.d_val_d_knot[i], &case["position_jacobian"][i]);
        }
        check(&spline.velocity_body(time, None), &case["velocity"]);
        check(
            &spline.acceleration_body(time, None, None, None),
            &case["acceleration"],
        );
        check(&spline.jerk_body(time, None, None), &case["jerk"]);
        if !case["velocity_jacobian"].is_null() {
            check(
                &spline.velocity_body(time, Some(&mut jacobian)),
                &case["velocity_jacobian_value"],
            );
            for i in 0..N {
                check(&jacobian.d_val_d_knot[i], &case["velocity_jacobian"][i]);
            }
            let mut velocity_jacobian = Jacobian::default();
            check(
                &spline.acceleration_body(
                    time,
                    Some(&mut jacobian),
                    None,
                    Some(&mut velocity_jacobian),
                ),
                &case["acceleration_jacobian_value"],
            );
            for i in 0..N {
                check(&jacobian.d_val_d_knot[i], &case["acceleration_jacobian"][i]);
                check(
                    &velocity_jacobian.d_val_d_knot[i],
                    &case["velocity_from_acceleration_jacobian"][i],
                );
            }
        }
    }
}
#[test]
fn official_f64_order4() {
    run::<f64, 4>("f64_4");
}
#[test]
fn official_f64_order5() {
    run::<f64, 5>("f64_5");
}
#[test]
fn official_f32_order4() {
    run::<f32, 4>("f32_4");
}
#[test]
fn official_f32_order5() {
    run::<f32, 5>("f32_5");
}
