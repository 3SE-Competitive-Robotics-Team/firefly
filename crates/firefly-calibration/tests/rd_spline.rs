use firefly_calibration::scalar::BasaltScalar;
use firefly_calibration::spline::rd_bezier::IntegralJacobian as BezierIntegral;
use firefly_calibration::spline::rd_spline::IntegralJacobian;
use firefly_calibration::spline::rd_spline::RdSpline;
use firefly_calibration::spline::spline_common;
use firefly_calibration::spline::spline_common::Jacobian;
use nalgebra::{SMatrix, SVector};
use serde_json::Value;

fn vector<S: BasaltScalar, const N: usize>(value: &Value) -> SVector<S, N> {
    SVector::from_fn(|r, _| S::from_f64_lossy(value[r].as_f64().unwrap()))
}
fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance * expected.abs().max(1.),
        "{actual} != {expected}"
    );
}
fn matrix<S: BasaltScalar, const R: usize, const C: usize>(
    actual: &SMatrix<S, R, C>,
    expected: &Value,
    tol: f64,
) {
    for r in 0..R {
        for c in 0..C {
            let a = actual[(r, c)].as_f64();
            let e = expected[r * C + c].as_f64().unwrap();
            assert!(
                (a - e).abs() <= tol * e.abs().max(1.),
                "[{r},{c}] {a} != {e}; expected {expected}"
            );
        }
    }
}
fn make<S: BasaltScalar, const N: usize>(case: &Value) -> RdSpline<S, 3, N> {
    let mut curve = RdSpline::new(200_000_000, 123_456_789);
    for knot in case["knots"].as_array().unwrap() {
        curve.knots_push_back(vector(knot));
    }
    curve
}
fn reference<S: BasaltScalar, const N: usize>(case: &Value, tol: f64) {
    matrix(
        &spline_common::compute_blending_matrix::<S, N, false>(),
        &case["blend"],
        tol,
    );
    matrix(
        &spline_common::compute_blending_matrix::<S, N, true>(),
        &case["cumulative"],
        tol,
    );
    matrix(
        &spline_common::compute_blending_matrix_bezier::<S, N>(),
        &case["bezier_blend"],
        tol,
    );
    matrix(
        &spline_common::compute_base_coefficients::<S, N>(),
        &case["base"],
        tol,
    );
    for (i, q) in spline_common::compute_quadratic_coefficients::<S, N>()
        .iter()
        .enumerate()
    {
        matrix(q, &case["quadratic"][i], tol);
    }
    let mut curve = make::<S, N>(case);
    assert_eq!(curve.min_time_ns(), case["min"].as_i64().unwrap());
    assert_eq!(curve.max_time_ns(), case["max"].as_i64().unwrap());
    for row in case["evaluate"].as_array().unwrap() {
        let time = row["time"].as_i64().unwrap();
        let mut j = Jacobian::default();
        let result = match row["derivative"].as_u64().unwrap() {
            0 => curve.evaluate::<0>(time, Some(&mut j)),
            1 => curve.velocity(time, Some(&mut j)),
            2 => curve.acceleration(time, Some(&mut j)),
            _ => unreachable!(),
        };
        matrix(&result, &row["value"], tol);
        assert_eq!(j.start_idx as u64, row["start"].as_u64().unwrap());
        matrix(
            &SVector::<S, N>::from(j.d_val_d_knot),
            &row["jacobian"],
            tol,
        );
    }
    curve.knots_pop_front();
    curve.knots_pop_back();
    assert_eq!(curve.min_time_ns(), case["popped_min"].as_i64().unwrap());
    assert_eq!(curve.max_time_ns(), case["popped_max"].as_i64().unwrap());
    matrix(curve.knots_front(), &case["popped_front"], tol);
    let converted = curve.cast::<f64>();
    assert_eq!(converted.time_interval_ns(), curve.time_interval_ns());
    for (actual, original) in converted.knots().iter().zip(curve.knots()) {
        assert_eq!(*actual, original.map(BasaltScalar::as_f64));
    }
}
fn integral_reference<const N: usize>(case: &Value) {
    let curve = make::<f64, N>(case);
    let tol = 2e-14;
    for row in case["integral"].as_array().unwrap() {
        let mut j = IntegralJacobian::default();
        let value = match row["derivative"].as_u64().unwrap() {
            0 => curve.evaluate_integral_squared::<0>(Some(&mut j)),
            1 => curve.evaluate_integral_squared::<1>(Some(&mut j)),
            2 => curve.evaluate_integral_squared::<2>(Some(&mut j)),
            _ => curve.evaluate_integral_squared::<N>(Some(&mut j)),
        };
        close(value, row["value"].as_f64().unwrap(), tol);
        for (i, knot) in j.d_int_d_knot.iter().enumerate() {
            matrix(knot, &row["jacobian"][i], tol);
        }
    }
    for (start, expected) in case["bezier"].as_array().unwrap().iter().enumerate() {
        let bezier = curve.segment_bezier_curve(start);
        let mut same_input = bezier.clone();
        for i in 0..N {
            matrix(bezier.knot(i), &expected["knots"][i], tol);
            *same_input.knot_mut(i) = vector(&expected["knots"][i]);
        }
        for row in expected["evaluate"].as_array().unwrap() {
            let time = row["time"].as_i64().unwrap();
            let mut j = Jacobian::default();
            let value = match row["derivative"].as_u64().unwrap() {
                0 => bezier.evaluate::<0>(time, Some(&mut j)),
                1 => bezier.velocity(time, Some(&mut j)),
                2 => bezier.acceleration(time, Some(&mut j)),
                _ => unreachable!(),
            };
            let baseline = match row["derivative"].as_u64().unwrap() {
                0 => same_input.evaluate::<0>(time, None),
                1 => same_input.velocity(time, None),
                2 => same_input.acceleration(time, None),
                _ => unreachable!(),
            };
            matrix(&baseline, &row["value"], tol);
            for axis in 0..3 {
                let mut propagated = 0.;
                for i in 0..N {
                    let delta = (bezier.knot(i)[axis] - same_input.knot(i)[axis])
                        .abs()
                        .next_up();
                    propagated =
                        (propagated + (delta * j.d_val_d_knot[i].abs()).next_up()).next_up();
                }
                let expected_value = row["value"][axis].as_f64().unwrap();
                assert!(
                    (value[axis] - expected_value).abs()
                        <= tol * expected_value.abs().max(1.) + propagated,
                    "converted value {} vs {expected_value}, input propagation {propagated}",
                    value[axis]
                );
            }
            matrix(
                &SVector::<f64, N>::from(j.d_val_d_knot),
                &row["jacobian"],
                tol,
            );
        }
        for row in expected["integral"].as_array().unwrap() {
            let mut j = BezierIntegral::default();
            let value = match row["derivative"].as_u64().unwrap() {
                0 => bezier.evaluate_integral_squared::<0>(Some(&mut j)),
                1 => bezier.evaluate_integral_squared::<1>(Some(&mut j)),
                2 => bezier.evaluate_integral_squared::<2>(Some(&mut j)),
                _ => unreachable!(),
            };
            close(value, row["value"].as_f64().unwrap(), tol);
            let derivative = row["derivative"].as_u64().unwrap() as usize;
            let bounds = bezier_jacobian_bounds::<N>(&bezier, expected, derivative);
            for (i, knot) in j.d_int_d_knot.iter().enumerate() {
                for axis in 0..3 {
                    let expected = row["jacobian"][i][axis].as_f64().unwrap();
                    let difference = (knot[axis] - expected).abs();
                    assert!(
                        difference <= bounds[(axis, i)],
                        "Bezier D={derivative} knot={i} axis={axis}: {difference} > {} (Rust {}, C++ {expected})",
                        bounds[(axis, i)],
                        knot[axis]
                    );
                }
            }
        }
    }
}

// Upward-rounded nonnegative product: the error-bound calculation must not
// underestimate its own absolute matrix-product paths.
fn upper_product<const R: usize, const K: usize, const C: usize>(
    a: &SMatrix<f64, R, K>,
    b: &SMatrix<f64, K, C>,
) -> SMatrix<f64, R, C> {
    SMatrix::from_fn(|r, c| {
        (0..K).fold(0., |sum, k| {
            (sum + (a[(r, k)] * b[(k, c)]).next_up()).next_up()
        })
    })
}
fn bezier_jacobian_bounds<const N: usize>(
    curve: &firefly_calibration::spline::rd_bezier::RdBezier<f64, 3, N>,
    expected: &Value,
    derivative: usize,
) -> SMatrix<f64, 3, N> {
    let rust = SMatrix::<f64, 3, N>::from_fn(|r, c| curve.knot(c)[r]);
    let cpp = SMatrix::<f64, 3, N>::from_fn(|r, c| expected["knots"][c][r].as_f64().unwrap());
    let blend = spline_common::compute_blending_matrix_bezier::<f64, N>().map(f64::abs);
    let q = spline_common::compute_quadratic_coefficients::<f64, N>()[derivative].map(f64::abs);
    let paths = |k: SMatrix<f64, 3, N>| {
        upper_product(
            &upper_product(&upper_product(&k, &blend), &q),
            &blend.transpose(),
        )
    };
    // Three length-N products: <= 3*(2N) operations along each path.
    // Q uses two multiplications and one division. Interval scaling uses
    // one division, <= 2D power multiplications, and the final multiplication.
    // One extra operation accounts for a rounded input subtraction in delta.
    // For these N=4/5 fixtures the integer Bezier coefficients are exact.
    let operations = (6 * N + 3 + 2 * derivative + 3) as f64;
    let u = f64::EPSILON / 2.;
    let gamma = (operations * u / (1. - operations * u)).next_up();
    let scaling = 5_f64.powi(2 * derivative as i32 - 1);
    let roundoff = paths(rust.map(f64::abs)) + paths(cpp.map(f64::abs));
    let input_error = paths((rust - cpp).map(|x| x.abs().next_up()));
    SMatrix::from_fn(|r, c| {
        ((roundoff[(r, c)] * gamma).next_up() + input_error[(r, c)])
            .next_up()
            .mul_add(scaling, 0.)
            .next_up()
    })
}
#[test]
fn official_cpp_spline_values_derivatives_and_integrals() {
    let f: Value = serde_json::from_str(include_str!("reference/rd_spline.json")).unwrap();
    reference::<f64, 4>(&f["f64_n4"], 2e-14);
    reference::<f64, 5>(&f["f64_n5"], 2e-14);
    reference::<f32, 4>(&f["f32_n4"], 2e-6);
    reference::<f32, 5>(&f["f32_n5"], 2e-6);
    integral_reference::<4>(&f["f64_n4"]);
    integral_reference::<5>(&f["f64_n5"]);
}
#[test]
fn random_draw_order_and_knot_mutations() {
    let mut curve = RdSpline::<f64, 1, 4>::new(10, 5);
    let mut sample = 0.;
    curve.gen_random_trajectory(6, true, || {
        sample += 0.1;
        sample
    });
    assert_eq!(
        curve.knots().iter().map(|k| k[0]).collect::<Vec<_>>(),
        [0.5, 0.5, 0.5, 0.5, 1., 1.500_000_000_000_000_2]
    );
    curve.resize(4);
    curve.knot_mut(2)[0] = 7.;
    assert_eq!(curve.knot(2)[0], 7.);
    curve.set_start_time_ns(99);
    assert_eq!(curve.min_time_ns(), 99);
    assert_eq!(spline_common::binomial_coefficient(5, 2), 10);
    assert_eq!(spline_common::binomial_coefficient(2, 5), 0);
}
#[test]
fn bezier_integral_jacobians_match_independent_central_differences() {
    use firefly_calibration::spline::rd_bezier::RdBezier;
    let mut curve = RdBezier::<f64, 3, 4>::new(200_000_000, 0);
    for i in 0..4 {
        *curve.knot_mut(i) = SVector::<f64, 3>::new(
            i as f64 * 0.37,
            (i * i) as f64 * 0.23,
            (i + 1) as f64 * -0.19,
        );
    }
    for derivative in 0..3 {
        let evaluate =
            |c: &RdBezier<f64, 3, 4>, j: Option<&mut BezierIntegral<f64, 3, 4>>| match derivative {
                0 => c.evaluate_integral_squared::<0>(j),
                1 => c.evaluate_integral_squared::<1>(j),
                _ => c.evaluate_integral_squared::<2>(j),
            };
        let mut jacobian = BezierIntegral::default();
        let _ = evaluate(&curve, Some(&mut jacobian));
        for knot in 0..4 {
            for axis in 0..3 {
                let mut plus = curve.clone();
                let mut minus = curve.clone();
                let step = 1e-3;
                plus.knot_mut(knot)[axis] += step;
                minus.knot_mut(knot)[axis] -= step;
                let numerical = (evaluate(&plus, None) - evaluate(&minus, None)) / (2. * step);
                close(numerical, jacobian.d_int_d_knot[knot][axis], 1e-8);
            }
        }
    }
}
#[test]
#[should_panic(expected = "assertion failed")]
fn evaluation_rejects_exclusive_end() {
    let mut curve = RdSpline::<f64, 1, 4>::new(10, 0);
    curve.resize(4);
    let _ = curve.evaluate::<0>(10, None);
}
