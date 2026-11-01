//! Regression fixtures for optical-flow arithmetic and tracker behavior.
use super::{
    patch::{OpticalFlowPatch, PatchScalar},
    patterns::*,
};
use firefly_calibration::image::image::Image;
use firefly_calibration::image::image::ImageScalar;
use nalgebra::{Isometry2, SMatrix, SVector, Vector2};
use serde_json::Value;

fn scalar(value: &Value) -> f64 {
    value
        .as_f64()
        .unwrap_or_else(|| match value.as_str().unwrap() {
            "NaN" => f64::NAN,
            "+Inf" => f64::INFINITY,
            "-Inf" => f64::NEG_INFINITY,
            other => panic!("unexpected scalar {other}"),
        })
}

fn compare<S: ImageScalar>(actual: S, expected: &Value, tolerance: f64, label: &str) {
    let a = actual.as_f64();
    let b = scalar(expected);
    if b.is_nan() {
        assert!(a.is_nan(), "{label}: {a} != NaN");
    } else if b.is_infinite() {
        assert_eq!(a, b, "{label}");
    } else {
        assert!(
            (a - b).abs() <= tolerance * (1.0 + b.abs()),
            "{label}: {a} != {b}"
        );
    }
}

fn matrix<S: ImageScalar, const R: usize, const C: usize>(
    actual: &SMatrix<S, R, C>,
    expected: &Value,
    tolerance: f64,
    label: &str,
) {
    assert_eq!(expected["rows"], R);
    assert_eq!(expected["cols"], C);
    for r in 0..R {
        for c in 0..C {
            compare(
                actual[(r, c)],
                &expected["row_major"][r * C + c],
                tolerance,
                &format!("{label}[{r},{c}]"),
            );
        }
    }
}

fn case<S: PatchScalar, P: Pattern<S, N>, const N: usize>(expected: &Value, tolerance: f64) {
    let scenario = expected["scenario"].as_str().unwrap();
    let mut pixels = vec![65535_u16; 52 * 40];
    for y in 0..40 {
        for x in 0..48 {
            pixels[y * 52 + x] = u16::try_from(match scenario {
                "black" => 0,
                "constant" => 400,
                "plane" => 900 + 17 * x + 23 * y,
                _ => 1000 + (17 * x * x + 31 * y * y + 7 * x * y) % 4000,
            })
            .unwrap();
        }
    }
    let image = Image::new(&pixels, 48, 40, 52);
    let pos = Vector2::from_fn(|r, _| {
        S::from_f64(scalar(&expected["position"]["row_major"][r])).unwrap()
    });
    let mut mean = S::zero();
    let mut data = SVector::zeros();
    OpticalFlowPatch::<S, P, N>::set_data(&image, &pos, &mut mean, &mut data, None);
    compare(mean, &expected["set_data_mean"], tolerance, "set_data_mean");
    matrix(&data, &expected["set_data"], tolerance, "set_data");
    let warp = Isometry2::new(
        Vector2::new(S::from_f64(0.1).unwrap(), S::from_f64(-0.2).unwrap()),
        S::from_f64(0.03).unwrap(),
    );
    OpticalFlowPatch::<S, P, N>::set_data(&image, &pos, &mut mean, &mut data, Some(&warp));
    compare(mean, &expected["warped_mean"], tolerance, "warped_mean");
    matrix(&data, &expected["warped_data"], tolerance, "warped_data");
    let mut jacobian = SMatrix::zeros();
    OpticalFlowPatch::<S, P, N>::set_data_jac_se2(
        &image,
        &pos,
        &mut mean,
        &mut data,
        &mut jacobian,
    );
    compare(mean, &expected["set_data_jac_mean"], tolerance, "jac_mean");
    matrix(&data, &expected["set_data_jac_data"], tolerance, "jac_data");
    matrix(&jacobian, &expected["jacobian"], tolerance, "jacobian");
    let hessian = jacobian.transpose() * jacobian;
    matrix(&hessian, &expected["hessian"], tolerance, "hessian");
    matrix(
        &super::patch::ldlt_inverse(hessian),
        &expected["hessian_inverse"],
        tolerance,
        "hessian_inverse",
    );
    let patch = OpticalFlowPatch::<S, P, N>::new(&image, &pos);
    matrix(&P::pattern2(), &expected["pattern2"], 0.0, "pattern");
    compare(patch.mean, &expected["mean"], tolerance, "mean");
    matrix(&patch.data, &expected["data"], tolerance, "data");
    matrix(
        &patch.h_se2_inv_j_se2_t,
        &expected["h_inv_jt"],
        tolerance,
        "h_inv_jt",
    );
    assert_eq!(patch.valid, expected["valid"].as_bool().unwrap());
    let transformed = SMatrix::from_fn(|r, c| {
        S::from_f64(scalar(
            &expected["transformed_pattern"]["row_major"][r * N + c],
        ))
        .unwrap()
    });
    let mut residual = SVector::zeros();
    assert_eq!(
        patch.residual(&image, &transformed, &mut residual),
        expected["residual_valid"].as_bool().unwrap()
    );
    matrix(&residual, &expected["residual"], tolerance, "residual");
}

fn suite<S: PatchScalar>(scalar_name: &str, tolerance: f64) {
    for scenario in ["normal", "edge", "outside", "black", "constant", "plane"] {
        let path = format!(
            "{}/tests/reference/optical_flow_{scalar_name}_{scenario}.json",
            env!("CARGO_MANIFEST_DIR")
        );
        let values: Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        for expected in values.as_array().unwrap() {
            eprintln!("{scalar_name} {scenario} pattern {}", expected["pattern"]);
            match expected["pattern"].as_str().unwrap() {
                "24" => case::<S, Pattern24, 24>(expected, tolerance),
                "52" => case::<S, Pattern52, 52>(expected, tolerance),
                "51" => case::<S, Pattern51, 52>(expected, tolerance),
                "50" => case::<S, Pattern50, 52>(expected, tolerance),
                _ => unreachable!(),
            }
        }
    }
}

#[test]
fn cpp_reference_f64() {
    suite::<f64>("f64", 1e-10);
}
#[test]
fn cpp_reference_f32() {
    suite::<f32>("f32", 1e-5);
}
