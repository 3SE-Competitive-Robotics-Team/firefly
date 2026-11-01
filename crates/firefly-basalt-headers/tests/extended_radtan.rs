use firefly_basalt_headers::camera::{
    extended_camera::ExtendedUnifiedCamera, pinhole_radtan8_camera::PinholeRadtan8Camera,
};
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::{Matrix2, SMatrix, SVector, Vector2, Vector4};
use serde_json::Value;

fn vector<S: BasaltScalar, const N: usize>(value: &Value) -> SVector<S, N> {
    SVector::from_fn(|r, _| S::from_f64_lossy(value[r].as_f64().unwrap()))
}
fn compare<S: BasaltScalar, const R: usize, const C: usize>(
    actual: &SMatrix<S, R, C>,
    expected: &Value,
    tolerance: f64,
) {
    for r in 0..R {
        for c in 0..C {
            let a = actual[(r, c)].as_f64();
            let e = &expected[r * C + c];
            match e.as_str() {
                Some("NaN") => assert!(a.is_nan(), "[{r},{c}] {a} must be NaN"),
                Some("+Inf") => assert!(a.is_infinite() && a.is_sign_positive()),
                Some("-Inf") => assert!(a.is_infinite() && a.is_sign_negative()),
                _ => {
                    let e = e.as_f64().unwrap();
                    assert!(
                        (a - e).abs() <= tolerance * e.abs().max(1.),
                        "[{r},{c}] {a} != {e}"
                    );
                }
            }
        }
    }
}
macro_rules! project_cases {
    ($camera:expr,$cases:expr,$s:ty,$n:literal,$tol:expr) => {
        for case in $cases.as_array().unwrap() {
            let point = vector::<$s, 4>(&case["point"]);
            let mut pixel = Vector2::zeros();
            let mut jp = SMatrix::<$s, 2, 4>::zeros();
            let mut ji = SMatrix::<$s, 2, $n>::zeros();
            assert_eq!(
                $camera.project(&point, &mut pixel, Some(&mut jp), Some(&mut ji)),
                case["valid"].as_bool().unwrap()
            );
            compare(&pixel, &case["pixel"], $tol);
            compare(&jp, &case["j_point"], $tol);
            compare(&ji, &case["j_param"], $tol);
            let mut pixel3 = Vector2::zeros();
            $camera.project(
                &point.fixed_rows::<3>(0).into_owned(),
                &mut pixel3,
                None,
                None,
            );
            compare(&pixel3, &case["pixel"], $tol);
        }
    };
}
fn extended<S: BasaltScalar>(fixtures: &Value, tolerance: f64) {
    for f in fixtures.as_array().unwrap() {
        let camera = ExtendedUnifiedCamera::new(vector::<S, 6>(&f["param"]));
        project_cases!(camera, f["project"], S, 6, tolerance);
        for case in f["unproject"].as_array().unwrap() {
            let mut ray = Vector4::zeros();
            let mut jp = SMatrix::<S, 4, 2>::zeros();
            let mut ji = SMatrix::<S, 4, 6>::zeros();
            assert_eq!(
                camera.unproject(
                    &vector::<S, 2>(&case["pixel"]),
                    &mut ray,
                    Some(&mut jp),
                    Some(&mut ji)
                ),
                case["valid"].as_bool().unwrap()
            );
            compare(&ray, &case["ray"], tolerance);
            compare(&jp, &case["j_pixel"], tolerance);
            compare(&ji, &case["j_param"], tolerance);
        }
        let mut initialized = ExtendedUnifiedCamera::<S>::default();
        initialized.set_from_init(Vector4::from([100., 120., 50., 60.].map(S::from_f64_lossy)));
        compare(initialized.param(), &f["init"], tolerance);
        let mut inc = SVector::zeros();
        inc[4] = S::from_f64_lossy(1.5);
        inc[5] = S::from_f64_lossy(-2.);
        initialized.increment(&inc);
        compare(initialized.param(), &f["increment"], tolerance);
    }
}
fn radtan<S: BasaltScalar>(fixtures: &Value, tolerance: f64) {
    for f in fixtures.as_array().unwrap() {
        let mut camera = PinholeRadtan8Camera::with_rpmax(
            vector::<S, 12>(&f["param"]),
            S::from_f64_lossy(f["rpmax"].as_f64().unwrap()),
        );
        let reference_radius = f["computed_rpmax"].as_f64().unwrap();
        assert!(
            (camera.compute_rpmax().as_f64() - reference_radius).abs()
                <= tolerance * reference_radius.abs().max(1.)
        );
        assert_eq!(
            camera.rpmax().as_f64(),
            S::from_f64_lossy(f["rpmax"].as_f64().unwrap()).as_f64()
        );
        project_cases!(camera, f["project"], S, 12, tolerance);
        for case in f["unproject"].as_array().unwrap() {
            let mut ray = Vector4::zeros();
            assert_eq!(
                camera.unproject(&vector::<S, 2>(&case["pixel"]), &mut ray, None, None),
                case["valid"].as_bool().unwrap()
            );
            compare(&ray, &case["ray"], tolerance);
        }
        for case in f["distort"].as_array().unwrap() {
            let mut dist = Vector2::zeros();
            let mut j = Matrix2::zeros();
            camera.distort(&vector::<S, 2>(&case["xy"]), &mut dist, Some(&mut j));
            compare(&dist, &case["dist"], tolerance);
            compare(&j, &case["jacobian"], tolerance);
        }
        let mut initialized = PinholeRadtan8Camera::<S>::default();
        initialized.set_from_init(Vector4::from([100., 120., 50., 60.].map(S::from_f64_lossy)));
        compare(initialized.param(), &f["init"], tolerance);
        assert_eq!(initialized.rpmax(), S::zero());
        let mut inc = SVector::zeros();
        inc[4] = S::from_f64_lossy(1.5);
        inc[5] = S::from_f64_lossy(-2.);
        initialized.increment(&inc);
        compare(initialized.param(), &f["increment"], tolerance);
        let reference = f["increment_rpmax"].as_f64().unwrap();
        assert!(
            (initialized.rpmax().as_f64() - reference).abs() <= tolerance * reference.abs().max(1.)
        );
    }
}
#[test]
fn extended_cpp_all_outputs() {
    extended::<f64>(
        &serde_json::from_str(include_str!("reference/extended_f64.json")).unwrap(),
        2e-14,
    );
    extended::<f32>(
        &serde_json::from_str(include_str!("reference/extended_f32.json")).unwrap(),
        2e-6,
    );
}
#[test]
fn radtan_cpp_all_outputs() {
    radtan::<f64>(
        &serde_json::from_str(include_str!("reference/radtan8_f64.json")).unwrap(),
        2e-14,
    );
    radtan::<f32>(
        &serde_json::from_str(include_str!("reference/radtan8_f32.json")).unwrap(),
        2e-6,
    );
}
#[test]
#[should_panic(expected = "unprojection Jacobians are unsupported")]
fn radtan_rejects_unimplemented_jacobians() {
    let camera = PinholeRadtan8Camera::<f64>::test_projections()
        .into_iter()
        .next()
        .unwrap();
    camera.unproject(
        &Vector2::new(320., 240.),
        &mut Vector4::zeros(),
        Some(&mut SMatrix::<f64, 4, 2>::zeros()),
        None,
    );
}
