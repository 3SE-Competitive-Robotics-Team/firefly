use firefly_calibration::camera::bal_camera::BalCamera;
use firefly_calibration::camera::double_sphere_camera::DoubleSphereCamera;
use firefly_calibration::camera::fov_camera::FovCamera;
use firefly_calibration::camera::kannala_brandt_camera4::KannalaBrandtCamera4;
use firefly_calibration::camera::unified_camera::UnifiedCamera;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{SMatrix, SVector, Vector2, Vector4};
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
            let actual = actual[(r, c)].as_f64();
            let expected = &expected[r * C + c];
            match expected.as_str() {
                Some("NaN") => assert!(actual.is_nan(), "[{r},{c}]: {actual} must be NaN"),
                Some("+Inf") => assert!(actual.is_infinite() && actual.is_sign_positive()),
                Some("-Inf") => assert!(actual.is_infinite() && actual.is_sign_negative()),
                _ => {
                    let expected = expected.as_f64().unwrap();
                    assert!(
                        (actual - expected).abs() <= tolerance * expected.abs().max(1.),
                        "[{r},{c}]: {actual} != {expected}"
                    );
                }
            }
        }
    }
}
macro_rules! reference {
    ($function:ident,$camera:ident,$n:literal) => {
        fn $function<S: BasaltScalar>(fixtures: &Value, tolerance: f64) {
            for f in fixtures.as_array().unwrap() {
                let camera = $camera::new(vector::<S, $n>(&f["param"]));
                for case in f["project"].as_array().unwrap() {
                    let point = vector::<S, 4>(&case["point"]);
                    let mut pixel = Vector2::zeros();
                    let mut jp = SMatrix::<S, 2, 4>::zeros();
                    let mut ji = SMatrix::<S, 2, $n>::zeros();
                    assert_eq!(
                        camera.project(&point, &mut pixel, Some(&mut jp), Some(&mut ji)),
                        case["valid"].as_bool().unwrap()
                    );
                    compare(&pixel, &case["pixel"], tolerance);
                    compare(&jp, &case["j_point"], tolerance);
                    compare(&ji, &case["j_param"], tolerance);
                    let mut pixel3 = Vector2::zeros();
                    let point3 = point.fixed_rows::<3>(0).into_owned();
                    camera.project(&point3, &mut pixel3, None, None);
                    compare(&pixel3, &case["pixel"], tolerance);
                }
                for case in f["unproject"].as_array().unwrap() {
                    let pixel = vector::<S, 2>(&case["pixel"]);
                    let mut ray = Vector4::zeros();
                    let mut jp = SMatrix::<S, 4, 2>::zeros();
                    let mut ji = SMatrix::<S, 4, $n>::zeros();
                    assert_eq!(
                        if $n == 3 {
                            camera.unproject(&pixel, &mut ray, None, None)
                        } else {
                            camera.unproject(&pixel, &mut ray, Some(&mut jp), Some(&mut ji))
                        },
                        case["valid"].as_bool().unwrap()
                    );
                    compare(&ray, &case["ray"], tolerance);
                    if $n != 3 {
                        compare(&jp, &case["j_pixel"], tolerance);
                        compare(&ji, &case["j_param"], tolerance);
                    }
                }
                let mut initialized = $camera::<S>::default();
                initialized.set_from_init(Vector4::from([1., 2., 3., 4.].map(S::from_f64_lossy)));
                compare(initialized.param(), &f["init"], tolerance);
                initialized.increment(&SVector::repeat(S::from_f64_lossy(2.)));
                compare(initialized.param(), &f["increment_high"], tolerance);
                initialized.increment(&SVector::repeat(S::from_f64_lossy(-4.)));
                compare(initialized.param(), &f["increment_low"], tolerance);
                compare(
                    initialized.cast::<f64>().param(),
                    &f["increment_low"],
                    tolerance,
                );
            }
        }
    };
}
reference!(double_sphere_reference, DoubleSphereCamera, 6);
reference!(unified_reference, UnifiedCamera, 5);
reference!(kb4_reference, KannalaBrandtCamera4, 8);
reference!(fov_reference, FovCamera, 5);
reference!(bal_reference, BalCamera, 3);

#[test]
fn official_cpp_fov_all_outputs() {
    fov_reference::<f32>(
        &serde_json::from_str(include_str!("reference/fov_f32.json")).unwrap(),
        2e-6,
    );
    fov_reference::<f64>(
        &serde_json::from_str(include_str!("reference/fov_f64.json")).unwrap(),
        2e-14,
    );
}
#[test]
fn official_cpp_bal_all_outputs() {
    bal_reference::<f32>(
        &serde_json::from_str(include_str!("reference/bal_f32.json")).unwrap(),
        2e-6,
    );
    bal_reference::<f64>(
        &serde_json::from_str(include_str!("reference/bal_f64.json")).unwrap(),
        2e-14,
    );
}

#[test]
fn official_cpp_double_sphere_all_outputs() {
    double_sphere_reference::<f32>(
        &serde_json::from_str(include_str!("reference/double_sphere_f32.json")).unwrap(),
        2e-6,
    );
    double_sphere_reference::<f64>(
        &serde_json::from_str(include_str!("reference/double_sphere_f64.json")).unwrap(),
        2e-14,
    );
}
#[test]
fn official_cpp_unified_all_outputs() {
    unified_reference::<f32>(
        &serde_json::from_str(include_str!("reference/unified_f32.json")).unwrap(),
        2e-6,
    );
    unified_reference::<f64>(
        &serde_json::from_str(include_str!("reference/unified_f64.json")).unwrap(),
        2e-14,
    );
}

#[test]
fn official_cpp_kb4_all_outputs() {
    kb4_reference::<f32>(
        &serde_json::from_str(include_str!("reference/kb4_f32.json")).unwrap(),
        2e-6,
    );
    kb4_reference::<f64>(
        &serde_json::from_str(include_str!("reference/kb4_f64.json")).unwrap(),
        2e-14,
    );
}

macro_rules! derivatives {
    ($test:ident,$camera:ident,$n:literal) => {
        #[test]
        fn $test() {
            let camera = $camera::<f64>::test_projections()[0].clone();
            let point = Vector4::new(0.4, -0.2, 1.5, 0.7);
            let pixel = if $n == 8 {
                Vector2::new(camera.param()[2] + 20., camera.param()[3] - 15.)
            } else {
                Vector2::new(180., 140.)
            };
            let mut proj = Vector2::zeros();
            let mut jp = SMatrix::<f64, 2, 4>::zeros();
            let mut ji = SMatrix::<f64, 2, $n>::zeros();
            camera.project(&point, &mut proj, Some(&mut jp), Some(&mut ji));
            let mut ray = Vector4::zeros();
            let mut ju = SMatrix::<f64, 4, 2>::zeros();
            let mut jui = SMatrix::<f64, 4, $n>::zeros();
            camera.unproject(&pixel, &mut ray, Some(&mut ju), Some(&mut jui));
            let step = 1e-6;
            for axis in 0..4 {
                let mut plus = point;
                let mut minus = point;
                plus[axis] += step;
                minus[axis] -= step;
                let mut a = Vector2::zeros();
                let mut b = Vector2::zeros();
                camera.project(&plus, &mut a, None, None);
                camera.project(&minus, &mut b, None, None);
                assert!(((a - b) / (2. * step) - jp.column(axis)).amax() < 1e-7);
            }
            for axis in 0..2 {
                let mut plus = pixel;
                let mut minus = pixel;
                plus[axis] += step;
                minus[axis] -= step;
                let mut a = Vector4::zeros();
                let mut b = Vector4::zeros();
                camera.unproject(&plus, &mut a, None, None);
                camera.unproject(&minus, &mut b, None, None);
                assert!(((a - b) / (2. * step) - ju.column(axis)).amax() < 1e-8);
            }
            for axis in 0..$n {
                let mut plus = *camera.param();
                let mut minus = *camera.param();
                plus[axis] += step;
                minus[axis] -= step;
                let cp = $camera::new(plus);
                let cm = $camera::new(minus);
                let mut a = Vector2::zeros();
                let mut b = Vector2::zeros();
                cp.project(&point, &mut a, None, None);
                cm.project(&point, &mut b, None, None);
                assert!(((a - b) / (2. * step) - ji.column(axis)).amax() < 1e-7);
                let mut a = Vector4::zeros();
                let mut b = Vector4::zeros();
                cp.unproject(&pixel, &mut a, None, None);
                cm.unproject(&pixel, &mut b, None, None);
                assert!(((a - b) / (2. * step) - jui.column(axis)).amax() < 1e-8);
            }
        }
    };
}
derivatives!(double_sphere_derivatives, DoubleSphereCamera, 6);
derivatives!(unified_derivatives, UnifiedCamera, 5);
derivatives!(kb4_derivatives, KannalaBrandtCamera4, 8);
derivatives!(fov_derivatives, FovCamera, 5);

#[test]
#[should_panic(expected = "Jacobians for unprojection are not implemented")]
fn bal_rejects_unsupported_unprojection_derivatives() {
    let camera = BalCamera::<f64>::test_projections()[0].clone();
    camera.unproject(
        &Vector2::zeros(),
        &mut Vector4::zeros(),
        Some(&mut SMatrix::<f64, 4, 2>::zeros()),
        None,
    );
}

fn newton_reference<S: BasaltScalar>(fixtures: &Value, tolerance: f64) {
    for fixture in fixtures.as_array().unwrap() {
        let camera = KannalaBrandtCamera4::new(vector::<S, 8>(&fixture["param"]));
        for case in fixture["solve_theta"].as_array().unwrap() {
            let radius = S::from_f64_lossy(case["radius"].as_f64().unwrap());
            let mut derivative = S::from_f64_lossy(42.);
            let theta = match case["iterations"].as_u64().unwrap() {
                0 => camera.solve_theta::<0>(radius, &mut derivative),
                1 => camera.solve_theta::<1>(radius, &mut derivative),
                3 => camera.solve_theta::<3>(radius, &mut derivative),
                _ => unreachable!(),
            };
            compare(
                &SVector::<S, 1>::repeat(theta),
                &serde_json::json!([case["theta"]]),
                tolerance,
            );
            compare(
                &SVector::<S, 1>::repeat(derivative),
                &serde_json::json!([case["derivative"]]),
                tolerance,
            );
        }
    }
}

#[test]
fn official_cpp_kb4_newton_iteration_contract() {
    newton_reference::<f32>(
        &serde_json::from_str(include_str!("reference/kb4_f32.json")).unwrap(),
        2e-6,
    );
    newton_reference::<f64>(
        &serde_json::from_str(include_str!("reference/kb4_f64.json")).unwrap(),
        2e-14,
    );
}

#[test]
fn strict_radial_boundary_and_upstream_validity_semantics() {
    let ds = DoubleSphereCamera::<f64>::new(SVector::from([1., 1., 0., 0., 0.2, 1.]));
    let mut ray = Vector4::zeros();
    assert!(!ds.unproject(&Vector2::new(1., 0.), &mut ray, None, None));
    assert!(ds.unproject(&Vector2::new(1_f64.next_down(), 0.), &mut ray, None, None));
    // The upstream UCM test uses scaled r²; validity does not imply a finite ray.
    let ucm = UnifiedCamera::<f64>::new(SVector::from([1., 1., 0., 0., 0.75]));
    assert!(ucm.unproject(&Vector2::new(2., 0.), &mut ray, None, None));
    assert!(ray[0].is_nan());
    let mut ds = DoubleSphereCamera::<f64>::default();
    ds.increment(&SVector::from([0., 0., 0., 0., f64::NAN, f64::NAN]));
    assert!(ds.param()[4].is_nan() && ds.param()[5].is_nan());
    let mut ucm = UnifiedCamera::<f64>::default();
    ucm.increment(&SVector::from([0., 0., 0., 0., f64::NAN]));
    assert!(ucm.param()[4].is_nan());
}
