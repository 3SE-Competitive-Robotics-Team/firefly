use firefly_basalt_headers::camera::pinhole_camera::PinholeCamera;
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::{SMatrix, Vector2, Vector4};
use serde_json::Value;

fn vector<S: BasaltScalar>(value: &Value) -> Vector4<S> {
    Vector4::from_fn(|row, _| S::from_f64_lossy(value[row].as_f64().unwrap()))
}

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
                "[{row},{col}]: {actual} != {expected}"
            );
        }
    }
}

fn reference<S: BasaltScalar>(fixtures: &Value, tolerance: f64) {
    for fixture in fixtures.as_array().unwrap() {
        let camera = PinholeCamera::new(vector::<S>(&fixture["param"]));
        let point = vector::<S>(&fixture["point"]);
        let mut pixel = Vector2::zeros();
        let mut point_jacobian = SMatrix::<S, 2, 4>::zeros();
        let mut param_jacobian = SMatrix::<S, 2, 4>::zeros();
        assert_eq!(
            camera.project(
                &point,
                &mut pixel,
                Some(&mut point_jacobian),
                Some(&mut param_jacobian)
            ),
            fixture["valid"].as_bool().unwrap()
        );
        compare(&pixel, &fixture["pixel"], tolerance);
        compare(&point_jacobian, &fixture["j_point"], tolerance);
        compare(&param_jacobian, &fixture["j_param"], tolerance);
        let mut ray = Vector4::zeros();
        let mut pixel_jacobian = SMatrix::<S, 4, 2>::zeros();
        let mut intrinsics_jacobian = SMatrix::<S, 4, 4>::zeros();
        assert!(camera.unproject(
            &pixel,
            &mut ray,
            Some(&mut pixel_jacobian),
            Some(&mut intrinsics_jacobian)
        ));
        compare(&ray, &fixture["ray"], tolerance);
        compare(&pixel_jacobian, &fixture["j_pixel"], tolerance);
        compare(&intrinsics_jacobian, &fixture["j_intrinsics"], tolerance);
    }
}

#[test]
fn official_cpp_projection_and_all_jacobians() {
    let fixtures: Value =
        serde_json::from_str(include_str!("reference/pinhole_camera.json")).unwrap();
    reference::<f32>(&fixtures["f32"], 2e-6);
    reference::<f64>(&fixtures["f64"], 2e-14);
}

#[test]
fn projection_derivatives_match_central_differences() {
    let camera = PinholeCamera::<f64>::test_projections()[0].clone();
    let point = Vector4::new(0.4, -0.2, 1.5, 0.7);
    let mut pixel = Vector2::zeros();
    let mut jacobian = SMatrix::<f64, 2, 4>::zeros();
    assert!(camera.project(&point, &mut pixel, Some(&mut jacobian), None));
    for axis in 0..4 {
        let mut plus = point;
        let mut minus = point;
        plus[axis] += 1e-6;
        minus[axis] -= 1e-6;
        let mut pixel_plus = Vector2::zeros();
        let mut pixel_minus = Vector2::zeros();
        camera.project(&plus, &mut pixel_plus, None, None);
        camera.project(&minus, &mut pixel_minus, None, None);
        assert!(((pixel_plus - pixel_minus) / 2e-6 - jacobian.column(axis)).amax() < 5e-8);
    }
}

#[test]
fn depth_boundary_and_zero_intrinsics_preserve_source_behavior() {
    let camera = PinholeCamera::<f64>::test_projections()[0].clone();
    let threshold = <f64 as BasaltScalar>::epsilon_sqrt();
    let mut pixel = Vector2::zeros();
    for (depth, expected) in [
        (threshold, true),
        (threshold.next_down(), false),
        (0.0, false),
        (-1.0, false),
        (f64::NAN, false),
    ] {
        assert_eq!(
            camera.project(&Vector4::new(1.0, 2.0, depth, 0.0), &mut pixel, None, None),
            expected
        );
    }
    let zero = PinholeCamera::<f64>::default();
    let mut ray = Vector4::zeros();
    assert!(zero.unproject(&Vector2::zeros(), &mut ray, None, None));
    assert!(ray[0].is_nan());
    assert_eq!(ray[3], 0.0);
}
