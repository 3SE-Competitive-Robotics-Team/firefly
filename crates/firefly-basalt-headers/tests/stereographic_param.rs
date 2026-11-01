use firefly_basalt_headers::camera::stereographic_param::StereographicParam;
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::{SMatrix, Vector2, Vector4};
use serde_json::Value;

fn compare<S: BasaltScalar, const R: usize, const C: usize>(
    actual: &SMatrix<S, R, C>,
    expected: &Value,
    tolerance: f64,
) {
    for row in 0..R {
        for column in 0..C {
            let expected = expected[row * C + column].as_f64().unwrap();
            let actual = actual[(row, column)].as_f64();
            assert!(
                (actual - expected).abs() <= tolerance * expected.abs().max(1.0),
                "{actual} != {expected}"
            );
        }
    }
}

fn reference<S: BasaltScalar>(fixtures: &Value, tolerance: f64) {
    for fixture in fixtures.as_array().unwrap() {
        let point = Vector4::<S>::from_fn(|row, _| {
            S::from_f64_lossy(fixture["point"][row].as_f64().unwrap())
        });
        let mut project_jacobian = SMatrix::<S, 2, 4>::zeros();
        let projection = StereographicParam::project(&point, Some(&mut project_jacobian));
        let mut unproject_jacobian = SMatrix::<S, 4, 2>::zeros();
        let ray = StereographicParam::unproject(&projection, Some(&mut unproject_jacobian));
        compare(&projection, &fixture["projection"], tolerance);
        compare(&project_jacobian, &fixture["j_project"], tolerance);
        compare(&ray, &fixture["ray"], tolerance);
        compare(&unproject_jacobian, &fixture["j_unproject"], tolerance);
    }
}

#[test]
fn official_cpp_projection_and_jacobians() {
    let fixtures: Value =
        serde_json::from_str(include_str!("reference/stereographic_param.json")).unwrap();
    reference::<f32>(&fixtures["f32"], 2e-6);
    reference::<f64>(&fixtures["f64"], 2e-14);
}

#[test]
fn inverse_and_jacobians_preserve_direction_and_scale_nullspace() {
    for point in [
        Vector4::new(1.0, 2.0, 3.0, 7.0),
        Vector4::new(-2.0, 0.3, -1.0, 0.0),
        Vector4::new(0.0, 0.0, 1.0, 1.0),
    ] {
        let mut project_jacobian = SMatrix::<f64, 2, 4>::zeros();
        let projection = StereographicParam::project(&point, Some(&mut project_jacobian));
        let mut unproject_jacobian = SMatrix::<f64, 4, 2>::zeros();
        let ray = StereographicParam::unproject(&projection, Some(&mut unproject_jacobian));
        assert!((ray.fixed_rows::<3>(0) - point.fixed_rows::<3>(0).normalize()).amax() < 1e-14);
        assert_eq!(ray[3], 0.0);
        assert!((project_jacobian * point).amax() < 1e-14);
        for axis in 0..4 {
            let mut plus = point;
            let mut minus = point;
            plus[axis] += 1e-6;
            minus[axis] -= 1e-6;
            let numerical = (StereographicParam::project(&plus, None)
                - StereographicParam::project(&minus, None))
                / 2e-6;
            assert!((numerical - project_jacobian.column(axis)).amax() < 1e-8);
        }
        for axis in 0..2 {
            let mut plus = projection;
            let mut minus = projection;
            plus[axis] += 1e-6;
            minus[axis] -= 1e-6;
            let numerical = (StereographicParam::unproject(&plus, None)
                - StereographicParam::unproject(&minus, None))
                / 2e-6;
            assert!((numerical - unproject_jacobian.column(axis)).amax() < 1e-8);
        }
    }
}

#[test]
fn projection_pole_is_not_silently_regularized() {
    let projection = StereographicParam::<f64>::project(&Vector4::new(0.0, 0.0, -1.0, 1.0), None);
    assert!(projection.x.is_nan() && projection.y.is_nan());
    assert_eq!(
        StereographicParam::<f32>::unproject(&Vector2::zeros(), None),
        Vector4::new(0.0, 0.0, 1.0, 0.0)
    );
}
