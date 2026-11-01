use firefly_basalt_headers::{camera::generic_camera::GenericCamera, scalar::BasaltScalar};
use nalgebra::{DVector, Matrix4, SMatrix, SVector, Vector2, Vector4};
use serde_json::Value;
fn vector<S: BasaltScalar, const N: usize>(v: &Value) -> SVector<S, N> {
    SVector::from_fn(|r, _| S::from_f64_lossy(v[r].as_f64().unwrap()))
}
fn compare<S: BasaltScalar, const R: usize, const C: usize>(
    actual: &SMatrix<S, R, C>,
    expected: &Value,
    tol: f64,
) {
    for r in 0..R {
        for c in 0..C {
            let actual = actual[(r, c)].as_f64();
            let expected = &expected[r * C + c];
            match expected.as_str() {
                Some("NaN") => assert!(actual.is_nan()),
                Some("+Inf") => assert!(actual.is_infinite() && actual.is_sign_positive()),
                Some("-Inf") => assert!(actual.is_infinite() && actual.is_sign_negative()),
                _ => {
                    let expected = expected.as_f64().unwrap();
                    assert!(
                        (actual - expected).abs() <= tol * expected.abs().max(1.),
                        "[{r},{c}]: {actual} != {expected}"
                    );
                }
            }
        }
    }
}
fn rows<S: BasaltScalar, const N: usize>(actual: &[SVector<S, N>], expected: &Value, tol: f64) {
    assert_eq!(actual.len(), expected.as_array().unwrap().len());
    for (i, row) in actual.iter().enumerate() {
        compare(row, &expected[i], tol);
    }
}
fn booleans(actual: &[bool], expected: &Value) {
    assert_eq!(
        actual,
        expected
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_bool().unwrap())
            .collect::<Vec<_>>()
    );
}
fn reference<S: BasaltScalar>(fixtures: &Value, tol: f64) {
    for f in fixtures.as_array().unwrap() {
        let mut camera = GenericCamera::<S>::from_name(f["requested_name"].as_str().unwrap());
        assert_eq!(camera.name(), f["name"].as_str().unwrap());
        assert_eq!(camera.num_parameters(), f["n"].as_u64().unwrap() as usize);
        camera.set_from_init(Vector4::from(
            [200., 220., 100., 90.].map(S::from_f64_lossy),
        ));
        let increment = DVector::from_fn(camera.num_parameters(), |r, _| {
            S::from_f64_lossy(0.001) * S::from_usize(r + 1).unwrap()
        });
        camera.apply_increment(&increment);
        for (i, value) in camera.param().iter().enumerate() {
            assert!(
                (value.as_f64() - f["param"][i].as_f64().unwrap()).abs()
                    <= tol * f["param"][i].as_f64().unwrap().abs().max(1.)
            );
        }
        let single = &f["single_project"];
        let point = vector::<S, 4>(&single["point"]);
        let mut pixel = Vector2::zeros();
        let mut jp = SMatrix::<S, 2, 4>::zeros();
        assert_eq!(
            camera.project(&point, &mut pixel, Some(&mut jp)),
            single["valid"].as_bool().unwrap()
        );
        compare(&pixel, &single["pixel"], tol);
        compare(&jp, &single["jacobian"], tol);
        let unprojection = &f["single_unproject"];
        let mut ray = Vector4::zeros();
        let mut ju = SMatrix::<S, 4, 2>::zeros();
        assert_eq!(
            camera.unproject(
                &pixel,
                &mut ray,
                if unprojection.get("jacobian").is_some() {
                    Some(&mut ju)
                } else {
                    None
                }
            ),
            unprojection["valid"].as_bool().unwrap()
        );
        compare(&ray, &unprojection["ray"], tol);
        if unprojection.get("jacobian").is_some() {
            compare(&ju, &unprojection["jacobian"], tol);
        }
        let points: Vec<Vector4<S>> = f["batch"]["points"]
            .as_array()
            .unwrap()
            .iter()
            .map(vector)
            .collect();
        let mut pixels = Vec::new();
        let mut valid = Vec::new();
        camera.project_points(&points, &mut pixels, &mut valid);
        rows(&pixels, &f["batch"]["pixels"], tol);
        booleans(&valid, &f["batch"]["valid"]);
        let transform = Matrix4::from_fn(|r, c| {
            S::from_f64_lossy(f["transformed"]["transform"][r * 4 + c].as_f64().unwrap())
        });
        let mut angles = Vec::new();
        camera.project_with_angles(&points, &transform, &mut pixels, &mut valid, &mut angles);
        rows(&pixels, &f["transformed"]["pixels"], tol);
        rows(&angles, &f["transformed"]["angles"], tol);
        booleans(&valid, &f["transformed"]["valid"]);
        camera.project_transformed(&points, &transform, &mut pixels, &mut valid);
        rows(&pixels, &f["transformed"]["pixels"], tol);
        booleans(&valid, &f["transformed"]["valid"]);
        let points3: Vec<_> = points
            .iter()
            .map(|p| p.fixed_rows::<3>(0).into_owned())
            .collect();
        camera.project_transformed(&points3, &transform, &mut pixels, &mut valid);
        rows(&pixels, &f["transformed3"]["pixels"], tol);
        booleans(&valid, &f["transformed3"]["valid"]);
        let input_pixels: Vec<Vector2<S>> = f["unproject_batch"]["pixels"]
            .as_array()
            .unwrap()
            .iter()
            .map(vector)
            .collect();
        let mut rays = Vec::new();
        camera.unproject_points(&input_pixels, &mut rays, &mut valid);
        rows(&rays, &f["unproject_batch"]["rays"], tol);
        booleans(&valid, &f["unproject_batch"]["valid"]);
        assert_eq!(camera.cast::<f32>().name(), camera.name());
        camera.project_points(&[], &mut pixels, &mut valid);
        assert_eq!(pixels, Vec::<Vector2<S>>::new());
        assert_eq!(valid, Vec::<bool>::new());
    }
}
#[test]
fn official_cpp_generic_dispatch_and_batches() {
    reference::<f32>(
        &serde_json::from_str(include_str!("reference/generic_camera_f32.json")).unwrap(),
        2e-6,
    );
    reference::<f64>(
        &serde_json::from_str(include_str!("reference/generic_camera_f64.json")).unwrap(),
        2e-14,
    );
}
#[test]
#[should_panic(expected = "assertion `left == right` failed")]
fn rejects_wrong_increment_size() {
    GenericCamera::<f64>::from_name("kb4").apply_increment(&DVector::zeros(4));
}
