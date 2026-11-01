use firefly_basalt::{
    utils::{
        ba_utils::{compute_rel_pose, linearize_point},
        common_types::TimeCamId,
    },
    vi_estimator::landmark_database::Keypoint,
};
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{
    Isometry3, Quaternion, SMatrix, SVector, Translation3, UnitQuaternion, Vector2, Vector4,
};
use serde_json::Value;
fn scalar<S: BasaltScalar>(v: f64) -> S {
    S::from_f64_lossy(v)
}
fn fixed<S: BasaltScalar, const R: usize, const C: usize>(v: &Value) -> SMatrix<S, R, C> {
    SMatrix::from_row_iterator(
        v["values"]
            .as_array()
            .unwrap()
            .iter()
            .map(|x| scalar(x.as_f64().unwrap())),
    )
}
fn check<S: BasaltScalar, const R: usize, const C: usize>(
    actual: &SMatrix<S, R, C>,
    expected: &Value,
    tolerance: f64,
    label: &str,
) {
    let expected = fixed::<S, R, C>(expected);
    for r in 0..R {
        for c in 0..C {
            let a = actual[(r, c)].as_f64();
            let b = expected[(r, c)].as_f64();
            assert!(
                (a - b).abs() <= tolerance * (1. + b.abs()),
                "{label} {r},{c}: {a} != {b}"
            );
        }
    }
}
fn run<S: BasaltScalar>(text: &str, tolerance: f64) {
    let data: Value = serde_json::from_str(text).unwrap();
    let poses: Vec<_> = data["poses"]
        .as_array()
        .unwrap()
        .iter()
        .map(|p| {
            let q = fixed::<S, 4, 1>(&p["quaternion"]);
            Isometry3::from_parts(
                Translation3::from(fixed::<S, 3, 1>(&p["translation"])),
                UnitQuaternion::new_unchecked(Quaternion::new(q[3], q[0], q[1], q[2])),
            )
        })
        .collect();
    let mut host = SMatrix::zeros();
    let mut target = SMatrix::zeros();
    let relative = compute_rel_pose(
        &poses[0],
        &poses[1],
        &poses[2],
        &poses[3],
        Some(&mut host),
        Some(&mut target),
    );
    check(
        &relative.to_homogeneous(),
        &data["transform"],
        tolerance,
        "relative",
    );
    check(&host, &data["host_jacobian"], tolerance, "host jacobian");
    check(
        &target,
        &data["target_jacobian"],
        tolerance,
        "target jacobian",
    );
    for case in data["points"].as_array().unwrap() {
        let mut camera = GenericCamera::<S>::from_name(case["camera"].as_str().unwrap());
        camera.set_from_init(SVector::<S, 4>::from_row_slice(
            &[320., 315., 320., 240.].map(scalar),
        ));
        let point = Keypoint::new(
            fixed(&case["direction"]),
            scalar(case["inverse_distance"].as_f64().unwrap()),
            TimeCamId::default(),
        );
        let mut residual = Vector2::repeat(scalar(6.));
        let mut jpose = SMatrix::repeat(scalar(7.));
        let mut jpoint = SMatrix::repeat(scalar(8.));
        let mut projection = Vector4::repeat(scalar(9.));
        let valid = linearize_point(
            &fixed(&case["observation"]),
            &point,
            &fixed(&data["transform"]),
            &camera,
            &mut residual,
            Some(&mut jpose),
            Some(&mut jpoint),
            Some(&mut projection),
        );
        assert_eq!(valid, case["valid"].as_bool().unwrap());
        check(&residual, &case["residual"], tolerance, "residual");
        check(&jpose, &case["pose_jacobian"], tolerance, "pose derivative");
        check(
            &jpoint,
            &case["point_jacobian"],
            tolerance,
            "point derivative",
        );
        check(&projection, &case["projection"], tolerance, "projection");
    }
}
#[test]
fn original_ba_utils_f32() {
    run::<f32>(include_str!("reference/ba_utils_f32.json"), 1e-5);
}
#[test]
fn original_ba_utils_f64() {
    run::<f64>(include_str!("reference/ba_utils_f64.json"), 1e-10);
}
