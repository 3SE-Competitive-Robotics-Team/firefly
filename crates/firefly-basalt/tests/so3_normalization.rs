use firefly_basalt::imu::imu_types::PoseState;
use firefly_basalt::utils::sophus_utils::so3_cast;
use firefly_basalt::utils::sophus_utils::so3_inverse;
use firefly_basalt::utils::sophus_utils::so3_product;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{Isometry3, Quaternion, SVector, Translation3, UnitQuaternion, Vector4};
use serde_json::Value;
fn quaternion<S: BasaltScalar>(value: &Value) -> UnitQuaternion<S> {
    UnitQuaternion::new_unchecked(Quaternion::from_vector(Vector4::from_fn(|r, _| {
        S::from_f64_lossy(value[r].as_f64().unwrap())
    })))
}
fn compare<S: BasaltScalar>(actual: &UnitQuaternion<S>, value: &Value, tolerance: f64) {
    for i in 0..4 {
        let actual = actual.quaternion().coords[i].as_f64();
        let expected = value[i].as_f64().unwrap();
        assert!(
            (actual - expected).abs() <= tolerance,
            "coefficient {i}: {actual} != {expected}"
        );
    }
}
fn reference<S: BasaltScalar>(fixture: &Value, tolerance: f64) {
    let initial = quaternion::<S>(&fixture["initial"]);
    let step = quaternion::<S>(&fixture["step"]);
    let mut rotation = initial;
    let mut state = PoseState::new(42, Isometry3::from_parts(Translation3::identity(), initial));
    let increment = SVector::<S, 6>::from_fn(|r, _| {
        S::from_f64_lossy(fixture["increment"][r].as_f64().unwrap())
    });
    let mut completed = 0;
    for sample in fixture["samples"].as_array().unwrap() {
        let steps = sample["steps"].as_u64().unwrap();
        while completed < steps {
            rotation = so3_product(&rotation, &step);
            state.apply_inc(&increment);
            completed += 1;
        }
        compare(&rotation, &sample["product"], tolerance);
        compare(&state.t_w_i.rotation, &sample["state"], tolerance);
        compare(&so3_inverse(&rotation), &sample["inverse"], tolerance);
        assert!((rotation.quaternion().norm_squared().as_f64() - 1.).abs() < tolerance);
        assert_eq!(state.t_ns, 42);
    }
    compare(&so3_cast::<S, f32>(&initial), &fixture["cast_f32"], 3e-6);
    compare(&so3_cast::<S, f64>(&initial), &fixture["cast_f64"], 3e-14);
    assert_eq!(
        so3_cast::<S, S>(&initial).quaternion().coords,
        initial.quaternion().coords
    );
}
#[test]
fn official_cpp_long_rotation_products_and_casts() {
    let fixture: Value =
        serde_json::from_str(include_str!("reference/primitives/so3_normalization.json")).unwrap();
    reference::<f32>(&fixture["f32"], 3e-6);
    reference::<f64>(&fixture["f64"], 3e-14);
}
