use firefly_basalt::vi_estimator::ba_base::BundleAdjustmentBase;
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::{Isometry3, Quaternion, SVector, Translation3, UnitQuaternion};
use serde_json::Value;

fn vector<S: BasaltScalar, const N: usize>(value: &Value) -> SVector<S, N> {
    SVector::from_fn(|i, _| S::from_f64_lossy(value[i].as_f64().unwrap()))
}
fn compare<S: BasaltScalar>(name: &str, tolerance: f64) {
    let reference: Value =
        serde_json::from_str(include_str!("reference/triangulation.json")).unwrap();
    for (case, input) in reference[name].as_array().unwrap().iter().enumerate() {
        let q = vector::<S, 4>(&input["quaternion"]);
        let transform = Isometry3::from_parts(
            Translation3::from(vector::<S, 3>(&input["translation"])),
            UnitQuaternion::new_unchecked(Quaternion::new(q[3], q[0], q[1], q[2])),
        );
        let result = BundleAdjustmentBase::<S>::triangulate(
            &vector(&input["first"]),
            &vector(&input["second"]),
            &transform,
        );
        for i in 0..4 {
            let actual = result[i].as_f64();
            let expected = &input["result"][i];
            match expected.as_str() {
                Some("NaN") => assert!(actual.is_nan(), "{name} case{case} [{i}]"),
                Some("+Inf") => assert_eq!(actual, f64::INFINITY),
                Some("-Inf") => assert_eq!(actual, f64::NEG_INFINITY),
                _ => {
                    let expected = expected.as_f64().unwrap();
                    assert!(
                        (actual - expected).abs() <= tolerance * (1.0 + expected.abs()),
                        "{name} case{case} [{i}]:{actual} vs {expected}"
                    );
                }
            }
        }
    }
}
#[test]
fn original_jacobi_triangulation_float() {
    compare::<f32>("f32", 1e-5);
}
#[test]
fn original_jacobi_triangulation_double() {
    compare::<f64>("f64", 1e-10);
}
