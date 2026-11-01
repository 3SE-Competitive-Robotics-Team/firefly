use firefly_basalt::utils::sophus_utils::right_jacobian_inv_se3_decoupled;
use firefly_basalt::utils::sophus_utils::right_jacobian_se3_decoupled;
use firefly_basalt::utils::sophus_utils::se3_matrix;
use firefly_basalt::utils::sophus_utils::so3_matrix;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{Isometry3, Quaternion, SVector, Translation3, UnitQuaternion};
use serde_json::Value;

fn run<S: BasaltScalar>(key: &str) {
    let fixture: Value =
        serde_json::from_str(include_str!("reference/primitives/rotation_matrix.json")).unwrap();
    for case in fixture[key].as_array().unwrap() {
        let q = &case["quaternion"];
        let scalar = |i: usize| S::from_f64_lossy(q[i].as_f64().unwrap());
        let rotation = UnitQuaternion::new_unchecked(Quaternion::new(
            scalar(3),
            scalar(0),
            scalar(1),
            scalar(2),
        ));
        let pose = Isometry3::from_parts(
            Translation3::new(
                S::from_f64_lossy(0.125),
                S::from_f64_lossy(-0.25),
                S::from_f64_lossy(0.375),
            ),
            rotation,
        );
        let matrix = se3_matrix(&pose);
        let rotation = so3_matrix(&rotation);
        for r in 0..4 {
            for c in 0..4 {
                assert_eq!(
                    matrix[(r, c)].as_f64(),
                    case["matrix"][r * 4 + c].as_f64().unwrap(),
                    "{key} [{r},{c}]"
                );
                if r < 3 && c < 3 {
                    assert_eq!(matrix[(r, c)], rotation[(r, c)]);
                }
            }
        }
    }
}
#[test]
fn f32_matches_eigen_rotation_matrix_exactly() {
    run::<f32>("f32");
}
#[test]
fn f64_matches_eigen_rotation_matrix_exactly() {
    run::<f64>("f64");
}

fn jacobians<S: BasaltScalar>(key: &str, tolerance: f64) {
    let fixture: Value =
        serde_json::from_str(include_str!("reference/primitives/rotation_matrix.json")).unwrap();
    for case in fixture[key].as_array().unwrap() {
        let tangent = SVector::<S, 6>::from_fn(|i, _| {
            S::from_f64_lossy(case["tangent"][i].as_f64().unwrap())
        });
        for (actual, label) in [
            (right_jacobian_se3_decoupled(&tangent), "jacobian"),
            (
                right_jacobian_inv_se3_decoupled(&tangent),
                "inverse_jacobian",
            ),
        ] {
            for row in 0..6 {
                for col in 0..6 {
                    let expected = case[label][row * 6 + col].as_f64().unwrap();
                    let actual = actual[(row, col)].as_f64();
                    assert!(
                        (actual - expected).abs() <= tolerance * (1. + expected.abs()),
                        "{key} {label}[{row},{col}]: {actual} != {expected}"
                    );
                }
            }
        }
    }
}
#[test]
fn original_f32_decoupled_se3_jacobians() {
    jacobians::<f32>("f32", 1e-5);
}
#[test]
fn original_f64_decoupled_se3_jacobians() {
    jacobians::<f64>("f64", 1e-10);
}
