use firefly_basalt::utils::{
    sim_utils::SimObservations,
    test_utils::{JacobianOptions, test_jacobian},
    vis_utils::{CAM_COLOR, getcolor, render_camera},
};
use nalgebra::{DMatrix, DVector, Matrix4};
use serde_json::Value;

fn fixture() -> Value {
    serde_json::from_str(include_str!("reference/utility_helpers.json")).unwrap()
}
#[test]
fn original_cereal_simulated_observations() {
    let reference = &fixture()["simulation"]["value0"];
    let observations: SimObservations = serde_json::from_value(reference.clone()).unwrap();
    assert_eq!(serde_json::to_value(observations).unwrap(), *reference);
}
#[test]
fn original_camera_vertices_and_color_ramp() {
    let reference = fixture();
    for case in reference["colors"].as_array().unwrap() {
        assert_eq!(
            serde_json::to_value(getcolor(case["p"].as_f64().unwrap() as f32, 4.)).unwrap(),
            case["rgb"]
        );
    }
    for case in reference["drawings"].as_array().unwrap() {
        let mut transform = Matrix4::identity();
        transform[(0, 3)] = 0.25;
        let drawing = render_camera(
            transform,
            2.5,
            CAM_COLOR,
            case["size"].as_f64().unwrap() as f32,
        );
        assert_eq!(drawing.color, CAM_COLOR);
        assert_eq!(drawing.line_width, 2.5);
        assert_eq!(drawing.transform, transform);
        for (line, expected) in drawing.lines.iter().zip(case["lines"].as_array().unwrap()) {
            for i in 0..3 {
                assert_eq!(f64::from(line[i]), expected[i].as_f64().unwrap());
            }
        }
    }
}
#[test]
fn original_jacobian_reporting_predicate() {
    assert_eq!(JacobianOptions::default().eps, 1e-8);
    assert_eq!(JacobianOptions::default().max_norm, 1e-4);
    for case in fixture()["checks"].as_array().unwrap() {
        let kind = case["kind"].as_i64().unwrap();
        let mut analytic = DMatrix::<f64>::identity(2, 2);
        if kind == 1 {
            analytic[(0, 0)] = 2.;
        }
        if kind == 2 {
            analytic[(0, 0)] = f64::NAN;
        }
        let result = test_jacobian(
            "probe",
            &analytic,
            |value| {
                if kind == 3 {
                    DVector::repeat(2, f64::NAN)
                } else {
                    value.clone()
                }
            },
            &DVector::from_vec(vec![0.25, -0.5]),
        );
        assert_eq!(result.reported, case["reported"].as_bool().unwrap());
        assert_eq!(result.name, "probe");
        assert_eq!(result.numerical.shape(), (2, 2));
    }
}
