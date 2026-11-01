use firefly_calibration::calibration::Calibration;
use serde::Deserialize;

#[derive(Deserialize)]
struct Archive {
    value0: Calibration<f64>,
}

#[test]
fn original_euroc_calibration_loads_and_preserves_coefficients() {
    let text = include_str!("reference/euroc_ds_calib.json");
    let source: serde_json::Value = serde_json::from_str(text).unwrap();
    let calibration = serde_json::from_str::<Archive>(text).unwrap().value0;
    assert_eq!(calibration.t_i_c.len(), 2);
    assert_eq!(calibration.intrinsics[0].name(), "ds");
    assert_eq!(calibration.vignette[0].num_knots(), 15);
    let saved = serde_json::to_value(&calibration).unwrap();
    for (name, value) in saved.as_object().unwrap() {
        assert_eq!(value, &source["value0"][name], "{name}");
    }
    let mut missing = source["value0"].clone();
    missing.as_object_mut().unwrap().remove("gyro_noise_std");
    assert!(serde_json::from_value::<Calibration<f64>>(missing).is_err());
}
