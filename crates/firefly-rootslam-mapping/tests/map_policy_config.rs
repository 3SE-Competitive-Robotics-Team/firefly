//! The map policy configuration surface.
//!
//! The pose-graph residual scale replaced a raw information matrix in the
//! policy, so the accepted document shape is pinned here.
use firefly_rootslam_mapping::{edge_information::EdgeInformation, map::MapPolicy};

#[test]
fn the_map_policy_reads_the_documented_toml_document() {
    let document = r"
redundancy_fraction = 0.9
redundant_observers = 3
max_keyframes = 1500
cull_batch = 30
essential_min_shared = 4

[edge_information]
reference_depth_m = 7.5
";
    let policy: MapPolicy = toml::from_str(document).unwrap();
    assert_eq!(policy.redundancy_fraction, 0.9);
    assert_eq!(policy.redundant_observers, 3);
    assert_eq!(policy.max_keyframes, 1500);
    assert_eq!(policy.cull_batch, 30);
    assert_eq!(policy.essential_min_shared, Some(4));
    assert_eq!(policy.edge_information.reference_depth_m, 7.5);
    // An absent weight pair means the scale comes from the camera model.
    assert_eq!(policy.edge_information.translation_pixels_per_meter, None);
    assert_eq!(policy.edge_information.rotation_pixels_per_radian, None);
    assert!(policy.validate().is_ok());
}

#[test]
fn the_map_policy_accepts_explicit_pixel_equivalent_weights() {
    let document = r"
redundancy_fraction = 0.9
redundant_observers = 3
max_keyframes = 1500
cull_batch = 30
essential_min_shared = 4

[edge_information]
reference_depth_m = 5.0
translation_pixels_per_meter = 120.0
rotation_pixels_per_radian = 640.0
";
    let policy: MapPolicy = toml::from_str(document).unwrap();
    assert_eq!(
        policy.edge_information.translation_pixels_per_meter,
        Some(120.0)
    );
    assert_eq!(
        policy.edge_information.rotation_pixels_per_radian,
        Some(640.0)
    );
    assert!(policy.validate().is_ok());
}

#[test]
fn an_omitted_scale_falls_back_to_the_camera_model_calibration() {
    let document = r"
redundancy_fraction = 0.9
redundant_observers = 3
max_keyframes = 1500
cull_batch = 30
essential_min_shared = 4
";
    let policy: MapPolicy = toml::from_str(document).unwrap();
    assert_eq!(policy.edge_information, EdgeInformation::default());
    assert!(policy.validate().is_ok());
}

#[test]
fn an_unknown_field_is_rejected() {
    let document = r"
redundancy_fraction = 0.9
redundant_observers = 3
max_keyframes = 1500
cull_batch = 30
essential_min_shared = 4
edge_sqrt_information = [[1.0]]
";
    assert!(toml::from_str::<MapPolicy>(document).is_err());
}
