use super::*;
use crate::{io::dataset_io::ImageData, optical_flow::OpticalFlowFactory};
use firefly_basalt_headers::{camera::generic_camera::GenericCamera, image::image::ManagedImage};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

#[test]
fn fixed_patch_arithmetic_matches_official_sse() {
    let reference: serde_json::Value = serde_json::from_str(include_str!(
        "../../tests/reference/optical_flow_arithmetic.json"
    ))
    .unwrap();
    let numbers = |value: &serde_json::Value| -> Vec<f32> {
        value
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap() as f32)
            .collect()
    };
    let inverse =
        nalgebra::SMatrix::<f32, 3, 52>::from_row_slice(&numbers(&reference["inverse_jacobian"]));
    let residual = SVector::<f32, 52>::from_row_slice(&numbers(&reference["residual"]));
    let actual = crate::optical_flow::patch::patch_increment(&inverse, &residual);
    assert_eq!(actual.as_slice(), numbers(&reference["increment"]));
    for case in reference["updates"].as_array().unwrap() {
        let increment = Vector3::from_row_slice(&numbers(&case["increment"]));
        let update = se2_exp(&increment);
        let expected_update = numbers(&case["update"]);
        let update_values = [
            update.linear[(0, 0)],
            update.linear[(0, 1)],
            update.translation.x,
            update.linear[(1, 0)],
            update.linear[(1, 1)],
            update.translation.y,
            0.0,
            0.0,
            1.0,
        ];
        assert_eq!(update_values.as_slice(), expected_update);
        let before = numbers(&case["before"]);
        let mut transform = AffineCompact2 {
            linear: Matrix2::new(before[0], before[1], before[3], before[4]),
            translation: Vector2::new(before[2], before[5]),
        };
        apply_se2_update(&mut transform, &update);
        let after = [
            transform.linear[(0, 0)],
            transform.linear[(0, 1)],
            transform.translation.x,
            transform.linear[(1, 0)],
            transform.linear[(1, 1)],
            transform.translation.y,
        ];
        assert_eq!(after.as_slice(), numbers(&case["after"]));
    }
}

fn calibration(cameras: i32) -> Calibration<f64> {
    let mut calibration = Calibration::default();
    for camera in 0..cameras {
        let mut model = GenericCamera::from_name("pinhole");
        model.set_from_init(Vector4::new(200.0, 200.0, 128.0, 96.0));
        calibration.intrinsics.push(model);
        calibration.t_i_c.push(Isometry3::from_parts(
            Translation3::new(f64::from(camera) * 0.1, 0.0, 0.0),
            UnitQuaternion::identity(),
        ));
    }
    calibration
}
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn input(cameras: i32, frame: i32) -> Arc<OpticalFlowInput> {
    let mut input = OpticalFlowInput {
        t_ns: 1_000_000_000 + i64::from(frame) * 20_000_000,
        img_data: Vec::new(),
    };
    for camera in 0..cameras {
        let mut image = ManagedImage::new(256, 192);
        for y in 0_i32..192 {
            for x in 0_i32..256 {
                let px = f64::from(x) - f64::from(frame) - f64::from(camera);
                image.data_mut()[y as usize * 256 + x as usize] = (30000.0
                    + 10000.0 * (0.13 * px).sin()
                    + 8000.0 * (0.17 * f64::from(y)).cos()
                    + 6000.0 * (0.11 * px + 0.07 * f64::from(y)).sin())
                    as u16;
            }
        }
        input.img_data.push(ImageData {
            img: Some(Arc::new(image)),
            exposure: 0.0,
        });
    }
    Arc::new(input)
}

#[test]
fn all_three_tracking_modes_and_queues_match_cpp() {
    for mode in ["frame_to_frame", "patch", "multiscale_frame_to_frame"] {
        for cameras in if mode == "multiscale_frame_to_frame" {
            2..=3
        } else {
            1..=3
        } {
            let calibration = calibration(cameras);
            let config = VioConfig {
                optical_flow_type: mode.into(),
                optical_flow_skip_frames: 2,
                ..VioConfig::default()
            };
            let tracker = OpticalFlowFactory::get_optical_flow(&config, &calibration).unwrap();
            let reference: serde_json::Value = serde_json::from_str(
                &std::fs::read_to_string(format!(
                    "{}/tests/reference/{mode}_{cameras}.json",
                    env!("CARGO_MANIFEST_DIR")
                ))
                .unwrap(),
            )
            .unwrap();
            // A missing image must not advance IDs, state or the skip-frame counter.
            tracker
                .input_queue()
                .send(Some(Arc::new(OpticalFlowInput {
                    t_ns: 0,
                    img_data: vec![ImageData::default(); usize::try_from(cameras).unwrap()],
                })))
                .unwrap();
            for frame in 0..3 {
                tracker
                    .input_queue()
                    .send(Some(input(cameras, frame)))
                    .unwrap();
            }
            tracker.input_queue().send(None).unwrap();
            for (frame, expected) in reference.as_array().unwrap().iter().enumerate() {
                let output = tracker.output_queue().recv().unwrap().unwrap();
                assert_eq!(output.t_ns, expected["timestamp"].as_i64().unwrap());
                for (camera, observations) in output.observations.iter().enumerate() {
                    let expected = expected["observations"][camera].as_array().unwrap();
                    assert_eq!(
                        observations.len(),
                        expected.len(),
                        "{mode} cameras {cameras} frame {frame} camera {camera}"
                    );
                    for ((&id, transform), expected) in observations.iter().zip(expected) {
                        assert_eq!(
                            id,
                            usize::try_from(expected["id"].as_u64().unwrap()).unwrap()
                        );
                        for row in 0..2 {
                            for col in 0..3 {
                                let actual = if col == 2 {
                                    transform.translation[row]
                                } else {
                                    transform.linear[(row, col)]
                                };
                                let expected =
                                    expected["transform"][row * 3 + col].as_f64().unwrap();
                                assert!(
                                    (f64::from(actual) - expected).abs() < 1e-4,
                                    "{mode} cam {camera} frame {frame} id {id} ({row},{col}): {actual} != {expected}"
                                );
                            }
                        }
                    }
                }
                if let Some(levels) = expected.get("pyramid_levels") {
                    let actual: Vec<Vec<(usize, usize)>> = output
                        .pyramid_levels
                        .iter()
                        .map(|m| m.iter().map(|(&id, &l)| (id, l)).collect())
                        .collect();
                    assert_eq!(serde_json::to_value(actual).unwrap(), *levels);
                }
            }
            assert!(tracker.output_queue().recv().unwrap().is_none());
            tracker.shutdown().unwrap();
        }
    }
}

#[test]
fn factory_supports_every_original_pattern_and_propagates_termination() {
    let calibration = calibration(2);
    for mode in ["frame_to_frame", "patch", "multiscale_frame_to_frame"] {
        for pattern in [24, 52, 51, 50] {
            let config = VioConfig {
                optical_flow_type: mode.into(),
                optical_flow_pattern: pattern,
                ..VioConfig::default()
            };
            let tracker = OpticalFlowFactory::get_optical_flow(&config, &calibration).unwrap();
            assert_eq!(
                tracker.patch_coord.ncols(),
                if pattern == 24 { 24 } else { 52 }
            );
            tracker.input_queue().send(None).unwrap();
            assert!(tracker.output_queue().recv().unwrap().is_none());
            tracker.shutdown().unwrap();
        }
    }
}

#[test]
fn worker_connects_backend_queue_and_reports_processing_errors() {
    let calibration = calibration(2);
    let (sender, receiver) = std::sync::mpsc::sync_channel(1);
    let tracker = OpticalFlowFactory::get_optical_flow_with_output(
        &VioConfig::default(),
        &calibration,
        sender,
    )
    .unwrap();
    tracker.input_queue().send(Some(input(2, 0))).unwrap();
    tracker.input_queue().send(Some(input(2, 1))).unwrap();
    tracker.input_queue().send(None).unwrap();
    assert_eq!(receiver.recv().unwrap().unwrap().t_ns, 1_000_000_000);
    assert_eq!(receiver.recv().unwrap().unwrap().t_ns, 1_020_000_000);
    assert!(receiver.recv().unwrap().is_none());
    tracker.shutdown().unwrap();

    let tracker =
        OpticalFlowFactory::get_optical_flow(&VioConfig::default(), &calibration).unwrap();
    tracker
        .input_queue()
        .send(Some(Arc::new(OpticalFlowInput::default())))
        .unwrap();
    assert!(tracker.output_queue().recv().unwrap().is_none());
    assert!(tracker.shutdown().is_err());
    // Drop must also stop a worker waiting for its first input.
    drop(OpticalFlowFactory::get_optical_flow(&VioConfig::default(), &calibration).unwrap());
}
