use firefly_basalt::{
    thirdparty::ethz_apriltag2::{
        tag_detector::TagDetector, tag_family::TagFamily, tag36h11::CODES,
    },
    utils::apriltag::ApriltagDetector,
};
use firefly_calibration::image::image::ManagedImage;

#[test]
fn original_tag_dictionary_rotations_and_one_bit_recovery() {
    let family = TagFamily::new(36, 11, &CODES, 2);
    for (id, &code) in CODES.iter().enumerate() {
        let mut rotated = code;
        for _ in 0..4 {
            let detection = family.decode(rotated);
            assert!(detection.good);
            assert_eq!(detection.id, id as i32);
            assert_eq!(detection.hamming_distance, 0);
            let corrected = family.decode(rotated ^ 1);
            assert!(corrected.good);
            assert_eq!(corrected.id, id as i32);
            assert_eq!(corrected.hamming_distance, 1);
            rotated = TagFamily::rotate90(rotated, 6);
        }
        assert_eq!(rotated, code);
    }
}

#[test]
fn original_cpp_tag_detection_and_subpixel_reference() {
    compare_reference(true);
}
#[test]
fn original_cpp_raw_tag_reference() {
    compare_reference(false);
}
#[test]
fn original_cpp_planar_tag_pose() {
    use firefly_basalt::thirdparty::ethz_apriltag2::tag_detection::TagDetection;
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("reference/apriltag.json")).unwrap();
    for reference in fixture.as_array().unwrap() {
        for tag in reference["raw"]
            .as_array()
            .unwrap()
            .iter()
            .chain(reference["pose_cases"].as_array().into_iter().flatten())
        {
            let mut detection = TagDetection::default();
            for (i, p) in detection.points.iter_mut().enumerate() {
                *p = (
                    tag["points"][i][0].as_f64().unwrap() as f32,
                    tag["points"][i][1].as_f64().unwrap() as f32,
                );
            }
            let pose = detection
                .relative_transform(0.16, 350., 370., 120., 120.)
                .unwrap();
            for row in 0..4 {
                for col in 0..4 {
                    let expected = tag["pose"][row * 4 + col].as_f64().unwrap();
                    assert!(
                        (pose[(row, col)] - expected).abs() < 1e-6,
                        "{row},{col}: {} vs {expected}",
                        pose[(row, col)]
                    );
                }
            }
        }
    }
}
fn compare_reference(check_subpixel: bool) {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("reference/apriltag.json")).unwrap();
    let raw = TagDetector::new(TagFamily::new(36, 11, &CODES, 2));
    for (case, reference) in fixture.as_array().unwrap().iter().enumerate() {
        let pixels: Vec<_> = reference["pixels"]
            .as_array()
            .unwrap()
            .iter()
            .map(|value| value.as_u64().unwrap() as u8)
            .collect();
        let mut detections = raw.extract_tags(240, 240, &pixels);
        detections.sort_by_key(|d| d.id);
        let expected = reference["raw"].as_array().unwrap();
        assert_eq!(detections.len(), expected.len(), "case {case}");
        for (detection, expected) in detections.iter().zip(expected) {
            assert_eq!(i64::from(detection.id), expected["id"].as_i64().unwrap());
            assert_eq!(
                i64::from(detection.hamming_distance),
                expected["hamming"].as_i64().unwrap()
            );
            for (corner, &point) in detection.points.iter().enumerate() {
                assert!(
                    (f64::from(point.0) - expected["points"][corner][0].as_f64().unwrap()).abs()
                        < 2e-3,
                    "case {case} raw x"
                );
                assert!(
                    (f64::from(point.1) - expected["points"][corner][1].as_f64().unwrap()).abs()
                        < 2e-3,
                    "case {case} raw y"
                );
            }
        }
        if !check_subpixel {
            continue;
        }
        let mut image = ManagedImage::new(240, 240);
        for (output, &input) in image.data_mut().iter_mut().zip(&pixels) {
            *output = u16::from(input) << 8;
        }
        let mut detector = ApriltagDetector::new(16);
        let (good, bad) = detector.detect_tags(&image).unwrap();
        for (prefix, result) in [("", good), ("rejected_", bad)] {
            let ids = &reference[format!("{prefix}ids")];
            let expected_ids: Vec<_> = ids
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_i64().unwrap() as i32)
                .collect();
            assert_eq!(result.corner_ids, expected_ids, "case {case}");
            for (i, point) in result.corners.iter().enumerate() {
                for coordinate in 0..2 {
                    assert!(
                        (point[coordinate]
                            - reference[format!("{prefix}points")][i][coordinate]
                                .as_f64()
                                .unwrap())
                        .abs()
                            < 2e-3,
                        "case {case}, {prefix}corner{i} coordinate{coordinate}"
                    );
                }
                assert!(
                    (result.radii[i] - reference[format!("{prefix}radii")][i].as_f64().unwrap())
                        .abs()
                        < 1e-4
                );
            }
        }
    }
}
