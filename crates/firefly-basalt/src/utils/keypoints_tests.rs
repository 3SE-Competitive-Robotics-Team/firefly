use super::*;
use serde_json::Value;

#[test]
fn real_dataset_cell_fast_candidates_match_original() {
    let reference: Value = serde_json::from_str(include_str!(
        "../../tests/reference/keypoints_dataset_cell.json"
    ))
    .unwrap();
    let pixels: Vec<u16> = serde_json::from_value(reference["pixels"].clone()).unwrap();
    let bytes = pixels.iter().map(|pixel| (pixel >> 8) as u8).collect();
    let image = Matrix::from_vec(40, 40, 1, bytes);
    let points = FastFeatureDetector::new(
        reference["threshold"].as_u64().unwrap() as u8,
        true,
        FastType::Type9_16,
    )
    .detect(&image)
    .unwrap();
    let actual: Vec<_> = points
        .iter()
        .map(|point| [point.pt.x, point.pt.y, point.response])
        .collect();
    let expected: Vec<[f32; 3]> = serde_json::from_value(reference["raw_fast"].clone()).unwrap();
    assert_eq!(actual, expected);
}

fn fixture(scenario: &str) -> Value {
    serde_json::from_str(
        &std::fs::read_to_string(format!(
            "{}/tests/reference/keypoints_{scenario}.json",
            env!("CARGO_MANIFEST_DIR")
        ))
        .unwrap(),
    )
    .unwrap()
}

fn input(scenario: &str) -> Vec<u16> {
    let stride = if scenario == "pitched" { 132 } else { 128 };
    let mut pixels = vec![65535; stride * 96];
    for y in 0..96 {
        for x in 0..128 {
            pixels[y * stride + x] = u16::try_from(match scenario {
                "checker" => {
                    if (x / 8 + y / 8) % 2 == 0 {
                        3000
                    } else {
                        60000
                    }
                }
                "black" => 0,
                _ => 1000 + (17 * x * x + 31 * y * y + 7 * x * y) % 64000,
            })
            .unwrap();
        }
    }
    pixels
}

fn corners(actual: &[Vector2<f64>], expected: &Value) {
    let expected = expected.as_array().unwrap();
    assert_eq!(actual.len(), expected.len());
    for (actual, expected) in actual.iter().zip(expected) {
        assert_eq!(actual.x, expected[0].as_f64().unwrap());
        assert_eq!(actual.y, expected[1].as_f64().unwrap());
    }
}

#[test]
fn detection_matches_cpp() {
    for scenario in ["normal", "checker", "black", "occupied", "pitched"] {
        let reference = fixture(scenario);
        let pixels = input(scenario);
        let image = Image::new(
            &pixels,
            128,
            96,
            if scenario == "pitched" { 132 } else { 128 },
        );
        let mut kd = KeypointsData::default();
        let occupied = if scenario == "occupied" {
            vec![Vector2::new(45.2, 44.7)]
        } else {
            vec![]
        };
        detect_keypoints(&image, &mut kd, 32, 2, &occupied).unwrap();
        eprintln!("FAST {scenario}");
        corners(&kd.corners, &reference["corners"]);
        detect_keypoints_mapping(&image, &mut kd, 20).unwrap();
        eprintln!("GFTT {scenario}");
        corners(&kd.corners, &reference["mapping"]);
    }
}

#[test]
fn angles_and_descriptor_bits_match_cpp() {
    for scenario in ["normal", "checker", "black", "occupied", "pitched"] {
        let reference = fixture(scenario);
        let pixels = input(scenario);
        let image = Image::new(
            &pixels,
            128,
            96,
            if scenario == "pitched" { 132 } else { 128 },
        );
        let mut kd = KeypointsData {
            corners: vec![
                Vector2::new(31.25, 30.75),
                Vector2::new(53.8, 46.2),
                Vector2::new(81.0, 62.0),
            ],
            ..KeypointsData::default()
        };
        compute_angles(&image, &mut kd, true);
        compute_descriptors(&image, &mut kd);
        for (i, angle) in kd.corner_angles.iter().enumerate() {
            assert!((angle - reference["angles"][i].as_f64().unwrap()).abs() < 1e-12);
            for word in 0..4 {
                assert_eq!(
                    kd.corner_descriptors[i][word],
                    reference["descriptors"][i][word].as_u64().unwrap()
                );
            }
        }
        compute_angles(&image, &mut kd, false);
        assert_eq!(kd.corner_angles, vec![0.0; 3]);
    }
}

#[test]
fn mutual_matching_ties_match_cpp() {
    let first = [[0, 0, 0, 0], [3, 0, 0, 0], [3, 0, 0, 0], [255, 0, 0, 0]];
    let second = [[3, 0, 0, 0], [0, 0, 0, 0], [127, 0, 0, 0]];
    let mut matches = Vec::new();
    match_descriptors(&first, &second, &mut matches, 10, 1.2);
    assert_eq!(
        serde_json::to_value(&matches).unwrap(),
        fixture("normal")["matches"]
    );
    match_descriptors(&first, &second, &mut matches, 0, 1.2);
    assert_eq!(matches, []);
    // Preserve the source's -1 sentinel and unordered_map::operator[] insertion
    // even for a threshold beyond the 256-bit Hamming distance range.
    match_descriptors(&first, &[], &mut matches, 501, 1.0);
    assert_eq!(
        serde_json::to_value(matches).unwrap(),
        fixture("normal")["empty_matches"]
    );
}

#[test]
fn complete_ransac_and_refinement_match_cpp() {
    use nalgebra::{Vector3, Vector4};
    let reference = fixture("ransac");
    let rotation = Rotation3::from_euler_angles(0.04, -0.08, 0.12);
    let translation = Vector3::new(0.3, -0.2, 0.1);
    let mut first = KeypointsData::default();
    let mut second = KeypointsData::default();
    let mut matches = MatchData::default();
    for i in 0..48 {
        let p = Vector3::new(
            f64::from(i % 6) * 0.6 - 1.4,
            f64::from(i / 6) * 0.4 - 1.2,
            4.0 + f64::from(i % 7) * 0.35,
        );
        let x = p.normalize();
        let y = (rotation.inverse() * (p - translation)).normalize();
        first.corners_3d.push(Vector4::new(x.x, x.y, x.z, 0.0));
        second.corners_3d.push(Vector4::new(y.x, y.y, y.z, 0.0));
        matches.matches.push((i, i));
    }
    find_inliers_ransac(&first, &second, 1e-8, 30, &mut matches).unwrap();
    assert_eq!(
        serde_json::to_value(&matches.inliers).unwrap(),
        reference["inliers"]
    );
    let actual = matches.t_i_j.to_homogeneous();
    for row in 0..4 {
        for col in 0..4 {
            assert!(
                (actual[(row, col)] - reference["matrix"][row * 4 + col].as_f64().unwrap()).abs()
                    < 1e-6
            );
        }
    }
    find_inliers_ransac(&first, &second, 1e-8, 49, &mut matches).unwrap();
    assert_eq!(matches.inliers, []);
    assert!(
        find_inliers_ransac(&KeypointsData::default(), &second, 1e-8, 30, &mut matches).is_err()
    );
}
