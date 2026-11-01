use firefly_basalt::utils::common_types::CerealBinary;
use firefly_calibration::observation::{CalibCornerData, CalibInitPoseData};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector2};

fn bytes(reference: &str) -> Vec<u8> {
    reference
        .as_bytes()
        .as_chunks::<2>()
        .0
        .iter()
        .map(|s| u8::from_str_radix(std::str::from_utf8(s).unwrap(), 16).unwrap())
        .collect()
}
fn compare<T: CerealBinary>(value: &T, reference: &str) {
    let expected = bytes(reference);
    let mut encoded = Vec::new();
    value.write_cereal_binary(&mut encoded).unwrap();
    assert_eq!(encoded, expected);
    let mut input = expected.as_slice();
    let decoded = T::read_cereal_binary(&mut input).unwrap();
    assert_eq!(input, [] as [u8; 0]);
    let mut roundtrip = Vec::new();
    decoded.write_cereal_binary(&mut roundtrip).unwrap();
    assert_eq!(roundtrip, expected);
    for end in 0..expected.len() {
        assert!(
            T::read_cereal_binary(&mut &expected[..end]).is_err(),
            "truncation at {end}"
        );
    }
}
#[test]
#[cfg(all(target_endian = "little", target_pointer_width = "64"))]
fn official_calibration_native_archives() {
    let reference: serde_json::Value =
        serde_json::from_str(include_str!("reference/calibration_archive.json")).unwrap();
    let corners = CalibCornerData {
        corners: vec![Vector2::new(1.25, -2.5), Vector2::new(0., 1024.125)],
        corner_ids: vec![-1, 321],
        radii: vec![0., 2.75],
    };
    let pose = CalibInitPoseData {
        t_a_c: Isometry3::from_parts(
            Translation3::new(1.5, -2., 0.125),
            UnitQuaternion::new_unchecked(Quaternion::new(0.5, 0.5, 0.5, 0.5)),
        ),
        num_inliers: 17,
        reprojected_corners: vec![Vector2::new(12.5, -6.25), Vector2::new(3., 4.)],
    };
    compare(&corners, reference["corners"].as_str().unwrap());
    compare(&pose, reference["pose"].as_str().unwrap());
    compare(
        &CalibCornerData::default(),
        reference["empty_corners"].as_str().unwrap(),
    );
    compare(
        &CalibInitPoseData::default(),
        reference["empty_pose"].as_str().unwrap(),
    );
}
