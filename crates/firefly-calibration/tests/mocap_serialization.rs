use firefly_calibration::calibration::MocapCalibration;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use serde_json::Value;
#[test]
fn official_cereal_mocap_archive_preserves_transforms_and_integer_offsets() {
    let fixture: Value =
        serde_json::from_str(include_str!("reference/mocap_calibration.json")).unwrap();
    let calibration = MocapCalibration {
        t_moc_w: Isometry3::from_parts(
            Translation3::new(1., 2., 3.),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.125, -0.25, 0.375)),
        ),
        t_i_mark: Isometry3::from_parts(
            Translation3::new(-1., 0.5, 0.25),
            UnitQuaternion::from_scaled_axis(Vector3::new(-0.25, 0.5, -0.125)),
        ),
        mocap_time_offset_ns: -1_234_567_890_123_456_789,
        mocap_to_imu_offset_ns: 1_234_567_890_123_456_789,
    };
    let archive = serde_json::to_value(&calibration).unwrap();
    assert_eq!(archive, fixture["value0"]);
    let decoded: MocapCalibration = serde_json::from_value(archive).unwrap();
    assert_eq!(decoded.t_moc_w, calibration.t_moc_w);
    assert_eq!(decoded.t_i_mark, calibration.t_i_mark);
    assert_eq!(
        decoded.mocap_time_offset_ns,
        calibration.mocap_time_offset_ns
    );
    assert_eq!(
        decoded.mocap_to_imu_offset_ns,
        calibration.mocap_to_imu_offset_ns
    );
}
