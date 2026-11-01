use firefly_basalt::calibration::cam_imu_calib::initialize_rotation;
use firefly_basalt::io::dataset_io::GyroData;
use firefly_basalt_headers::utils::sophus_utils::{so3_exp, so3_matrix, so3_product};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

#[test]
fn angular_initialization_matches_original_eigen_angle_axis() {
    let cases: Vec<serde_json::Value> =
        serde_json::from_str(include_str!("reference/cam_imu_calib.json")).unwrap();
    for case in cases {
        let visual: Vec<f64> = serde_json::from_value(case["visual"].clone()).unwrap();
        let imu: Vec<f64> = serde_json::from_value(case["imu"].clone()).unwrap();
        let expected: Vec<f64> = serde_json::from_value(case["rotation"].clone()).unwrap();
        let mut pose = Isometry3::identity();
        let mut poses = vec![(0, pose)];
        // Upstream excludes gyro sample zero even when it matches the first pose.
        let mut gyro = vec![GyroData {
            timestamp_ns: 0,
            data: Vector3::repeat(1e8),
        }];
        for (index, (velocity, measurement)) in visual
            .as_chunks::<3>()
            .0
            .iter()
            .zip(imu.as_chunks::<3>().0.iter())
            .enumerate()
        {
            let time = i64::try_from(index).unwrap() * 10_000_000;
            pose = Isometry3::from_parts(
                Translation3::identity(),
                so3_product(
                    &pose.rotation,
                    &so3_exp(&(Vector3::from_column_slice(velocity) * 0.01)),
                ),
            );
            poses.push((time + 10_000_000, pose));
            gyro.push(GyroData {
                timestamp_ns: time,
                data: Vector3::from_column_slice(measurement),
            });
        }
        let rotation =
            so3_matrix(&initialize_rotation(&poses, &gyro, &UnitQuaternion::identity()).unwrap());
        for row in 0..3 {
            for column in 0..3 {
                assert!((rotation[(row, column)] - expected[row * 3 + column]).abs() < 1e-10);
            }
        }
    }
}

#[test]
fn unobservable_rotation_and_missing_gyro_are_errors() {
    let poses = [
        (0, Isometry3::identity()),
        (10_000_000, Isometry3::identity()),
    ];
    let gyro = [GyroData::default(), GyroData::default()];
    assert!(initialize_rotation(&poses, &[], &UnitQuaternion::identity()).is_err());
    assert!(initialize_rotation(&poses, &gyro, &UnitQuaternion::identity()).is_err());
}
