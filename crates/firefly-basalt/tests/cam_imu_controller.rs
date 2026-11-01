use firefly_basalt::{
    calibration::{
        aprilgrid::AprilGrid,
        calibration_helper::{CalibCornerData, CalibInitPoseData},
        cam_imu_calib::CamImuCalib,
    },
    io::dataset_io::{AccelData, GyroData, ImageData, VioDataset},
    optimization::spline_optimize::SplineOptimization,
    utils::common_types::TimeCamId,
};
use firefly_basalt_headers::utils::sophus_utils::{se3_inverse, se3_matrix, se3_product, so3_exp};
use nalgebra::{Isometry3, Translation3, Vector2, Vector3, Vector4};
use serde_json::Value;
use std::{
    collections::BTreeMap,
    sync::{Arc, Mutex},
};

struct Dataset {
    times: Vec<i64>,
    accel: Vec<AccelData>,
    gyro: Vec<GyroData>,
    gt: Vec<i64>,
    poses: Vec<Isometry3<f64>>,
}
impl VioDataset for Dataset {
    fn num_cams(&self) -> usize {
        2
    }
    fn image_timestamps(&mut self) -> &mut Vec<i64> {
        &mut self.times
    }
    fn accel_data(&self) -> &[AccelData] {
        &self.accel
    }
    fn gyro_data(&self) -> &[GyroData] {
        &self.gyro
    }
    fn gt_timestamps(&self) -> &[i64] {
        &self.gt
    }
    fn gt_pose_data(&self) -> &[Isometry3<f64>] {
        &self.poses
    }
    fn mocap_to_imu_offset_ns(&self) -> i64 {
        0
    }
    fn image_data(&mut self, _: i64) -> firefly_error::Result<Vec<ImageData>> {
        Ok(vec![ImageData::default(); 2])
    }
}
fn pose(x: f64) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(x, -0.2 * x, -1.),
        so3_exp(&Vector3::new(0.1 * x, 0.2 * x * x, 0.3 * x * x * x)),
    )
}
fn assert_values(actual: impl IntoIterator<Item = f64>, expected: &Value) {
    let actual: Vec<_> = actual.into_iter().collect();
    let expected = expected.as_array().unwrap();
    assert_eq!(actual.len(), expected.len());
    for (a, b) in actual.iter().zip(expected) {
        let b = b.as_f64().unwrap();
        assert!((a - b).abs() <= 1e-10 * (1. + b.abs()), "{a} != {b}");
    }
}
fn assert_pose(actual: &Isometry3<f64>, expected: &Value) {
    let matrix = se3_matrix(actual);
    assert_values(
        (0..4).flat_map(|r| (0..4).map(move |c| matrix[(r, c)])),
        expected,
    );
}
fn setup() -> CamImuCalib {
    let mut dataset = Dataset {
        times: (1..=6).map(|i| i * 100_000_000).collect(),
        accel: Vec::new(),
        gyro: Vec::new(),
        gt: (1..=8).map(|i| i * 100_000_000).collect(),
        poses: (1..=8).map(|i| pose(0.1 * f64::from(i))).collect(),
    };
    for i in 0..=200 {
        let x = f64::from(i);
        let timestamp_ns = i64::from(i) * 5_000_000;
        dataset.accel.push(AccelData {
            timestamp_ns,
            data: Vector3::new(0.1 + 0.001 * x, -0.2, 9.7),
        });
        dataset.gyro.push(GyroData {
            timestamp_ns,
            data: Vector3::new(
                0.2 + 0.001 * x,
                -0.1 + 0.000_01 * x * x,
                0.3 - 0.000_003 * x * x,
            ),
        });
    }
    let mut optimizer = SplineOptimization::default();
    optimizer.reset_calib(2, &["pinhole".into(), "pinhole".into()]);
    optimizer.reset_mocap_calib();
    let calibration = optimizer.calib.as_mut().unwrap();
    for camera in 0..2 {
        let x = camera as f64;
        calibration.intrinsics[camera].set_from_init(Vector4::new(350., 370., 320., 240.));
        calibration.t_i_c[camera] = Isometry3::from_parts(
            Translation3::new(0.1 * x, 0.02, -0.01),
            so3_exp(&Vector3::new(0.01, -0.02 * x, 0.03)),
        );
    }
    let mut controller = CamImuCalib::new(
        Arc::new(Mutex::new(dataset)),
        AprilGrid::from_parameters(3, 3, 0.04, 0.3).unwrap(),
        optimizer,
    );
    for frame in 1..=6 {
        for camera in 0..2 {
            let id = TimeCamId {
                frame_id: frame * 100_000_000,
                cam_id: camera,
            };
            let transform = se3_product(
                &pose(0.1 * frame as f64),
                controller.optimizer.cam_t_i_c(camera),
            );
            controller.initial_poses.insert(
                id,
                CalibInitPoseData {
                    t_a_c: transform,
                    num_inliers: 36,
                    reprojected_corners: Vec::new(),
                },
            );
            let count = if frame == 2 && camera == 0 { 14 } else { 36 };
            let camera_matrix = se3_matrix(&se3_inverse(&transform));
            let corners =
                controller.april_grid.aprilgrid_corner_pos_3d[..count]
                    .iter()
                    .map(|point| {
                        let mut pixel = Vector2::zeros();
                        assert!(
                            controller.optimizer.calib.as_ref().unwrap().intrinsics[camera]
                                .project(&(camera_matrix * point), &mut pixel, None)
                        );
                        pixel
                    })
                    .collect();
            controller.corners.insert(
                id,
                CalibCornerData {
                    corners,
                    corner_ids: (0..count).map(|i| i as i32).collect(),
                    radii: vec![2.; count],
                },
            );
        }
    }
    controller
}
#[test]
fn original_controller_initialization_and_mocap() {
    let expected: Value =
        serde_json::from_str(include_str!("reference/cam_imu_controller.json")).unwrap();
    let mut controller = setup();
    controller.init_optimization().unwrap();
    assert_values(
        controller.optimizer.g().iter().copied(),
        &expected["gravity"],
    );
    assert!(
        (controller.optimizer.calib.as_ref().unwrap().imu_update_rate
            - expected["rate"].as_f64().unwrap())
        .abs()
            < 1e-10
    );
    assert_eq!(
        controller.optimizer.min_time_ns(),
        expected["min"].as_i64().unwrap()
    );
    assert_eq!(
        controller.optimizer.max_time_ns(),
        expected["max"].as_i64().unwrap()
    );
    for (index, value) in expected["spline_poses"]
        .as_array()
        .unwrap()
        .iter()
        .enumerate()
    {
        assert_pose(
            &controller.optimizer.t_w_i((index as i64 + 1) * 100_000_000),
            value,
        );
    }
    for value in expected["projections"].as_array().unwrap() {
        let id = TimeCamId {
            frame_id: value["time"].as_i64().unwrap(),
            cam_id: value["camera"].as_u64().unwrap() as usize,
        };
        let actual = &controller.reprojected_corners[&id];
        assert_eq!(
            actual.corners_proj.len(),
            value["points"].as_array().unwrap().len()
        );
        for (point, reference) in actual
            .corners_proj
            .iter()
            .zip(value["points"].as_array().unwrap())
        {
            assert_values(point.iter().copied(), reference);
        }
        assert_eq!(
            serde_json::to_value(&actual.corners_proj_success).unwrap(),
            value["valid"]
        );
    }
    controller.optimizer.mocap_calib.as_mut().unwrap().t_i_mark = Isometry3::from_parts(
        Translation3::new(0.02, 0.01, -0.03),
        so3_exp(&Vector3::new(0.1, -0.03, 0.04)),
    );
    controller.init_mocap().unwrap();
    assert!(controller.optimizer.mocap_initialized);
    assert_pose(&controller.optimizer.t_w_moc(), &expected["t_w_moc"]);
    assert_pose(&controller.optimizer.t_mark_i(), &expected["t_mark_i"]);
    let diagnostics = controller.recompute_data_log().unwrap();
    assert_eq!(diagnostics.imu.len(), 201);
    assert_eq!(diagnostics.poses.len(), 6);
    assert_eq!(diagnostics.mocap.len(), 8);
    assert_diagnostics(&diagnostics.imu, &expected["diagnostics"]["imu"]);
    assert_diagnostics(&diagnostics.poses, &expected["diagnostics"]["poses"]);
    assert_diagnostics(&diagnostics.mocap, &expected["diagnostics"]["mocap"]);
}

fn assert_diagnostics<const N: usize>(actual: &[[f32; N]], expected: &Value) {
    let expected = expected.as_array().unwrap();
    assert_eq!(actual.len(), expected.len());
    for (row, reference) in actual.iter().zip(expected) {
        let reference = reference.as_array().unwrap();
        assert_eq!(N, reference.len());
        for (value, reference) in row.iter().zip(reference) {
            let reference = reference.as_f64().unwrap();
            assert!((f64::from(*value) - reference).abs() <= 1e-5 * (1. + reference.abs()));
        }
    }
}

#[test]
fn too_few_corners_excludes_both_cameras_at_that_timestamp() {
    let expected: Value =
        serde_json::from_str(include_str!("reference/cam_imu_controller.json")).unwrap();
    let mut controller = setup();
    controller.init_optimization().unwrap();
    let mut stats = BTreeMap::new();
    controller.optimize_with_param(false, &mut stats);
    assert_eq!(stats["num_points"], 360.);
    assert_eq!(
        stats["num_points"],
        expected["stats"]["num_points"].as_f64().unwrap()
    );
}
