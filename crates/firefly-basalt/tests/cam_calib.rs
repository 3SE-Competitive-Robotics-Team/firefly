use firefly_basalt::{
    calibration::{
        calibration_helper::{CalibCornerData, CalibInitPoseData},
        cam_calib::CamCalib,
    },
    io::dataset_io::{AccelData, GyroData, ImageData, VioDataset},
    optimization::poses_optimize::PosesOptimization,
    utils::common_types::TimeCamId,
};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector2, Vector3, Vector4};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
};
struct Dataset {
    times: Vec<i64>,
}
impl VioDataset for Dataset {
    fn num_cams(&self) -> usize {
        3
    }
    fn image_timestamps(&mut self) -> &mut Vec<i64> {
        &mut self.times
    }
    fn accel_data(&self) -> &[AccelData] {
        &[]
    }
    fn gyro_data(&self) -> &[GyroData] {
        &[]
    }
    fn gt_timestamps(&self) -> &[i64] {
        &[]
    }
    fn gt_pose_data(&self) -> &[Isometry3<f64>] {
        &[]
    }
    fn mocap_to_imu_offset_ns(&self) -> i64 {
        0
    }
    fn image_data(&mut self, _: i64) -> firefly_error::Result<Vec<ImageData>> {
        Ok(vec![ImageData::default(); 3])
    }
}
struct Cache(PathBuf);
impl Drop for Cache {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).unwrap();
    }
}
fn setup() -> (CamCalib, Cache) {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let cache = Cache(
        root.join("../../logs")
            .join(format!("basalt-cam-calib-test-{}", std::process::id())),
    );
    std::fs::create_dir_all(&cache.0).unwrap();
    let mut controller = CamCalib::new(
        Path::new(""),
        "bag",
        &root.join("tests/reference/cam_calib_grid.json"),
        &cache.0,
        "probe",
        1,
        vec!["pinhole".into(); 3],
    )
    .unwrap();
    controller.vio_dataset = Some(Arc::new(Mutex::new(Dataset {
        times: vec![100, 200, 300],
    })));
    let mut opt = PosesOptimization::new();
    opt.reset_calib(3, &vec!["pinhole".into(); 3]);
    for camera in &mut opt.calib.as_mut().unwrap().intrinsics {
        camera.set_from_init(Vector4::new(350., 370., 320., 240.));
    }
    for frame in 0..3 {
        let time = (frame + 1) * 100;
        for camera in 0..3 {
            let pose = Isometry3::from_parts(
                Translation3::new(
                    0.1 * camera as f64,
                    -0.02 * frame as f64,
                    -1. - 0.1 * frame as f64,
                ),
                UnitQuaternion::from_scaled_axis(Vector3::new(
                    0.02 * frame as f64,
                    -0.03 * camera as f64,
                    0.01,
                )),
            );
            let num_inliers = match frame {
                0 => {
                    if camera == 2 {
                        15
                    } else {
                        20
                    }
                }
                1 => {
                    if camera == 0 {
                        14
                    } else {
                        25
                    }
                }
                _ => {
                    if camera == 2 {
                        14
                    } else {
                        30
                    }
                }
            };
            let id = TimeCamId::new(time, camera);
            controller.calib_init_poses.insert(
                id,
                CalibInitPoseData {
                    t_a_c: pose,
                    num_inliers,
                    ..Default::default()
                },
            );
            let mut corners = CalibCornerData::default();
            for (i, point) in controller
                .april_grid
                .aprilgrid_corner_pos_3d
                .iter()
                .enumerate()
            {
                let mut pixel = Vector2::zeros();
                assert!(opt.calib.as_ref().unwrap().intrinsics[camera].project(
                    &(pose.inverse().to_homogeneous() * point),
                    &mut pixel,
                    None
                ));
                corners.corners.push(pixel + Vector2::new(0.1, -0.2));
                corners.corner_ids.push(i as i32);
                corners.radii.push(2.);
            }
            controller.calib_corners.insert(id, corners);
        }
    }
    controller.calib_opt = Some(opt);
    (controller, cache)
}
fn compare_pose(actual: Isometry3<f64>, expected: &serde_json::Value) {
    let matrix = actual.to_homogeneous();
    for row in 0..4 {
        for col in 0..4 {
            assert!((matrix[(row, col)] - expected[row * 4 + col].as_f64().unwrap()).abs() < 1e-12);
        }
    }
}
#[test]
fn original_controller_camera_graph_projections_and_stats() {
    let reference: serde_json::Value =
        serde_json::from_str(include_str!("reference/cam_calib.json")).unwrap();
    let (mut controller, _cache) = setup();
    controller.init_cam_extrinsics().unwrap();
    for (camera, pose) in controller
        .calib_opt
        .as_ref()
        .unwrap()
        .calib
        .as_ref()
        .unwrap()
        .t_i_c
        .iter()
        .enumerate()
    {
        compare_pose(*pose, &reference["extrinsics"][camera]);
    }
    controller.init_optimization().unwrap();
    for (i, time) in [100, 200, 300].into_iter().enumerate() {
        compare_pose(
            controller.calib_opt.as_ref().unwrap().get_t_w_i(time),
            &reference["poses"][i],
        );
    }
    for expected in reference["projections"].as_array().unwrap() {
        let id = TimeCamId::new(
            expected["time"].as_i64().unwrap(),
            expected["camera"].as_u64().unwrap() as usize,
        );
        let actual = &controller.reprojected_corners[&id];
        let valid: Vec<bool> = serde_json::from_value(expected["valid"].clone()).unwrap();
        assert_eq!(actual.corners_proj_success, valid);
        for (i, point) in actual.corners_proj.iter().enumerate() {
            for k in 0..2 {
                assert!((point[k] - expected["points"][i][k].as_f64().unwrap()).abs() < 1e-10);
            }
        }
    }
    let mut stats = BTreeMap::new();
    controller
        .optimize_with_param(false, Some(&mut stats))
        .unwrap();
    for (name, value) in stats {
        let expected = reference["stats"][&name].as_f64().unwrap();
        assert!(
            (value - expected).abs() < 1e-9 * (1. + expected.abs()),
            "{name}: {value} vs {expected}"
        );
    }
    let samples: usize = controller
        .angle_statistics
        .polar_count
        .iter()
        .flatten()
        .sum();
    assert_eq!(samples, 3 * 3 * 36);
    assert_eq!(
        samples,
        controller
            .angle_statistics
            .azimuth_count
            .iter()
            .flatten()
            .sum::<usize>()
    );
}

#[test]
fn original_controller_cache_and_image_subsampling() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let cache = Cache(root.join("../../logs").join(format!(
        "basalt-cam-calib-cache-test-{}",
        std::process::id()
    )));
    std::fs::create_dir_all(&cache.0).unwrap();
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("reference/calibration_archive.json")).unwrap();
    let decode = |key: &str| {
        fixture[key]
            .as_str()
            .unwrap()
            .as_bytes()
            .as_chunks::<2>()
            .0
            .iter()
            .map(|p| u8::from_str_radix(std::str::from_utf8(p).unwrap(), 16).unwrap())
            .collect::<Vec<_>>()
    };
    let mut corners = decode("corner_map");
    corners.extend_from_slice(&0_u64.to_ne_bytes());
    std::fs::write(cache.0.join("probe_detected_corners.cereal"), corners).unwrap();
    std::fs::write(cache.0.join("probe_init_poses.cereal"), decode("pose_map")).unwrap();
    let mut controller = CamCalib::new(
        &root.join("tests/reference/dataset_rosbag_none.bag"),
        "bag",
        &root.join("tests/reference/cam_calib_grid.json"),
        &cache.0,
        "probe",
        2,
        vec!["pinhole".into(); 2],
    )
    .unwrap();
    controller.load_dataset().unwrap();
    assert_eq!(
        controller.calib_corners[&TimeCamId::new(123, 1)].corner_ids,
        [-1, 321]
    );
    assert_eq!(
        controller.calib_init_poses[&TimeCamId::new(123, 1)].num_inliers,
        17
    );
    assert_eq!(
        *controller
            .vio_dataset
            .as_ref()
            .unwrap()
            .lock()
            .unwrap()
            .image_timestamps(),
        [100_000_000_000, 100_002_000_000]
    );
    assert!(controller.calib_corners_rejected.is_empty());
}
