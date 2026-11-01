use firefly_basalt::calibration::calibration_helper::CalibHelper;
use firefly_calibration::aprilgrid::AprilGrid;
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::camera::pinhole_camera::PinholeCamera;
use firefly_calibration::observation::{
    CalibCornerData, CalibCornerMap, CalibInitPoseMap, TimeCamId,
};
use nalgebra::{Isometry3, Vector2, Vector3, Vector4};

#[derive(Default)]
struct TestDataset {
    times: Vec<i64>,
}
impl firefly_basalt::io::dataset_io::VioDataset for TestDataset {
    fn num_cams(&self) -> usize {
        2
    }
    fn image_timestamps(&mut self) -> &mut Vec<i64> {
        &mut self.times
    }
    fn accel_data(&self) -> &[firefly_basalt::io::dataset_io::AccelData] {
        &[]
    }
    fn gyro_data(&self) -> &[firefly_basalt::io::dataset_io::GyroData] {
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
    fn image_data(
        &mut self,
        _: i64,
    ) -> firefly_error::Result<Vec<firefly_basalt::io::dataset_io::ImageData>> {
        Ok(Vec::new())
    }
}

#[test]
fn original_aprilgrid_and_vignette_reference() {
    use firefly_basalt::{calibration::vignette::VignetteEstimator, io::dataset_io::VioDatasetPtr};
    use std::{
        collections::BTreeMap,
        sync::{Arc, Mutex},
    };
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("reference/calibration.json")).unwrap();
    let grid = AprilGrid::from_parameters(2, 1, 0.04, 0.25).unwrap();
    for (key, points) in [
        ("corners", &grid.aprilgrid_corner_pos_3d),
        ("vignette", &grid.aprilgrid_vignette_pos_3d),
    ] {
        for (i, point) in points.iter().enumerate() {
            for k in 0..4 {
                assert!((point[k] - fixture[key][i][k].as_f64().unwrap()).abs() < 1e-15);
            }
        }
    }
    let centers = [Vector2::new(20., 15.); 2];
    let resolutions = [Vector2::new(40, 30); 2];
    let mut data = BTreeMap::new();
    for frame in 0..3 {
        for camera in 0..2 {
            let points = (0..grid.aprilgrid_vignette_pos_3d.len())
                .map(|i| {
                    let radius = (2 + (i + frame * 3 + camera) % 17) as f64;
                    Vector3::new(
                        20. + radius * 0.6,
                        15. + radius * 0.8,
                        if i % 9 == 0 {
                            -1.
                        } else {
                            100. + i as f64 * 2. - frame as f64 * 3. - camera as f64
                        },
                    )
                })
                .collect();
            data.insert(
                TimeCamId {
                    frame_id: frame as i64,
                    cam_id: camera,
                },
                points,
            );
        }
    }
    let dataset: VioDatasetPtr = Arc::new(Mutex::new(TestDataset::default()));
    let mut estimator = VignetteEstimator::new(&dataset, &centers, &resolutions, &data, &grid);
    for stage in 0..4 {
        match stage {
            1 => estimator.opt_irradience(),
            2 => estimator.opt_vign(),
            3 => estimator.optimize(),
            _ => {}
        }
        let expected = &fixture["stages"][stage];
        for (camera, spline) in estimator.vign_param().iter().enumerate() {
            for (k, knot) in spline.knots().iter().enumerate() {
                assert!((knot[0] - expected["knots"][camera][k].as_f64().unwrap()).abs() < 1e-12);
            }
        }
        let mut errors = BTreeMap::new();
        estimator.compute_error(Some(&mut errors));
        for (i, values) in errors.values().enumerate() {
            for (k, value) in values.iter().enumerate() {
                assert!((value - expected["errors"][i][2][k].as_f64().unwrap()).abs() < 1e-10);
            }
        }
        for (i, values) in estimator.compute_data_log().iter().enumerate() {
            for (k, value) in values.iter().enumerate() {
                assert_eq!(*value, expected["data_log"][i][k].as_f64().unwrap() as f32);
            }
        }
    }
}

#[test]
fn aprilgrid_corner_order_and_vignette_geometry() {
    let grid = AprilGrid::from_parameters(3, 2, 0.04, 0.25).unwrap();
    assert_eq!(grid.aprilgrid_corner_pos_3d.len(), 24);
    assert_eq!(grid.aprilgrid_vignette_pos_3d.len(), 85);
    for (id, point) in grid.aprilgrid_corner_pos_3d.iter().enumerate() {
        let tag = id / 4;
        let corner = id % 4;
        let expected = Vector4::new(
            (tag % 3) as f64 * 0.05 + [0., 0.04, 0.04, 0.][corner],
            (tag / 3) as f64 * 0.05 + [0., 0., 0.04, 0.04][corner],
            0.,
            1.,
        );
        assert!((point - expected).amax() < 1e-15);
    }
    assert!(
        (grid.aprilgrid_vignette_pos_3d[0] - Vector4::new(0.045, 0.04 / 6., 0., 1.)).amax() < 1e-15
    );
    assert!(AprilGrid::from_parameters(0, 2, 0.04, 0.25).is_err());
}

#[test]
fn pinhole_initialization_and_kneip_target_pose() {
    let grid = AprilGrid::from_parameters(6, 6, 0.04, 0.25).unwrap();
    let parameters = Vector4::new(350., 370., 319.5, 239.5);
    let camera = GenericCamera::Pinhole(PinholeCamera::new(parameters));
    let poses = [
        Isometry3::new(Vector3::new(-0.1, -0.1, 0.9), Vector3::new(0.3, -0.4, 0.1)),
        Isometry3::new(
            Vector3::new(-0.1, -0.15, 0.7),
            Vector3::new(-0.35, 0.15, 0.2),
        ),
        Isometry3::new(Vector3::new(-0.15, -0.1, 1.1), Vector3::new(0.2, 0.3, -0.1)),
    ];
    let views: Vec<_> = poses
        .iter()
        .map(|pose| CalibCornerData {
            corners: grid
                .aprilgrid_corner_pos_3d
                .iter()
                .map(|point| {
                    let mut pixel = Vector2::zeros();
                    assert!(camera.project(&(pose.to_homogeneous() * point), &mut pixel, None));
                    pixel
                })
                .collect(),
            corner_ids: (0..grid.aprilgrid_corner_pos_3d.len() as i32).collect(),
            radii: Vec::new(),
        })
        .collect();
    let mut estimated = Vector4::zeros();
    assert!(
        CalibHelper::initialize_intrinsics_pinhole(&views, &grid, 640, 480, &mut estimated)
            .unwrap()
    );
    assert!((estimated - parameters).amax() < 1e-3, "{estimated:?}");
    let calibration = Calibration {
        intrinsics: vec![camera],
        ..Default::default()
    };
    let observations: CalibCornerMap = views
        .into_iter()
        .enumerate()
        .map(|(i, view)| {
            (
                TimeCamId {
                    frame_id: i as i64,
                    cam_id: 0,
                },
                view,
            )
        })
        .collect();
    let mut result = CalibInitPoseMap::new();
    CalibHelper::init_cam_poses(
        &calibration,
        &grid.aprilgrid_corner_pos_3d,
        &observations,
        &mut result,
    );
    for (i, pose) in poses.iter().enumerate() {
        let estimated = &result[&TimeCamId {
            frame_id: i as i64,
            cam_id: 0,
        }];
        assert_eq!(estimated.num_inliers, grid.aprilgrid_corner_pos_3d.len());
        assert!((estimated.t_a_c.to_homogeneous() - pose.inverse().to_homogeneous()).amax() < 1e-6);
    }
}
