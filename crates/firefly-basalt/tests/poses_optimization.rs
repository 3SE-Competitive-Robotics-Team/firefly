//! Original C++ calibration normal blocks and nonlinear geometric regression.
use firefly_basalt::{
    io::dataset_io::AprilgridCornersData,
    optimization::{
        linearize::{CalibCommonData, ReprojectionStats},
        poses_linearize::LinearizePosesOpt,
        poses_optimize::PosesOptimization,
    },
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use nalgebra::{Isometry3, Vector2, Vector3, Vector4};
use std::collections::BTreeMap;

fn calibration() -> Calibration<f64> {
    Calibration {
        intrinsics: ["pinhole", "ds"]
            .into_iter()
            .map(|name| {
                let mut c = GenericCamera::from_name(name);
                c.set_from_init(Vector4::new(200.0, 205.0, 160.0, 120.0));
                c
            })
            .collect(),
        t_i_c: vec![
            Isometry3::identity(),
            Isometry3::translation(0.12, 0.0, 0.0),
        ],
        ..Default::default()
    }
}
fn points() -> Vec<Vector4<f64>> {
    (0..12)
        .map(|i| {
            Vector4::new(
                (f64::from(i % 4) - 1.5) * 0.3,
                (f64::from(i / 4) - 1.0) * 0.25,
                3.0 + 0.02 * f64::from(i),
                1.0,
            )
        })
        .collect()
}
fn poses() -> BTreeMap<i64, Isometry3<f64>> {
    BTreeMap::from([
        (10, Isometry3::translation(0.01, 0.02, 0.0)),
        (20, Isometry3::translation(0.15, -0.02, 0.03)),
    ])
}
fn observations(
    calibration: &Calibration<f64>,
    poses: &BTreeMap<i64, Isometry3<f64>>,
    points: &[Vector4<f64>],
    noise: bool,
) -> Vec<AprilgridCornersData> {
    let mut result = Vec::new();
    for (&timestamp_ns, pose) in poses {
        for camera in 0..2 {
            let transform = (pose * calibration.t_i_c[camera])
                .inverse()
                .to_homogeneous();
            let corner_pos = points
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    let mut pixel = Vector2::zeros();
                    assert!(calibration.intrinsics[camera].project(
                        &(transform * p),
                        &mut pixel,
                        None
                    ));
                    if noise {
                        pixel += Vector2::new(if i == 0 { 4.0 } else { 0.15 }, -0.1);
                    }
                    pixel
                })
                .collect();
            result.push(AprilgridCornersData {
                timestamp_ns,
                cam_id: i32::try_from(camera).unwrap(),
                corner_pos,
                corner_id: (0..12).collect(),
            });
        }
    }
    result
}

#[test]
fn calibration_blocks_match_original_cpp_for_two_camera_models_and_huber_weights() {
    let cases: serde_json::Value =
        serde_json::from_str(include_str!("reference/poses_optimization.json")).unwrap();
    let calibration = calibration();
    let poses = poses();
    let points = points();
    let observations = observations(&calibration, &poses, &points, true);
    let pose_offsets = BTreeMap::from([(10, 0), (20, 6)]);
    for case in cases.as_array().unwrap() {
        let common = CalibCommonData {
            calibration: &calibration,
            mocap_calibration: None,
            aprilgrid_corner_pos_3d: &points,
            offset_t_i_c: &[12, 18, 24],
            offset_intrinsics: &[24, 28, 34],
            mocap_block_offset: 0,
            bias_block_offset: 0,
            offset_poses: &pose_offsets,
            g: None,
            opt_intrinsics: case["opt_intrinsics"].as_bool().unwrap(),
            opt_cam_time_offset: false,
            opt_g: false,
            opt_imu_scale: false,
            pose_var_inv: 0.0,
            gyro_var_inv: Vector3::zeros(),
            accel_var_inv: Vector3::zeros(),
            mocap_var_inv: 0.0,
            huber_thresh: 1.0,
        };
        let mut lin = LinearizePosesOpt::new(34, &poses, common);
        for observation in &observations {
            lin.linearize(observation);
        }
        lin.accum.setup_solver();
        let near = |a: f64, b: f64| assert!((a - b).abs() <= 1e-10 * (1.0 + b.abs()), "{a} != {b}");
        near(lin.stats.error, case["error"].as_f64().unwrap());
        near(
            lin.stats.reprojection_error,
            case["reprojection_error"].as_f64().unwrap(),
        );
        assert_eq!(
            lin.stats.num_points,
            usize::try_from(case["num_points"].as_u64().unwrap()).unwrap()
        );
        for r in 0..34 {
            near(lin.accum.b()[r], case["b"][r].as_f64().unwrap());
            for c in 0..34 {
                near(
                    lin.accum.sparse_matrix().columns[c]
                        .get(&r)
                        .copied()
                        .unwrap_or(0.0),
                    case["h"][r * 34 + c].as_f64().unwrap(),
                );
            }
        }
    }
}

#[test]
fn pose_calibration_reduces_reprojection_error_and_holds_camera_zero_fixed() {
    let calibration = calibration();
    let truth = poses();
    let points = points();
    let observations = observations(&calibration, &truth, &points, false);
    let fixed_camera = calibration.intrinsics[0].param();
    let fixed_extrinsic = calibration.t_i_c[0];
    let mut optimizer = PosesOptimization::new();
    optimizer.calib = Some(calibration);
    optimizer.set_aprilgrid_corners_3d(&points);
    for (time, mut pose) in truth {
        pose.translation.vector += Vector3::new(0.025, -0.03, 0.01);
        optimizer.add_pose_measurement(time, pose);
    }
    for obs in observations {
        optimizer.add_aprilgrid_measurement(
            obs.timestamp_ns,
            obs.cam_id,
            &obs.corner_pos,
            &obs.corner_id,
        );
    }
    optimizer.init();
    let mut stats = ReprojectionStats::default();
    for _ in 0..10 {
        if optimizer.optimize(false, 1.0, 1e-10, &mut stats) {
            break;
        }
    }
    assert!(
        stats.error < 1e-8,
        "remaining reprojection cost {}",
        stats.error
    );
    assert_eq!(
        optimizer.calib.as_ref().unwrap().intrinsics[0].param(),
        fixed_camera
    );
    assert_eq!(optimizer.calib.as_ref().unwrap().t_i_c[0], fixed_extrinsic);
}
