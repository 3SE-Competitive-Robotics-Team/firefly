use firefly_basalt::spline::se3_spline::Se3Spline;
use firefly_basalt::utils::sophus_utils::so3_exp;
use firefly_basalt::{
    io::dataset_io::{AccelData, AprilgridCornersData, GyroData, MocapPoseData, PoseData},
    optimization::{
        accumulator::DenseAccumulator,
        linearize::CalibCommonData,
        spline_linearize::{ComputeErrorSplineOpt, LinearizeSplineOpt, SplineMeasurement},
    },
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::calibration::MocapCalibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use nalgebra::{DMatrix, Isometry3, Translation3, Vector2, Vector3, Vector4};
use serde_json::Value;
use std::collections::BTreeMap;
fn check(actual: f64, expected: f64, label: &str) {
    assert!(
        (actual - expected).abs() <= 1e-10 * (1. + expected.abs()),
        "{label}: {actual} != {expected}"
    );
}
fn compare<M: SplineMeasurement>(
    measurement: &M,
    spline: &Se3Spline<f64, 4>,
    common: CalibCommonData<'_>,
    fixture: &Value,
) {
    let mut linearization =
        LinearizeSplineOpt::<4, DenseAccumulator<f64>>::new(72, spline, common.clone());
    linearization.linearize(measurement);
    let mut error = ComputeErrorSplineOpt::new(72, spline, common);
    error.compute_error(measurement);
    let mut expected = DMatrix::zeros(72, 72);
    for entry in fixture["h"].as_array().unwrap() {
        expected[(
            entry[0].as_u64().unwrap() as usize,
            entry[1].as_u64().unwrap() as usize,
        )] = entry[2].as_f64().unwrap();
    }
    for row in 0..72 {
        for col in 0..72 {
            check(
                linearization.accum.h()[(row, col)],
                expected[(row, col)],
                &format!("H[{row},{col}]"),
            );
        }
        check(
            linearization.accum.b()[row],
            fixture["b"][row].as_f64().unwrap(),
            "b",
        );
    }
    check(
        linearization.stats.error,
        fixture["error"].as_f64().unwrap(),
        "error",
    );
    check(
        error.stats.error,
        fixture["computed_error"].as_f64().unwrap(),
        "computed error",
    );
    check(
        linearization.stats.reprojection_error,
        fixture["reprojection_error"].as_f64().unwrap(),
        "reprojection",
    );
    assert_eq!(
        linearization.stats.num_points as u64,
        fixture["num_points"].as_u64().unwrap()
    );
}
#[test]
fn all_five_measurement_families_match_original() {
    let fixture: Value =
        serde_json::from_str(include_str!("reference/spline_linearize.json")).unwrap();
    let mut spline = Se3Spline::new(20_000_000, 1_000_000_000);
    for i in 0..4 {
        let i = f64::from(i);
        spline.knots_push_back(&Isometry3::from_parts(
            Translation3::new(0.125 * i, 0.0625 * i * i, -0.03125 * i),
            so3_exp(&Vector3::new(0.025 * i, -0.015 * i * i, 0.01 * (i % 3.))),
        ));
    }
    let mut camera = GenericCamera::from_name("pinhole");
    camera.set_from_init(Vector4::new(320., 315., 320., 240.));
    let calibration = Calibration {
        intrinsics: vec![camera],
        t_i_c: vec![Isometry3::translation(0.125, 0., 0.)],
        ..Calibration::default()
    };
    let mocap = MocapCalibration {
        t_moc_w: Isometry3::translation(0.2, -0.1, 0.3),
        t_i_mark: Isometry3::translation(0.05, 0.1, 0.),
        ..MocapCalibration::default()
    };
    let corners = [
        Vector4::new(-0.25, -0.25, 3., 1.),
        Vector4::new(0.25, -0.25, 3., 1.),
        Vector4::new(0.25, 0.25, 3., 1.),
    ];
    let gravity = Vector3::new(0., 0., -9.81);
    let offsets = BTreeMap::new();
    let pose = PoseData {
        timestamp_ns: 1_007_000_000,
        data: Isometry3::from_parts(
            Translation3::new(0.2, 0.3, -0.1),
            so3_exp(&Vector3::new(0.1, -0.2, 0.3)),
        ),
    };
    let accel = AccelData {
        timestamp_ns: pose.timestamp_ns,
        data: Vector3::new(0.1, -0.2, 0.3),
    };
    let gyro = GyroData {
        timestamp_ns: pose.timestamp_ns,
        data: accel.data,
    };
    let marker = MocapPoseData {
        timestamp_ns: pose.timestamp_ns,
        data: pose.data,
    };
    let image = AprilgridCornersData {
        timestamp_ns: pose.timestamp_ns,
        cam_id: 0,
        corner_id: vec![0, 1, 2],
        corner_pos: vec![
            Vector2::new(300., 200.),
            Vector2::new(350., 205.),
            Vector2::new(355., 250.),
        ],
    };
    for case in fixture.as_array().unwrap() {
        let full = case["full"].as_bool().unwrap();
        let common = CalibCommonData {
            calibration: &calibration,
            mocap_calibration: Some(&mocap),
            aprilgrid_corner_pos_3d: &corners,
            offset_t_i_c: &[48],
            offset_intrinsics: &[54],
            mocap_block_offset: 58,
            bias_block_offset: 24,
            offset_poses: &offsets,
            g: Some(&gravity),
            opt_intrinsics: full,
            opt_cam_time_offset: full,
            opt_g: full,
            opt_imu_scale: full,
            pose_var_inv: 2.,
            gyro_var_inv: Vector3::new(5., 6., 7.),
            accel_var_inv: Vector3::new(2., 3., 4.),
            mocap_var_inv: 3.,
            huber_thresh: 2.,
        };
        compare(&pose, &spline, common.clone(), &case["pose"]);
        compare(&accel, &spline, common.clone(), &case["accel"]);
        compare(&gyro, &spline, common.clone(), &case["gyro"]);
        compare(&marker, &spline, common.clone(), &case["mocap"]);
        compare(&image, &spline, common, &case["aprilgrid"]);
    }
}
