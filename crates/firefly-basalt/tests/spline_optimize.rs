use firefly_basalt::optimization::{
    linearize::ReprojectionStats,
    spline_optimize::{Options, SplineOptimization},
};
use firefly_basalt_headers::utils::sophus_utils::so3_exp;
use nalgebra::{Isometry3, Translation3, Vector3};
use serde_json::Value;
fn check(actual: f64, expected: f64, label: &str) {
    assert!(
        (actual - expected).abs() <= 1e-10 * (1. + expected.abs()),
        "{label}: {actual} != {expected}"
    );
}
#[test]
fn sparse_lm_iterations_match_original() {
    let fixture: Value =
        serde_json::from_str(include_str!("reference/spline_optimize.json")).unwrap();
    let mut optimizer = SplineOptimization::<4>::new(20_000_000, 1e-3);
    optimizer.reset_calib(1, &["pinhole".to_owned()]);
    let calibration = optimizer.calib.as_mut().unwrap();
    calibration.imu_update_rate = 200.;
    calibration.gyro_noise_std.fill(0.01);
    calibration.accel_noise_std.fill(0.1);
    optimizer.init_spline(&Isometry3::identity(), 4);
    optimizer.set_g(Vector3::new(0., 0., -9.81));
    for (index, time) in [0, 7_000_000, 19_000_000].into_iter().enumerate() {
        let index = index as f64;
        optimizer.add_pose_measurement(
            time,
            Isometry3::from_parts(
                Translation3::new(0.01 * index, -0.02 * index, 0.005 * index),
                so3_exp(&Vector3::new(0.001 * index, -0.002 * index, 0.003 * index)),
            ),
        );
        optimizer.add_accel_measurement(time, Vector3::new(0., 0., -9.81));
        optimizer.add_gyro_measurement(time, Vector3::new(0.01, -0.02, 0.03));
    }
    optimizer.init();
    let options = Options {
        use_intrinsics: false,
        use_poses: true,
        use_april_corners: false,
        opt_cam_time_offset: false,
        opt_imu_scale: false,
        use_mocap: false,
        huber_thresh: 2.,
        stop_thresh: 1e-12,
        print_info: false,
    };
    for case in fixture.as_array().unwrap() {
        let mut stats = ReprojectionStats::default();
        assert_eq!(
            optimizer.optimize(options, &mut stats),
            case["converged"].as_bool().unwrap()
        );
        check(stats.error, case["error"].as_f64().unwrap(), "error");
        check(
            stats.reprojection_error,
            case["reprojection"].as_f64().unwrap(),
            "reprojection",
        );
        assert_eq!(stats.num_points as u64, case["points"].as_u64().unwrap());
        for i in 0..4 {
            let pose = optimizer.spline().knot(i).to_homogeneous();
            for row in 0..4 {
                for col in 0..4 {
                    check(
                        pose[(row, col)],
                        case["knots"][i][row * 4 + col].as_f64().unwrap(),
                        "knot",
                    );
                }
            }
        }
        for i in 0..3 {
            check(
                optimizer.g()[i],
                case["gravity"][i].as_f64().unwrap(),
                "gravity",
            );
        }
        for i in 0..9 {
            check(
                optimizer.accel_bias().param()[i],
                case["accel_bias"][i].as_f64().unwrap(),
                "accel bias",
            );
        }
        for i in 0..12 {
            check(
                optimizer.gyro_bias().param()[i],
                case["gyro_bias"][i].as_f64().unwrap(),
                "gyro bias",
            );
        }
    }
}
