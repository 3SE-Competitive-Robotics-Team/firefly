//! 已知加速、转向与双目解析投影，隔离图像匹配误差验证滤波器尺度。
use super::*;
use firefly_vio_core::{
    cam::{CamRadtan, SharedCamera},
    sensor::{CameraData, ImuData},
    track::HistogramMethod,
};
use firefly_vio_types::quat_ops::rot_2_quat;
use nalgebra::{Matrix3, UnitQuaternion};
use std::{collections::HashMap, sync::Arc};

/// 解析双目场景：2 相机、倾斜 20° 安装。返回管理器与 `R_ItoC`。
fn analytic_stereo_manager() -> (VioManager, Matrix3<f64>) {
    let intrinsics = [168.607, 168.607, 160., 120., 0., 0., 0., 0.];
    let cameras: BTreeMap<usize, SharedCamera> = (0..2)
        .map(|id| {
            (
                id,
                Arc::new(CamRadtan::new(320, 240, &intrinsics)) as SharedCamera,
            )
        })
        .collect();
    let tracker = TrackKlt::new(
        cameras
            .iter()
            .map(|(&id, c)| (id, c.clone()))
            .collect::<HashMap<_, _>>(),
        200,
        0,
        true,
        HistogramMethod::None,
        20,
        5,
        5,
        10,
    );
    let mut options = VioManagerOptions::default();
    options.state_options.num_cameras = 2;
    let mut manager = VioManager::new(options, cameras, tracker);
    let tilt = 20_f64.to_radians();
    let rc = Matrix3::new(
        0.,
        -1.,
        0.,
        -tilt.sin(),
        0.,
        -tilt.cos(),
        tilt.cos(),
        0.,
        -tilt.sin(),
    );
    for (id, y) in [(0, -0.025), (1, 0.025)] {
        let c = manager.state.calib_imu_to_cam.get_mut(&id).unwrap();
        c.set_value(rot_2_quat(&rc), -rc * Vector3::new(0., y, 0.));
        c.set_fej(rot_2_quat(&rc), -rc * Vector3::new(0., y, 0.));
    }
    (manager, rc)
}

/// 解析真值轨迹位置（m）。
fn trajectory_position(t: f64) -> Vector3<f64> {
    Vector3::new(
        0.1 * t * t,
        0.3 * (1. - (0.6 * t).cos()),
        1. + 0.1 * (0.4 * t).sin(),
    )
}

/// 解析真值轨迹速度（m/s）。
fn trajectory_velocity(t: f64) -> Vector3<f64> {
    Vector3::new(0.2 * t, 0.18 * (0.6 * t).sin(), 0.04 * (0.4 * t).cos())
}

/// 解析真值轨迹姿态。
fn trajectory_rotation(t: f64) -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.2 * (0.5 * t).sin())
}

/// 解析双目点云：8 层沿行进方向（x 间距 2m）+ 每层 20 点在 y/z 上以 0.15m
/// 密排，贴近真实特征密度。
fn analytic_points() -> Vec<Vector3<f64>> {
    (0..160)
        .map(|i| {
            let layer = i / 20;
            let rem = i % 20;
            let y = -0.3 + f64::from(rem % 5) * 0.15;
            let z = 0.6 + f64::from(rem / 5) * 0.15;
            Vector3::new(3.0 + f64::from(layer) * 2.0, y, z)
        })
        .collect()
}

/// 10 秒轨迹的运行结果。
struct TrajectoryOutcome {
    /// 最大位置误差（m）。
    peak: f64,
    /// 最大 SLAM 路标数。
    slam_peak: usize,
}

/// 跑 10 秒加速/转向轨迹（解析 IMU 与双目投影），返回观测量。
fn run_accelerated_trajectory(manager: &mut VioManager, rc: &Matrix3<f64>) -> TrajectoryOutcome {
    let mut initial = [0.; 17];
    initial[4] = 1.;
    initial[7] = 1.;
    initial[10] = 0.04;
    manager.initialize_with_gt(&initial);
    let points = analytic_points();
    let mut peak = 0_f64;
    let mut slam_peak = 0;
    for k in 0..=1000 {
        let t = f64::from(k) * 0.01;
        let r = trajectory_rotation(t);
        let accel = Vector3::new(0.2, 0.108 * (0.6 * t).cos(), -0.016 * (0.4 * t).sin());
        manager.feed_measurement_imu(&ImuData {
            timestamp: t,
            wm: Vector3::new(0., 0., 0.1 * (0.5 * t).cos()),
            am: r.inverse() * (accel + Vector3::new(0., 0., 9.81)),
        });
        if k == 0 || k % 10 != 0 {
            continue;
        }
        manager.propagate_and_clone(t);
        for (id, p) in points.iter().enumerate() {
            for (cam, y) in [(0, -0.025), (1, 0.025)] {
                let pc =
                    rc * (r.inverse() * (p - trajectory_position(t)) - Vector3::new(0., y, 0.));
                let (u, v) = (168.607 * pc.x / pc.z + 160., 168.607 * pc.y / pc.z + 120.);
                if pc.z > 0.5 && (4. ..316.).contains(&u) && (4. ..236.).contains(&v) {
                    manager.track_feats.database_mut().update_feature(
                        id,
                        t,
                        cam,
                        u as f32,
                        v as f32,
                        (pc.x / pc.z) as f32,
                        (pc.y / pc.z) as f32,
                    );
                }
            }
        }
        manager.do_feature_propagate_update(&CameraData {
            timestamp: t,
            sensor_ids: vec![0, 1],
            images: vec![],
            masks: vec![],
        });
        peak = peak.max((manager.state.imu.pos() - trajectory_position(t)).norm());
        slam_peak = slam_peak.max(manager.state.features_slam.len());
    }
    TrajectoryOutcome { peak, slam_peak }
}

/// 度量尺度契约：解析投影下位置/速度误差必须小于 1cm。
fn assert_metric_scale(manager: &VioManager, peak: f64, slam_peak: usize, tag: &str) {
    assert!(slam_peak > 0, "{tag}：SLAM 路标必须参与更新");
    assert!(peak < 0.01, "{tag}：解析投影位置误差={peak}m");
    assert!(
        (manager.state.imu.vel() - trajectory_velocity(10.)).norm() < 0.01,
        "{tag}：终点速度误差超限"
    );
    assert!(
        (manager.state.imu.pos().x - 10.).abs() < 0.01,
        "{tag}：终点 x 误差超限"
    );
}

#[test]
fn accelerated_ten_meter_stereo_has_metric_scale() {
    let (mut manager, rc) = analytic_stereo_manager();
    let outcome = run_accelerated_trajectory(&mut manager, &rc);
    assert_metric_scale(&manager, outcome.peak, outcome.slam_peak, "纯 MSCKF");
}

/// 小幅周期倾斜与平移下，精确观测不得产生虚假的加速度零偏。
#[test]
fn sustained_tilted_hover_preserves_zero_bias() {
    let (mut manager, rc) = analytic_stereo_manager();
    let mut initial = [0.; 17];
    initial[4] = 1.;
    initial[7] = 1.;
    manager.initialize_with_gt(&initial);
    let points = analytic_points();
    let mut peak_bias = 0_f64;
    let mut peak_tilt = 0_f64;
    let mut slam_peak = 0;
    for k in 0..=6000 {
        let t = f64::from(k) * 0.01;
        let angle = 0.08 * (0.7 * t).sin();
        let r = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);
        let position = Vector3::new(0.1 * (1. - (0.5 * t).cos()), 0., 1.);
        manager.feed_measurement_imu(&ImuData {
            timestamp: t,
            wm: Vector3::new(0.056 * (0.7 * t).cos(), 0., 0.),
            am: r.inverse() * Vector3::new(0.025 * (0.5 * t).cos(), 0., 9.81),
        });
        if k == 0 || k % 10 != 0 {
            continue;
        }
        manager.propagate_and_clone(t);
        for (id, p) in points.iter().enumerate() {
            for (cam, y) in [(0, -0.025), (1, 0.025)] {
                let pc = rc * (r.inverse() * (p - position) - Vector3::new(0., y, 0.));
                let (u, v) = (168.607 * pc.x / pc.z + 160., 168.607 * pc.y / pc.z + 120.);
                if pc.z > 0.5 && (4. ..316.).contains(&u) && (4. ..236.).contains(&v) {
                    manager.track_feats.database_mut().update_feature(
                        id,
                        t,
                        cam,
                        u as f32,
                        v as f32,
                        (pc.x / pc.z) as f32,
                        (pc.y / pc.z) as f32,
                    );
                }
            }
        }
        manager.do_feature_propagate_update(&CameraData {
            timestamp: t,
            sensor_ids: vec![0, 1],
            images: vec![],
            masks: vec![],
        });
        slam_peak = slam_peak.max(manager.state.features_slam.len());
        peak_bias = peak_bias.max(manager.state.imu.bias_a().norm());
        let estimated_gravity = manager.state.imu.pose().rot() * Vector3::z();
        peak_tilt = peak_tilt.max(estimated_gravity.angle(&(r.inverse() * Vector3::z())));
    }
    assert!(slam_peak > 0, "persistent landmarks must participate");
    assert!(peak_bias < 0.005, "peak accel bias {peak_bias}");
    assert!(peak_tilt < 0.001, "peak tilt {peak_tilt}");
}
