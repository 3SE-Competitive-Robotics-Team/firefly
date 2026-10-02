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

#[test]
fn accelerated_ten_meter_stereo_has_metric_scale() {
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
    let position = |t: f64| {
        Vector3::new(
            0.1 * t * t,
            0.3 * (1. - (0.6 * t).cos()),
            1. + 0.1 * (0.4 * t).sin(),
        )
    };
    let velocity = |t: f64| Vector3::new(0.2 * t, 0.18 * (0.6 * t).sin(), 0.04 * (0.4 * t).cos());
    let rotation =
        |t: f64| UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.2 * (0.5 * t).sin());
    let mut initial = [0.; 17];
    initial[4] = 1.;
    initial[7] = 1.;
    initial[10] = 0.04;
    manager.initialize_with_gt(&initial);
    let points: Vec<_> = (0..160)
        .map(|i| {
            Vector3::new(
                3. + (i % 16) as f64,
                -3. + (i / 16) as f64 * 0.65,
                0.2 + (i % 7) as f64 * 0.5,
            )
        })
        .collect();
    let mut peak = 0_f64;
    let mut slam_peak = 0;
    for k in 0..=1000 {
        let t = k as f64 * 0.01;
        let r = rotation(t);
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
                let pc = rc * (r.inverse() * (p - position(t)) - Vector3::new(0., y, 0.));
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
        peak = peak.max((manager.state.imu.pos() - position(t)).norm());
        slam_peak = slam_peak.max(manager.state.features_slam.len());
    }
    assert!(slam_peak > 0, "visual landmark updates must participate");
    assert!(peak < 0.01, "analytic projection position error={peak}m");
    assert!((manager.state.imu.vel() - velocity(10.)).norm() < 0.01);
    assert!((manager.state.imu.pos().x - 10.).abs() < 0.01);
}
