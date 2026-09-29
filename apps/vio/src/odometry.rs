//! 在 IMU 时钟上预测里程计；位置与速度统一使用重力对齐局部世界系。

use firefly_pubsub::odom::OdomMessage;
use firefly_vio::vio_manager::VioManager;
use firefly_vio_types::quat_ops::quat_2_rot;

/// 测量覆盖不足或估计器未就绪时不生成输出。
pub fn sample(vio: &mut VioManager, timestamp: f64) -> Option<OdomMessage> {
    if !vio.initialized() || !timestamp.is_finite() {
        return None;
    }
    if timestamp > vio.propagator.latest_imu_timestamp()? {
        return None;
    }
    let state_time = vio.state.timestamp + vio.time_offset();
    let (q, p, v) = if (timestamp - state_time).abs() < 1e-9 {
        (
            vio.state.imu.quat(),
            vio.state.imu.pos(),
            vio.state.imu.vel(),
        )
    } else {
        let predicted = vio.fast_state_propagate(timestamp)?;
        (
            predicted.q,
            predicted.p,
            quat_2_rot(&predicted.q).transpose() * predicted.v,
        )
    };
    Some(OdomMessage {
        timestamp,
        position_x: p.x,
        position_y: p.y,
        position_z: p.z,
        velocity_x: v.x,
        velocity_y: v.y,
        velocity_z: v.z,
        quat_x: q[0],
        quat_y: q[1],
        quat_z: q[2],
        quat_w: q[3],
        is_initialized: true,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_vio::options::VioManagerOptions;
    use firefly_vio_core::{
        sensor::ImuData,
        track::{HistogramMethod, TrackKlt},
    };
    use nalgebra::{DVector, Vector3};
    use std::collections::{BTreeMap, HashMap};

    fn moving_estimator(offset: f64) -> VioManager {
        let mut options = VioManagerOptions::default();
        options.state_options.do_calib_camera_timeoffset = true;
        let tracker = TrackKlt::new(
            HashMap::new(),
            100,
            0,
            false,
            HistogramMethod::None,
            10,
            5,
            5,
            15,
        );
        let mut vio = VioManager::new(options, BTreeMap::new(), tracker);
        vio.state
            .calib_dt_cam_to_imu
            .as_mut()
            .unwrap()
            .set_value(DVector::from_element(1, offset));
        let mut initial = [0.0; 17];
        initial[0] = 1.0;
        initial[3] = std::f64::consts::FRAC_1_SQRT_2;
        initial[4] = std::f64::consts::FRAC_1_SQRT_2;
        initial[8] = 1.0;
        vio.initialize_with_gt(&initial);
        vio.timelastupdate = 1.0;
        for i in 90..=130 {
            vio.feed_measurement_imu(&ImuData {
                timestamp: f64::from(i) * 0.01,
                wm: Vector3::zeros(),
                am: Vector3::new(0.0, 0.0, 9.81),
            });
        }
        vio
    }

    #[test]
    fn prediction_preserves_timestamp_world_velocity_and_filter_state() {
        for offset in [-0.02, 0.0, 0.02] {
            let mut vio = moving_estimator(offset);
            let before = vio.state.cov.clone();
            let at_state = sample(&mut vio, 1.0 + offset).unwrap();
            assert!(at_state.position_x.abs() < 1e-12);
            for t in [1.1, 1.2, 1.3] {
                let msg = sample(&mut vio, t).unwrap();
                assert!((msg.timestamp - t).abs() < 1e-12);
                assert!((msg.position_x - (t - 1.0 - offset)).abs() < 1e-10);
                assert!((msg.velocity_x - 1.0).abs() < 1e-10);
                assert!(msg.velocity_y.abs() < 1e-10);
            }
            assert!((vio.state.timestamp - 1.0).abs() < 1e-12);
            assert_eq!(vio.state.cov, before);
            assert!(vio.state.clones_imu.is_empty());
        }
    }

    #[test]
    fn missing_imu_and_uninitialized_state_do_not_produce_fresh_odometry() {
        let mut vio = moving_estimator(0.02);
        assert!(sample(&mut vio, 0.95).is_none());
        assert!(sample(&mut vio, 1.31).is_none());
        assert!(sample(&mut vio, 1.1).is_some());
        // 相机推进而 IMU 停止时，即使滤波状态已外推，也不能伪装测量覆盖。
        vio.state.timestamp = 1.4;
        assert!(sample(&mut vio, 1.42).is_none());
        vio.is_initialized_vio = false;
        assert!(sample(&mut vio, 1.2).is_none());
    }

    #[test]
    fn first_and_subsequent_propagations_respect_time_offset() {
        for offset in [-0.02, 0.02] {
            let mut vio = moving_estimator(offset);
            vio.propagate_to(1.1);
            assert!((vio.state.imu.pos().x - 0.1).abs() < 1e-10);
            // 在线标定改变偏移时，起点必须保留上次传播的时钟对应关系。
            vio.state
                .calib_dt_cam_to_imu
                .as_mut()
                .unwrap()
                .set_value(DVector::from_element(1, offset + 0.01));
            vio.propagate_to(1.2);
            assert!((vio.state.imu.pos().x - 0.21).abs() < 1e-10);
            let predicted = sample(&mut vio, 1.25).unwrap();
            assert!((predicted.position_x - (0.25 - offset)).abs() < 1e-10);
        }
    }
}
