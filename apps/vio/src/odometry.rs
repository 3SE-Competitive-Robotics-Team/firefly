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

/// 原始滤波状态的六维边缘分布；与预测里程计分开，禁止给旧协方差贴新时间戳。
/// `OpenVINS` JPL 左误差的 δθ 等于 Hamilton body→odom 的右误差，bg 为机体系加性误差。
pub fn attitude_aid(
    vio: &VioManager,
    session: u64,
) -> Option<firefly_pubsub::attitude::AttitudeAidMessage> {
    use firefly_vio_types::var::Variable;
    if !vio.initialized() || session == 0 {
        return None;
    }
    let timestamp = vio.state_imu_timestamp();
    if !timestamp.is_finite() || timestamp > vio.propagator.latest_imu_timestamp()? {
        return None;
    }
    let id = usize::try_from(vio.state.imu.id()).ok()?;
    let indices = [id, id + 1, id + 2, id + 9, id + 10, id + 11];
    let q = vio.state.imu.quat();
    let bg = vio.state.imu.bias_g();
    Some(firefly_pubsub::attitude::AttitudeAidMessage {
        timestamp,
        session,
        quat_xyzw: [q[0], q[1], q[2], q[3]],
        gyro_bias: [bg.x, bg.y, bg.z],
        covariance: std::array::from_fn(|k| vio.state.cov[(indices[k / 6], indices[k % 6])]),
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
    fn attitude_aid_preserves_state_clock_and_covariance_cross_terms() {
        use firefly_vio_types::var::Variable;
        let mut vio = moving_estimator(0.02);
        vio.propagate_to(1.1);
        let id = vio.state.imu.id() as usize;
        vio.state.cov[(id, id + 10)] = 2e-6;
        vio.state.cov[(id + 10, id)] = 2e-6;
        let aid = attitude_aid(&vio, 42).unwrap();
        assert!((aid.timestamp - 1.12).abs() < 1e-12);
        assert_eq!(aid.session, 42);
        assert!((aid.covariance[4] - 2e-6).abs() < 1e-15);
        assert!((aid.covariance[24] - 2e-6).abs() < 1e-15);
        vio.state
            .calib_dt_cam_to_imu
            .as_mut()
            .unwrap()
            .set_value(DVector::from_element(1, 0.04));
        assert!((attitude_aid(&vio, 42).unwrap().timestamp - 1.12).abs() < 1e-12);
        assert!(attitude_aid(&vio, 0).is_none());
    }

    #[test]
    fn jpl_error_is_hamilton_right_error_without_sign_or_covariance_flip() {
        use firefly_vio_types::var::{JplQuat, Variable};
        use nalgebra::{Quaternion, UnitQuaternion};
        let nominal = UnitQuaternion::from_euler_angles(0.3, -0.2, 0.8);
        let q = nominal.quaternion();
        let mut jpl = JplQuat::default();
        jpl.set_value(nalgebra::Vector4::new(q.i, q.j, q.k, q.w));
        let delta = Vector3::new(1e-5, -2e-5, 3e-5);
        jpl.update(&DVector::from_column_slice(delta.as_slice()));
        let updated = jpl.value();
        let hamilton = UnitQuaternion::new_normalize(Quaternion::new(
            updated[3], updated[0], updated[1], updated[2],
        ));
        let expected = nominal * UnitQuaternion::from_scaled_axis(delta);
        assert!((hamilton.inverse() * expected).angle() < 1e-12);
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
