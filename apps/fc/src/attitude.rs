//! VIO 主姿态输入的时间、有效性与会话检查；不修正独立 ESKF。
use firefly_pubsub::attitude::AttitudeMessage;
use nalgebra::{UnitQuaternion, Vector3};
use std::time::Instant;

#[derive(Clone, Copy)]
pub struct Snapshot {
    pub message: AttitudeMessage,
    pub rotation: UnitQuaternion<f32>,
    pub rate: Vector3<f32>,
    received: Instant,
    posterior_received: Instant,
}

#[derive(Default)]
pub struct Input {
    pub latest: Option<Snapshot>,
    session: Option<u64>,
    session_fault: bool,
    valid: bool,
}

impl Input {
    pub fn observe(&mut self, m: AttitudeMessage, now: Instant) -> bool {
        if m.session == 0 || self.session_fault {
            return false;
        }
        if self.session.is_some_and(|s| s != m.session) {
            self.session_fault = true;
            self.valid = false;
            log::error!("VIO 会话改变，禁止恢复位置控制；需重启 FC");
            return false;
        }
        if self
            .latest
            .is_some_and(|s| m.timestamp <= s.message.timestamp)
        {
            return false;
        }
        let pose = firefly_base::RigidTransform::from_parts(
            firefly_base::FrameId::ODOM,
            firefly_base::FrameId::BODY,
            [0.; 3],
            m.quat_xyzw,
        );
        if !m.timestamp.is_finite()
            || !m.posterior_time.is_finite()
            || m.posterior_time < 0.
            || m.posterior_time > m.timestamp
            || self
                .latest
                .is_some_and(|s| m.posterior_time < s.message.posterior_time)
            || !m
                .angular_velocity
                .iter()
                .chain(m.gyro_bias.iter())
                .chain(m.attitude_variance.iter())
                .all(|x| (*x as f32).is_finite())
            || m.attitude_variance.iter().any(|x| *x < 0.)
            || pose.is_err()
        {
            self.valid = false;
            return false;
        }
        let posterior_received = self
            .latest
            .filter(|s| m.posterior_time <= s.message.posterior_time)
            .map_or(now, |s| s.posterior_received);
        self.latest = Some(Snapshot {
            message: m,
            rotation: pose.expect("validated pose").isometry().rotation.cast(),
            rate: Vector3::from(m.angular_velocity).cast(),
            received: now,
            posterior_received,
        });
        self.session = Some(m.session);
        self.valid = true;
        true
    }

    /// VIO 会话是否仍然有效（会话改变后不可恢复，需重启 FC）。
    #[must_use]
    pub fn session_ok(&self) -> bool {
        !self.session_fault
    }

    pub fn fresh(&self, imu_time: f64) -> bool {
        self.valid
            && !self.session_fault
            && self.latest.is_some_and(|s| {
                s.received.elapsed() <= super::IMU_STALE_LIMIT
                    && s.posterior_received.elapsed() <= super::ODOM_STALE_LIMIT
                    && (imu_time - s.message.timestamp).abs()
                        <= super::IMU_STALE_LIMIT.as_secs_f64()
                    && imu_time - s.message.posterior_time <= super::ODOM_STALE_LIMIT.as_secs_f64()
            })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Source {
    #[default]
    Unavailable,
    Vio,
    Standby,
}
impl Source {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Unavailable => "等待姿态",
            Self::Vio => "VIO 后验 IMU 预测",
            Self::Standby => "独立 ESKF 安全热备",
        }
    }
    pub const fn code(self) -> f64 {
        match self {
            Self::Unavailable => 0.,
            Self::Vio => 1.,
            Self::Standby => 2.,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn message(timestamp: f64) -> AttitudeMessage {
        AttitudeMessage {
            timestamp,
            posterior_time: timestamp,
            session: 7,
            quat_xyzw: [0., 0., 0., 1.],
            angular_velocity: [0.; 3],
            gyro_bias: [0.; 3],
            attitude_variance: [0.01; 3],
        }
    }

    #[test]
    fn duplicate_and_invalid_data_do_not_renew_health() {
        let mut input = Input::default();
        let old = Instant::now().checked_sub(Duration::from_secs(2)).unwrap();
        assert!(input.observe(message(1.), old));
        assert!(!input.observe(message(1.), Instant::now()));
        assert!(!input.fresh(1.));
        assert!(input.observe(message(2.), Instant::now()));
        assert!(input.fresh(2.));
        let mut bad = message(2.01);
        bad.angular_velocity[0] = f64::NAN;
        assert!(!input.observe(bad, Instant::now()));
        assert!(!input.fresh(2.01));
        assert!(input.observe(message(2.02), Instant::now()));
        assert!(!input.fresh(2.2));
    }

    #[test]
    fn prediction_cannot_hide_stale_posterior_or_new_session() {
        let mut input = Input::default();
        assert!(input.observe(
            message(1.),
            Instant::now().checked_sub(Duration::from_secs(2)).unwrap()
        ));
        let mut predicted = message(1.01);
        predicted.posterior_time = 1.;
        assert!(input.observe(predicted, Instant::now()));
        assert!(!input.fresh(1.01));
        assert!(input.observe(message(1.02), Instant::now()));
        assert!(input.fresh(1.02));
        predicted = message(2.);
        predicted.posterior_time = 1.02;
        assert!(input.observe(predicted, Instant::now()));
        assert!(!input.fresh(2.));
        let mut restarted = message(2.01);
        restarted.session = 8;
        assert!(!input.observe(restarted, Instant::now()));
        assert!(!input.observe(message(2.02), Instant::now()));
        assert!(!input.fresh(2.02));
    }
}
