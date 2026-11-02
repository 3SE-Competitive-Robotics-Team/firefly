//! VIO 主姿态输入的时间、有效性与会话检查；不修正独立 ESKF。
//!
//! VIO 送来的角速度是未滤波的原始陀螺：旋翼与机体振动会原样进速率环，把力矩
//! 需求顶到混控器满量程之外触发饱和，饱和再经 airmode 解饱和变成推力尖峰。
//! 这里对测量角速度做一阶低通与幅值上限；滤波器按**测量自身的时间戳**推进——
//! 测量变慢时滤波器不会跟着变快，否则截止频率会被悄悄改掉。
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

/// 测量角速度一阶低通与幅值上限；截止频率非正时直通。
struct RateFilter {
    cutoff_hz: f32,
    limit_rad_s: f32,
    state: Option<Vector3<f32>>,
    last_stamp: f64,
}

impl Default for RateFilter {
    fn default() -> Self {
        Self {
            cutoff_hz: 0.,
            limit_rad_s: 0.,
            state: None,
            last_stamp: 0.,
        }
    }
}

impl RateFilter {
    fn update(&mut self, rate: Vector3<f32>, timestamp: f64) -> Vector3<f32> {
        let rate = self.limit(rate);
        if !self.cutoff_hz.is_finite() || self.cutoff_hz <= 0. {
            return rate;
        }
        let Some(previous) = self.state else {
            self.state = Some(rate);
            self.last_stamp = timestamp;
            return rate;
        };
        let elapsed = timestamp - self.last_stamp;
        if !elapsed.is_finite() || elapsed <= 0. {
            // 时间戳没有前进：保持状态，不推进滤波器。
            return previous;
        }
        // 采样间隔异常拉长时按上限步进，避免一次大跳把状态冲掉。
        let dt = elapsed.min(0.05) as f32;
        self.last_stamp = timestamp;
        // 一阶低通的精确离散化：alpha = 1 − e^{−2π·fc·dt}。
        let alpha = 1. - (-2. * std::f32::consts::PI * self.cutoff_hz * dt).exp();
        let filtered = previous + alpha * (rate - previous);
        self.state = Some(filtered);
        filtered
    }

    /// 幅值上限：单点野值不得原样变成力矩指令。
    fn limit(&self, rate: Vector3<f32>) -> Vector3<f32> {
        if !self.limit_rad_s.is_finite() || self.limit_rad_s <= 0. {
            return rate;
        }
        let norm = rate.norm();
        if norm > self.limit_rad_s {
            rate * (self.limit_rad_s / norm)
        } else {
            rate
        }
    }
}

#[derive(Default)]
pub struct Input {
    pub latest: Option<Snapshot>,
    session: Option<u64>,
    session_fault: bool,
    valid: bool,
    rate_filter: RateFilter,
}

impl Input {
    /// 设置测量角速度的低通截止频率（Hz）与幅值上限（rad/s）。
    ///
    /// 截止频率非正表示不滤波，幅值上限非正表示不限幅。两者都作用在 VIO 送来的
    /// 测量上，独立 ESKF 的角速度自带平滑，不经这里。
    pub fn set_rate_measurement_filter(&mut self, cutoff_hz: f32, limit_rad_s: f32) {
        self.rate_filter.cutoff_hz = cutoff_hz;
        self.rate_filter.limit_rad_s = limit_rad_s;
    }

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
                .all(|x| (*x as f32).is_finite())
            || pose.is_err()
        {
            self.valid = false;
            return false;
        }
        let posterior_received = self
            .latest
            .filter(|s| m.posterior_time <= s.message.posterior_time)
            .map_or(now, |s| s.posterior_received);
        let rate = self
            .rate_filter
            .update(Vector3::from(m.angular_velocity).cast(), m.timestamp);
        self.latest = Some(Snapshot {
            message: m,
            rotation: pose.expect("validated pose").isometry().rotation.cast(),
            rate,
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
        }
    }

    fn message_with_rate(timestamp: f64, rate: [f64; 3]) -> AttitudeMessage {
        AttitudeMessage {
            angular_velocity: rate,
            ..message(timestamp)
        }
    }

    /// 送入一串正弦测量，返回滤波后序列的稳态幅值。
    fn filtered_amplitude(cutoff_hz: f32, hz: f64) -> f32 {
        let mut input = Input::default();
        input.set_rate_measurement_filter(cutoff_hz, 0.);
        let rate = 1e-3;
        let mut peak = 0f32;
        for index in 0..2000i32 {
            let t = f64::from(index) * rate;
            let value = (2. * std::f64::consts::PI * hz * t).sin();
            assert!(input.observe(message_with_rate(t, [value, 0., 0.]), Instant::now()));
            if index > 1000 {
                peak = peak.max(input.latest.expect("snapshot").rate.x.abs());
            }
        }
        peak
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

    #[test]
    fn the_measurement_lowpass_attenuates_the_rotor_band() {
        // 旋翼与机体振动集中在 20~30 Hz，一阶低通在 10 Hz 截止时必须把它压下去。
        let vibration = filtered_amplitude(10., 25.);
        assert!(vibration < 0.5, "25 Hz 幅值未压下去：{vibration}");
        assert!(vibration > 0.1, "25 Hz 被整段滤掉：{vibration}");
    }

    #[test]
    fn the_measurement_lowpass_keeps_the_flight_band() {
        // 真实运动在 0~5 Hz，10 Hz 截止必须几乎无损地通过。
        let flight = filtered_amplitude(10., 1.);
        assert!(flight > 0.9, "1 Hz 幅值被削：{flight}");
        assert!(flight < 1.05, "1 Hz 幅值被放大：{flight}");
    }

    #[test]
    fn a_non_positive_cutoff_passes_the_measurement_through() {
        let mut input = Input::default();
        input.set_rate_measurement_filter(0., 0.);
        assert!(input.observe(message_with_rate(1., [0.5, -0.25, 0.]), Instant::now()));
        let rate = input.latest.expect("snapshot").rate;
        assert!((rate.x - 0.5).abs() < 1e-6);
        assert!((rate.y + 0.25).abs() < 1e-6);
    }

    #[test]
    fn the_amplitude_limit_bounds_a_single_wild_sample() {
        let mut input = Input::default();
        input.set_rate_measurement_filter(0., 20.);
        assert!(input.observe(message_with_rate(1., [100., 0., 0.]), Instant::now()));
        let rate = input.latest.expect("snapshot").rate;
        assert!((rate.x - 20.).abs() < 1e-4, "{rate:?}");
        assert!(rate.norm() <= 20. + 1e-4);
    }

    #[test]
    fn a_timestamp_gap_keeps_the_filter_state_continuous() {
        let mut input = Input::default();
        input.set_rate_measurement_filter(10., 0.);
        assert!(input.observe(message_with_rate(1., [1., 0., 0.]), Instant::now()));
        // 时间戳跳过一秒：滤波器按上限步进，状态连续而不是跳回测量值。
        assert!(input.observe(message_with_rate(2., [0., 0., 0.]), Instant::now()));
        let rate = input.latest.expect("snapshot").rate;
        assert!(rate.x > 0. && rate.x < 1., "{rate:?}");
    }
}
