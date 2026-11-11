//! 独立 IMU 热备：测量时钟与静止窗口；不接收视觉状态或修正。
use crate::{AttitudeState, Gate, Matrix6, Options, invalid};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{UnitQuaternion, Vector3};
use std::collections::VecDeque;

/// SI 单位的机体系测量，timestamp 为传感器时钟秒。
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub timestamp: f64,
    pub gyro: Vector3<f64>,
    pub accel: Vector3<f64>,
}
const STATIONARY_CAPACITY: usize = 4096;

/// 独立姿态估计器；IMU 缺口锁存，VIO 失联仍可用连续 IMU 传播。
pub struct Estimator {
    options: Options,
    window: VecDeque<Sample>,
    latest: Option<Sample>,
    phase: Phase,
    rate: Vector3<f64>,
    gate: Gate,
}

// One estimator owns one filter; keep the filter inline in its active phase.
#[allow(clippy::large_enum_variant)]
enum Phase {
    Initializing,
    Tracking(AttitudeState),
    Discontinuous,
}
impl Estimator {
    /// # Errors
    /// Options 无效。
    pub fn new(options: Options) -> Result<Self> {
        options.validate()?;
        Ok(Self {
            options,
            window: VecDeque::with_capacity(STATIONARY_CAPACITY),
            latest: None,
            phase: Phase::Initializing,
            rate: Vector3::zeros(),
            gate: Gate::default(),
        })
    }
    #[must_use]
    pub fn state(&self) -> Option<&AttitudeState> {
        match &self.phase {
            Phase::Tracking(state) => Some(state),
            Phase::Initializing | Phase::Discontinuous => None,
        }
    }
    #[must_use]
    pub const fn rate(&self) -> Vector3<f64> {
        self.rate
    }
    #[must_use]
    pub const fn gate(&self) -> Gate {
        self.gate
    }
    /// 每个时间戳只消费一次；只有地面允许静止初始化与加计修正。
    /// # Errors
    /// 非有限测量或超过 `max_dt` 的间断。间断后实例不可继续使用。
    #[fastrace::trace]
    pub fn observe(&mut self, sample: Sample, grounded: bool) -> Result<bool> {
        if !sample.timestamp.is_finite()
            || sample.timestamp < 0.
            || !sample
                .gyro
                .iter()
                .chain(sample.accel.iter())
                .all(|x| x.is_finite())
        {
            return Err(invalid("non-finite IMU measurement"));
        }
        if self.latest.is_some_and(|s| sample.timestamp <= s.timestamp) {
            return Ok(false);
        }
        let dt = self.latest.map_or(0., |s| sample.timestamp - s.timestamp);
        if matches!(self.phase, Phase::Discontinuous)
            || (matches!(self.phase, Phase::Tracking(_)) && dt > self.options.max_dt + 1e-9)
        {
            self.phase = Phase::Discontinuous;
            return Err(Error::new(
                ErrorKind::Timeout,
                "IMU time gap; estimator restart required",
            ));
        }
        if dt > self.options.max_dt {
            self.window.clear();
        }
        self.latest = Some(sample);
        let stationary = self.stationary(sample, grounded);
        if matches!(self.phase, Phase::Initializing) {
            if !stationary {
                return Ok(true);
            }
            let (gyro, accel, _, _) = self.moments();
            let rotation = UnitQuaternion::rotation_between(&accel, &Vector3::z())
                .ok_or_else(|| invalid("ambiguous initial gravity direction"))?;
            let mut covariance = Matrix6::zeros();
            covariance.fixed_view_mut::<3, 3>(3, 3).copy_from(
                &crate::gyro_mean_covariance(
                    self.window.iter().map(|s| (s.timestamp, s.gyro)),
                    self.options.gyro_density,
                    self.options.bias_walk,
                )
                .ok_or_else(|| invalid("invalid stationary gyro covariance"))?,
            );
            // 航向固定本地坐标规范，与 VIO 的 odom 航向独立。
            // 协方差描述此规范下的局部小角误差，不能把全局未知航向当作 π rad 误差。
            covariance
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&(nalgebra::Matrix3::identity() * 1e-4));
            self.phase = Phase::Tracking(AttitudeState {
                rotation,
                bias: gyro,
                covariance,
            });
            self.rate = sample.gyro - gyro;
        } else if let Phase::Tracking(state) = &mut self.phase {
            state.predict(sample.gyro, dt, &self.options)?;
            self.gate = state.correct_accel(sample.accel, dt, stationary, &self.options)?;
            let alpha = 1. - (-std::f64::consts::TAU * self.options.rate_cutoff_hz * dt).exp();
            self.rate += (sample.gyro - state.bias - self.rate) * alpha;
        }
        Ok(true)
    }

    fn moments(&self) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        let n = self.window.len() as f64;
        let gyro = self.window.iter().map(|s| s.gyro).sum::<Vector3<f64>>() / n;
        let accel = self.window.iter().map(|s| s.accel).sum::<Vector3<f64>>() / n;
        let gv = self
            .window
            .iter()
            .map(|s| (s.gyro - gyro).map(|x| x * x))
            .sum::<Vector3<f64>>()
            / n;
        let av = self
            .window
            .iter()
            .map(|s| (s.accel - accel).map(|x| x * x))
            .sum::<Vector3<f64>>()
            / n;
        (gyro, accel, gv, av)
    }
    fn stationary(&mut self, s: Sample, grounded: bool) -> bool {
        if !grounded {
            self.window.clear();
            return false;
        }
        if self.window.len() == STATIONARY_CAPACITY {
            self.window.pop_front();
        }
        self.window.push_back(s);
        while self.window.len() > 2
            && self.window[1].timestamp <= s.timestamp - self.options.stationary_seconds
        {
            self.window.pop_front();
        }
        if self.window.len() < 20
            || s.timestamp - self.window[0].timestamp + 1e-9 < self.options.stationary_seconds
        {
            return false;
        }
        let (g, a, gv, av) = self.moments();
        // 对 1/N 中心化样本方差，独立异方差白噪声的期望为
        // (N-1)/N² · Σ density²/Δti；首样本采用相邻周期。
        let n = self.window.len() as f64;
        let mut inverse_periods = 0.;
        for i in 1..self.window.len() {
            let dt = self.window[i].timestamp - self.window[i - 1].timestamp;
            inverse_periods += if i == 1 { 2. / dt } else { 1. / dt };
        }
        let noise_scale = (n - 1.) / n.powi(2) * inverse_periods;
        g.norm() <= self.options.stationary_gyro_max
            && (a.norm() - self.options.gravity).abs() <= self.options.stationary_accel_error
            && gv.max()
                <= self.options.stationary_gyro_std.powi(2)
                    + self.options.gyro_density.powi(2) * noise_scale
            && av.max()
                <= self.options.stationary_accel_std.powi(2)
                    + self.options.accel_density.powi(2) * noise_scale
    }
}
