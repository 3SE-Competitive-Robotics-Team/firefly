//! 测量时钟、静止窗口与有界延迟重放；不接受跨 VIO 会话外援。
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
/// 同一 IMU 时刻的局部 odom 系分布；禁止传入地图校正姿态。
#[derive(Clone, Debug)]
pub struct Aid {
    pub timestamp: f64,
    pub session: u64,
    pub state: AttitudeState,
}
#[derive(Clone)]
struct Step {
    sample: Sample,
    state: AttitudeState,
    stationary: bool,
    dt: f64,
}
const HISTORY_CAPACITY: usize = 2048;
const STATIONARY_CAPACITY: usize = 4096;

/// 独立姿态估计器；IMU 缺口锁存，VIO 失联仍可用连续 IMU 传播。
pub struct Estimator {
    options: Options,
    history: VecDeque<Step>,
    window: VecDeque<Sample>,
    latest: Option<Sample>,
    state: Option<AttitudeState>,
    rate: Vector3<f64>,
    continuous: bool,
    aligned: bool,
    session: Option<u64>,
    session_fault: bool,
    last_aid: Option<f64>,
    /// 同一 IMU 区间内可有多次外援；保留最近外援时刻的后验。
    aid_checkpoint: Option<(f64, AttitudeState)>,
    gate: Gate,
}
impl Estimator {
    /// # Errors
    /// Options 无效。
    pub fn new(options: Options) -> Result<Self> {
        options.validate()?;
        Ok(Self {
            options,
            history: VecDeque::with_capacity(HISTORY_CAPACITY),
            window: VecDeque::with_capacity(STATIONARY_CAPACITY),
            latest: None,
            state: None,
            rate: Vector3::zeros(),
            continuous: true,
            aligned: false,
            session: None,
            session_fault: false,
            last_aid: None,
            aid_checkpoint: None,
            gate: Gate::default(),
        })
    }
    #[must_use]
    pub fn state(&self) -> Option<&AttitudeState> {
        self.state.as_ref().filter(|_| self.continuous)
    }
    #[must_use]
    pub const fn aligned(&self) -> bool {
        self.aligned && self.continuous
    }
    #[must_use]
    pub const fn rate(&self) -> Vector3<f64> {
        self.rate
    }
    #[must_use]
    pub const fn gate(&self) -> Gate {
        self.gate
    }
    #[must_use]
    pub const fn last_aid(&self) -> Option<f64> {
        self.last_aid
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
        if !self.continuous || (self.state.is_some() && dt > self.options.max_dt + 1e-9) {
            self.continuous = false;
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
        if self.state.is_none() {
            if !stationary {
                return Ok(true);
            }
            let (gyro, accel, _, _) = self.moments();
            let rotation = UnitQuaternion::rotation_between(&accel, &Vector3::z())
                .ok_or_else(|| invalid("ambiguous initial gravity direction"))?;
            let mut covariance = Matrix6::identity() * 1e-6;
            // 重力只约束倾角；绕世界竖直轴的初始航向是自由规范。
            let up = rotation.inverse() * Vector3::z();
            covariance.fixed_view_mut::<3, 3>(0, 0).copy_from(
                &(nalgebra::Matrix3::identity() * 1e-4
                    + up * up.transpose() * std::f64::consts::PI.powi(2)),
            );
            self.state = Some(AttitudeState {
                rotation,
                bias: gyro,
                covariance,
            });
            self.rate = sample.gyro - gyro;
        } else if let Some(state) = &mut self.state {
            state.predict(sample.gyro, dt, &self.options)?;
            self.gate = state.correct_accel(sample.accel, dt, stationary, &self.options)?;
            let alpha = 1. - (-std::f64::consts::TAU * self.options.rate_cutoff_hz * dt).exp();
            self.rate += (sample.gyro - state.bias - self.rate) * alpha;
        }
        if let Some(state) = &self.state {
            if self.history.len() == HISTORY_CAPACITY {
                self.history.pop_front();
            }
            self.history.push_back(Step {
                sample,
                state: state.clone(),
                stationary,
                dt,
            });
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
        g.norm() <= self.options.stationary_gyro_max
            && (a.norm() - self.options.gravity).abs() <= self.options.stationary_accel_error
            && gv.max() <= self.options.stationary_gyro_std.powi(2)
            && av.max() <= self.options.stationary_accel_std.powi(2)
    }

    /// 外援在其观测时刻融合，再按已接收 IMU 重放到当前时刻。
    /// 首次外援确定 odom 航向规范并接管该时刻分布；后续使用 CI。
    /// # Errors
    /// 外援非有限、协方差无效或会话变化。乱序/缓存外外援返回 false。
    #[fastrace::trace]
    #[allow(clippy::float_cmp)] // 测量时间戳相等时复用已完成的样本更新
    pub fn aid(&mut self, aid: &Aid) -> Result<bool> {
        aid.state.validate()?;
        if !aid.timestamp.is_finite() || aid.timestamp < 0. || aid.session == 0 {
            return Err(invalid("invalid attitude aid metadata"));
        }
        if self.session_fault || self.session.is_some_and(|s| s != aid.session) {
            self.session_fault = true;
            self.aligned = false;
            return Err(Error::new(
                ErrorKind::Unsupported,
                "VIO session changed; FC restart required",
            ));
        }
        if !self.continuous
            || self.last_aid.is_some_and(|t| aid.timestamp <= t)
            || self
                .history
                .back()
                .is_none_or(|s| aid.timestamp > s.sample.timestamp)
            || self
                .history
                .front()
                .is_none_or(|s| aid.timestamp < s.sample.timestamp)
        {
            return Ok(false);
        }
        let Some(index) = self
            .history
            .iter()
            .rposition(|s| s.sample.timestamp <= aid.timestamp)
        else {
            return Ok(false);
        };
        let base = &self.history[index];
        let (base_time, mut state) = self
            .aid_checkpoint
            .as_ref()
            .filter(|(t, _)| *t > base.sample.timestamp)
            .map_or_else(
                || (base.sample.timestamp, base.state.clone()),
                |(t, state)| (*t, state.clone()),
            );
        if aid.timestamp > base_time {
            state.predict(
                self.history[index + 1].sample.gyro,
                aid.timestamp - base_time,
                &self.options,
            )?;
        }
        if self.session.is_none() {
            state = aid.state.clone();
        } else if !state.intersect(&aid.state)? {
            return Ok(false);
        }
        self.aid_checkpoint = Some((aid.timestamp, state.clone()));
        if aid.timestamp == self.history[index].sample.timestamp {
            self.history[index].state = state.clone();
        }
        let mut previous = aid.timestamp;
        for step in self.history.iter_mut().skip(index + 1) {
            state.predict(
                step.sample.gyro,
                step.sample.timestamp - previous,
                &self.options,
            )?;
            self.gate =
                state.correct_accel(step.sample.accel, step.dt, step.stationary, &self.options)?;
            previous = step.sample.timestamp;
            step.state = state.clone();
        }
        if let Some(old) = &self.state {
            self.rate += old.bias - state.bias;
        }
        self.state = Some(state);
        self.session = Some(aid.session);
        self.last_aid = Some(aid.timestamp);
        self.aligned = true;
        Ok(true)
    }
}
