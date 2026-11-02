//! VIO 后验之间的惯性预积分：姿态、位置、速度。
//!
//! 后验只按图像速率到达（约 46 Hz），飞控的位置与速度环却是 1 kHz。把后验锚在
//! 自己的时刻上，用每条 IMU 向前传播，就得到连续的高频位姿；飞控拿到的反馈
//! 延迟因此从一整个后验间隔降到一条 IMU。
//!
//! 约定与上游 Basalt 一致：IMU 加速度计读的是机体系比力 `f_b`，即
//! `f_b = Rᵀ(a_world − g_world)`；重力取世界系 `[0, 0, −9.81]`。
//! 因此世界系真实加速度是 `a_world = R·f_b + g_world`。
use nalgebra::{UnitQuaternion, Vector3};
use std::collections::VecDeque;

/// 世界系重力加速度（m/s²）。
const GRAVITY: Vector3<f64> = Vector3::new(0., 0., -9.81);

/// IMU 历史长度上限（样本）；1 kHz 下覆盖两秒。
const HISTORY_LIMIT: usize = 4096;

/// 单次积分允许的最大步长（s）；超出的部分视为观测空缺，不参与积分。
const MAX_STEP_S: f64 = 0.05;

/// 单条 IMU 样本（原始读数；零偏在传播时按当前后验扣除）。
#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub timestamp: f64,
    pub gyro: Vector3<f64>,
    pub accel: Vector3<f64>,
}

/// 传播锚点：VIO 后验的位姿与速度，落在后验自己的时刻上。
#[derive(Clone, Copy, Debug)]
pub struct Anchor {
    /// 后验自身的时刻；飞控用它判断后验新鲜度。
    pub posterior_time: f64,
    pub rotation: UnitQuaternion<f64>,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
}

/// 传播出的世界系状态。
#[derive(Clone, Copy, Debug)]
pub struct Propagated {
    pub rotation: UnitQuaternion<f64>,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub timestamp: f64,
}

/// 惯性传播器：持有后验锚点、IMU 历史与当前零偏。
#[derive(Default)]
pub struct Propagator {
    anchor: Option<Anchor>,
    /// 已按当前锚点传播到的时刻与状态；锚点换了就作废。
    propagated: Option<Propagated>,
    history: VecDeque<ImuSample>,
    gyro_bias: Vector3<f64>,
    accel_bias: Vector3<f64>,
}

impl Propagator {
    /// 设置当前后验的零偏（rad/s、m/s²）。
    pub fn set_bias(&mut self, gyro_bias: Vector3<f64>, accel_bias: Vector3<f64>) {
        self.gyro_bias = gyro_bias;
        self.accel_bias = accel_bias;
    }

    /// 当前零偏下的机体系角速度（rad/s）。
    #[must_use]
    pub fn corrected_gyro(&self, gyro: Vector3<f64>) -> Vector3<f64> {
        gyro - self.gyro_bias
    }

    /// 当前陀螺零偏（rad/s）。
    #[must_use]
    pub fn gyro_bias(&self) -> Vector3<f64> {
        self.gyro_bias
    }

    /// 用新的后验重设锚点；已有的传播结果作废，下次按新锚点重算。
    pub fn set_anchor(&mut self, anchor: Anchor) {
        self.anchor = Some(anchor);
        self.propagated = None;
    }

    /// 锚点的后验时刻；没有锚点时为 `None`。
    #[must_use]
    pub fn posterior_time(&self) -> Option<f64> {
        self.anchor.map(|anchor| anchor.posterior_time)
    }

    /// 收下一条 IMU，返回传播到该时刻的状态。
    ///
    /// 锚点按后验自己的时刻记账，比当前 IMU 时刻早；状态因此由锚点经 IMU 历史
    /// 重算得到，而发布时刻始终是 IMU 时刻——单调递增，飞控按时间戳去重不会丢样。
    pub fn push(&mut self, sample: ImuSample) -> Option<Propagated> {
        self.history.push_back(sample);
        while self.history.len() > HISTORY_LIMIT {
            self.history.pop_front();
        }
        self.anchor?;
        let mut state = match self.propagated {
            Some(state) => state,
            None => self.reanchor(sample.timestamp)?,
        };
        if sample.timestamp > state.timestamp {
            let dt = sample.timestamp - state.timestamp;
            step(
                &mut state,
                sample.gyro - self.gyro_bias,
                sample.accel - self.accel_bias,
                dt.min(MAX_STEP_S),
                sample.timestamp,
            );
        }
        self.propagated = Some(state);
        Some(state)
    }

    /// 从锚点经 IMU 历史重算到给定时刻。
    ///
    /// 锚点落在历史之外（后验长时间未到）时，锚点之后的第一段观测空缺不参与积分，
    /// 传播从历史里第一条可用样本重新起步。
    fn reanchor(&mut self, until: f64) -> Option<Propagated> {
        let anchor = self.anchor?;
        let mut state = Propagated {
            rotation: anchor.rotation,
            position: anchor.position,
            velocity: anchor.velocity,
            timestamp: anchor.posterior_time,
        };
        let gyro_bias = self.gyro_bias;
        let accel_bias = self.accel_bias;
        let mut used = false;
        for sample in &self.history {
            if sample.timestamp <= state.timestamp {
                continue;
            }
            if sample.timestamp > until {
                break;
            }
            let dt = sample.timestamp - state.timestamp;
            if !dt.is_finite() || dt <= 0. {
                continue;
            }
            step(
                &mut state,
                sample.gyro - gyro_bias,
                sample.accel - accel_bias,
                dt.min(MAX_STEP_S),
                sample.timestamp,
            );
            used = true;
        }
        used.then_some(state)
    }
}

/// 单步传播：半步姿态把比力换到世界系，加重力得到真实加速度。
fn step(state: &mut Propagated, gyro: Vector3<f64>, accel: Vector3<f64>, dt: f64, timestamp: f64) {
    let half_rotation = state.rotation * UnitQuaternion::from_scaled_axis(gyro * (0.5 * dt));
    let accel_world = half_rotation.transform_vector(&accel) + GRAVITY;
    state.position += state.velocity * dt + accel_world * (0.5 * dt * dt);
    state.velocity += accel_world * dt;
    state.rotation *= UnitQuaternion::from_scaled_axis(gyro * dt);
    state.timestamp = timestamp;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn anchor() -> Anchor {
        Anchor {
            posterior_time: 1.,
            rotation: UnitQuaternion::identity(),
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
        }
    }

    fn sample(timestamp: f64, gyro: Vector3<f64>, accel: Vector3<f64>) -> ImuSample {
        ImuSample {
            timestamp,
            gyro,
            accel,
        }
    }

    #[test]
    fn a_stationary_body_stays_put() {
        let mut propagator = Propagator::default();
        propagator.set_anchor(anchor());
        let mut state = None;
        for index in 0..100i32 {
            state = propagator.push(sample(
                1. + f64::from(index) * 0.001,
                Vector3::zeros(),
                Vector3::new(0., 0., 9.81),
            ));
        }
        let state = state.expect("propagated state");
        assert!(state.position.norm() < 1e-9, "{:?}", state.position);
        assert!(state.velocity.norm() < 1e-9, "{:?}", state.velocity);
    }

    #[test]
    fn a_constant_acceleration_integrates_to_the_closed_form() {
        let mut propagator = Propagator::default();
        propagator.set_anchor(anchor());
        // 比力为零，重力给出 -9.81 m/s²。样本从锚点之后一拍开始，
        // 100 拍对应 0.1 s 的积分时长。
        let mut state = None;
        for index in 0..100i32 {
            state = propagator.push(sample(
                1. + f64::from(index + 1) * 0.001,
                Vector3::zeros(),
                Vector3::zeros(),
            ));
        }
        let state = state.expect("propagated state");
        let t = 0.1;
        assert!(
            (state.velocity.z + 9.81 * t).abs() < 1e-6,
            "{:?}",
            state.velocity
        );
        assert!(
            (state.position.z + 0.5 * 9.81 * t * t).abs() < 1e-6,
            "{:?}",
            state.position
        );
    }

    #[test]
    fn the_specific_force_follows_the_body_rotation() {
        let mut propagator = Propagator::default();
        propagator.set_anchor(anchor());
        // 机体系里恒定的 1 m/s² 侧向比力：机体绕 z 转 1 s，世界系里的加速度方向
        // 必须跟着转，积分出的速度是 (sin t, 1 − cos t)。
        let mut state = None;
        for index in 0..1000i32 {
            state = propagator.push(sample(
                1. + f64::from(index + 1) * 0.001,
                Vector3::new(0., 0., 1.),
                Vector3::new(1., 0., 9.81),
            ));
        }
        let state = state.expect("propagated state");
        assert!((state.rotation.angle() - 1.).abs() < 1e-6);
        assert!(
            (state.velocity.x - 1f64.sin()).abs() < 1e-3,
            "{:?}",
            state.velocity
        );
        assert!(
            (state.velocity.y - (1. - 1f64.cos())).abs() < 1e-3,
            "{:?}",
            state.velocity
        );
        assert!(state.velocity.z.abs() < 1e-9, "{:?}", state.velocity);
    }

    #[test]
    fn a_new_anchor_moves_the_propagated_state() {
        let mut propagator = Propagator::default();
        propagator.set_anchor(anchor());
        propagator.push(sample(1.001, Vector3::zeros(), Vector3::new(0., 0., 9.81)));
        let mut moved = anchor();
        moved.position = Vector3::new(5., 0., 0.);
        propagator.set_anchor(moved);
        let state = propagator
            .push(sample(1.002, Vector3::zeros(), Vector3::new(0., 0., 9.81)))
            .expect("propagated state");
        assert!(state.position.x > 4.9, "{:?}", state.position);
    }

    #[test]
    fn the_published_timestamp_follows_the_imu_clock() {
        let mut propagator = Propagator::default();
        propagator.set_anchor(anchor());
        let state = propagator
            .push(sample(1.005, Vector3::zeros(), Vector3::new(0., 0., 9.81)))
            .expect("propagated state");
        assert!((state.timestamp - 1.005).abs() < 1e-9);
        assert_eq!(propagator.posterior_time(), Some(1.));
    }

    #[test]
    fn a_long_observation_gap_does_not_integrate_as_one_huge_step() {
        let mut propagator = Propagator::default();
        propagator.set_anchor(anchor());
        // 锚点在 1.000 s，两秒后才来第一条 IMU：观测空缺不参与积分，
        // 传播按最大步长起步，时间戳仍然对齐 IMU 时钟。
        let state = propagator
            .push(sample(3.0, Vector3::zeros(), Vector3::new(0., 0., 9.81)))
            .expect("propagated state");
        assert!((state.timestamp - 3.0).abs() < 1e-9);
        assert!(state.position.z.abs() < 0.05, "{:?}", state.position);
    }
}
