//! VIO 后验之间的 IMU 状态传播：姿态、位置与速度。
//!
//! VIO 后验按图像速率到达。将位姿、速度和零偏锚定在后验自身的时刻，再用覆盖锚点
//! 和当前 IMU 时刻的相邻样本插值传播，使 FC 收到同一预测状态的姿态和里程计。
use nalgebra::{UnitQuaternion, Vector3};
use std::collections::VecDeque;

const GRAVITY: Vector3<f64> = Vector3::new(0., 0., -9.81);
/// 与在线 OKVIS2-X 相同，保留最多 5 秒且最多 10000 条 IMU 样本。
const HISTORY_NS: i64 = 5_000_000_000;
const HISTORY_LIMIT: usize = 10_000;

#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub timestamp_ns: i64,
    pub gyro: Vector3<f64>,
    pub accel: Vector3<f64>,
}

#[derive(Clone, Copy, Debug)]
pub struct Anchor {
    pub posterior_time_ns: i64,
    pub rotation: UnitQuaternion<f64>,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
}

#[derive(Clone, Copy, Debug)]
pub struct Propagated {
    pub rotation: UnitQuaternion<f64>,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub timestamp_ns: i64,
}

#[derive(Default)]
pub struct Propagator {
    anchor: Option<Anchor>,
    propagated: Option<Propagated>,
    history: VecDeque<ImuSample>,
    gyro_bias: Vector3<f64>,
    accel_bias: Vector3<f64>,
}

impl Propagator {
    pub fn set_bias(&mut self, gyro_bias: Vector3<f64>, accel_bias: Vector3<f64>) {
        self.gyro_bias = gyro_bias;
        self.accel_bias = accel_bias;
    }

    #[must_use]
    pub fn corrected_gyro(&self, gyro: Vector3<f64>) -> Vector3<f64> {
        gyro - self.gyro_bias
    }

    #[must_use]
    pub fn gyro_bias(&self) -> Vector3<f64> {
        self.gyro_bias
    }

    /// 使用不早于当前锚点的 VIO 后验重置传播起点。
    pub fn set_anchor(&mut self, anchor: Anchor) {
        if self
            .anchor
            .is_some_and(|current| anchor.posterior_time_ns < current.posterior_time_ns)
        {
            return;
        }
        self.anchor = Some(anchor);
        self.propagated = None;
    }

    #[must_use]
    pub fn posterior_time_ns(&self) -> Option<i64> {
        self.anchor.map(|anchor| anchor.posterior_time_ns)
    }

    /// 添加一条 IMU 样本；仅当历史覆盖锚点和该样本时刻时返回预测状态。
    pub fn push(&mut self, sample: ImuSample) -> Option<Propagated> {
        let previous = self.history.back().copied();
        if previous.is_some_and(|previous| sample.timestamp_ns <= previous.timestamp_ns) {
            return None;
        }
        self.history.push_back(sample);
        self.trim_history(sample.timestamp_ns);
        self.anchor?;

        let mut state = match (self.propagated, previous) {
            (Some(state), Some(previous)) if state.timestamp_ns == previous.timestamp_ns => state,
            _ => self.reanchor(sample.timestamp_ns)?,
        };
        if state.timestamp_ns < sample.timestamp_ns {
            let previous = previous?;
            if !integrate_interval(
                &mut state,
                previous,
                sample,
                sample.timestamp_ns,
                self.gyro_bias,
                self.accel_bias,
            ) {
                return None;
            }
        }
        if state.timestamp_ns != sample.timestamp_ns {
            return None;
        }
        self.propagated = Some(state);
        Some(state)
    }

    fn trim_history(&mut self, newest_ns: i64) {
        while self.history.len() > 2
            && (self.history.len() > HISTORY_LIMIT
                || newest_ns - self.history[1].timestamp_ns > HISTORY_NS)
        {
            self.history.pop_front();
        }
    }

    /// 收到新后验后，从覆盖锚点的相邻 IMU 样本重放至当前时刻。
    fn reanchor(&self, until_ns: i64) -> Option<Propagated> {
        let anchor = self.anchor?;
        let first = self.history.front()?;
        let last = self.history.back()?;
        if first.timestamp_ns > anchor.posterior_time_ns
            || last.timestamp_ns < until_ns
            || until_ns <= anchor.posterior_time_ns
        {
            return None;
        }

        let mut state = Propagated {
            rotation: anchor.rotation,
            position: anchor.position,
            velocity: anchor.velocity,
            timestamp_ns: anchor.posterior_time_ns,
        };
        for (left, right) in self.history.iter().zip(self.history.iter().skip(1)) {
            if right.timestamp_ns <= state.timestamp_ns {
                continue;
            }
            let end_ns = until_ns.min(right.timestamp_ns);
            if !integrate_interval(
                &mut state,
                *left,
                *right,
                end_ns,
                self.gyro_bias,
                self.accel_bias,
            ) {
                return None;
            }
            if state.timestamp_ns == until_ns {
                return Some(state);
            }
        }
        None
    }
}

/// 在相邻 IMU 样本间线性插值，并用平均角速度/比力积分到区间终点。
fn integrate_interval(
    state: &mut Propagated,
    left: ImuSample,
    right: ImuSample,
    end_ns: i64,
    gyro_bias: Vector3<f64>,
    accel_bias: Vector3<f64>,
) -> bool {
    let start_ns = state.timestamp_ns;
    if left.timestamp_ns > start_ns || right.timestamp_ns < end_ns || end_ns <= start_ns {
        return false;
    }
    let interval_ns = right.timestamp_ns - left.timestamp_ns;
    if interval_ns <= 0 {
        return false;
    }
    let fraction =
        |timestamp_ns: i64| (timestamp_ns - left.timestamp_ns) as f64 / interval_ns as f64;
    let interpolate = |a: Vector3<f64>, b: Vector3<f64>, t: f64| a + (b - a) * t;
    let start_fraction = fraction(start_ns);
    let end_fraction = fraction(end_ns);
    let gyro_start = interpolate(left.gyro, right.gyro, start_fraction);
    let gyro_end = interpolate(left.gyro, right.gyro, end_fraction);
    let accel_start = interpolate(left.accel, right.accel, start_fraction);
    let accel_end = interpolate(left.accel, right.accel, end_fraction);
    let dt = (end_ns - start_ns) as f64 * 1e-9;

    let angular_velocity = 0.5 * (gyro_start + gyro_end) - gyro_bias;
    let next_rotation = state.rotation * UnitQuaternion::from_scaled_axis(angular_velocity * dt);
    let specific_force = 0.5 * (accel_start + accel_end) - accel_bias;
    let accel_world = 0.5
        * (state.rotation.transform_vector(&specific_force)
            + next_rotation.transform_vector(&specific_force))
        + GRAVITY;

    state.position += state.velocity * dt + accel_world * (0.5 * dt * dt);
    state.velocity += accel_world * dt;
    state.rotation = next_rotation;
    state.timestamp_ns = end_ns;
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn anchor(timestamp_ns: i64) -> Anchor {
        Anchor {
            posterior_time_ns: timestamp_ns,
            rotation: UnitQuaternion::identity(),
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
        }
    }

    fn sample(timestamp_ns: i64, gyro: Vector3<f64>, accel: Vector3<f64>) -> ImuSample {
        ImuSample {
            timestamp_ns,
            gyro,
            accel,
        }
    }

    #[test]
    fn requires_imu_history_to_cover_both_anchor_and_prediction_time() {
        let mut missing_start = Propagator::default();
        missing_start.set_anchor(anchor(1_000_000_000));
        assert!(
            missing_start
                .push(sample(1_001_000_000, Vector3::zeros(), Vector3::zeros()))
                .is_none()
        );

        let mut covered = Propagator::default();
        covered.set_anchor(anchor(1_000_000_000));
        covered.push(sample(999_000_000, Vector3::zeros(), Vector3::zeros()));
        assert!(
            covered
                .push(sample(1_001_000_000, Vector3::zeros(), Vector3::zeros()))
                .is_some()
        );
    }

    #[test]
    fn integrates_stationary_specific_force_without_motion() {
        let mut propagator = Propagator::default();
        propagator.set_anchor(anchor(1_000_000_000));
        propagator.push(sample(
            999_000_000,
            Vector3::zeros(),
            Vector3::new(0., 0., 9.81),
        ));
        let state = propagator
            .push(sample(
                1_001_000_000,
                Vector3::zeros(),
                Vector3::new(0., 0., 9.81),
            ))
            .expect("covered prediction");
        assert!(state.position.norm() < 1e-12);
        assert!(state.velocity.norm() < 1e-12);
        assert_eq!(state.timestamp_ns, 1_001_000_000);
    }

    #[test]
    fn interpolates_start_measurement_and_integrates_full_large_interval() {
        let mut propagator = Propagator::default();
        propagator.set_anchor(anchor(500_000_000));
        propagator.push(sample(0, Vector3::zeros(), Vector3::zeros()));
        propagator.push(sample(1_000_000_000, Vector3::zeros(), Vector3::zeros()));
        let state = propagator
            .push(sample(2_000_000_000, Vector3::zeros(), Vector3::zeros()))
            .expect("covered prediction");
        assert!((state.velocity.z + 14.715).abs() < 1e-9);
        assert!((state.position.z + 11.03625).abs() < 1e-9);
    }

    #[test]
    fn a_new_posterior_replays_buffered_measurements() {
        let mut propagator = Propagator::default();
        propagator.push(sample(
            999_000_000,
            Vector3::zeros(),
            Vector3::new(0., 0., 9.81),
        ));
        propagator.push(sample(
            1_001_000_000,
            Vector3::zeros(),
            Vector3::new(0., 0., 9.81),
        ));
        propagator.push(sample(
            1_002_000_000,
            Vector3::zeros(),
            Vector3::new(0., 0., 9.81),
        ));
        propagator.set_anchor(anchor(1_000_000_000));
        let state = propagator
            .push(sample(
                1_003_000_000,
                Vector3::zeros(),
                Vector3::new(0., 0., 9.81),
            ))
            .expect("replayed prediction");
        assert!(state.position.norm() < 1e-12);
        assert!(state.velocity.norm() < 1e-12);
    }
}
