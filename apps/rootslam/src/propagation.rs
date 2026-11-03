//! VIO posterior buffering around the shared OKVIS2-X IMU predictor.
use firefly_imu_propagation::{Bias, Prediction, State, propagate};
use nalgebra::{UnitQuaternion, Vector3};
use std::collections::VecDeque;

pub use firefly_imu_propagation::ImuSample;

/// 与 OKVIS2-X 相同，保留最多 5 秒且最多 10000 条 IMU 样本。
const HISTORY_NS: i128 = 5_000_000_000;
const HISTORY_LIMIT: usize = 10_000;
const GRAVITY_WORLD: Vector3<f64> = Vector3::new(0., 0., -9.81);

#[derive(Clone, Copy, Debug)]
pub struct Anchor {
    pub posterior_time_ns: i128,
    pub rotation: UnitQuaternion<f64>,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
}

#[derive(Default)]
pub struct Propagator {
    anchor: Option<Anchor>,
    propagated: Option<State>,
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
    pub fn gyro_bias(&self) -> Vector3<f64> {
        self.gyro_bias
    }

    /// 使用不早于当前锚点的 VIO 后验重置预测起点。
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
    pub fn posterior_time_ns(&self) -> Option<i128> {
        self.anchor.map(|anchor| anchor.posterior_time_ns)
    }

    /// 添加一条 IMU 样本；只要锚点和目标时刻被历史覆盖，就调用共享 OKVIS 预测 API。
    pub fn push(&mut self, sample: ImuSample) -> Option<Prediction> {
        if self
            .history
            .back()
            .is_some_and(|previous| sample.timestamp_ns <= previous.timestamp_ns)
        {
            return None;
        }
        self.history.push_back(sample);
        self.trim_history(sample.timestamp_ns);

        let anchor = self.anchor?;
        let start = self.propagated.unwrap_or(State {
            timestamp_ns: anchor.posterior_time_ns,
            rotation: anchor.rotation.into_inner(),
            position: anchor.position,
            velocity: anchor.velocity,
        });
        if sample.timestamp_ns <= start.timestamp_ns {
            return None;
        }

        let measurements = self.history.make_contiguous();
        let first_after_state = measurements
            .partition_point(|measurement| measurement.timestamp_ns <= start.timestamp_ns);
        let start_index = first_after_state.saturating_sub(1);
        let prediction = propagate(
            start,
            &measurements[start_index..],
            sample.timestamp_ns,
            Bias {
                gyro: self.gyro_bias,
                accel: self.accel_bias,
            },
            GRAVITY_WORLD,
        )
        .ok()?;
        self.propagated = Some(prediction.state);
        Some(prediction)
    }

    fn trim_history(&mut self, newest_ns: i128) {
        while self.history.len() > 2
            && (self.history.len() > HISTORY_LIMIT
                || newest_ns - self.history[1].timestamp_ns > HISTORY_NS)
        {
            self.history.pop_front();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn anchor(timestamp_ns: i128) -> Anchor {
        Anchor {
            posterior_time_ns: timestamp_ns,
            rotation: UnitQuaternion::identity(),
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
        }
    }

    fn sample(timestamp_ns: i128, gyro: Vector3<f64>, accel: Vector3<f64>) -> ImuSample {
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
            .expect("covered prediction")
            .state;
        assert!(state.position.norm() < 1e-12);
        assert!(state.velocity.norm() < 1e-12);
        assert_eq!(state.timestamp_ns, 1_001_000_000);
    }

    #[test]
    fn integrates_full_covered_interval() {
        let mut propagator = Propagator::default();
        propagator.set_anchor(anchor(500_000_000));
        propagator.push(sample(0, Vector3::zeros(), Vector3::zeros()));
        let state = propagator
            .push(sample(2_000_000_000, Vector3::zeros(), Vector3::zeros()))
            .expect("covered prediction")
            .state;
        assert!((state.velocity.z + 14.715).abs() < 1e-9);
        assert!((state.position.z + 11.03625).abs() < 1e-9);
    }

    #[test]
    fn a_new_posterior_replaces_the_prediction_origin() {
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
            .expect("replayed prediction")
            .state;
        assert!(state.position.norm() < 1e-12);
        assert!(state.velocity.norm() < 1e-12);
    }
}
