//! Bounded posterior replay used by every online SLAM adapter.
use crate::{Bias, ImuSample, Prediction, PropagationError, State, propagate};
use nalgebra::Vector3;
use std::collections::VecDeque;

pub struct Predictor {
    history: VecDeque<ImuSample>,
    history_ns: i128,
    gravity: Vector3<f64>,
    posterior: Option<i128>,
    state: Option<State>,
    bias: Bias,
    last_published: Option<i128>,
}

impl Predictor {
    #[must_use]
    pub fn new(history_seconds: f64, gravity: Vector3<f64>) -> Self {
        Self {
            history: VecDeque::new(),
            history_ns: (history_seconds * 1e9).round() as i128,
            gravity,
            posterior: None,
            state: None,
            bias: Bias {
                gyro: Vector3::zeros(),
                accel: Vector3::zeros(),
            },
            last_published: None,
        }
    }

    /// Accept the entire posterior atomically, including its biases; older commits are ignored.
    pub fn posterior(&mut self, state: State, bias: Bias) -> bool {
        if self.posterior.is_some_and(|time| state.timestamp_ns < time) {
            return false;
        }
        self.posterior = Some(state.timestamp_ns);
        self.state = Some(state);
        self.bias = bias;
        true
    }

    #[must_use]
    pub fn posterior_time_ns(&self) -> Option<i128> {
        self.posterior
    }

    #[must_use]
    pub fn bias(&self) -> Bias {
        self.bias
    }

    /// Append a fresh sample, replay after a posterior commit, then advance incrementally.
    /// # Errors
    /// Returns numerical/input failures from the shared motion integrator.
    pub fn imu(&mut self, sample: ImuSample) -> Result<Option<Prediction>, PropagationError> {
        let time = sample.timestamp_ns;
        if self
            .history
            .back()
            .is_some_and(|last| time <= last.timestamp_ns)
        {
            return Ok(None);
        }
        self.history.push_back(sample);
        while self.history.len() > 2
            && (self.history.len() > 10_000
                || time - self.history[1].timestamp_ns > self.history_ns)
        {
            self.history.pop_front();
        }
        let Some(state) = self.state else {
            return Ok(None);
        };
        if time <= state.timestamp_ns || self.last_published.is_some_and(|last| time <= last) {
            return Ok(None);
        }
        let measurements = self.history.make_contiguous();
        let after = measurements.partition_point(|m| m.timestamp_ns <= state.timestamp_ns);
        if after == 0 || measurements.len() - (after - 1) < 2 {
            return Ok(None);
        }
        let mut prediction = propagate(
            state,
            &measurements[after - 1..],
            time,
            self.bias,
            self.gravity,
        )?;
        // OKVIS Transformation::set normalizes the propagated quaternion before the next step.
        let norm2 = prediction.state.rotation.coords.norm_squared();
        if norm2 > 0.0 {
            prediction.state.rotation.coords /= norm2.sqrt();
        }
        self.state = Some(prediction.state);
        self.last_published = Some(time);
        Ok(Some(prediction))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Quaternion;

    fn sample(time: i128) -> ImuSample {
        ImuSample {
            timestamp_ns: time,
            gyro: Vector3::zeros(),
            accel: Vector3::new(1., 0., 9.81),
        }
    }
    fn state(time: i128) -> State {
        State {
            timestamp_ns: time,
            rotation: Quaternion::identity(),
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
        }
    }
    fn bias() -> Bias {
        Bias {
            gyro: Vector3::zeros(),
            accel: Vector3::zeros(),
        }
    }

    #[test]
    fn delayed_posterior_replays_the_same_covered_motion_and_rejects_old_biases() {
        let mut p = Predictor::new(5., Vector3::new(0., 0., -9.81));
        for t in 0..=1000 {
            assert!(p.imu(sample(t * 1_000_000)).unwrap().is_none());
        }
        assert!(p.posterior(state(500_000_000), bias()));
        let predicted = p.imu(sample(1_001_000_000)).unwrap().unwrap();
        assert!((predicted.state.velocity.x - 0.501).abs() < 1e-10);
        assert!((predicted.state.position.x - 0.5 * 0.501_f64.powi(2)).abs() < 1e-10);
        assert!(predicted.state.velocity.z.abs() < 1e-10);
        assert!(!p.posterior(
            state(0),
            Bias {
                gyro: Vector3::repeat(1.),
                ..bias()
            }
        ));
        assert_eq!(p.bias(), bias());
        assert!(p.imu(sample(1_001_000_000)).unwrap().is_none());
        assert!(p.posterior(state(900_000_000), bias()));
        let replayed = p.imu(sample(1_002_000_000)).unwrap().unwrap();
        assert!((replayed.state.velocity.x - 0.102).abs() < 1e-10);
    }

    #[test]
    fn missing_and_expired_history_never_extrapolate() {
        let mut p = Predictor::new(0.005, Vector3::zeros());
        p.posterior(state(0), bias());
        assert!(p.imu(sample(1_000_000)).unwrap().is_none());
        for t in 2..20 {
            assert!(p.imu(sample(t * 1_000_000)).unwrap().is_none());
        }
        assert!(p.history.len() <= 8);
    }
}
