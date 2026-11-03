//! Shared stationary gravity sampling for visual-inertial estimators.

use nalgebra::{UnitQuaternion, Vector3};

const MIN_DURATION_NS: i128 = 100_000_000;
const MIN_ACCEL_NORM: f64 = 8.0;
const MAX_ACCEL_NORM: f64 = 11.5;
const MAX_GYRO_NORM: f64 = 0.2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    pub timestamp_ns: i128,
    pub gyro: Vector3<f64>,
    pub accel: Vector3<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GravityEstimate {
    pub start_ns: i128,
    pub end_ns: i128,
    pub sample_count: usize,
    /// Mean measured specific force over the stationary window, in IMU coordinates.
    pub mean_specific_force: Vector3<f64>,
}

impl GravityEstimate {
    /// Rotation from the IMU frame to a world frame whose positive Z axis is up.
    #[must_use]
    pub fn rotation_body_to_world(&self) -> Option<UnitQuaternion<f64>> {
        let norm = self.mean_specific_force.norm();
        if !norm.is_finite() || norm <= f64::EPSILON {
            return None;
        }
        UnitQuaternion::rotation_between(&(self.mean_specific_force / norm), &Vector3::z())
    }
}

#[derive(Clone, Debug, Default)]
pub struct StationaryGravitySampler {
    start_ns: Option<i128>,
    last_ns: Option<i128>,
    sample_count: usize,
    sum_specific_force: Vector3<f64>,
}

impl StationaryGravitySampler {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds one IMU sample and returns the estimate when a continuous stationary window is ready.
    ///
    /// Samples must be time ordered. Invalid, non-monotonic, or moving samples reset the window.
    pub fn push(&mut self, sample: ImuSample) -> Option<GravityEstimate> {
        let finite = sample
            .gyro
            .iter()
            .chain(sample.accel.iter())
            .all(|value| value.is_finite());
        let accel_norm = sample.accel.norm();
        let stationary = finite
            && (MIN_ACCEL_NORM..=MAX_ACCEL_NORM).contains(&accel_norm)
            && sample.gyro.norm() < MAX_GYRO_NORM;
        if !stationary
            || self
                .last_ns
                .is_some_and(|previous| sample.timestamp_ns <= previous)
        {
            self.reset();
            return None;
        }

        self.start_ns.get_or_insert(sample.timestamp_ns);
        self.last_ns = Some(sample.timestamp_ns);
        self.sample_count += 1;
        self.sum_specific_force += sample.accel;

        let start_ns = self.start_ns.expect("stationary sample sets window start");
        if sample.timestamp_ns - start_ns < MIN_DURATION_NS || self.sample_count < 2 {
            return None;
        }
        Some(GravityEstimate {
            start_ns,
            end_ns: sample.timestamp_ns,
            sample_count: self.sample_count,
            mean_specific_force: self.sum_specific_force / self.sample_count as f64,
        })
    }

    pub fn reset(&mut self) {
        self.start_ns = None;
        self.last_ns = None;
        self.sample_count = 0;
        self.sum_specific_force = Vector3::zeros();
    }
}

/// Finds the first continuous stationary gravity window in an ordered sample stream.
#[must_use]
pub fn sample_stationary_gravity(
    samples: impl IntoIterator<Item = ImuSample>,
) -> Option<GravityEstimate> {
    let mut sampler = StationaryGravitySampler::new();
    for sample in samples {
        if let Some(estimate) = sampler.push(sample) {
            return Some(estimate);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn estimates_mean_force_after_stationary_window() {
        let samples = (0..=100).map(|index| ImuSample {
            timestamp_ns: index * 1_000_000,
            gyro: Vector3::zeros(),
            accel: Vector3::new(0.0, 0.0, if index % 2 == 0 { 9.7 } else { 9.9 }),
        });
        let estimate = sample_stationary_gravity(samples).unwrap();
        assert_eq!(estimate.sample_count, 101);
        assert!(
            (estimate.mean_specific_force.z - 9.8).abs() < 0.002,
            "unexpected mean specific force: {}",
            estimate.mean_specific_force.z
        );
        assert_eq!(estimate.end_ns - estimate.start_ns, MIN_DURATION_NS);
    }

    #[test]
    fn movement_resets_stationary_window() {
        let mut sampler = StationaryGravitySampler::new();
        for index in 0..70 {
            let _ = sampler.push(ImuSample {
                timestamp_ns: index * 1_000_000,
                gyro: Vector3::zeros(),
                accel: Vector3::new(0.0, 0.0, 9.81),
            });
        }
        assert!(
            sampler
                .push(ImuSample {
                    timestamp_ns: 70_000_000,
                    gyro: Vector3::new(0.3, 0.0, 0.0),
                    accel: Vector3::new(0.0, 0.0, 9.81),
                })
                .is_none()
        );
        assert!(
            sampler
                .push(ImuSample {
                    timestamp_ns: 170_000_000,
                    gyro: Vector3::zeros(),
                    accel: Vector3::new(0.0, 0.0, 9.81),
                })
                .is_none()
        );
    }

    #[test]
    fn alignment_maps_measured_force_to_world_up() {
        let estimate = GravityEstimate {
            start_ns: 0,
            end_ns: MIN_DURATION_NS,
            sample_count: 101,
            mean_specific_force: Vector3::new(0.0, 9.81, 0.0),
        };
        let rotation = estimate.rotation_body_to_world().unwrap();
        let aligned = rotation * estimate.mean_specific_force.normalize();
        assert!((aligned - Vector3::z()).norm() < 1e-12);
    }
}
