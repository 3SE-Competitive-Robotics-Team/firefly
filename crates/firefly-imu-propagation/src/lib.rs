//! Shared OKVIS2-X motion integration for online IMU state prediction.
//!
//! The integration order and endpoint interpolation mirror the no-covariance
//! `ImuError::propagation` path. Both online VIO adapters use this API so their
//! IMU predictions cannot drift into separate numerical implementations.

use nalgebra::{Matrix3, Quaternion, Vector3};

mod predictor;
pub use predictor::Predictor;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    pub timestamp_ns: i128,
    pub gyro: Vector3<f64>,
    pub accel: Vector3<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct State {
    pub timestamp_ns: i128,
    /// Rotation from the IMU sensor frame to the world frame.
    pub rotation: Quaternion<f64>,
    /// IMU position and velocity in the world frame.
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bias {
    pub gyro: Vector3<f64>,
    pub accel: Vector3<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Prediction {
    pub state: State,
    /// Corrected angular rate at the prediction endpoint, in IMU coordinates.
    pub angular_velocity: Vector3<f64>,
    pub steps: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PropagationError {
    MissingMeasurements,
    StartNotCovered,
    EndNotCovered,
    NoIntegrationSteps,
    InvalidMeasurements,
}

/// Propagates one state using the OKVIS2-X motion-only preintegration path.
///
/// `measurements` must be time ordered and cover both `state.timestamp_ns`
/// and `end_ns`. Accelerometer samples are specific force in m/s²; `gravity_world`
/// is the signed gravity vector in world coordinates (normally `[0, 0, -9.81]`).
/// Biases are held constant over this interval, matching the OKVIS live predictor.
///
/// # Errors
/// Returns an error if measurements are missing, do not cover the interval, or contain
/// non-finite values, or if the interval contains no valid integration step.
#[allow(clippy::too_many_lines)] // Preserve the operation order of OKVIS's numerical reference.
pub fn propagate(
    state: State,
    measurements: &[ImuSample],
    end_ns: i128,
    bias: Bias,
    gravity_world: Vector3<f64>,
) -> Result<Prediction, PropagationError> {
    let first = measurements
        .first()
        .ok_or(PropagationError::MissingMeasurements)?;
    let last = measurements
        .last()
        .ok_or(PropagationError::MissingMeasurements)?;
    if first.timestamp_ns > state.timestamp_ns {
        return Err(PropagationError::StartNotCovered);
    }
    if last.timestamp_ns < end_ns {
        return Err(PropagationError::EndNotCovered);
    }
    if end_ns < state.timestamp_ns
        || !state
            .position
            .iter()
            .chain(state.velocity.iter())
            .all(|x| x.is_finite())
        || !bias
            .gyro
            .iter()
            .chain(bias.accel.iter())
            .all(|x| x.is_finite())
        || !gravity_world.iter().all(|x| x.is_finite())
        || !measurements.iter().all(|sample| {
            sample
                .gyro
                .iter()
                .chain(sample.accel.iter())
                .all(|x| x.is_finite())
        })
    {
        return Err(PropagationError::InvalidMeasurements);
    }

    if end_ns == state.timestamp_ns {
        return Ok(Prediction {
            state,
            angular_velocity: interpolate_gyro(measurements, end_ns)? - bias.gyro,
            steps: 0,
        });
    }

    let mut delta_rotation = Quaternion::identity();
    let mut acceleration_integral = Vector3::zeros();
    let mut acceleration_double_integral = Vector3::zeros();
    let mut time_ns = state.timestamp_ns;
    let mut started = false;
    let mut steps = 0;
    let mut total = 0.0;

    for pair in measurements.windows(2) {
        let [left, right] = pair else { unreachable!() };
        let (mut gyro0, mut gyro1) = (left.gyro, right.gyro);
        let (mut accel0, mut accel1) = (left.accel, right.accel);
        let mut next_ns = right.timestamp_ns;
        let mut dt = seconds(next_ns - time_ns);
        if end_ns < next_ns {
            let interval = seconds(right.timestamp_ns - left.timestamp_ns);
            next_ns = end_ns;
            dt = seconds(next_ns - time_ns);
            let ratio = dt / interval;
            gyro1 = (1.0 - ratio) * gyro0 + ratio * gyro1;
            accel1 = (1.0 - ratio) * accel0 + ratio * accel1;
        }
        if dt <= 0.0 {
            continue;
        }
        if !started {
            started = true;
            let ratio = dt / seconds(next_ns - left.timestamp_ns);
            gyro0 = ratio * gyro0 + (1.0 - ratio) * gyro1;
            accel0 = ratio * accel0 + (1.0 - ratio) * accel1;
        }

        let angular_velocity = 0.5 * (gyro0 + gyro1) - bias.gyro;
        let delta_q = delta_q(&(angular_velocity * dt));
        let next_rotation = delta_rotation * delta_q;
        let rotation0 = rotation_matrix(&delta_rotation);
        let rotation1 = rotation_matrix(&next_rotation);
        let acceleration = 0.5 * (accel0 + accel1) - bias.accel;
        let next_acceleration_integral =
            acceleration_integral + 0.5 * (rotation0 + rotation1) * acceleration * dt;
        acceleration_double_integral +=
            acceleration_integral * dt + 0.25 * (rotation0 + rotation1) * acceleration * dt * dt;
        delta_rotation = next_rotation;
        acceleration_integral = next_acceleration_integral;
        total += dt;
        time_ns = next_ns;
        steps += 1;
        if next_ns == end_ns {
            break;
        }
    }
    if steps == 0 || time_ns != end_ns {
        return Err(PropagationError::NoIntegrationSteps);
    }

    let dt = total;
    let endpoint_gyro = measurements
        .iter()
        .rev()
        .find(|sample| sample.timestamp_ns == end_ns)
        .map_or_else(
            || interpolate_gyro(measurements, end_ns),
            |sample| Ok(sample.gyro),
        )?
        - bias.gyro;
    let rotation = state.rotation * delta_rotation;
    let world_from_start = rotation_matrix(&state.rotation);
    let position = state.position
        + state.velocity * dt
        + world_from_start * acceleration_double_integral
        + 0.5 * gravity_world * dt * dt;
    let velocity = state.velocity + world_from_start * acceleration_integral + gravity_world * dt;

    Ok(Prediction {
        state: State {
            timestamp_ns: end_ns,
            rotation,
            position,
            velocity,
        },
        angular_velocity: endpoint_gyro,
        steps,
    })
}

fn seconds(duration_ns: i128) -> f64 {
    duration_ns as f64 * 1.0e-9
}

fn delta_q(delta_alpha: &Vector3<f64>) -> Quaternion<f64> {
    let half_norm = 0.5 * delta_alpha.norm();
    let vector = sinc(half_norm) * 0.5 * delta_alpha;
    Quaternion::new(half_norm.cos(), vector.x, vector.y, vector.z)
}

fn sinc(value: f64) -> f64 {
    if value.abs() > 1.0e-6 {
        value.sin() / value
    } else {
        let square = value * value;
        1.0 - square / 6.0 + square * square / 120.0 - square * square * square / 5040.0
    }
}

fn rotation_matrix(q: &Quaternion<f64>) -> Matrix3<f64> {
    let tx = 2.0 * q.i;
    let ty = 2.0 * q.j;
    let tz = 2.0 * q.k;
    let twx = tx * q.w;
    let twy = ty * q.w;
    let twz = tz * q.w;
    let txx = tx * q.i;
    let txy = ty * q.i;
    let txz = tz * q.i;
    let tyy = ty * q.j;
    let tyz = tz * q.j;
    let tzz = tz * q.k;
    Matrix3::new(
        1.0 - (tyy + tzz),
        txy - twz,
        txz + twy,
        txy + twz,
        1.0 - (txx + tzz),
        tyz - twx,
        txz - twy,
        tyz + twx,
        1.0 - (txx + tyy),
    )
}

fn interpolate_gyro(
    samples: &[ImuSample],
    time_ns: i128,
) -> Result<Vector3<f64>, PropagationError> {
    let Some(pair) = samples.windows(2).find(|pair| {
        pair[0].timestamp_ns < pair[1].timestamp_ns
            && pair[0].timestamp_ns <= time_ns
            && time_ns <= pair[1].timestamp_ns
    }) else {
        return Err(PropagationError::EndNotCovered);
    };
    let interval = pair[1].timestamp_ns - pair[0].timestamp_ns;
    if interval <= 0 {
        return Err(PropagationError::InvalidMeasurements);
    }
    let ratio = (time_ns - pair[0].timestamp_ns) as f64 / interval as f64;
    Ok(pair[0].gyro + (pair[1].gyro - pair[0].gyro) * ratio)
}
