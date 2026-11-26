use super::{IMU_STALE_LIMIT, Inputs};
use firefly_flight::LandingHorizontalMotion;
use nalgebra::{UnitQuaternion, Vector3};

/// Use direct odometry when at least six established landmarks have binocular support.
const MIN_STEREO_LANDMARKS: u32 = 6;
const MAX_VISUAL_AGE_S: f64 = 0.15;
const MAX_INERTIAL_INTERVAL_S: f64 = 5.;

struct Anchor {
    visual_time: f64,
    imu_time: f64,
    velocity: Vector3<f32>,
    odom_from_standby: UnitQuaternion<f32>,
    impulse: Vector3<f64>,
    acceleration_error: Vector3<f64>,
    vertical: VerticalEstimate,
}

#[derive(Clone, Copy)]
pub(super) struct VerticalEstimate {
    pub time: f64,
    pub z: f32,
    pub vz: f32,
}

#[derive(Default)]
pub(super) struct Observer {
    anchor: Option<Anchor>,
    calibration: Calibration,
}

#[derive(Default)]
struct Calibration {
    start: Option<MotionSample>,
    fit: Option<(f64, Vector3<f64>)>,
}

#[derive(Clone, Copy)]
struct MotionSample {
    time: f64,
    impulse: Vector3<f64>,
    velocity: Vector3<f32>,
}

impl Calibration {
    // The difference of two velocity changes isolates acceleration error, including
    // gravity leakage from standby tilt. Actual approach braking is present in both.
    fn observe(
        &mut self,
        now: MotionSample,
        standby_from_odom: UnitQuaternion<f32>,
    ) -> Option<Vector3<f64>> {
        let start = *self.start.get_or_insert(now);
        let dt = now.time - start.time;
        if dt >= 1. {
            let measured = standby_from_odom * (now.velocity - start.velocity);
            let error = (now.impulse - start.impulse - measured.cast::<f64>()) / dt;
            self.fit = (dt <= 1.15 && error.norm() <= 0.15).then_some((now.time, error));
            self.start = Some(now);
        }
        self.fit
            .filter(|(t, _)| now.time - t <= 1.15)
            .map(|(_, e)| e)
    }
}

impl Observer {
    pub(super) fn vertical(&self) -> Option<VerticalEstimate> {
        self.anchor.as_ref().map(|a| a.vertical)
    }

    pub(super) fn observe(
        &mut self,
        inputs: &Inputs,
        landing_height: Option<f32>,
    ) -> LandingHorizontalMotion {
        use LandingHorizontalMotion::{InertialSpeed, Odometry, Unavailable};
        let (Some(imu), Some(odom), Some(vio)) =
            (inputs.attitude, inputs.estimate, inputs.primary.latest)
        else {
            *self = Self::default();
            return Unavailable;
        };
        if landing_height.is_none()
            || !inputs.primary.fresh(imu.timestamp)
            || !imu.ready
            || imu.received.elapsed() > IMU_STALE_LIMIT
            || !odom.initialized
            || odom.received_at.elapsed() > IMU_STALE_LIMIT
            || (odom.timestamp - imu.timestamp).abs() > IMU_STALE_LIMIT.as_secs_f64()
            || (odom.timestamp - vio.message.timestamp).abs() > IMU_STALE_LIMIT.as_secs_f64()
        {
            *self = Self::default();
            return Unavailable;
        }
        let posterior = vio.message.posterior_time;
        if vio.message.stereo_landmarks >= MIN_STEREO_LANDMARKS {
            if (imu.timestamp - posterior).abs() > MAX_VISUAL_AGE_S {
                *self = Self::default();
                return Unavailable;
            }
            let error = self.calibration.observe(
                MotionSample {
                    time: imu.timestamp,
                    impulse: imu.impulse,
                    velocity: odom.velocity,
                },
                imu.rotation / vio.rotation,
            );
            self.anchor = error
                .filter(|_| landing_height.is_some_and(|h| h.abs() <= 0.3))
                .map(|acceleration_error| Anchor {
                    visual_time: posterior,
                    imu_time: imu.timestamp,
                    acceleration_error,
                    velocity: odom.velocity,
                    odom_from_standby: vio.rotation / imu.rotation,
                    impulse: imu.impulse,
                    vertical: VerticalEstimate {
                        time: imu.timestamp,
                        z: odom.position.z,
                        vz: odom.velocity.z,
                    },
                });
            return Odometry;
        }
        self.calibration.start = None;
        let Some(anchor) = &mut self.anchor else {
            return Unavailable;
        };
        if !(0.0..=MAX_INERTIAL_INTERVAL_S).contains(&(imu.timestamp - anchor.visual_time)) {
            return Unavailable;
        }
        let dv = imu.impulse
            - anchor.impulse
            - anchor.acceleration_error * (imu.timestamp - anchor.imu_time);
        let velocity = anchor.velocity + anchor.odom_from_standby * dv.cast::<f32>();
        let vz = velocity.z;
        let dt = (imu.timestamp - anchor.vertical.time) as f32;
        anchor.vertical.z += anchor.vertical.vz.midpoint(vz) * dt;
        anchor.vertical.time = imu.timestamp;
        anchor.vertical.vz = vz;
        InertialSpeed(velocity.xy().norm())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_preserves_real_acceleration_in_a_different_heading_frame() {
        let mut calibration = Calibration::default();
        let rotation =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_2);
        let mut sample = MotionSample {
            time: 0.,
            impulse: Vector3::zeros(),
            velocity: Vector3::zeros(),
        };
        assert!(calibration.observe(sample, rotation).is_none());
        sample.time = 1.;
        sample.velocity.x = 0.4;
        sample.impulse = Vector3::new(0.04, 0.4, 0.);
        let error = calibration.observe(sample, rotation).unwrap();
        assert!((error - Vector3::new(0.04, 0., 0.)).norm() < 1e-6);
        sample.time = 2.;
        sample.impulse.y += 0.3;
        assert!(calibration.observe(sample, rotation).is_none());
    }
}
