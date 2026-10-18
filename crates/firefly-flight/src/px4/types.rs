//! Shared types and math conventions for the PX4 control core.
//!
//! This module is a 1:1 port of the PX4-Autopilot control stack
//! (`4dbd2e069a5c30c2e53e47e842095d2576dc38c4`), keeping PX4's internal
//! **NED world frame / FRD body frame** conventions. Conversion to the
//! product's Z-up / FLU frames happens only at the outer boundary.
//!
//! Units: m, m/s, m/s², rad, rad/s, rad/s². Actuator commands are normalized
//! to `[0, 1]`; torque commands are normalized to `[-1, 1]`; thrust is
//! normalized by the vehicle's maximum collective thrust.

use nalgebra::{SMatrix, SVector, UnitQuaternion, Vector2, Vector3, Vector4};

/// Number of actuators supported by this product (a single X quad).
pub const NUM_ACTUATORS: usize = 4;
/// Control axes kept for this product: roll, pitch, yaw, `thrust_z`.
pub const NUM_AXES: usize = 4;

/// Axis order of [`ControlVector`]/[`EffectivenessMatrix`] rows.
pub const AXIS_ROLL: usize = 0;
pub const AXIS_PITCH: usize = 1;
pub const AXIS_YAW: usize = 2;
pub const AXIS_THRUST_Z: usize = 3;

pub type Vector2f = Vector2<f32>;
pub type Vector3f = Vector3<f32>;
pub type Vector4f = Vector4<f32>;
pub type ActuatorVector = SVector<f32, NUM_ACTUATORS>;
pub type ControlVector = SVector<f32, NUM_AXES>;
pub type EffectivenessMatrix = SMatrix<f32, NUM_AXES, NUM_ACTUATORS>;
pub type MixMatrix = SMatrix<f32, NUM_ACTUATORS, NUM_AXES>;
pub type Quatf = UnitQuaternion<f32>;

/// Standard gravity (m/s²), PX4's `CONSTANTS_ONE_G`.
pub const CONSTANTS_ONE_G: f32 = 9.80665;

/// PX4 `math::constrain`.
#[must_use]
pub fn constrain(value: f32, min: f32, max: f32) -> f32 {
    value.max(min).min(max)
}

/// PX4 `math::radians`.
#[must_use]
pub fn radians(degrees: f32) -> f32 {
    degrees * core::f32::consts::PI / 180.0
}

/// PX4 `math::degrees`.
#[must_use]
pub fn degrees(radians: f32) -> f32 {
    radians * 180.0 / core::f32::consts::PI
}

/// PX4 `math::sq`.
#[must_use]
pub fn sq(value: f32) -> f32 {
    value * value
}

/// PX4 `PX4_ISFINITE`.
#[must_use]
pub fn is_finite(value: f32) -> bool {
    value.is_finite()
}

/// Component-wise vector helpers mirroring `matrix::Vector3f`.
pub trait Vector3fExt {
    /// PX4 `emult`.
    #[must_use]
    fn emult(self, other: Self) -> Self;
    /// PX4 `isAllFinite`.
    #[must_use]
    fn is_all_finite(&self) -> bool;
    /// PX4 `nanToZero`.
    #[must_use]
    fn nan_to_zero(self) -> Self;
}

impl Vector3fExt for Vector3f {
    fn emult(self, other: Self) -> Self {
        self.component_mul(&other)
    }

    fn is_all_finite(&self) -> bool {
        self.iter().all(|v| v.is_finite())
    }

    fn nan_to_zero(self) -> Self {
        self.map(|v| if v.is_finite() { v } else { 0.0 })
    }
}

/// PX4 `Quatf` helpers. PX4 and nalgebra both use Hamilton convention.
#[must_use]
pub fn quat_canonical(q: Quatf) -> Quatf {
    if q.scalar() < 0.0 {
        UnitQuaternion::new_unchecked(-q.into_inner())
    } else {
        q
    }
}

/// Vector part of the quaternion (`x, y, z`), PX4 `Quatf::imag()`.
#[must_use]
pub fn quat_imag(q: Quatf) -> Vector3f {
    q.vector().into_owned()
}

/// PX4 `Quatf::dcm_z()`: body +z axis expressed in the world frame.
#[must_use]
pub fn quat_dcm_z(q: Quatf) -> Vector3f {
    q * Vector3::z()
}

/// PX4 `Quatf::rotateVector()`.
#[must_use]
pub fn quat_rotate(q: Quatf, v: Vector3f) -> Vector3f {
    q * v
}

/// PX4 `Quatf(v0, v1)`: shortest rotation taking `v0` to `v1`.
#[must_use]
pub fn quat_from_two_vectors(v0: Vector3f, v1: Vector3f) -> Quatf {
    UnitQuaternion::rotation_between(&v0, &v1).unwrap_or_else(UnitQuaternion::identity)
}

/// PX4 `Quatf(AxisAnglef(v))`: rotation vector (axis × angle).
#[must_use]
pub fn quat_from_rotation_vector(v: Vector3f) -> Quatf {
    UnitQuaternion::from_scaled_axis(v)
}

/// PX4 `Quatf::normalized()`.
#[must_use]
pub fn quat_normalize(q: Quatf) -> Quatf {
    UnitQuaternion::new_normalize(q.into_inner())
}

/// `trajectory_setpoint_s`: NAN means "not set".
#[derive(Clone, Copy, Debug)]
pub struct TrajectorySetpoint {
    pub position: Vector3f,
    pub velocity: Vector3f,
    pub acceleration: Vector3f,
    pub jerk: Vector3f,
    pub yaw: f32,
    pub yawspeed: f32,
}

impl TrajectorySetpoint {
    /// PX4 `PositionControl::empty_trajectory_setpoint`.
    pub const EMPTY: Self = Self {
        position: Vector3f::new(f32::NAN, f32::NAN, f32::NAN),
        velocity: Vector3f::new(f32::NAN, f32::NAN, f32::NAN),
        acceleration: Vector3f::new(f32::NAN, f32::NAN, f32::NAN),
        jerk: Vector3f::new(f32::NAN, f32::NAN, f32::NAN),
        yaw: f32::NAN,
        yawspeed: f32::NAN,
    };
}

/// `PositionControlStates`: current vehicle state (NED).
#[derive(Clone, Copy, Debug, Default)]
pub struct PositionControlStates {
    pub position: Vector3f,
    pub velocity: Vector3f,
    pub acceleration: Vector3f,
    pub yaw: f32,
}

/// `vehicle_attitude_setpoint_s` subset used by the control core.
#[derive(Clone, Copy, Debug)]
pub struct VehicleAttitudeSetpoint {
    pub q_d: Quatf,
    /// Thrust in body FRD, normalized by maximum collective thrust (negative z = up).
    pub thrust_body: Vector3f,
    pub yaw_sp_move_rate: f32,
}

impl Default for VehicleAttitudeSetpoint {
    fn default() -> Self {
        Self {
            q_d: UnitQuaternion::identity(),
            thrust_body: Vector3::zeros(),
            yaw_sp_move_rate: 0.0,
        }
    }
}

/// `vehicle_rates_setpoint_s` subset.
#[derive(Clone, Copy, Debug, Default)]
pub struct VehicleRatesSetpoint {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub thrust_body: Vector3f,
}

/// `vehicle_torque_setpoint_s` subset.
#[derive(Clone, Copy, Debug, Default)]
pub struct VehicleTorqueSetpoint {
    pub xyz: Vector3f,
}

/// `vehicle_thrust_setpoint_s` subset.
#[derive(Clone, Copy, Debug, Default)]
pub struct VehicleThrustSetpoint {
    pub xyz: Vector3f,
}

/// `rate_ctrl_status_s`: integrator terms for logging/diagnostics.
#[derive(Clone, Copy, Debug, Default)]
pub struct RateCtrlStatus {
    pub rollspeed_integ: f32,
    pub pitchspeed_integ: f32,
    pub yawspeed_integ: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quaternion_helpers_match_px4_semantics() {
        // canonical flips the antipodal representation.
        let q = UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.0, 4.0));
        let c = quat_canonical(q);
        assert!(c.scalar() >= 0.0);
        assert!((c * Vector3::x() - q * Vector3::x()).norm() < 1e-6);

        // dcm_z is the body +z direction in world.
        let q = UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.3, 0.0));
        assert!((quat_dcm_z(q) - q * Vector3::z()).norm() < 1e-6);

        // two-vector rotation maps v0 onto v1.
        let v0 = Vector3::new(0.0, 0.0, 1.0);
        let v1 = Vector3::new(0.2, -0.1, 0.97).normalize();
        assert!((quat_from_two_vectors(v0, v1) * v0 - v1).norm() < 1e-5);

        // rotation vector round trip.
        let v = Vector3::new(0.01, -0.02, 0.003);
        assert!((quat_from_rotation_vector(v).scaled_axis() - v).norm() < 1e-6);

        // imag is the vector part.
        let q = UnitQuaternion::from_scaled_axis(Vector3::new(0.1, 0.2, 0.3));
        assert!((quat_imag(q) - q.vector().into_owned()).norm() < 1e-6);
    }
}
