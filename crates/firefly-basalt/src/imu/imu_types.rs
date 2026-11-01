// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are in ../../LICENSE.
//! Translation of `include/basalt/imu/imu_types.h` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
//! Local increments are `[translation, rotation, velocity, gyro_bias, accel_bias]`.

use crate::utils::sophus_utils::{so3_cast, so3_exp, so3_inverse, so3_log, so3_product};
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{Isometry3, SVector, Translation3, Vector3};

pub const POSE_SIZE: usize = 6;
pub const POSE_VEL_SIZE: usize = 9;
pub const POSE_VEL_BIAS_SIZE: usize = 15;

#[derive(Clone, Debug)]
pub struct PoseState<S: BasaltScalar> {
    pub t_ns: i64,
    /// Body-to-world transformation.
    pub t_w_i: Isometry3<S>,
}

impl<S: BasaltScalar> Default for PoseState<S> {
    fn default() -> Self {
        Self {
            t_ns: 0,
            t_w_i: Isometry3::identity(),
        }
    }
}

impl<S: BasaltScalar> PoseState<S> {
    #[must_use]
    pub fn new(t_ns: i64, t_w_i: Isometry3<S>) -> Self {
        Self { t_ns, t_w_i }
    }

    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> PoseState<T> {
        let cast = |value: S| T::from_f64_lossy(value.as_f64());
        PoseState::new(
            self.t_ns,
            Isometry3::from_parts(
                Translation3::from(self.t_w_i.translation.vector.map(cast)),
                so3_cast(&self.t_w_i.rotation),
            ),
        )
    }

    /// Adds world-frame translation and left-multiplies the rotation increment.
    pub fn apply_inc(&mut self, increment: &SVector<S, 6>) {
        Self::inc_pose(increment, &mut self.t_w_i);
    }

    pub fn inc_pose(increment: &SVector<S, 6>, pose: &mut Isometry3<S>) {
        pose.translation.vector += increment.fixed_rows::<3>(0);
        pose.rotation = so3_product(
            &so3_exp(&increment.fixed_rows::<3>(3).into_owned()),
            &pose.rotation,
        );
    }
}

/// Pose and world-frame velocity at one timestamp.
#[derive(Clone, Debug, Default)]
pub struct PoseVelState<S: BasaltScalar> {
    pub pose: PoseState<S>,
    pub vel_w_i: Vector3<S>,
}

impl<S: BasaltScalar> PoseVelState<S> {
    #[must_use]
    pub fn new(t_ns: i64, t_w_i: Isometry3<S>, vel_w_i: Vector3<S>) -> Self {
        Self {
            pose: PoseState::new(t_ns, t_w_i),
            vel_w_i,
        }
    }

    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> PoseVelState<T> {
        PoseVelState {
            pose: self.pose.cast(),
            vel_w_i: self.vel_w_i.map(|value| T::from_f64_lossy(value.as_f64())),
        }
    }

    pub fn apply_inc(&mut self, increment: &SVector<S, 9>) {
        self.pose
            .apply_inc(&increment.fixed_rows::<6>(0).into_owned());
        self.vel_w_i += increment.fixed_rows::<3>(6);
    }

    #[must_use]
    pub fn diff(&self, other: &Self) -> SVector<S, 9> {
        let mut result = SVector::<S, 9>::zeros();
        result
            .fixed_rows_mut::<3>(0)
            .copy_from(&(other.pose.t_w_i.translation.vector - self.pose.t_w_i.translation.vector));
        result
            .fixed_rows_mut::<3>(3)
            .copy_from(&so3_log(&so3_product(
                &other.pose.t_w_i.rotation,
                &so3_inverse(&self.pose.t_w_i.rotation),
            )));
        result
            .fixed_rows_mut::<3>(6)
            .copy_from(&(other.vel_w_i - self.vel_w_i));
        result
    }
}

/// Pose, velocity, and biases, using composition for the upstream inheritance.
#[derive(Clone, Debug, Default)]
pub struct PoseVelBiasState<S: BasaltScalar> {
    pub state: PoseVelState<S>,
    pub bias_gyro: Vector3<S>,
    pub bias_accel: Vector3<S>,
}

impl<S: BasaltScalar> PoseVelBiasState<S> {
    #[must_use]
    pub fn new(
        t_ns: i64,
        t_w_i: Isometry3<S>,
        vel_w_i: Vector3<S>,
        bias_gyro: Vector3<S>,
        bias_accel: Vector3<S>,
    ) -> Self {
        Self {
            state: PoseVelState::new(t_ns, t_w_i, vel_w_i),
            bias_gyro,
            bias_accel,
        }
    }

    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> PoseVelBiasState<T> {
        let cast = |value: S| T::from_f64_lossy(value.as_f64());
        PoseVelBiasState {
            state: self.state.cast(),
            bias_gyro: self.bias_gyro.map(cast),
            bias_accel: self.bias_accel.map(cast),
        }
    }

    pub fn apply_inc(&mut self, increment: &SVector<S, 15>) {
        self.state
            .apply_inc(&increment.fixed_rows::<9>(0).into_owned());
        self.bias_gyro += increment.fixed_rows::<3>(9);
        self.bias_accel += increment.fixed_rows::<3>(12);
    }

    #[must_use]
    pub fn diff(&self, other: &Self) -> SVector<S, 15> {
        let mut result = SVector::<S, 15>::zeros();
        result
            .fixed_rows_mut::<9>(0)
            .copy_from(&self.state.diff(&other.state));
        result
            .fixed_rows_mut::<3>(9)
            .copy_from(&(other.bias_gyro - self.bias_gyro));
        result
            .fixed_rows_mut::<3>(12)
            .copy_from(&(other.bias_accel - self.bias_accel));
        result
    }
}

/// Timestamped inertial measurements, in the sensor frame.
#[derive(Clone, Debug, Default)]
pub struct ImuData<S: BasaltScalar> {
    pub t_ns: i64,
    pub accel: Vector3<S>,
    pub gyro: Vector3<S>,
}

impl<S: BasaltScalar> ImuData<S> {
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> ImuData<T> {
        let cast = |value: S| T::from_f64_lossy(value.as_f64());
        ImuData {
            t_ns: self.t_ns,
            accel: self.accel.map(cast),
            gyro: self.gyro.map(cast),
        }
    }
}
