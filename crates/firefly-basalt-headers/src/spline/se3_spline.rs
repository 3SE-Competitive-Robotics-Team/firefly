// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `spline/se3_spline.h` at headers revision
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
use super::{
    rd_spline::RdSpline,
    so3_spline::{Jacobian as So3Jacobian, So3Spline},
    spline_common::Jacobian as PositionJacobian,
};
use crate::{
    calibration::calib_bias::{CalibAccelBias, CalibGyroBias},
    scalar::BasaltScalar,
    utils::sophus_utils::{
        hat, left_jacobian_so3, so3_exp, so3_inverse, so3_log, so3_matrix, so3_product,
    },
};
use nalgebra::{Isometry3, Matrix3, SMatrix, SVector, Translation3, UnitQuaternion, Vector3};

#[derive(Clone, Debug)]
pub struct AccelPosSo3Jacobian<S: BasaltScalar, const N: usize> {
    pub start_idx: usize,
    pub d_val_d_knot: [SMatrix<S, 3, 6>; N],
}
impl<S: BasaltScalar, const N: usize> Default for AccelPosSo3Jacobian<S, N> {
    fn default() -> Self {
        Self {
            start_idx: 0,
            d_val_d_knot: std::array::from_fn(|_| SMatrix::zeros()),
        }
    }
}
#[derive(Clone, Debug)]
pub struct PosePosSo3Jacobian<S: BasaltScalar, const N: usize> {
    pub start_idx: usize,
    pub d_val_d_knot: [SMatrix<S, 6, 6>; N],
}
impl<S: BasaltScalar, const N: usize> Default for PosePosSo3Jacobian<S, N> {
    fn default() -> Self {
        Self {
            start_idx: 0,
            d_val_d_knot: std::array::from_fn(|_| SMatrix::zeros()),
        }
    }
}

/// Decoupled Euclidean translation and cumulative SO(3) splines.
#[derive(Clone, Debug)]
pub struct Se3Spline<S: BasaltScalar, const N: usize> {
    position: RdSpline<S, 3, N>,
    rotation: So3Spline<S, N>,
    dt_ns: i64,
}
impl<S: BasaltScalar, const N: usize> Se3Spline<S, N> {
    #[must_use]
    pub fn new(interval: i64, start: i64) -> Self {
        Self {
            position: RdSpline::new(interval, start),
            rotation: So3Spline::new(interval, start),
            dt_ns: interval,
        }
    }
    /// # Panics
    /// Requires a nonnegative knot count.
    pub fn gen_random_trajectory(
        &mut self,
        count: i32,
        static_init: bool,
        mut sample: impl FnMut() -> S,
    ) {
        self.rotation.gen_random_trajectory(
            usize::try_from(count).expect("nonnegative knot count"),
            static_init,
            || Vector3::from_fn(|_, _| sample()),
        );
        self.position
            .gen_random_trajectory(count, static_init, sample);
    }
    pub fn set_knot(&mut self, pose: &Isometry3<S>, index: usize) {
        *self.rotation.knot_mut(index) = pose.rotation;
        *self.position.knot_mut(index) = pose.translation.vector;
    }
    pub fn set_knots(&mut self, pose: &Isometry3<S>, count: usize) {
        self.rotation.resize(count);
        self.position.resize(count);
        for index in 0..count {
            self.set_knot(pose, index);
        }
    }
    /// # Panics
    /// Requires equal knot intervals.
    pub fn copy_knots_from(&mut self, other: &Self) {
        assert_eq!(self.dt_ns, other.dt_ns);
        self.rotation.resize(other.num_knots());
        self.position.resize(other.num_knots());
        for i in 0..other.num_knots() {
            self.set_knot(&other.knot(i), i);
        }
    }
    pub fn knots_push_back(&mut self, knot: &Isometry3<S>) {
        self.rotation.knots_push_back(knot.rotation);
        self.position.knots_push_back(knot.translation.vector);
    }
    pub fn knots_pop_back(&mut self) {
        self.rotation.knots_pop_back();
        self.position.knots_pop_back();
    }
    #[must_use]
    pub fn knots_front(&self) -> Isometry3<S> {
        Isometry3::from_parts(
            Translation3::from(*self.position.knots_front()),
            *self.rotation.knots_front(),
        )
    }
    /// # Panics
    /// Requires nonempty, synchronized position and rotation splines.
    pub fn knots_pop_front(&mut self) {
        self.rotation.knots_pop_front();
        self.position.knots_pop_front();
        assert_eq!(self.rotation.min_time_ns(), self.position.min_time_ns());
        assert_eq!(self.rotation.num_knots(), self.position.num_knots());
    }
    #[must_use]
    /// # Panics
    /// Requires nonempty, synchronized position and rotation splines.
    pub fn last_knot(&self) -> Isometry3<S> {
        assert_eq!(self.rotation.num_knots(), self.position.num_knots());
        self.knot(self.num_knots() - 1)
    }
    #[must_use]
    pub fn knot(&self, index: usize) -> Isometry3<S> {
        Isometry3::from_parts(
            Translation3::from(*self.position.knot(index)),
            *self.rotation.knot(index),
        )
    }
    #[must_use]
    pub fn knot_so3(&self, index: usize) -> &UnitQuaternion<S> {
        self.rotation.knot(index)
    }
    pub fn knot_so3_mut(&mut self, index: usize) -> &mut UnitQuaternion<S> {
        self.rotation.knot_mut(index)
    }
    #[must_use]
    pub fn knot_pos(&self, index: usize) -> &Vector3<S> {
        self.position.knot(index)
    }
    pub fn knot_pos_mut(&mut self, index: usize) -> &mut Vector3<S> {
        self.position.knot_mut(index)
    }
    pub fn set_start_time_ns(&mut self, time: i64) {
        self.position.set_start_time_ns(time);
        self.rotation.set_start_time_ns(time);
    }
    pub fn apply_inc(&mut self, index: usize, increment: &SVector<S, 6>) {
        *self.position.knot_mut(index) += increment.fixed_rows::<3>(0);
        *self.rotation.knot_mut(index) = so3_product(
            &so3_exp(&increment.fixed_rows::<3>(3).into_owned()),
            self.rotation.knot(index),
        );
    }
    #[must_use]
    /// # Panics
    /// Requires synchronized spline bounds and sufficient knots.
    pub fn max_time_ns(&self) -> i64 {
        assert_eq!(self.rotation.max_time_ns(), self.position.max_time_ns());
        self.position.max_time_ns()
    }
    #[must_use]
    /// # Panics
    /// Requires synchronized spline start times.
    pub fn min_time_ns(&self) -> i64 {
        assert_eq!(self.rotation.min_time_ns(), self.position.min_time_ns());
        self.position.min_time_ns()
    }
    #[must_use]
    pub fn num_knots(&self) -> usize {
        self.position.num_knots()
    }
    #[must_use]
    pub const fn time_interval_ns(&self) -> i64 {
        self.dt_ns
    }
    #[must_use]
    pub fn trans_accel_world(&self, time: i64) -> Vector3<S> {
        self.position.acceleration(time, None)
    }
    #[must_use]
    pub fn trans_vel_world(&self, time: i64) -> Vector3<S> {
        self.position.velocity(time, None)
    }
    #[must_use]
    pub fn rot_vel_body(&self, time: i64) -> Vector3<S> {
        self.rotation.velocity_body(time, None)
    }
    #[must_use]
    pub fn pose(&self, time: i64, jacobian: Option<&mut PosePosSo3Jacobian<S, N>>) -> Isometry3<S> {
        if jacobian.is_none() {
            return Isometry3::from_parts(
                Translation3::from(self.position.evaluate::<0>(time, None)),
                self.rotation.evaluate(time, None),
            );
        }
        let mut rotation_jacobian = So3Jacobian::default();
        let mut position_jacobian = PositionJacobian::default();
        let rotation = self.rotation.evaluate(time, Some(&mut rotation_jacobian));
        let position = self
            .position
            .evaluate::<0>(time, Some(&mut position_jacobian));
        if let Some(j) = jacobian {
            let inverse = so3_matrix(&so3_inverse(&rotation));
            j.start_idx = rotation_jacobian.start_idx;
            for i in 0..N {
                j.d_val_d_knot[i].fill(S::zero());
                j.d_val_d_knot[i]
                    .fixed_view_mut::<3, 3>(0, 0)
                    .copy_from(&(inverse * position_jacobian.d_val_d_knot[i]));
                j.d_val_d_knot[i]
                    .fixed_view_mut::<3, 3>(3, 3)
                    .copy_from(&(inverse * rotation_jacobian.d_val_d_knot[i]));
            }
        }
        Isometry3::from_parts(Translation3::from(position), rotation)
    }
    #[must_use]
    pub fn d_pose_d_t(&self, time: i64) -> SVector<S, 6> {
        let mut result = SVector::<S, 6>::zeros();
        result.fixed_rows_mut::<3>(0).copy_from(
            &(so3_inverse(&self.rotation.evaluate(time, None)) * self.trans_vel_world(time)),
        );
        result
            .fixed_rows_mut::<3>(3)
            .copy_from(&self.rot_vel_body(time));
        result
    }
    #[must_use]
    pub fn gyro_residual(
        &self,
        time: i64,
        measurement: &Vector3<S>,
        bias: &CalibGyroBias<S>,
        knots: Option<&mut So3Jacobian<S, N>>,
        bias_jacobian: Option<&mut SMatrix<S, 3, 12>>,
    ) -> Vector3<S> {
        if let Some(j) = bias_jacobian {
            j.fill(S::zero());
            for i in 0..3 {
                j[(i, i)] = S::one();
                for axis in 0..3 {
                    j[(i, 3 + axis * 3 + i)] = -measurement[axis];
                }
            }
        }
        self.rotation.velocity_body(time, knots) - bias.calibrated(measurement)
    }
    #[must_use]
    /// # Panics
    /// Requires a supported timestamp and synchronized position and rotation knots.
    #[expect(
        clippy::too_many_arguments,
        reason = "Original optional residual Jacobian interface"
    )]
    pub fn accel_residual(
        &self,
        time: i64,
        measurement: &Vector3<S>,
        bias: &CalibAccelBias<S>,
        gravity: &Vector3<S>,
        knots: Option<&mut AccelPosSo3Jacobian<S, N>>,
        bias_jacobian: Option<&mut SMatrix<S, 3, 9>>,
        gravity_jacobian: Option<&mut Matrix3<S>>,
    ) -> Vector3<S> {
        if knots.is_none() && bias_jacobian.is_none() && gravity_jacobian.is_none() {
            return so3_inverse(&self.rotation.evaluate(time, None))
                * (self.position.acceleration(time, None) + gravity)
                - bias.calibrated(measurement);
        }
        let mut rotation_jacobian = So3Jacobian::default();
        let mut position_jacobian = PositionJacobian::default();
        let rotation = self.rotation.evaluate(time, Some(&mut rotation_jacobian));
        let acceleration = self
            .position
            .acceleration(time, Some(&mut position_jacobian));
        let inverse = so3_matrix(&so3_inverse(&rotation));
        let temporary = inverse * hat(&(acceleration + gravity));
        assert_eq!(rotation_jacobian.start_idx, position_jacobian.start_idx);
        assert_eq!(self.position.num_knots(), self.rotation.num_knots());
        if let Some(j) = knots {
            j.start_idx = position_jacobian.start_idx;
            for i in 0..N {
                j.d_val_d_knot[i]
                    .fixed_view_mut::<3, 3>(0, 0)
                    .copy_from(&(inverse * position_jacobian.d_val_d_knot[i]));
                j.d_val_d_knot[i]
                    .fixed_view_mut::<3, 3>(0, 3)
                    .copy_from(&(temporary * rotation_jacobian.d_val_d_knot[i]));
            }
        }
        if let Some(j) = bias_jacobian {
            j.fill(S::zero());
            for i in 0..3 {
                j[(i, i)] = S::one();
                j[(i, i + 3)] = -measurement[0];
            }
            j[(1, 6)] = -measurement[1];
            j[(2, 7)] = -measurement[1];
            j[(2, 8)] = -measurement[2];
        }
        if let Some(j) = gravity_jacobian {
            *j = inverse;
        }
        inverse * (acceleration + gravity) - bias.calibrated(measurement)
    }
    #[must_use]
    pub fn position_residual(
        &self,
        time: i64,
        measurement: &Vector3<S>,
        jacobian: Option<&mut PositionJacobian<S, N>>,
    ) -> Vector3<S> {
        self.position.evaluate::<0>(time, jacobian) - measurement
    }
    #[must_use]
    pub fn orientation_residual(
        &self,
        time: i64,
        measurement: &UnitQuaternion<S>,
        mut jacobian: Option<&mut So3Jacobian<S, N>>,
    ) -> Vector3<S> {
        let result = so3_log(&so3_product(
            &self.rotation.evaluate(time, jacobian.as_deref_mut()),
            &so3_inverse(measurement),
        ));
        if let Some(j) = jacobian {
            let rotation_jacobian = left_jacobian_so3(&result);
            for value in &mut j.d_val_d_knot {
                *value = rotation_jacobian * (*value);
            }
        }
        result
    }
    /// Writes the upstream knot diagnostics to a caller-selected sink.
    /// # Errors
    /// Propagates errors from the supplied formatting sink.
    pub fn write_knots(
        &self,
        output: &mut impl std::fmt::Write,
        positions_only: bool,
    ) -> std::fmt::Result {
        for i in 0..self.num_knots() {
            if positions_only {
                writeln!(output, "{}", self.position.knot(i).transpose())?;
            } else {
                writeln!(
                    output,
                    "{i}: p:{} q: {}",
                    self.position.knot(i).transpose(),
                    self.rotation.knot(i).quaternion().coords.transpose()
                )?;
            }
        }
        Ok(())
    }
}
