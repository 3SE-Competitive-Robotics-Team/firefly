// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
// License notices and conditions are in ../../LICENSE and next to the LDLT helper.
//! Translation of `include/basalt/imu/preintegration.h` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.

use super::imu_types::{ImuData, PoseVelState};
use crate::{
    scalar::BasaltScalar,
    utils::sophus_utils::{
        hat, left_jacobian_inv_so3, right_jacobian_inv_so3, right_jacobian_so3, so3_exp,
        so3_inverse, so3_log, so3_matrix, so3_product,
    },
};
use nalgebra::{Matrix3, SMatrix, SVector, Vector3};
use std::sync::OnceLock;

pub type Matrix9<S> = SMatrix<S, 9, 9>;
pub type Matrix9x3<S> = SMatrix<S, 9, 3>;

/// Integrated inertial pseudo-measurement, with bias Jacobians and covariance.
#[derive(Clone, Debug)]
pub struct IntegratedImuMeasurement<S: BasaltScalar> {
    start_t_ns: i64,
    delta_state: PoseVelState<S>,
    cov: Matrix9<S>,
    sqrt_cov_inv: OnceLock<Matrix9<S>>,
    d_state_d_ba: Matrix9x3<S>,
    d_state_d_bg: Matrix9x3<S>,
    bias_gyro_lin: Vector3<S>,
    bias_accel_lin: Vector3<S>,
}

impl<S: BasaltScalar> Default for IntegratedImuMeasurement<S> {
    fn default() -> Self {
        Self::new(0, Vector3::zeros(), Vector3::zeros())
    }
}

impl<S: BasaltScalar> IntegratedImuMeasurement<S> {
    #[must_use]
    pub fn new(start_t_ns: i64, bias_gyro_lin: Vector3<S>, bias_accel_lin: Vector3<S>) -> Self {
        Self {
            start_t_ns,
            delta_state: PoseVelState::new(0, nalgebra::Isometry3::identity(), Vector3::zeros()),
            cov: Matrix9::zeros(),
            sqrt_cov_inv: OnceLock::new(),
            d_state_d_ba: Matrix9x3::zeros(),
            d_state_d_bg: Matrix9x3::zeros(),
            bias_gyro_lin,
            bias_accel_lin,
        }
    }

    /// Propagates without gravity; accelerations are rotated at the half-step.
    /// # Panics
    /// Requires a strictly later measurement timestamp and a representable interval.
    #[must_use]
    pub fn propagate_state(
        current: &PoseVelState<S>,
        data: &ImuData<S>,
        state_jacobian: Option<&mut Matrix9<S>>,
        accel_jacobian: Option<&mut Matrix9x3<S>>,
        gyro_jacobian: Option<&mut Matrix9x3<S>>,
    ) -> PoseVelState<S> {
        assert!(data.t_ns > current.pose.t_ns);
        let dt = S::from_i64_lossy(
            data.t_ns
                .checked_sub(current.pose.t_ns)
                .expect("IMU interval overflow"),
        ) * S::from_f64_lossy(1e-9);
        let half = S::from_f64_lossy(0.5);
        let half_rotation = so3_product(
            &current.pose.t_w_i.rotation,
            &so3_exp(&(data.gyro * (half * dt))),
        );
        let half_matrix = so3_matrix(&half_rotation);
        let accel_world = half_matrix * data.accel;
        let mut next = current.clone();
        next.pose.t_ns = data.t_ns;
        next.pose.t_w_i.rotation =
            so3_product(&current.pose.t_w_i.rotation, &so3_exp(&(data.gyro * dt)));
        next.vel_w_i = current.vel_w_i + accel_world * dt;
        next.pose.t_w_i.translation.vector = current.pose.t_w_i.translation.vector
            + current.vel_w_i * dt
            + accel_world * half * dt * dt;
        if let Some(jacobian) = state_jacobian {
            *jacobian = Matrix9::identity();
            jacobian
                .fixed_view_mut::<3, 3>(0, 6)
                .copy_from(&(Matrix3::identity() * dt));
            let velocity_rotation = hat(&(-accel_world * dt));
            jacobian
                .fixed_view_mut::<3, 3>(6, 3)
                .copy_from(&velocity_rotation);
            jacobian
                .fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&(velocity_rotation * dt * half));
        }
        if let Some(jacobian) = accel_jacobian {
            jacobian.fill(S::zero());
            jacobian
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&(half_matrix * half * dt * dt));
            jacobian
                .fixed_view_mut::<3, 3>(6, 0)
                .copy_from(&(half_matrix * dt));
        }
        if let Some(jacobian) = gyro_jacobian {
            jacobian.fill(S::zero());
            let jr = right_jacobian_so3(&(data.gyro * dt));
            let jr_half = right_jacobian_so3(&(data.gyro * (half * dt)));
            jacobian
                .fixed_view_mut::<3, 3>(3, 0)
                .copy_from(&(so3_matrix(&next.pose.t_w_i.rotation) * jr * dt));
            let velocity_gyro = hat(&(-accel_world * dt)) * half_matrix * jr_half * half * dt;
            jacobian
                .fixed_view_mut::<3, 3>(6, 0)
                .copy_from(&velocity_gyro);
            jacobian
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&(velocity_gyro * (half * dt)));
        }
        next
    }

    /// Adds one measurement with diagonal acceleration and angular-rate noise.
    /// # Panics
    /// Requires increasing timestamps relative to the integration start time.
    pub fn integrate(&mut self, data: &ImuData<S>, accel_cov: &Vector3<S>, gyro_cov: &Vector3<S>) {
        let corrected = ImuData {
            t_ns: data
                .t_ns
                .checked_sub(self.start_t_ns)
                .expect("relative IMU timestamp overflow"),
            accel: data.accel - self.bias_accel_lin,
            gyro: data.gyro - self.bias_gyro_lin,
        };
        let mut state_jacobian = Matrix9::zeros();
        let mut accel_jacobian = Matrix9x3::zeros();
        let mut gyro_jacobian = Matrix9x3::zeros();
        self.delta_state = Self::propagate_state(
            &self.delta_state,
            &corrected,
            Some(&mut state_jacobian),
            Some(&mut accel_jacobian),
            Some(&mut gyro_jacobian),
        );
        self.cov = state_jacobian * self.cov * state_jacobian.transpose()
            + accel_jacobian * Matrix3::from_diagonal(accel_cov) * accel_jacobian.transpose()
            + gyro_jacobian * Matrix3::from_diagonal(gyro_cov) * gyro_jacobian.transpose();
        self.sqrt_cov_inv.take();
        self.d_state_d_ba = -accel_jacobian + state_jacobian * self.d_state_d_ba;
        self.d_state_d_bg = -gyro_jacobian + state_jacobian * self.d_state_d_bg;
    }

    /// Predicts pose and velocity; the output timestamp is retained, as upstream.
    pub fn predict_state(
        &self,
        initial: &PoseVelState<S>,
        gravity: &Vector3<S>,
        output: &mut PoseVelState<S>,
    ) {
        let dt = S::from_i64_lossy(self.delta_state.pose.t_ns) * S::from_f64_lossy(1e-9);
        output.pose.t_w_i.rotation = so3_product(
            &initial.pose.t_w_i.rotation,
            &self.delta_state.pose.t_w_i.rotation,
        );
        output.vel_w_i =
            initial.vel_w_i + gravity * dt + initial.pose.t_w_i.rotation * self.delta_state.vel_w_i;
        output.pose.t_w_i.translation.vector = initial.pose.t_w_i.translation.vector
            + initial.vel_w_i * dt
            + gravity * S::from_f64_lossy(0.5) * dt * dt
            + initial.pose.t_w_i.rotation * self.delta_state.pose.t_w_i.translation.vector;
    }

    /// Unwhitened residual and optional analytic Jacobians in the upstream order.
    /// # Panics
    /// Panics if accelerometer bias affects the rotation block or the rotation
    /// residual violates the principal-logarithm range.
    // Preserve the source residual's optional-output API and bias notation.
    #[allow(clippy::too_many_arguments, clippy::similar_names)]
    pub fn residual(
        &self,
        initial: &PoseVelState<S>,
        gravity: &Vector3<S>,
        next: &PoseVelState<S>,
        current_bg: &Vector3<S>,
        current_ba: &Vector3<S>,
        state0_jacobian: Option<&mut Matrix9<S>>,
        state1_jacobian: Option<&mut Matrix9<S>>,
        bg_jacobian: Option<&mut Matrix9x3<S>>,
        ba_jacobian: Option<&mut Matrix9x3<S>>,
    ) -> SVector<S, 9> {
        let dt = S::from_i64_lossy(self.delta_state.pose.t_ns) * S::from_f64_lossy(1e-9);
        let gyro_diff = self.d_state_d_bg * (current_bg - self.bias_gyro_lin);
        let accel_diff = self.d_state_d_ba * (current_ba - self.bias_accel_lin);
        assert!(
            accel_diff
                .fixed_rows::<3>(3)
                .iter()
                .all(|value| *value == S::zero())
        );
        let rotation_inv = so3_matrix(&so3_inverse(&initial.pose.t_w_i.rotation));
        let position_delta = rotation_inv
            * (next.pose.t_w_i.translation.vector
                - initial.pose.t_w_i.translation.vector
                - initial.vel_w_i * dt
                - gravity * S::from_f64_lossy(0.5) * dt * dt);
        let velocity_delta = rotation_inv * (next.vel_w_i - initial.vel_w_i - gravity * dt);
        let rotation_residual = so3_log(&so3_product(
            &so3_product(
                &so3_product(
                    &so3_exp(&gyro_diff.fixed_rows::<3>(3).into_owned()),
                    &self.delta_state.pose.t_w_i.rotation,
                ),
                &so3_inverse(&next.pose.t_w_i.rotation),
            ),
            &initial.pose.t_w_i.rotation,
        ));
        let mut residual = SVector::<S, 9>::zeros();
        residual.fixed_rows_mut::<3>(0).copy_from(
            &(position_delta
                - (self.delta_state.pose.t_w_i.translation.vector
                    + gyro_diff.fixed_rows::<3>(0)
                    + accel_diff.fixed_rows::<3>(0))),
        );
        residual
            .fixed_rows_mut::<3>(3)
            .copy_from(&rotation_residual);
        residual.fixed_rows_mut::<3>(6).copy_from(
            &(velocity_delta
                - (self.delta_state.vel_w_i
                    + gyro_diff.fixed_rows::<3>(6)
                    + accel_diff.fixed_rows::<3>(6))),
        );
        if state0_jacobian.is_some() || state1_jacobian.is_some() {
            let jr_inv = right_jacobian_inv_so3(&rotation_residual);
            if let Some(jacobian) = state0_jacobian {
                jacobian.fill(S::zero());
                jacobian
                    .fixed_view_mut::<3, 3>(0, 0)
                    .copy_from(&(-rotation_inv));
                jacobian
                    .fixed_view_mut::<3, 3>(0, 3)
                    .copy_from(&(hat(&position_delta) * rotation_inv));
                jacobian
                    .fixed_view_mut::<3, 3>(3, 3)
                    .copy_from(&(jr_inv * rotation_inv));
                jacobian
                    .fixed_view_mut::<3, 3>(6, 3)
                    .copy_from(&(hat(&velocity_delta) * rotation_inv));
                jacobian
                    .fixed_view_mut::<3, 3>(0, 6)
                    .copy_from(&(-rotation_inv * dt));
                jacobian
                    .fixed_view_mut::<3, 3>(6, 6)
                    .copy_from(&(-rotation_inv));
            }
            if let Some(jacobian) = state1_jacobian {
                jacobian.fill(S::zero());
                jacobian
                    .fixed_view_mut::<3, 3>(0, 0)
                    .copy_from(&rotation_inv);
                jacobian
                    .fixed_view_mut::<3, 3>(3, 3)
                    .copy_from(&(-jr_inv * rotation_inv));
                jacobian
                    .fixed_view_mut::<3, 3>(6, 6)
                    .copy_from(&rotation_inv);
            }
        }
        if let Some(jacobian) = ba_jacobian {
            *jacobian = -self.d_state_d_ba;
        }
        if let Some(jacobian) = bg_jacobian {
            *jacobian = -self.d_state_d_bg;
            jacobian.fixed_view_mut::<3, 3>(3, 0).copy_from(
                &(left_jacobian_inv_so3(&rotation_residual)
                    * self.d_state_d_bg.fixed_view::<3, 3>(3, 0)),
            );
        }
        residual
    }

    #[must_use]
    pub fn dt_ns(&self) -> i64 {
        self.delta_state.pose.t_ns
    }
    #[must_use]
    pub fn start_t_ns(&self) -> i64 {
        self.start_t_ns
    }
    #[must_use]
    pub fn cov(&self) -> &Matrix9<S> {
        &self.cov
    }
    #[must_use]
    pub fn delta_state(&self) -> &PoseVelState<S> {
        &self.delta_state
    }
    #[must_use]
    pub fn d_state_d_ba(&self) -> &Matrix9x3<S> {
        &self.d_state_d_ba
    }
    #[must_use]
    pub fn d_state_d_bg(&self) -> &Matrix9x3<S> {
        &self.d_state_d_bg
    }
    #[must_use]
    pub fn sqrt_cov_inv(&self) -> &Matrix9<S> {
        self.sqrt_cov_inv
            .get_or_init(|| sqrt_inverse_ldlt(self.cov))
    }
    #[must_use]
    pub fn cov_inv(&self) -> Matrix9<S> {
        self.sqrt_cov_inv().transpose() * self.sqrt_cov_inv()
    }
}

// Real 9x9 specialization of Eigen 3.4.0 LDLT.h factorization and triangular solve.
// Copyright (C) 2008-2011 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2009 Keir Mierle <mierle@gmail.com>
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2011 Timothy E. Holy <tim.holy@gmail.com>
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain
// one at https://mozilla.org/MPL/2.0/.
fn sqrt_inverse_ldlt<S: BasaltScalar>(mut matrix: Matrix9<S>) -> Matrix9<S> {
    let mut swaps = std::array::from_fn::<_, 9, _>(|i| i);
    for k in 0..9 {
        let mut pivot = k;
        for j in k + 1..9 {
            if matrix[(j, j)].abs() > matrix[(pivot, pivot)].abs() {
                pivot = j;
            }
        }
        swaps[k] = pivot;
        if pivot != k {
            for j in 0..k {
                matrix.swap((k, j), (pivot, j));
            }
            for i in pivot + 1..9 {
                matrix.swap((i, k), (i, pivot));
            }
            matrix.swap((k, k), (pivot, pivot));
            for i in k + 1..pivot {
                matrix.swap((i, k), (pivot, i));
            }
        }
        let mut temp = SVector::<S, 9>::zeros();
        for j in 0..k {
            temp[j] = matrix[(j, j)] * matrix[(k, j)];
        }
        let mut update = S::zero();
        for j in 0..k {
            update += matrix[(k, j)] * temp[j];
        }
        matrix[(k, k)] -= update;
        for i in k + 1..9 {
            let mut update = S::zero();
            for j in 0..k {
                update += matrix[(i, j)] * temp[j];
            }
            matrix[(i, k)] -= update;
        }
        let diagonal = matrix[(k, k)];
        let valid = diagonal.abs() > S::zero();
        if k == 0 && !valid {
            swaps = std::array::from_fn(|i| i);
            break;
        }
        if valid {
            for i in k + 1..9 {
                matrix[(i, k)] /= diagonal;
            }
        }
    }
    let mut result = Matrix9::identity();
    for (row, &pivot) in swaps.iter().enumerate() {
        result.swap_rows(row, pivot);
    }
    for row in 0..9 {
        for column in 0..9 {
            let mut update = S::zero();
            for j in 0..row {
                update += matrix[(row, j)] * result[(j, column)];
            }
            result[(row, column)] -= update;
        }
    }
    let min_normal = S::min_positive_value();
    for row in 0..9 {
        let inverse = if matrix[(row, row)] < min_normal {
            S::zero()
        } else {
            S::one() / matrix[(row, row)].sqrt()
        };
        result.row_mut(row).apply(|value| *value *= inverse);
    }
    result
}
