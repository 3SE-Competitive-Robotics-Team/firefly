// Source: original Basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc,
// include/basalt/linearization/imu_block.hpp; repository BSD-3-Clause license.
//! Whitened inertial and bias random-walk rows for two adjacent states.

use crate::imu::preintegration::IntegratedImuMeasurement;
use crate::imu::preintegration::Matrix9;
use crate::imu::preintegration::Matrix9x3;
use crate::{
    optimization::accumulator::DenseAccumulator,
    utils::imu_types::{AbsOrderMap, ImuLinData, PoseVelBiasStateWithLin},
};
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SVector};
use std::collections::BTreeMap;

pub struct ImuBlock<'a, S: BasaltScalar> {
    jp: DMatrix<S>,
    r: DVector<S>,
    measurement: &'a IntegratedImuMeasurement<S>,
    data: &'a ImuLinData<'a, S>,
    order: &'a AbsOrderMap,
}
impl<'a, S: BasaltScalar> ImuBlock<'a, S> {
    #[must_use]
    pub fn new(
        measurement: &'a IntegratedImuMeasurement<S>,
        data: &'a ImuLinData<'a, S>,
        order: &'a AbsOrderMap,
    ) -> Self {
        Self {
            jp: DMatrix::zeros(15, 30),
            r: DVector::zeros(15),
            measurement,
            data,
            order,
        }
    }
    fn timestamps(&self) -> [i64; 2] {
        [
            self.measurement.start_t_ns(),
            self.measurement
                .start_t_ns()
                .checked_add(self.measurement.dt_ns())
                .expect("IMU end timestamp overflow"),
        ]
    }
    fn offsets(&self) -> [usize; 2] {
        self.timestamps().map(|timestamp| {
            usize::try_from(self.order.abs_order_map[&timestamp].0).expect("negative state offset")
        })
    }

    /// Evaluates residuals at current state and Jacobians at stored linearization points.
    /// # Panics
    /// Requires both endpoint states and a positive measurement duration.
    #[allow(clippy::too_many_lines)]
    pub fn linearize_imu(&mut self, states: &BTreeMap<i64, PoseVelBiasStateWithLin<S>>) -> S {
        self.jp.fill(S::zero());
        self.r.fill(S::zero());
        let [start, end] = self.timestamps();
        let initial = &states[&start];
        let next = &states[&end];
        let mut initial_jacobian = Matrix9::zeros();
        let mut next_jacobian = Matrix9::zeros();
        let mut gyro_jacobian = Matrix9x3::zeros();
        let mut accel_jacobian = Matrix9x3::zeros();
        let lin = initial.state_lin();
        let mut residual = self.measurement.residual(
            &lin.state,
            self.data.g,
            &next.state_lin().state,
            &lin.bias_gyro,
            &lin.bias_accel,
            Some(&mut initial_jacobian),
            Some(&mut next_jacobian),
            Some(&mut gyro_jacobian),
            Some(&mut accel_jacobian),
        );
        if initial.is_linearized() || next.is_linearized() {
            let current = initial.state();
            residual = self.measurement.residual(
                &current.state,
                self.data.g,
                &next.state().state,
                &current.bias_gyro,
                &current.bias_accel,
                None,
                None,
                None,
                None,
            );
        }
        let sqrt_information = self.measurement.sqrt_cov_inv();
        let whitened = sqrt_information * residual;
        let half = S::from_f64_lossy(0.5);
        let imu_error = half * whitened.norm_squared();
        self.jp
            .fixed_view_mut::<9, 9>(0, 0)
            .copy_from(&(sqrt_information * initial_jacobian));
        self.jp
            .fixed_view_mut::<9, 9>(0, 15)
            .copy_from(&(sqrt_information * next_jacobian));
        self.jp
            .fixed_view_mut::<9, 3>(0, 9)
            .copy_from(&(sqrt_information * gyro_jacobian));
        self.jp
            .fixed_view_mut::<9, 3>(0, 12)
            .copy_from(&(sqrt_information * accel_jacobian));
        self.r.fixed_rows_mut::<9>(0).copy_from(&whitened);
        let dt = S::from_i64_lossy(self.measurement.dt_ns()) * S::from_f64_lossy(1e-9);
        let gyro_weight = self.data.gyro_bias_weight_sqrt / dt.sqrt();
        let gyro_residual = initial.state().bias_gyro - next.state().bias_gyro;
        self.jp
            .fixed_view_mut::<3, 3>(9, 9)
            .copy_from(&Matrix3::from_diagonal(&gyro_weight));
        self.jp
            .fixed_view_mut::<3, 3>(9, 24)
            .copy_from(&Matrix3::from_diagonal(&(-gyro_weight)));
        let whitened_gyro = gyro_weight.component_mul(&gyro_residual);
        self.r.fixed_rows_mut::<3>(9).copy_from(&whitened_gyro);
        let accel_weight = self.data.accel_bias_weight_sqrt / dt.sqrt();
        let accel_residual = initial.state().bias_accel - next.state().bias_accel;
        self.jp
            .fixed_view_mut::<3, 3>(12, 12)
            .copy_from(&Matrix3::from_diagonal(&accel_weight));
        self.jp
            .fixed_view_mut::<3, 3>(12, 27)
            .copy_from(&Matrix3::from_diagonal(&(-accel_weight)));
        let whitened_accel = accel_weight.component_mul(&accel_residual);
        self.r.fixed_rows_mut::<3>(12).copy_from(&whitened_accel);
        imu_error + half * whitened_gyro.norm_squared() + half * whitened_accel.norm_squared()
    }

    /// Adds these rows to an externally allocated square-root system.
    /// # Panics
    /// Requires valid state offsets and sufficient destination dimensions.
    pub fn add_dense_q2jp_q2r(
        &self,
        jacobian: &mut DMatrix<S>,
        residual: &mut DVector<S>,
        row_start: usize,
    ) {
        for (block, offset) in self.offsets().into_iter().enumerate() {
            for row in 0..15 {
                for col in 0..15 {
                    jacobian[(row_start + row, offset + col)] += self.jp[(row, block * 15 + col)];
                }
            }
        }
        for row in 0..15 {
            residual[row_start + row] += self.r[row];
        }
    }

    /// Adds the corresponding normal equations using the same state ordering.
    /// # Panics
    /// Requires valid state offsets and sufficient accumulator dimensions.
    pub fn add_dense_h_b(&self, accumulator: &mut DenseAccumulator<S>) {
        let h = self.jp.transpose() * &self.jp;
        let b = self.jp.transpose() * &self.r;
        let offsets = self.offsets();
        for row_block in 0..2 {
            for col_block in 0..2 {
                let block = SMatrix::<S, 15, 15>::from_fn(|row, col| {
                    h[(row_block * 15 + row, col_block * 15 + col)]
                });
                accumulator.add_h(offsets[row_block], offsets[col_block], &block);
            }
            let block = SVector::<S, 15>::from_fn(|row, _| b[row_block * 15 + row]);
            accumulator.add_b(offsets[row_block], &block);
        }
    }

    /// # Panics
    /// Requires valid state offsets and a scaling entry for each state coordinate.
    pub fn scale_jp_cols(&mut self, scaling: &DVector<S>) {
        for (block, offset) in self.offsets().into_iter().enumerate() {
            for col in 0..15 {
                for row in 0..15 {
                    self.jp[(row, block * 15 + col)] *= scaling[offset + col];
                }
            }
        }
    }

    /// # Panics
    /// Requires valid state offsets and enough output entries.
    pub fn add_jp_diag2(&self, diagonal: &mut DVector<S>) {
        for (block, offset) in self.offsets().into_iter().enumerate() {
            for col in 0..15 {
                diagonal[offset + col] += self.jp.column(block * 15 + col).norm_squared();
            }
        }
    }

    /// Accumulates the quadratic model's predicted decrease for the given update.
    /// # Panics
    /// Requires valid state offsets and enough update entries.
    pub fn back_substitute(&self, increment: &DVector<S>, model_decrease: &mut S) {
        let mut reduced = DVector::zeros(30);
        for (block, offset) in self.offsets().into_iter().enumerate() {
            reduced
                .rows_mut(block * 15, 15)
                .copy_from(&increment.rows(offset, 15));
        }
        let projected = &self.jp * reduced;
        *model_decrease -= projected.dot(&(&projected * S::from_f64_lossy(0.5) + &self.r));
    }
}
