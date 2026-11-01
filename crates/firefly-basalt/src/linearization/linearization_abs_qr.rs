// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original `linearization_abs_qr.hpp/.cpp`, preserving per-landmark QR blocks.
use super::{
    imu_block::ImuBlock, landmark_block::RelPoseLin,
    landmark_block_abs_dynamic::LandmarkBlockAbsDynamic, linearization_base::Options,
};
use crate::{
    optimization::accumulator::DenseAccumulator,
    utils::{
        ba_utils::compute_rel_pose,
        imu_types::{AbsOrderMap, ImuLinData, MargLinData},
    },
    vi_estimator::ba_base::BundleAdjustmentBase,
};
use firefly_calibration::observation::TimeCamId;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector, Matrix4, SMatrix};
use rayon::prelude::*;
use std::collections::{BTreeSet, HashMap, HashSet};

pub struct LinearizationAbsQr<'a, S: BasaltScalar, const POSE_SIZE: usize> {
    landmark_blocks: Vec<LandmarkBlockAbsDynamic<S, POSE_SIZE>>,
    landmark_offsets: Vec<usize>,
    imu_blocks: Vec<ImuBlock<'a, S>>,
    relative_pose_lin: HashMap<(TimeCamId, TimeCamId), RelPoseLin<S>>,
    order: &'a AbsOrderMap,
    marginalization: Option<&'a MargLinData<S>>,
    pose_damping: S,
    pose_damping_sqrt: S,
    marg_scaling: DVector<S>,
    num_cameras: usize,
    num_rows: usize,
}
impl<'a, S: BasaltScalar, const POSE_SIZE: usize> LinearizationAbsQr<'a, S, POSE_SIZE> {
    #[must_use]
    #[expect(
        clippy::too_many_arguments,
        reason = "Original linearization factory inputs"
    )]
    /// # Panics
    /// Requires matching robust-loss settings, a host in the ordering, and valid observation links.
    pub fn new(
        estimator: &BundleAdjustmentBase<S>,
        order: &'a AbsOrderMap,
        options: Options<S>,
        marginalization: Option<&'a MargLinData<S>>,
        imu: Option<&'a ImuLinData<'a, S>>,
        used_frames: Option<&BTreeSet<i64>>,
        lost_landmarks: Option<&HashSet<usize>>,
        _last_state_to_marg: i64,
    ) -> Self {
        assert_eq!(options.lb_options.huber_parameter, estimator.huber_thresh);
        assert_eq!(options.lb_options.obs_std_dev, estimator.obs_std_dev);
        let mut relative_pose_lin = HashMap::new();
        for (&host, targets) in estimator.lmdb.observations() {
            assert!(!targets.is_empty());
            for (&target, observations) in targets {
                assert!(!observations.is_empty());
                relative_pose_lin.insert(
                    (host, target),
                    RelPoseLin {
                        t_t_h: Matrix4::zeros(),
                        d_rel_d_h: SMatrix::zeros(),
                        d_rel_d_t: SMatrix::zeros(),
                    },
                );
            }
        }
        let landmark_blocks: Vec<_> = estimator
            .lmdb
            .landmarks()
            .par_iter()
            .filter(|(id, point)| {
                used_frames.is_none() && lost_landmarks.is_none()
                    || used_frames.is_some_and(|frames| frames.contains(&point.host_kf_id.frame_id))
                    || lost_landmarks.is_some_and(|lost| lost.contains(id))
            })
            .map(|(&id, point)| {
                LandmarkBlockAbsDynamic::new(
                    id,
                    point,
                    &relative_pose_lin,
                    order,
                    options.lb_options,
                )
            })
            .collect();
        let mut num_rows = 0;
        let landmark_offsets = landmark_blocks
            .iter()
            .map(|block| {
                let start = num_rows;
                num_rows += block.num_q2_rows();
                start
            })
            .collect();
        let imu_blocks = imu.map_or_else(Vec::new, |data| {
            data.imu_meas
                .values()
                .map(|measurement| ImuBlock::new(measurement, data, order))
                .collect()
        });
        Self {
            landmark_blocks,
            landmark_offsets,
            imu_blocks,
            relative_pose_lin,
            order,
            marginalization,
            pose_damping: S::zero(),
            pose_damping_sqrt: S::zero(),
            marg_scaling: DVector::zeros(0),
            num_cameras: estimator.frame_poses.len(),
            num_rows,
        }
    }
    pub fn linearize_problem(&mut self, estimator: &BundleAdjustmentBase<S>) -> (S, bool) {
        self.pose_damping = S::zero();
        self.pose_damping_sqrt = S::zero();
        self.marg_scaling = DVector::zeros(0);
        for (&(host, target), relative) in &mut self.relative_pose_lin {
            if host == target {
                relative.t_t_h = Matrix4::identity();
                relative.d_rel_d_h.fill(S::zero());
                relative.d_rel_d_t.fill(S::zero());
            } else {
                let host_state = estimator.get_pose_state_with_lin(host.frame_id);
                let target_state = estimator.get_pose_state_with_lin(target.frame_id);
                let mut pose = compute_rel_pose(
                    host_state.pose_lin(),
                    &estimator.calib.t_i_c[host.cam_id],
                    target_state.pose_lin(),
                    &estimator.calib.t_i_c[target.cam_id],
                    Some(&mut relative.d_rel_d_h),
                    Some(&mut relative.d_rel_d_t),
                );
                if host_state.is_linearized() || target_state.is_linearized() {
                    pose = compute_rel_pose(
                        host_state.pose(),
                        &estimator.calib.t_i_c[host.cam_id],
                        target_state.pose(),
                        &estimator.calib.t_i_c[target.cam_id],
                        None,
                        None,
                    );
                }
                relative.t_t_h = crate::utils::sophus_utils::se3_matrix(&pose);
            }
        }
        let (mut error, valid) = self
            .landmark_blocks
            .par_iter_mut()
            .map(|block| {
                block.update_relative_poses(&self.relative_pose_lin);
                let error = block.linearize_landmark(
                    estimator.lmdb.landmark(block.landmark_id),
                    &estimator.calib,
                );
                (error, !block.is_numerical_failure())
            })
            .reduce(|| (S::zero(), true), |(a, av), (b, bv)| (a + b, av && bv));
        for block in &mut self.imu_blocks {
            error += block.linearize_imu(&estimator.frame_states);
        }
        if let Some(prior) = self.marginalization {
            error += estimator.compute_marg_prior_error(prior);
        }
        (error, valid)
    }
    pub fn perform_qr(&mut self) {
        self.landmark_blocks
            .par_iter_mut()
            .for_each(LandmarkBlockAbsDynamic::perform_qr);
    }
    /// # Panics
    /// Requires nonnegative damping.
    pub fn set_pose_damping(&mut self, lambda: S) {
        assert!(lambda >= S::zero());
        self.pose_damping = lambda;
        self.pose_damping_sqrt = lambda.sqrt();
    }
    #[must_use]
    pub fn has_pose_damping(&self) -> bool {
        self.pose_damping > S::zero()
    }
    /// # Panics
    /// Requires a pose increment matching the ordering.
    pub fn back_substitute(
        &mut self,
        estimator: &mut BundleAdjustmentBase<S>,
        increment: &DVector<S>,
    ) -> S {
        assert_eq!(increment.len(), self.order.total_size);
        let mut decrease = S::zero();
        for block in &mut self.landmark_blocks {
            block.back_substitute(
                estimator.lmdb.landmark_mut(block.landmark_id),
                increment,
                &mut decrease,
            );
        }
        for block in &self.imu_blocks {
            block.back_substitute(increment, &mut decrease);
        }
        if let Some(prior) = self.marginalization {
            decrease += estimator.compute_marg_prior_model_cost_change(
                prior,
                &self.marg_scaling,
                &increment.rows(0, prior.h.ncols()).into_owned(),
            );
        }
        decrease
    }
    #[must_use]
    pub fn get_jp_diag2(&self) -> DVector<S> {
        let mut result = self
            .landmark_blocks
            .par_iter()
            .fold(
                || DVector::zeros(self.order.total_size),
                |mut diagonal, block| {
                    block.add_pose_diagonal_squared(&mut diagonal);
                    diagonal
                },
            )
            .reduce(|| DVector::zeros(self.order.total_size), |a, b| a + b);
        for block in &self.imu_blocks {
            block.add_jp_diag2(&mut result);
        }
        if let Some(prior) = self.marginalization {
            for col in 0..prior.h.ncols() {
                let scaling = if self.marg_scaling.is_empty() {
                    S::one()
                } else {
                    self.marg_scaling[col]
                };
                result[col] += prior.h.column(col).map(|v| v * scaling).norm_squared();
            }
        }
        result
    }
    pub fn scale_jl_cols(&mut self) {
        self.landmark_blocks
            .par_iter_mut()
            .for_each(LandmarkBlockAbsDynamic::scale_landmark_columns);
    }
    /// # Panics
    /// Requires unscaled marginalization data and valid block states.
    pub fn scale_jp_cols(&mut self, scaling: &DVector<S>) {
        self.landmark_blocks
            .par_iter_mut()
            .for_each(|block| block.scale_pose_columns(scaling));
        for block in &mut self.imu_blocks {
            block.scale_jp_cols(scaling);
        }
        if let Some(prior) = self.marginalization {
            assert!(self.marg_scaling.is_empty());
            self.marg_scaling = scaling.rows(0, prior.h.ncols()).into_owned();
        }
    }
    pub fn set_landmark_damping(&mut self, lambda: S) {
        self.landmark_blocks
            .par_iter_mut()
            .for_each(|block| block.set_landmark_damping(lambda));
    }
    #[must_use]
    /// # Panics
    /// Requires a square-root marginalization prior and valid block dimensions.
    pub fn get_dense_q2jp_q2r(
        &self,
        estimator: &BundleAdjustmentBase<S>,
    ) -> (DMatrix<S>, DVector<S>) {
        let imu_start = self.num_rows;
        let damping_start = imu_start + self.imu_blocks.len() * 15;
        let marg_start = damping_start
            + if self.has_pose_damping() {
                self.order.total_size
            } else {
                0
            };
        let rows = marg_start + self.marginalization.map_or(0, |prior| prior.h.nrows());
        let mut jacobian = DMatrix::zeros(rows, self.order.total_size);
        let mut residual = DVector::zeros(rows);
        for (block, &offset) in self.landmark_blocks.iter().zip(&self.landmark_offsets) {
            block.get_dense_q2jp_q2r(&mut jacobian, &mut residual, offset);
        }
        for (index, block) in self.imu_blocks.iter().enumerate() {
            block.add_dense_q2jp_q2r(&mut jacobian, &mut residual, imu_start + index * 15);
        }
        if self.has_pose_damping() {
            for i in 0..self.num_cameras * POSE_SIZE {
                jacobian[(damping_start + i, i)] = self.pose_damping_sqrt;
            }
        }
        if let Some(prior) = self.marginalization {
            assert!(prior.is_sqrt);
            let delta = estimator.compute_delta(&prior.order);
            for col in 0..prior.h.ncols() {
                let scaling = if self.marg_scaling.is_empty() {
                    S::one()
                } else {
                    self.marg_scaling[col]
                };
                for row in 0..prior.h.nrows() {
                    jacobian[(marg_start + row, col)] = prior.h[(row, col)] * scaling;
                }
            }
            residual
                .rows_mut(marg_start, prior.h.nrows())
                .copy_from(&(&prior.h * delta + &prior.b));
        }
        (jacobian, residual)
    }
    #[must_use]
    /// # Panics
    /// Requires an unscaled marginalization prior.
    pub fn get_dense_h_b(&self, estimator: &BundleAdjustmentBase<S>) -> (DMatrix<S>, DVector<S>) {
        let size = self.order.total_size;
        let (mut hessian, mut rhs) = self
            .landmark_blocks
            .par_iter()
            .fold(
                || (DMatrix::zeros(size, size), DVector::zeros(size)),
                |(mut h, mut b), block| {
                    block.add_dense_h_b(&mut h, &mut b);
                    (h, b)
                },
            )
            .reduce(
                || (DMatrix::zeros(size, size), DVector::zeros(size)),
                |(h, b), (other_h, other_b)| (h + other_h, b + other_b),
            );
        let mut accumulator = DenseAccumulator::default();
        accumulator.reset(size);
        for block in &self.imu_blocks {
            block.add_dense_h_b(&mut accumulator);
        }
        hessian += accumulator.h();
        rhs += accumulator.b();
        if self.has_pose_damping() {
            for i in 0..size {
                hessian[(i, i)] += self.pose_damping;
            }
        }
        if let Some(prior) = self.marginalization {
            assert!(
                self.marg_scaling.is_empty(),
                "upstream prior H/b scaling is unsupported"
            );
            estimator.linearize_marg_prior(prior, self.order, &mut hessian, &mut rhs);
        }
        (hessian, rhs)
    }
}
