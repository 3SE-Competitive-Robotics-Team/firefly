// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original Basalt `linearization_abs_sc.hpp/.cpp`.
use super::{
    imu_block::ImuBlock,
    linearization_base::{Options, normal_to_square_root, select_observations},
};
use crate::{
    optimization::accumulator::DenseAccumulator,
    utils::imu_types::{AbsOrderMap, ImuLinData, MargLinData},
    vi_estimator::{
        ba_base::BundleAdjustmentBase,
        sc_ba_base::{self, AbsLinData},
    },
};
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector};
use rayon::prelude::*;
use std::collections::{BTreeSet, HashSet};
pub struct LinearizationAbsSc<'a, S: BasaltScalar, const POSE_SIZE: usize> {
    data: Vec<AbsLinData<S>>,
    imu_blocks: Vec<ImuBlock<'a, S>>,
    order: &'a AbsOrderMap,
    marginalization: Option<&'a MargLinData<S>>,
    used: Option<BTreeSet<i64>>,
    lost: Option<HashSet<usize>>,
    last_timestamp: i64,
    pose_damping: S,
}
impl<'a, S: BasaltScalar, const POSE_SIZE: usize> LinearizationAbsSc<'a, S, POSE_SIZE> {
    #[must_use]
    #[expect(clippy::too_many_arguments, reason = "Original factory inputs")]
    /// # Panics
    /// Requires six-dimensional poses and matching robust-loss settings.
    pub fn new(
        estimator: &BundleAdjustmentBase<S>,
        order: &'a AbsOrderMap,
        options: Options<S>,
        marginalization: Option<&'a MargLinData<S>>,
        imu: Option<&'a ImuLinData<'a, S>>,
        used: Option<&BTreeSet<i64>>,
        lost: Option<&HashSet<usize>>,
        last_timestamp: i64,
    ) -> Self {
        assert_eq!(POSE_SIZE, 6);
        assert_eq!(options.lb_options.huber_parameter, estimator.huber_thresh);
        assert_eq!(options.lb_options.obs_std_dev, estimator.obs_std_dev);
        Self {
            data: Vec::new(),
            imu_blocks: imu.map_or_else(Vec::new, |data| {
                data.imu_meas
                    .values()
                    .map(|measurement| ImuBlock::new(measurement, data, order))
                    .collect()
            }),
            order,
            marginalization,
            used: used.cloned(),
            lost: lost.cloned(),
            last_timestamp,
            pose_damping: S::zero(),
        }
    }
    pub fn linearize_problem(&mut self, estimator: &BundleAdjustmentBase<S>) -> (S, bool) {
        self.pose_damping = S::zero();
        let observations = select_observations(
            estimator.lmdb.observations(),
            self.used.as_ref(),
            self.lost.as_ref(),
            self.last_timestamp,
        );
        let (data, mut error) = sc_ba_base::linearize_helper_abs(&observations, estimator);
        self.data = data;
        for block in &mut self.imu_blocks {
            error += block.linearize_imu(&estimator.frame_states);
        }
        if let Some(prior) = self.marginalization {
            error += estimator.compute_marg_prior_error(prior);
        }
        (error, true)
    }
    /// The original Schur path has no QR stage.
    pub fn perform_qr(&mut self) {}
    /// # Panics
    /// Requires nonnegative damping.
    pub fn set_pose_damping(&mut self, lambda: S) {
        assert!(lambda >= S::zero());
        self.pose_damping = lambda;
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
        let negated = -increment;
        for data in &self.data {
            sc_ba_base::update_points_abs(
                self.order,
                data,
                &negated,
                &mut estimator.lmdb,
                Some(&mut decrease),
            );
        }
        for block in &self.imu_blocks {
            block.back_substitute(increment, &mut decrease);
        }
        if let Some(prior) = self.marginalization {
            decrease += estimator.compute_marg_prior_model_cost_change(
                prior,
                &DVector::zeros(0),
                &increment.rows(0, prior.h.ncols()).into_owned(),
            );
        }
        decrease
    }
    #[must_use]
    pub fn get_dense_h_b(&self, estimator: &BundleAdjustmentBase<S>) -> (DMatrix<S>, DVector<S>) {
        let create = || {
            let mut accumulator = DenseAccumulator::default();
            accumulator.reset(self.order.total_size);
            accumulator
        };
        let mut accumulator = self
            .data
            .par_iter()
            .fold(create, |mut accumulator, data| {
                sc_ba_base::linearize_abs(data, self.order, &mut accumulator);
                accumulator
            })
            .reduce(create, |mut a, b| {
                a.join(&b);
                a
            });
        for block in &self.imu_blocks {
            block.add_dense_h_b(&mut accumulator);
        }
        let mut hessian = accumulator.h().clone();
        let mut rhs = accumulator.b().clone();
        if self.has_pose_damping() {
            for i in 0..hessian.nrows() {
                hessian[(i, i)] += self.pose_damping;
            }
        }
        if let Some(prior) = self.marginalization {
            estimator.linearize_marg_prior(prior, self.order, &mut hessian, &mut rhs);
        }
        (hessian, rhs)
    }
    #[must_use]
    pub fn get_dense_q2jp_q2r(
        &self,
        estimator: &BundleAdjustmentBase<S>,
    ) -> (DMatrix<S>, DVector<S>) {
        let (h, b) = self.get_dense_h_b(estimator);
        normal_to_square_root(h, &b)
    }
    /// # Panics
    /// This operation aborts in the original Schur implementation.
    pub fn get_jp_diag2(&self) -> DVector<S> {
        panic!("Original abs SC does not implement Jacobian scaling")
    }
    /// # Panics
    /// This operation aborts in the original Schur implementation.
    pub fn scale_jl_cols(&mut self) {
        panic!("Original abs SC does not implement landmark scaling")
    }
    /// # Panics
    /// This operation aborts in the original Schur implementation.
    pub fn scale_jp_cols(&mut self, _scaling: &DVector<S>) {
        panic!("Original abs SC does not implement pose scaling")
    }
    /// # Panics
    /// This operation aborts in the original Schur implementation.
    pub fn set_landmark_damping(&mut self, _lambda: S) {
        panic!("Original abs SC does not implement landmark damping")
    }
}
