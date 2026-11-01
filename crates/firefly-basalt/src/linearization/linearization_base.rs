// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Shared configuration from original Basalt `linearization_base.hpp`.
use super::landmark_block;
use super::{
    linearization_abs_qr::LinearizationAbsQr, linearization_abs_sc::LinearizationAbsSc,
    linearization_rel_sc::LinearizationRelSc,
};
use crate::utils::vio_config::LinearizationType;
use crate::{
    utils::imu_types::{AbsOrderMap, ImuLinData, MargLinData},
    vi_estimator::ba_base::BundleAdjustmentBase,
};
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector};
use std::collections::{BTreeSet, HashSet};

#[derive(Clone, Copy, Debug)]
pub struct Options<S: BasaltScalar> {
    pub lb_options: landmark_block::Options<S>,
    pub linearization_type: LinearizationType,
}
#[must_use]
pub fn is_linearization_sqrt(kind: LinearizationType) -> bool {
    match kind {
        LinearizationType::AbsQr => true,
        LinearizationType::AbsSc | LinearizationType::RelSc => false,
    }
}

/// The three concrete implementations of the original virtual interface.
pub enum LinearizationBase<'a, S: BasaltScalar, const POSE_SIZE: usize = 6> {
    AbsQr(LinearizationAbsQr<'a, S, POSE_SIZE>),
    AbsSc(LinearizationAbsSc<'a, S, POSE_SIZE>),
    RelSc(LinearizationRelSc<'a, S, POSE_SIZE>),
}
macro_rules! dispatch{($value:expr,$method:ident $(,$arg:expr)*)=>{match $value{Self::AbsQr(value)=>value.$method($($arg),*),Self::AbsSc(value)=>value.$method($($arg),*),Self::RelSc(value)=>value.$method($($arg),*)}};}
impl<'a, S: BasaltScalar, const POSE_SIZE: usize> LinearizationBase<'a, S, POSE_SIZE> {
    #[must_use]
    #[expect(clippy::too_many_arguments, reason = "Original factory parameters")]
    pub fn create(
        estimator: &BundleAdjustmentBase<S>,
        order: &'a AbsOrderMap,
        options: Options<S>,
        marginalization: Option<&'a MargLinData<S>>,
        imu: Option<&'a ImuLinData<'a, S>>,
        used: Option<&BTreeSet<i64>>,
        lost: Option<&HashSet<usize>>,
        last_timestamp: i64,
    ) -> Self {
        match options.linearization_type {
            LinearizationType::AbsQr => Self::AbsQr(LinearizationAbsQr::new(
                estimator,
                order,
                options,
                marginalization,
                imu,
                used,
                lost,
                last_timestamp,
            )),
            LinearizationType::AbsSc => Self::AbsSc(LinearizationAbsSc::new(
                estimator,
                order,
                options,
                marginalization,
                imu,
                used,
                lost,
                last_timestamp,
            )),
            LinearizationType::RelSc => Self::RelSc(LinearizationRelSc::new(
                estimator,
                order,
                options,
                marginalization,
                imu,
                used,
                lost,
                last_timestamp,
            )),
        }
    }
    pub fn linearize_problem(&mut self, estimator: &BundleAdjustmentBase<S>) -> (S, bool) {
        dispatch!(self, linearize_problem, estimator)
    }
    pub fn perform_qr(&mut self) {
        dispatch!(self, perform_qr);
    }
    pub fn back_substitute(
        &mut self,
        estimator: &mut BundleAdjustmentBase<S>,
        increment: &DVector<S>,
    ) -> S {
        dispatch!(self, back_substitute, estimator, increment)
    }
    #[must_use]
    pub fn get_dense_h_b(&self, estimator: &BundleAdjustmentBase<S>) -> (DMatrix<S>, DVector<S>) {
        dispatch!(self, get_dense_h_b, estimator)
    }
    #[must_use]
    pub fn get_dense_q2jp_q2r(
        &self,
        estimator: &BundleAdjustmentBase<S>,
    ) -> (DMatrix<S>, DVector<S>) {
        dispatch!(self, get_dense_q2jp_q2r, estimator)
    }
    pub fn set_pose_damping(&mut self, lambda: S) {
        dispatch!(self, set_pose_damping, lambda);
    }
    #[must_use]
    pub fn has_pose_damping(&self) -> bool {
        dispatch!(self, has_pose_damping)
    }
    pub fn set_landmark_damping(&mut self, lambda: S) {
        dispatch!(self, set_landmark_damping, lambda);
    }
    pub fn scale_jl_cols(&mut self) {
        dispatch!(self, scale_jl_cols);
    }
    pub fn scale_jp_cols(&mut self, scaling: &DVector<S>) {
        dispatch!(self, scale_jp_cols, scaling);
    }
    #[must_use]
    pub fn get_jp_diag2(&self) -> DVector<S> {
        dispatch!(self, get_jp_diag2)
    }
}

pub(crate) fn select_observations(
    observations: &crate::vi_estimator::landmark_database::ObservationIndex,
    used: Option<&std::collections::BTreeSet<i64>>,
    lost: Option<&std::collections::HashSet<usize>>,
    last_timestamp: i64,
) -> crate::vi_estimator::landmark_database::ObservationIndex {
    let Some(used) = used else {
        return observations.clone();
    };
    let mut selected = crate::vi_estimator::landmark_database::ObservationIndex::new();
    for (&host, targets) in observations {
        let include_host = used.contains(&host.frame_id);
        let mut target_map = std::collections::BTreeMap::new();
        for (&target, ids) in targets {
            if target.frame_id > last_timestamp {
                continue;
            }
            if include_host {
                target_map.insert(target, ids.clone());
            } else if let Some(lost) = lost {
                let ids: std::collections::BTreeSet<_> =
                    ids.iter().filter(|id| lost.contains(id)).copied().collect();
                if !ids.is_empty() {
                    target_map.insert(target, ids);
                }
            }
        }
        if !target_map.is_empty() {
            selected.insert(host, target_map);
        }
    }
    selected
}
pub(crate) fn normal_to_square_root<S: BasaltScalar>(
    hessian: nalgebra::DMatrix<S>,
    rhs: &nalgebra::DVector<S>,
) -> (nalgebra::DMatrix<S>, nalgebra::DVector<S>) {
    use nalgebra::DMatrix;
    let size = hessian.nrows();
    let ldlt = crate::utils::ldlt::Ldlt::new(hessian);
    let diagonal = ldlt
        .diagonal()
        .map(|v| if v < S::zero() { S::zero() } else { v }.sqrt());
    let mut root = DMatrix::identity(size, size);
    ldlt.permute_rows(&mut root);
    root = ldlt.lower().transpose() * root;
    for row in 0..size {
        root.row_mut(row).scale_mut(diagonal[row]);
    }
    let mut result = DMatrix::from_column_slice(size, 1, rhs.as_slice());
    ldlt.permute_rows(&mut result);
    ldlt.solve_lower(&mut result);
    for row in 0..size {
        result[(row, 0)] = if diagonal[row] > S::min_positive_value().sqrt() {
            result[(row, 0)] / diagonal[row]
        } else {
            S::zero()
        };
    }
    (root, result.column(0).into_owned())
}
