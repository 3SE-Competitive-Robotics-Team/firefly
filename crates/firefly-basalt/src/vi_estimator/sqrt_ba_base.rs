// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original `sqrt_ba_base.h/.cpp`: shared Schur operations and square-root prior diagnostics.
use super::sc_ba_base;
pub use super::sc_ba_base::{
    AbsLinData, FrameAbsLinData, FrameRelLinData, RelLinData, RelLinDataBase, linearize_abs,
    linearize_helper, linearize_helper_abs, linearize_rel, linearize_rel_to_abs, update_points,
    update_points_abs,
};
use crate::utils::imu_types::{MargLinData, PoseStateWithLin, PoseVelBiasStateWithLin};
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector};
use std::collections::BTreeMap;

/// The reference computes all diagnostic normal products in double precision.
fn normal_prior<S: BasaltScalar>(prior: &MargLinData<S>) -> (DMatrix<f64>, DVector<f64>) {
    let matrix = prior.h.map(BasaltScalar::as_f64);
    let residual = prior.b.map(BasaltScalar::as_f64);
    if prior.is_sqrt {
        (matrix.transpose() * &matrix, matrix.transpose() * residual)
    } else {
        (matrix, residual)
    }
}
/// Test the six gauge directions and a caller-supplied Eigen-style random vector.
/// # Panics
/// Requires a complete matching prior ordering and valid state dimensions.
#[must_use]
pub fn check_nullspace<S: BasaltScalar>(
    prior: &MargLinData<S>,
    states: &BTreeMap<i64, PoseVelBiasStateWithLin<S>>,
    poses: &BTreeMap<i64, PoseStateWithLin<S>>,
    random: &DVector<f64>,
) -> DVector<f64> {
    assert_eq!(prior.h.ncols(), prior.order.total_size);
    let (h, b) = normal_prior(prior);
    let directions = sc_ba_base::nullspace_increments(&prior.order, states, poses, random);
    DVector::from_fn(7, |i, _| {
        directions[i].dot(&(&h * &directions[i])) + directions[i].dot(&b)
    })
}
#[must_use]
pub fn check_eigenvalues<S: BasaltScalar>(prior: &MargLinData<S>) -> DVector<f64> {
    let (h, _) = normal_prior(prior);
    sc_ba_base::check_eigenvalues(&h)
}
