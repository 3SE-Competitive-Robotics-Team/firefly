/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Object-safe residual evaluation and the central-difference Jacobian checker.

use crate::ceres::parameter_block::Manifold;
use nalgebra::{DMatrix, DVector};

#[derive(Clone, Debug)]
pub struct ErrorEvaluation {
    pub residuals: DVector<f64>,
    pub jacobians: Vec<Option<DMatrix<f64>>>,
    pub minimal_jacobians: Vec<Option<DMatrix<f64>>>,
}
/// Boolean masks correspond to the non-null Ceres output pointers.
pub trait ErrorInterface {
    fn residual_dim(&self) -> usize;
    fn parameter_blocks(&self) -> usize;
    fn parameter_block_dim(&self, block: usize) -> usize;
    fn evaluate_with_minimal_jacobians(
        &self,
        parameters: &[&[f64]],
        jacobians: Option<&[bool]>,
        jacobians_minimal: Option<&[bool]>,
    ) -> Option<ErrorEvaluation>;
    fn type_info(&self) -> &'static str;
}
/// The Ceres problem supplies residual lookup and parameter manifold metadata.
pub trait JacobianCheckProblem {
    type ResidualId: Copy;
    fn residual(&self, id: Self::ResidualId) -> Option<&dyn ErrorInterface>;
    fn parameter_blocks(&self, id: Self::ResidualId) -> Vec<&[f64]>;
    fn manifold(&self, id: Self::ResidualId, block: usize) -> Option<&dyn Manifold>;
}
pub fn jacobians_correct<P: JacobianCheckProblem>(
    problem: &P,
    id: P::ResidualId,
    tolerance: f64,
) -> bool {
    const DELTA: f64 = 1.0e-7;

    let Some(error) = problem.residual(id) else {
        return false;
    };
    let parameters = problem.parameter_blocks(id);
    if parameters.len() != error.parameter_blocks() {
        return false;
    }
    let requested = vec![true; parameters.len()];
    let Some(analytic) =
        error.evaluate_with_minimal_jacobians(&parameters, Some(&requested), Some(&requested))
    else {
        return false;
    };

    for i in 0..parameters.len() {
        let dim = error.parameter_block_dim(i);
        if parameters[i].len() != dim {
            return false;
        }
        let manifold = problem.manifold(id, i);
        let min_dim = manifold.map_or(dim, Manifold::tangent_size);
        let mut numerical = DMatrix::zeros(error.residual_dim(), min_dim);
        for j in 0..min_dim {
            let mut dp = vec![0.0; min_dim];
            dp[j] = DELTA;
            let mut dm = vec![0.0; min_dim];
            dm[j] = -DELTA;
            let mut pp = parameters[i].to_vec();
            let mut pm = pp.clone();
            if let Some(manifold) = manifold {
                manifold.plus(parameters[i], &dp, &mut pp);
                manifold.plus(parameters[i], &dm, &mut pm);
            } else {
                for k in 0..dim {
                    pp[k] += dp[k];
                    pm[k] += dm[k];
                }
            }
            let mut perturbed = parameters.clone();
            perturbed[i] = &pp;
            let Some(rp) = error.evaluate_with_minimal_jacobians(&perturbed, None, None) else {
                return false;
            };
            perturbed[i] = &pm;
            let Some(rm) = error.evaluate_with_minimal_jacobians(&perturbed, None, None) else {
                return false;
            };
            numerical.set_column(j, &((1.0 / (2.0 * DELTA)) * (rp.residuals - rm.residuals)));
        }
        let Some(Some(analytical)) = analytic.minimal_jacobians.get(i) else {
            return false;
        };
        if analytical.shape() != numerical.shape() {
            return false;
        }
        let is_approx = (analytical - &numerical).norm_squared()
            <= tolerance * tolerance * analytical.norm_squared().min(numerical.norm_squared());
        if !is_approx {
            log::error!(
                "analytical Jacobian {i}\n{analytical}\nvs num. diff. Jacobian\n{numerical}"
            );
            return false;
        }
    }
    true
}
