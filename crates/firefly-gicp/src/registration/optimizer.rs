//! 优化器（对照 `registration/optimizer.hpp`）。

use nalgebra::{Matrix4, Matrix6, Vector6};

use crate::ann::NearestNeighborSearch;
use crate::factors::{Factor, GeneralFactor};
use crate::points::traits::PointCloudTrait;
use crate::registration::reduction::SerialReduction;
use crate::registration::reduction_rayon::ParallelReduction;
use crate::registration::registration_result::RegistrationResult;
use crate::registration::rejector::CorrespondenceRejector;
use crate::registration::termination_criteria::TerminationCriteria;
use crate::util::lie::se3_exp;

/// Gauss-Newton 优化器（对照 `GaussNewtonOptimizer`）。
#[derive(Clone, Copy, Debug)]
pub struct GaussNewtonOptimizer {
    /// 打印调试信息。
    pub verbose: bool,
    /// 最大迭代。
    pub max_iterations: usize,
    /// 阻尼系数 `lambda`（加在对角线上）。
    pub lambda: f64,
}

impl Default for GaussNewtonOptimizer {
    fn default() -> Self {
        Self {
            verbose: false,
            max_iterations: 20,
            lambda: 1e-6,
        }
    }
}

impl GaussNewtonOptimizer {
    /// 串行归约优化（主入口）。
    pub fn optimize_serial<Target, Source, Tree, Rej, F, G>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        rejector: &Rej,
        criteria: &TerminationCriteria,
        reduction: &SerialReduction,
        init_t: &Matrix4<f64>,
        factors: &mut [F],
        general_factor: &G,
    ) -> RegistrationResult
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
        Tree: NearestNeighborSearch,
        Rej: CorrespondenceRejector,
        F: Factor,
        G: GeneralFactor,
    {
        let mut result = RegistrationResult::new(*init_t);
        for i in 0..self.max_iterations {
            if result.converged {
                break;
            }
            let (mut h, mut b, mut e) = reduction.linearize(
                target,
                source,
                target_tree,
                rejector,
                &result.t_target_source,
                factors,
            );
            general_factor.update_linearized_system(
                target,
                source,
                target_tree,
                &result.t_target_source,
                &mut h,
                &mut b,
                &mut e,
            );

            let delta = solve_damped(&h, &b, self.lambda);

            if self.verbose {
                eprintln!(
                    "iter={i} e={e} lambda={} dt={} dr={}",
                    self.lambda,
                    delta.fixed_rows::<3>(3).norm(),
                    delta.fixed_rows::<3>(0).norm()
                );
            }

            result.converged = criteria.converged(&delta);
            result.t_target_source *= se3_exp(&delta);
            result.iterations = i;
            result.h = h;
            result.b = b;
            result.error = e;
        }
        result.num_inliers = factors.iter().filter(|f| f.is_inlier()).count();
        result
    }

    /// 并行归约优化（rayon）。
    pub fn optimize_parallel<Target, Source, Tree, Rej, F, G>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        rejector: &Rej,
        criteria: &TerminationCriteria,
        reduction: &ParallelReduction,
        init_t: &Matrix4<f64>,
        factors: &mut [F],
        general_factor: &G,
    ) -> RegistrationResult
    where
        Target: PointCloudTrait + Sync,
        Source: PointCloudTrait + Sync,
        Tree: NearestNeighborSearch + Sync,
        Rej: CorrespondenceRejector + Sync,
        F: Factor + Send + Sync,
        G: GeneralFactor + Sync,
    {
        let mut result = RegistrationResult::new(*init_t);
        for i in 0..self.max_iterations {
            if result.converged {
                break;
            }
            let (mut h, mut b, mut e) = reduction.linearize(
                target,
                source,
                target_tree,
                rejector,
                &result.t_target_source,
                factors,
            );
            general_factor.update_linearized_system(
                target,
                source,
                target_tree,
                &result.t_target_source,
                &mut h,
                &mut b,
                &mut e,
            );

            let delta = solve_damped(&h, &b, self.lambda);

            if self.verbose {
                eprintln!(
                    "iter={i} e={e} lambda={} dt={} dr={}",
                    self.lambda,
                    delta.fixed_rows::<3>(3).norm(),
                    delta.fixed_rows::<3>(0).norm()
                );
            }

            result.converged = criteria.converged(&delta);
            result.t_target_source *= se3_exp(&delta);
            result.iterations = i;
            result.h = h;
            result.b = b;
            result.error = e;
        }
        result.num_inliers = factors.iter().filter(|f| f.is_inlier()).count();
        result
    }
}

/// Levenberg-Marquardt 优化器（对照 `LevenbergMarquardtOptimizer`）。
#[derive(Clone, Copy, Debug)]
pub struct LevenbergMarquardtOptimizer {
    /// 打印调试信息。
    pub verbose: bool,
    /// 最大迭代。
    pub max_iterations: usize,
    /// 最大内层尝试。
    pub max_inner_iterations: usize,
    /// 初始 lambda。
    pub init_lambda: f64,
    /// lambda 增减因子。
    pub lambda_factor: f64,
}

impl Default for LevenbergMarquardtOptimizer {
    fn default() -> Self {
        Self {
            verbose: false,
            max_iterations: 20,
            max_inner_iterations: 10,
            init_lambda: 1e-3,
            lambda_factor: 10.0,
        }
    }
}

impl LevenbergMarquardtOptimizer {
    /// 串行归约优化。
    pub fn optimize_serial<Target, Source, Tree, Rej, F, G>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        rejector: &Rej,
        criteria: &TerminationCriteria,
        reduction: &SerialReduction,
        init_t: &Matrix4<f64>,
        factors: &mut [F],
        general_factor: &G,
    ) -> RegistrationResult
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
        Tree: NearestNeighborSearch,
        Rej: CorrespondenceRejector,
        F: Factor,
        G: GeneralFactor,
    {
        let mut lambda = self.init_lambda;
        let mut result = RegistrationResult::new(*init_t);

        for i in 0..self.max_iterations {
            if result.converged {
                break;
            }
            let (h, b, e) = reduction.linearize(
                target,
                source,
                target_tree,
                rejector,
                &result.t_target_source,
                factors,
            );
            let (mut cur_h, mut cur_b, mut cur_e) = (h, b, e);
            general_factor.update_linearized_system(
                target,
                source,
                target_tree,
                &result.t_target_source,
                &mut cur_h,
                &mut cur_b,
                &mut cur_e,
            );
            let cur_t = result.t_target_source;
            let mut cur_e_val = cur_e;

            let mut success = false;
            for j in 0..self.max_inner_iterations {
                let delta = solve_damped(&cur_h, &cur_b, lambda);
                let new_t = cur_t * se3_exp(&delta);
                let mut new_e = reduction.error(target, source, &new_t, factors);
                general_factor.update_error(target, source, &new_t, &mut new_e);

                if self.verbose {
                    eprintln!(
                        "iter={i} inner={j} e={cur_e_val} new_e={new_e} lambda={lambda} dt={} dr={}",
                        delta.fixed_rows::<3>(3).norm(),
                        delta.fixed_rows::<3>(0).norm()
                    );
                }

                if new_e <= cur_e_val {
                    result.converged = criteria.converged(&delta);
                    result.t_target_source = new_t;
                    lambda /= self.lambda_factor;
                    success = true;
                    cur_e_val = new_e;
                    result.h = cur_h;
                    result.b = cur_b;
                    result.error = cur_e_val;
                    break;
                } else {
                    lambda *= self.lambda_factor;
                }
            }

            result.iterations = i;
            // 若本轮 h/b 未被更新（success 情况下已更新），则补写
            if !success {
                result.h = cur_h;
                result.b = cur_b;
                result.error = cur_e_val;
                break;
            }
            // 同步当前 T 供下一轮 linearize
            let _ = cur_t;
        }

        result.num_inliers = factors.iter().filter(|f| f.is_inlier()).count();
        result
    }

    /// 并行归约优化。
    pub fn optimize_parallel<Target, Source, Tree, Rej, F, G>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        rejector: &Rej,
        criteria: &TerminationCriteria,
        reduction: &ParallelReduction,
        init_t: &Matrix4<f64>,
        factors: &mut [F],
        general_factor: &G,
    ) -> RegistrationResult
    where
        Target: PointCloudTrait + Sync,
        Source: PointCloudTrait + Sync,
        Tree: NearestNeighborSearch + Sync,
        Rej: CorrespondenceRejector + Sync,
        F: Factor + Send + Sync,
        G: GeneralFactor + Sync,
    {
        let mut lambda = self.init_lambda;
        let mut result = RegistrationResult::new(*init_t);

        for i in 0..self.max_iterations {
            if result.converged {
                break;
            }
            let (h, b, e) = reduction.linearize(
                target,
                source,
                target_tree,
                rejector,
                &result.t_target_source,
                factors,
            );
            let (mut cur_h, mut cur_b, mut cur_e) = (h, b, e);
            general_factor.update_linearized_system(
                target,
                source,
                target_tree,
                &result.t_target_source,
                &mut cur_h,
                &mut cur_b,
                &mut cur_e,
            );
            let cur_t = result.t_target_source;
            let mut cur_e_val = cur_e;

            let mut success = false;
            for j in 0..self.max_inner_iterations {
                let delta = solve_damped(&cur_h, &cur_b, lambda);
                let new_t = cur_t * se3_exp(&delta);
                let mut new_e = reduction.error(target, source, &new_t, factors);
                general_factor.update_error(target, source, &new_t, &mut new_e);

                if self.verbose {
                    eprintln!(
                        "iter={i} inner={j} e={cur_e_val} new_e={new_e} lambda={lambda} dt={} dr={}",
                        delta.fixed_rows::<3>(3).norm(),
                        delta.fixed_rows::<3>(0).norm()
                    );
                }

                if new_e <= cur_e_val {
                    result.converged = criteria.converged(&delta);
                    result.t_target_source = new_t;
                    lambda /= self.lambda_factor;
                    success = true;
                    cur_e_val = new_e;
                    result.h = cur_h;
                    result.b = cur_b;
                    result.error = cur_e_val;
                    break;
                } else {
                    lambda *= self.lambda_factor;
                }
            }

            result.iterations = i;
            if !success {
                result.h = cur_h;
                result.b = cur_b;
                result.error = cur_e_val;
                break;
            }
        }

        result.num_inliers = factors.iter().filter(|f| f.is_inlier()).count();
        result
    }
}

fn solve_damped(h: &Matrix6<f64>, b: &Vector6<f64>, lambda: f64) -> Vector6<f64> {
    let mut a = *h;
    for i in 0..6 {
        a[(i, i)] += lambda;
    }
    // 解 (H+λI) Δ = -b
    a.lu().solve(&(-b)).unwrap_or(Vector6::zeros())
}
