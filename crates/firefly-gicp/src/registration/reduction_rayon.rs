//! 并行归约（对照 `registration/reduction_omp.hpp` / `reduction_tbb.hpp`，统一用 rayon）。

use nalgebra::{Matrix4, Matrix6, Vector6};
use rayon::prelude::*;

use crate::ann::NearestNeighborSearch;
use crate::factors::Factor;
use crate::points::traits::PointCloudTrait;
use crate::registration::rejector::CorrespondenceRejector;

/// 并行归约（rayon 版，对照 `ParallelReductionOMP/TBB`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct ParallelReduction {
    /// 线程数提示（0 表示使用 rayon 全局池）。
    pub num_threads: usize,
}

impl ParallelReduction {
    /// 线性化求和（并行，对照 `ParallelReductionOMP/TBB`）。
    pub fn linearize<Target, Source, Tree, Rej, F>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        rejector: &Rej,
        t: &Matrix4<f64>,
        factors: &mut [F],
    ) -> (Matrix6<f64>, Vector6<f64>, f64)
    where
        Target: PointCloudTrait + Sync,
        Source: PointCloudTrait + Sync,
        Tree: NearestNeighborSearch + Sync,
        Rej: CorrespondenceRejector + Sync,
        F: Factor + Send,
    {
        let len = factors.len();
        // 使用 par_iter_mut 并行线性化（rayon 保证 &mut 切片互斥）
        let results: Vec<Option<(Matrix6<f64>, Vector6<f64>, f64)>> = factors
            .par_iter_mut()
            .enumerate()
            .map(|(i, factor)| {
                let mut h = Matrix6::zeros();
                let mut b = Vector6::zeros();
                let mut e = 0.0;
                let reject_fn = |tgt: &Target,
                                 src: &Source,
                                 tf: &Matrix4<f64>,
                                 ti: usize,
                                 si: usize,
                                 d: f64| {
                    rejector.should_reject(tgt, src, tf, ti, si, d)
                };
                if factor.linearize(
                    target,
                    source,
                    target_tree,
                    t,
                    i,
                    &reject_fn,
                    &mut h,
                    &mut b,
                    &mut e,
                ) {
                    Some((h, b, e))
                } else {
                    None
                }
            })
            .collect();

        let mut sum_h = Matrix6::zeros();
        let mut sum_b = Vector6::zeros();
        let mut sum_e = 0.0;
        for r in results.into_iter().flatten() {
            sum_h += r.0;
            sum_b += r.1;
            sum_e += r.2;
        }
        let _ = len;
        (sum_h, sum_b, sum_e)
    }

    /// 误差求和（并行）。
    pub fn error<Target, Source, F>(
        &self,
        target: &Target,
        source: &Source,
        t: &Matrix4<f64>,
        factors: &[F],
    ) -> f64
    where
        Target: PointCloudTrait + Sync,
        Source: PointCloudTrait + Sync,
        F: Factor + Sync,
    {
        factors.par_iter().map(|f| f.error(target, source, t)).sum()
    }
}
