//! 归约（对照 `registration/reduction.hpp`）。

use nalgebra::{Matrix4, Matrix6, Vector6};

use crate::ann::NearestNeighborSearch;
use crate::factors::Factor;
use crate::points::traits::PointCloudTrait;
use crate::registration::rejector::CorrespondenceRejector;

/// 串行归约（对照 `SerialReduction`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct SerialReduction;

impl SerialReduction {
    /// 线性化求和（对照 `linearize`）。
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
        Target: PointCloudTrait,
        Source: PointCloudTrait,
        Tree: NearestNeighborSearch,
        Rej: CorrespondenceRejector,
        F: Factor,
    {
        let mut sum_h = Matrix6::zeros();
        let mut sum_b = Vector6::zeros();
        let mut sum_e = 0.0;

        // 将 rejector 封装为闭包以适配 Factor::linearize 的 `Fn` 约束
        let reject_fn =
            |tgt: &Target, src: &Source, tf: &Matrix4<f64>, ti: usize, si: usize, d: f64| {
                rejector.should_reject(tgt, src, tf, ti, si, d)
            };

        for (i, factor) in factors.iter_mut().enumerate() {
            let mut h = Matrix6::zeros();
            let mut b = Vector6::zeros();
            let mut e = 0.0;
            if !factor.linearize(
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
                continue;
            }
            sum_h += h;
            sum_b += b;
            sum_e += e;
        }
        (sum_h, sum_b, sum_e)
    }

    /// 误差求和（对照 `error`）。
    pub fn error<Target, Source, F>(
        &self,
        target: &Target,
        source: &Source,
        t: &Matrix4<f64>,
        factors: &[F],
    ) -> f64
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
        F: Factor,
    {
        let mut sum = 0.0;
        for f in factors {
            sum += f.error(target, source, t);
        }
        sum
    }
}
