//! 配准主循环（对照 `registration/registration.hpp`）。

use nalgebra::Matrix4;

use crate::ann::NearestNeighborSearch;
use crate::factors::{Factor, GeneralFactor, NullFactor};
use crate::points::traits::PointCloudTrait;
use crate::registration::optimizer::{GaussNewtonOptimizer, LevenbergMarquardtOptimizer};
use crate::registration::reduction::SerialReduction;
use crate::registration::reduction_rayon::ParallelReduction;
use crate::registration::registration_result::RegistrationResult;
use crate::registration::rejector::{CorrespondenceRejector, DistanceRejector};
use crate::registration::termination_criteria::TerminationCriteria;

/// 配准器（对照 `Registration<PointFactor, Reduction, GeneralFactor, Rejector, Optimizer>`）。
///
/// 为保持与 C++ 模板参数对应，Rust 侧以泛型聚合全部可插拔组件；
/// 常用场景可直接使用 `Registration::default()`（GICP + 串行 + LM）。
pub struct Registration<
    F,
    R = SerialReduction,
    G = NullFactor,
    Rej = DistanceRejector,
    Opt = LevenbergMarquardtOptimizer,
> where
    F: Factor + Default + Clone,
{
    /// 终止判定。
    pub criteria: TerminationCriteria,
    /// 对应剔除器。
    pub rejector: Rej,
    /// 因子设定（用于批量构造 `factors`）。
    pub point_factor_setting: F,
    /// 通用因子。
    pub general_factor: G,
    /// 归约器。
    pub reduction: R,
    /// 优化器。
    pub optimizer: Opt,
    #[doc(hidden)]
    pub _phantom: std::marker::PhantomData<F>,
}

impl<F> Default
    for Registration<F, SerialReduction, NullFactor, DistanceRejector, LevenbergMarquardtOptimizer>
where
    F: Factor + Default + Clone,
{
    fn default() -> Self {
        Self {
            criteria: TerminationCriteria::default(),
            rejector: DistanceRejector::default(),
            point_factor_setting: F::default(),
            general_factor: NullFactor,
            reduction: SerialReduction,
            optimizer: LevenbergMarquardtOptimizer::default(),
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<F, R, G, Rej, Opt> Registration<F, R, G, Rej, Opt>
where
    F: Factor + Default + Clone,
    G: GeneralFactor + Default,
    Rej: CorrespondenceRejector + Default,
    Opt: Default,
{
    /// 新建。
    pub fn new() -> Self
    where
        Self: Default,
    {
        Self::default()
    }
}

// --- 串行 + GaussNewton 特化 ---

impl<F, G, Rej> Registration<F, SerialReduction, G, Rej, GaussNewtonOptimizer>
where
    F: Factor + Default + Clone,
    G: GeneralFactor + Clone,
    Rej: CorrespondenceRejector,
{
    /// 对齐（串行 + GN，对照 `align`）。
    pub fn align_serial_gn<Target, Source, Tree>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        init_t: &Matrix4<f64>,
    ) -> RegistrationResult
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
        Tree: NearestNeighborSearch,
    {
        if target.num_points() <= 10 {
            eprintln!(
                "warning: target point cloud is too small. |target|={}",
                target.num_points()
            );
        }
        if source.num_points() <= 10 {
            eprintln!(
                "warning: source point cloud is too small. |source|={}",
                source.num_points()
            );
        }

        let mut factors = vec![self.point_factor_setting.clone(); source.num_points()];
        self.optimizer.optimize_serial(
            target,
            source,
            target_tree,
            &self.rejector,
            &self.criteria,
            &self.reduction,
            init_t,
            &mut factors,
            &self.general_factor,
        )
    }
}

impl<F, G, Rej> Registration<F, SerialReduction, G, Rej, LevenbergMarquardtOptimizer>
where
    F: Factor + Default + Clone,
    G: GeneralFactor + Clone,
    Rej: CorrespondenceRejector,
{
    /// 对齐（串行 + LM）。
    pub fn align_serial<Target, Source, Tree>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        init_t: &Matrix4<f64>,
    ) -> RegistrationResult
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
        Tree: NearestNeighborSearch,
    {
        if target.num_points() <= 10 {
            eprintln!(
                "warning: target point cloud is too small. |target|={}",
                target.num_points()
            );
        }
        if source.num_points() <= 10 {
            eprintln!(
                "warning: source point cloud is too small. |source|={}",
                source.num_points()
            );
        }
        let mut factors = vec![self.point_factor_setting.clone(); source.num_points()];
        self.optimizer.optimize_serial(
            target,
            source,
            target_tree,
            &self.rejector,
            &self.criteria,
            &self.reduction,
            init_t,
            &mut factors,
            &self.general_factor,
        )
    }
}

// --- 并行 + LM/GN ---

impl<F, G, Rej> Registration<F, ParallelReduction, G, Rej, LevenbergMarquardtOptimizer>
where
    F: Factor + Default + Clone + Send + Sync,
    G: GeneralFactor + Clone + Sync,
    Rej: CorrespondenceRejector + Sync,
{
    /// 对齐（并行 + LM）。
    pub fn align_parallel<Target, Source, Tree>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        init_t: &Matrix4<f64>,
    ) -> RegistrationResult
    where
        Target: PointCloudTrait + Sync,
        Source: PointCloudTrait + Sync,
        Tree: NearestNeighborSearch + Sync,
    {
        if target.num_points() <= 10 {
            eprintln!(
                "warning: target point cloud is too small. |target|={}",
                target.num_points()
            );
        }
        if source.num_points() <= 10 {
            eprintln!(
                "warning: source point cloud is too small. |source|={}",
                source.num_points()
            );
        }
        let mut factors = vec![self.point_factor_setting.clone(); source.num_points()];
        self.optimizer.optimize_parallel(
            target,
            source,
            target_tree,
            &self.rejector,
            &self.criteria,
            &self.reduction,
            init_t,
            &mut factors,
            &self.general_factor,
        )
    }
}

impl<F, G, Rej> Registration<F, ParallelReduction, G, Rej, GaussNewtonOptimizer>
where
    F: Factor + Default + Clone + Send + Sync,
    G: GeneralFactor + Clone + Sync,
    Rej: CorrespondenceRejector + Sync,
{
    /// 对齐（并行 + GN）。
    pub fn align_parallel_gn<Target, Source, Tree>(
        &self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        init_t: &Matrix4<f64>,
    ) -> RegistrationResult
    where
        Target: PointCloudTrait + Sync,
        Source: PointCloudTrait + Sync,
        Tree: NearestNeighborSearch + Sync,
    {
        if target.num_points() <= 10 {
            eprintln!(
                "warning: target point cloud is too small. |target|={}",
                target.num_points()
            );
        }
        if source.num_points() <= 10 {
            eprintln!(
                "warning: source point cloud is too small. |source|={}",
                source.num_points()
            );
        }
        let mut factors = vec![self.point_factor_setting.clone(); source.num_points()];
        self.optimizer.optimize_parallel(
            target,
            source,
            target_tree,
            &self.rejector,
            &self.criteria,
            &self.reduction,
            init_t,
            &mut factors,
            &self.general_factor,
        )
    }
}

/// 便捷：默认 GICP + 串行 + LM 的对齐函数（不需显式构造 Registration）。
pub fn align_default_gicp_serial<Target, Source, Tree>(
    target: &Target,
    source: &Source,
    target_tree: &Tree,
    init_t: &Matrix4<f64>,
) -> RegistrationResult
where
    Target: PointCloudTrait,
    Source: PointCloudTrait,
    Tree: NearestNeighborSearch,
{
    let reg: Registration<crate::factors::GicpFactor> = Registration::default();
    reg.align_serial(target, source, target_tree, init_t)
}
