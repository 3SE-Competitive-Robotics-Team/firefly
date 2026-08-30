//! 因子与鲁棒核（对照 `factors/`）。

pub mod general_factor;
pub mod gicp_factor;
pub mod icp_factor;
pub mod plane_icp_factor;
pub mod robust_kernel;

pub use general_factor::{GeneralFactor, NullFactor, RestrictDofFactor};
pub use gicp_factor::{GicpFactor, GicpSetting};
pub use icp_factor::{IcpFactor, IcpSetting};
pub use plane_icp_factor::{PlaneIcpFactor, PlaneIcpSetting};
pub use robust_kernel::{Cauchy, CauchySetting, Huber, HuberSetting};

/// C++ 命名别名（便于对照源码定位）。
pub type ICPFactor = IcpFactor;
pub type GICPFactor = GicpFactor;
pub type PointToPlaneICPFactor = PlaneIcpFactor;

use nalgebra::{Matrix4, Matrix6, Vector6};

use crate::ann::NearestNeighborSearch;
use crate::points::traits::PointCloudTrait;

/// 鲁棒因子包装（对照 `RobustFactor<Kernel, Factor>`）。
#[derive(Clone, Debug)]
pub struct RobustFactor<K, F> {
    /// 鲁棒核。
    pub robust_kernel: K,
    /// 内层因子。
    pub factor: F,
}

impl<K, F> RobustFactor<K, F> {
    /// 构造。
    pub fn new(robust_kernel: K, factor: F) -> Self {
        Self {
            robust_kernel,
            factor,
        }
    }
}

impl<K: Default, F: Default> Default for RobustFactor<K, F> {
    fn default() -> Self {
        Self {
            robust_kernel: K::default(),
            factor: F::default(),
        }
    }
}

/// Huber + GICP 便捷类型。
pub type HuberGicpFactor = RobustFactor<Huber, GicpFactor>;
/// Cauchy + GICP 便捷类型。
pub type CauchyGicpFactor = RobustFactor<Cauchy, GicpFactor>;

/// 因子 trait（供 `Reduction` 泛型约束）。
pub trait Factor {
    /// 线性化，返回是否内点。
    fn linearize<Target, Source, Tree, Rejector>(
        &mut self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        t: &Matrix4<f64>,
        source_index: usize,
        rejector: &Rejector,
        h: &mut Matrix6<f64>,
        b: &mut Vector6<f64>,
        e: &mut f64,
    ) -> bool
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
        Tree: NearestNeighborSearch,
        Rejector: Fn(&Target, &Source, &Matrix4<f64>, usize, usize, f64) -> bool;

    /// 误差。
    fn error<Target, Source>(&self, target: &Target, source: &Source, t: &Matrix4<f64>) -> f64
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait;

    /// 是否内点。
    fn is_inlier(&self) -> bool;
}

impl Factor for IcpFactor {
    fn linearize<Target, Source, Tree, Rejector>(
        &mut self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        t: &Matrix4<f64>,
        source_index: usize,
        rejector: &Rejector,
        h: &mut Matrix6<f64>,
        b: &mut Vector6<f64>,
        e: &mut f64,
    ) -> bool
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
        Tree: NearestNeighborSearch,
        Rejector: Fn(&Target, &Source, &Matrix4<f64>, usize, usize, f64) -> bool,
    {
        IcpFactor::linearize(
            self,
            target,
            source,
            target_tree,
            t,
            source_index,
            rejector,
            h,
            b,
            e,
        )
    }

    fn error<Target, Source>(&self, target: &Target, source: &Source, t: &Matrix4<f64>) -> f64
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        IcpFactor::error(self, target, source, t)
    }

    fn is_inlier(&self) -> bool {
        IcpFactor::is_inlier(self)
    }
}

impl Factor for PlaneIcpFactor {
    fn linearize<Target, Source, Tree, Rejector>(
        &mut self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        t: &Matrix4<f64>,
        source_index: usize,
        rejector: &Rejector,
        h: &mut Matrix6<f64>,
        b: &mut Vector6<f64>,
        e: &mut f64,
    ) -> bool
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
        Tree: NearestNeighborSearch,
        Rejector: Fn(&Target, &Source, &Matrix4<f64>, usize, usize, f64) -> bool,
    {
        PlaneIcpFactor::linearize(
            self,
            target,
            source,
            target_tree,
            t,
            source_index,
            rejector,
            h,
            b,
            e,
        )
    }

    fn error<Target, Source>(&self, target: &Target, source: &Source, t: &Matrix4<f64>) -> f64
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        PlaneIcpFactor::error(self, target, source, t)
    }

    fn is_inlier(&self) -> bool {
        PlaneIcpFactor::is_inlier(self)
    }
}

impl Factor for GicpFactor {
    fn linearize<Target, Source, Tree, Rejector>(
        &mut self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        t: &Matrix4<f64>,
        source_index: usize,
        rejector: &Rejector,
        h: &mut Matrix6<f64>,
        b: &mut Vector6<f64>,
        e: &mut f64,
    ) -> bool
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
        Tree: NearestNeighborSearch,
        Rejector: Fn(&Target, &Source, &Matrix4<f64>, usize, usize, f64) -> bool,
    {
        GicpFactor::linearize(
            self,
            target,
            source,
            target_tree,
            t,
            source_index,
            rejector,
            h,
            b,
            e,
        )
    }

    fn error<Target, Source>(&self, target: &Target, source: &Source, t: &Matrix4<f64>) -> f64
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        GicpFactor::error(self, target, source, t)
    }

    fn is_inlier(&self) -> bool {
        GicpFactor::is_inlier(self)
    }
}

impl<K, F> Factor for RobustFactor<K, F>
where
    K: RobustKernel + Clone,
    F: Factor + Clone,
{
    fn linearize<Target, Source, Tree, Rejector>(
        &mut self,
        target: &Target,
        source: &Source,
        target_tree: &Tree,
        t: &Matrix4<f64>,
        source_index: usize,
        rejector: &Rejector,
        h: &mut Matrix6<f64>,
        b: &mut Vector6<f64>,
        e: &mut f64,
    ) -> bool
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
        Tree: NearestNeighborSearch,
        Rejector: Fn(&Target, &Source, &Matrix4<f64>, usize, usize, f64) -> bool,
    {
        if !self.factor.linearize(
            target,
            source,
            target_tree,
            t,
            source_index,
            rejector,
            h,
            b,
            e,
        ) {
            return false;
        }
        let w = self.robust_kernel.weight(e.sqrt());
        *h *= w;
        *b *= w;
        *e *= w;
        true
    }

    fn error<Target, Source>(&self, target: &Target, source: &Source, t: &Matrix4<f64>) -> f64
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        let e = self.factor.error(target, source, t);
        self.robust_kernel.weight(e.sqrt()) * e
    }

    fn is_inlier(&self) -> bool {
        self.factor.is_inlier()
    }
}

/// 鲁棒核 trait（供 `RobustFactor` 约束）。
pub trait RobustKernel {
    /// 权重。
    fn weight(&self, e: f64) -> f64;
}

impl RobustKernel for Huber {
    fn weight(&self, e: f64) -> f64 {
        Huber::weight(self, e)
    }
}

impl RobustKernel for Cauchy {
    fn weight(&self, e: f64) -> f64 {
        Cauchy::weight(self, e)
    }
}
