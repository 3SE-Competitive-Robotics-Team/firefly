//! Point-to-Plane ICP 因子（对照 `factors/plane_icp_factor.hpp`）。

use nalgebra::{Matrix4, Matrix6, Vector6};

use crate::ann::NearestNeighborSearch;
use crate::points::traits::PointCloudTrait;
use crate::util::lie::skew;

/// Point-to-Plane 因子设定（空）。
#[derive(Clone, Copy, Debug, Default)]
pub struct PlaneIcpSetting;

/// Point-to-Plane 因子（对照 `PointToPlaneICPFactor`）。
#[derive(Clone, Debug)]
pub struct PlaneIcpFactor {
    target_index: Option<usize>,
    source_index: Option<usize>,
    _setting: PlaneIcpSetting,
}

impl Default for PlaneIcpFactor {
    fn default() -> Self {
        Self::new(PlaneIcpSetting)
    }
}

impl PlaneIcpFactor {
    /// 构造。
    pub fn new(setting: PlaneIcpSetting) -> Self {
        Self {
            target_index: None,
            source_index: None,
            _setting: setting,
        }
    }

    /// 是否为内点。
    pub fn is_inlier(&self) -> bool {
        self.target_index.is_some()
    }

    /// 线性化（对照 `linearize`）。
    pub fn linearize<Target, Source, Tree, Rejector>(
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
        self.source_index = Some(source_index);
        self.target_index = None;

        let transed = t * source.point(source_index);
        let mut k_index = 0usize;
        let mut k_sq_dist = 0.0f64;
        let found = target_tree.nearest_neighbor_search(&transed, &mut k_index, &mut k_sq_dist);
        if found == 0 || rejector(target, source, t, k_index, source_index, k_sq_dist) {
            return false;
        }
        self.target_index = Some(k_index);
        let target_normal = target.normal(k_index);
        let residual = target.point(k_index) - transed;
        // err = n ∘ residual（逐元素相乘）
        let err = target_normal.component_mul(&residual);

        let r = t.fixed_view::<3, 3>(0, 0).into_owned();
        let p_s = source.point(source_index).fixed_rows::<3>(0).into_owned();
        let skew_p = skew(&p_s);
        let n3 = target_normal.fixed_rows::<3>(0).into_owned();
        let n_diag = nalgebra::Matrix3::from_diagonal(&n3);

        let mut j46 = nalgebra::Matrix::<
            f64,
            nalgebra::Const<4>,
            nalgebra::Const<6>,
            nalgebra::ArrayStorage<f64, 4, 6>,
        >::zeros();
        j46.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(n_diag * r * skew_p));
        j46.fixed_view_mut::<3, 3>(0, 3).copy_from(&(n_diag * (-r)));

        let j3 = j46.fixed_view::<3, 6>(0, 0).into_owned();
        let err3 = err.fixed_rows::<3>(0).into_owned();
        *h = j3.transpose() * j3;
        *b = j3.transpose() * err3;
        *e = 0.5 * err3.norm_squared();
        true
    }

    /// 误差。
    pub fn error<Target, Source>(&self, target: &Target, source: &Source, t: &Matrix4<f64>) -> f64
    where
        Target: PointCloudTrait,
        Source: PointCloudTrait,
    {
        let Some(ti) = self.target_index else {
            return 0.0;
        };
        let Some(si) = self.source_index else {
            return 0.0;
        };
        let residual = target.point(ti) - t * source.point(si);
        let err = target.normal(ti).component_mul(&residual);
        0.5 * err.fixed_rows::<3>(0).norm_squared()
    }
}
