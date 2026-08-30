//! GICP 因子（对照 `factors/gicp_factor.hpp`）。

use nalgebra::{Matrix4, Matrix6, Vector6};

use crate::ann::NearestNeighborSearch;
use crate::points::traits::PointCloudTrait;
use crate::util::lie::skew;

/// GICP 因子设定（空）。
#[derive(Clone, Copy, Debug, Default)]
pub struct GicpSetting;

/// GICP 因子（对照 `GICPFactor`）。
#[derive(Clone, Debug)]
pub struct GicpFactor {
    target_index: Option<usize>,
    source_index: Option<usize>,
    mahalanobis: Matrix4<f64>,
    _setting: GicpSetting,
}

impl Default for GicpFactor {
    fn default() -> Self {
        Self::new(GicpSetting)
    }
}

impl GicpFactor {
    /// 构造。
    pub fn new(setting: GicpSetting) -> Self {
        Self {
            target_index: None,
            source_index: None,
            mahalanobis: Matrix4::zeros(),
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

        // RCR = C_t + R C_s Rᵀ（仅 3×3），求逆得信息矩阵
        let cov_t = target.cov(k_index);
        let cov_s = source.cov(source_index);
        let rcr = cov_t + t * cov_s * t.transpose();
        let rcr3 = rcr.fixed_view::<3, 3>(0, 0).into_owned();
        let mah3 = rcr3
            .try_inverse()
            .unwrap_or_else(nalgebra::Matrix3::identity);
        self.mahalanobis = Matrix4::zeros();
        self.mahalanobis
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&mah3);

        let residual = target.point(k_index) - transed;

        let r = t.fixed_view::<3, 3>(0, 0).into_owned();
        let p_s = source.point(source_index).fixed_rows::<3>(0).into_owned();
        let skew_p = skew(&p_s);

        let mut j46 = nalgebra::Matrix::<
            f64,
            nalgebra::Const<4>,
            nalgebra::Const<6>,
            nalgebra::ArrayStorage<f64, 4, 6>,
        >::zeros();
        j46.fixed_view_mut::<3, 3>(0, 0).copy_from(&(r * skew_p));
        j46.fixed_view_mut::<3, 3>(0, 3).copy_from(&(-r));

        let j3 = j46.fixed_view::<3, 6>(0, 0).into_owned();
        let res3 = residual.fixed_rows::<3>(0).into_owned();
        // H = Jᵀ M J, b = Jᵀ M r, e = 0.5 rᵀ M r
        *h = j3.transpose() * mah3 * j3;
        *b = j3.transpose() * mah3 * res3;
        *e = 0.5 * res3.dot(&(mah3 * res3));

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
        let transed = t * source.point(si);
        let residual = target.point(ti) - transed;
        let res3 = residual.fixed_rows::<3>(0).into_owned();
        let mah3 = self.mahalanobis.fixed_view::<3, 3>(0, 0).into_owned();
        0.5 * res3.dot(&(mah3 * res3))
    }
}
