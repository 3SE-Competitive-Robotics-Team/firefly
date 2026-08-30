//! Point-to-Point ICP 因子（对照 `factors/icp_factor.hpp`）。

use nalgebra::{Matrix4, Matrix6, Vector4, Vector6};

use crate::ann::NearestNeighborSearch;
use crate::points::traits::PointCloudTrait;
use crate::util::lie::skew;

/// ICP 因子设定（空，对照 `ICPFactor::Setting`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct IcpSetting;

/// Point-to-Point 因子（对照 `ICPFactor`）。
#[derive(Clone, Debug)]
pub struct IcpFactor {
    target_index: Option<usize>,
    source_index: Option<usize>,
    _setting: IcpSetting,
}

impl Default for IcpFactor {
    fn default() -> Self {
        Self::new(IcpSetting)
    }
}

impl IcpFactor {
    /// 构造。
    pub fn new(setting: IcpSetting) -> Self {
        Self {
            target_index: None,
            source_index: None,
            _setting: setting,
        }
    }

    /// 是否为内点（对照 `inlier()`）。
    pub fn is_inlier(&self) -> bool {
        self.target_index.is_some()
    }

    /// 目标索引。
    pub fn target_index(&self) -> Option<usize> {
        self.target_index
    }

    /// 源索引。
    pub fn source_index(&self) -> Option<usize> {
        self.source_index
    }

    /// 线性化（对照 `linearize`）。
    ///
    /// 返回 `true` 若为内点并写入 `H,b,e`；否则返回 `false` 且不修改输出。
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

        let residual: Vector4<f64> = target.point(k_index) - transed;

        // J = [ R*skew(p_s) | -R ]  (4×6，末行零)
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

        // H = JᵀJ, b = Jᵀ residual, e = 0.5 ||residual||²（仅前三维参与）
        let j3 = j46.fixed_view::<3, 6>(0, 0).into_owned();
        let res3 = residual.fixed_rows::<3>(0).into_owned();
        *h = j3.transpose() * j3;
        *b = j3.transpose() * res3;
        *e = 0.5 * res3.norm_squared();

        true
    }

    /// 误差（对照 `error`，需先 `linearize` 成功）。
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
        0.5 * residual.fixed_rows::<3>(0).norm_squared()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ann::kdtree::KdTree;
    use crate::points::point_cloud::PointCloud;
    use crate::points::traits::PointCloudMut;
    use nalgebra::Vector4;

    fn dummy_rejector<T, S>(
        _t: &T,
        _s: &S,
        _m: &Matrix4<f64>,
        _ti: usize,
        _si: usize,
        _d: f64,
    ) -> bool {
        false
    }

    #[test]
    fn icp_linearize_identity() {
        let mut target = PointCloud::new();
        target.resize(3);
        for i in 0..3 {
            target.set_point(i, Vector4::new(i as f64, 0.0, 0.0, 1.0));
        }
        let mut source = PointCloud::new();
        source.resize(1);
        source.set_point(0, Vector4::new(0.1, 0.0, 0.0, 1.0));

        let tree: KdTree<PointCloud> = KdTree::new(target.clone());
        let t = Matrix4::identity();
        let mut factor = IcpFactor::default();
        let mut h = Matrix6::zeros();
        let mut b = Vector6::zeros();
        let mut e = 0.0;
        let ok = factor.linearize(
            &target,
            &source,
            &tree,
            &t,
            0,
            &dummy_rejector,
            &mut h,
            &mut b,
            &mut e,
        );
        assert!(ok);
        assert!(factor.is_inlier());
        // 最近点为 (0,0,0)，残差 (-0.1,0,0)，e = 0.5*0.01 =0.005
        assert!((e - 0.005).abs() < 1e-12);
    }
}
