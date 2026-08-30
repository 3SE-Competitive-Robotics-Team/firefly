//! 高斯体素（对照 `ann/gaussian_voxelmap.hpp`）。
//!
//! 每个体素聚合均值与协方差（VGICP），`add` 累积变换后的点与协方差的和，
//! `finalize` 归一化。体素恒含 1 个点（均值）。

use nalgebra::{Matrix4, Vector4};

use crate::points::traits::PointCloudTrait;

/// 高斯体素设定（空，对照 `GaussianVoxel::Setting`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct GaussianVoxelSetting;

/// 高斯体素：均值 + 协方差（对照 `GaussianVoxel`）。
#[derive(Clone, Debug)]
pub struct GaussianVoxel {
    finalized: bool,
    num_points: usize,
    mean: Vector4<f64>,
    cov: Matrix4<f64>,
}

impl Default for GaussianVoxel {
    fn default() -> Self {
        Self {
            finalized: false,
            num_points: 0,
            mean: Vector4::zeros(),
            cov: Matrix4::zeros(),
        }
    }
}

impl GaussianVoxel {
    /// 新建空体素。
    pub fn new() -> Self {
        Self::default()
    }

    /// 体素内点数（恒 1，对照 `size() == 1`）。
    pub fn size(&self) -> usize {
        1
    }

    /// 是否已归一化。
    pub fn is_finalized(&self) -> bool {
        self.finalized
    }

    /// 均值。
    pub fn mean(&self) -> Vector4<f64> {
        self.mean
    }

    /// 协方差。
    pub fn cov(&self) -> Matrix4<f64> {
        self.cov
    }

    /// 加入一点（对照 `add`）。
    ///
    /// - `transformed_pt`：`T * points[i]`；
    /// - `points`：源点云；
    /// - `i`：源索引；
    /// - `t`：变换矩阵。
    pub fn add<P: PointCloudTrait>(
        &mut self,
        _setting: &GaussianVoxelSetting,
        transformed_pt: &Vector4<f64>,
        points: &P,
        i: usize,
        t: &Matrix4<f64>,
    ) {
        if self.finalized {
            self.finalized = false;
            self.mean *= self.num_points as f64;
            self.cov *= self.num_points as f64;
        }
        self.num_points += 1;
        self.mean += transformed_pt;
        self.cov += t * points.cov(i) * t.transpose();
    }

    /// 归一化均值与协方差（对照 `finalize`）。
    pub fn finalize(&mut self) {
        if self.finalized {
            return;
        }
        if self.num_points == 0 {
            return;
        }
        let n = self.num_points as f64;
        self.mean /= n;
        self.cov /= n;
        self.finalized = true;
    }

    /// 取第 0 个点（均值）。
    pub fn point(&self, _i: usize) -> Vector4<f64> {
        self.mean
    }

    /// 取第 0 个协方差。
    pub fn cov_at(&self, _i: usize) -> Matrix4<f64> {
        self.cov
    }

    /// 最近邻（恒 1）。
    pub fn nearest_neighbor_search(
        &self,
        pt: &Vector4<f64>,
        k_index: &mut usize,
        k_sq_dist: &mut f64,
    ) -> usize {
        *k_index = 0;
        *k_sq_dist = (self.mean - pt).norm_squared();
        1
    }

    /// k 近邻（恒 1）。
    pub fn knn_search(
        &self,
        pt: &Vector4<f64>,
        _k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        self.nearest_neighbor_search(pt, &mut k_indices[0], &mut k_sq_dists[0])
    }

    /// 压入结果容器（对照 `Traits::knn_search`）。
    pub fn knn_search_result(
        &self,
        pt: &Vector4<f64>,
        result: &mut crate::ann::knn_result::KnnResult,
    ) {
        result.push(0, (self.mean - pt).norm_squared());
    }
}

impl crate::points::traits::PointCloudTrait for GaussianVoxel {
    fn num_points(&self) -> usize {
        1
    }

    fn has_points(&self) -> bool {
        true
    }

    fn has_normals(&self) -> bool {
        false
    }

    fn has_covs(&self) -> bool {
        true
    }

    fn point(&self, i: usize) -> Vector4<f64> {
        self.point(i)
    }

    fn normal(&self, _i: usize) -> Vector4<f64> {
        Vector4::zeros()
    }

    fn cov(&self, i: usize) -> Matrix4<f64> {
        self.cov_at(i)
    }
}

impl crate::ann::NearestNeighborSearch for GaussianVoxel {
    fn knn_search(
        &self,
        query: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        self.knn_search(query, k, k_indices, k_sq_dists)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::points::traits::PointCloudMut;
    use nalgebra::Vector4;

    #[test]
    fn add_and_finalize() {
        let mut v = GaussianVoxel::new();
        let setting = GaussianVoxelSetting;
        let t = Matrix4::identity();
        let mut cloud = crate::points::point_cloud::PointCloud::new();
        cloud.resize(2);
        cloud.set_point(0, Vector4::new(1.0, 0.0, 0.0, 1.0));
        cloud.set_point(1, Vector4::new(3.0, 0.0, 0.0, 1.0));
        for c in 0..2 {
            let mut cov = Matrix4::zeros();
            cov[(0, 0)] = 1.0;
            cloud.set_cov(c, cov);
        }
        v.add(&setting, &cloud.point(0), &cloud, 0, &t);
        v.add(&setting, &cloud.point(1), &cloud, 1, &t);
        v.finalize();
        assert!((v.mean().x - 2.0).abs() < 1e-12);
        assert!((v.cov()[(0, 0)] - 1.0).abs() < 1e-12);
        // 再加入一点应反归一化
        v.add(&setting, &Vector4::new(5.0, 0.0, 0.0, 1.0), &cloud, 0, &t);
        assert!(!v.is_finalized());
        v.finalize();
        assert!((v.mean().x - 3.0).abs() < 1e-12);
    }
}
