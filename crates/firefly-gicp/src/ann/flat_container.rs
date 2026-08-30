//! 扁平点容器（对照 official `ann/flat_container.hpp`）。
//!
//! 单个体素内的点容器：每个 cell 至多存 `max_num_points_in_cell` 个点，且忽略与
//! 已有点过近的点（`min_sq_dist_in_cell`）。`add` 写入已变换后的点与属性；搜索接口
//! 与 KdTree 一致。本结构是增量体素地图（第三阶段）的底层存储。

use nalgebra::{Matrix4, Vector4};

use crate::ann::NearestNeighborSearch;
use crate::ann::knn_result::KnnResult;
use crate::points::traits::PointCloudTrait;

/// 扁平容器参数（对照 `FlatContainer::Setting`）。
#[derive(Clone, Copy, Debug)]
pub struct FlatContainerSetting {
    /// cell 内两点最小平方距离，过近则忽略。
    pub min_sq_dist_in_cell: f64,
    /// cell 内最大点数。
    pub max_num_points_in_cell: usize,
}

impl Default for FlatContainerSetting {
    fn default() -> Self {
        Self {
            min_sq_dist_in_cell: 0.1 * 0.1,
            max_num_points_in_cell: 10,
        }
    }
}

/// 扁平点容器（对照 `FlatContainer<HasNormals, HasCovs>`）。
///
/// `HAS_NORMALS`/`HAS_COVS` 在编译期决定是否写入法向/协方差；运行时字段始终存在，
/// 仅当对应开关为真时填充。
#[derive(Clone, Debug, Default)]
pub struct FlatContainer<const HAS_NORMALS: bool, const HAS_COVS: bool> {
    points: Vec<Vector4<f64>>,
    normals: Vec<Vector4<f64>>,
    covs: Vec<Matrix4<f64>>,
}

impl<const HAS_NORMALS: bool, const HAS_COVS: bool> FlatContainer<HAS_NORMALS, HAS_COVS> {
    /// 构造并预留少量容量。
    pub fn new() -> Self {
        let mut f = FlatContainer {
            points: Vec::new(),
            normals: Vec::new(),
            covs: Vec::new(),
        };
        f.points.reserve(5);
        f
    }

    /// 点数。
    pub fn size(&self) -> usize {
        self.points.len()
    }

    /// 取第 `i` 个点。
    pub fn point(&self, i: usize) -> Vector4<f64> {
        self.points[i]
    }

    /// 取第 `i` 个法向（仅 `HAS_NORMALS` 时有效）。
    pub fn normal(&self, i: usize) -> Vector4<f64> {
        self.normals[i]
    }

    /// 取第 `i` 个协方差（仅 `HAS_COVS` 时有效）。
    pub fn cov(&self, i: usize) -> Matrix4<f64> {
        self.covs[i]
    }

    /// 加入变换后的点；若 cell 已满或与已有点过近则忽略。
    ///
    /// - `transformed_pt`：`T · points[i]`（已变换坐标）；
    /// - `points`：源点云（读取法向/协方差用于同步变换）；
    /// - `i`：源点云索引；
    /// - `t`：变换矩阵 `T`。
    pub fn add(
        &mut self,
        setting: &FlatContainerSetting,
        transformed_pt: &Vector4<f64>,
        points: &impl PointCloudTrait,
        i: usize,
        t: &Matrix4<f64>,
    ) {
        if self.points.len() >= setting.max_num_points_in_cell {
            return;
        }
        if self
            .points
            .iter()
            .any(|p| (p - transformed_pt).norm_squared() < setting.min_sq_dist_in_cell)
        {
            return;
        }

        self.points.push(*transformed_pt);
        if HAS_NORMALS {
            self.normals.push(*t * points.normal(i));
        }
        if HAS_COVS {
            self.covs.push(*t * points.cov(i) * t.transpose());
        }
    }

    /// 收尾（对照 `finalize`，本容器无需额外操作）。
    pub fn finalize(&mut self) {}

    /// 最近邻搜索。
    pub fn nearest_neighbor_search(
        &self,
        pt: &Vector4<f64>,
        k_index: &mut usize,
        k_sq_dist: &mut f64,
    ) -> usize {
        if self.points.is_empty() {
            return 0;
        }
        let mut idx = [0usize];
        let mut dist = [0.0f64];
        let found;
        {
            let mut result = KnnResult::new(&mut idx, &mut dist, 1);
            self.push_all(pt, &mut result);
            found = result.num_found();
        } // 释放对 idx/dist 的可变借用后再读取
        *k_index = idx[0];
        *k_sq_dist = dist[0];
        found
    }

    /// k 近邻搜索。
    pub fn knn_search(
        &self,
        pt: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        if self.points.is_empty() {
            return 0;
        }
        let mut result = KnnResult::new(k_indices, k_sq_dists, k);
        self.push_all(pt, &mut result);
        result.num_found()
    }

    fn push_all(&self, pt: &Vector4<f64>, result: &mut KnnResult) {
        for (i, p) in self.points.iter().enumerate() {
            let d = (p - pt).norm_squared();
            result.push(i, d);
        }
    }
}

/// 仅存点的扁平容器。
pub type FlatContainerPoints = FlatContainer<false, false>;
/// 带法向的扁平容器。
pub type FlatContainerNormal = FlatContainer<true, false>;
/// 带协方差的扁平容器。
pub type FlatContainerCov = FlatContainer<false, true>;
/// 带法向与协方差的扁平容器。
pub type FlatContainerNormalCov = FlatContainer<true, true>;

impl<const HAS_NORMALS: bool, const HAS_COVS: bool> NearestNeighborSearch
    for FlatContainer<HAS_NORMALS, HAS_COVS>
{
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
    use crate::points::point_cloud::PointCloud;
    use crate::points::traits::{PointCloudMut, PointCloudTrait};

    #[test]
    fn add_respects_capacity_and_min_dist() {
        let mut c = FlatContainerPoints::new();
        let setting = FlatContainerSetting {
            min_sq_dist_in_cell: 0.25,
            max_num_points_in_cell: 3,
        };
        let src = PointCloud::new();
        let t = Matrix4::identity();
        // 三个相距足够远的点，应全部加入
        c.add(&setting, &Vector4::new(0.0, 0.0, 0.0, 1.0), &src, 0, &t);
        c.add(&setting, &Vector4::new(1.0, 0.0, 0.0, 1.0), &src, 1, &t);
        c.add(&setting, &Vector4::new(2.0, 0.0, 0.0, 1.0), &src, 2, &t);
        assert_eq!(c.size(), 3);
        // 第四个：超过容量被忽略
        c.add(&setting, &Vector4::new(3.0, 0.0, 0.0, 1.0), &src, 3, &t);
        assert_eq!(c.size(), 3);
    }

    #[test]
    fn add_ignores_too_close() {
        let mut c = FlatContainerPoints::new();
        let setting = FlatContainerSetting {
            min_sq_dist_in_cell: 0.25,
            max_num_points_in_cell: 10,
        };
        let src = PointCloud::new();
        let t = Matrix4::identity();
        c.add(&setting, &Vector4::new(0.0, 0.0, 0.0, 1.0), &src, 0, &t);
        // 距已有点 0.1 < 0.5 的开方 → 忽略
        c.add(&setting, &Vector4::new(0.1, 0.0, 0.0, 1.0), &src, 1, &t);
        assert_eq!(c.size(), 1);
    }

    #[test]
    fn knn_correct() {
        let mut c = FlatContainerPoints::new();
        let setting = FlatContainerSetting::default();
        let src = PointCloud::new();
        let t = Matrix4::identity();
        for x in 0..5 {
            c.add(
                &setting,
                &Vector4::new(x as f64, 0.0, 0.0, 1.0),
                &src,
                x,
                &t,
            );
        }
        let q = Vector4::new(2.3, 0.0, 0.0, 1.0);
        let mut idx = [0usize; 3];
        let mut dist = [0.0f64; 3];
        let n = c.knn_search(&q, 3, &mut idx, &mut dist);
        assert_eq!(n, 3);
        // 最近为 x=2（距离 0.3^2），其次 x=3，x=1
        assert_eq!(idx[0], 2);
        assert!((dist[0] - 0.09).abs() < 1e-12);
        assert_eq!(idx[1], 3);
        assert_eq!(idx[2], 1);

        let mut n_idx = 0usize;
        let mut n_dist = 0.0f64;
        let found = c.nearest_neighbor_search(&q, &mut n_idx, &mut n_dist);
        assert_eq!(found, 1);
        assert_eq!(n_idx, 2);
    }

    #[test]
    fn normals_and_covs_transformed() {
        let mut cloud = PointCloud::new();
        cloud.resize(1);
        cloud.set_point(0, Vector4::new(1.0, 2.0, 3.0, 1.0));
        cloud.set_normal(0, Vector4::new(0.0, 0.0, 1.0, 0.0));
        let mut cov = Matrix4::zeros();
        cov[(0, 0)] = 2.0;
        cloud.set_cov(0, cov);

        // 平移 (10,0,0)
        let mut t = Matrix4::identity();
        t[(0, 3)] = 10.0;

        let mut c = FlatContainerNormalCov::new();
        let setting = FlatContainerSetting::default();
        c.add(&setting, &(t * cloud.point(0)), &cloud, 0, &t);
        assert_eq!(c.size(), 1);
        let tp = c.point(0);
        assert!((tp.x - 11.0).abs() < 1e-12);
        // 法向为方向量，平移不变
        assert!((c.normal(0).z - 1.0).abs() < 1e-12);
        // 协方差平移不变（仅左上 3×3）
        assert!((c.cov(0)[(0, 0)] - 2.0).abs() < 1e-12);
    }
}
