//! 增量体素地图（对照 `ann/incremental_voxelmap.hpp`）。
//!
//! 支持增量插入与 LRU 清理；每个体素含任意 `VoxelContents`（如 `GaussianVoxel`
//! 或 `FlatContainer`）。全局点索引高 32 位为体素 id，低 32 位为体素内点 id。

use std::collections::HashMap;

use nalgebra::{Matrix4, Vector4};

use crate::ann::NearestNeighborSearch;
use crate::ann::flat_container::{FlatContainer, FlatContainerSetting};
use crate::ann::gaussian_voxel::{GaussianVoxel, GaussianVoxelSetting};
use crate::ann::knn_result::KnnResult;
use crate::points::traits::PointCloudTrait;
use crate::util::fast_floor::fast_floor;

/// 体素元信息（对照 `VoxelInfo`）。
#[derive(Clone, Debug)]
pub struct VoxelInfo {
    /// 上次访问的 lru 计数。
    pub lru: usize,
    /// 整数体素坐标。
    pub coord: [i32; 3],
}

/// 增量体素地图（对照 `IncrementalVoxelMap<VoxelContents>`）。
pub struct IncrementalVoxelMap<C: VoxelContents> {
    inv_leaf_size: f64,
    /// LRU 视野：超过该步数未访问的体素被清理。
    pub lru_horizon: usize,
    /// LRU 清理周期。
    pub lru_clear_cycle: usize,
    lru_counter: usize,
    /// 搜索偏移（1 / 7 / 27）。
    pub search_offsets: Vec<[i32; 3]>,
    /// 体素内容参数。
    pub voxel_setting: C::Setting,
    /// 扁平体素列表。
    pub flat_voxels: Vec<(VoxelInfo, C)>,
    /// 体素坐标到扁平索引的映射。
    pub voxels: HashMap<[i32; 3], usize>,
    _phantom: std::marker::PhantomData<C>,
}

/// 体素内容抽象，用于 `IncrementalVoxelMap` 多态。
pub trait VoxelContents: Default {
    /// 对应的 Setting 类型。
    type Setting: Default + Clone;

    /// 加入一点。
    fn add<P: PointCloudTrait>(
        &mut self,
        setting: &Self::Setting,
        transformed_pt: &Vector4<f64>,
        points: &P,
        i: usize,
        t: &Matrix4<f64>,
    );

    /// 收尾归一化。
    fn finalize(&mut self);

    /// 体素内点数。
    fn voxel_size(&self) -> usize;

    /// 取点。
    fn voxel_point(&self, i: usize) -> Vector4<f64>;

    /// 取法向。
    fn voxel_normal(&self, i: usize) -> Vector4<f64>;

    /// 取协方差。
    fn voxel_cov(&self, i: usize) -> Matrix4<f64>;

    /// 近邻压入结果。
    fn voxel_knn_search(
        &self,
        pt: &Vector4<f64>,
        result: &mut KnnResult,
        index_transform: &dyn Fn(usize) -> usize,
    );
}

impl VoxelContents for GaussianVoxel {
    type Setting = GaussianVoxelSetting;

    fn add<P: PointCloudTrait>(
        &mut self,
        setting: &Self::Setting,
        transformed_pt: &Vector4<f64>,
        points: &P,
        i: usize,
        t: &Matrix4<f64>,
    ) {
        GaussianVoxel::add(self, setting, transformed_pt, points, i, t);
    }

    fn finalize(&mut self) {
        GaussianVoxel::finalize(self);
    }

    fn voxel_size(&self) -> usize {
        GaussianVoxel::size(self)
    }

    fn voxel_point(&self, i: usize) -> Vector4<f64> {
        GaussianVoxel::point(self, i)
    }

    fn voxel_normal(&self, _i: usize) -> Vector4<f64> {
        Vector4::zeros()
    }

    fn voxel_cov(&self, i: usize) -> Matrix4<f64> {
        GaussianVoxel::cov_at(self, i)
    }

    fn voxel_knn_search(
        &self,
        pt: &Vector4<f64>,
        result: &mut KnnResult,
        index_transform: &dyn Fn(usize) -> usize,
    ) {
        let d = (self.mean() - pt).norm_squared();
        result.push(index_transform(0), d);
    }
}

impl<const HAS_NORMALS: bool, const HAS_COVS: bool> VoxelContents
    for FlatContainer<HAS_NORMALS, HAS_COVS>
{
    type Setting = FlatContainerSetting;

    fn add<P: PointCloudTrait>(
        &mut self,
        setting: &Self::Setting,
        transformed_pt: &Vector4<f64>,
        points: &P,
        i: usize,
        t: &Matrix4<f64>,
    ) {
        FlatContainer::add(self, setting, transformed_pt, points, i, t);
    }

    fn finalize(&mut self) {
        FlatContainer::finalize(self);
    }

    fn voxel_size(&self) -> usize {
        FlatContainer::size(self)
    }

    fn voxel_point(&self, i: usize) -> Vector4<f64> {
        FlatContainer::point(self, i)
    }

    fn voxel_normal(&self, i: usize) -> Vector4<f64> {
        FlatContainer::normal(self, i)
    }

    fn voxel_cov(&self, i: usize) -> Matrix4<f64> {
        FlatContainer::cov(self, i)
    }

    fn voxel_knn_search(
        &self,
        pt: &Vector4<f64>,
        result: &mut KnnResult,
        index_transform: &dyn Fn(usize) -> usize,
    ) {
        for i in 0..self.size() {
            let d = (self.point(i) - pt).norm_squared();
            result.push(index_transform(i), d);
        }
    }
}

impl<C> Default for IncrementalVoxelMap<C>
where
    C: VoxelContents,
    C::Setting: Default,
{
    fn default() -> Self {
        Self::new(1.0)
    }
}

impl<C> IncrementalVoxelMap<C>
where
    C: VoxelContents,
    C::Setting: Default,
{
    /// 构造（对照 `explicit IncrementalVoxelMap(double leaf_size)`）。
    pub fn new(leaf_size: f64) -> Self {
        let mut m = Self {
            inv_leaf_size: 1.0 / leaf_size,
            lru_horizon: 100,
            lru_clear_cycle: 10,
            lru_counter: 0,
            search_offsets: Vec::new(),
            voxel_setting: C::Setting::default(),
            flat_voxels: Vec::new(),
            voxels: HashMap::new(),
            _phantom: std::marker::PhantomData,
        };
        m.set_search_offsets(1);
        m
    }

    /// 点数（体素数，对 `GaussianVoxel` 即点数）。
    pub fn size(&self) -> usize {
        self.flat_voxels.len()
    }

    /// 总点数（所有体素内点数之和，用于 `FlatContainer` 场景）。
    pub fn total_points(&self) -> usize {
        self.flat_voxels.iter().map(|(_, v)| v.voxel_size()).sum()
    }

    /// 插入点云（对照 `insert`）。
    pub fn insert<P: PointCloudTrait>(&mut self, points: &P, t: &Matrix4<f64>) {
        for i in 0..points.num_points() {
            let pt = t * points.point(i);
            let f = fast_floor(&pt);
            let coord = [f[0], f[1], f[2]];

            let idx = if let Some(&idx) = self.voxels.get(&coord) {
                idx
            } else {
                let info = VoxelInfo {
                    lru: self.lru_counter,
                    coord,
                };
                let idx = self.flat_voxels.len();
                self.flat_voxels.push((info, C::default()));
                self.voxels.insert(coord, idx);
                idx
            };
            self.flat_voxels[idx].0.lru = self.lru_counter;
            let transformed = pt;
            self.flat_voxels[idx]
                .1
                .add(&self.voxel_setting, &transformed, points, i, t);
        }

        self.lru_counter += 1;
        if self.lru_counter.is_multiple_of(self.lru_clear_cycle) {
            self.flat_voxels
                .retain(|(info, _)| info.lru + self.lru_horizon >= self.lru_counter);
            self.voxels.clear();
            for (i, (info, _)) in self.flat_voxels.iter().enumerate() {
                self.voxels.insert(info.coord, i);
            }
        }

        for (_, v) in &mut self.flat_voxels {
            v.finalize();
        }
    }

    /// 插入（单位变换便捷）。
    pub fn insert_identity<P: PointCloudTrait>(&mut self, points: &P) {
        self.insert(points, &Matrix4::identity());
    }

    /// 全局点索引：高位体素 id，低位点 id（对照 `calc_index`）。
    pub fn calc_index(&self, voxel_id: usize, point_id: usize) -> usize {
        (voxel_id << Self::POINT_ID_BITS) | point_id
    }

    /// 提取体素 id。
    pub fn voxel_id(&self, i: usize) -> usize {
        i >> Self::POINT_ID_BITS
    }

    /// 提取点 id。
    pub fn point_id(&self, i: usize) -> usize {
        i & ((1usize << Self::POINT_ID_BITS) - 1)
    }

    /// 设置搜索偏移模式（1/7/27，对照 `set_search_offsets`）。
    pub fn set_search_offsets(&mut self, num_offsets: i32) {
        self.search_offsets = match num_offsets {
            7 => vec![
                [0, 0, 0],
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
                [-1, 0, 0],
                [0, -1, 0],
                [0, 0, -1],
            ],
            27 => {
                let mut v = Vec::with_capacity(27);
                for x in -1..=1 {
                    for y in -1..=1 {
                        for z in -1..=1 {
                            v.push([x, y, z]);
                        }
                    }
                }
                v
            }
            _ => {
                if num_offsets != 1 {
                    eprintln!(
                        "warning: unsupported search_offsets={num_offsets} (supported values: 1, 7, 27)"
                    );
                }
                vec![[0, 0, 0]]
            }
        };
    }

    /// 取体素内点（全局索引）。
    pub fn point_at(&self, global_index: usize) -> Vector4<f64> {
        let vid = self.voxel_id(global_index);
        let pid = self.point_id(global_index);
        self.flat_voxels[vid].1.voxel_point(pid)
    }

    /// 取体素内法向。
    pub fn normal_at(&self, global_index: usize) -> Vector4<f64> {
        let vid = self.voxel_id(global_index);
        let pid = self.point_id(global_index);
        self.flat_voxels[vid].1.voxel_normal(pid)
    }

    /// 取体素内协方差。
    pub fn cov_at(&self, global_index: usize) -> Matrix4<f64> {
        let vid = self.voxel_id(global_index);
        let pid = self.point_id(global_index);
        self.flat_voxels[vid].1.voxel_cov(pid)
    }

    /// 最近邻搜索（对照 `nearest_neighbor_search`）。
    pub fn nearest_neighbor_search(
        &self,
        pt: &Vector4<f64>,
        k_index: &mut usize,
        k_sq_dist: &mut f64,
    ) -> usize {
        if self.flat_voxels.is_empty() {
            return 0;
        }
        let mut idx = [usize::MAX];
        let mut dist = [f64::MAX];
        let n = self.knn_search(pt, 1, &mut idx, &mut dist);
        if n > 0 {
            *k_index = idx[0];
            *k_sq_dist = dist[0];
        }
        n
    }

    /// k 近邻搜索（对照 `knn_search`）。
    pub fn knn_search(
        &self,
        pt: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        if self.flat_voxels.is_empty() || k == 0 {
            return 0;
        }
        let mut result = KnnResult::new(k_indices, k_sq_dists, k);
        let center_f = fast_floor(&(pt * self.inv_leaf_size));
        let center = [center_f[0], center_f[1], center_f[2]];

        for off in &self.search_offsets {
            let coord = [center[0] + off[0], center[1] + off[1], center[2] + off[2]];
            let Some(&voxel_index) = self.voxels.get(&coord) else {
                continue;
            };
            let voxel = &self.flat_voxels[voxel_index].1;
            let transform = |i: usize| self.calc_index(voxel_index, i);
            voxel.voxel_knn_search(pt, &mut result, &transform);
        }
        result.num_found()
    }

    const POINT_ID_BITS: usize = 32;

    /// 逆叶尺寸（测试用）。
    pub fn inv_leaf_size(&self) -> f64 {
        self.inv_leaf_size
    }
}

impl<C> PointCloudTrait for IncrementalVoxelMap<C>
where
    C: VoxelContents,
{
    fn num_points(&self) -> usize {
        self.total_points()
    }

    fn has_points(&self) -> bool {
        self.total_points() > 0
    }

    fn has_normals(&self) -> bool {
        // 保守：若底层体素可能含法向则返回 true，调用方按需检查
        true
    }

    fn has_covs(&self) -> bool {
        true
    }

    fn point(&self, i: usize) -> Vector4<f64> {
        self.point_at(i)
    }

    fn normal(&self, i: usize) -> Vector4<f64> {
        self.normal_at(i)
    }

    fn cov(&self, i: usize) -> Matrix4<f64> {
        self.cov_at(i)
    }
}

impl<C> NearestNeighborSearch for IncrementalVoxelMap<C>
where
    C: VoxelContents,
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

/// 高斯体素地图别名（对照 `using GaussianVoxelMap = IncrementalVoxelMap<GaussianVoxel>`）。
pub type GaussianVoxelMap = IncrementalVoxelMap<GaussianVoxel>;

/// 点索引列表（对照 `traits::point_indices`）。
pub fn point_indices<C: VoxelContents>(map: &IncrementalVoxelMap<C>) -> Vec<usize> {
    let mut indices = Vec::new();
    for (voxel_id, (_, voxel)) in map.flat_voxels.iter().enumerate() {
        for point_id in 0..voxel.voxel_size() {
            indices.push(map.calc_index(voxel_id, point_id));
        }
    }
    indices
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::points::point_cloud::PointCloud;
    use crate::points::traits::PointCloudMut;
    use nalgebra::Vector4;

    #[test]
    fn gaussian_voxelmap_insert_and_search() {
        let mut cloud = PointCloud::new();
        cloud.resize(4);
        for i in 0..4 {
            cloud.set_point(i, Vector4::new(i as f64, 0.0, 0.0, 1.0));
            let mut cov = Matrix4::zeros();
            cov[(0, 0)] = 1.0;
            cloud.set_cov(i, cov);
        }
        let mut map = GaussianVoxelMap::new(0.5);
        map.insert_identity(&cloud);
        // 4 个点落在不同体素（间距 1 > 0.5）
        assert_eq!(map.flat_voxels.len(), 4);
        let mut idx = 0usize;
        let mut dist = 0.0f64;
        let n = map.nearest_neighbor_search(&Vector4::new(0.1, 0.0, 0.0, 1.0), &mut idx, &mut dist);
        assert_eq!(n, 1);
        assert!(dist < 1.0);
    }

    #[test]
    fn lru_cleanup() {
        let mut cloud = PointCloud::new();
        cloud.resize(1);
        cloud.set_point(0, Vector4::new(0.0, 0.0, 0.0, 1.0));
        cloud.set_cov(0, Matrix4::identity());

        let mut map = GaussianVoxelMap::new(1.0);
        map.lru_horizon = 2;
        map.lru_clear_cycle = 2;
        map.insert_identity(&cloud);
        assert_eq!(map.flat_voxels.len(), 1);

        let mut far = PointCloud::new();
        far.resize(1);
        far.set_point(0, Vector4::new(100.0, 0.0, 0.0, 1.0));
        far.set_cov(0, Matrix4::identity());
        // 多次插入远处点，触发 LRU 清理
        for _ in 0..5 {
            map.insert_identity(&far);
        }
        // 原点体素应被清理
        assert!(
            map.flat_voxels
                .iter()
                .all(|(info, _)| info.coord != [0, 0, 0])
        );
    }

    #[test]
    fn search_offsets_7_and_27() {
        let mut map = GaussianVoxelMap::new(1.0);
        map.set_search_offsets(7);
        assert_eq!(map.search_offsets.len(), 7);
        map.set_search_offsets(27);
        assert_eq!(map.search_offsets.len(), 27);
        map.set_search_offsets(1);
        assert_eq!(map.search_offsets.len(), 1);
    }
}
