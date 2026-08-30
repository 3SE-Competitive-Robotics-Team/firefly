//! 近似最近邻搜索（对照 official `ann/`）。
//!
//! 含 KdTree（最近邻搜索）、KNN 结果容器、扁平体素容器。本模块定义
//! [`NearestNeighborSearch`] trait 作为各类搜索结构的统一接口（对照
//! `ann/traits.hpp` 的分派角色）。

pub mod flat_container;
pub mod gaussian_voxel;
pub mod incremental_voxelmap;
pub mod kdtree;
pub mod knn_result;
pub mod projective_search;
pub mod sequential_voxelmap_accessor;

pub use flat_container::{
    FlatContainer, FlatContainerCov, FlatContainerNormal, FlatContainerNormalCov,
    FlatContainerPoints, FlatContainerSetting,
};
pub use gaussian_voxel::{GaussianVoxel, GaussianVoxelSetting};
pub use incremental_voxelmap::{
    GaussianVoxelMap, IncrementalVoxelMap, VoxelContents, VoxelInfo, point_indices,
};
pub use kdtree::{
    AxisAlignedProjection, INVALID_NODE, KdTree, KdTreeBuilder, KdTreeNode, NormalProjection,
    Projection, ProjectionSetting, UnsafeKdTree,
};
pub use knn_result::{INVALID_INDEX, KnnResult, KnnSetting};
pub use projective_search::{
    BorderClamp, BorderRepeat, EquirectangularProjection, ProjectiveProjection, ProjectiveSearch,
    UnsafeProjectiveSearch,
};
pub use sequential_voxelmap_accessor::{SequentialVoxelMapAccessor, create_sequential_accessor};

use nalgebra::Vector4;

/// 最近邻搜索统一接口（对照 `ann/traits.hpp` 的 `knn_search` 分派）。
///
/// KdTree 与扁平容器均实现该 trait，使法向估计等算法与具体搜索结构解耦。
pub trait NearestNeighborSearch {
    /// k 近邻搜索：`k_indices`/`k_sq_dists` 长度须 ≥ `k`，返回找到的邻居数。
    fn knn_search(
        &self,
        query: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize;

    /// 最近邻搜索：命中返回 1 并写入结果，否则返回 0。
    fn nearest_neighbor_search(
        &self,
        query: &Vector4<f64>,
        k_index: &mut usize,
        k_sq_dist: &mut f64,
    ) -> usize {
        let mut idx = [INVALID_INDEX];
        let mut dist = [0.0f64];
        let n = self.knn_search(query, 1, &mut idx, &mut dist);
        *k_index = idx[0];
        *k_sq_dist = dist[0];
        n
    }
}
