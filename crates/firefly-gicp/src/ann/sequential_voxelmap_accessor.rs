//! 体素地图顺序访问器（对照 `ann/sequential_voxelmap_accessor.hpp`）。
//!
//! 把 `IncrementalVoxelMap` 展平为顺序点云，供配准时作 source 使用。

use nalgebra::{Matrix4, Vector4};

use crate::ann::incremental_voxelmap::{IncrementalVoxelMap, VoxelContents, point_indices};
use crate::points::traits::PointCloudTrait;

/// 顺序体素图访问器（对照 `SequentialVoxelMapAccessor<VoxelMap>`）。
pub struct SequentialVoxelMapAccessor<'a, C: VoxelContents> {
    voxelmap: &'a IncrementalVoxelMap<C>,
    indices: Vec<usize>,
}

impl<'a, C: VoxelContents> SequentialVoxelMapAccessor<'a, C> {
    /// 构造（对照 `SequentialVoxelMapAccessor(voxelmap)`）。
    pub fn new(voxelmap: &'a IncrementalVoxelMap<C>) -> Self {
        let indices = point_indices(voxelmap);
        Self { voxelmap, indices }
    }

    /// 点数。
    pub fn size(&self) -> usize {
        self.indices.len()
    }

    /// 访问底层体素图。
    pub fn voxelmap(&self) -> &IncrementalVoxelMap<C> {
        self.voxelmap
    }

    /// 全局索引列表。
    pub fn indices(&self) -> &[usize] {
        &self.indices
    }
}

/// 便捷构造（对照 `create_sequential_accessor`）。
pub fn create_sequential_accessor<C: VoxelContents>(
    voxelmap: &IncrementalVoxelMap<C>,
) -> SequentialVoxelMapAccessor<'_, C> {
    SequentialVoxelMapAccessor::new(voxelmap)
}

impl<'a, C: VoxelContents> PointCloudTrait for SequentialVoxelMapAccessor<'a, C> {
    fn num_points(&self) -> usize {
        self.indices.len()
    }

    fn has_points(&self) -> bool {
        !self.indices.is_empty()
    }

    fn has_normals(&self) -> bool {
        true
    }

    fn has_covs(&self) -> bool {
        true
    }

    fn point(&self, i: usize) -> Vector4<f64> {
        self.voxelmap.point(self.indices[i])
    }

    fn normal(&self, i: usize) -> Vector4<f64> {
        self.voxelmap.normal(self.indices[i])
    }

    fn cov(&self, i: usize) -> Matrix4<f64> {
        self.voxelmap.cov(self.indices[i])
    }
}

// `SequentialVoxelMapAccessor` 本身不直接做最近邻搜索，仅作点云。
// 若需把它当 target 使用，需配合底层 voxelmap 的搜索（VGICP 场景 target 即 voxelmap）。

impl<'a, C: VoxelContents> Clone for SequentialVoxelMapAccessor<'a, C> {
    fn clone(&self) -> Self {
        Self {
            voxelmap: self.voxelmap,
            indices: self.indices.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ann::incremental_voxelmap::GaussianVoxelMap;
    use crate::points::point_cloud::PointCloud;
    use crate::points::traits::PointCloudMut;
    use nalgebra::Vector4;

    #[test]
    fn accessor_sequential() {
        let mut cloud = PointCloud::new();
        cloud.resize(3);
        for i in 0..3 {
            cloud.set_point(i, Vector4::new(i as f64, 0.0, 0.0, 1.0));
            cloud.set_cov(i, Matrix4::identity());
        }
        let mut map = GaussianVoxelMap::new(0.5);
        map.insert_identity(&cloud);
        let acc = SequentialVoxelMapAccessor::new(&map);
        assert_eq!(acc.size(), 3);
        assert_eq!(acc.num_points(), 3);
        // 顺序访问点应与原点一致（体素均值即原点，因每点独占体素）
        for i in 0..3 {
            assert!((acc.point(i).x - i as f64).abs() < 1e-12);
        }
    }
}
