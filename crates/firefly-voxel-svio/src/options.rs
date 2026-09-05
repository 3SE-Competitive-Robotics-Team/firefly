//! 体素选择参数（对照 Voxel-SVIO `voxel_parameter/*` 配置项）。
//!
//! 纯数据结构：`serde::Deserialize + #[serde(default)]`，缺键回落默认值。

use serde::Deserialize;

/// 体素选择参数。
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct VoxelOptions {
    /// 总开关（默认关闭，保持现有行为；实验经配置开启）。
    pub enabled: bool,
    /// 体素边长（米，默认 0.1，对照 `voxel_size`）。
    pub voxel_size: f64,
    /// 每体素上限点数（默认 5，对照 `max_num_points_in_voxel`）。
    pub max_points_per_voxel: usize,
    /// 体素内最小点间距（米，默认 0.03，对照 `min_distance_points`）。
    pub min_point_distance: f64,
    /// 查询邻域半径（体素格，默认 1 即 27 邻域，对照 `nb_voxels_visited`）。
    pub neighbor_radius: i32,
    /// 每体素 feeding 全部点（默认 false 即每体素只取首点，对照 `use_all_points`）。
    pub use_all_points: bool,
}

impl Default for VoxelOptions {
    fn default() -> Self {
        Self {
            enabled: false,
            voxel_size: 0.1,
            max_points_per_voxel: 5,
            min_point_distance: 0.03,
            neighbor_radius: 1,
            use_all_points: false,
        }
    }
}
