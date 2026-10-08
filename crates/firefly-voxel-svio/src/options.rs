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

#[cfg(test)]
mod tests {
    use super::*;

    /// 缺键回落默认值（最小化配置合法，对照本仓库 TOML 约定）。
    #[test]
    fn partial_toml_falls_back_to_defaults() {
        let o: VoxelOptions = toml::from_str("voxel_size = 0.2").expect("partial table");
        assert!((o.voxel_size - 0.2).abs() < 1e-12);
        let default = VoxelOptions::default();
        assert_eq!(o.enabled, default.enabled);
        assert_eq!(o.max_points_per_voxel, default.max_points_per_voxel);
        assert_eq!(o.neighbor_radius, default.neighbor_radius);
        assert_eq!(o.use_all_points, default.use_all_points);
    }

    /// 全部键可解析且逐字段生效（对照配置键名）。
    #[test]
    fn full_toml_roundtrips_all_keys() {
        let o: VoxelOptions = toml::from_str(
            "enabled = true\nvoxel_size = 0.25\nmax_points_per_voxel = 7\n\
             min_point_distance = 0.05\nneighbor_radius = 0\nuse_all_points = true",
        )
        .expect("full table");
        assert!(o.enabled);
        assert!((o.voxel_size - 0.25).abs() < 1e-12);
        assert_eq!(o.max_points_per_voxel, 7);
        assert!((o.min_point_distance - 0.05).abs() < 1e-12);
        assert_eq!(o.neighbor_radius, 0);
        assert!(o.use_all_points);
    }

    /// 未知键不得导致解析失败（允许配置文件携带其他段/将来键）。
    #[test]
    fn unknown_keys_are_ignored() {
        let o: VoxelOptions = toml::from_str("future_key = 1\nvoxel_size = 0.2").unwrap();
        assert!((o.voxel_size - 0.2).abs() < 1e-12);
    }

    /// 空表等价于默认值。
    #[test]
    fn empty_toml_is_default() {
        let o: VoxelOptions = toml::from_str("").unwrap();
        assert!(!o.enabled);
        assert!((o.voxel_size - VoxelOptions::default().voxel_size).abs() < 1e-12);
    }
}
