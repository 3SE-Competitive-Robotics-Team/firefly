//! 地图参数配置（对照 `voxel_map.h:35` `VoxelMapConfig` 与
//! `voxel_map.cpp:36-53` `loadVoxelConfig` 的默认值）。

/// 根体素边长（m，论文 V-A 固定 0.5m）。
pub const ROOT_SIZE: f64 = 0.5;
/// 八叉最大层数（论文 V-B：细分至最大层 3）。
pub const MAX_LAYER: usize = 3;
/// 每层体素内平面拟合所需的最少点数（对照 `layer_init_num` `{5,5,5,5,5}`）。
pub const LAYER_INIT_NUM: [usize; 5] = [5, 5, 5, 5, 5];
/// 成熟平面上保留的最大点数（对照 `max_points_num` 50，视觉点候选）。
pub const MAX_POINTS_PER_PLANE: usize = 50;
/// 增量更新的点数阈值（对照 `update_size_threshold` 5）。
pub const UPDATE_SIZE_THRESHOLD: usize = 5;
/// 平面判据：最小特征值阈值（对照 `planner_threshold` 0.01）。
pub const PLANER_THRESHOLD: f64 = 0.01;
/// 地图滑窗半宽（根体素数，对照 `half_map_size` 100；测试用小值）。
pub const HALF_MAP_SIZE: i64 = 100;
/// 滑窗触界触发平移的距离（m，对照 `sliding_thresh` 8）。
pub const SLIDING_THRESH: f64 = 8.0;

/// 地图参数（全部带默认值，纯数据）。
#[derive(Debug, Clone, Copy)]
pub struct VoxelMapOptions {
    /// 根体素边长（m）。
    pub root_size: f64,
    /// 八叉最大层数。
    pub max_layer: usize,
    /// 平面拟合每层最少点数。
    pub layer_init_num: [usize; 5],
    /// 成熟平面保留点数上限（视觉点候选）。
    pub max_points_per_plane: usize,
    /// 增量更新阈值。
    pub update_size_threshold: usize,
    /// 平面判据特征值阈值。
    pub planer_threshold: f64,
    /// 地图滑窗半宽（根体素数）。
    pub half_map_size: i64,
    /// 滑窗触界阈值（m）。
    pub sliding_thresh: f64,
    /// 图像网格边长（像素，论文 V-C 30×30）。
    pub grid_size: usize,
    /// 补丁尺寸（像素，论文 V-C 11×11）。
    pub patch_size: usize,
    /// 补丁金字塔层数（论文 V-C 3 层）。
    pub patch_pyramid_level: usize,
    /// 补丁增补帧间隔（帧，论文 V-D >20）。
    pub patch_add_frame_gap: u32,
    /// 补丁增补像素偏移（像素，论文 V-D >40）。
    pub patch_add_pixel_dist: f64,
    /// 法向精化收敛阈值（法向角，rad，对照 `normal_update < 0.0001`）。
    pub normal_converge_thresh: f64,
    /// 观察数上限（超出后删除最低分补丁，对照 `obs_ >= 30`）。
    pub max_obs_per_point: usize,
    /// 参考补丁评分需要的观察数下限（对照 `obs_ > 5`）。
    pub min_obs_for_score: usize,
    /// 法向精化收敛所需观察数（对照 `obs_ > 10`）。
    pub min_obs_for_converge: usize,
    /// 视锥角（rad，可见性查询的 `FoV`）。
    pub fov: f64,
    /// 光线投射深度范围 [min, max]（m，对照论文 VII-A `d_min`/`d_max`）。
    pub ray_depth_min: f64,
    pub ray_depth_max: f64,
}

impl Default for VoxelMapOptions {
    fn default() -> Self {
        Self {
            root_size: ROOT_SIZE,
            max_layer: MAX_LAYER,
            layer_init_num: LAYER_INIT_NUM,
            max_points_per_plane: MAX_POINTS_PER_PLANE,
            update_size_threshold: UPDATE_SIZE_THRESHOLD,
            planer_threshold: PLANER_THRESHOLD,
            half_map_size: HALF_MAP_SIZE,
            sliding_thresh: SLIDING_THRESH,
            grid_size: 30,
            patch_size: 11,
            patch_pyramid_level: 3,
            patch_add_frame_gap: 20,
            patch_add_pixel_dist: 40.0,
            normal_converge_thresh: 0.0001,
            max_obs_per_point: 30,
            min_obs_for_score: 5,
            min_obs_for_converge: 10,
            fov: 70f64.to_radians(),
            ray_depth_min: 0.5,
            ray_depth_max: 20.0,
        }
    }
}

/// 平面拟合参数（从 [`VoxelMapOptions`] 派生，供 `plane::fit_plane` 使用）。
#[derive(Debug, Clone, Copy)]
pub struct PlaneOptions {
    /// 平面判据特征值阈值。
    pub planer_threshold: f64,
}

impl From<&VoxelMapOptions> for PlaneOptions {
    fn from(o: &VoxelMapOptions) -> Self {
        Self {
            planer_threshold: o.planer_threshold,
        }
    }
}

impl VoxelMapOptions {
    /// 网格行列数（按图像尺寸对齐）。
    #[must_use]
    pub fn grid_dims(&self, width: usize, height: usize) -> (usize, usize) {
        let cols = width.div_ceil(self.grid_size);
        let rows = height.div_ceil(self.grid_size);
        (cols, rows)
    }
}
