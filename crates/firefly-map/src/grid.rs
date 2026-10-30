//! 派生的局部占据栅格：滚动窗口、固定先验、增量障碍膨胀。
//!
//! 深度融合与窗口外观测由 OKVIS 维护；调用方将局部地图快照栅格化后写入占据状态。
//! 窗口坐标随窗口下界平移，跨平移簿记使用世界锚定的全局索引。
//! 未知和自由状态均采用乐观查询，窗口外也不阻挡；世界系虚拟墙例外。
//! 膨胀沿用 EGO-Planner-v2 的邻域计数语义，观测更新不在此处重复融合。

mod buffer;
mod builder;
mod inflation;
mod occupancy;
#[cfg(test)]
mod snapshot_tests;
mod window;

pub use builder::GridMapBuilder;

use buffer::Buffer;
use std::collections::HashSet;
use std::sync::Arc;

use nalgebra::Vector3;

/// 障碍体素在膨胀计数缓冲中的标志增量，对照 `grid_map.h:28` `GRID_MAP_OBS_FLAG 32767`。
pub const GRID_MAP_OBS_FLAG: u16 = 32767;

/// 体素占据状态；`Unknown` 在规划查询中按自由处理。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoxelState {
    Unknown,
    Free,
    Occupied,
}

/// 虚拟地面/天花板（官方 `enable_virtual_wall` + `virtual_ground`/`virtual_ceil`）。
///
/// 世界坐标 z 平面，命中区间视为不可飞（对照官方 `getOccupancy` 返回 -1 在下游
/// 布尔语境的实效）。单位：米。单侧启用时另一侧用 ±∞ 哨兵。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VirtualWall {
    /// 虚拟地板：`p.z <= ground` 不可飞。
    pub ground: f64,
    /// 虚拟天花板：`p.z >= ceil` 不可飞。
    pub ceil: f64,
}

impl VirtualWall {
    fn blocks(&self, z: f64) -> bool {
        z >= self.ceil || z <= self.ground
    }
}

#[derive(Debug, Clone)]
pub struct GridMap {
    /// 世界系固定原点 = 全局体素索引 `[0,0,0]` 的角点；不随窗口平移。
    world_origin: Vector3<f64>,
    resolution: f64,
    /// 环形缓冲尺寸（体素数）= 2 × `local_update_range`（官方 `ringbuffer_size3i_`）。
    dims: [usize; 3],
    /// 膨胀缓冲尺寸 = `dims` + 2×`inf_grid`（官方 `ringbuffer_inf_size3i_`）：比占据
    /// 窗口每侧多 `inf_grid` 格余量，窗口边缘障碍的膨胀计数有处可写。
    inf_dims: [usize; 3],
    /// 局部更新半径（体素数，官方 `local_update_range3i_`），窗口半宽。
    local_update_range3i: [i64; 3],
    /// 当前窗口下界（全局索引，含端点；官方 `ringbuffer_lowbound3i_`）。
    low: [i64; 3],
    /// 当前窗口上界（全局索引，含端点；官方 `ringbuffer_upbound3i_`）。
    up: [i64; 3],
    /// 膨胀缓冲窗口下界（官方 `ringbuffer_inf_lowbound3i_`，= low − `inf_grid`）。
    inf_low: [i64; 3],
    /// 膨胀缓冲窗口上界（官方 `ringbuffer_inf_upbound3i_`，= up + `inf_grid`）。
    inf_up: [i64; 3],
    /// 环形寻址原点（官方 `ringbuffer_origin3i_`），恒落在 `[low, up]` 内；
    /// 存储地址 = `(全局索引 − origin) % dims`。
    ring_origin: [i64; 3],
    /// 膨胀缓冲环形寻址原点（官方 `ringbuffer_inf_origin3i_`）。
    inf_ring_origin: [i64; 3],
    /// 上次窗口中心（官方 `center_last3i_`，平移方向判据）。
    center_last: [i64; 3],
    /// 局部快照派生的二值占据；不累计传感器概率。
    occupancy: Buffer<bool>,
    /// 世界锚定的场地先验；自由快照与动态障碍移除不能清除。
    prior: Arc<HashSet<[i64; 3]>>,
    /// 膨胀计数缓冲（对照官方 `occupancy_buffer_inflate_`）：障碍体素自身 `±GRID_MAP_OBS_FLAG`，周围 `inf_grid` 格内 `±1`；`>0` 即膨胀。
    inflate: Buffer<u16>,
    /// 虚拟地面/天花板；None = 不启用（官方 `enable_virtual_wall = false` 默认）。
    virtual_wall: Option<VirtualWall>,
    /// 构造时按 `obstacles_inflation` 计算的膨胀步长（官方 `inf_grid_`，恒 ≤ 4），用于 `change_inf_buf` 增量更新。
    inf_grid: i32,
    /// 占据/膨胀状态版本号。
    revision: u64,
    /// 上次完成膨胀时的版本号（与 `revision` 相等即已同步）。
    inflated_revision: u64,
    /// 占据状态变化的缓冲地址，每次同步前去重，容量不超过占据窗口。
    inflation_dirty: Vec<usize>,
    inflation_pending: Buffer<bool>,
    /// 清空观测或直接编辑膨胀计数后，按占据层重建。
    inflation_rebuild: bool,
}

impl GridMap {
    /// Changes whenever occupancy, inflation inputs, or the rolling window changes.
    #[must_use]
    pub const fn revision(&self) -> u64 {
        self.revision
    }
}
