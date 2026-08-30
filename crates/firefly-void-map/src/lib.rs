//! DIVO 自适应体素地图（P2 实现）。
//!
//! 技术蓝本为 FAST-LIVO2 论文第 V 节（Local Mapping）与官方实现
//! `include/voxel_map.h` / `src/voxel_map.cpp`：
//! 哈希+八叉根体素、叶体素局部平面（中心/法向/协方差）、视觉地图点与
//! 参考补丁、法向量离线精化、按需光线投射、环形缓冲滑窗。

use firefly_void_types::state::State;

/// 体素地图占位结构（P2 完整实现）。
pub struct VoxelMap;

impl VoxelMap {
    /// 更新地图几何（P2 实现）。
    pub fn update(&mut self, _state: &State) {}
}
