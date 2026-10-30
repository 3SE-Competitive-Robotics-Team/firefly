//! 障碍邻域计数、占据变化同步与膨胀查询。

use super::window::wrap_index;
use super::{GRID_MAP_OBS_FLAG, GridMap};
use nalgebra::Vector3;

/// 环形缓冲轴坐标加一个 ±1 偏移（单侧回绕）。
#[inline]
fn wrap_offset(coord: usize, delta: i32, size: usize) -> usize {
    let mut v = coord as i32 + delta;
    if v < 0 {
        v += size as i32;
    } else if v >= size as i32 {
        v -= size as i32;
    }
    v as usize
}

impl GridMap {
    /// 占据/膨胀状态版本号加一。占据状态或虚拟墙发生变化时调用。
    pub(super) fn touch(&mut self) {
        self.revision = self.revision.wrapping_add(1);
    }

    pub(super) fn mark_inflation_dirty(&mut self, address: usize) {
        self.touch();
        if !self.inflation_pending[address] {
            self.inflation_pending[address] = true;
            self.inflation_dirty.push(address);
        }
    }

    /// 以 `center` 为中心的 3×3×3 膨胀占用位掩码（含虚拟墙；越界格视为不阻挡）。
    ///
    /// 位序：`((di+1)*3+(dj+1))*3+(dk+1)`，`di/dj/dk ∈ {-1,0,1}`。
    #[inline]
    #[must_use]
    pub fn inflated_neighborhood_mask(&self, center: [usize; 3]) -> u32 {
        let dims = self.dims;
        let g = self.global_index(center);
        let c = [
            wrap_index(g[0], self.inf_ring_origin[0], self.inf_dims[0] as i64),
            wrap_index(g[1], self.inf_ring_origin[1], self.inf_dims[1] as i64),
            wrap_index(g[2], self.inf_ring_origin[2], self.inf_dims[2] as i64),
        ];
        // 虚拟墙只依赖 z：先算三层的阻挡状态，避免 27 次浮点换算。
        let mut wall_blocked = [false; 3];
        if let Some(wall) = self.virtual_wall {
            for (slot, dk) in (-1i32..=1).enumerate() {
                let cz = self.world_origin.z
                    + (g[2] + i64::from(dk)) as f64 * self.resolution
                    + 0.5 * self.resolution;
                wall_blocked[slot] = wall.blocks(cz);
            }
        }
        let sx = [
            wrap_offset(c[0], -1, self.inf_dims[0]),
            c[0],
            wrap_offset(c[0], 1, self.inf_dims[0]),
        ];
        let sy = [
            wrap_offset(c[1], -1, self.inf_dims[1]),
            c[1],
            wrap_offset(c[1], 1, self.inf_dims[1]),
        ];
        let sz = [
            wrap_offset(c[2], -1, self.inf_dims[2]),
            c[2],
            wrap_offset(c[2], 1, self.inf_dims[2]),
        ];
        let stride_x = self.inf_dims[1] * self.inf_dims[2];
        let stride_y = self.inf_dims[2];
        let mut out = 0u32;
        for di in -1i32..=1 {
            let nx = center[0] as i32 + di;
            if nx < 0 || nx >= dims[0] as i32 {
                continue;
            }
            let plane = sx[(di + 1) as usize] * stride_x;
            for dj in -1i32..=1 {
                let ny = center[1] as i32 + dj;
                if ny < 0 || ny >= dims[1] as i32 {
                    continue;
                }
                let row = plane + sy[(dj + 1) as usize] * stride_y;
                for dk in -1i32..=1 {
                    let nz = center[2] as i32 + dk;
                    if nz < 0 || nz >= dims[2] as i32 {
                        continue;
                    }
                    let index = ((di + 1) * 3 + (dj + 1)) as usize * 3 + (dk + 1) as usize;
                    if wall_blocked[(dk + 1) as usize]
                        || self.inflate[row + sz[(dk + 1) as usize]] > 0
                    {
                        out |= 1 << index;
                    }
                }
            }
        }
        out
    }

    /// 直接读取膨胀计数（测试用，对照官方 `occupancy_buffer_inflate_`）。
    #[must_use]
    pub fn inflate_at(&self, idx: [usize; 3]) -> u16 {
        self.inflate[self.inf_addr(self.global_index(idx))]
    }

    /// 索引是否落在膨胀层内（对照官方 `getInflateOccupancy` 下游 `>0` 判膨胀）。
    #[must_use]
    pub fn is_inflated(&self, idx: [usize; 3]) -> bool {
        let g = self.global_index(idx);
        if let Some(wall) = self.virtual_wall {
            let cz = self.world_origin.z + (g[2] as f64 + 0.5) * self.resolution;
            if wall.blocks(cz) {
                return true;
            }
        }
        self.inflate[self.inf_addr(g)] > 0
    }

    /// 位置是否落在膨胀层内（官方 `getInflateOccupancy(pos)`：虚拟墙命中视为
    /// 占据，膨胀窗口外视为非膨胀）。
    #[must_use]
    pub fn is_occupied_inflated(&self, p: Vector3<f64>) -> bool {
        if self.virtual_wall.is_some_and(|w| w.blocks(p.z)) {
            return true;
        }
        let g = self.pos_to_global_index(p);
        if !self.in_inf_window(g) {
            return false;
        }
        self.inflate[self.inf_addr(g)] > 0
    }

    /// 膨胀计数缓冲增量更新（对照 `grid_map.h:259-300` `changeInfBuf`）。
    /// `dir=true` 加障碍：自身 `+GRID_MAP_OBS_FLAG`，周围 `inf_grid` 格内每个 `+1`；
    /// `dir=false` 移除障碍：反向减；膨胀窗口外的邻域跳过。
    pub fn change_inf_buf(&mut self, dir: bool, idx: [usize; 3]) {
        self.inflation_rebuild = true;
        self.change_inf_buf_global(dir, self.global_index(idx), true);
    }

    /// 全局索引版膨胀增量更新（`clearBuffer` 共用；`inflate_obstacles`
    /// 重建时调用不计版本，重建结束统一同步）。
    pub(super) fn change_inf_buf_global(&mut self, dir: bool, g: [i64; 3], track: bool) {
        if track {
            self.touch();
        }
        let step = self.inf_grid;
        let center = self.inf_addr(g);
        if dir {
            self.inflate[center] = self.inflate[center].saturating_add(GRID_MAP_OBS_FLAG);
        } else {
            self.inflate[center] = self.inflate[center].saturating_sub(GRID_MAP_OBS_FLAG);
        }
        for di in -step..=step {
            for dj in -step..=step {
                for dk in -step..=step {
                    let gn = [
                        g[0] + i64::from(di),
                        g[1] + i64::from(dj),
                        g[2] + i64::from(dk),
                    ];
                    if !self.in_inf_window(gn) {
                        continue;
                    }
                    let l = self.inf_addr(gn);
                    if dir {
                        self.inflate[l] = self.inflate[l].saturating_add(1);
                    } else {
                        self.inflate[l] = self.inflate[l].saturating_sub(1);
                    }
                }
            }
        }
    }

    /// 同步膨胀层（官方 `clearAndInflateLocalMap` 的膨胀步骤，计数语义；步长为构造期确定的 `inf_grid`）。
    /// 每个占据体素 `+GRID_MAP_OBS_FLAG`（自身）并周围 `inf_grid` 格内 `+1`，多障碍叠加计数。
    /// 只更新占据状态变化体素的邻域；清空观测或直接编辑计数后全量重建。
    /// 同步前同一体素的多次变化合并为最终状态，未变化的障碍不重复膨胀。
    pub fn inflate_obstacles(&mut self) {
        if self.inflated_revision == self.revision {
            return;
        }
        if self.inflation_rebuild {
            self.inflate.fill(0);
            for address in 0..self.occupancy.len() {
                if self.occupancy[address] {
                    self.change_inf_buf_global(true, self.addr_global(address), false);
                }
            }
            self.inflation_rebuild = false;
        } else {
            for i in 0..self.inflation_dirty.len() {
                let address = self.inflation_dirty[i];
                let g = self.addr_global(address);
                let occupied = self.occupancy[address];
                let inflated = self.inflate[self.inf_addr(g)] >= GRID_MAP_OBS_FLAG;
                if occupied != inflated {
                    self.change_inf_buf_global(occupied, g, false);
                }
            }
        }
        for address in self.inflation_dirty.drain(..) {
            self.inflation_pending[address] = false;
        }
        self.inflated_revision = self.revision;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{GridMapBuilder, VoxelState};

    /// 逐目标格独立统计 Chebyshev 邻域内的障碍，覆盖膨胀窗口外沿。
    fn assert_inflation_counts(map: &GridMap) {
        let mut obstacles = Vec::new();
        for x in 0..map.dims[0] {
            for y in 0..map.dims[1] {
                for z in 0..map.dims[2] {
                    if map.state([x, y, z]) == VoxelState::Occupied {
                        obstacles.push(map.global_index([x, y, z]));
                    }
                }
            }
        }
        for x in map.inf_low[0]..=map.inf_up[0] {
            for y in map.inf_low[1]..=map.inf_up[1] {
                for z in map.inf_low[2]..=map.inf_up[2] {
                    let g = [x, y, z];
                    let mut expected = 0;
                    for obstacle in &obstacles {
                        if (0..3)
                            .all(|axis| (g[axis] - obstacle[axis]).abs() <= i64::from(map.inf_grid))
                        {
                            expected += 1;
                        }
                        if g == *obstacle {
                            expected += GRID_MAP_OBS_FLAG;
                        }
                    }
                    assert_eq!(map.inflate[map.inf_addr(g)], expected, "voxel {g:?}");
                }
            }
        }
    }

    #[test]
    fn sparse_inflation_coalesces_threshold_crossings() {
        let mut map = GridMapBuilder::new(1.0, [8, 8, 8]).build().unwrap();
        map.set_state([3, 3, 3], VoxelState::Occupied);
        map.set_state([4, 3, 3], VoxelState::Occupied);
        map.inflate_obstacles();
        for _ in 0..100 {
            map.set_state([3, 3, 3], VoxelState::Free);
            map.set_state([3, 3, 3], VoxelState::Occupied);
        }
        assert_eq!(map.inflation_dirty.len(), 1);
        map.set_state([3, 3, 3], VoxelState::Free);
        map.set_state([0, 0, 0], VoxelState::Occupied);
        map.inflate_obstacles();
        assert_eq!(map.inflation_dirty, [] as [usize; 0]);
        assert!(map.inflation_pending.iter().all(|pending| !pending));
        assert_inflation_counts(&map);
        let counts = map.inflate.clone();
        map.inflate_obstacles();
        assert_eq!(map.inflate, counts);
    }

    #[test]
    fn sparse_inflation_matches_geometry_across_window_changes() {
        for radius in [0.0, 1.0, 4.0] {
            let mut map = GridMapBuilder::new(1.0, [8, 8, 8])
                .with_obstacles_inflation(radius)
                .build()
                .unwrap();
            map.set_prior_occupied([2, 2, 2]);
            for round in 0..12 {
                for i in 0..40 {
                    let index = [(i * 3 + round) % 8, (i * 5 + round) % 8, i % 8];
                    map.set_state(index, VoxelState::Occupied);
                    if i % 3 == 0 {
                        map.set_state(index, VoxelState::Free);
                    }
                }
                // 待同步变化、单轴/多轴平移、跨整窗跳转和原地同步。
                let centers = [
                    [5.0, 4.0, 4.0],
                    [5.0, 3.0, 4.0],
                    [5.0, 3.0, 5.0],
                    [4.0, 4.0, 4.0],
                    [4.0, 4.0, 4.0],
                    [-10.0, 4.0, 4.0],
                    [-9.0, 3.0, 5.0],
                    [4.0, 4.0, 4.0],
                    [5.0, 5.0, 5.0],
                    [6.0, 6.0, 6.0],
                    [4.0, 4.0, 4.0],
                    [4.0, 4.0, 4.0],
                ];
                map.move_ring_buffer(Vector3::from(centers[round]));
                map.inflate_obstacles();
                assert_inflation_counts(&map);
                if round % 4 == 0 {
                    map.clear_observations();
                    assert_inflation_counts(&map);
                }
            }
        }
    }

    #[test]
    fn inflate_skips_recompute_when_map_unchanged() {
        let mut map = GridMapBuilder::new(0.5, [20, 20, 8]).build().unwrap();
        map.set_state([5, 5, 2], VoxelState::Occupied);
        map.inflate_obstacles();
        let mut snap = Vec::new();
        for z in 0..8 {
            for y in 0..20 {
                for x in 0..20 {
                    snap.push(map.inflate_at([x, y, z]));
                }
            }
        }
        // 无变更再调：结果逐位一致（是否跳过由版本保证，此处验证幂等）。
        map.inflate_obstacles();
        let mut snap2 = Vec::new();
        for z in 0..8 {
            for y in 0..20 {
                for x in 0..20 {
                    snap2.push(map.inflate_at([x, y, z]));
                }
            }
        }
        assert_eq!(snap, snap2);
        // 同值写入不触发版本变化（占据状态未改变）。
        map.set_state([5, 5, 2], VoxelState::Occupied);
        map.inflate_obstacles();
        let mut snap3 = Vec::new();
        for z in 0..8 {
            for y in 0..20 {
                for x in 0..20 {
                    snap3.push(map.inflate_at([x, y, z]));
                }
            }
        }
        assert_eq!(snap, snap3);
        // 真实变化重算：清除后邻域计数归零。
        map.set_state([5, 5, 2], VoxelState::Free);
        map.inflate_obstacles();
        assert_eq!(map.inflate_at([5, 5, 2]), 0);
    }

    #[test]
    fn zero_inflation_preserves_occupied_cells() {
        let mut map = GridMapBuilder::new(1.0, [10, 10, 10])
            .with_obstacles_inflation(0.0)
            .build()
            .unwrap();
        map.set_state([5, 5, 5], VoxelState::Occupied);
        map.inflate_obstacles();
        assert!(map.is_occupied_inflated(Vector3::new(5.5, 5.5, 5.5)));
        assert!(!map.is_occupied_inflated(Vector3::new(4.5, 5.5, 5.5)));
        map.set_state([5, 5, 5], VoxelState::Free);
        map.inflate_obstacles();
        assert!(!map.is_occupied_inflated(Vector3::new(5.5, 5.5, 5.5)));
    }

    #[test]
    fn inflation_expands_obstacles() {
        let mut m = GridMapBuilder::new(0.5, [10, 10, 10])
            .with_obstacles_inflation(0.6)
            .build()
            .unwrap();
        m.set_state([5, 5, 5], VoxelState::Occupied);
        m.inflate_obstacles(); // step = ceil((0.6-1e-5)/0.5)=2
        // 本体与 2 格邻域均在膨胀层（计数>0）
        assert!(m.is_occupied_inflated(Vector3::new(2.5, 2.5, 2.5)));
        assert!(m.is_occupied_inflated(Vector3::new(3.5, 2.5, 2.5)));
        // 3 格外不在膨胀层
        assert!(!m.is_occupied_inflated(Vector3::new(4.5, 2.5, 2.5)));
        // 原图状态不受膨胀影响
        assert!(!m.is_occupied(Vector3::new(3.5, 2.5, 2.5)));
        // 计数语义：中心 = 32767+1，邻域 =1
        assert_eq!(m.inflate_at([5, 5, 5]), GRID_MAP_OBS_FLAG + 1);
        assert_eq!(m.inflate_at([6, 5, 5]), 1);
    }

    #[test]
    fn change_inf_buf_counts() {
        let mut m = GridMapBuilder::new(0.5, [10, 10, 10]).build().unwrap();
        // 单障碍增量：自身 32768，周围 1（默认 step=1 对应 0.2/0.5）
        m.change_inf_buf(true, [5, 5, 5]);
        assert_eq!(m.inflate_at([5, 5, 5]), GRID_MAP_OBS_FLAG + 1);
        assert_eq!(m.inflate_at([6, 5, 5]), 1);
        assert_eq!(m.inflate_at([7, 5, 5]), 0);
        assert!(m.is_inflated([5, 5, 5]));
        assert!(m.is_inflated([6, 5, 5]));
        assert!(!m.is_inflated([7, 5, 5]));
        // 多障碍叠加：邻域计数叠加
        m.change_inf_buf(true, [5, 6, 5]);
        assert_eq!(m.inflate_at([5, 5, 5]), GRID_MAP_OBS_FLAG + 2);
        assert_eq!(m.inflate_at([5, 6, 5]), GRID_MAP_OBS_FLAG + 2);
        assert_eq!(m.inflate_at([6, 5, 5]), 2);
        // 移除一个：移除中心 FLAG，邻域 -1；[5,5,5] 仅剩来自 [5,6,5] 的邻域计数 1
        m.change_inf_buf(false, [5, 5, 5]);
        assert_eq!(m.inflate_at([5, 5, 5]), 1);
        assert_eq!(m.inflate_at([5, 6, 5]), GRID_MAP_OBS_FLAG + 1);
        assert_eq!(m.inflate_at([6, 5, 5]), 1);
        // 再移除
        m.change_inf_buf(false, [5, 6, 5]);
        assert_eq!(m.inflate_at([5, 5, 5]), 0);
        assert_eq!(m.inflate_at([5, 6, 5]), 0);
        assert!(!m.is_inflated([5, 5, 5]));
    }

    // --- ringbuffer 平移 ---

    /// 3×3×3 膨胀位掩码必须与逐格 `is_inflated` 一致（含环形窗平移）。
    #[test]
    fn inflated_neighborhood_mask_matches_cell_queries() {
        let mut m = GridMapBuilder::new(0.5, [12, 10, 8])
            .with_obstacles_inflation(0.5)
            .with_virtual_wall(-0.1, 2.9)
            .build()
            .unwrap();
        for idx in [[2, 2, 2], [8, 6, 5], [9, 8, 6], [3, 7, 1]] {
            m.set_state(idx, VoxelState::Occupied);
        }
        m.inflate_obstacles();
        for center in [
            Vector3::new(0.5, 0.5, 0.5),
            Vector3::new(3.5, 2.5, 2.5),
            Vector3::new(6.0, 4.0, 3.0),
            Vector3::new(2.5, 3.5, 2.0),
        ] {
            m.move_ring_buffer(center);
            let dims = m.dims();
            for x in 0..dims[0] {
                for y in 0..dims[1] {
                    for z in 0..dims[2] {
                        let mask = m.inflated_neighborhood_mask([x, y, z]);
                        let mut expected = 0u32;
                        for di in -1i32..=1 {
                            let nx = x as i32 + di;
                            if nx < 0 || nx >= dims[0] as i32 {
                                continue;
                            }
                            for dj in -1i32..=1 {
                                let ny = y as i32 + dj;
                                if ny < 0 || ny >= dims[1] as i32 {
                                    continue;
                                }
                                for dk in -1i32..=1 {
                                    let nz = z as i32 + dk;
                                    if nz < 0 || nz >= dims[2] as i32 {
                                        continue;
                                    }
                                    if m.is_inflated([nx as usize, ny as usize, nz as usize]) {
                                        let index = ((di + 1) * 3 + (dj + 1)) as usize * 3
                                            + (dk + 1) as usize;
                                        expected |= 1 << index;
                                    }
                                }
                            }
                        }
                        assert_eq!(mask, expected, "center {x},{y},{z}");
                    }
                }
            }
        }
    }
}
