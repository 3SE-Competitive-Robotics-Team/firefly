//! 派生占据状态、固定先验、观测重置与虚拟墙查询。

use super::{GridMap, VirtualWall, VoxelState};
use nalgebra::Vector3;

impl GridMap {
    /// 写入派生占据状态；自由或未知不能覆盖固定先验。
    pub fn set_state(&mut self, idx: [usize; 3], state: VoxelState) {
        let g = self.global_index(idx);
        let a = self.occ_addr(g);
        let occupied = state == VoxelState::Occupied || self.prior.contains(&g);
        if self.occupancy[a] != occupied {
            self.occupancy[a] = occupied;
            self.mark_inflation_dirty(a);
        }
    }
    /// 二值占据查询；未知状态按自由处理。
    #[must_use]
    pub fn state(&self, idx: [usize; 3]) -> VoxelState {
        let occ = self.occupancy[self.occ_addr(self.global_index(idx))];
        if occ {
            VoxelState::Occupied
        } else {
            VoxelState::Free
        }
    }

    /// 清除当前派生占据，保留世界锚定先验与虚拟墙。
    pub fn clear_observations(&mut self) {
        self.occupancy.fill(false);
        self.inflate.fill(0);
        self.inflation_rebuild = true;
        let prior: Vec<_> = self
            .prior
            .iter()
            .filter_map(|g| self.window_index(*g))
            .collect();
        for idx in prior {
            self.set_state(idx, VoxelState::Occupied);
        }
        self.touch();
        self.inflate_obstacles();
    }

    /// 登记不可由在线观测清除的场地占据体素。
    pub fn set_prior_occupied(&mut self, idx: [usize; 3]) {
        let global = self.global_index(idx);
        std::sync::Arc::make_mut(&mut self.prior).insert(global);
        self.set_state(idx, VoxelState::Occupied);
    }

    /// 为已构建实例注入虚拟墙（覆盖已有配置，影响膨胀查询语义）。
    pub fn set_virtual_wall(&mut self, wall: VirtualWall) {
        self.virtual_wall = Some(wall);
        self.touch();
    }

    #[must_use]
    pub fn virtual_wall(&self) -> Option<VirtualWall> {
        self.virtual_wall
    }

    /// 官方 `getOccupancy(pos)`：虚拟墙命中视为占据；窗口外视为自由
    /// （官方 `!isInBuf → 0`）；否则读取派生占据状态。
    #[must_use]
    pub fn is_occupied(&self, p: Vector3<f64>) -> bool {
        if self.virtual_wall.is_some_and(|w| w.blocks(p.z)) {
            return true;
        }
        let Some(g) = self.window_global(p) else {
            return false;
        };
        self.occupancy[self.occ_addr(g)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{GridMapBuilder, VoxelState};

    #[test]
    fn correction_clears_sensor_obstacles_but_preserves_world_prior() {
        let mut map = GridMapBuilder::new(0.1, [20; 3]).build().unwrap();
        map.set_prior_occupied([2, 2, 2]);
        map.set_state([15, 15, 15], VoxelState::Occupied);
        map.inflate_obstacles();
        map.clear_observations();
        assert_eq!(map.state([2, 2, 2]), VoxelState::Occupied);
        assert_eq!(map.state([15, 15, 15]), VoxelState::Free);
        assert_eq!(map.inflate_at([15, 15, 15]), 0);
        assert!(map.inflate_at([2, 2, 2]) > 0);
    }

    #[test]
    fn index_and_state_mapping() {
        let g = GridMapBuilder::new(0.5, [4, 4, 4]).build().unwrap();
        // 索引映射
        let p = Vector3::new(0.25, 0.75, 1.25);
        assert_eq!(g.index_of(p), Some([0, 1, 2]));
        // 占据状态的读写往返。
        let mut g = GridMapBuilder::new(0.5, [4, 4, 4]).build().unwrap();
        let idx = g.index_of(p).unwrap();
        g.set_state(idx, VoxelState::Occupied);
        assert!(g.is_occupied(p));
        assert_eq!(g.state(idx), VoxelState::Occupied);
        // Free 映射到 Free
        g.set_state(idx, VoxelState::Free);
        assert_eq!(g.state(idx), VoxelState::Free);
        assert!(!g.is_occupied(p));
    }

    /// 窗口外视为自由（官方 getOccupancy `!isInBuf → 0`）；虚拟墙例外，
    /// 世界系物理约束在窗口外仍生效。
    #[test]
    fn out_of_window_is_free_unless_virtual_wall() {
        let m = GridMapBuilder::new(1.0, [4, 4, 4])
            .with_virtual_wall(f64::NEG_INFINITY, 2.0)
            .build()
            .unwrap();
        assert!(!m.is_occupied(Vector3::new(100.0, 0.0, 0.5)));
        assert!(!m.is_occupied_inflated(Vector3::new(-50.0, 0.0, 0.5)));
        // 窗外但 z ≥ ceil → 墙命中
        assert!(m.is_occupied(Vector3::new(100.0, 0.0, 2.5)));
    }

    /// 地图 z∈[0,5)，虚拟墙 ground=1.0 / ceil=3.0：三档验证三个查询接口。
    #[test]
    fn virtual_wall_blocks_floor_and_ceiling() {
        let mut m = GridMapBuilder::new(0.5, [10, 10, 10])
            .with_virtual_wall(1.0, 3.0)
            .with_obstacles_inflation(0.6)
            .build()
            .unwrap();
        // 障碍在 (4.25,4.25,2.25)，探针取远端角落避免落入膨胀层
        m.set_state([8, 8, 4], VoxelState::Occupied);
        m.inflate_obstacles();
        let below = Vector3::new(1.0, 1.0, 0.25); // 带内自由区但 z ≤ ground
        let above = Vector3::new(1.0, 1.0, 4.75); // 带内自由区但 z ≥ ceil
        let inside = Vector3::new(1.0, 1.0, 2.0); // 带内自由区
        // is_occupied
        assert!(m.is_occupied(below));
        assert!(m.is_occupied(above));
        assert!(!m.is_occupied(inside));
        // 墙不掩盖体素占据：带内真实障碍仍可查
        assert!(m.is_occupied(Vector3::new(4.25, 4.25, 2.25)));
        // is_occupied_inflated
        assert!(m.is_occupied_inflated(below));
        assert!(m.is_occupied_inflated(above));
        assert!(!m.is_occupied_inflated(inside));
        // is_inflated（索引查询，按体素中心 z 判墙）：z=0 层中心 0.25 ≤ ground
        assert!(m.is_inflated([5, 5, 0]));
        assert!(m.is_inflated([5, 5, 9])); // 中心 4.75 ≥ ceil
        assert!(!m.is_inflated([5, 5, 4])); // 中心 2.25 在带内且非膨胀层
    }

    #[test]
    fn no_virtual_wall_matches_plain_queries() {
        let mut plain = GridMapBuilder::new(0.5, [10, 10, 10])
            .with_obstacles_inflation(0.6)
            .build()
            .unwrap();
        assert_eq!(plain.virtual_wall(), None);
        // 障碍在 (2.5,2.5,0.25)，查询点取远端角落避免落入膨胀层
        plain.set_state([5, 5, 0], VoxelState::Occupied);
        plain.inflate_obstacles();
        // 未配置时全图任意高度均不受墙影响
        for z in [0.25, 1.0, 2.5, 4.75] {
            let p = Vector3::new(4.25, 4.25, z);
            assert!(!plain.is_occupied(p));
            assert!(!plain.is_occupied_inflated(p));
        }
        assert!(!plain.is_inflated([5, 5, 9]));
        // set_virtual_wall 注入后生效（单侧：只拦上方）
        plain.set_virtual_wall(VirtualWall {
            ground: f64::NEG_INFINITY,
            ceil: 1.0,
        });
        assert_eq!(plain.virtual_wall().map(|w| w.ceil), Some(1.0));
        assert!(plain.is_occupied(Vector3::new(4.25, 4.25, 2.5)));
        assert!(plain.is_occupied_inflated(Vector3::new(4.25, 4.25, 2.5)));
        assert!(!plain.is_occupied(Vector3::new(4.25, 4.25, 0.25)));
    }
}
