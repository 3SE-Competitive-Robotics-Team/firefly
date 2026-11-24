use super::voxel_box::VoxelBox;
use super::{GRID_MAP_OBS_FLAG, GridMap, VoxelState};
use nalgebra::Vector3;

/// 环形缓冲轴索引：窗口内索引只需一次加减；极端越界回退取模。
#[inline]
pub(super) fn wrap_index(value: i64, origin: i64, size: i64) -> usize {
    let mut v = value - origin;
    if v < 0 {
        v += size;
    }
    if v >= size {
        v -= size;
    }
    if v < 0 || v >= size {
        v = v.rem_euclid(size);
    }
    usize::try_from(v).expect("wrapped index is non-negative")
}

impl GridMap {
    #[must_use]
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    pub(crate) fn world_origin(&self) -> Vector3<f64> {
        self.world_origin
    }

    /// 当前窗口下界角点（世界坐标）＝窗口坐标 `[0,0,0]` 体素的角点；随
    /// `move_ring_buffer` 平移，构造后等于 builder 注入的 origin。
    #[must_use]
    pub fn origin(&self) -> Vector3<f64> {
        Vector3::new(
            self.world_origin.x + self.bounds.low[0] as f64 * self.resolution,
            self.world_origin.y + self.bounds.low[1] as f64 * self.resolution,
            self.world_origin.z + self.bounds.low[2] as f64 * self.resolution,
        )
    }

    #[must_use]
    pub fn dims(&self) -> [usize; 3] {
        self.dims
    }

    #[must_use]
    pub fn size(&self) -> Vector3<f64> {
        Vector3::new(
            self.dims[0] as f64 * self.resolution,
            self.dims[1] as f64 * self.resolution,
            self.dims[2] as f64 * self.resolution,
        )
    }

    /// 局部更新范围（米，官方 `local_update_range3d_`；窗口半宽随分辨率取整后的值）。
    #[must_use]
    pub fn local_update_range(&self) -> Vector3<f64> {
        Vector3::new(
            (self.dims[0] / 2) as f64 * self.resolution,
            (self.dims[1] / 2) as f64 * self.resolution,
            (self.dims[2] / 2) as f64 * self.resolution,
        )
    }
    /// 世界位置 → 全局体素索引（世界锚定整数，不随窗口平移；纯映射，不含窗口判定）。
    #[must_use]
    pub fn pos_to_global_index(&self, p: Vector3<f64>) -> [i64; 3] {
        [
            ((p.x - self.world_origin.x) / self.resolution).floor() as i64,
            ((p.y - self.world_origin.y) / self.resolution).floor() as i64,
            ((p.z - self.world_origin.z) / self.resolution).floor() as i64,
        ]
    }

    /// 世界位置 → 全局体素索引；仅在落入当前窗口时返回。
    pub(super) fn window_global(&self, p: Vector3<f64>) -> Option<[i64; 3]> {
        let g = self.pos_to_global_index(p);
        self.bounds.contains(g).then_some(g)
    }

    pub(super) fn in_inf_window(&self, g: [i64; 3]) -> bool {
        self.inf_bounds.contains(g)
    }

    /// 窗口坐标 → 全局体素索引（世界锚定）。
    #[must_use]
    pub fn global_index(&self, coord: [usize; 3]) -> [i64; 3] {
        [
            coord[0] as i64 + self.bounds.low[0],
            coord[1] as i64 + self.bounds.low[1],
            coord[2] as i64 + self.bounds.low[2],
        ]
    }

    /// 全局体素索引 → 窗口坐标；不在当前窗口时返回 `None`。
    #[must_use]
    pub fn window_index(&self, global: [i64; 3]) -> Option<[usize; 3]> {
        if !self.bounds.contains(global) {
            return None;
        }
        Some([
            (global[0] - self.bounds.low[0]) as usize,
            (global[1] - self.bounds.low[1]) as usize,
            (global[2] - self.bounds.low[2]) as usize,
        ])
    }
    /// 全局索引 → 占据缓冲线性地址：`(id − ring_origin) % size` 取模映射后行主序展开。
    pub(super) fn occ_addr(&self, g: [i64; 3]) -> usize {
        let c = [
            wrap_index(g[0], self.ring_origin[0], self.dims[0] as i64),
            wrap_index(g[1], self.ring_origin[1], self.dims[1] as i64),
            wrap_index(g[2], self.ring_origin[2], self.dims[2] as i64),
        ];
        (c[0] * self.dims[1] + c[1]) * self.dims[2] + c[2]
    }

    /// 全局索引 → 膨胀缓冲线性地址（独立环形原点/尺寸，同上映射）。
    pub(super) fn inf_addr(&self, g: [i64; 3]) -> usize {
        let c = [
            wrap_index(g[0], self.inf_ring_origin[0], self.inf_dims[0] as i64),
            wrap_index(g[1], self.inf_ring_origin[1], self.inf_dims[1] as i64),
            wrap_index(g[2], self.inf_ring_origin[2], self.inf_dims[2] as i64),
        ];
        (c[0] * self.inf_dims[1] + c[1]) * self.inf_dims[2] + c[2]
    }

    /// 占据缓冲线性地址 → 全局索引（官方 `BufIdx2GlobalIdx` 的逆映射：
    /// 分量加环形原点，超出上界回绕一个 buffer 尺寸）。
    pub(super) fn addr_global(&self, a: usize) -> [i64; 3] {
        let mut c = [0usize; 3];
        c[2] = a % self.dims[2];
        c[1] = a / self.dims[2] % self.dims[1];
        c[0] = a / (self.dims[1] * self.dims[2]);
        let mut g = [0i64; 3];
        for i in 0..3 {
            let offset = c[i] as i64;
            let until_wrap = self.bounds.end[i] - self.ring_origin[i];
            g[i] = if offset < until_wrap {
                self.ring_origin[i] + offset
            } else {
                self.bounds.low[i] + (offset - until_wrap)
            };
        }
        g
    }
    #[must_use]
    pub fn index_of(&self, p: Vector3<f64>) -> Option<[usize; 3]> {
        self.window_index(self.pos_to_global_index(p))
    }

    /// 按整格移动窗口，保留交集内的占据与膨胀贡献。中心须有限且窗口边界可用 i64 表示。
    pub fn move_ring_buffer(&mut self, center: Vector3<f64>) {
        if !center.iter().all(|v| v.is_finite()) {
            return;
        }
        let center = self.pos_to_global_index(center);
        let Some(next) = VoxelBox::centered(center, self.dims) else {
            return;
        };
        let Some(next_inflated) = VoxelBox::centered(center, self.inf_dims) else {
            return;
        };

        // Pending edits still refer to addresses in the old window.
        self.inflate_obstacles();
        if self.bounds == next {
            return;
        }
        let retained = self.bounds.intersection(next);
        self.bounds
            .visit_difference(retained, |slab| self.clear_region(slab));

        self.bounds = next;
        self.inf_bounds = next_inflated;
        self.ring_origin = next.ring_origin();
        self.inf_ring_origin = next_inflated.ring_origin();
        self.touch();
        for &global in std::sync::Arc::clone(&self.prior).iter() {
            if let Some(index) = self.window_index(global)
                && !retained.is_some_and(|shared| shared.contains(global))
            {
                self.set_state(index, VoxelState::Occupied);
            }
        }
        self.inflate_obstacles();
    }

    fn clear_region(&mut self, region: VoxelBox) {
        for x in region.low[0]..region.end[0] {
            for y in region.low[1]..region.end[1] {
                let mut z = region.low[2];
                while z < region.end[2] {
                    let g = [x, y, z];
                    let a = self.occ_addr(g);
                    let ia = self.inf_addr(g);
                    // Skip known-empty page runs, stopping at either ring row's wrap boundary.
                    let skip = self
                        .occupancy
                        .uniform_len(a, false)
                        .min(self.inflate.uniform_len(ia, 0))
                        .min(self.dims[2] - a % self.dims[2])
                        .min(self.inf_dims[2] - ia % self.inf_dims[2])
                        .min((region.end[2] - z) as usize);
                    if skip > 0 {
                        z += skip as i64;
                        continue;
                    }
                    if self.occupancy[a] {
                        self.occupancy[a] = false;
                    }
                    if self.inflate[ia] >= GRID_MAP_OBS_FLAG {
                        self.change_inf_buf_global(false, g, false);
                    }
                    z += 1;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{GridMapBuilder, VoxelState};

    #[test]
    fn returning_window_restores_only_prior_until_snapshot_is_applied() {
        let mut map = GridMapBuilder::new(1.0, [8; 3]).build().unwrap();
        let prior = Vector3::new(2.5, 3.5, 4.5);
        let observed = Vector3::new(5.5, 3.5, 4.5);
        map.set_prior_occupied([2, 3, 4]);
        map.set_state([5, 3, 4], VoxelState::Occupied);
        for state in [VoxelState::Free, VoxelState::Unknown] {
            map.set_state([2, 3, 4], state);
            assert!(map.is_occupied(prior));
        }
        map.move_ring_buffer(Vector3::new(20.0, 4.0, 4.0));
        map.move_ring_buffer(Vector3::new(4.0, 4.0, 4.0));
        assert!(map.is_occupied_inflated(prior));
        assert!(!map.is_occupied_inflated(observed));
        map.set_state(map.index_of(observed).unwrap(), VoxelState::Occupied);
        map.inflate_obstacles();
        assert!(map.is_occupied_inflated(observed));
        map.clear_observations();
        assert!(map.is_occupied_inflated(prior));
        assert!(!map.is_occupied_inflated(observed));
    }

    #[test]
    fn stationary_center_is_noop() {
        let mut m = GridMapBuilder::new(1.0, [8, 8, 8]).build().unwrap();
        m.set_state([2, 3, 4], VoxelState::Occupied);
        let center = Vector3::new(4.0, 4.0, 4.0); // 初始窗口中心（dims/2 = 4）
        m.move_ring_buffer(center);
        assert_eq!(m.index_of(Vector3::new(2.5, 3.5, 4.5)), Some([2, 3, 4]));
        assert_eq!(m.state([2, 3, 4]), VoxelState::Occupied);
        assert_eq!(m.origin(), Vector3::zeros());
    }

    #[test]
    fn move_translates_window_and_clears_left_behind() {
        let mut m = GridMapBuilder::new(1.0, [8, 8, 8]).build().unwrap();
        m.set_state([1, 4, 4], VoxelState::Occupied);
        assert!(m.is_occupied(Vector3::new(1.5, 4.5, 4.5)));
        m.move_ring_buffer(Vector3::new(6.5, 4.5, 4.5));
        assert_eq!(m.origin(), Vector3::new(2.0, 0.0, 0.0));
        assert_eq!(m.index_of(Vector3::new(1.5, 4.5, 4.5)), None);
        assert!(!m.is_occupied(Vector3::new(1.5, 4.5, 4.5)));
        let idx = m
            .index_of(Vector3::new(8.5, 4.5, 4.5))
            .expect("新区域在窗口内");
        m.set_state(idx, VoxelState::Occupied);
        assert_eq!(m.state(idx), VoxelState::Occupied);
        m.set_state([1, 4, 4], VoxelState::Occupied);
        assert!(m.is_occupied(Vector3::new(3.5, 4.5, 4.5)));
        assert_eq!(m.global_index([1, 4, 4]), [3, 4, 4]);
    }

    #[test]
    fn ring_address_wraps_across_boundary() {
        let mut m = GridMapBuilder::new(1.0, [4, 4, 4]).build().unwrap();
        m.set_state([3, 2, 2], VoxelState::Occupied);
        for cx in [3.5, 4.5] {
            m.move_ring_buffer(Vector3::new(cx, 2.5, 2.5));
        }
        assert_eq!(m.origin(), Vector3::new(2.0, 0.0, 0.0));
        assert!(m.is_occupied(Vector3::new(3.5, 2.5, 2.5)));
        assert_eq!(m.state([1, 2, 2]), VoxelState::Occupied); // 窗口 [2,5] 中 x=3 → 坐标 1
        m.move_ring_buffer(Vector3::new(5.5, 2.5, 2.5));
        assert!(m.is_occupied(Vector3::new(3.5, 2.5, 2.5)));
        m.move_ring_buffer(Vector3::new(6.5, 2.5, 2.5));
        assert!(!m.is_occupied(Vector3::new(3.5, 2.5, 2.5)));
        m.set_state([2, 2, 2], VoxelState::Occupied);
        assert!(m.is_occupied(Vector3::new(6.5, 2.5, 2.5)));
        assert!(!m.is_occupied(Vector3::new(3.5, 2.5, 2.5)));
    }

    #[test]
    fn inflate_layer_moves_with_ring_buffer() {
        let mut m = GridMapBuilder::new(1.0, [8, 4, 4])
            .with_obstacles_inflation(0.6)
            .build()
            .unwrap(); // step=1
        m.set_state([2, 2, 2], VoxelState::Occupied);
        m.set_state([0, 2, 2], VoxelState::Occupied);
        m.inflate_obstacles();
        assert_eq!(m.inflate_at([2, 2, 2]), GRID_MAP_OBS_FLAG + 1);
        m.move_ring_buffer(Vector3::new(5.5, 2.5, 2.5));
        assert!(m.is_occupied(Vector3::new(2.5, 2.5, 2.5)));
        assert_eq!(m.inflate_at([1, 2, 2]), GRID_MAP_OBS_FLAG + 1);
        assert!(m.is_occupied_inflated(Vector3::new(3.5, 2.5, 2.5)));
        assert!(!m.is_occupied(Vector3::new(0.5, 2.5, 2.5)));
        assert_eq!(m.window_index([0, 2, 2]), None);
        assert!(!m.is_occupied_inflated(Vector3::new(0.5, 2.5, 2.5)));
        assert_eq!(m.inflate_at([0, 2, 2]), 1);
    }

    #[test]
    fn per_axis_translation_is_independent() {
        let mut m = GridMapBuilder::new(1.0, [6, 6, 6]).build().unwrap();
        m.set_state([2, 2, 2], VoxelState::Occupied);
        m.set_state([2, 3, 2], VoxelState::Occupied);
        m.move_ring_buffer(Vector3::new(3.5, 2.5, 3.5));
        assert_eq!(m.origin(), Vector3::new(0.0, -1.0, 0.0));
        assert!(m.is_occupied(Vector3::new(2.5, 2.5, 2.5)));
        assert!(m.is_occupied(Vector3::new(2.5, 3.5, 2.5)));
        m.set_state([2, 0, 2], VoxelState::Occupied); // 全局 y=... 窗口坐标 0 → 全局 y=-1
        assert!(m.is_occupied(Vector3::new(2.5, -0.5, 2.5)));
        m.move_ring_buffer(Vector3::new(3.5, 3.5, 3.5)); // y 窗口回到 [0,5]
        assert!(!m.is_occupied(Vector3::new(2.5, -0.5, 2.5)));
        assert_eq!(m.window_index([-1, 0, 2]), None);
    }

    #[test]
    fn global_window_index_roundtrip() {
        let mut m = GridMapBuilder::new(0.5, [10, 10, 10]).build().unwrap();
        m.move_ring_buffer(Vector3::new(3.0, 3.0, 3.0)); // 中心 5→6? res=0.5: floor(3/0.5)=6
        for g in [
            [6i64, 6, 6],
            [2, 9, 1],
            [10, 3, 10], // z=10 = 窗口上界（含端点）
        ] {
            let Some(c) = m.window_index(g) else {
                panic!("global {g:?} 应在窗口内");
            };
            assert_eq!(m.global_index(c), g);
        }
        assert_eq!(m.window_index([-100, 0, 0]), None);
        assert_eq!(
            m.pos_to_global_index(Vector3::new(1.49, -0.01, 0.0)),
            [2, -1, 0]
        );
    }
}
