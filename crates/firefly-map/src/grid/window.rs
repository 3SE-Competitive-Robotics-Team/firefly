//! 世界/窗口坐标、环形寻址与滚动窗口与固定先验恢复。

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

    /// 当前窗口下界角点（世界坐标）＝窗口坐标 `[0,0,0]` 体素的角点；随
    /// `move_ring_buffer` 平移，构造后等于 builder 注入的 origin。
    #[must_use]
    pub fn origin(&self) -> Vector3<f64> {
        Vector3::new(
            self.world_origin.x + self.low[0] as f64 * self.resolution,
            self.world_origin.y + self.low[1] as f64 * self.resolution,
            self.world_origin.z + self.low[2] as f64 * self.resolution,
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
            self.local_update_range3i[0] as f64 * self.resolution,
            self.local_update_range3i[1] as f64 * self.resolution,
            self.local_update_range3i[2] as f64 * self.resolution,
        )
    }

    // --- 全局索引 ↔ 窗口坐标换算 ---

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
        self.in_window_global(g).then_some(g)
    }

    /// 全局索引是否在当前占据窗口内（对照官方 `isInBuf`）。
    pub(super) fn in_window_global(&self, g: [i64; 3]) -> bool {
        g.iter()
            .zip(&self.low)
            .zip(&self.up)
            .all(|((&gi, &lo), &up)| gi >= lo && gi <= up)
    }

    /// 全局索引是否在膨胀缓冲窗口内（对照官方 `isInInfBuf`）。
    pub(super) fn in_inf_window(&self, g: [i64; 3]) -> bool {
        g.iter()
            .zip(&self.inf_low)
            .zip(&self.inf_up)
            .all(|((&gi, &lo), &up)| gi >= lo && gi <= up)
    }

    /// 窗口坐标 → 全局体素索引（世界锚定）。
    #[must_use]
    pub fn global_index(&self, coord: [usize; 3]) -> [i64; 3] {
        [
            coord[0] as i64 + self.low[0],
            coord[1] as i64 + self.low[1],
            coord[2] as i64 + self.low[2],
        ]
    }

    /// 全局体素索引 → 窗口坐标；不在当前窗口时返回 `None`。
    #[must_use]
    pub fn window_index(&self, global: [i64; 3]) -> Option<[usize; 3]> {
        if !self.in_window_global(global) {
            return None;
        }
        Some([
            (global[0] - self.low[0]) as usize,
            (global[1] - self.low[1]) as usize,
            (global[2] - self.low[2]) as usize,
        ])
    }

    // --- 环形寻址（对照官方 globalIdx2BufIdx / globalIdx2InfBufIdx，grid_map.h:318）---

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
            g[i] = c[i] as i64 + self.ring_origin[i];
            if g[i] > self.up[i] {
                g[i] -= self.dims[i] as i64;
            }
        }
        g
    }

    // --- 对外体素读写（窗口坐标）---

    #[must_use]
    pub fn index_of(&self, p: Vector3<f64>) -> Option<[usize; 3]> {
        self.window_index(self.pos_to_global_index(p))
    }

    /// ringbuffer 平移（对照官方 `moveRingBuffer`，grid_map.cpp:399-457）：
    /// 新中心 = 相机位置的全局体素索引，窗口随之重定心；逐轴按中心相对
    /// 上次的位置判方向，清除移出窗口的体素（负向移动清上侧、正向清下侧，
    /// 对照 `clearBuffer` `grid_map.cpp:747`）；环形寻址原点 while 循环对齐到
    /// 新窗口内（grid_map.cpp:431-449）。相机未动时无副作用。
    ///
    /// 官方调用时机：感知更新入口先平移再投影（updateOccupancyCallback
    /// `grid_map.cpp:150` 与 `cloudCallback` `grid_map.cpp:358`）。
    pub fn move_ring_buffer(&mut self, center: Vector3<f64>) {
        let cg = self.pos_to_global_index(center);
        if cg != self.center_last {
            // 在旧寻址下同步障碍源，确保清除条带能够完整撤销它们的邻域贡献。
            self.inflate_obstacles();
            self.touch();
        }
        let lr = self.local_update_range3i;
        let mut new_low = [0i64; 3];
        let mut new_up = [0i64; 3];
        let mut new_inf_low = [0i64; 3];
        let mut new_inf_up = [0i64; 3];
        let ig = i64::from(self.inf_grid);
        for i in 0..3 {
            new_low[i] = cg[i] - lr[i];
            // 官方 upbound3i 含端点减一（grid_map.cpp:406）
            new_up[i] = cg[i] + lr[i] - 1;
            new_inf_low[i] = new_low[i] - ig;
            new_inf_up[i] = new_up[i] + ig;
        }

        // 清除移出窗口的体素：判据与清除均基于旧边界（官方在更新 md_ 边界前调用）
        if cg[0] < self.center_last[0] {
            self.clear_buffer(0, new_up[0]);
        }
        if cg[0] > self.center_last[0] {
            self.clear_buffer(1, new_low[0]);
        }
        if cg[1] < self.center_last[1] {
            self.clear_buffer(2, new_up[1]);
        }
        if cg[1] > self.center_last[1] {
            self.clear_buffer(3, new_low[1]);
        }
        if cg[2] < self.center_last[2] {
            self.clear_buffer(4, new_up[2]);
        }
        if cg[2] > self.center_last[2] {
            self.clear_buffer(5, new_low[2]);
        }

        // 环形寻址原点对齐新窗口（while 循环保持模尺寸不变，grid_map.cpp:431-449）
        for i in 0..3 {
            while self.ring_origin[i] < new_low[i] {
                self.ring_origin[i] += self.dims[i] as i64;
            }
            while self.ring_origin[i] > new_up[i] {
                self.ring_origin[i] -= self.dims[i] as i64;
            }
            while self.inf_ring_origin[i] < new_inf_low[i] {
                self.inf_ring_origin[i] += self.inf_dims[i] as i64;
            }
            while self.inf_ring_origin[i] > new_inf_up[i] {
                self.inf_ring_origin[i] -= self.inf_dims[i] as i64;
            }
        }

        self.center_last = cg;
        self.low = new_low;
        self.up = new_up;
        self.inf_low = new_inf_low;
        self.inf_up = new_inf_up;
        if !self.prior.is_empty() {
            let restores: Vec<[usize; 3]> = self
                .prior
                .iter()
                .filter_map(|g| self.window_index(*g))
                .collect();
            for idx in restores {
                self.set_state(idx, VoxelState::Occupied);
            }
        }
        self.inflate_obstacles();
    }

    /// 清除移出窗口的体素条带（对照官方 `clearBuffer(casein, bound)`，
    /// `grid_map.cpp:747`）：case 偶数 = 向负方向移动（清 `[bound, 旧上界]`）、
    /// 奇数 = 向正方向移动（清 `[旧下界, bound]`），正交两轴取旧窗口全域；
    /// 移动方向的边界列多清一格，与官方一致。占据状态清空，
    /// 障碍源从膨胀层反向移除（`>= FLAG`：firefly 的邻域更新跳过窗外体素，
    /// 窗缘障碍可能恰持 FLAG，须一并清除）。
    fn clear_buffer(&mut self, case: usize, bound: i64) {
        let x0 = if case == 0 { bound } else { self.low[0] };
        let x1 = if case == 1 { bound } else { self.up[0] };
        let y0 = if case == 2 { bound } else { self.low[1] };
        let y1 = if case == 3 { bound } else { self.up[1] };
        let z0 = if case == 4 { bound } else { self.low[2] };
        let z1 = if case == 5 { bound } else { self.up[2] };
        for x in x0.max(self.low[0])..=x1.min(self.up[0]) {
            for y in y0.max(self.low[1])..=y1.min(self.up[1]) {
                for z in z0.max(self.low[2])..=z1.min(self.up[2]) {
                    let g = [x, y, z];
                    let a = self.occ_addr(g);
                    let value = self.occupancy[a];
                    if value {
                        self.touch();
                    }
                    self.occupancy[a] = false;
                    let ia = self.inf_addr(g);
                    if self.inflate[ia] >= GRID_MAP_OBS_FLAG {
                        self.change_inf_buf_global(false, g, true);
                    }
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

    /// 相机未动：`move_ring_buffer` 无副作用，内容原样保留。
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

    /// 平移后：旧区域被清空、新区域可查询、窗口坐标随 origin 刚性平移。
    #[test]
    fn move_translates_window_and_clears_left_behind() {
        let mut m = GridMapBuilder::new(1.0, [8, 8, 8]).build().unwrap();
        // 障碍在全局 x=1（世界 x∈[1,2)）
        m.set_state([1, 4, 4], VoxelState::Occupied);
        assert!(m.is_occupied(Vector3::new(1.5, 4.5, 4.5)));
        // 相机右移两格（中心 4 → 6）：窗口 [0,7] → [2,9]
        m.move_ring_buffer(Vector3::new(6.5, 4.5, 4.5));
        // origin 随窗口平移（全局 x=2 处为新窗口角点）
        assert_eq!(m.origin(), Vector3::new(2.0, 0.0, 0.0));
        // 旧障碍已移出窗口 → 查询为自由（且 index_of 返回 None）
        assert_eq!(m.index_of(Vector3::new(1.5, 4.5, 4.5)), None);
        assert!(!m.is_occupied(Vector3::new(1.5, 4.5, 4.5)));
        // 新区域可查询可写入
        let idx = m
            .index_of(Vector3::new(8.5, 4.5, 4.5))
            .expect("新区域在窗口内");
        m.set_state(idx, VoxelState::Occupied);
        assert_eq!(m.state(idx), VoxelState::Occupied);
        // 幸存区域数据完好：窗口坐标 1 = 全局 x=3（新窗口 [2,9] 与旧窗口 [0,7] 重叠区）
        m.set_state([1, 4, 4], VoxelState::Occupied);
        assert!(m.is_occupied(Vector3::new(3.5, 4.5, 4.5)));
        assert_eq!(m.global_index([1, 4, 4]), [3, 4, 4]);
    }

    /// 环形寻址跨边界：多次小步平移后环形原点调整、存储地址回绕，
    /// 幸存数据经取模映射正确读写；出窗数据被清除后槽位可复用。
    #[test]
    fn ring_address_wraps_across_boundary() {
        let mut m = GridMapBuilder::new(1.0, [4, 4, 4]).build().unwrap();
        // 障碍在全局 x=3（初始窗口 [0,3] 最右列）
        m.set_state([3, 2, 2], VoxelState::Occupied);
        // +1 平移两次：窗口 [0,3] → [1,4] → [2,5]，x=3 始终在窗口内，
        // 环形原点从 0 跳到 4 但存储地址不变、数据不搬移
        for cx in [3.5, 4.5] {
            m.move_ring_buffer(Vector3::new(cx, 2.5, 2.5));
        }
        assert_eq!(m.origin(), Vector3::new(2.0, 0.0, 0.0));
        assert!(m.is_occupied(Vector3::new(3.5, 2.5, 2.5)));
        assert_eq!(m.state([1, 2, 2]), VoxelState::Occupied); // 窗口 [2,5] 中 x=3 → 坐标 1
        // 第三次 +1：窗口 [3,6]，x=3 落入清除条带（官方 clearBuffer(1, new_low)
        // 含边界列，grid_map.cpp:747-757），占据清除、槽位回收
        m.move_ring_buffer(Vector3::new(5.5, 2.5, 2.5));
        assert!(!m.is_occupied(Vector3::new(3.5, 2.5, 2.5)));
        // 回收槽位写入新障碍：全局 x=6（窗口坐标 3）经环形映射可查询。
        m.set_state([3, 2, 2], VoxelState::Occupied);
        assert!(m.is_occupied(Vector3::new(6.5, 2.5, 2.5)));
        assert!(!m.is_occupied(Vector3::new(3.5, 2.5, 2.5)));
    }

    /// 膨胀层随平移：幸存障碍计数完好、窗口边缘障碍移出时障碍源被反向移除、
    /// 回收槽位不留残留计数。
    #[test]
    fn inflate_layer_moves_with_ring_buffer() {
        let mut m = GridMapBuilder::new(1.0, [8, 4, 4])
            .with_obstacles_inflation(0.6)
            .build()
            .unwrap(); // step=1
        // 障碍 A 全局 x=2（右移后幸存）、B 全局 x=0（右移后出窗）
        m.set_state([2, 2, 2], VoxelState::Occupied);
        m.set_state([0, 2, 2], VoxelState::Occupied);
        m.inflate_obstacles();
        assert_eq!(m.inflate_at([2, 2, 2]), GRID_MAP_OBS_FLAG + 1);
        // 相机右移一格（中心 4 → 5，窗口 [0,7] → [1,8]）：A 幸存、B 出窗清除
        m.move_ring_buffer(Vector3::new(5.5, 2.5, 2.5));
        // A 在新窗口坐标 1，本体与邻域计数完好
        assert!(m.is_occupied(Vector3::new(2.5, 2.5, 2.5)));
        assert_eq!(m.inflate_at([1, 2, 2]), GRID_MAP_OBS_FLAG + 1);
        assert!(m.is_occupied_inflated(Vector3::new(3.5, 2.5, 2.5)));
        // B 出窗：占据清除，其障碍源从膨胀层反向移除（槽位无 FLAG 残留）
        assert!(!m.is_occupied(Vector3::new(0.5, 2.5, 2.5)));
        assert_eq!(m.window_index([0, 2, 2]), None);
        assert!(!m.is_occupied_inflated(Vector3::new(0.5, 2.5, 2.5)));
        // B 的邻域计数随障碍源移除：全局 x=1（窗口坐标 0）只剩 A 的 +1
        assert_eq!(m.inflate_at([0, 2, 2]), 1);
    }

    /// 各轴独立平移：仅移动轴清除条带，正交轴内容不受影响。
    #[test]
    fn per_axis_translation_is_independent() {
        let mut m = GridMapBuilder::new(1.0, [6, 6, 6]).build().unwrap();
        m.set_state([2, 2, 2], VoxelState::Occupied);
        m.set_state([2, 3, 2], VoxelState::Occupied);
        // 仅 y 向下移一格（中心 y 3 → 2，y 窗口 [0,5] → [-1,4]）：清除条带
        // y∈[4,5]（官方 clearBuffer(2, new_up) 含边界列），y=3 幸存
        m.move_ring_buffer(Vector3::new(3.5, 2.5, 3.5));
        assert_eq!(m.origin(), Vector3::new(0.0, -1.0, 0.0));
        // x/z 未动：全局 (x=2,z=2) 数据保留（y=3 在新窗口 [−1,4] 内、清条带外）
        assert!(m.is_occupied(Vector3::new(2.5, 2.5, 2.5)));
        assert!(m.is_occupied(Vector3::new(2.5, 3.5, 2.5)));
        // 全局 y=5 处写入再上移清除
        m.set_state([2, 0, 2], VoxelState::Occupied); // 全局 y=... 窗口坐标 0 → 全局 y=-1
        assert!(m.is_occupied(Vector3::new(2.5, -0.5, 2.5)));
        m.move_ring_buffer(Vector3::new(3.5, 3.5, 3.5)); // y 窗口回到 [0,5]
        assert!(!m.is_occupied(Vector3::new(2.5, -0.5, 2.5)));
        assert_eq!(m.window_index([-1, 0, 2]), None);
    }

    /// 全局/窗口双参照系换算往返一致。
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
