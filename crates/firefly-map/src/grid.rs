//! 占据栅格地图（log-odds 概率表示，对照 EGO-Planner-v2 `plan_env/grid_map`）。
//!
//! - `VoxelState` 枚举保留以兼容历史测试代码，但语义已变化：
//!   初始占据概率为 `clamp_min_log_`（`logit(p_min)`，`p_min=0.12`），`state()` 查询
//!   时 `occupancy >= min_occupancy_log_` 判为 `Occupied`，否则为 `Free`；
//!   `Unknown` 保留在枚举中但不再作为初始状态或查询结果（仅 `set_state(Unknown)` 可显式写入）。
//! - 存储为 `occupancy: Vec<f64>`（log-odds），对照官方 `occupancy_buffer_`（`grid_map.cpp` 34-77 行参数初始化、`grid_map.h:27` logit 宏）。
//! - 膨胀层 `inflate` 为计数缓冲 `Vec<u16>`（对照官方 `occupancy_buffer_inflate_`），通过 `change_inf_buf` 增量更新，判定 `>0` 即膨胀；
//!   膨胀步长 `inf_grid` 构造时按 `obstacles_inflation` 计算，超 4 自动放大分辨率（官方 grid_map.cpp:44-48）。

use firefly_error::{Error, ErrorKind};
use nalgebra::Vector3;

/// 障碍体素在膨胀计数缓冲中的标志增量，对照 `grid_map.h:28` `GRID_MAP_OBS_FLAG 32767`。
pub const GRID_MAP_OBS_FLAG: u16 = 32767;

/// 体素占据状态（兼容层，见模块头注）。
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

/// `logit(p) = ln(p/(1-p))`，对照 `grid_map.h:27`。
#[inline]
fn logit(p: f64) -> f64 {
    (p / (1.0 - p)).ln()
}

#[derive(Debug, Clone)]
pub struct GridMap {
    origin: Vector3<f64>,
    resolution: f64,
    dims: [usize; 3],
    /// log-odds 占据概率（对照官方 `occupancy_buffer_`），初始为 `clamp_min_log_`。
    occupancy: Vec<f64>,
    /// 膨胀计数缓冲（对照官方 `occupancy_buffer_inflate_`）：障碍体素自身 `±GRID_MAP_OBS_FLAG`，周围 `inf_grid` 格内 `±1`；`>0` 即膨胀。
    inflate: Vec<u16>,
    /// 虚拟地面/天花板；None = 不启用（官方 `enable_virtual_wall = false` 默认）。
    virtual_wall: Option<VirtualWall>,
    // --- log-odds 参数（对照 grid_map.cpp 34-77 行，默认值与官方一致）---
    pub(crate) prob_hit_log: f64,
    pub(crate) prob_miss_log: f64,
    pub(crate) clamp_min_log: f64,
    pub(crate) clamp_max_log: f64,
    pub(crate) min_occupancy_log: f64,
    pub(crate) fading_time: f64,
    /// 构造时按 `obstacles_inflation` 计算的膨胀步长（官方 `inf_grid_`，恒 ≤ 4），用于 `change_inf_buf` 增量更新。
    inf_grid: i32,
}

#[derive(Debug)]
pub struct GridMapBuilder {
    origin: Vector3<f64>,
    resolution: f64,
    dims: [usize; 3],
    virtual_wall: Option<VirtualWall>,
    // log-odds 概率参数（可按部署调整，缺省为官方值）
    p_min: f64,
    p_max: f64,
    p_occ: f64,
    p_hit: f64,
    p_miss: f64,
    fading_time: f64,
    /// 障碍膨胀半径（米，官方 `grid_map/obstacles_inflation`）。
    obstacles_inflation: f64,
}

impl GridMapBuilder {
    #[must_use]
    pub fn new(resolution: f64, dims: [usize; 3]) -> Self {
        Self {
            origin: Vector3::zeros(),
            resolution,
            dims,
            virtual_wall: None,
            p_min: 0.12,
            p_max: 0.97,
            p_occ: 0.80,
            p_hit: 0.70,
            p_miss: 0.35,
            fading_time: 1000.0,
            obstacles_inflation: 0.2,
        }
    }

    #[must_use]
    pub fn with_origin(mut self, origin: Vector3<f64>) -> Self {
        self.origin = origin;
        self
    }

    /// 启用虚拟地面/天花板（单位：米，世界坐标 z；单侧不启用传 ±∞ 哨兵）。
    #[must_use]
    pub fn with_virtual_wall(mut self, ground: f64, ceil: f64) -> Self {
        self.virtual_wall = Some(VirtualWall { ground, ceil });
        self
    }

    /// 覆盖 `p_hit`（命中概率，默认 0.70）。
    #[must_use]
    pub fn with_p_hit(mut self, p: f64) -> Self {
        self.p_hit = p;
        self
    }

    /// 覆盖 `p_miss`（miss 概率，默认 0.35）。
    #[must_use]
    pub fn with_p_miss(mut self, p: f64) -> Self {
        self.p_miss = p;
        self
    }

    /// 覆盖障碍膨胀半径（米，默认 0.2）；超过 4 格时构建自动放大分辨率（官方 grid_map.cpp:44-48）。
    #[must_use]
    pub fn with_obstacles_inflation(mut self, radius: f64) -> Self {
        self.obstacles_inflation = radius;
        self
    }

    /// 覆盖 `p_min/p_max/p_occ/fading_time`（默认 0.12/0.97/0.80/1000.0）。
    #[must_use]
    pub fn with_odds_params(
        mut self,
        p_min: f64,
        p_max: f64,
        p_occ: f64,
        fading_time: f64,
    ) -> Self {
        self.p_min = p_min;
        self.p_max = p_max;
        self.p_occ = p_occ;
        self.fading_time = fading_time;
        self
    }

    /// # Errors
    ///
    /// `InvalidArgument`：resolution 必须为正。
    pub fn build(self) -> firefly_error::Result<GridMap> {
        if self.resolution <= 0.0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "resolution must be positive",
            ));
        }
        // 膨胀步长与分辨率（官方 grid_map.cpp:44-48）：inf_grid > 4 时放大分辨率
        let mut resolution = self.resolution;
        let mut inf_grid = ((self.obstacles_inflation - 1e-5) / resolution).ceil() as i32;
        if inf_grid > 4 {
            inf_grid = 4;
            resolution = self.obstacles_inflation / f64::from(inf_grid);
        }
        let capacity = self.dims[0] * self.dims[1] * self.dims[2];
        let prob_hit_log = logit(self.p_hit);
        let prob_miss_log = logit(self.p_miss);
        let clamp_min_log = logit(self.p_min);
        let clamp_max_log = logit(self.p_max);
        let min_occupancy_log = logit(self.p_occ);
        let occupancy = vec![clamp_min_log; capacity];
        Ok(GridMap {
            origin: self.origin,
            resolution,
            dims: self.dims,
            occupancy,
            inflate: vec![0; capacity],
            virtual_wall: self.virtual_wall,
            prob_hit_log,
            prob_miss_log,
            clamp_min_log,
            clamp_max_log,
            min_occupancy_log,
            fading_time: self.fading_time,
            inf_grid,
        })
    }
}

impl GridMap {
    #[must_use]
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    #[must_use]
    pub fn origin(&self) -> Vector3<f64> {
        self.origin
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

    #[must_use]
    pub fn index_of(&self, p: Vector3<f64>) -> Option<[usize; 3]> {
        let rel = p - self.origin;
        let mut idx = [0usize; 3];
        for (i, v) in idx.iter_mut().enumerate() {
            let c = (rel[i] / self.resolution).floor();
            if c < 0.0 || c >= self.dims[i] as f64 {
                return None;
            }
            *v = c as usize;
        }
        Some(idx)
    }

    /// 体素占据状态查询（`occupancy >= min_occupancy_log_` → Occupied，否则 Free）。
    /// `Unknown` 保留在枚举中但不再作为初始/查询结果（见模块头注）。
    #[must_use]
    pub fn state(&self, idx: [usize; 3]) -> VoxelState {
        let occ = self.occupancy[self.linear(idx)];
        if occ >= self.min_occupancy_log {
            VoxelState::Occupied
        } else {
            VoxelState::Free
        }
    }

    /// 兼容接口：`Occupied` → `clamp_max_log_`，`Free`/`Unknown` → `clamp_min_log_`。
    pub fn set_state(&mut self, idx: [usize; 3], state: VoxelState) {
        let linear = self.linear(idx);
        match state {
            VoxelState::Occupied => self.occupancy[linear] = self.clamp_max_log,
            VoxelState::Free | VoxelState::Unknown => self.occupancy[linear] = self.clamp_min_log,
        }
    }

    /// log-odds 增量更新（对照 `grid_map.cpp:658-669` 的 clamp 语义）。
    pub fn update_occupancy(&mut self, idx: [usize; 3], delta: f64) {
        let linear = self.linear(idx);
        let v = self.occupancy[linear] + delta;
        self.occupancy[linear] = v.clamp(self.clamp_min_log, self.clamp_max_log);
    }

    /// 直接读取体素的 log-odds 值（测试/诊断用）。
    #[must_use]
    pub fn occupancy_at(&self, idx: [usize; 3]) -> f64 {
        self.occupancy[self.linear(idx)]
    }

    /// 直接读取膨胀计数（测试用，对照官方 `occupancy_buffer_inflate_`）。
    #[must_use]
    pub fn inflate_at(&self, idx: [usize; 3]) -> u16 {
        self.inflate[self.linear(idx)]
    }

    #[must_use]
    pub fn prob_hit_log(&self) -> f64 {
        self.prob_hit_log
    }

    #[must_use]
    pub fn prob_miss_log(&self) -> f64 {
        self.prob_miss_log
    }

    #[must_use]
    pub fn clamp_min_log(&self) -> f64 {
        self.clamp_min_log
    }

    #[must_use]
    pub fn clamp_max_log(&self) -> f64 {
        self.clamp_max_log
    }

    #[must_use]
    pub fn min_occupancy_log(&self) -> f64 {
        self.min_occupancy_log
    }

    #[must_use]
    pub fn fading_time(&self) -> f64 {
        self.fading_time
    }

    /// 为已构建实例注入虚拟墙（覆盖已有配置）。
    pub fn set_virtual_wall(&mut self, wall: VirtualWall) {
        self.virtual_wall = Some(wall);
    }

    #[must_use]
    pub fn virtual_wall(&self) -> Option<VirtualWall> {
        self.virtual_wall
    }

    /// 官方 `getOccupancy(pos)`：越界或虚拟墙命中均视为占据；否则阈值判定。
    #[must_use]
    pub fn is_occupied(&self, p: Vector3<f64>) -> bool {
        if self.virtual_wall.is_some_and(|w| w.blocks(p.z)) {
            return true;
        }
        self.index_of(p).is_none_or(|idx| {
            let occ = self.occupancy[self.linear(idx)];
            occ >= self.min_occupancy_log
        })
    }

    /// 索引是否落在膨胀层内（对照官方 `getInflateOccupancy` 下游 `>0` 判膨胀）。
    #[must_use]
    pub fn is_inflated(&self, idx: [usize; 3]) -> bool {
        if let Some(wall) = self.virtual_wall {
            let cz = self.origin.z + (idx[2] as f64 + 0.5) * self.resolution;
            if wall.blocks(cz) {
                return true;
            }
        }
        self.inflate[self.linear(idx)] > 0
    }

    /// 位置是否落在膨胀层内（官方 `getInflateOccupancy(pos)`，越界或虚拟墙命中视为占据）。
    #[must_use]
    pub fn is_occupied_inflated(&self, p: Vector3<f64>) -> bool {
        if self.virtual_wall.is_some_and(|w| w.blocks(p.z)) {
            return true;
        }
        self.index_of(p).is_none_or(|idx| self.is_inflated(idx))
    }

    /// 膨胀计数缓冲增量更新（对照 `grid_map.h:259-300` `changeInfBuf`）。
    /// `dir=true` 加障碍：自身 `+GRID_MAP_OBS_FLAG`，周围 `inf_grid` 格内每个 `+1`；
    /// `dir=false` 移除障碍：反向减；越界跳过。
    pub fn change_inf_buf(&mut self, dir: bool, idx: [usize; 3]) {
        let step = self.inf_grid;
        let center = self.linear(idx);
        if dir {
            self.inflate[center] = self.inflate[center].saturating_add(GRID_MAP_OBS_FLAG);
        } else {
            self.inflate[center] = self.inflate[center].saturating_sub(GRID_MAP_OBS_FLAG);
        }
        let [dx, dy, dz] = self.dims;
        for di in -step..=step {
            for dj in -step..=step {
                for dk in -step..=step {
                    let nx = idx[0] as i32 + di;
                    let ny = idx[1] as i32 + dj;
                    let nz = idx[2] as i32 + dk;
                    if nx < 0 || ny < 0 || nz < 0 {
                        continue;
                    }
                    if nx >= dx as i32 || ny >= dy as i32 || nz >= dz as i32 {
                        continue;
                    }
                    let l = self.linear([nx as usize, ny as usize, nz as usize]);
                    if dir {
                        self.inflate[l] = self.inflate[l].saturating_add(1);
                    } else {
                        self.inflate[l] = self.inflate[l].saturating_sub(1);
                    }
                }
            }
        }
    }

    /// 重算膨胀层（官方 `clearAndInflateLocalMap` 的膨胀步骤，计数语义；步长为构造期确定的 `inf_grid`）。
    /// 每个占据体素 `+GRID_MAP_OBS_FLAG`（自身）并周围 `inf_grid` 格内 `+1`，多障碍叠加计数。
    pub fn inflate_obstacles(&mut self) {
        self.inflate.fill(0);
        if self.inf_grid <= 0 {
            return;
        }
        let step = self.inf_grid;
        let [dx, dy, dz] = self.dims;
        // 收集占据体素
        let mut occupied = Vec::new();
        for x in 0..dx {
            for y in 0..dy {
                for z in 0..dz {
                    let occ = self.occupancy[self.linear([x, y, z])];
                    if occ >= self.min_occupancy_log {
                        occupied.push([x, y, z]);
                    }
                }
            }
        }
        for idx in occupied {
            let center = self.linear(idx);
            self.inflate[center] = self.inflate[center].saturating_add(GRID_MAP_OBS_FLAG);
            for di in -step..=step {
                for dj in -step..=step {
                    for dk in -step..=step {
                        let nx = idx[0] as i32 + di;
                        let ny = idx[1] as i32 + dj;
                        let nz = idx[2] as i32 + dk;
                        if nx < 0 || ny < 0 || nz < 0 {
                            continue;
                        }
                        if nx >= dx as i32 || ny >= dy as i32 || nz >= dz as i32 {
                            continue;
                        }
                        let l = self.linear([nx as usize, ny as usize, nz as usize]);
                        self.inflate[l] = self.inflate[l].saturating_add(1);
                    }
                }
            }
        }
    }

    /// 地图衰减（对照官方 `grid_map.cpp:204-222` `fadingCallback`，必须 2Hz 调用）。
    ///
    /// - `reduce = (clamp_max_log_ - min_occupancy_log_) / (fading_time * 2)` 固定值，对应官方 2Hz 定时器（每 0.5s）。
    /// - `low_thres = clamp_min_log_ + reduce`，仅 `> low_thres` 的体素衰减，避免跌破下界。
    /// - 衰减前 `>= min_occupancy` 且衰减后 `<` 的体素（障碍→自由）调用 `change_inf_buf(false, idx)` 增量移除膨胀。
    pub fn fade(&mut self) {
        if self.fading_time <= 0.0 {
            return;
        }
        let reduce = (self.clamp_max_log - self.min_occupancy_log) / (self.fading_time * 2.0);
        if reduce <= 0.0 {
            return;
        }
        let low_thres = self.clamp_min_log + reduce;
        let [dx, dy, dz] = self.dims;
        // 先更新占据并收集跨阈值索引
        let mut crossed: Vec<[usize; 3]> = Vec::new();
        for x in 0..dx {
            for y in 0..dy {
                for z in 0..dz {
                    let l = self.linear([x, y, z]);
                    let occ = self.occupancy[l];
                    if occ > low_thres {
                        let was_occupied = occ >= self.min_occupancy_log;
                        let mut new_occ = occ - reduce;
                        if new_occ < self.clamp_min_log {
                            new_occ = self.clamp_min_log;
                        }
                        self.occupancy[l] = new_occ;
                        if was_occupied && new_occ < self.min_occupancy_log {
                            crossed.push([x, y, z]);
                        }
                    }
                }
            }
        }
        // 增量移除膨胀（计数缓冲）
        for idx in crossed {
            self.change_inf_buf(false, idx);
        }
    }

    fn linear(&self, idx: [usize; 3]) -> usize {
        (idx[0] * self.dims[1] + idx[1]) * self.dims[2] + idx[2]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builder_rejects_bad_resolution() {
        let r = GridMapBuilder::new(0.0, [10, 10, 10]).build();
        assert_eq!(r.unwrap_err().kind(), ErrorKind::InvalidArgument);
    }

    #[test]
    fn index_and_state_mapping() {
        let g = GridMapBuilder::new(0.5, [4, 4, 4]).build().unwrap();
        // 索引映射
        let p = Vector3::new(0.25, 0.75, 1.25);
        assert_eq!(g.index_of(p), Some([0, 1, 2]));
        // set/get 往返（log-odds：Occupied → clamp_max ≥阈值）
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

    #[test]
    fn out_of_bounds_is_occupied() {
        let m = GridMapBuilder::new(1.0, [4, 4, 4]).build().unwrap();
        assert!(m.is_occupied(Vector3::new(100.0, 0.0, 0.0)));
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

    #[test]
    fn log_odds_defaults_match_official() {
        let m = GridMapBuilder::new(0.5, [10, 10, 10]).build().unwrap();
        // 对照官方 p_hit=0.70 etc，logit 计算
        let hit = (0.70_f64 / 0.30).ln();
        let miss = (0.35_f64 / 0.65).ln();
        let min = (0.12_f64 / 0.88).ln();
        let max = (0.97_f64 / 0.03).ln();
        let occ = (0.80_f64 / 0.20).ln();
        assert!((m.prob_hit_log() - hit).abs() < 1e-12);
        assert!((m.prob_miss_log() - miss).abs() < 1e-12);
        assert!((m.clamp_min_log() - min).abs() < 1e-12);
        assert!((m.clamp_max_log() - max).abs() < 1e-12);
        assert!((m.min_occupancy_log() - occ).abs() < 1e-12);
        assert!((m.fading_time() - 1000.0).abs() < 1e-12);
        // 初始占据为 clamp_min，对应 Free
        assert_eq!(m.state([0, 0, 0]), VoxelState::Free);
        assert!((m.occupancy_at([0, 0, 0]) - min).abs() < 1e-12);
    }

    #[test]
    fn update_occupancy_clamps() {
        let mut m = GridMapBuilder::new(0.5, [4, 4, 4]).build().unwrap();
        let idx = [1, 1, 1];
        // 连续命中直到上界
        for _ in 0..10 {
            m.update_occupancy(idx, m.prob_hit_log());
        }
        assert!((m.occupancy_at(idx) - m.clamp_max_log()).abs() < 1e-12);
        // 再命中不越界
        m.update_occupancy(idx, 100.0);
        assert!((m.occupancy_at(idx) - m.clamp_max_log()).abs() < 1e-12);
        // 连续 miss 直到下界
        for _ in 0..20 {
            m.update_occupancy(idx, m.prob_miss_log());
        }
        assert!((m.occupancy_at(idx) - m.clamp_min_log()).abs() < 1e-12);
        // 再 miss 不越界
        m.update_occupancy(idx, -100.0);
        assert!((m.occupancy_at(idx) - m.clamp_min_log()).abs() < 1e-12);
        // set_state 兼容
        m.set_state(idx, VoxelState::Occupied);
        assert!((m.occupancy_at(idx) - m.clamp_max_log()).abs() < 1e-12);
        assert_eq!(m.state(idx), VoxelState::Occupied);
        m.set_state(idx, VoxelState::Free);
        assert!((m.occupancy_at(idx) - m.clamp_min_log()).abs() < 1e-12);
        assert_eq!(m.state(idx), VoxelState::Free);
        m.set_state(idx, VoxelState::Unknown);
        assert!((m.occupancy_at(idx) - m.clamp_min_log()).abs() < 1e-12);
        assert_eq!(m.state(idx), VoxelState::Free);
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

    #[test]
    fn builder_enlarges_resolution_when_inflation_exceeds_4() {
        // ceil((0.3-1e-5)/0.05)=6 > 4 → 分辨率放大为 0.3/4（官方 grid_map.cpp:44-48）
        let mut m = GridMapBuilder::new(0.05, [40, 40, 40])
            .with_obstacles_inflation(0.3)
            .build()
            .unwrap();
        assert!((m.resolution() - 0.3 / 4.0).abs() < 1e-12);
        // step=4：本体与 4 格邻域在膨胀层，5 格外不在
        m.set_state([20, 20, 20], VoxelState::Occupied);
        m.inflate_obstacles();
        assert_eq!(m.inflate_at([24, 20, 20]), 1);
        assert_eq!(m.inflate_at([25, 20, 20]), 0);
    }

    #[test]
    fn fade_decays_and_updates_inflation() {
        let mut m = GridMapBuilder::new(0.5, [10, 10, 10])
            .with_obstacles_inflation(0.6)
            .build()
            .unwrap();
        // 构造占据体素并膨胀（step=1 对应 0.2 也有 step=2 对应 0.6，按 0.6 膨胀以验证增量移除覆盖邻域）
        m.set_state([5, 5, 5], VoxelState::Occupied);
        m.inflate_obstacles();
        assert!(m.is_occupied(Vector3::new(2.5, 2.5, 2.5)));
        assert!(m.is_occupied_inflated(Vector3::new(3.0, 2.5, 2.5)));
        // 固定 reduce 语义：必须 2Hz 调用，约 2000 次从 max 衰减到阈值以下
        let reduce = (m.clamp_max_log() - m.min_occupancy_log()) / (m.fading_time() * 2.0);
        let steps_to_cross =
            ((m.clamp_max_log() - m.min_occupancy_log()) / reduce).ceil() as usize + 2;
        for _ in 0..steps_to_cross {
            m.fade();
        }
        // 跨阈值后变为 Free
        assert_eq!(m.state([5, 5, 5]), VoxelState::Free);
        assert!(!m.is_occupied(Vector3::new(2.5, 2.5, 2.5)));
        // 增量移除：膨胀计数归零，>0 判定 false
        assert_eq!(m.inflate_at([5, 5, 5]), 0);
        assert!(!m.is_occupied_inflated(Vector3::new(3.0, 2.5, 2.5)));
        assert!(!m.is_inflated([5, 5, 5]));
        // low_thres 以下体素不继续衰减（已在 clamp_min）
        let before = m.occupancy_at([0, 0, 0]);
        m.fade();
        assert!((m.occupancy_at([0, 0, 0]) - before).abs() < 1e-12);
    }
}
