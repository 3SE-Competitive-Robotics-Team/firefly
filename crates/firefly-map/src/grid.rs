//! 占据栅格地图（log-odds 概率表示 + ringbuffer 局部窗口，对照 EGO-Planner-v2
//! `plan_env/grid_map`）。
//!
//! - 存储为 `occupancy: Vec<f64>`（log-odds），初始为 `clamp_min_log_`
//!   （`logit(p_min)`，`p_min=0.12`）；`state()` 按 `occupancy >= min_occupancy_log_`
//!   判 `Occupied`，否则 `Free`。对照官方 `occupancy_buffer_`（参数初始化
//!   `grid_map.cpp:34-77`、logit 宏 `grid_map.h:27`）。`VoxelState::Unknown`
//!   仅兼容历史测试代码（写入等价 `clamp_min`）。
//! - 膨胀层为计数缓冲（对照官方 `occupancy_buffer_inflate_`）：障碍体素自身
//!   `±GRID_MAP_OBS_FLAG`、周围 `inf_grid` 格内 ±1，`>0` 判膨胀；`inf_grid`
//!   构造时按 `obstacles_inflation` 计算，超 4 自动放大分辨率
//!   （官方 grid_map.cpp:44-48）。
//! - **ringbuffer 局部窗口**：buffer 尺寸 = 2 × `local_update_range`（官方
//!   `ringbuffer_size3i_ = 2 * local_update_range3i_`，`grid_map.cpp:55`），随相机
//!   平移。`move_ring_buffer(center)` 对照官方 `moveRingBuffer`
//!   （grid_map.cpp:399-457）：逐轴按中心移动方向清除移出窗口的体素
//!   （occupancy 重置 `clamp_min_log`、障碍源从膨胀层反向移除，对照
//!   `clearBuffer` `grid_map.cpp:747`），环形寻址原点取模对齐新边界。
//!   存储地址 = `(全局索引 − 环形原点) % buffer尺寸`（官方 `globalIdx2BufIdx`，
//!   `grid_map.h:318`），存活体素不搬移、地址环形复用。
//! - **双参照系**：对外体素坐标是**窗口坐标**——原点取当前窗口下界角点并随
//!   平移移动（`origin()` 返回其世界坐标）。A* / 体素遍历等索引式消费方的
//!   几何框架因此跨平移保持刚性；官方消费方全部按位置查询，无此约定。
//!   跨平移持久的簿记用世界锚定的全局索引（[`GridMap::pos_to_global_index`] /
//!   [`GridMap::global_index`] / [`GridMap::window_index`] 换算）。位置查询
//!   （`is_occupied` 等）按当前窗口判定：窗口外视为自由（官方 `getOccupancy`
//!   的 `!isInBuf → 0` 分支）；虚拟墙例外——墙是世界系物理约束，窗口外仍生效。
//! - 官方另有 `GridMapBigmap` 全局地图变体（`grid_map_bigmap.h/cpp`），
//!   swarm-playground 中无任何引用，不对齐。

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
    /// 局部更新范围（米，官方 `grid_map/local_update_range_x/y/z`，launch
    /// advanced_param.xml:79-81 默认 5.5/5.5/2.0）。Some 时覆盖 `dims`：
    /// buffer 尺寸 = 2 × ceil(range / resolution)（官方 `grid_map.cpp:55`）。
    local_update_range: Option<[f64; 3]>,
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
            local_update_range: None,
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

    /// 以局部更新范围（米）确定 buffer 尺寸 = 2 × ceil(range/resolution)
    /// （官方 `ringbuffer_size3i_ = 2 * local_update_range3i_`，`grid_map.cpp:55`），
    /// 覆盖 `new(resolution, dims)` 给出的 dims。官方 launch 默认 5.5/5.5/2.0。
    #[must_use]
    pub fn with_local_update_range(mut self, range: [f64; 3]) -> Self {
        self.local_update_range = Some(range);
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
    /// `InvalidArgument`：resolution 非正、dims 含零、或 `local_update_range`
    /// 非有限/非正。
    pub fn build(self) -> firefly_error::Result<GridMap> {
        if self.resolution <= 0.0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "resolution must be positive",
            ));
        }
        if let Some(range) = self.local_update_range
            && range.iter().any(|&v| !v.is_finite() || v <= 0.0)
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "local_update_range must be finite and positive",
            ));
        }
        // 膨胀步长与分辨率（官方 grid_map.cpp:44-48）：inf_grid > 4 时放大分辨率；
        // 先定分辨率再换算范围/尺寸（与官方顺序一致）
        let mut resolution = self.resolution;
        let mut inf_grid = ((self.obstacles_inflation - 1e-5) / resolution).ceil() as i32;
        if inf_grid > 4 {
            inf_grid = 4;
            resolution = self.obstacles_inflation / f64::from(inf_grid);
        }
        let dims = match self.local_update_range {
            Some(range) => {
                let inv = 1.0 / resolution;
                [
                    (2.0 * range[0] * inv).ceil().max(1.0) as usize,
                    (2.0 * range[1] * inv).ceil().max(1.0) as usize,
                    (2.0 * range[2] * inv).ceil().max(1.0) as usize,
                ]
            }
            None => self.dims,
        };
        if dims.contains(&0) {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "dims must be positive",
            ));
        }
        let capacity = dims[0] * dims[1] * dims[2];
        // 膨胀缓冲每侧多 inf_grid 格（官方 grid_map.cpp:56
        // `ringbuffer_inf_size3i_ = ringbuffer_size3i_ + 2*inf_grid`）
        let g = inf_grid.unsigned_abs() as usize;
        let inf_dims = [dims[0] + 2 * g, dims[1] + 2 * g, dims[2] + 2 * g];
        let inf_capacity = inf_dims[0] * inf_dims[1] * inf_dims[2];
        let prob_hit_log = logit(self.p_hit);
        let prob_miss_log = logit(self.p_miss);
        let clamp_min_log = logit(self.p_min);
        let clamp_max_log = logit(self.p_max);
        let min_occupancy_log = logit(self.p_occ);

        // 初始窗口覆盖整个 buffer：低维全局索引 [0, dims)，环形原点对齐 0，
        // 与静态地图行为一致（首次 move_ring_buffer 才开始平移）
        let dims_i64 = [dims[0] as i64, dims[1] as i64, dims[2] as i64];
        let ig = i64::from(inf_grid);
        Ok(GridMap {
            world_origin: self.origin,
            resolution,
            dims,
            inf_dims,
            local_update_range3i: [dims_i64[0] / 2, dims_i64[1] / 2, dims_i64[2] / 2],
            low: [0; 3],
            up: [dims_i64[0] - 1, dims_i64[1] - 1, dims_i64[2] - 1],
            inf_low: [-ig; 3],
            inf_up: [
                dims_i64[0] - 1 + ig,
                dims_i64[1] - 1 + ig,
                dims_i64[2] - 1 + ig,
            ],
            ring_origin: [0; 3],
            inf_ring_origin: [0; 3],
            center_last: [dims_i64[0] / 2, dims_i64[1] / 2, dims_i64[2] / 2],
            occupancy: vec![clamp_min_log; capacity],
            inflate: vec![0; inf_capacity],
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

    /// 世界位置 → 全局体素索引；仅在落入当前窗口时返回（crate 内光线步进用）。
    pub(crate) fn window_global(&self, p: Vector3<f64>) -> Option<[i64; 3]> {
        let g = self.pos_to_global_index(p);
        self.in_window_global(g).then_some(g)
    }

    /// 全局索引是否在当前占据窗口内（对照官方 `isInBuf`）。
    pub(crate) fn in_window_global(&self, g: [i64; 3]) -> bool {
        g.iter()
            .zip(&self.low)
            .zip(&self.up)
            .all(|((&gi, &lo), &up)| gi >= lo && gi <= up)
    }

    /// 全局索引是否在膨胀缓冲窗口内（对照官方 `isInInfBuf`）。
    fn in_inf_window(&self, g: [i64; 3]) -> bool {
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
    fn occ_addr(&self, g: [i64; 3]) -> usize {
        let mut c = [0usize; 3];
        for i in 0..3 {
            let s = self.dims[i] as i64;
            let mut v = (g[i] - self.ring_origin[i]) % s;
            if v < 0 {
                v += s;
            }
            c[i] = v as usize;
        }
        (c[0] * self.dims[1] + c[1]) * self.dims[2] + c[2]
    }

    /// 全局索引 → 膨胀缓冲线性地址（独立环形原点/尺寸，同上映射）。
    fn inf_addr(&self, g: [i64; 3]) -> usize {
        let mut c = [0usize; 3];
        for i in 0..3 {
            let s = self.inf_dims[i] as i64;
            let mut v = (g[i] - self.inf_ring_origin[i]) % s;
            if v < 0 {
                v += s;
            }
            c[i] = v as usize;
        }
        (c[0] * self.inf_dims[1] + c[1]) * self.inf_dims[2] + c[2]
    }

    /// 占据缓冲线性地址 → 全局索引（官方 `BufIdx2GlobalIdx` 的逆映射：
    /// 分量加环形原点，超出上界回绕一个 buffer 尺寸）。
    fn addr_global(&self, a: usize) -> [i64; 3] {
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

    /// 体素占据状态查询（`occupancy >= min_occupancy_log_` → Occupied，否则 Free）。
    /// `Unknown` 保留在枚举中但不再作为初始/查询结果（见模块头注）。
    #[must_use]
    pub fn state(&self, idx: [usize; 3]) -> VoxelState {
        let occ = self.occupancy[self.occ_addr(self.global_index(idx))];
        if occ >= self.min_occupancy_log {
            VoxelState::Occupied
        } else {
            VoxelState::Free
        }
    }

    /// 兼容接口：`Occupied` → `clamp_max_log_`，`Free`/`Unknown` → `clamp_min_log_`。
    pub fn set_state(&mut self, idx: [usize; 3], state: VoxelState) {
        let a = self.occ_addr(self.global_index(idx));
        match state {
            VoxelState::Occupied => self.occupancy[a] = self.clamp_max_log,
            VoxelState::Free | VoxelState::Unknown => self.occupancy[a] = self.clamp_min_log,
        }
    }

    /// log-odds 增量更新（对照 `grid_map.cpp:658-669` 的 clamp 语义）。
    pub fn update_occupancy(&mut self, idx: [usize; 3], delta: f64) {
        self.update_occupancy_global(self.global_index(idx), delta);
    }

    /// 全局索引版 log-odds 增量更新（crate 内光线步进用）。
    pub(crate) fn update_occupancy_global(&mut self, g: [i64; 3], delta: f64) {
        let a = self.occ_addr(g);
        let v = self.occupancy[a] + delta;
        self.occupancy[a] = v.clamp(self.clamp_min_log, self.clamp_max_log);
    }

    /// 直接读取体素的 log-odds 值（测试/诊断用）。
    #[must_use]
    pub fn occupancy_at(&self, idx: [usize; 3]) -> f64 {
        self.occupancy[self.occ_addr(self.global_index(idx))]
    }

    /// 直接读取膨胀计数（测试用，对照官方 `occupancy_buffer_inflate_`）。
    #[must_use]
    pub fn inflate_at(&self, idx: [usize; 3]) -> u16 {
        self.inflate[self.inf_addr(self.global_index(idx))]
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

    /// 官方 `getOccupancy(pos)`：虚拟墙命中视为占据；窗口外视为自由
    /// （官方 `!isInBuf → 0`）；否则阈值判定。
    #[must_use]
    pub fn is_occupied(&self, p: Vector3<f64>) -> bool {
        if self.virtual_wall.is_some_and(|w| w.blocks(p.z)) {
            return true;
        }
        let Some(g) = self.window_global(p) else {
            return false;
        };
        self.occupancy[self.occ_addr(g)] >= self.min_occupancy_log
    }

    /// 索引是否落在膨胀层内（对照官方 `getInflateOccupancy` 下游 `>0` 判膨胀）。
    #[must_use]
    pub fn is_inflated(&self, idx: [usize; 3]) -> bool {
        if let Some(wall) = self.virtual_wall {
            let cz =
                self.world_origin.z + (self.global_index(idx)[2] as f64 + 0.5) * self.resolution;
            if wall.blocks(cz) {
                return true;
            }
        }
        self.inflate[self.inf_addr(self.global_index(idx))] > 0
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
        self.change_inf_buf_global(dir, self.global_index(idx));
    }

    /// 全局索引版膨胀增量更新（`clearBuffer`/`fade`/`inflate_obstacles` 共用）。
    fn change_inf_buf_global(&mut self, dir: bool, g: [i64; 3]) {
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
    }

    /// 清除移出窗口的体素条带（对照官方 `clearBuffer(casein, bound)`，
    /// `grid_map.cpp:747`）：case 偶数 = 向负方向移动（清 `[bound, 旧上界]`）、
    /// 奇数 = 向正方向移动（清 `[旧下界, bound]`），正交两轴取旧窗口全域；
    /// 移动方向的边界列多清一格，与官方一致。occupancy 重置 `clamp_min_log`，
    /// 障碍源从膨胀层反向移除（`>= FLAG`：firefly 的邻域更新跳过窗外体素，
    /// 窗缘障碍可能恰持 FLAG，须一并清除）。`count_hit`/`flag` 缓冲无对应物，跳过。
    fn clear_buffer(&mut self, case: usize, bound: i64) {
        let x0 = if case == 0 { bound } else { self.low[0] };
        let x1 = if case == 1 { bound } else { self.up[0] };
        let y0 = if case == 2 { bound } else { self.low[1] };
        let y1 = if case == 3 { bound } else { self.up[1] };
        let z0 = if case == 4 { bound } else { self.low[2] };
        let z1 = if case == 5 { bound } else { self.up[2] };
        for x in x0..=x1 {
            for y in y0..=y1 {
                for z in z0..=z1 {
                    let g = [x, y, z];
                    let a = self.occ_addr(g);
                    self.occupancy[a] = self.clamp_min_log;
                    let ia = self.inf_addr(g);
                    if self.inflate[ia] >= GRID_MAP_OBS_FLAG {
                        self.change_inf_buf_global(false, g);
                    }
                }
            }
        }
    }

    /// 重算膨胀层（官方 `clearAndInflateLocalMap` 的膨胀步骤，计数语义；步长为构造期确定的 `inf_grid`）。
    /// 每个占据体素 `+GRID_MAP_OBS_FLAG`（自身）并周围 `inf_grid` 格内 `+1`，多障碍叠加计数。
    pub fn inflate_obstacles(&mut self) {
        self.inflate.iter_mut().for_each(|v| *v = 0);
        if self.inf_grid <= 0 {
            return;
        }
        let threshold = self.min_occupancy_log;
        let occupied: Vec<[i64; 3]> = (0..self.occupancy.len())
            .filter(|&a| self.occupancy[a] >= threshold)
            .map(|a| self.addr_global(a))
            .collect();
        for g in occupied {
            self.change_inf_buf_global(true, g);
        }
    }

    /// 地图衰减（对照官方 `grid_map.cpp:204-222` `fadingCallback`，必须 2Hz 调用）。
    ///
    /// - `reduce = (clamp_max_log_ - min_occupancy_log_) / (fading_time * 2)` 固定值，对应官方 2Hz 定时器（每 0.5s）。
    /// - `low_thres = clamp_min_log_ + reduce`，仅 `> low_thres` 的体素衰减，避免跌破下界。
    /// - 衰减前 `>= min_occupancy` 且衰减后 `<` 的体素（障碍→自由）经全局索引
    ///   调用 `change_inf_buf_global(false, g)` 增量移除膨胀（官方
    ///   `BufIdx2GlobalIdx(i)` 反解后跨缓冲定位）。
    pub fn fade(&mut self) {
        if self.fading_time <= 0.0 {
            return;
        }
        let reduce = (self.clamp_max_log - self.min_occupancy_log) / (self.fading_time * 2.0);
        if reduce <= 0.0 {
            return;
        }
        let low_thres = self.clamp_min_log + reduce;
        // 先更新占据并收集跨阈值的全局索引
        let mut crossed: Vec<[i64; 3]> = Vec::new();
        for a in 0..self.occupancy.len() {
            let occ = self.occupancy[a];
            if occ > low_thres {
                let was_occupied = occ >= self.min_occupancy_log;
                let mut new_occ = occ - reduce;
                if new_occ < self.clamp_min_log {
                    new_occ = self.clamp_min_log;
                }
                self.occupancy[a] = new_occ;
                if was_occupied && new_occ < self.min_occupancy_log {
                    crossed.push(self.addr_global(a));
                }
            }
        }
        // 增量移除膨胀（计数缓冲）
        for g in crossed {
            self.change_inf_buf_global(false, g);
        }
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
    fn builder_rejects_bad_dims_and_range() {
        assert_eq!(
            GridMapBuilder::new(0.5, [0, 10, 10])
                .build()
                .unwrap_err()
                .kind(),
            ErrorKind::InvalidArgument
        );
        assert_eq!(
            GridMapBuilder::new(0.5, [1, 1, 1])
                .with_local_update_range([5.5, 0.0, 2.0])
                .build()
                .unwrap_err()
                .kind(),
            ErrorKind::InvalidArgument
        );
    }

    #[test]
    fn local_update_range_sets_buffer_size() {
        // 官方 launch advanced_param.xml:79-81 默认 5.5/5.5/2.0，res 0.4 →
        // ceil(5.5/0.4)=14、ceil(2/0.4)=5，buffer = 2× → [28,28,10]
        let m = GridMapBuilder::new(0.4, [1, 1, 1])
            .with_local_update_range([5.5, 5.5, 2.0])
            .build()
            .unwrap();
        assert_eq!(m.dims(), [28, 28, 10]);
        let lr = m.local_update_range();
        assert!((lr.x - 5.6).abs() < 1e-12); // 14 格 × 0.4
        assert!((lr.y - 5.6).abs() < 1e-12);
        assert!((lr.z - 2.0).abs() < 1e-12);
        // 初始窗口覆盖整个 buffer（静态语义）
        assert_eq!(m.index_of(Vector3::new(0.1, 0.1, 0.1)), Some([0, 0, 0]));
        assert_eq!(m.index_of(Vector3::new(11.1, 11.1, 3.9)), Some([27, 27, 9]));
        assert_eq!(m.index_of(Vector3::new(11.3, 0.1, 0.1)), None);
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

    // --- ringbuffer 平移 ---

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
        m.update_occupancy(idx, m.prob_hit_log());
        assert!(m.occupancy_at(idx) > m.clamp_min_log());
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
        // 回收槽位写入新障碍：全局 x=6（窗口坐标 3）经环形映射可查询，
        // 旧 x=3 不复现
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

    /// fade 作用于当前窗口内容：平移后被清空区域不参与衰减，幸存障碍正常衰减。
    #[test]
    fn fade_works_after_translation() {
        let mut m = GridMapBuilder::new(1.0, [6, 4, 4]).build().unwrap();
        m.set_state([1, 2, 2], VoxelState::Occupied);
        // 右移两格：障碍仍在窗内（全局 x=1 → 窗口坐标 0... 窗口 [2,7]，x=1 出窗）
        m.move_ring_buffer(Vector3::new(4.5, 2.5, 2.5));
        assert!(!m.is_occupied(Vector3::new(1.5, 2.5, 2.5)));
        // 窗内新障碍（窗口坐标 5 = 全局 x=6）
        m.set_state([5, 2, 2], VoxelState::Occupied);
        assert!(m.is_occupied(Vector3::new(6.5, 2.5, 2.5)));
        let reduce = (m.clamp_max_log() - m.min_occupancy_log()) / (m.fading_time() * 2.0);
        let steps = ((m.clamp_max_log() - m.min_occupancy_log()) / reduce).ceil() as usize + 2;
        for _ in 0..steps {
            m.fade();
        }
        // 幸存障碍衰减至 Free，出窗障碍不影响判定
        assert!(!m.is_occupied(Vector3::new(6.5, 2.5, 2.5)));
        assert_eq!(m.state([4, 2, 2]), VoxelState::Free);
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
