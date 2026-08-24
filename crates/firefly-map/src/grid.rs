//! 占据栅格地图。

use firefly_error::{Error, ErrorKind};
use nalgebra::Vector3;

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
    origin: Vector3<f64>,
    resolution: f64,
    dims: [usize; 3],
    voxels: Vec<VoxelState>,
    /// 膨胀层（官方 `occupancy_buffer_inflate_`）：占据体素全向膨胀后的不可达标记。
    inflate: Vec<u8>,
    /// 虚拟地面/天花板；None = 不启用（官方 `enable_virtual_wall = false` 默认）。
    virtual_wall: Option<VirtualWall>,
}

#[derive(Debug)]
pub struct GridMapBuilder {
    origin: Vector3<f64>,
    resolution: f64,
    dims: [usize; 3],
    virtual_wall: Option<VirtualWall>,
}

impl GridMapBuilder {
    #[must_use]
    pub fn new(resolution: f64, dims: [usize; 3]) -> Self {
        Self {
            origin: Vector3::zeros(),
            resolution,
            dims,
            virtual_wall: None,
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
        let capacity = self.dims[0] * self.dims[1] * self.dims[2];
        let voxels = vec![VoxelState::Unknown; capacity];
        Ok(GridMap {
            origin: self.origin,
            resolution: self.resolution,
            dims: self.dims,
            voxels,
            inflate: vec![0; capacity],
            virtual_wall: self.virtual_wall,
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

    #[must_use]
    pub fn state(&self, idx: [usize; 3]) -> VoxelState {
        self.voxels[self.linear(idx)]
    }

    pub fn set_state(&mut self, idx: [usize; 3], state: VoxelState) {
        let linear = self.linear(idx);
        self.voxels[linear] = state;
    }

    /// 为已构建实例注入虚拟墙（覆盖已有配置）。
    pub fn set_virtual_wall(&mut self, wall: VirtualWall) {
        self.virtual_wall = Some(wall);
    }

    #[must_use]
    pub fn virtual_wall(&self) -> Option<VirtualWall> {
        self.virtual_wall
    }

    /// 官方 `getOccupancy(pos)`：越界或虚拟墙命中均视为占据。
    #[must_use]
    pub fn is_occupied(&self, p: Vector3<f64>) -> bool {
        if self.virtual_wall.is_some_and(|w| w.blocks(p.z)) {
            return true;
        }
        self.index_of(p)
            .is_none_or(|idx| self.state(idx) == VoxelState::Occupied)
    }

    /// 索引是否落在膨胀层内（官方 `getInflateOccupancy`）；以体素中心 z 判虚拟墙。
    #[must_use]
    pub fn is_inflated(&self, idx: [usize; 3]) -> bool {
        if let Some(wall) = self.virtual_wall {
            let cz = self.origin.z + (idx[2] as f64 + 0.5) * self.resolution;
            if wall.blocks(cz) {
                return true;
            }
        }
        self.inflate[self.linear(idx)] == 1
    }

    /// 位置是否落在膨胀层内（官方 `getInflateOccupancy(pos)`，越界或虚拟墙命中视为占据）。
    #[must_use]
    pub fn is_occupied_inflated(&self, p: Vector3<f64>) -> bool {
        if self.virtual_wall.is_some_and(|w| w.blocks(p.z)) {
            return true;
        }
        self.index_of(p).is_none_or(|idx| self.is_inflated(idx))
    }

    /// 按半径重算膨胀层（官方 `clearAndInflateLocalMap` 的膨胀步骤）：
    /// 每个占据体素全向膨胀 `ceil(radius / resolution)` 格。
    pub fn inflate_obstacles(&mut self, radius: f64) {
        self.inflate.fill(0);
        let step = (radius / self.resolution).ceil() as i32;
        if step <= 0 {
            return;
        }
        let [dx, dy, dz] = self.dims;
        let mut occupied = Vec::new();
        for x in 0..dx {
            for y in 0..dy {
                for z in 0..dz {
                    if self.voxels[self.linear([x, y, z])] == VoxelState::Occupied {
                        occupied.push([x as i32, y as i32, z as i32]);
                    }
                }
            }
        }
        for [x, y, z] in occupied {
            for i in -step..=step {
                for j in -step..=step {
                    for k in -step..=step {
                        let (nx, ny, nz) = (x + i, y + j, z + k);
                        if nx < 0
                            || ny < 0
                            || nz < 0
                            || nx >= dx as i32
                            || ny >= dy as i32
                            || nz >= dz as i32
                        {
                            continue;
                        }
                        let l = self.linear([nx as usize, ny as usize, nz as usize]);
                        self.inflate[l] = 1;
                    }
                }
            }
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
        // set/get 往返
        let mut g = GridMapBuilder::new(0.5, [4, 4, 4]).build().unwrap();
        let idx = g.index_of(p).unwrap();
        g.set_state(idx, VoxelState::Occupied);
        assert!(g.is_occupied(p));
    }

    #[test]
    fn out_of_bounds_is_occupied() {
        let m = GridMapBuilder::new(1.0, [4, 4, 4]).build().unwrap();
        assert!(m.is_occupied(Vector3::new(100.0, 0.0, 0.0)));
    }

    #[test]
    fn inflation_expands_obstacles() {
        let mut m = GridMapBuilder::new(0.5, [10, 10, 10]).build().unwrap();
        m.set_state([5, 5, 5], VoxelState::Occupied);
        m.inflate_obstacles(0.6); // step = ceil(0.6/0.5) = 2
        // 本体与 2 格邻域均在膨胀层
        assert!(m.is_occupied_inflated(Vector3::new(2.5, 2.5, 2.5)));
        assert!(m.is_occupied_inflated(Vector3::new(3.5, 2.5, 2.5)));
        // 3 格外不在膨胀层
        assert!(!m.is_occupied_inflated(Vector3::new(4.5, 2.5, 2.5)));
        // 原图状态不受膨胀影响
        assert!(!m.is_occupied(Vector3::new(3.5, 2.5, 2.5)));
    }

    /// 地图 z∈[0,5)，虚拟墙 ground=1.0 / ceil=3.0：三档验证三个查询接口。
    #[test]
    fn virtual_wall_blocks_floor_and_ceiling() {
        let mut m = GridMapBuilder::new(0.5, [10, 10, 10])
            .with_virtual_wall(1.0, 3.0)
            .build()
            .unwrap();
        // 障碍在 (4.25,4.25,2.25)，探针取远端角落避免落入膨胀层
        m.set_state([8, 8, 4], VoxelState::Occupied);
        m.inflate_obstacles(0.6);
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
        let mut plain = GridMapBuilder::new(0.5, [10, 10, 10]).build().unwrap();
        assert_eq!(plain.virtual_wall(), None);
        // 障碍在 (2.5,2.5,0.25)，查询点取远端角落避免落入膨胀层
        plain.set_state([5, 5, 0], VoxelState::Occupied);
        plain.inflate_obstacles(0.6);
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
