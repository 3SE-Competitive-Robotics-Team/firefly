//! 占据栅格地图。

use firefly_error::{Error, ErrorKind};
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoxelState {
    Unknown,
    Free,
    Occupied,
}

#[derive(Debug, Clone)]
pub struct GridMap {
    origin: Vector3<f64>,
    resolution: f64,
    dims: [usize; 3],
    voxels: Vec<VoxelState>,
    /// 膨胀层（官方 `occupancy_buffer_inflate_`）：占据体素全向膨胀后的不可达标记。
    inflate: Vec<u8>,
}

#[derive(Debug)]
pub struct GridMapBuilder {
    origin: Vector3<f64>,
    resolution: f64,
    dims: [usize; 3],
}

impl GridMapBuilder {
    #[must_use]
    pub fn new(resolution: f64, dims: [usize; 3]) -> Self {
        Self {
            origin: Vector3::zeros(),
            resolution,
            dims,
        }
    }

    #[must_use]
    pub fn with_origin(mut self, origin: Vector3<f64>) -> Self {
        self.origin = origin;
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

    #[must_use]
    pub fn is_occupied(&self, p: Vector3<f64>) -> bool {
        self.index_of(p)
            .is_none_or(|idx| self.state(idx) == VoxelState::Occupied)
    }

    /// 索引是否落在膨胀层内（官方 `getInflateOccupancy`）。
    #[must_use]
    pub fn is_inflated(&self, idx: [usize; 3]) -> bool {
        self.inflate[self.linear(idx)] == 1
    }

    /// 位置是否落在膨胀层内（官方 `getInflateOccupancy(pos)`，越界视为占据）。
    #[must_use]
    pub fn is_occupied_inflated(&self, p: Vector3<f64>) -> bool {
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
}
