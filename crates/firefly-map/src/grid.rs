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
}
