//! 场景通用格式（JSON）：几何障碍 + 起终点，可体素化为 `GridMap`。
//!
//! 地图文件示例：
//! ```json
//! {
//!   "resolution": 0.4,
//!   "origin": [0, 0, 0],
//!   "bounds": [24, 12, 6],
//!   "obstacles": [
//!     { "type": "box", "center": [4, 3, 1.5], "size": [0.8, 6, 3] },
//!     { "type": "sphere", "center": [8, 2, 1], "radius": 0.8 }
//!   ],
//!   "start": [1, 1, 1],
//!   "goal": [20, 3, 1]
//! }
//! ```

use std::fs;

use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::{GridMap, GridMapBuilder, VoxelState};

/// 场景：分辨率、原点、包围盒、障碍物、起终点。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scene {
    pub resolution: f64,
    #[serde(default = "zero_origin")]
    pub origin: [f64; 3],
    /// 体素包围盒维度（[x, y, z] 格数）。
    pub bounds: [usize; 3],
    pub obstacles: Vec<Obstacle>,
    pub start: [f64; 3],
    pub goal: [f64; 3],
}

/// 几何障碍物。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Obstacle {
    Box { center: [f64; 3], size: [f64; 3] },
    Sphere { center: [f64; 3], radius: f64 },
}

fn zero_origin() -> [f64; 3] {
    [0.0; 3]
}

impl Scene {
    /// 从 JSON 文件加载场景。
    ///
    /// # Errors
    ///
    /// `NotFound`：文件不存在；`InvalidData`：JSON 解析失败或字段非法。
    pub fn from_json(path: impl AsRef<std::path::Path>) -> Result<Self> {
        let text = fs::read_to_string(path.as_ref())
            .map_err(|e| Error::new(ErrorKind::NotFound, "scene file not found").with_source(e))?;
        let scene: Scene = serde_json::from_str(&text).map_err(|e| {
            Error::new(ErrorKind::InvalidArgument, "invalid scene json").with_source(e)
        })?;
        scene.validate()?;
        Ok(scene)
    }

    /// 校验字段（分辨率/包围盒/起终点合法）。
    fn validate(&self) -> Result<()> {
        let ok = self.resolution > 0.0
            && self.bounds.iter().all(|&d| d > 0)
            && self.start.len() == 3
            && self.goal.len() == 3;
        if !ok {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "scene fields must be positive",
            ));
        }
        Ok(())
    }

    /// 体素化：障碍覆盖的体素标 `Occupied`，其余 `Unknown`。
    ///
    /// # Errors
    ///
    /// `InvalidArgument`：分辨率非法。
    pub fn to_grid_map(&self) -> Result<GridMap> {
        let mut map = GridMapBuilder::new(self.resolution, self.bounds)
            .with_origin(Vector3::new(self.origin[0], self.origin[1], self.origin[2]))
            .build()?;
        for obstacle in &self.obstacles {
            for idx in self.voxels_of(obstacle) {
                map.set_state(idx, VoxelState::Occupied);
            }
        }
        Ok(map)
    }

    /// 障碍覆盖的体素索引列表。
    fn voxels_of(&self, obstacle: &Obstacle) -> Vec<[usize; 3]> {
        let mut out = Vec::new();
        let (lo, hi) = match obstacle {
            Obstacle::Box { center, size } => {
                let c = Vector3::new(center[0], center[1], center[2]);
                let s = Vector3::new(size[0], size[1], size[2]);
                (c - s / 2.0, c + s / 2.0)
            }
            Obstacle::Sphere { center, radius } => {
                let c = Vector3::new(center[0], center[1], center[2]);
                (
                    c - Vector3::new(*radius, *radius, *radius),
                    c + Vector3::new(*radius, *radius, *radius),
                )
            }
        };
        let (i0, i1, j0, j1, k0, k1) = self.voxel_range(lo, hi);
        for i in i0..i1 {
            for j in j0..j1 {
                for k in k0..k1 {
                    let p = self.voxel_center([i, j, k]);
                    let inside = match obstacle {
                        Obstacle::Box { .. } => true,
                        Obstacle::Sphere { center, radius } => {
                            let c = Vector3::new(center[0], center[1], center[2]);
                            (p - c).norm() <= *radius
                        }
                    };
                    if inside {
                        out.push([i, j, k]);
                    }
                }
            }
        }
        out
    }

    /// 体素索引范围（含下界、不含上界），夹在包围盒内。
    fn voxel_range(
        &self,
        lo: Vector3<f64>,
        hi: Vector3<f64>,
    ) -> (usize, usize, usize, usize, usize, usize) {
        let res = self.resolution;
        let origin = Vector3::new(self.origin[0], self.origin[1], self.origin[2]);
        let clamp =
            |v: f64, dim: usize| v.clamp(origin[dim], origin[dim] + self.bounds[dim] as f64 * res);
        let i0 = ((clamp(lo.x, 0) - origin.x) / res).floor() as usize;
        let i1 = ((clamp(hi.x, 0) - origin.x) / res).ceil() as usize;
        let j0 = ((clamp(lo.y, 1) - origin.y) / res).floor() as usize;
        let j1 = ((clamp(hi.y, 1) - origin.y) / res).ceil() as usize;
        let k0 = ((clamp(lo.z, 2) - origin.z) / res).floor() as usize;
        let k1 = ((clamp(hi.z, 2) - origin.z) / res).ceil() as usize;
        (
            i0,
            i1.min(self.bounds[0]),
            j0,
            j1.min(self.bounds[1]),
            k0,
            k1.min(self.bounds[2]),
        )
    }

    fn voxel_center(&self, idx: [usize; 3]) -> Vector3<f64> {
        Vector3::new(
            self.origin[0] + (idx[0] as f64 + 0.5) * self.resolution,
            self.origin[1] + (idx[1] as f64 + 0.5) * self.resolution,
            self.origin[2] + (idx[2] as f64 + 0.5) * self.resolution,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_roundtrip_and_voxelization() {
        let json = r#"{
            "resolution": 0.5,
            "bounds": [8, 8, 4],
            "obstacles": [
                { "type": "box", "center": [2, 2, 1], "size": [1, 4, 2] },
                { "type": "sphere", "center": [5, 2, 1], "radius": 0.8 }
            ],
            "start": [0.5, 0.5, 0.5],
            "goal": [7, 7, 0.5]
        }"#;
        let scene: Scene = serde_json::from_str(json).unwrap();
        assert_eq!(scene.obstacles.len(), 2);
        let map = scene.to_grid_map().unwrap();
        // box 覆盖 (2,2,1) 附近
        assert!(map.is_occupied(Vector3::new(2.0, 2.0, 1.0)));
        // 起点不被占据
        assert!(!map.is_occupied(Vector3::new(0.5, 0.5, 0.5)));
        // sphere 中心覆盖
        assert!(map.is_occupied(Vector3::new(5.0, 2.0, 1.0)));
    }

    #[test]
    fn invalid_scene_rejected() {
        let json = r#"{"resolution": 0, "bounds": [8, 8, 4], "obstacles": [], "start": [0,0,0], "goal": [1,1,1]}"#;
        let scene: Scene = serde_json::from_str(json).unwrap();
        assert!(scene.to_grid_map().is_err());
    }
}
