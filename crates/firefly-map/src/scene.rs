//! 场景设计器：用几何体程序化定义障碍 + 起终点，导出 `FFMap` 标准格式。
//!
//! 场景是*内存中的设计工具*，不是地图格式本身；地图一律以
//! `MapFile`（`.ffmap`）落盘（见 `docs/map-format.md`）。

use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector3;

use super::format::{MapFile, Motion, Shape};
use super::{GridMap, GridMapBuilder};

/// 场景：分辨率、原点、体素包围盒、几何障碍、起终点、动态障碍。
#[derive(Debug, Clone)]
pub struct Scene {
    pub resolution: f64,
    pub origin: [f64; 3],
    pub dims: [usize; 3],
    pub obstacles: Vec<Obstacle>,
    pub start: [f64; 3],
    pub goal: [f64; 3],
    /// 动态障碍（运动航点）。
    pub motions: Vec<Motion>,
}

impl Default for Scene {
    fn default() -> Self {
        Self {
            resolution: 0.4,
            origin: [0.0; 3],
            dims: [50, 20, 8],
            obstacles: Vec::new(),
            start: [1.0, 4.0, 1.0],
            goal: [45.0, 4.0, 1.0],
            motions: Vec::new(),
        }
    }
}

/// 几何障碍物。
#[derive(Debug, Clone, Copy)]
pub enum Obstacle {
    Box { center: [f64; 3], size: [f64; 3] },
    Sphere { center: [f64; 3], radius: f64 },
}
impl Scene {
    /// 校验场景字段合法。
    fn validate(&self) -> Result<()> {
        let ok = self.resolution > 0.0 && self.dims.iter().all(|&d| d > 0);
        if !ok {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "scene resolution/dims must be positive",
            ));
        }
        Ok(())
    }

    /// 导出为标准 `FFMap` 文件。
    ///
    /// # Errors
    ///
    /// `InvalidArgument`：场景字段非法。
    pub fn to_map_file(&self) -> Result<MapFile> {
        self.validate()?;
        let grid = self.to_grid_map()?;
        let mut occupied = Vec::new();
        for obstacle in &self.obstacles {
            for idx in Self::voxels_of(*obstacle, &grid) {
                let c = grid_voxel_center(&grid, idx);
                occupied.push([c.x, c.y, c.z]);
            }
        }
        Ok(MapFile {
            resolution: self.resolution,
            origin: self.origin,
            dims: self.dims,
            occupied,
            motions: self.motions.clone(),
        })
    }

    /// 静态障碍栅格化。
    ///
    /// # Errors
    ///
    /// `InvalidArgument`：场景字段非法。
    pub fn to_grid_map(&self) -> Result<GridMap> {
        self.validate()?;
        let map = GridMapBuilder::new(self.resolution, self.dims)
            .with_origin(Vector3::new(self.origin[0], self.origin[1], self.origin[2]))
            .build()?;
        Ok(map)
    }

    /// 障碍覆盖的体素索引列表（静态部分）。
    fn voxels_of(obstacle: Obstacle, map: &GridMap) -> Vec<[usize; 3]> {
        match obstacle {
            Obstacle::Box { center, size } => Shape::Box { center, size }
                .voxels_at(Vector3::new(center[0], center[1], center[2]), map),
            Obstacle::Sphere { center, radius } => Shape::Sphere { center, radius }
                .voxels_at(Vector3::new(center[0], center[1], center[2]), map),
        }
    }
}

/// 体素中心世界坐标。
fn grid_voxel_center(map: &GridMap, idx: [usize; 3]) -> Vector3<f64> {
    Vector3::new(
        map.origin().x + (idx[0] as f64 + 0.5) * map.resolution(),
        map.origin().y + (idx[1] as f64 + 0.5) * map.resolution(),
        map.origin().z + (idx[2] as f64 + 0.5) * map.resolution(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::VoxelState;

    fn sample_scene() -> Scene {
        Scene {
            resolution: 0.5,
            dims: [10, 10, 6],
            obstacles: vec![
                Obstacle::Box {
                    center: [2.0, 2.0, 1.0],
                    size: [1.0, 4.0, 2.0],
                },
                Obstacle::Sphere {
                    center: [5.0, 2.0, 1.0],
                    radius: 0.8,
                },
            ],
            start: [0.5, 0.5, 0.5],
            goal: [7.0, 7.0, 0.5],
            ..Scene::default()
        }
    }

    #[test]
    fn export_map_file_voxelizes_obstacles() {
        let scene = sample_scene();
        let map = scene.to_map_file().unwrap();
        // box 中心被占据
        let grid = map.to_grid_map().unwrap();
        assert!(grid.is_occupied(Vector3::new(2.0, 2.0, 1.0)));
        // 起点不被占据
        assert!(!grid.is_occupied(Vector3::new(0.5, 0.5, 0.5)));
        assert!(grid.is_occupied(Vector3::new(5.0, 2.0, 1.0)));
        assert!(!map.occupied.is_empty());
        // 不重复体素（去重？先对称断言）
        let mut dedup = map.occupied.clone();
        dedup.sort_by(|a, b| a.partial_cmp(b).unwrap());
        dedup.dedup();
        assert_eq!(dedup.len(), map.occupied.len());
    }

    #[test]
    fn map_file_roundtrip_preserves_scene() {
        let scene = sample_scene();
        let map = scene.to_map_file().unwrap();
        let text = map.to_string();
        let reparsed: MapFile = text.parse().unwrap();
        assert!((reparsed.resolution - scene.resolution).abs() < 1e-9);
        assert_eq!(reparsed.dims, scene.dims);
    }

    #[test]
    fn invalid_scene_rejected() {
        let scene = Scene {
            resolution: 0.0,
            ..Scene::default()
        };
        assert_eq!(
            scene.to_map_file().unwrap_err().kind(),
            ErrorKind::InvalidArgument
        );
    }

    #[test]
    fn voxel_state_unknown_by_default() {
        let scene = sample_scene();
        let grid = scene.to_map_file().unwrap().to_grid_map().unwrap();
        // 未被占据的位置是 Unknown，不是 Free
        let idx = grid.index_of(Vector3::new(4.5, 4.5, 0.5)).unwrap();
        assert_eq!(grid.state(idx), VoxelState::Unknown);
    }
}
