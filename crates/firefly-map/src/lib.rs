//! 环境地图领域。
//!
//! 占据栅格地图（局部感知增量更新）+ 平面障碍模型
//! `(x − s)ᵀ v = 0`（EGO-Planner v2 障碍表示，d = (p − s)ᵀ v）。

mod grid;
mod plane;
mod scene;

pub use grid::{GridMap, GridMapBuilder, VoxelState};
pub use plane::{Plane, PlaneDistance};
pub use scene::{Obstacle, Scene};
