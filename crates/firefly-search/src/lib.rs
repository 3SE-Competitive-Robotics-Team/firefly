//! 前端路径搜索。
//!
//! A* 在占据栅格上搜索无碰撞路径，为后端优化提供引导路径
//! （EGO-Planner v2 的 {s, v} 平面障碍即从引导路径生成）。

mod astar;

pub use astar::{Astar, AstarConfig, Path, simplify_path};
