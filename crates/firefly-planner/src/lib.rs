//! EGO 规划编排（应用层）。
//!
//! 组合领域能力：A* 前端（firefly-search）→ MINCO 后端
//! （firefly-trajectory）→ L-BFGS 优化（firefly-optimize），
//! 环境来自 firefly-map。配置参数取自论文 Table S6。

mod config;
pub mod init;
pub mod manager;
pub mod multitopo;
pub mod objective;
mod obstacles;
mod planner;

pub use config::PlannerConfig;
pub use manager::{LocalTraj, ManagerOptions, PlannerManager, Reference, TickReport};
pub use obstacles::{CollisionSpan, PointsToCheck, SamplePoint};
pub use planner::{FormationSpec, InitSource, PlanResult, Planner, State};
