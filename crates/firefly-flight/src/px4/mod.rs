//! PX4 多旋翼控制与分配算法的 Rust 移植。
//!
//! 参考固定版本见 `~/Projects/PX4-Autopilot`（提交 `4dbd2e0`，BSD-3）。模块内部保持 PX4 的 NED/FRD
//! 约定与归一化指令；与产品 Z-up/FLU 坐标的转换在 [`controller::Controller`] 边界发生。
//! 模块按 PX4 类与命名空间组织，不包含平台调度和参数服务。

pub mod allocation;
pub mod attitude_control;
pub mod attitude_control_math;
pub mod control_math;
pub mod controller;
pub mod frames;
pub mod position_control;
pub mod rate_control;
pub mod types;
pub mod velocity_smoothing;
