//! PX4 多旋翼控制核心的 1:1 移植。
//!
//! 参考固定版本见 `~/Projects/PX4-Autopilot`（提交 `4dbd2e0`，BSD-3）。模块内部保持 PX4 的 NED/FRD
//! 约定与归一化指令；与产品 Z-up/FLU 坐标的转换只在 [`crate::control`] 的边界发生。
//! 每个文件对应一个 PX4 类（或一个命名空间），函数、字段与数学步骤保持一致。

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
