//! 多旋翼控制核心；坐标、单位与归一化约定见 [`types`]，产品边界见 [`controller`]。

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
