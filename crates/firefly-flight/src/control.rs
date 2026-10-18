//! 位置参考（产品世界系，Z 上）。
//!
//! 产品飞控（`apps/fc`）由 [`crate::FlightFsm`] 生成该参考，再经
//! [`crate::Controller`] 转到 PX4 的 NED 与归一化指令。手动飞行（`apps/quad`）
//! 不经过本结构，直接用 [`crate::Controller::attitude_mode`]。

use nalgebra::Vector3;

/// 位置模式参考（世界系，Z 上）。
#[derive(Clone, Copy, Debug, Default)]
pub struct PositionSetpoint {
    /// 期望位置（m）。
    pub position: Vector3<f32>,
    /// 期望速度（m/s）。
    pub velocity: Vector3<f32>,
    /// 期望加速度（m/s²，世界系）：位置环前馈。
    pub acceleration: Vector3<f32>,
    /// 期望航向（rad，世界 Z；水平面内机体 `+X` 的方向）。
    pub yaw: f32,
    /// 期望航向角速度（rad/s，姿态内环前馈）。
    pub yaw_rate: f32,
}
