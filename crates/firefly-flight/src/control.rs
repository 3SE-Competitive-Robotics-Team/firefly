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
