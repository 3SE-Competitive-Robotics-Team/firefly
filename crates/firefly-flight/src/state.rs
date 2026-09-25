//! 机体状态与控制输出、六自由度积分。

use glam::{Quat, Vec3};

use crate::{G, params::QuadParams};

/// 控制输出：机体产生的世界系外力（N）与世界系力矩（N·m）。
///
/// 力含重力补偿与气动阻尼，**不含重力本身**——悬停时 `force.z ≈ m·g`，重力由消费方
///（`MuJoCo` 的 `xfrc_applied` 或 [`integrate`]）另加。可直接写进 `xfrc_applied`。
#[derive(Clone, Copy, Debug, Default)]
pub struct Wrench {
    /// 世界系力（N）。
    pub force: Vec3,
    /// 世界系力矩（N·m）。
    pub torque: Vec3,
}

/// 六自由度机体状态。
#[derive(Clone, Copy, Debug, Default)]
pub struct QuadState {
    /// 世界系位置（m）。
    pub position: Vec3,
    /// 世界系速度（m/s）。
    pub velocity: Vec3,
    /// 姿态（机体→世界）。
    pub attitude: Quat,
    /// 机体系角速度（rad/s）。
    pub ang_vel: Vec3,
}

impl QuadState {
    /// 水平航向（rad，世界 Z；机体 `+X` 在水平面的投影）。
    #[must_use]
    pub fn yaw(&self) -> f32 {
        let fwd = self.attitude * Vec3::X;
        fwd.y.atan2(fwd.x)
    }
}

/// 六自由度积分（半隐式欧拉）：重力世界系 −`z`，姿态 `q̇ = 0.5·q⊗ω_body`。
///
/// `wrench` 由 [`angle_mode`](crate::angle_mode)/[`position_mode`](crate::position_mode)
/// 给出（已含气动阻尼）；积分本身不再追加阻尼。
pub fn integrate(state: &mut QuadState, wrench: &Wrench, params: &QuadParams, dt: f32) {
    let accel = wrench.force / params.mass + Vec3::NEG_Z * G;
    state.velocity += accel * dt;
    state.position += state.velocity * dt;

    // 世界系力矩 → 机体系角加速度（惯量取对角元）：ω̇ = I⁻¹·R⁻¹·τ
    let ang_accel = (state.attitude.inverse() * wrench.torque) / Vec3::from(params.inertia);
    state.ang_vel += ang_accel * dt;

    let wq = Quat::from_xyzw(state.ang_vel.x, state.ang_vel.y, state.ang_vel.z, 0.0);
    let dq = (state.attitude * wq) * (0.5 * dt);
    state.attitude = Quat::from_xyzw(
        state.attitude.x + dq.x,
        state.attitude.y + dq.y,
        state.attitude.z + dq.z,
        state.attitude.w + dq.w,
    )
    .normalize();
}
