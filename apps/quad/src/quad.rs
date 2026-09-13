//! 四旋翼动力学（6-DOF 刚体 + 角度模式姿态控制）。
//!
//! 机体轴：`+X` 前、`+Y` 左、`+Z` 上。姿态由 `Transform.rotation`（机体→世界）表示，
//! 角速度存机体系。推力沿机体 `+Z`，重力沿世界 `-Z`。
//!
//! 控制为角度模式：`WASD` 给期望俯仰/横滚（松手回正），`Space/Shift` 给升降速度，
//! `Q/E` 给偏航角速度。姿态误差 → 机体角速度指令 → 角加速度，简单比例控制。

use bevy::prelude::*;

use crate::config::QuadConfig;

/// 重力加速度（m/s²）。
const G: f32 = 9.81;

/// 6-DOF 四旋翼状态（挂机体根实体上）。
#[derive(Component, Default)]
pub struct Quad {
    /// 世界系速度（m/s）。
    pub velocity: Vec3,
    /// 机体系角速度（rad/s）。
    pub ang_vel: Vec3,
    /// 累积偏航（rad）。
    pub yaw: f32,
}

/// 每帧控制输入（归一化 -1..1）。
#[derive(Resource, Default)]
pub struct QuadInput {
    /// 俯仰（W=+1 前倾）。
    pub pitch: f32,
    /// 横滚（D=+1 右倾）。
    pub roll: f32,
    /// 偏航（E=+1 右偏）。
    pub yaw: f32,
    /// 油门（Space=+1 上升）。
    pub throttle: f32,
    /// 重置（R）。
    pub reset: bool,
}

/// 键盘 → 控制输入。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
pub fn read_input(keys: Res<ButtonInput<KeyCode>>, mut input: ResMut<QuadInput>) {
    let axis = |pos: KeyCode, neg: KeyCode| {
        f32::from(u8::from(keys.pressed(pos))) - f32::from(u8::from(keys.pressed(neg)))
    };
    input.pitch = axis(KeyCode::KeyW, KeyCode::KeyS);
    input.roll = axis(KeyCode::KeyD, KeyCode::KeyA);
    input.yaw = axis(KeyCode::KeyE, KeyCode::KeyQ);
    input.throttle = axis(KeyCode::Space, KeyCode::ShiftLeft);
    input.reset = keys.just_pressed(KeyCode::KeyR);
}

/// 定步长积分（`FixedUpdate`）。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
pub fn dynamics(
    time: Res<Time>,
    cfg: Res<QuadConfig>,
    input: Res<QuadInput>,
    drone: Single<(&mut Quad, &mut Transform)>,
) {
    let (mut quad, mut transform) = drone.into_inner();
    let body = &cfg.drone;
    let ctl = &cfg.control;
    let dt = time.delta_secs().min(1.0 / 120.0);

    if input.reset {
        quad.velocity = Vec3::ZERO;
        quad.ang_vel = Vec3::ZERO;
        quad.yaw = 0.0;
        transform.translation = Vec3::from(cfg.start);
        transform.rotation = Quat::IDENTITY;
        return;
    }

    // 期望姿态：+roll 左倾、+pitch 前倾（绕 +Y 正转把机体 +Z 压向前）。
    let tilt = ctl.tilt_max_deg.to_radians();
    let roll = -input.roll * tilt;
    let pitch = input.pitch * tilt;
    quad.yaw -= input.yaw * ctl.yaw_rate_max * dt;
    let q_des = Quat::from_rotation_z(quad.yaw)
        * Quat::from_rotation_y(pitch)
        * Quat::from_rotation_x(roll);

    // 姿态误差（机体系）→ 角速度指令 → 角加速度。
    let q_err = transform.rotation.inverse() * q_des;
    let (axis, angle) = q_err.to_axis_angle();
    let e = if angle.is_finite() {
        axis * angle
    } else {
        Vec3::ZERO
    };
    let rate_des = e * ctl.attitude_kp;
    let rate_error = rate_des - quad.ang_vel;
    quad.ang_vel += rate_error * ctl.rate_kp * dt;
    quad.ang_vel *= 1.0 - (body.angular_drag * dt).min(0.5);

    // 推力：垂直速度控制 + 倾斜补偿（保持高度）。
    let up_z = (transform.rotation * Vec3::Z).z.clamp(0.3, 1.0);
    let vz_des = input.throttle * ctl.climb_rate_max;
    let a_vert = G + ctl.vz_kp * (vz_des - quad.velocity.z);
    let thrust = (body.mass * a_vert / up_z).clamp(0.0, body.max_thrust);
    let force = transform.rotation * Vec3::new(0.0, 0.0, thrust);
    let accel = force / body.mass + Vec3::new(0.0, 0.0, -G)
        - quad.velocity * (body.linear_drag / body.mass);
    quad.velocity += accel * dt;
    transform.translation += quad.velocity * dt;

    // 姿态积分：q_dot = 0.5 · q ⊗ ω_body。
    let wq = Quat::from_xyzw(quad.ang_vel.x, quad.ang_vel.y, quad.ang_vel.z, 0.0);
    let dq = (transform.rotation * wq) * (0.5 * dt);
    transform.rotation = Quat::from_xyzw(
        transform.rotation.x + dq.x,
        transform.rotation.y + dq.y,
        transform.rotation.z + dq.z,
        transform.rotation.w + dq.w,
    )
    .normalize();
}
