//! 飞行 demo 的 Bevy 接线：键盘 → [`AngleCommand`] → `firefly-flight` 的角度模式
//! → 旋翼分配（限幅）→ 六自由度积分 → 写回 `Transform`。
//!
//! 模型与飞控本体（推力沿机体 z、4 旋翼分配、姿态内环、惯量、饱和）在
//! `firefly-flight`，本模块只做输入与 ECS 搬运。

use bevy::prelude::*;
use firefly_flight::{AngleCommand, QuadState, angle_mode, integrate};

use crate::config::QuadConfig;

/// 机体状态（挂机体根实体；位置/姿态同步到 `Transform`）。
#[derive(Component, Default)]
pub struct Quad {
    /// 六自由度状态。
    pub state: QuadState,
    /// 最近一次分配的 4 电机推力（N，HUD 显示）。
    pub motors: [f32; 4],
    /// 最近一次分配是否触限。
    pub saturated: bool,
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

/// 定步长积分（`FixedUpdate`）：角度模式 → 期望力/力矩 → 旋翼分配 → 实际力/力矩 → 积分。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
pub fn dynamics(
    time: Res<Time>,
    cfg: Res<QuadConfig>,
    input: Res<QuadInput>,
    drone: Single<(&mut Quad, &mut Transform)>,
) {
    let (mut quad, mut transform) = drone.into_inner();
    let dt = time.delta_secs().min(1.0 / 120.0);

    if input.reset {
        quad.state = QuadState {
            position: Vec3::from(cfg.start),
            ..default()
        };
    } else {
        let cmd = AngleCommand {
            pitch: input.pitch,
            roll: input.roll,
            yaw: input.yaw,
            throttle: input.throttle,
        };
        let desired = angle_mode(&quad.state, &cmd, &cfg.drone, &cfg.control);
        let alloc = cfg.airframe.allocate(quad.state.attitude, &desired);
        quad.motors = alloc.motors;
        quad.saturated = alloc.saturated;
        integrate(&mut quad.state, &alloc.realized, &cfg.drone, dt);
    }

    transform.translation = quad.state.position;
    transform.rotation = quad.state.attitude;
}
