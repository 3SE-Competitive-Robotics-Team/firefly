//! 第三人称飞行的 Bevy 接线：键盘 → 期望姿态/推力 → PX4 控制链
//! （`AttitudeControl` → `RateControl` → `ControlAllocation`）→ 六自由度积分 → `Transform`。
//!
//! 模型与飞控本体在 `firefly-flight`，本模块只做输入、坐标搬运和手柄到
//! 姿态/推力的映射。

use bevy::prelude::*;
use firefly_flight::{Controller, G, Px4Params, QuadState, integrate, yaw_of};
use nalgebra::{UnitQuaternion, Vector3};

use crate::config::QuadConfig;

/// 机体状态（挂机体根实体；位置/姿态同步到 `Transform`）。
#[derive(Component)]
pub struct Quad {
    /// 六自由度状态。
    pub state: QuadState,
    /// 最近一次分配的 4 电机推力（N，HUD 显示）。
    pub motors: [f32; 4],
    /// 最近一次分配是否触限。
    pub saturated: bool,
    /// PX4 控制链（角速度 → 姿态 → 分配）。
    pub ctrl: Controller,
}

impl Quad {
    /// 以飞控参数构造；机体参数在每 tick 由 `QuadConfig` 注入。
    #[must_use]
    pub fn new(params: Px4Params) -> Self {
        Self {
            state: QuadState::default(),
            motors: [0.0; 4],
            saturated: false,
            ctrl: Controller::new(params),
        }
    }
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

/// 定步长（`FixedUpdate`）：手柄 → 期望姿态/推力 → PX4 控制链 → 积分。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
pub fn dynamics(
    time: Res<Time<Fixed>>,
    cfg: Res<QuadConfig>,
    input: Res<QuadInput>,
    drone: Single<(&mut Quad, &mut Transform)>,
) {
    let (mut quad, mut transform) = drone.into_inner();
    // 固定 240Hz 步长；PX4 内环参数按这个 dt 配置。
    let dt = time.delta_secs();

    if input.reset {
        quad.state = QuadState {
            position: cfg.start.into(),
            ..default()
        };
        quad.ctrl.reset();
    } else {
        // `state` 复制一份，避免同时借用 `quad.ctrl`（可变）与 `quad.state`（不可变）。
        let state = quad.state;
        // 期望姿态：当前航向下叠加倾角；偏航是机体 z 的角速度前馈，不写进姿态。
        let tilt = cfg.control.position.tilt_max_deg.to_radians();
        let q_sp = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw_of(state.attitude))
            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), input.pitch * tilt)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), input.roll * tilt);
        let yaw_rate = -input.yaw * cfg.manual.yaw_rate_max;

        // 升降速度环 + 倾斜补偿：推力沿机体 z，倾斜时须补 1/cosθ 才维持高度。
        let up_z = (state.attitude * Vector3::z()).z.clamp(0.3, 1.0);
        let vz_des = input.throttle * cfg.manual.climb_rate_max;
        let a_vert = G + cfg.manual.vz_kp * (vz_des - state.velocity.z);
        let thrust_norm = (cfg.drone.mass * a_vert / up_z).max(0.0) / cfg.airframe.max_thrust();

        let motors = quad.ctrl.attitude_mode(
            &state,
            q_sp,
            thrust_norm,
            yaw_rate,
            &cfg.drone,
            &cfg.airframe,
            dt,
        );
        let realized = cfg.airframe.realize(state.attitude, &motors);
        quad.motors = motors;
        quad.saturated = quad.ctrl.saturated;
        integrate(&mut quad.state, &realized, &cfg.drone, dt);
    }

    transform.translation = Vec3::from_array(quad.state.position.into());
    transform.rotation = Quat::from_array(quad.state.attitude.coords.into());
}
