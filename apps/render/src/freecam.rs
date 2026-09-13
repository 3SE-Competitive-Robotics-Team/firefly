//! 自由浏览相机：`F` 在「跟随无人机」与「自由飞行」间切换。
//!
//! 自由模式：`WASD` 相对视角平移、鼠标控制偏航/俯仰、`Q/E` 升降、`Shift` 加速，
//! `F` 或 `Esc` 退出并释放光标。只作用于主视角相机（`With<Camera>, Without<Eye>`），
//! 传感器 rig 相机不动；退出后自动回到跟随（`rig::follow_main` 在自由模式下让位）。

use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, CursorOptions, PrimaryWindow};

use crate::rig::Eye;
use crate::ui::{FOLLOW_HINT, FREE_HINT, ModeLabel};

/// 俯仰限幅（约 ±86°，防翻转）。
const PITCH_LIMIT: f32 = 1.5;
/// 加速倍率（按住 `Shift`）。
const BOOST: f32 = 4.0;

/// 自由浏览状态。
#[derive(Resource)]
pub struct FreeCam {
    /// 是否处于自由模式。
    pub enabled: bool,
    /// 鼠标灵敏度（弧度/像素）。
    sensitivity: f32,
    /// 平移速度（米/秒）。
    speed: f32,
    /// 偏航（弧度）。
    yaw: f32,
    /// 俯仰（弧度，已限幅）。
    pitch: f32,
}

impl Default for FreeCam {
    fn default() -> Self {
        Self {
            enabled: false,
            sensitivity: 0.0025,
            speed: 12.0,
            yaw: 0.0,
            pitch: 0.0,
        }
    }
}

/// `F` 切换、`Esc` 退出；进入时锁定并隐藏光标，退出时释放。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
pub fn toggle_freecam(
    keys: Res<ButtonInput<KeyCode>>,
    mut free: ResMut<FreeCam>,
    mut cursor: Single<&mut CursorOptions, With<PrimaryWindow>>,
    cam: Single<&Transform, (With<Camera>, Without<Eye>)>,
) {
    let toggled = keys.just_pressed(KeyCode::KeyF);
    let escaped = keys.just_pressed(KeyCode::Escape);
    if toggled {
        free.enabled = !free.enabled;
    } else if escaped && free.enabled {
        free.enabled = false;
    } else {
        return;
    }

    if free.enabled {
        // 从当前朝向接手，避免进入时跳变。
        let (yaw, pitch, _) = cam.rotation.to_euler(EulerRot::YXZ);
        free.yaw = yaw;
        free.pitch = pitch.clamp(-PITCH_LIMIT, PITCH_LIMIT);
        cursor.grab_mode = CursorGrabMode::Locked;
        cursor.visible = false;
        log::info!("自由浏览：WASD 平移 / 鼠标视角 / QE 升降 / Shift 加速 / F 或 Esc 退出");
    } else {
        cursor.grab_mode = CursorGrabMode::None;
        cursor.visible = true;
        log::info!("跟随无人机（F 进入自由浏览）");
    }
}

/// 自由模式下的相机更新（鼠标视角 + WASD 平移）。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
pub fn freecam_move(
    time: Res<Time>,
    mouse: Res<AccumulatedMouseMotion>,
    keys: Res<ButtonInput<KeyCode>>,
    mut free: ResMut<FreeCam>,
    mut cam: Single<&mut Transform, (With<Camera>, Without<Eye>)>,
) {
    if !free.enabled {
        return;
    }

    free.yaw -= mouse.delta.x * free.sensitivity;
    free.pitch = (free.pitch - mouse.delta.y * free.sensitivity).clamp(-PITCH_LIMIT, PITCH_LIMIT);
    cam.rotation = Quat::from_euler(EulerRot::YXZ, free.yaw, free.pitch, 0.0);

    let mut dir = Vec3::ZERO;
    if keys.pressed(KeyCode::KeyW) {
        dir += cam.rotation * Vec3::NEG_Z;
    }
    if keys.pressed(KeyCode::KeyS) {
        dir += cam.rotation * Vec3::Z;
    }
    if keys.pressed(KeyCode::KeyD) {
        dir += cam.rotation * Vec3::X;
    }
    if keys.pressed(KeyCode::KeyA) {
        dir -= cam.rotation * Vec3::X;
    }
    if keys.pressed(KeyCode::KeyE) {
        dir += Vec3::Z;
    }
    if keys.pressed(KeyCode::KeyQ) {
        dir -= Vec3::Z;
    }
    if dir != Vec3::ZERO {
        let boost = if keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]) {
            BOOST
        } else {
            1.0
        };
        cam.translation += dir.normalize() * free.speed * boost * time.delta_secs();
    }
}

/// 左上角模式提示（仅模式变化时写，避免每帧触发 UI 变更检测）。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
pub fn update_mode_label(free: Res<FreeCam>, mut label: Single<&mut Text, With<ModeLabel>>) {
    let want = if free.enabled { FREE_HINT } else { FOLLOW_HINT };
    if label.as_str() != want {
        label.0.clear();
        label.0.push_str(want);
    }
}
