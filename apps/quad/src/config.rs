//! `configs/quad.toml`：微无人机 demo 的全部可调参数（缺失键回落默认值）。

use bevy::prelude::*;
use serde::Deserialize;

/// 顶层配置。
#[derive(Resource, Deserialize, Clone, Debug)]
pub struct QuadConfig {
    /// 场地 glb（相对 `models/`）。
    pub field: String,
    /// 初始位置（米，世界系）。
    pub start: [f32; 3],
    #[serde(default)]
    pub drone: DroneConfig,
    #[serde(default)]
    pub control: ControlConfig,
    #[serde(default)]
    pub camera: CameraConfig,
}

/// 机体（250g 级圈圈机量级）。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct DroneConfig {
    /// 质量（kg）。
    #[serde(default = "d_mass")]
    pub mass: f32,
    /// 半轴距（米）。
    #[serde(default = "d_arm")]
    pub arm: f32,
    /// 四电机总推力上限（N）。
    #[serde(default = "d_max_thrust")]
    pub max_thrust: f32,
    /// 平移阻尼（N·s/m）。
    #[serde(default = "d_linear_drag")]
    pub linear_drag: f32,
    /// 角速度阻尼（N·m·s/rad）。
    #[serde(default = "d_angular_drag")]
    pub angular_drag: f32,
}

fn d_mass() -> f32 {
    0.25
}
fn d_arm() -> f32 {
    0.06
}
fn d_max_thrust() -> f32 {
    7.5
}
fn d_linear_drag() -> f32 {
    0.20
}
fn d_angular_drag() -> f32 {
    0.02
}

impl Default for DroneConfig {
    fn default() -> Self {
        Self {
            mass: d_mass(),
            arm: d_arm(),
            max_thrust: d_max_thrust(),
            linear_drag: d_linear_drag(),
            angular_drag: d_angular_drag(),
        }
    }
}

/// 控制（角度模式）。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct ControlConfig {
    #[serde(default = "d_tilt")]
    pub tilt_max_deg: f32,
    #[serde(default = "d_yaw_rate")]
    pub yaw_rate_max: f32,
    #[serde(default = "d_climb")]
    pub climb_rate_max: f32,
    #[serde(default = "d_att_kp")]
    pub attitude_kp: f32,
    #[serde(default = "d_rate_kp")]
    pub rate_kp: f32,
    #[serde(default = "d_vz_kp")]
    pub vz_kp: f32,
}

fn d_tilt() -> f32 {
    32.0
}
fn d_yaw_rate() -> f32 {
    3.0
}
fn d_climb() -> f32 {
    3.0
}
fn d_att_kp() -> f32 {
    9.0
}
fn d_rate_kp() -> f32 {
    6.0
}
fn d_vz_kp() -> f32 {
    4.0
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            tilt_max_deg: d_tilt(),
            yaw_rate_max: d_yaw_rate(),
            climb_rate_max: d_climb(),
            attitude_kp: d_att_kp(),
            rate_kp: d_rate_kp(),
            vz_kp: d_vz_kp(),
        }
    }
}

/// 第三人称追踪相机。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct CameraConfig {
    #[serde(default = "d_back")]
    pub back: f32,
    #[serde(default = "d_up")]
    pub up: f32,
    #[serde(default = "d_look")]
    pub look_ahead: f32,
    #[serde(default = "d_smooth")]
    pub smooth: f32,
}

fn d_back() -> f32 {
    1.1
}
fn d_up() -> f32 {
    0.45
}
fn d_look() -> f32 {
    0.8
}
fn d_smooth() -> f32 {
    6.0
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            back: d_back(),
            up: d_up(),
            look_ahead: d_look(),
            smooth: d_smooth(),
        }
    }
}

/// 读配置；缺文件/解析失败即报错退出。
#[must_use]
pub fn load(path: &str) -> QuadConfig {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| {
        log::error!("读取 {path} 失败：{e}");
        std::process::exit(1);
    });
    toml::from_str(&text).unwrap_or_else(|e| {
        log::error!("解析 {path} 失败：{e}");
        std::process::exit(1);
    })
}
