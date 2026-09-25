//! 机体与飞控参数（`serde` 可反序列化，缺键回落默认值）。

use serde::Deserialize;

/// 机体参数（刚体本身；旋翼布局与推力上限见 [`crate::Airframe`]）。
///
/// 默认值按 219g 级 2.5~3 寸微型机取（对角 ~127mm）。质量/惯量与 sim 侧 `MuJoCo`
/// 几何同源（见 `packages/firefly-mujoco`；sim 以 `mjModel` 实测值为准并运行时发布）。
/// `linear_drag`/`angular_drag` 是**被控对象的气动阻尼**，由 [`crate::integrate`] 施加；
/// 无气动模型的对象（如 `MuJoCo`）在 [`crate::Airframe`] 消息里发布 0。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct QuadParams {
    /// 质量（kg）。
    #[serde(default = "d_mass")]
    pub mass: f32,
    /// 惯量对角元（kg·m²，机体系 xyz）。
    #[serde(default = "d_inertia")]
    pub inertia: [f32; 3],
    /// 平移阻尼（N·s/m）：按 `-k·v` 计入世界系力。
    #[serde(default = "d_linear_drag")]
    pub linear_drag: f32,
    /// 角速度阻尼率（1/s）：按 `-I·k·ω_body` 计入力矩。
    #[serde(default = "d_angular_drag")]
    pub angular_drag: f32,
}

fn d_mass() -> f32 {
    0.219
}

fn d_inertia() -> [f32; 3] {
    [2.7e-4, 3.3e-4, 5.6e-4]
}

fn d_linear_drag() -> f32 {
    0.15
}

fn d_angular_drag() -> f32 {
    0.05
}

impl Default for QuadParams {
    fn default() -> Self {
        Self {
            mass: d_mass(),
            inertia: d_inertia(),
            linear_drag: d_linear_drag(),
            angular_drag: d_angular_drag(),
        }
    }
}

/// 飞控参数（两种模式共用；`pos_kp`/`vel_kp` 只被 [`position_mode`](crate::position_mode) 使用）。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct ControlParams {
    /// 最大俯仰/横滚（度）。
    #[serde(default = "d_tilt")]
    pub tilt_max_deg: f32,
    /// 偏航角速度指令上限（rad/s，角度模式）。
    #[serde(default = "d_yaw_rate")]
    pub yaw_rate_max: f32,
    /// 升降速度指令上限（m/s，角度模式）。
    #[serde(default = "d_climb")]
    pub climb_rate_max: f32,
    /// 姿态误差 → 机体角速度指令（1/s）。
    #[serde(default = "d_att_kp")]
    pub attitude_kp: f32,
    /// 角速度误差 → 角加速度（1/s）。
    #[serde(default = "d_rate_kp")]
    pub rate_kp: f32,
    /// 垂直速度误差 → 垂直加速度（1/s，角度模式）。
    #[serde(default = "d_vz_kp")]
    pub vz_kp: f32,
    /// 位置误差 → 加速度（1/s²，位置模式）。
    #[serde(default = "d_pos_kp")]
    pub pos_kp: f32,
    /// 速度误差 → 加速度（1/s，位置模式）。
    #[serde(default = "d_vel_kp")]
    pub vel_kp: f32,
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
    10.0
}

fn d_rate_kp() -> f32 {
    20.0
}

fn d_vz_kp() -> f32 {
    4.0
}

// ω ≈ 2.4 rad/s、ζ ≈ 1.0（位置环临界阻尼）。
fn d_pos_kp() -> f32 {
    6.0
}

fn d_vel_kp() -> f32 {
    5.0
}

impl Default for ControlParams {
    fn default() -> Self {
        Self {
            tilt_max_deg: d_tilt(),
            yaw_rate_max: d_yaw_rate(),
            climb_rate_max: d_climb(),
            attitude_kp: d_att_kp(),
            rate_kp: d_rate_kp(),
            vz_kp: d_vz_kp(),
            pos_kp: d_pos_kp(),
            vel_kp: d_vel_kp(),
        }
    }
}
