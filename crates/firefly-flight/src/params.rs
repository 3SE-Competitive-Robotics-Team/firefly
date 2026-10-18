//! 机体与飞控参数（`serde` 可反序列化，缺键回落默认值）。

use serde::Deserialize;

/// 机体参数（刚体本身；旋翼布局与推力上限见 [`crate::Airframe`]）。
///
/// 默认值按 219g 级 2.5~3 寸微型机取（对角 ~127mm）。质量/惯量与 sim 侧 `MuJoCo`
/// 几何同源（见 `packages/firefly-mujoco`；sim 从 `mjModel` 读取并在运行时发布）。
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

/// PX4 多旋翼控制核心的参数（与 `px4` 模块的类一一对应）。
///
/// 默认值取自 PX4-Autopilot `4dbd2e0` 的 `MC_*`/`MPC_*` 参数；产品包线
/// （219g、32° 倾角、3m/s 水平速度）的差异在下方注释中标出。
/// 归一化约定与 PX4 一致：力矩 `[-1,1]`、推力 `[0,1]`、执行器 `[0,1]`。
#[derive(Deserialize, Clone, Copy, Debug, Default)]
#[serde(default)]
pub struct Px4Params {
    /// 角速度环。
    pub rate: RateParams,
    /// 姿态环。
    pub attitude: AttitudeParams,
    /// 位置/速度环。
    pub position: PositionParams,
    /// 控制分配。
    pub allocation: AllocationParams,
    /// 悬停推力（归一化，`[0.1,0.9]`）；`None` 时由机体质量与推力上限计算。
    pub hover_thrust: Option<f32>,
}

/// 角速度环参数（PX4 `MC_*RATE_*`）。
#[derive(Deserialize, Clone, Copy, Debug)]
#[serde(default)]
pub struct RateParams {
    /// 全局增益 K：P/I/D 的公共缩放。
    pub k: [f32; 3],
    /// 角速度误差 → 归一化力矩（1/(rad/s)）。
    pub p: [f32; 3],
    /// 角速度误差积分 → 归一化力矩（1/rad）。
    pub i: [f32; 3],
    /// 角加速度 → 归一化力矩（s/rad）。
    pub d: [f32; 3],
    /// 角速度设定值前馈（s）。
    pub ff: [f32; 3],
    /// 积分限幅（归一化力矩）。
    pub int_lim: [f32; 3],
    /// 角速度设定值上限（deg/s）。
    pub max_deg_s: [f32; 3],
}

impl Default for RateParams {
    fn default() -> Self {
        Self {
            k: [1.0; 3],
            p: [0.15, 0.15, 0.2],
            i: [0.2, 0.2, 0.1],
            d: [0.003, 0.003, 0.0],
            ff: [0.0; 3],
            int_lim: [0.3; 3],
            max_deg_s: [220.0, 220.0, 200.0],
        }
    }
}

/// 姿态环参数（PX4 `MC_ROLL_P` / `MC_PITCH_P` / `MC_YAW_P` 等）。
#[derive(Deserialize, Clone, Copy, Debug)]
#[serde(default)]
pub struct AttitudeParams {
    /// 姿态误差 → 角速度设定值（1/s）。
    pub p: [f32; 3],
    /// 偏航权重 `[0,1]`：在滚转/俯仰之前降低偏航优先级。
    pub yaw_weight: f32,
    /// 二阶参考模型自然频率（rad/s）。
    pub ref_model_freq: f32,
    /// 参考模型前馈缩放 `[0,1]`。
    pub ff_gain: f32,
    /// 参考模型前馈角速度限幅（rad/s）；0 = 不限。
    pub ff_limit: f32,
}

impl Default for AttitudeParams {
    fn default() -> Self {
        Self {
            p: [4.0, 4.0, 2.8],
            yaw_weight: 0.4,
            ref_model_freq: 50.0,
            ff_gain: 1.0,
            ff_limit: 0.0,
        }
    }
}

/// 位置/速度环参数（PX4 `MPC_*`）。
#[derive(Deserialize, Clone, Copy, Debug)]
#[serde(default)]
pub struct PositionParams {
    /// 位置误差 → 速度设定值（1/s）。
    pub p: [f32; 3],
    /// 速度误差 → 加速度（1/s）。
    pub vel_p: [f32; 3],
    /// 速度误差积分 → 加速度（1/s²）。
    pub vel_i: [f32; 3],
    /// 速度导数 → 加速度（s）。
    pub vel_d: [f32; 3],
    /// 水平速度上限（m/s）。
    pub vel_max_horizontal: f32,
    /// 上升速度上限（m/s）。
    pub vel_max_up: f32,
    /// 下降速度上限（m/s）。
    pub vel_max_down: f32,
    /// 最大倾角（deg）。
    pub tilt_max_deg: f32,
    /// 归一化集合推力下限。
    pub thrust_min: f32,
    /// 归一化集合推力上限。
    pub thrust_max: f32,
    /// 垂直推力饱和时为水平控制保留的推力余量（归一化）。
    pub horizontal_thrust_margin: f32,
}

impl Default for PositionParams {
    fn default() -> Self {
        Self {
            p: [0.95, 0.95, 1.0],
            vel_p: [1.8, 1.8, 4.0],
            vel_i: [0.4, 0.4, 2.0],
            vel_d: [0.2, 0.2, 0.0],
            // PX4 默认 12 m/s；产品任务包线 1.5 m/s，留一倍余量。
            vel_max_horizontal: 3.0,
            vel_max_up: 3.0,
            vel_max_down: 1.5,
            // PX4 默认 45°；产品机体限制 32°。
            tilt_max_deg: 32.0,
            thrust_min: 0.12,
            thrust_max: 1.0,
            horizontal_thrust_margin: 0.3,
        }
    }
}

/// 控制分配参数。
#[derive(Deserialize, Clone, Copy, Debug)]
#[serde(default)]
pub struct AllocationParams {
    /// 0 关闭；1 滚转/俯仰 airmode；2 滚转/俯仰/偏航 airmode。
    ///
    /// 产品默认 1：垂直推力饱和时优先保住滚转/俯仰权限，避免大位置误差下
    /// 集合推力顶满、姿态权限归零。
    pub airmode: u8,
}

impl Default for AllocationParams {
    fn default() -> Self {
        Self { airmode: 1 }
    }
}
