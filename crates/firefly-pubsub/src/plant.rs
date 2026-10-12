//! 被控对象消息：状态（`Firefly/PlantState`，200Hz）与机体/执行器描述
//! （`Firefly/Airframe`，1Hz 电平）。
//!
//! 这两条话题是**被控对象属性**的唯一来源：质量/惯量/气动阻尼来自物理模型，
//! 旋翼位置/旋向/单电机推力上限/反扭矩系数来自机体布局——飞控订阅后装配
//! [`firefly-flight`] 的参数，不另抄一份（改几何即改被控对象，无第二处同步）。
//! `#[repr(C)]` 定长结构，满足 `ZeroCopySend` 约束；trace 上下文在 User Header。

use iceoryx2::prelude::*;

/// 被控对象状态话题（sim → 飞控）。
pub const PLANT_STATE_TOPIC: &str = "Firefly/PlantState";
/// 机体/执行器描述话题（sim → 飞控，持续重发，晚订阅必收）。
pub const AIRFRAME_TOPIC: &str = "Firefly/Airframe";

/// 状态订阅缓冲区深度与服务容量（Python 侧同值）：40ms（飞控 1kHz 排空，覆盖短促停顿后仍拿最新样本）。
pub const PLANT_STATE_BUFFER_SIZE: usize = 8;
/// 机体描述订阅缓冲深度与服务容量（1Hz 电平，Python 侧同值）。
pub const AIRFRAME_BUFFER_SIZE: usize = 4;

/// 被控对象状态（真值）：位置/速度世界系，姿态为**机体→世界** Hamilton
/// 四元数 `[x,y,z,w]`（与 [`firefly_flight::QuadState`] 同约定），角速度机体系。
///
/// ⚠️ 与 [`crate::odom::OdomMessage`] 的 JPL `q_GtoI`（估计姿态）**不同约定**：
/// 本消息仅供评测与可视化，禁止用于控制、初始化或健康判据。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyPlantStateMessage")]
pub struct PlantStateMessage {
    /// 仿真时刻（秒）。
    pub timestamp: f64,
    /// 位置（世界系，米）。
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    /// 速度（世界系，米/秒）。
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub velocity_z: f64,
    /// 姿态（机体→世界 Hamilton 四元数，`[x,y,z,w]`）。
    pub quat_x: f64,
    pub quat_y: f64,
    pub quat_z: f64,
    pub quat_w: f64,
    /// 角速度（机体系，rad/s）。
    pub angular_velocity_x: f64,
    pub angular_velocity_y: f64,
    pub angular_velocity_z: f64,
}

impl Default for PlantStateMessage {
    fn default() -> Self {
        Self {
            timestamp: -1.0,
            position_x: 0.0,
            position_y: 0.0,
            position_z: 0.0,
            velocity_x: 0.0,
            velocity_y: 0.0,
            velocity_z: 0.0,
            quat_x: 0.0,
            quat_y: 0.0,
            quat_z: 0.0,
            quat_w: 1.0,
            angular_velocity_x: 0.0,
            angular_velocity_y: 0.0,
            angular_velocity_z: 0.0,
        }
    }
}

/// 机体/执行器描述：刚体属性（质量/惯量/阻尼）+ 执行器属性（旋翼几何/旋向/
/// 单电机推力上限/反扭矩系数）。
///
/// 惯量是**机体系**对角元（物理模型若以主轴系给出，发布端须先转回机体系）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyAirframeMessage")]
pub struct AirframeMessage {
    /// 仿真时刻（秒）。
    pub timestamp: f64,
    /// 质量（kg）。
    pub mass: f64,
    /// 转动惯量（机体系对角元，kg·m²）。
    pub inertia_x: f64,
    pub inertia_y: f64,
    pub inertia_z: f64,
    /// 平移阻尼（N·s/m，无气动模型时为 0）。
    pub linear_drag: f64,
    /// 角速度阻尼率（1/s，无气动模型时为 0）。
    pub angular_drag: f64,
    /// 4 旋翼的机体系安装位置（m）——分配与合成共用的唯一几何。
    pub rotor_positions: [[f64; 3]; 4],
    /// 4 旋翼旋向（`+1` = 反扭矩指向机体 `+Z`）。
    pub rotor_spins: [f64; 4],
    /// 单电机推力上限（N）。
    pub max_thrust_per_motor: f64,
    /// 反扭矩系数 `c_τ`（m）：偏航力矩 = `c_τ · 推力`。
    pub torque_coefficient: f64,
}

impl Default for AirframeMessage {
    fn default() -> Self {
        Self {
            timestamp: -1.0,
            mass: 0.0,
            inertia_x: 0.0,
            inertia_y: 0.0,
            inertia_z: 0.0,
            linear_drag: 0.0,
            angular_drag: 0.0,
            rotor_positions: [[0.0; 3]; 4],
            rotor_spins: [1.0; 4],
            max_thrust_per_motor: 0.0,
            torque_coefficient: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plant_state_message_is_plain_old_data() {
        let m = PlantStateMessage::default();
        assert_eq!(std::mem::size_of::<PlantStateMessage>(), 112);
        assert!((m.quat_w - 1.0).abs() < 1e-12);
        assert!((m.timestamp + 1.0).abs() < 1e-9);
    }

    #[test]
    fn airframe_message_is_plain_old_data() {
        let m = AirframeMessage::default();
        // 7 × f64（timestamp + 质量 + 惯量 3 + 阻尼 2）+ 12 × f64（旋翼位置）
        // + 4 × f64（旋向）+ 2 × f64（推力上限 + 反扭矩系数）= 25 × 8
        assert_eq!(std::mem::size_of::<AirframeMessage>(), 200);
        assert!(m.rotor_spins.iter().all(|s| (*s - 1.0).abs() < 1e-12));
    }
}
