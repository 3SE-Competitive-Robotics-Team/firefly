//! 机体（airframe）：4 旋翼几何/旋向/推力上限与正模型（电机 → 力/力矩）。
//!
//! 四旋翼唯一力源是 4 个沿机体 `+Z` 的旋翼推力：水平力只能靠倾斜，滚转/俯仰力矩
//! 来自推力差，偏航力矩来自旋翼反扭矩。[`Airframe::realize`] 把 4 电机推力还原为
//! 机体的世界系力/力矩，供 `integrate` 或 `MuJoCo` 被控对象使用。
//!
//! 控制分配不在这里：产品链路使用 PX4 的
//! [`ControlAllocationSequentialDesaturation`](crate::px4::allocation::ControlAllocationSequentialDesaturation)，
//! 效能矩阵由 [`crate::px4::allocation::effectiveness_from_rotors`] 从同一份旋翼几何构造。

use nalgebra::{UnitQuaternion, Vector3};
use serde::Deserialize;

use crate::state::Wrench;

/// 单个旋翼：机体系安装位置 + 旋向。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct Rotor {
    /// 机体系安装位置（m）。
    #[serde(default)]
    pub position: [f32; 3],
    /// 旋向：`+1` 表示该旋翼对机体的反扭矩指向机体 `+Z`。
    #[serde(default = "d_spin")]
    pub spin: f32,
}

fn d_spin() -> f32 {
    1.0
}

/// 机体：旋翼布局 + 执行器上限（不含量/惯量，那属 [`QuadParams`](crate::QuadParams)）。
///
/// 默认值按 219g 级 2.5~3 寸微型机取（轴距 ~127mm 对角、X 布局、推重比 ≈ 3）。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct Airframe {
    /// 4 个旋翼的安装位置与旋向。
    #[serde(default = "d_rotors")]
    pub rotors: [Rotor; 4],
    /// 单电机推力上限（N）。
    #[serde(default = "d_max_thrust_per_motor")]
    pub max_thrust_per_motor: f32,
    /// 反扭矩系数 `c_τ`（m）：偏航力矩 = `c_τ · 推力`，小型桨典型 0.01~0.02 m。
    #[serde(default = "d_torque_coefficient")]
    pub torque_coefficient: f32,
}

/// 默认 X 布局半轴距（m）：旋翼位于 `(±ARM, ±ARM, 0)`，对角 127mm。
const ARM: f32 = 0.045;

fn d_rotors() -> [Rotor; 4] {
    [
        Rotor {
            position: [ARM, -ARM, 0.0],
            spin: 1.0,
        },
        Rotor {
            position: [-ARM, ARM, 0.0],
            spin: 1.0,
        },
        Rotor {
            position: [ARM, ARM, 0.0],
            spin: -1.0,
        },
        Rotor {
            position: [-ARM, -ARM, 0.0],
            spin: -1.0,
        },
    ]
}

fn d_max_thrust_per_motor() -> f32 {
    1.61
}

fn d_torque_coefficient() -> f32 {
    0.016
}

impl Default for Airframe {
    fn default() -> Self {
        Self {
            rotors: d_rotors(),
            max_thrust_per_motor: d_max_thrust_per_motor(),
            torque_coefficient: d_torque_coefficient(),
        }
    }
}

impl Airframe {
    /// 4 电机总推力上限（N）。
    #[must_use]
    pub fn max_thrust(&self) -> f32 {
        4.0 * self.max_thrust_per_motor
    }

    /// 4 电机推力 → 机体产生的世界系力/力矩（不含气动阻尼与重力）。
    ///
    /// 刚体等效：力 `R·Σ T_i ẑ`，力矩 `R·(Σ r_i × T_i ẑ + Σ s_i·c_τ·T_i ẑ)`。
    /// 输入电机推力单位 N，顺序与 [`Airframe::rotors`] 一致。
    #[must_use]
    pub fn realize(&self, attitude: UnitQuaternion<f32>, motors: &[f32; 4]) -> Wrench {
        let c = self.torque_coefficient;
        let mut total = 0.0f32;
        let mut torque_body = Vector3::zeros();
        for (rotor, t) in self.rotors.iter().zip(motors) {
            let r = Vector3::from(rotor.position);
            total += *t;
            torque_body += r.cross(&(Vector3::z() * *t)) + Vector3::z() * (rotor.spin * c * *t);
        }
        Wrench {
            force: attitude * (Vector3::z() * total),
            torque: attitude * torque_body,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn realize_geometry_signs() {
        let air = Airframe::default();
        let w = air.realize(UnitQuaternion::identity(), &[0.1, 0.0, 0.0, 0.0]);
        assert!(w.torque.x < 0.0, "前右加推力应产生负滚转：{:?}", w.torque);
        assert!(w.torque.y < 0.0, "前右加推力应产生负俯仰：{:?}", w.torque);
        assert!((w.force.z - 0.1).abs() < 1e-6);
        assert!((w.torque.z - 0.1 * air.torque_coefficient).abs() < 1e-6);
    }

    /// 悬停：4 电机均分 `mg`，合力竖直、无力矩。
    #[test]
    fn realize_hover_is_symmetric() {
        let air = Airframe::default();
        let per_motor = 0.219 * crate::G / 4.0;
        let w = air.realize(UnitQuaternion::identity(), &[per_motor; 4]);
        assert!((w.force.z - 0.219 * crate::G).abs() < 1e-5);
        assert!(w.force.xy().norm() < 1e-6);
        assert!(w.torque.norm() < 1e-6);
    }
}
