//! 机体（airframe）：4 旋翼几何/旋向/推力上限 + 分配（mixer）与合成（正模型）。
//!
//! 四旋翼唯一力源是 4 个沿机体 `+Z` 的旋翼推力：水平力只能靠倾斜，滚转/俯仰力矩
//! 来自推力差，偏航力矩来自旋翼反扭矩。控制器给出**期望**世界系力/力矩，
//! [`Airframe::allocate`] 分配到 4 个电机并逐电机限幅，[`Airframe::realize`]
//! 把电机推力还原为机体产生的世界系力/力矩——两者共用同一份几何，互为逆运算
//! （单测断言往返一致）。限幅是**唯一**的饱和点：控制律不限幅，真实电机在分配处饱和。

use glam::{Quat, Vec3};
use serde::Deserialize;

use crate::state::{QuadState, Wrench};

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

/// 一次分配的落点：电机推力 + 限幅后机体实际产生的世界系力/力矩 + 是否触限。
#[derive(Clone, Copy, Debug, Default)]
pub struct Allocation {
    /// 各电机推力（N，已限幅到 `[0, max_thrust_per_motor]`）。
    pub motors: [f32; 4],
    /// 限幅后机体实际产生的世界系力/力矩（不含气动阻尼与重力）。
    pub realized: Wrench,
    /// 是否有机电触限（期望推力被削）。
    pub saturated: bool,
}

impl Airframe {
    /// 4 电机总推力上限（N）。
    #[must_use]
    pub fn max_thrust(&self) -> f32 {
        4.0 * self.max_thrust_per_motor
    }

    /// 期望世界系力/力矩 → 4 电机推力（N，逐电机限幅）。
    ///
    /// 分配取期望力的**机体 z 分量**为集合推力（水平力由姿态倾斜产生，旋翼给不出
    /// 侧向力），期望力矩取机体三轴；解出 4 电机推力后逐电机限幅到
    /// `[0, max_thrust_per_motor]`，再回代 [`realize`](Self::realize) 得到实际落点。
    #[must_use]
    pub fn allocate(&self, attitude: Quat, desired: &Wrench) -> Allocation {
        let body_force = attitude.inverse() * desired.force;
        let body_torque = attitude.inverse() * desired.torque;
        let c = self.torque_coefficient;

        // A·T = [集合推力, τ_x, τ_y, τ_z]：行 2 为 (r×ẑ)_x = y，行 3 为 (r×ẑ)_y = −x
        let mut a = [[0.0f32; 4]; 4];
        for (i, rotor) in self.rotors.iter().enumerate() {
            let [x, y, _] = rotor.position;
            a[0][i] = 1.0;
            a[1][i] = y;
            a[2][i] = -x;
            a[3][i] = c * rotor.spin;
        }
        let rhs = [body_force.z, body_torque.x, body_torque.y, body_torque.z];
        let Some(raw) = solve4(&a, &rhs) else {
            // 退化布局（几何奇异）：给不出任何推力，如实报告饱和
            return Allocation {
                motors: [0.0; 4],
                realized: Wrench::default(),
                saturated: true,
            };
        };

        let max = self.max_thrust_per_motor;
        let mut motors = [0.0f32; 4];
        let mut saturated = false;
        for (i, t) in raw.iter().enumerate() {
            let clamped = t.clamp(0.0, max);
            saturated |= (clamped - t).abs() > 1e-6;
            motors[i] = clamped;
        }
        Allocation {
            motors,
            realized: self.realize(attitude, &motors),
            saturated,
        }
    }

    /// 4 电机推力 → 机体产生的世界系力/力矩（不含气动阻尼与重力）。
    ///
    /// 刚体等效：力 `R·Σ T_i ẑ`，力矩 `R·(Σ r_i × T_i ẑ + Σ s_i·c_τ·T_i ẑ)`。
    #[must_use]
    pub fn realize(&self, attitude: Quat, motors: &[f32; 4]) -> Wrench {
        let c = self.torque_coefficient;
        let mut total = 0.0f32;
        let mut torque_body = Vec3::ZERO;
        for (rotor, t) in self.rotors.iter().zip(motors) {
            let r = Vec3::from(rotor.position);
            total += *t;
            torque_body += r.cross(Vec3::Z * *t) + Vec3::Z * (rotor.spin * c * *t);
        }
        Wrench {
            force: attitude * (Vec3::Z * total),
            torque: attitude * torque_body,
        }
    }

    /// 一步闭环：期望力/力矩 → 分配 → 实际力/力矩（plant 与 demo 共用的入口）。
    #[must_use]
    pub fn act(&self, state: &QuadState, desired: &Wrench) -> Allocation {
        self.allocate(state.attitude, desired)
    }
}

/// 4×4 线性方程求解（列主元高斯消元，无外部依赖）；奇异返回 `None`。
fn solve4(a: &[[f32; 4]; 4], b: &[f32; 4]) -> Option<[f32; 4]> {
    let mut m = *a;
    let mut x = *b;
    for col in 0..4 {
        let pivot = (col..4).max_by(|&i, &j| {
            m[i][col]
                .abs()
                .partial_cmp(&m[j][col].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        if m[pivot][col].abs() < 1e-9 {
            return None;
        }
        m.swap(col, pivot);
        x.swap(col, pivot);
        for row in (col + 1)..4 {
            let factor = m[row][col] / m[col][col];
            let pivot_row = m[col];
            for (k, value) in m[row].iter_mut().enumerate().skip(col) {
                *value -= factor * pivot_row[k];
            }
            x[row] -= factor * x[col];
        }
    }
    let mut out = [0.0f32; 4];
    for row in (0..4).rev() {
        let mut acc = x[row];
        for k in (row + 1)..4 {
            acc -= m[row][k] * out[k];
        }
        out[row] = acc / m[row][row];
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn attitude() -> Quat {
        Quat::from_rotation_z(0.7) * Quat::from_rotation_x(0.2)
    }

    /// 分配与合成互为逆运算：期望在能力范围内时，实际落点 ≈ 期望。
    #[test]
    fn allocate_then_realize_round_trips() {
        let air = Airframe::default();
        let q = attitude();
        let desired = Wrench {
            force: q * Vec3::new(0.0, 0.0, 3.0),
            torque: q * Vec3::new(0.01, -0.015, 0.001),
        };
        let alloc = air.allocate(q, &desired);
        assert!(!alloc.saturated, "能力范围内不应触限");
        assert!(
            (alloc.realized.force - desired.force).length() < 1e-4,
            "力 {:?} vs {:?}",
            alloc.realized.force,
            desired.force
        );
        assert!(
            (alloc.realized.torque - desired.torque).length() < 1e-4,
            "力矩 {:?} vs {:?}",
            alloc.realized.torque,
            desired.torque
        );
    }

    /// 悬停分配对称：集合推力均分到 4 个电机，力矩 ≈ 0。
    #[test]
    fn hover_allocation_is_symmetric() {
        let air = Airframe::default();
        let weight = 0.219 * crate::G;
        let alloc = air.allocate(
            Quat::IDENTITY,
            &Wrench {
                force: Vec3::Z * weight,
                torque: Vec3::ZERO,
            },
        );
        assert!(!alloc.saturated);
        for t in alloc.motors {
            assert!((t - weight / 4.0).abs() < 1e-5, "电机推力不均衡：{t}");
        }
        assert!(alloc.realized.torque.length() < 1e-5);
    }

    /// 纯偏航力矩只靠反扭矩：4 电机推力均值不变，成对角差动。
    #[test]
    fn yaw_torque_uses_reaction_only() {
        let air = Airframe::default();
        let collective = 0.5;
        let alloc = air.allocate(
            Quat::IDENTITY,
            &Wrench {
                force: Vec3::Z * collective,
                torque: Vec3::Z * 0.002,
            },
        );
        let sum: f32 = alloc.motors.iter().sum();
        assert!((sum - collective).abs() < 1e-4, "集合推力被偏航改变：{sum}");
        assert!(
            (alloc.realized.torque.z - 0.002).abs() < 1e-5,
            "偏航力矩 {}",
            alloc.realized.torque.z
        );
        // 同旋向的两个对角电机推力同向变化
        assert!((alloc.motors[0] - alloc.motors[1]).abs() < 1e-6);
        assert!((alloc.motors[2] - alloc.motors[3]).abs() < 1e-6);
        assert!(alloc.motors[0] > alloc.motors[2]);
    }

    /// 电机触限：单电机被压到 `[0, T_max]`，饱和标志置位，实际力矩小于期望。
    #[test]
    fn saturation_clamps_motors() {
        let air = Airframe::default();
        let desired = Wrench {
            force: Vec3::Z * air.max_thrust() * 2.0,
            torque: Vec3::ZERO,
        };
        let alloc = air.allocate(Quat::IDENTITY, &desired);
        assert!(alloc.saturated);
        for t in alloc.motors {
            assert!(t >= 0.0 && t <= air.max_thrust_per_motor + 1e-6);
        }
        let sum: f32 = alloc.motors.iter().sum();
        assert!((sum - air.max_thrust()).abs() < 1e-4);
    }

    /// 拉力上限只由分配决定：负推力（下压）被压到 0，不给"反向拉力"。
    #[test]
    fn negative_collective_clamps_to_zero() {
        let air = Airframe::default();
        let alloc = air.allocate(
            Quat::IDENTITY,
            &Wrench {
                force: Vec3::NEG_Z * 5.0,
                torque: Vec3::ZERO,
            },
        );
        assert!(alloc.motors.iter().all(|t| t.abs() < 1e-6));
        assert!(alloc.realized.force.length() < 1e-6);
    }

    /// 合成侧几何符号：前右（+x, −y）加推力 → `(r×ẑ) = (y·T, −x·T, 0)` 给出负滚转、负俯仰。
    #[test]
    fn realize_geometry_signs() {
        let air = Airframe::default();
        let w = air.realize(Quat::IDENTITY, &[0.1, 0.0, 0.0, 0.0]);
        assert!(w.torque.x < 0.0, "前右加推力应产生负滚转：{:?}", w.torque);
        assert!(w.torque.y < 0.0, "前右加推力应产生负俯仰：{:?}", w.torque);
        assert!((w.force.z - 0.1).abs() < 1e-6);
        assert!((w.torque.z - 0.1 * air.torque_coefficient).abs() < 1e-6);
    }
}
