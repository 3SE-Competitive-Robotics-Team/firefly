//! 四旋翼飞行力学 + 飞控（唯一实现）：机体模型、旋翼分配、姿态内环、角度/位置两种控制模式。
//!
//! 机体轴 `+X` 前、`+Y` 左、`+Z` 上；姿态为机体→世界四元数，角速度存机体系。
//! 推力只能沿机体 `+Z`（4 个旋翼共同作用）：水平加速必须靠倾斜（姿态↔平移耦合）、
//! 滚转/俯仰力矩来自推力差、偏航力矩来自旋翼反扭矩——这正是四旋翼与
//! "世界系全驱动扳手"模型的根本区别，也是本 crate 存在的理由：闭环评估（sim 侧）、
//! 飞控进程（`apps/fc`）与飞行 demo（`apps/quad`）必须用同一套模型。
//!
//! 数据流（每一环都有唯一定义）：
//! ```text
//! 参考 → angle_mode/position_mode → 期望 Wrench（不限幅）
//!      → Airframe::allocate → 4 电机推力（逐电机限幅，唯一的饱和点）
//!      → Airframe::realize → 实际 Wrench（世界系）
//!      → integrate（自积分）或 MJCF 的 site gear（被控对象侧同构实现）
//! ```
//!
//! 参数分两层：[`QuadParams`] 是刚体（质量/惯量/气动阻尼，由被控对象发布），
//! [`Airframe`] 是执行器（旋翼几何/旋向/单电机推力上限/反扭矩系数）。两种消费方式
//! （[`integrate`] 自积分、`MuJoCo` 外部物理）共用同一套参数，`serde` 可反序列化、
//! 缺键回落默认值。
//!
//! 控制模式：
//! - [`angle_mode`]：角度模式（飞行手柄）——期望俯仰/横滚/偏航角速度 + 升降速度；
//! - [`position_mode`]：位置模式（飞控外环）——位置/速度/偏航参考 → 期望推力矢量
//!   与期望姿态 → 姿态内环。

mod airframe;
mod control;
mod estimator;
mod params;
mod state;

pub use airframe::{Airframe, Allocation, Rotor};
pub use control::{AngleCommand, PositionSetpoint, angle_mode, position_mode};
pub use estimator::AttitudeEstimator;
pub use params::{ControlParams, QuadParams};
pub use state::{QuadState, Wrench, integrate, yaw_of};

/// 重力加速度（m/s²）。
pub const G: f32 = 9.81;
