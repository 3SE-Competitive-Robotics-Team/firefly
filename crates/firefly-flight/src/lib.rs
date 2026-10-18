//! 四旋翼飞行力学 + 飞控（唯一实现）：机体模型、旋翼分配、姿态内环、角度/位置两种控制模式。
//!
//! 机体轴 `+X` 前、`+Y` 左、`+Z` 上；姿态为机体→世界四元数，角速度存机体系。
//! 推力只能沿机体 `+Z`（4 个旋翼共同作用）：水平加速必须靠倾斜（姿态↔平移耦合）、
//! 滚转/俯仰力矩来自推力差、偏航力矩来自旋翼反扭矩——这正是四旋翼与
//! "世界系全驱动扳手"模型的根本区别，也是本 crate 存在的理由：闭环评估（sim 侧）、
//! 飞控进程（`apps/fc`）与第三人称飞行游戏（`apps/quad`）必须用同一套模型与 PX4 控制链。
//!
//! 数据流（每一环都有唯一定义）：
//! ```text
//! 参考 → Controller（PX4：PositionControl → AttitudeControl → RateControl → 控制分配）
//!      → 4 电机推力（归一化执行器，逐电机限幅）
//!      → Airframe::realize → 实际 Wrench（世界系）
//!      → integrate（自积分）或 MJCF 的 site gear（被控对象侧同构实现）
//! ```
//!
//! 控制核心是 PX4-Autopilot `4dbd2e0` 的 1:1 移植（[`Controller`]，参考源码在
//! `~/Projects/PX4-Autopilot`，BSD-3，不纳入本仓库）；模块内部保持 PX4 的 NED/FRD 与归一化指令，
//! 产品 Z-up/FLU 的转换只发生在 [`Controller`] 边界。
//!
//! 参数分两层：[`QuadParams`] 是刚体（质量/惯量/气动阻尼，由被控对象发布），
//! [`Airframe`] 是执行器（旋翼几何/旋向/单电机推力上限/反扭矩系数）。两种消费方式
//! （[`integrate`] 自积分、`MuJoCo` 外部物理）共用同一套参数，`serde` 可反序列化、
//! 缺键回落默认值。
//!
//! 手动飞行（`apps/quad` 第三人称游戏）走 [`Controller::attitude_mode`]：
//! 手柄 → 期望姿态/归一化推力 → 同一套 PX4 姿态/角速度/分配链。
//!
//! [`FlightFsm`] 是飞控的模式与安全层：解锁、起飞、位置保持/跟踪、降落、失效保护，
//! 输出每 tick 该飞的参考与电机使能——它不产生推力，推力仍走上面的控制链。

mod airframe;
mod control;
mod fsm;
mod params;
pub mod px4;
mod safety;
mod state;

pub use airframe::{Airframe, Rotor};
pub use control::PositionSetpoint;
pub use fsm::{Command, Event, FlightFsm, FlightState, FsmParams, Health, Output, Reject};
pub use params::{Px4Params, QuadParams};
pub use px4::controller::Controller;
pub use safety::{ArmingCheck, FailsafeAction, FailsafeFlags};
pub use state::{QuadState, Wrench, integrate, yaw_of};

/// 重力加速度（m/s²）。
pub const G: f32 = 9.81;
