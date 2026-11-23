//! 机体 FLU（`+X` 前、`+Y` 左、`+Z` 上），姿态为机体→世界，角速度在机体系。
//! [`Controller`] 在入口转换到 NED 世界系和 FRD 机体系，控制量使用归一化单位。
//! 刚体参数见 [`QuadParams`]，旋翼几何与推力上限见 [`Airframe`]。

mod airframe;
mod control;
mod fsm;
#[cfg(feature = "ipc")]
pub mod ipc;
pub mod multirotor;
mod params;
pub mod precision;
mod safety;
mod state;
pub mod sticks;

pub use airframe::{Airframe, Rotor};
pub use control::PositionSetpoint;
pub use fsm::{Command, Event, FlightFsm, FlightState, FsmParams, Health, Output, Reject};
pub use multirotor::controller::Controller;
pub use params::{ControlParams, QuadParams};
pub use safety::{ArmingCheck, FailsafeAction, FailsafeFlags};
pub use state::{QuadState, Wrench, integrate, yaw_of};

/// 重力加速度（m/s²）。
pub const G: f32 = 9.81;
