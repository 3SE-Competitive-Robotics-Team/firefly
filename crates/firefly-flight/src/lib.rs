//! 四旋翼飞行力学 + 飞控（唯一实现）：机体模型、姿态内环、角度/位置两种控制模式。
//!
//! 机体轴 `+X` 前、`+Y` 左、`+Z` 上；姿态为机体→世界四元数，角速度存机体系。
//! **推力只能沿机体 `+Z`**——水平加速必须靠倾斜（姿态↔平移耦合），这正是四旋翼与
//! "世界系全驱动扳手"模型的根本区别，也是本 crate 存在的理由：闭环评估（sim 侧）
//! 与飞行 demo（`apps/quad`）必须用同一套模型，否则两边动态不同源。
//!
//! 接口约定：控制输出为[`Wrench`]（**世界系**力 N + 力矩 N·m），可直接写进 `MuJoCo`
//! 的 `xfrc_applied`，也可交 [`integrate`] 自己积分；两种消费方式共用同一套参数
//!（[`QuadParams`]/[`ControlParams`]，`serde` 可反序列化、缺键回落默认值）。
//!
//! 控制模式：
//! - [`angle_mode`]：角度模式（飞行手柄）——期望俯仰/横滚/偏航角速度 + 升降速度；
//! - [`position_mode`]：位置模式（飞控外环）——位置/速度/偏航参考 → 期望推力矢量
//!   与期望姿态 → 姿态内环，含倾角与推力限幅。

mod control;
mod params;
mod state;

pub use control::{AngleCommand, PositionSetpoint, angle_mode, position_mode};
pub use params::{ControlParams, QuadParams};
pub use state::{QuadState, Wrench, integrate};

/// 重力加速度（m/s²）。
pub const G: f32 = 9.81;
