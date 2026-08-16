//! `VIO` 基础类型（对照 `OpenVINS` `ov_core/src/types`）。
//!
//! 本 crate 只含纯数学与数据结构，不涉及 IO 与估计逻辑：
//! - `quat_ops`：JPL 四元数 / SO(3) / SE(3) 运算集；
//! - （规划中）IMU 测量、变量类型（Vec/Pose/Landmark）。

pub mod quat_ops;
pub mod var;
