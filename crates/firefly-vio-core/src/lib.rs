//! `VIO` 核心数学与数据结构（对照 `OpenVINS` `ov_core`）。
//!
//! 与 `firefly-vio-types` 的纯类型不同，本 crate 提供传感器数据、
//! IMU 标定模型与传播/更新数学（无 IO，纯计算）。

pub mod imu_model;
pub mod sensor;
