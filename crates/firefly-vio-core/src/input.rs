//! 传感器输入源端口（合成 / 物理环境 / 真实驱动统一入口）。
//!
//! [`SensorInput`] 抽象"按时间推进产出 IMU 与相机数据"的数据源，
//! 让 VIO 编排层不关心数据来自哪里：
//! - 合成源（固定场景生成，闭环自测，见 `apps/vio` 的 `SyntheticInput`）；
//! - 物理环境源（`iceoryx2` 订阅 `MuJoCo` 发布的传感器数据，见 `apps/vio`
//!   的 `IceoryxInput`）；
//! - 未来真实驱动源（realsense/串口）。

use crate::sensor::{CameraData, ImuData};

/// 传感器输入源：按时间推进产出 IMU 与相机数据。
pub trait SensorInput {
    /// 推进一帧：合成源走一个采样周期，物理源拉取新数据。
    fn advance(&mut self);
    /// 当前传感器/仿真时刻（秒）。
    fn now(&self) -> f64;
    /// 当前帧的 IMU 样本（`None` = 本帧无）。
    fn next_imu(&mut self) -> Option<ImuData>;
    /// 当前帧的相机数据（`None` = 本帧无）。
    fn next_camera(&mut self) -> Option<CameraData>;
    /// 本帧最近收到的消息携带的 trace 上下文 `(trace_id, span_id, sampled)`。
    ///
    /// 供编排层续接跨进程 trace（无 trace 上下文时返回 `None`，编排层自建
    /// 新 trace）。默认无。
    fn last_trace(&self) -> Option<(u128, u64, bool)> {
        None
    }
}
