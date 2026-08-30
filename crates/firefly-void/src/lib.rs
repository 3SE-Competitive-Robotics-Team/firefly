//! DIVO 应用接口层：Odometry trait 与管线组装（P4 实现）。
//!
//! 技术蓝本为 FAST-LIVO2 `src/LIVMapper.cpp` 的
//! `stateEstimationAndMapping` 主循环：scan 重组合 → 传播 → 深度更新 →
//! 视觉更新 → 建图。本 crate 定义 [`Odometry`] trait，供 `apps/void` 接线。

use firefly_void_types::sensor::{CameraFrame, ImuSample};
use firefly_void_types::state::State;

/// 里程计抽象：算法实现与 iceoryx2 接线分离（对照 `LIVMapper` 的公开接口）。
pub trait Odometry {
    /// 馈入一次 IMU 测量（内部推进状态与协方差）。
    fn feed_imu(&mut self, sample: &ImuSample);

    /// 馈入一帧深度+图像测量（传播后执行深度/视觉顺序更新）。
    fn feed_frame(&mut self, camera: &CameraFrame<'_>);

    /// 当前估计状态。
    fn state(&self) -> &State;
}
