//! 传感器数据结构（对照 `common_lib.h:62` `MeasureGroup` 与 ROS 消息解包）。
//!
//! 仿真输入（DESIGN.md §3）：IMU 100Hz、左目灰度 10Hz、深度图 10Hz；
//! 深度相机与左目共面，时间戳以仿真秒计。

use nalgebra::Vector3;

/// 一帧 IMU 测量（对照 `sensor_msgs::Imu`：角速度/加速度，单位 rad/s、m/s²）。
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    /// 时间戳（仿真秒）。
    pub t: f64,
    /// `ω_m`：原始角速度（机体系，rad/s，未去零偏）。
    pub omega: Vector3<f64>,
    /// `a_m`：原始加速度（机体系，m/s²，未去零偏）。
    pub acc: Vector3<f64>,
}

/// 一帧相机测量：左目灰度图（对照 `cv::Mat` 灰度 + `MeasureGroup.img`）。
///
/// 图像以 `&[u8]` 灰度数据引用持有，配合 `width`/`height` 构成 320×240
/// 左目灰度帧；本阶段只定义结构，像素访问由 `firefly-void-measure` 在 P3 实现。
#[derive(Debug)]
pub struct CameraFrame<'a> {
    /// 时间戳（仿真秒）。
    pub t: f64,
    /// 左目灰度图数据（行主序，`width × height`，单通道 8bit）。
    pub left_gray: &'a [u8],
    /// 图像宽度（像素）。
    pub width: usize,
    /// 图像高度（像素）。
    pub height: usize,
}

/// 一帧深度测量（对照 `sensor_msgs::Image` 深度图反投影的输入）。
///
/// `depth` 为行主序的深度值（单位 m，`0.0` 表示无效/空洞，与仿真
/// 深度噪声模型一致：disparity σ∝z²、5-15% 空洞）。
#[derive(Debug)]
pub struct DepthFrame<'a> {
    /// 时间戳（仿真秒）。
    pub t: f64,
    /// 深度图数据（行主序，`width × height`，单位 m）。
    pub depth: &'a [f64],
    /// 图像宽度（像素）。
    pub width: usize,
    /// 图像高度（像素）。
    pub height: usize,
}
