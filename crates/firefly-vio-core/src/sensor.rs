//! 传感器测量数据（对照 `OpenVINS` `ov_core/utils/sensor_data.h`）。

use nalgebra::Vector3;

/// IMU 测量：时间戳 + 角速度（gyro）+ 加速度（accel）。
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub timestamp: f64,
    pub wm: Vector3<f64>,
    pub am: Vector3<f64>,
}

/// 灰度图像（共享内存友好的平面数据）。
#[derive(Debug, Clone)]
pub struct GrayImage {
    pub width: usize,
    pub height: usize,
    pub data: Vec<u8>,
}

/// 相机测量：时间戳 + 各相机（`sensor_id`）的图像与掩码。
#[derive(Debug, Clone)]
pub struct CameraData {
    pub timestamp: f64,
    pub sensor_ids: Vec<i32>,
    pub images: Vec<GrayImage>,
    pub masks: Vec<GrayImage>,
}

/// 时间戳最小的相机 id（数据排序用，对照 `sensor_data.h` 的 `operator<`）。
#[must_use]
pub fn min_sensor_id(cam: &CameraData) -> i32 {
    cam.sensor_ids.iter().copied().min().unwrap_or(0)
}
