//! 传感器测量数据（对照 `OpenVINS` `ov_core/utils/sensor_data.h`）。

use nalgebra::Vector3;
use std::sync::Arc;

/// IMU 测量：时间戳 + 角速度（gyro）+ 加速度（accel）。
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub timestamp: f64,
    pub wm: Vector3<f64>,
    pub am: Vector3<f64>,
}

/// 自有的连续灰度像素，按行存储。
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
    /// 不可变图像可由输入、当前帧和上一帧状态共享。
    pub images: Vec<Arc<GrayImage>>,
    /// 与各图像同尺寸；零表示有效，不能以空图代替全零掩码。
    pub masks: Vec<Arc<GrayImage>>,
}

impl CameraData {
    /// 检查完整图像/掩码配对及唯一相机 id；必须在更新跟踪状态前调用。
    #[must_use]
    pub fn is_valid(&self) -> bool {
        !self.sensor_ids.is_empty()
            && self.sensor_ids.len() == self.images.len()
            && self.images.len() == self.masks.len()
            && self
                .sensor_ids
                .iter()
                .enumerate()
                .all(|(i, id)| !self.sensor_ids[..i].contains(id))
            && self.images.iter().zip(&self.masks).all(|(image, mask)| {
                image.width > 0
                    && image.height > 0
                    && image.width.checked_mul(image.height) == Some(image.data.len())
                    && mask.width == image.width
                    && mask.height == image.height
                    && mask.data.len() == image.data.len()
            })
    }
}

/// 时间戳最小的相机 id（数据排序用，对照 `sensor_data.h` 的 `operator<`）。
#[must_use]
pub fn min_sensor_id(cam: &CameraData) -> i32 {
    cam.sensor_ids.iter().copied().min().unwrap_or(0)
}
