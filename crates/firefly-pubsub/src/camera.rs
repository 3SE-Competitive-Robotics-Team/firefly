//! 图像消息（双目灰度 + 深度）与话题。
//!
//! `#[repr(C)]` 定长结构，满足 `ZeroCopySend` 约束（自包含、无堆指针、
//! 统一内存布局、`'static`、无 `Drop`）。图像数据内嵌定长数组，
//! iceoryx2 共享内存零拷贝传输；trace 上下文由发布中间件写入
//! **User Header**（见 [`crate::trace`]）。
//!
//! 图像分辨率统一（[`IMAGE_WIDTH`] × [`IMAGE_HEIGHT`]），由合成/物理环境
//! （`MuJoCo`）渲染后发布；订阅端转换成领域层 `GrayImage`/`CameraData`。

use iceoryx2::prelude::*;

/// 图像宽度（像素）。
pub const IMAGE_WIDTH: usize = 320;
/// 图像高度（像素）。
pub const IMAGE_HEIGHT: usize = 240;
/// 灰度/深度图像素总数（320×240）。
pub const IMAGE_SIZE: usize = IMAGE_WIDTH * IMAGE_HEIGHT;

/// 左目灰度话题（`sensor_id = 0`）。
pub const CAMERA_LEFT_TOPIC: &str = "Firefly/CameraLeft";
/// 右目灰度话题（`sensor_id = 1`）。
pub const CAMERA_RIGHT_TOPIC: &str = "Firefly/CameraRight";
/// 深度话题。
pub const DEPTH_TOPIC: &str = "Firefly/Depth";

/// 灰度图像消息（定长 u8 数组，零拷贝）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyGrayImageMessage")]
pub struct GrayImageMessage {
    /// 传感器时钟时间戳（秒）。
    pub timestamp: f64,
    /// 相机 id（左目 0 / 右目 1）。
    pub sensor_id: i32,
    /// 图像宽度（像素）。
    pub width: u32,
    /// 图像高度（像素）。
    pub height: u32,
    /// 灰度像素（行主序）。
    pub data: [u8; IMAGE_SIZE],
}

impl Default for GrayImageMessage {
    // 定长共享内存消息（76KB），大数组是设计使然
    #[allow(clippy::large_stack_arrays)]
    fn default() -> Self {
        Self {
            timestamp: -1.0,
            sensor_id: 0,
            width: 0,
            height: 0,
            data: [0u8; IMAGE_SIZE],
        }
    }
}

/// 深度图像消息（定长 f32 数组，米制，零拷贝）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyDepthImageMessage")]
pub struct DepthImageMessage {
    /// 传感器时钟时间戳（秒）。
    pub timestamp: f64,
    /// 相机 id。
    pub sensor_id: i32,
    /// 图像宽度（像素）。
    pub width: u32,
    /// 图像高度（像素）。
    pub height: u32,
    /// 深度像素（米，行主序）。
    pub data: [f32; IMAGE_SIZE],
}

impl Default for DepthImageMessage {
    // 定长共享内存消息（307KB），大数组是设计使然
    #[allow(clippy::large_stack_arrays)]
    fn default() -> Self {
        Self {
            timestamp: -1.0,
            sensor_id: 0,
            width: 0,
            height: 0,
            data: [0.0; IMAGE_SIZE],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gray_image_is_plain_old_data() {
        let m = GrayImageMessage::default();
        // 8(ts) + 12(sensor_id/width/height) + 76800(data)，对齐 8
        assert_eq!(std::mem::size_of::<GrayImageMessage>(), 76824);
        assert!((m.timestamp + 1.0).abs() < 1e-9);
        assert_eq!(m.data.len(), IMAGE_SIZE);
    }

    #[test]
    fn depth_image_is_plain_old_data() {
        let m = DepthImageMessage::default();
        // 8(ts) + 12 + 4*76800(data)，对齐 8
        assert_eq!(std::mem::size_of::<DepthImageMessage>(), 307_224);
        assert!(m.data.iter().all(|&v| v == 0.0));
    }
}
