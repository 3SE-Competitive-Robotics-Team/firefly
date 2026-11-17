//! 双目/深度使用 [`IMAGE_WIDTH`] × [`IMAGE_HEIGHT`]，下视使用
//! [`DOWN_WIDTH`] × [`DOWN_HEIGHT`]；订阅端按对应定长消息接收。

use iceoryx2::prelude::*;

pub const IMAGE_WIDTH: usize = 320;
pub const IMAGE_HEIGHT: usize = 240;
pub const IMAGE_SIZE: usize = IMAGE_WIDTH * IMAGE_HEIGHT;

pub const DOWN_WIDTH: usize = 352;
pub const DOWN_HEIGHT: usize = 288;
pub const DOWN_SIZE: usize = DOWN_WIDTH * DOWN_HEIGHT;
pub const CAMERA_DOWN_TOPIC: &str = "Firefly/CameraDown";
/// 下视相机保留 id；2 为现有深度源。
pub const DOWN_SENSOR_ID: i32 = 3;

/// 下视原图，独立于双目的分辨率与消息签名。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyDownImageMessage")]
pub struct DownImageMessage {
    pub timestamp: f64,
    pub sensor_id: i32,
    pub width: u32,
    pub height: u32,
    pub data: [u8; DOWN_SIZE],
}

impl Default for DownImageMessage {
    #[allow(clippy::large_stack_arrays)]
    fn default() -> Self {
        Self {
            timestamp: -1.0,
            sensor_id: DOWN_SENSOR_ID,
            width: DOWN_WIDTH as u32,
            height: DOWN_HEIGHT as u32,
            data: [0; DOWN_SIZE],
        }
    }
}

/// 左目灰度话题（`sensor_id = 0`）。
pub const CAMERA_LEFT_TOPIC: &str = "Firefly/CameraLeft";
/// 右目灰度话题（`sensor_id = 1`）。
pub const CAMERA_RIGHT_TOPIC: &str = "Firefly/CameraRight";
pub const DEPTH_TOPIC: &str = "Firefly/Depth";

#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyGrayImageMessage")]
pub struct GrayImageMessage {
    /// 传感器时钟时间戳（秒）。
    pub timestamp: f64,
    /// 相机 id（左目 0 / 右目 1）。
    pub sensor_id: i32,
    pub width: u32,
    pub height: u32,
    /// 灰度像素（行主序）。
    pub data: [u8; IMAGE_SIZE],
}

impl Default for GrayImageMessage {
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
    pub sensor_id: i32,
    pub width: u32,
    pub height: u32,
    /// 深度像素（米，行主序）。
    pub data: [f32; IMAGE_SIZE],
}

impl Default for DepthImageMessage {
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
    fn down_image_has_cif_layout_and_independent_sensor_identity() {
        let message = DownImageMessage::default();
        assert_eq!(std::mem::size_of::<DownImageMessage>(), 101_400);
        assert_eq!(std::mem::offset_of!(DownImageMessage, data), 20);
        assert_eq!(
            (message.width, message.height, message.sensor_id),
            (352, 288, 3)
        );
        assert_eq!(message.data.len(), 101_376);
    }

    #[test]
    fn gray_image_is_plain_old_data() {
        let m = GrayImageMessage::default();
        assert_eq!(std::mem::size_of::<GrayImageMessage>(), 76824);
        assert!((m.timestamp + 1.0).abs() < 1e-9);
        assert_eq!(m.data.len(), IMAGE_SIZE);
    }

    #[test]
    fn depth_image_is_plain_old_data() {
        let m = DepthImageMessage::default();
        assert_eq!(std::mem::size_of::<DepthImageMessage>(), 307_224);
        assert!(m.data.iter().all(|&v| v == 0.0));
    }
}

/// 采集时间域的节拍；容纳 1ms 测量时钟量化，保留相位并跳过停顿后的积压拍。
#[derive(Default)]
pub struct FrameCadence {
    last: Option<f64>,
    next: f64,
    rate: u32,
}

impl FrameCadence {
    pub fn admit(&mut self, timestamp: f64, rate: u32) -> bool {
        if rate == 0 || !timestamp.is_finite() {
            return false;
        }
        let period = 1. / f64::from(rate);
        if self.rate != rate {
            self.next = self.last.map_or(timestamp, |last| last + period);
            self.rate = rate;
        }
        let quantum = 0.001 + 1e-9;
        if timestamp + quantum < self.next
            || self
                .last
                .is_some_and(|last| timestamp <= last || timestamp - last + quantum < period)
        {
            return false;
        }
        self.last = Some(timestamp);
        self.next += period;
        if self.next <= timestamp {
            self.next = timestamp + period;
        }
        true
    }
}

#[cfg(test)]
mod cadence_tests {
    use super::FrameCadence;
    #[test]
    fn cadence_caps_30_hz_source_at_15_without_catchup() {
        let mut cadence = FrameCadence::default();
        assert_eq!(
            (0..300)
                .filter(|i| cadence.admit(f64::from(*i) / 30., 15))
                .count(),
            150
        );
        assert!(!cadence.admit(0., 15));
        assert!(cadence.admit(100., 15));
        assert!(!cadence.admit(100., 15));
    }

    #[test]
    fn millisecond_capture_clock_preserves_15_and_30_fps() {
        for rate in [15, 30] {
            let mut cadence = FrameCadence::default();
            let count = (0..600)
                .filter(|i| {
                    let timestamp = (f64::from(*i) / 30. * 1000.).round() / 1000.;
                    cadence.admit(timestamp, rate)
                })
                .count();
            assert_eq!(count, rate as usize * 20);
        }
        let mut cadence = FrameCadence::default();
        let count = (0..6000)
            .filter(|i| cadence.admit(f64::from(*i) / 300., 15))
            .count();
        assert!(count <= 300, "fast sources obey the same long-term rate");
    }
}
