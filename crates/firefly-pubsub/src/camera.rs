//! 双目/深度使用 [`IMAGE_WIDTH`] × [`IMAGE_HEIGHT`]，下视使用
//! [`DOWN_WIDTH`] × [`DOWN_HEIGHT`]；订阅端按对应定长消息接收。

mod publisher;
pub use publisher::{DepthImagePublisher, DownImagePublisher, GrayImagePublisher};

use iceoryx2::prelude::*;

// Both views use the same IPC name and field layout. The publisher initializes
// the pixel array in its private slot before any subscriber can see it.
macro_rules! image_layout {
    ($(#[$attr:meta])* $vis:vis $name:ident, $wire:literal, $pixel:ty, $len:ident) => {
        $(#[$attr])*
        #[repr(C)]
        #[derive(Debug, Clone, Copy, ZeroCopySend)]
        #[type_name($wire)]
        $vis struct $name {
            pub timestamp: f64,
            pub sensor_id: i32,
            pub width: u32,
            pub height: u32,
            pub data: [$pixel; $len],
        }
    };
}

macro_rules! image_message {
    ($(#[$attr:meta])* $name:ident, $slot:ident, $wire:literal, $pixel:ty, $len:ident) => {
        image_layout!($(#[$attr])* pub $name, $wire, $pixel, $len);
        image_layout!($slot, $wire, std::mem::MaybeUninit<$pixel>, $len);
    };
}

pub const CAMERA_RIG_TOPIC: &str = "Firefly/CameraRig";

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, ZeroCopySend)]
pub struct CameraCalibration {
    pub resolution: [u32; 2],
    /// fx, fy, cx, cy in pixel-center coordinates.
    pub intrinsics: [f64; 4],
    pub translation: [f64; 3],
    /// RDF optical frame to body.
    pub rotation_xyzw: [f64; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, ZeroCopySend)]
#[type_name("FireflyCameraRig")]
pub struct CameraRig {
    pub left: CameraCalibration,
    pub right: CameraCalibration,
    pub down: CameraCalibration,
}

pub const IMAGE_WIDTH: usize = 320;
pub const IMAGE_HEIGHT: usize = 240;
pub const IMAGE_SIZE: usize = IMAGE_WIDTH * IMAGE_HEIGHT;

pub const DOWN_WIDTH: usize = 352;
pub const DOWN_HEIGHT: usize = 288;
pub const DOWN_SIZE: usize = DOWN_WIDTH * DOWN_HEIGHT;
pub const CAMERA_DOWN_TOPIC: &str = "Firefly/CameraDown";
/// 下视相机保留 id；2 为现有深度源。
pub const DOWN_SENSOR_ID: i32 = 3;

image_message!(
    /// Downward monochrome frame in row-major order.
    DownImageMessage,
    DownSlot,
    "FireflyDownImageMessage",
    u8,
    DOWN_SIZE
);

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

image_message!(
    /// Row-major stereo pixels with a sensor-clock timestamp and left (0) or right (1) ID.
    GrayImageMessage,
    GraySlot,
    "FireflyGrayImageMessage",
    u8,
    IMAGE_SIZE
);

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

image_message!(
    /// Axial depth in metres, in row-major left-camera pixel order.
    DepthImageMessage,
    DepthSlot,
    "FireflyDepthImageMessage",
    f32,
    IMAGE_SIZE
);

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
    fn partial_depth_images_are_discarded_and_the_next_loan_is_complete() {
        use crate::{node::create_node, subscriber::Subscriber};
        let node = create_node().unwrap();
        let topic = format!("Firefly/Test/DepthPixels/{}", std::process::id());
        let subscriber = Subscriber::<DepthImageMessage>::with_topic(&node, &topic).unwrap();
        let publisher = DepthImagePublisher::open(&node, &topic, None).unwrap();
        for n in [IMAGE_SIZE - 1, IMAGE_SIZE + 1] {
            assert!(
                publisher
                    .publish_pixels(1., 0, std::iter::repeat_n(1., n))
                    .is_err()
            );
            assert!(subscriber.receive().unwrap().is_none());
        }
        let pixels = (0..IMAGE_SIZE).map(|i| f32::from_bits(i as u32));
        publisher.publish_pixels(2., 0, pixels.clone()).unwrap();
        let sample = subscriber.receive().unwrap().unwrap();
        assert_eq!(
            (sample.timestamp, sample.width, sample.height),
            (2., IMAGE_WIDTH as u32, IMAGE_HEIGHT as u32)
        );
        assert!(
            sample
                .data
                .iter()
                .zip(pixels)
                .all(|(a, b)| a.to_bits() == b.to_bits())
        );
    }

    #[test]
    fn depth_rows_reach_the_subscriber_in_image_order() {
        use crate::{node::create_node, subscriber::Subscriber};
        let node = create_node().unwrap();
        let topic = format!("Firefly/Test/DepthRows/{}", std::process::id());
        let subscriber = Subscriber::<DepthImageMessage>::with_topic(&node, &topic).unwrap();
        let publisher = DepthImagePublisher::open(&node, &topic, None).unwrap();
        publisher
            .publish_rows(3., 0, |y| {
                std::array::from_fn(|x| (y * IMAGE_WIDTH + x) as f32)
            })
            .unwrap();
        let sample = subscriber.receive().unwrap().unwrap();
        assert_eq!(
            (
                sample.timestamp,
                sample.sensor_id,
                sample.width,
                sample.height
            ),
            (3., 0, IMAGE_WIDTH as u32, IMAGE_HEIGHT as u32)
        );
        assert!(sample.data.iter().enumerate().all(|(i, v)| *v == i as f32));
    }

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
    schedule: Option<FrameSchedule>,
}

struct FrameSchedule {
    last: f64,
    next: f64,
    rate: u32,
}

impl FrameCadence {
    pub fn admit(&mut self, timestamp: f64, rate: u32) -> bool {
        if rate == 0 || !timestamp.is_finite() {
            return false;
        }
        let period = 1. / f64::from(rate);
        let Some(schedule) = &mut self.schedule else {
            self.schedule = Some(FrameSchedule {
                last: timestamp,
                next: timestamp + period,
                rate,
            });
            return true;
        };
        if schedule.rate != rate {
            schedule.next = schedule.last + period;
            schedule.rate = rate;
        }
        let quantum = 0.001 + 1e-9;
        if timestamp + quantum < schedule.next
            || timestamp <= schedule.last
            || timestamp - schedule.last + quantum < period
        {
            return false;
        }
        schedule.last = timestamp;
        schedule.next += period;
        if schedule.next <= timestamp {
            schedule.next = timestamp + period;
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
    fn rate_change_reschedules_from_last_admitted_frame() {
        let mut cadence = FrameCadence::default();
        assert!(!cadence.admit(f64::NAN, 30));
        assert!(!cadence.admit(0., 0));
        assert!(cadence.admit(1., 30));
        assert!(!cadence.admit(1.04, 10));
        assert!(cadence.admit(1.1, 10));
        assert!(cadence.admit(1.15, 20));
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
