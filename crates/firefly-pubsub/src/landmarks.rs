//! 原子 VIO 关键帧：图像、标定、观测和几何初值来自同一相机更新快照。
use iceoryx2::prelude::*;

pub const TOPIC: &str = "Firefly/LandmarkKeyframe";
pub const MAX_POINTS: usize = 1024;
/// 5Hz 关键帧覆盖 10s 等待窗口，额外留有有界余量。
pub const KEYFRAME_BUFFER: usize = 64;
pub const CORRECTION_BUFFER: usize = 4;
pub const CORRECTION_TOPIC: &str = "Firefly/LandmarkCorrection";

/// 后端完成一次约束处理后的全图快照；`accepted` 决定是否允许融合跨区域路标。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyLandmarkCorrection")]
pub struct Correction {
    pub from: f64,
    pub to: f64,
    pub accepted: bool,
    /// false 表示离线地图锚点引发的全图修正，不是回环事务确认。
    pub loop_event: bool,
    pub count: u32,
    pub poses: [crate::odom::OdomMessage; 4096],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyLandmarkObservation")]
pub struct Observation {
    pub track: u64,
    pub pixel: [f32; 2],
    pub normalized: [f64; 2],
    pub right_normalized: [f64; 2],
    pub has_right: bool,
    pub point_body: [f64; 3],
    pub has_point: bool,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyLandmarkKeyframe")]
pub struct Keyframe {
    pub session: u64,
    pub camera_time: f64,
    pub odom: crate::odom::OdomMessage,
    /// `OpenCV` camera←body，行主序旋转与平移；相机轴为右、下、前。
    pub camera_rotation: [f64; 9],
    pub camera_translation: [f64; 3],
    pub intrinsics: [f64; 8],
    pub fisheye: bool,
    pub stereo: bool,
    pub right_rotation: [f64; 9],
    pub right_translation: [f64; 3],
    pub right_intrinsics: [f64; 8],
    pub right_fisheye: bool,
    pub right_image: [u8; 320 * 240],
    pub width: u32,
    pub height: u32,
    pub image: [u8; 320 * 240],
    pub count: u32,
    pub observations: [Observation; MAX_POINTS],
}
