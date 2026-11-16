//! 视觉降落目标：与图像时刻配对的 odom 系机体接触位置。
use iceoryx2::prelude::*;
pub const LANDING_TARGET_TOPIC: &str = "Firefly/LandingTarget";

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, ZeroCopySend)]
#[type_name("FireflyLandingTargetMessage")]
pub struct LandingTargetMessage {
    pub timestamp: f64,
    pub session: u64,
    pub position: [f64; 3],
    pub reprojection_px: f64,
    pub confidence: f64,
    /// red=0，blue=1。
    pub team: u32,
    pub valid: u32,
}
