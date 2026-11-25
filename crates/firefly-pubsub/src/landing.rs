//! 视觉降落目标：与图像时刻配对的 odom 系机体接触位置。
use iceoryx2::prelude::*;
pub const LANDING_TARGET_TOPIC: &str = "Firefly/LandingTarget";

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, ZeroCopySend)]
#[type_name("FireflyLandingTargetMessageV2")]
pub struct LandingTargetMessage {
    pub timestamp: f64,
    pub session: u64,
    pub position: [f64; 3],
    pub reprojection_px: f64,
    pub confidence: f64,
    /// red=0，blue=1。
    pub team: u32,
    pub valid: u32,
    /// 原始图像六自由度观测可用（0/1），时间与图像采集时刻一致。
    pub h_pose_valid: u32,
    /// H 图案中心在 FLU 机体系中的位置，米。
    pub body_from_h_translation: [f64; 3],
    /// 机体 ← H 的 xyzw；右乘 H 系绕 Z 的半周旋转为等价观测。
    pub body_from_h_xyzw: [f64; 4],
    pub h_pose_reprojection_px: f64,
}
