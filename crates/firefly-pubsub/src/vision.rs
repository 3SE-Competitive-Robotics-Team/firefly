use iceoryx2::prelude::*;

pub const POSE_OBS_TOPIC: &str = "Firefly/PoseObservation";

/// 观测来源：外部视觉定位。
pub const OBS_SOURCE_VISUAL: u32 = 1;

/// 位姿观测：全局系位姿 + 协方差 + 质量门控字段，由定位后端接收。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyPoseObservation")]
pub struct PoseObservation {
    /// 观测对应时刻（秒，与 `odom` 同一时钟，用于插值对齐）。
    pub timestamp: f64,
    pub source: u32,
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    /// 全局姿态四元数 `[x, y, z, w]`。
    pub quat_x: f64,
    pub quat_y: f64,
    pub quat_z: f64,
    pub quat_w: f64,
    /// 机体位姿右扰动协方差 `R`（6×6 行主序，`[rot, trans]`，表达在 body 系）。
    pub covariance: [f64; 36],
    pub num_inliers: u32,
    pub total_points: u32,
    pub error: f64,
    pub converged: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_observation_has_fixed_ipc_size() {
        assert_eq!(std::mem::size_of::<PoseObservation>(), 384);
    }
}
