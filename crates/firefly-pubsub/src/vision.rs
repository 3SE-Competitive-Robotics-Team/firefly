use iceoryx2::prelude::*;

pub const FEATURE_TOPIC: &str = "Firefly/Features";

pub const POSE_OBS_TOPIC: &str = "Firefly/PoseObservation";

/// 单帧最大特征点数（与 `aliked-n16-k512.onnx` 导出约定一致）。
pub const MAX_FEATURES: usize = 512;
/// ALIKED-N16 描述子维度（与导出约定一致）。
pub const DESC_DIM: usize = 128;

/// 观测来源：视觉全局定位（`aliked` + `lightglue` + `PnP`）。
pub const OBS_SOURCE_VISUAL: u32 = 1;

/// 特征消息：定长 512 点（有效数为 `count`，其余为零填充）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyFeatureMessage")]
pub struct FeatureMessage {
    /// 图像时间戳（秒，相机时钟）。
    pub timestamp: f64,
    /// 有效特征点数（`≤ MAX_FEATURES`）。
    pub count: u32,
    /// 像素坐标 `[N, 2]`（原图系）。
    pub keypoints: [[f32; 2]; MAX_FEATURES],
    pub descriptors: [[f32; DESC_DIM]; MAX_FEATURES],
    pub scores: [f32; MAX_FEATURES],
}

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
    fn vision_messages_are_plain_old_data() {
        assert_eq!(std::mem::size_of::<FeatureMessage>(), 268_304);
        assert_eq!(std::mem::size_of::<PoseObservation>(), 384);
        assert_eq!(MAX_FEATURES, 512);
        assert_eq!(DESC_DIM, 128);
    }
}
