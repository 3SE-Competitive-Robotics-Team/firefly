//! 视觉定位消息（iceoryx2 zero-copy）。
//!
//! `aliked` 进程发布特征，`lightglue`/`gicp` 进程发布位姿观测，`fusion`
//! 进程统一融合。对照 VINS-Fusion 的 `keyframe_pose`/`keyframe_point` 分工：
//! 特征与观测分离，融合核保持传感器无关。

use iceoryx2::prelude::*;

/// 特征话题（`aliked` 进程发布，`lightglue` 进程订阅）。
pub const FEATURE_TOPIC: &str = "Firefly/Features";

/// 位姿观测话题（`lightglue` 视觉观测 + `gicp` 几何观测发布，`fusion` 订阅）。
pub const POSE_OBS_TOPIC: &str = "Firefly/PoseObservation";

/// 单帧最大特征点数（与 `aliked-n16-k512.onnx` 导出约定一致）。
pub const MAX_FEATURES: usize = 512;
/// ALIKED-N16 描述子维度（与导出约定一致）。
pub const DESC_DIM: usize = 128;

/// 观测来源（`PoseObservation::source`）。
pub const OBS_SOURCE_GICP: u32 = 0;
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
    /// 描述子 `[N, 128]`。
    pub descriptors: [[f32; DESC_DIM]; MAX_FEATURES],
    /// 检测得分 `[N]`。
    pub scores: [f32; MAX_FEATURES],
}

/// 位姿观测：全局系位姿 + 协方差 + 质量门控字段，直接喂 `FusionFilter`。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyPoseObservation")]
pub struct PoseObservation {
    /// 观测对应时刻（秒，与 `odom` 同一时钟，用于插值对齐）。
    pub timestamp: f64,
    /// 来源（`OBS_SOURCE_*`）。
    pub source: u32,
    /// 全局位置。
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    /// 全局姿态四元数 `[x, y, z, w]`。
    pub quat_x: f64,
    pub quat_y: f64,
    pub quat_z: f64,
    pub quat_w: f64,
    /// 观测协方差 `R`（6×6 行主序，`[rot, trans]`，与 `FusionFilter` 一致）。
    pub covariance: [f64; 36],
    /// 内点数（门控用）。
    pub num_inliers: u32,
    /// 总点数（门控用）。
    pub total_points: u32,
    /// 配准残差（门控用）。
    pub error: f64,
    /// 是否收敛。
    pub converged: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vision_messages_are_plain_old_data() {
        // 8 + 4 + 4096 + 262144 + 2048 = 268300，尾部按 f64 补齐 +4。
        assert_eq!(std::mem::size_of::<FeatureMessage>(), 268_304);
        assert_eq!(std::mem::size_of::<PoseObservation>(), 384);
        assert_eq!(MAX_FEATURES, 512);
        assert_eq!(DESC_DIM, 128);
    }
}
