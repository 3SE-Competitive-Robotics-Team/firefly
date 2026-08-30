//! 里程计消息（iceoryx2 zero-copy）。
//!
//! `#[repr(C)]` 定长结构，满足 `ZeroCopySend` 约束（自包含、无堆指针、
//! 统一内存布局、`'static`、无 `Drop`）。trace 上下文（`trace_id`/`span_id`/
//! 双时间戳）由发布中间件写入 **User Header**（见 [`crate::trace`]），
//! payload 保持精简。

use iceoryx2::prelude::*;

/// 真值话题（MuJoCo 物理环境发布，仿真阶段感知位姿源）。
pub const GROUND_TRUTH_TOPIC: &str = "Firefly/GroundTruth";

/// 校正后里程计话题（GICP 融合进程发布，planner 订阅，低频全局矫正 VIO 漂移）。
pub const CORRECTED_ODOM_TOPIC: &str = "Firefly/CorrectedOdometry";

/// 里程计消息（对照 `docs/architecture.md` 的 `topic: odom`）。
///
/// 字段布局与 `firefly-vio` 的 `State` 输出对应：
/// - 位置/速度在全局系（`p_IinG`/`v_IinG`）；
/// - 姿态为 JPL 四元数 `q_GtoI`（标量在最后，与 `firefly-vio-types` 一致）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyOdomMessage")]
pub struct OdomMessage {
    /// 相机时钟时间戳（秒）。
    pub timestamp: f64,
    /// 位置 `p_IinG`。
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    /// 速度 `v_IinG`。
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub velocity_z: f64,
    /// 姿态四元数 `q_GtoI = [x, y, z, w]`。
    pub quat_x: f64,
    pub quat_y: f64,
    pub quat_z: f64,
    pub quat_w: f64,
    /// 估计器是否已初始化。
    pub is_initialized: bool,
}

impl Default for OdomMessage {
    fn default() -> Self {
        Self {
            timestamp: -1.0,
            position_x: 0.0,
            position_y: 0.0,
            position_z: 0.0,
            velocity_x: 0.0,
            velocity_y: 0.0,
            velocity_z: 0.0,
            quat_x: 0.0,
            quat_y: 0.0,
            quat_z: 0.0,
            quat_w: 1.0,
            is_initialized: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn odom_message_is_plain_old_data() {
        // ZeroCopySend 约束的自检：定长（可逐字节复制）、默认值合法
        let m = OdomMessage::default();
        assert_eq!(std::mem::size_of::<OdomMessage>(), 96);
        assert!((m.quat_w - 1.0).abs() < 1e-12);
        assert!(!m.is_initialized);
    }
}
