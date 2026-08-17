//! 参考状态消息（闭环控制：规划 → 物理环境）。
//!
//! 规划进程把执行中轨迹的参考状态（位置/速度）发布到 `Firefly/Reference`，
//! `MuJoCo` 物理环境订阅后施加 PD 控制，实现闭环。`#[repr(C)]` 定长零拷贝。

use iceoryx2::prelude::*;

/// 参考状态话题（规划 → 物理环境）。
pub const REFERENCE_TOPIC: &str = "Firefly/Reference";

/// 参考状态消息：轨迹在当前时刻的目标位置/速度。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyReferenceMessage")]
pub struct ReferenceMessage {
    /// 仿真时钟时间戳（秒）。
    pub timestamp: f64,
    /// 参考位置 `p`（地图系，米）。
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    /// 参考速度 `v`（米/秒）。
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub velocity_z: f64,
}

impl Default for ReferenceMessage {
    fn default() -> Self {
        Self {
            timestamp: -1.0,
            position_x: 0.0,
            position_y: 0.0,
            position_z: 0.0,
            velocity_x: 0.0,
            velocity_y: 0.0,
            velocity_z: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_message_is_plain_old_data() {
        let m = ReferenceMessage::default();
        // 7 × f64（timestamp + 位置 3 + 速度 3）
        assert_eq!(std::mem::size_of::<ReferenceMessage>(), 56);
        assert!((m.timestamp + 1.0).abs() < 1e-9);
    }
}
