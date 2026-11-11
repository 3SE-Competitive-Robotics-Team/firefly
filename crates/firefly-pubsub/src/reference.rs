//! 规划参考消息（规划 → 飞控）。
//!
//! 轨迹输出进程把执行中轨迹的参考状态（位置/速度）发布到 `Firefly/Reference`，
//! 默认局部规划器直接输出 odom 参考，携带恒等变换快照。
//! 通用外部参考可携带参考系←odom 快照，飞控将其转换到 odom。`#[repr(C)]` 定长零拷贝。

use iceoryx2::prelude::*;

/// 参考话题（规划 → 飞控）。
pub const REFERENCE_TOPIC: &str = "Firefly/Reference";

/// 参考状态消息：轨迹在当前时刻的目标位置/速度。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyReferenceMessage")]
pub struct ReferenceMessage {
    /// 仿真时钟时间戳（秒）。
    pub timestamp: f64,
    /// 参考位置 `p`（参考系，米）。
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    /// 参考速度 `v`（米/秒）。
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub velocity_z: f64,
    /// 参考加速度 `a`（m/s²，地图系；位置环前馈，消掉 a/kp 稳态滞后）。
    pub acceleration_x: f64,
    pub acceleration_y: f64,
    pub acceleration_z: f64,
    /// 参考偏航角 yaw（rad，地图系 x 轴为零向，包装在 [-π,π]）。
    pub yaw: f64,
    /// 参考偏航角速度（rad/s）。
    pub yaw_dot: f64,
    /// 与参考原子发布的参考系←odom 快照；局部 planner 固定恒等且 `has_alignment=1`。
    pub alignment_translation: [f64; 3],
    pub alignment_rotation: [f64; 4],
    /// 1 使用消息快照；0 使用同刻配对的外部地图变换。
    pub has_alignment: u64,
    /// 生成此参考的目标身份，回显 GoalMessage.timestamp；启动预置目标为 0。
    pub goal_timestamp: f64,
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
            acceleration_x: 0.0,
            acceleration_y: 0.0,
            acceleration_z: 0.0,
            yaw: 0.0,
            yaw_dot: 0.0,
            alignment_translation: [0.; 3],
            alignment_rotation: [0., 0., 0., 1.],
            has_alignment: 0,
            goal_timestamp: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_message_is_plain_old_data() {
        let m = ReferenceMessage::default();
        // 12 × f64 参考 + 7 × f64 变换 + u64 标记 + f64 目标身份。
        assert_eq!(std::mem::size_of::<ReferenceMessage>(), 168);
        assert!((m.timestamp + 1.0).abs() < 1e-9);
    }
}
