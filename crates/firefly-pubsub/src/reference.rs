use iceoryx2::prelude::*;

pub const REFERENCE_TOPIC: &str = "Firefly/Reference";

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
        assert_eq!(std::mem::size_of::<ReferenceMessage>(), 168);
        assert!((m.timestamp + 1.0).abs() < 1e-9);
    }
}
