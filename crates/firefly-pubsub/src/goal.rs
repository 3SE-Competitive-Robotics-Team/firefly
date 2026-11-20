use iceoryx2::prelude::*;

pub const GOAL_TOPIC: &str = "Firefly/Goal";

impl crate::request::MissionRequest for GoalMessage {
    const TOPIC: &'static str = GOAL_TOPIC;
    const BUFFER_SIZE: usize = 4;
}
pub const STOP: u64 = 0;
pub const PASS: u64 = 1;

/// 飞行目标消息：目标点位置（odom 系，米）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyGoalMessage")]
pub struct GoalMessage {
    /// 发送时刻（墙钟秒），同时作为单调递增的目标身份；重复投递保持不变。
    /// 规划仍以自身 sim 时钟为准，参考与执行状态回显此值供任务层关联。
    pub timestamp: f64,
    /// 目标位置 `p`（odom 系，米）。
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    /// Optional terminal heading in odom radians. `has_yaw` must be 0 or 1.
    pub yaw: f64,
    pub has_yaw: u64,
    /// STOP uses a stationary goal. PASS continues to the supplied successor.
    pub arrival: u64,
    pub next_position: [f64; 3],
}

impl GoalMessage {
    /// Validate the complete command before changing the active goal.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        let point = [self.position_x, self.position_y, self.position_z];
        self.timestamp.is_finite()
            && point.iter().all(|x| x.is_finite())
            && self.yaw.is_finite()
            && match self.arrival {
                STOP => self.has_yaw <= 1,
                PASS => {
                    self.has_yaw == 0
                        && self.next_position.iter().all(|x| x.is_finite())
                        && point
                            .iter()
                            .zip(self.next_position)
                            .any(|(a, b)| (a - b).abs() >= 1e-6)
                }
                _ => false,
            }
    }
}

impl Default for GoalMessage {
    fn default() -> Self {
        Self {
            timestamp: -1.0,
            position_x: 0.0,
            position_y: 0.0,
            position_z: 0.0,
            yaw: 0.0,
            has_yaw: 0,
            arrival: STOP,
            next_position: [0.; 3],
        }
    }
}
