//! 飞行目标消息（外部工具 → 规划进程）。
//!
//! CLI（`ffctl planner goal X Y Z`）或其它工具把目标点发布到 `Firefly/Goal`，
//! 规划进程订阅后经 `PlannerManager::set_goal` 动态重目标（重算全局路径 +
//! 重新规划），无人机即飞往该点。`#[repr(C)]` 定长零拷贝：发布端 `apps/ffctl`、
//! 订阅端 `apps/planner`（都在 Rust 侧）。

use iceoryx2::prelude::*;

/// 目标话题（外部工具 → 规划进程）。
pub const GOAL_TOPIC: &str = "Firefly/Goal";

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
        }
    }
}
