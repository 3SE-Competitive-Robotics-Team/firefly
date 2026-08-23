//! 飞行目标消息（外部工具 → 规划进程）。
//!
//! CLI（`uv run firefly-goal X Y Z`）或其它工具把目标点发布到 `Firefly/Goal`，
//! 规划进程订阅后经 `PlannerManager::set_goal` 动态重目标（重算全局路径 +
//! 重新规划），无人机即飞往该点。`#[repr(C)]` 定长零拷贝，与 Python 侧
//! `firefly_mujoco.messages.GoalMessage` 布局/类型名严格一致。

use iceoryx2::prelude::*;

/// 目标话题（外部工具 → 规划进程）。
pub const GOAL_TOPIC: &str = "Firefly/Goal";

/// 飞行目标消息：目标点位置（地图系，米）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyGoalMessage")]
pub struct GoalMessage {
    /// 发送时刻（墙钟秒，仅诊断用；规划以自身 sim 时钟为准）。
    pub timestamp: f64,
    /// 目标位置 `p`（地图系，米）。
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
}

impl Default for GoalMessage {
    fn default() -> Self {
        Self {
            timestamp: -1.0,
            position_x: 0.0,
            position_y: 0.0,
            position_z: 0.0,
        }
    }
}
