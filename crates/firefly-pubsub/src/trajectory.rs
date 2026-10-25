//! 规划器与独立轨迹输出进程之间的完整多项式、心跳和输出门控。
use iceoryx2::prelude::*;

pub const TRAJECTORY_TOPIC: &str = "Firefly/Trajectory";
pub const PLANNER_HEARTBEAT_TOPIC: &str = "Firefly/PlannerHeartbeat";
pub const TRAJECTORY_CONTROL_TOPIC: &str = "Firefly/TrajectoryControl";

/// 整条轨迹以 `[TrajectoryPieceMessage]` 单个原子样本发送，不限制段数。
/// 元数据在各段保持一致；系数按轴、升幂排列。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyTrajectoryPieceMessage")]
pub struct TrajectoryPieceMessage {
    pub start_time: f64,
    pub traj_id: u64,
    pub revision: u64,
    pub duration: f64,
    pub coefficients: [[f64; 6]; 3],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyPlannerHeartbeatMessage")]
pub struct PlannerHeartbeatMessage {
    pub sequence: u64,
}

/// 输出门控与地图坐标版本；参考仅由已接收轨迹产生。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, ZeroCopySend)]
#[type_name("FireflyTrajectoryControlMessage")]
pub struct TrajectoryControlMessage {
    pub revision: u64,
    pub enabled: u64,
}
