//! 规划器与独立轨迹输出进程之间的完整多项式、心跳和输出门控。
use iceoryx2::prelude::*;

pub const TRAJECTORY_TOPIC: &str = "Firefly/Trajectory";
pub const PLANNER_HEARTBEAT_TOPIC: &str = "Firefly/PlannerHeartbeat";
pub const TRAJECTORY_CONTROL_TOPIC: &str = "Firefly/TrajectoryControl";
pub const PLANNER_STATUS_TOPIC: &str = "Firefly/PlannerStatus";
/// 外部工具向规划进程发送强制停止请求。
pub const MANDATORY_STOP_TOPIC: &str = "Firefly/MandatoryStop";

/// 收到即停；timestamp 为诊断用墙钟秒，不改变停止语义。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyMandatoryStop")]
pub struct MandatoryStopMessage {
    pub timestamp: f64,
}

impl Default for MandatoryStopMessage {
    fn default() -> Self {
        Self { timestamp: -1.0 }
    }
}

/// 目标执行快照；finished 只表示规划轨迹正常结束，不表示实测到点。
/// `goal_timestamp` 回显 `GoalMessage` 的发送时刻，作为同一会话内的目标身份。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, ZeroCopySend)]
#[type_name("FireflyPlannerStatusMessage")]
pub struct PlannerStatusMessage {
    pub timestamp: f64,
    pub goal_timestamp: f64,
    pub finished: u64,
}

/// 整条轨迹以 `[TrajectoryPieceMessage]` 单个原子样本发送，不限制段数。
/// 元数据在各段保持一致；系数按轴、升幂排列。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyTrajectoryPieceMessage")]
pub struct TrajectoryPieceMessage {
    pub start_time: f64,
    pub goal_timestamp: f64,
    pub traj_id: u64,
    pub duration: f64,
    pub coefficients: [[f64; 6]; 3],
    /// Yaw polynomial in radians, with the same start, duration and ascending powers.
    pub yaw_coefficients: [f64; 6],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyPlannerHeartbeatMessage")]
pub struct PlannerHeartbeatMessage {
    pub sequence: u64,
}

/// 输出门控；轨迹与参考均在局部 odom 坐标系。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, ZeroCopySend)]
#[type_name("FireflyTrajectoryControlMessage")]
pub struct TrajectoryControlMessage {
    pub enabled: u64,
}
