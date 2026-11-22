use firefly_error::Result;
use firefly_pubsub::{
    node::IpcNode,
    publish::Publisher,
    trajectory::{
        PLANNER_HEARTBEAT_TOPIC, PLANNER_STATUS_TOPIC, PlannerHeartbeatMessage,
        PlannerStatusMessage, TRAJECTORY_CONTROL_TOPIC, TRAJECTORY_TOPIC, TrajectoryControlMessage,
        TrajectoryPieceMessage,
    },
};

pub struct OutputPublisher {
    trajectory: Publisher<[TrajectoryPieceMessage]>,
    heartbeat: Publisher<PlannerHeartbeatMessage>,
    control: Publisher<TrajectoryControlMessage>,
    status: Publisher<PlannerStatusMessage>,
    pub goal_timestamp: f64,
    pub passed: bool,
    enabled: bool,
    sequence: u64,
}
impl OutputPublisher {
    pub fn new(node: &IpcNode) -> Result<Self> {
        Ok(Self {
            trajectory: Publisher::with_slice_topic(node, TRAJECTORY_TOPIC)?,
            heartbeat: Publisher::with_topic(node, PLANNER_HEARTBEAT_TOPIC)?,
            control: Publisher::with_topic(node, TRAJECTORY_CONTROL_TOPIC)?,
            status: Publisher::with_topic(node, PLANNER_STATUS_TOPIC)?,
            goal_timestamp: 0.0,
            passed: false,
            enabled: false,
            sequence: 0,
        })
    }
    pub fn heartbeat(&mut self) -> Result<()> {
        self.sequence = self.sequence.wrapping_add(1);
        self.heartbeat.publish(PlannerHeartbeatMessage {
            sequence: self.sequence,
        })?;
        Ok(())
    }
    pub fn status(&self, timestamp: f64, finished: bool) -> Result<()> {
        self.status.publish(PlannerStatusMessage {
            timestamp,
            goal_timestamp: self.goal_timestamp,
            finished: u64::from(finished),
            passed: u64::from(self.passed),
        })?;
        Ok(())
    }
    pub fn control(&mut self, enabled: bool) -> Result<()> {
        self.enabled = enabled;
        self.control.publish(TrajectoryControlMessage {
            enabled: u64::from(enabled),
        })?;
        Ok(())
    }
    pub fn trajectory(&self, command: &firefly_planner::LocalTraj) -> Result<()> {
        if !self.enabled {
            return Ok(());
        }
        self.trajectory
            .publish_slice_with(command.traj.pieces(), |piece| TrajectoryPieceMessage {
                start_time: command.start_time,
                goal_timestamp: self.goal_timestamp,
                traj_id: u64::from(command.traj_id),
                duration: command.traj.durations()[piece],
                coefficients: std::array::from_fn(|axis| {
                    std::array::from_fn(|power| {
                        command.traj.coefficients()[(6 * piece + power, axis)]
                    })
                }),
                yaw_coefficients: std::array::from_fn(|power| {
                    command.yaw.coefficients()[(6 * piece + power, 0)]
                }),
            })?;
        Ok(())
    }
}
