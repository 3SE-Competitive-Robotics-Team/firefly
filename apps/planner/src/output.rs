//! 规划进程只发布完整轨迹与心跳；输出模式在独立进程中以 100Hz 采样。
use crate::navigation::NavigationReceiver;
use firefly_error::{Error, ErrorKind, Result};
use firefly_planner::{TrajectoryCommand, TrajectoryServer};
use firefly_pubsub::{
    node::{IpcNode, create_node},
    publish::Publisher,
    reference::{REFERENCE_TOPIC, ReferenceMessage},
    subscriber::Subscriber,
    trajectory::{
        PLANNER_HEARTBEAT_TOPIC, PLANNER_STATUS_TOPIC, PlannerHeartbeatMessage,
        PlannerStatusMessage, TRAJECTORY_CONTROL_TOPIC, TRAJECTORY_TOPIC, TrajectoryControlMessage,
        TrajectoryPieceMessage,
    },
};
use firefly_trajectory::Trajectory;
use iceoryx2::prelude::*;
use nalgebra::{DMatrix, DVector};
use std::time::{Duration, Instant};

pub struct OutputPublisher {
    trajectory: Publisher<[TrajectoryPieceMessage]>,
    heartbeat: Publisher<PlannerHeartbeatMessage>,
    control: Publisher<TrajectoryControlMessage>,
    status: Publisher<PlannerStatusMessage>,
    pub goal_timestamp: f64,
    revision: Option<u64>,
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
            revision: None,
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
        })?;
        Ok(())
    }
    pub fn control(&mut self, revision: Option<u64>, enabled: bool) -> Result<()> {
        self.revision = revision;
        self.control.publish(TrajectoryControlMessage {
            revision: revision.unwrap_or_default(),
            enabled: u64::from(enabled && revision.is_some()),
        })?;
        Ok(())
    }
    pub fn trajectory(&self, command: &firefly_planner::LocalTraj) -> Result<()> {
        let Some(revision) = self.revision else {
            return Ok(());
        };
        self.trajectory
            .publish_slice_with(command.traj.pieces(), |piece| TrajectoryPieceMessage {
                start_time: command.start_time,
                goal_timestamp: self.goal_timestamp,
                traj_id: u64::from(command.traj_id),
                revision,
                duration: command.traj.durations()[piece],
                coefficients: std::array::from_fn(|axis| {
                    std::array::from_fn(|power| {
                        command.traj.coefficients()[(6 * piece + power, axis)]
                    })
                }),
            })?;
        Ok(())
    }
}

fn decode(pieces: &[TrajectoryPieceMessage]) -> Result<TrajectoryCommand> {
    let first = pieces.first().ok_or_else(|| invalid("empty trajectory"))?;
    let traj_id = u32::try_from(first.traj_id).map_err(|_| invalid("invalid trajectory id"))?;
    if !first.start_time.is_finite()
        || !first.goal_timestamp.is_finite()
        || pieces.iter().any(|piece| {
            piece.start_time != first.start_time
                || piece.goal_timestamp != first.goal_timestamp
                || piece.traj_id != first.traj_id
                || piece.revision != first.revision
        })
    {
        return Err(invalid("inconsistent trajectory metadata"));
    }
    let coefficients = DMatrix::from_fn(6 * pieces.len(), 3, |row, axis| {
        pieces[row / 6].coefficients[axis][row % 6]
    });
    let durations = DVector::from_iterator(pieces.len(), pieces.iter().map(|p| p.duration));
    Ok(TrajectoryCommand {
        trajectory: Trajectory::from_coefficients(coefficients, durations)?,
        start_time: first.start_time,
        traj_id,
    })
}

/// 独立轨迹输出进程入口，对应上游 `traj_server` 节点。
#[allow(clippy::too_many_lines)]
pub fn run(yaw_hold: bool, timeout: f64) -> Result<()> {
    let node = create_node()?;
    let publisher = Publisher::<ReferenceMessage>::with_topic(&node, REFERENCE_TOPIC)?;
    let trajectories =
        Subscriber::<[TrajectoryPieceMessage]>::with_slice_topic(&node, TRAJECTORY_TOPIC)?;
    let heartbeat =
        Subscriber::<PlannerHeartbeatMessage>::with_topic(&node, PLANNER_HEARTBEAT_TOPIC)?;
    let controls =
        Subscriber::<TrajectoryControlMessage>::with_topic(&node, TRAJECTORY_CONTROL_TOPIC)?;
    let navigation = NavigationReceiver::start(false)?;
    let log_ipc = firefly_observability::init_ipc(&node, "traj-server");
    let mut server = TrajectoryServer::new(yaw_hold, timeout);
    let mut control = TrajectoryControlMessage::default();
    let mut trajectory_revision = None;
    let mut goal_timestamp = 0.0;
    let origin = Instant::now();
    let waitset = WaitSetBuilder::new()
        .create::<ipc::Service>()
        .map_err(|e| internal(format!("create output waitset: {e:?}")))?;
    let tick = waitset
        .attach_interval(Duration::from_millis(10))
        .map_err(|e| internal(format!("attach output timer: {e:?}")))?;
    let mut error = None;
    log::info!("独立轨迹输出启动：100Hz，心跳超时 {timeout}s");
    let result = waitset.wait_and_process(|event: WaitSetAttachmentId<ipc::Service>| {
        if navigation.terminated() {
            return CallbackProgression::Stop;
        }
        if !event.has_event_from(&tick) {
            return CallbackProgression::Continue;
        }
        let step = (|| -> Result<()> {
            while let Some(message) = controls.receive()? {
                if control.revision != message.revision
                    && trajectory_revision != Some(message.revision)
                {
                    server.invalidate();
                }
                control = *message;
            }
            while heartbeat.receive()?.is_some() {
                server.heartbeat(origin.elapsed().as_secs_f64());
            }
            while let Some(message) = trajectories.receive()? {
                if let Some(first) = message.first()
                    && first.revision >= control.revision
                {
                    let revision = first.revision;
                    server.receive(decode(&message)?);
                    trajectory_revision = Some(revision);
                    goal_timestamp = first.goal_timestamp;
                }
            }
            let (mut input, _) = navigation.snapshot()?;
            let Some(snapshot) = input.current(Instant::now()) else {
                return Ok(());
            };
            let Some(transform) = input.navigation() else {
                return Ok(());
            };
            if control.enabled == 0 || transform.revision != control.revision {
                return Ok(());
            }
            let references = if trajectory_revision == Some(control.revision) {
                server.sample(snapshot.timestamp, origin.elapsed().as_secs_f64())
            } else {
                [None, None]
            };
            for reference in references.into_iter().flatten() {
                publisher.publish(ReferenceMessage {
                    timestamp: snapshot.timestamp,
                    position_x: reference.position.x,
                    position_y: reference.position.y,
                    position_z: reference.position.z,
                    velocity_x: reference.velocity.x,
                    velocity_y: reference.velocity.y,
                    velocity_z: reference.velocity.z,
                    acceleration_x: reference.acceleration.x,
                    acceleration_y: reference.acceleration.y,
                    acceleration_z: reference.acceleration.z,
                    yaw: reference.yaw,
                    yaw_dot: reference.yaw_dot,
                    alignment_translation: transform.translation,
                    alignment_rotation: transform.rotation,
                    has_alignment: 1,
                    goal_timestamp,
                })?;
            }
            firefly_observability::pump_log_ipc(&log_ipc);
            Ok(())
        })();
        if let Err(e) = step {
            error = Some(e);
            return CallbackProgression::Stop;
        }
        CallbackProgression::Continue
    });
    result.map_err(|e| internal(format!("output waitset: {e:?}")))?;
    if let Some(error) = error {
        return Err(error);
    }
    Ok(())
}
fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message.to_owned())
}
fn internal(message: String) -> Error {
    Error::new(ErrorKind::Internal, message).with_operation("trajectory output")
}
