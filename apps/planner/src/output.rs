//! 规划进程只发布完整轨迹与心跳；输出模式在独立进程中以 100Hz 采样。
use crate::odometry::OdometryReceiver;
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

fn decode(pieces: &[TrajectoryPieceMessage]) -> Result<TrajectoryCommand> {
    let first = pieces.first().ok_or_else(|| invalid("empty trajectory"))?;
    let traj_id = u32::try_from(first.traj_id).map_err(|_| invalid("invalid trajectory id"))?;
    if !first.start_time.is_finite()
        || !first.goal_timestamp.is_finite()
        || pieces.iter().any(|piece| {
            piece.start_time != first.start_time
                || piece.goal_timestamp != first.goal_timestamp
                || piece.traj_id != first.traj_id
        })
    {
        return Err(invalid("inconsistent trajectory metadata"));
    }
    let coefficients = DMatrix::from_fn(6 * pieces.len(), 3, |row, axis| {
        pieces[row / 6].coefficients[axis][row % 6]
    });
    let durations = DVector::from_iterator(pieces.len(), pieces.iter().map(|p| p.duration));
    let command = TrajectoryCommand {
        yaw: Trajectory::from_coefficients(
            DMatrix::from_fn(6 * pieces.len(), 3, |row, axis| {
                if axis == 0 {
                    pieces[row / 6].yaw_coefficients[row % 6]
                } else {
                    0.
                }
            }),
            durations.clone(),
        )?,
        trajectory: Trajectory::from_coefficients(coefficients, durations)?,
        start_time: first.start_time,
        traj_id,
    };
    let end = command.trajectory.eval(command.trajectory.duration());
    let yaw = command.yaw.eval(command.yaw.duration());
    if !command.trajectory.duration().is_finite()
        || end
            .position
            .iter()
            .chain(end.velocity.iter())
            .chain(end.acceleration.iter())
            .chain(yaw.position.iter())
            .chain(yaw.velocity.iter())
            .chain(yaw.acceleration.iter())
            .any(|v| !v.is_finite())
        || end.velocity.norm() > 1e-6
        || end.acceleration.norm() > 1e-6
        || yaw.velocity.norm() > 1e-6
        || yaw.acceleration.norm() > 1e-6
    {
        return Err(invalid("trajectory must finish at rest"));
    }
    for pair in pieces.windows(2) {
        for (left, right) in pair[0]
            .coefficients
            .iter()
            .chain([&pair[0].yaw_coefficients])
            .zip(
                pair[1]
                    .coefficients
                    .iter()
                    .chain([&pair[1].yaw_coefficients]),
            )
        {
            let t = pair[0].duration;
            let p = ((((left[5] * t + left[4]) * t + left[3]) * t + left[2]) * t + left[1]) * t
                + left[0];
            let v = (((5. * left[5] * t + 4. * left[4]) * t + 3. * left[3]) * t + 2. * left[2]) * t
                + left[1];
            let a = ((20. * left[5] * t + 12. * left[4]) * t + 6. * left[3]) * t + 2. * left[2];
            if [p - right[0], v - right[1], a - 2. * right[2]]
                .iter()
                .any(|d| !d.is_finite() || d.abs() > 1e-6)
            {
                return Err(invalid("trajectory pieces must be C2 continuous"));
            }
        }
    }
    Ok(command)
}

/// 独立轨迹输出进程入口，对应上游 `traj_server` 节点。
#[allow(clippy::too_many_lines)]
pub fn run(timeout: f64) -> Result<()> {
    let node = create_node()?;
    let publisher = Publisher::<ReferenceMessage>::with_topic(&node, REFERENCE_TOPIC)?;
    let trajectories =
        Subscriber::<[TrajectoryPieceMessage]>::with_slice_topic(&node, TRAJECTORY_TOPIC)?;
    let heartbeat =
        Subscriber::<PlannerHeartbeatMessage>::with_topic(&node, PLANNER_HEARTBEAT_TOPIC)?;
    let controls =
        Subscriber::<TrajectoryControlMessage>::with_topic(&node, TRAJECTORY_CONTROL_TOPIC)?;
    let odometry = OdometryReceiver::start()?;
    let log_ipc = firefly_observability::init_ipc(&node, "traj-server");
    let mut server = TrajectoryServer::new(timeout);
    let mut control = TrajectoryControlMessage::default();
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
        if odometry.terminated() {
            return CallbackProgression::Stop;
        }
        if !event.has_event_from(&tick) {
            return CallbackProgression::Continue;
        }
        let step = (|| -> Result<()> {
            while let Some(message) = controls.receive()? {
                control = *message;
            }
            while heartbeat.receive()?.is_some() {
                server.heartbeat(origin.elapsed().as_secs_f64());
            }
            while let Some(message) = trajectories.receive()? {
                if let Some(first) = message.first() {
                    server.receive(decode(&message)?);
                    goal_timestamp = first.goal_timestamp;
                }
            }
            let (mut input, _) = odometry.snapshot()?;
            let Some(snapshot) = input.current(Instant::now()) else {
                return Ok(());
            };
            if control.enabled == 0 {
                server.finish();
            }
            let references = server.sample(snapshot.timestamp, origin.elapsed().as_secs_f64());
            if let Some(reference) = references {
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
                    alignment_translation: [0.; 3],
                    alignment_rotation: [0., 0., 0., 1.],
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_discontinuities_nonfinite_evaluation_and_nonstopping_commands() {
        let hold = TrajectoryPieceMessage {
            start_time: 1.,
            goal_timestamp: 1.,
            traj_id: 1,
            duration: 1.,
            coefficients: [[0.; 6]; 3],
            yaw_coefficients: [0.; 6],
        };
        assert!(decode(&[hold, hold]).is_ok());
        let mut moving = hold;
        moving.coefficients[0][1] = 1.;
        assert!(decode(&[moving]).is_err());
        let mut discontinuous = hold;
        discontinuous.yaw_coefficients[0] = 1.;
        assert!(decode(&[hold, discontinuous]).is_err());
        let mut overflow = hold;
        overflow.duration = f64::MAX;
        overflow.coefficients[0][5] = 1.;
        assert!(decode(&[overflow]).is_err());
    }
}
