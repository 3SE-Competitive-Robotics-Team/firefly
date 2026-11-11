//! Sample committed position and yaw on one clock. Motion decisions belong to planning.
use firefly_trajectory::Trajectory;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct Reference {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
    pub yaw: f64,
    pub yaw_dot: f64,
}

#[derive(Clone)]
pub struct TrajectoryCommand {
    pub trajectory: Trajectory,
    pub yaw: Trajectory,
    pub start_time: f64,
    pub traj_id: u32,
}

pub struct TrajectoryServer {
    execution: Execution,
    heartbeat: Option<f64>,
    timeout: f64,
}

enum Execution {
    Idle,
    Sampling(TrajectoryCommand),
    Finishing(TrajectoryCommand),
}

impl TrajectoryServer {
    #[must_use]
    pub fn new(timeout: f64) -> Self {
        Self {
            execution: Execution::Idle,
            heartbeat: None,
            timeout,
        }
    }
    pub fn receive(&mut self, command: TrajectoryCommand) {
        self.execution = Execution::Sampling(command);
    }
    pub fn heartbeat(&mut self, now: f64) {
        self.heartbeat = Some(now);
    }
    pub fn invalidate(&mut self) {
        self.execution = Execution::Idle;
    }
    /// Retain the already committed stopping tail when further planning is unavailable.
    pub fn finish(&mut self) {
        self.execution = match std::mem::replace(&mut self.execution, Execution::Idle) {
            Execution::Idle => Execution::Idle,
            Execution::Sampling(command) | Execution::Finishing(command) => {
                Execution::Finishing(command)
            }
        };
    }

    #[must_use]
    pub fn sample(&mut self, now: f64, heartbeat_now: f64) -> Option<Reference> {
        let heartbeat = self.heartbeat?;
        if heartbeat_now - heartbeat > self.timeout {
            self.finish();
        }
        let (command, finishing) = match &self.execution {
            Execution::Idle => return None,
            Execution::Sampling(command) => (command, false),
            Execution::Finishing(command) => (command, true),
        };
        let elapsed = now - command.start_time;
        if elapsed < 0. || !elapsed.is_finite() {
            return None;
        }
        if finishing && elapsed >= command.trajectory.duration() {
            return None;
        }
        let t = elapsed.min(command.trajectory.duration());
        let position = command.trajectory.eval(t);
        let yaw = command.yaw.eval(t);
        Some(Reference {
            position: position.position,
            velocity: position.velocity,
            acceleration: position.acceleration,
            yaw: crate::motion::unwrap_near(0., yaw.position.x),
            yaw_dot: yaw.velocity.x,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
    fn command() -> TrajectoryCommand {
        let start = Endpoint {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(1., 0., 0.),
            ..start
        };
        let trajectory = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[], &[2.])
            .unwrap()
            .solve()
            .unwrap();
        let yaw = crate::motion::yaw_trajectory(&trajectory, Vector3::zeros(), Some(1.)).unwrap();
        TrajectoryCommand {
            trajectory,
            yaw,
            start_time: 1.,
            traj_id: 1,
        }
    }
    #[test]
    fn samples_position_and_heading_at_the_same_time() {
        let command = command();
        let mut server = TrajectoryServer::new(0.5);
        assert!(server.sample(1., 0.).is_none());
        server.heartbeat(0.);
        assert!(server.sample(1., 0.).is_none());
        server.receive(command.clone());
        assert!(server.sample(0.9, 0.).is_none());
        let value = server.sample(2., 0.1).unwrap();
        assert!((value.position - command.trajectory.eval(1.).position).norm() < 1e-12);
        assert!((value.yaw - 0.5).abs() < 1e-12);
        assert!((value.yaw_dot - 0.9375).abs() < 1e-12);
    }
    #[test]
    fn lost_heartbeat_finishes_committed_stop_and_requires_new_command() {
        let mut server = TrajectoryServer::new(0.5);
        server.heartbeat(0.);
        server.receive(command());
        assert!(server.sample(2., 0.6).unwrap().velocity.x > 0.);
        server.heartbeat(0.7);
        assert!(server.sample(3., 0.7).is_none());
        server.receive(command());
        let hold = server.sample(3., 0.7).unwrap();
        assert!(hold.velocity.norm() < 1e-12);
    }

    #[test]
    fn explicit_finish_invalidation_and_new_command_preserve_execution_boundaries() {
        let mut server = TrajectoryServer::new(0.5);
        server.finish();
        server.heartbeat(0.);
        server.receive(command());
        assert!(server.sample(3., 0.1).is_some());

        server.finish();
        server.finish();
        assert!(server.sample(2., 0.1).is_some());
        assert!(server.sample(3., 0.1).is_none());
        server.heartbeat(0.2);
        assert!(server.sample(3., 0.2).is_none());

        server.invalidate();
        server.invalidate();
        assert!(server.sample(2., 0.2).is_none());
        server.receive(command());
        let hold = server.sample(4., 0.2).unwrap();
        assert!((hold.position - Vector3::x()).norm() < 1e-12);
        assert!(hold.velocity.norm() < 1e-12);
    }
}
