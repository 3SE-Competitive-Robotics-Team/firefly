//! 独立轨迹输出线程；规划调用不占用 100Hz 指令输出节拍。
use crate::navigation::NavigationReader;
use firefly_error::{Error, ErrorKind, Result};
use firefly_planner::{Reference, TrajectoryCommand, TrajectoryServer};
use firefly_pubsub::{
    node::create_node,
    publish::Publisher,
    reference::{REFERENCE_TOPIC, ReferenceMessage},
};
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

#[derive(Default)]
struct Input {
    revision: Option<u64>,
    enabled: bool,
    heartbeat: Option<Instant>,
    command: Option<(u64, TrajectoryCommand)>,
    hold: Option<Reference>,
    error: Option<String>,
}

pub struct OutputWorker {
    input: Arc<Mutex<Input>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl OutputWorker {
    #[allow(clippy::too_many_lines)]
    pub fn start(reader: NavigationReader, yaw_hold: bool, timeout: f64) -> Result<Self> {
        let input = Arc::new(Mutex::new(Input::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let (state, stopping) = (Arc::clone(&input), Arc::clone(&stop));
        let (ready, startup) = std::sync::mpsc::sync_channel(1);
        let thread = std::thread::Builder::new()
            .name("planner-traj-server".into())
            .spawn(move || {
                let result = (|| -> Result<()> {
                    let node = create_node()?;
                    let publisher =
                        Publisher::<ReferenceMessage>::with_topic(&node, REFERENCE_TOPIC)?;
                    let origin = Instant::now();
                    let mut server = TrajectoryServer::new(yaw_hold, timeout);
                    let mut revision = None;
                    let _ = ready.send(Ok(()));
                    while !stopping.load(Ordering::Relaxed) {
                        let tick = Instant::now();
                        let (enabled, next_revision, heartbeat, command, hold) = {
                            let mut input = state
                                .lock()
                                .map_err(|_| internal("output state poisoned"))?;
                            (
                                input.enabled,
                                input.revision,
                                input.heartbeat,
                                input.command.take(),
                                input.hold,
                            )
                        };
                        if revision != next_revision {
                            server.invalidate();
                            revision = next_revision;
                        }
                        if let Some((command_revision, command)) = command
                            && Some(command_revision) == revision
                        {
                            server.receive(command);
                        }
                        if let Some(heartbeat) = heartbeat {
                            server.heartbeat(heartbeat.duration_since(origin).as_secs_f64());
                        }
                        let mut navigation = reader.snapshot()?;
                        if enabled
                            && let Some(snapshot) = navigation.current(tick)
                            && let Some(transform) = navigation.navigation()
                            && Some(transform.revision) == revision
                        {
                            let reference = hold.or_else(|| {
                                server.sample(
                                    snapshot.timestamp,
                                    tick.duration_since(origin).as_secs_f64(),
                                )
                            });
                            if let Some(reference) = reference {
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
                                })?;
                            }
                        }
                        std::thread::sleep(
                            Duration::from_millis(10).saturating_sub(tick.elapsed()),
                        );
                    }
                    Ok(())
                })();
                if let Err(error) = result {
                    let message = error.to_string();
                    let _ = ready.send(Err(message.clone()));
                    if let Ok(mut input) = state.lock() {
                        input.error = Some(message);
                    }
                }
            })
            .map_err(|error| internal(error.to_string()))?;
        let mut worker = Self {
            input,
            stop,
            thread: Some(thread),
        };
        if let Err(error) = startup
            .recv()
            .map_err(|error| error.to_string())
            .and_then(|result| result)
        {
            worker.shutdown();
            return Err(internal(error));
        }
        Ok(worker)
    }

    pub fn heartbeat(&self) -> Result<()> {
        let mut input = self
            .input
            .lock()
            .map_err(|_| internal("output state poisoned"))?;
        if let Some(error) = &input.error {
            return Err(internal(error.clone()));
        }
        input.heartbeat = Some(Instant::now());
        Ok(())
    }
    pub fn control(
        &self,
        revision: Option<u64>,
        enabled: bool,
        hold: Option<Reference>,
    ) -> Result<()> {
        let mut input = self
            .input
            .lock()
            .map_err(|_| internal("output state poisoned"))?;
        if input.revision != revision {
            input.command = None;
        }
        input.revision = revision;
        input.enabled = enabled;
        input.hold = hold;
        Ok(())
    }
    pub fn trajectory(&self, command: TrajectoryCommand) -> Result<()> {
        let mut input = self
            .input
            .lock()
            .map_err(|_| internal("output state poisoned"))?;
        if let Some(revision) = input.revision {
            input.command = Some((revision, command));
        }
        Ok(())
    }
    pub fn check(&self) -> Result<()> {
        let input = self
            .input
            .lock()
            .map_err(|_| internal("output state poisoned"))?;
        if let Some(error) = &input.error {
            return Err(internal(error.clone()));
        }
        Ok(())
    }
    fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            log::error!("trajectory output thread panicked");
        }
    }
}
impl Drop for OutputWorker {
    fn drop(&mut self) {
        self.shutdown();
    }
}
fn internal(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::Internal, message.into()).with_operation("trajectory output")
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
    use nalgebra::Vector3;

    #[test]
    fn revision_change_drops_queued_trajectory_instead_of_relabeling_it() {
        let worker = OutputWorker {
            input: Arc::new(Mutex::new(Input::default())),
            stop: Arc::new(AtomicBool::new(false)),
            thread: None,
        };
        let endpoint = Endpoint {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let trajectory = MincoBuilder::new(SolverOrder::MinimumJerk, endpoint, endpoint)
            .build(&[], &[1.0])
            .unwrap()
            .solve()
            .unwrap();
        let command = TrajectoryCommand {
            trajectory,
            start_time: 0.0,
            traj_id: 1,
        };
        worker.control(Some(1), true, None).unwrap();
        worker.trajectory(command.clone()).unwrap();
        assert_eq!(worker.input.lock().unwrap().command.as_ref().unwrap().0, 1);
        worker.control(Some(2), true, None).unwrap();
        assert!(worker.input.lock().unwrap().command.is_none());
        worker.trajectory(command).unwrap();
        assert_eq!(worker.input.lock().unwrap().command.as_ref().unwrap().0, 2);
    }
}
