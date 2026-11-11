//! 独立 release 输出进程的真实 IPC 验证，需要独占规划相关话题。
use firefly_pubsub::{
    navigation::{NAVIGATION_TOPIC, NavigationMessage},
    node::create_node,
    odom::{ODOM_TOPIC, OdomMessage},
    publish::Publisher,
    reference::{REFERENCE_TOPIC, ReferenceMessage},
    subscriber::Subscriber,
    trajectory::{
        PLANNER_HEARTBEAT_TOPIC, PLANNER_STATUS_TOPIC, PlannerHeartbeatMessage,
        PlannerStatusMessage, TRAJECTORY_CONTROL_TOPIC, TRAJECTORY_TOPIC, TrajectoryControlMessage,
        TrajectoryPieceMessage,
    },
};
use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
use nalgebra::{Point3, Vector3};
use std::{
    process::{Child, Command},
    time::{Duration, Instant},
};

struct OutputProcess(Child);

#[test]
#[ignore = "requires release planner/ffctl and exclusive odometry/depth/goal/trajectory IPC topics"]
fn cli_heading_only_goal_reaches_output_without_translating() {
    use firefly_pubsub::camera::{DEPTH_TOPIC, DepthImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH};
    let node = create_node().unwrap();
    let odometry = Publisher::<OdomMessage>::with_topic(&node, ODOM_TOPIC).unwrap();
    let depth = Publisher::<DepthImageMessage>::with_topic(&node, DEPTH_TOPIC).unwrap();
    let references = Subscriber::<ReferenceMessage>::with_topic(&node, REFERENCE_TOPIC).unwrap();
    let release = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/release");
    let mut planner = OutputProcess(
        Command::new(release.join("planner"))
            .args(["--goal", "0", "0", "1"])
            .spawn()
            .unwrap(),
    );
    let mut output = OutputProcess(
        Command::new(release.join("planner"))
            .arg("--traj-server")
            .spawn()
            .unwrap(),
    );
    let origin = Instant::now();
    let mut goal_command: Option<OutputProcess> = None;
    let mut saw_rotation = false;
    loop {
        let t = origin.elapsed().as_secs_f64();
        assert!(t < 6., "heading-only goal did not finish");
        assert!(planner.0.try_wait().unwrap().is_none());
        assert!(output.0.try_wait().unwrap().is_none());
        odometry
            .publish(OdomMessage {
                timestamp: 1. + t,
                position_z: 1.,
                is_initialized: true,
                ..Default::default()
            })
            .unwrap();
        depth
            .publish_with(|image| {
                image.timestamp = 1. + t;
                image.sensor_id = 0;
                image.width = IMAGE_WIDTH as u32;
                image.height = IMAGE_HEIGHT as u32;
                image.data.fill(4.);
            })
            .unwrap();
        if t > 0.5 && goal_command.is_none() {
            let command = Command::new(release.join("ffctl"))
                .args(["planner", "goal", "0", "0", "1", "--yaw", "90"])
                .spawn()
                .unwrap();
            goal_command = Some(OutputProcess(command));
        }
        if let Some(command) = &mut goal_command
            && let Some(status) = command.0.try_wait().unwrap()
        {
            assert!(status.success(), "goal CLI failed: {status}");
        }
        while let Some(reference) = references.receive().unwrap() {
            if reference.goal_timestamp == 0. {
                continue;
            }
            assert!(reference.position_x.abs() < 1e-6 && reference.position_y.abs() < 1e-6);
            assert!((reference.position_z - 1.).abs() < 1e-6);
            saw_rotation |= reference.yaw_dot > 0.1;
            if (reference.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-3
                && reference.yaw_dot.abs() < 1e-3
            {
                assert!(saw_rotation);
                return;
            }
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[test]
#[ignore = "requires release planner and exclusive odometry/navigation/depth/trajectory IPC topics"]
#[allow(clippy::too_many_lines)]
fn local_planning_uses_odometry_and_ignores_map_corrections() {
    use firefly_pubsub::camera::{DEPTH_TOPIC, DepthImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH};
    let node = create_node().unwrap();
    let odometry = Publisher::<OdomMessage>::with_topic(&node, ODOM_TOPIC).unwrap();
    let navigation = Publisher::<NavigationMessage>::with_topic(&node, NAVIGATION_TOPIC).unwrap();
    let depth = Publisher::<DepthImageMessage>::with_topic(&node, DEPTH_TOPIC).unwrap();
    let references = Subscriber::<ReferenceMessage>::with_topic(&node, REFERENCE_TOPIC).unwrap();
    let binary =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/release/planner");
    let mut planner = OutputProcess(
        Command::new(&binary)
            .args(["--goal", "3", "0", "1"])
            .spawn()
            .unwrap(),
    );
    let mut output = OutputProcess(Command::new(binary).arg("--traj-server").spawn().unwrap());
    let epoch = Instant::now();
    for phase in 0..2 {
        let started = Instant::now();
        let mut samples = 0;
        while samples < 10 {
            assert!(
                started.elapsed() < Duration::from_secs(5),
                "odom planning phase {phase} stalled"
            );
            assert!(planner.0.try_wait().unwrap().is_none());
            assert!(output.0.try_wait().unwrap().is_none());
            let timestamp = 1. + epoch.elapsed().as_secs_f64();
            odometry
                .publish(OdomMessage {
                    timestamp,
                    position_z: 1.,
                    is_initialized: true,
                    ..Default::default()
                })
                .unwrap();
            // No Navigation at startup. Later, unrelated map jumps must not move local references.
            if phase == 1 {
                navigation
                    .publish(NavigationMessage {
                        odom: OdomMessage {
                            timestamp,
                            position_x: 100.,
                            is_initialized: true,
                            ..Default::default()
                        },
                        translation: [100., 0., 0.],
                        rotation: [0., 0., 0., 1.],
                        revision: 7,
                        ..Default::default()
                    })
                    .unwrap();
            }
            depth
                .publish_with(|image| {
                    image.timestamp = timestamp;
                    image.sensor_id = 0;
                    image.width = IMAGE_WIDTH as u32;
                    image.height = IMAGE_HEIGHT as u32;
                    image.data.fill(4.);
                })
                .unwrap();
            while let Some(reference) = references.receive().unwrap() {
                assert_eq!(reference.alignment_translation, [0.; 3]);
                assert_eq!(reference.alignment_rotation, [0., 0., 0., 1.]);
                assert!(reference.position_x.abs() < 10.);
                samples += 1;
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }
}

impl Drop for OutputProcess {
    fn drop(&mut self) {
        if self.0.try_wait().unwrap().is_none() {
            let _ = Command::new("kill")
                .args(["-INT", &self.0.id().to_string()])
                .status();
        }
        let _ = self.0.wait();
    }
}

#[test]
#[ignore = "requires release planner and exclusive Odometry/PlannerStatus/TrajectoryControl IPC topics"]
fn planner_status_continues_during_map_warmup() {
    let node = create_node().unwrap();
    let odometry = Publisher::<OdomMessage>::with_topic(&node, ODOM_TOPIC).unwrap();
    let statuses =
        Subscriber::<PlannerStatusMessage>::with_topic(&node, PLANNER_STATUS_TOPIC).unwrap();
    let controls =
        Subscriber::<TrajectoryControlMessage>::with_topic(&node, TRAJECTORY_CONTROL_TOPIC)
            .unwrap();
    let binary =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/release/planner");
    let mut process = OutputProcess(Command::new(binary).spawn().unwrap());
    let mut stamp = 1.;
    for _ in 0..2 {
        let start = Instant::now();
        let mut received = 0;
        let mut disabled = false;
        while received < 5 || !disabled {
            assert!(
                start.elapsed() < Duration::from_secs(3),
                "status stopped during map warmup"
            );
            assert!(process.0.try_wait().unwrap().is_none());
            stamp += 0.01;
            odometry
                .publish(OdomMessage {
                    timestamp: stamp,
                    is_initialized: true,
                    ..Default::default()
                })
                .unwrap();
            while let Some(status) = statuses.receive().unwrap() {
                assert_eq!(status.finished, 0);
                assert!(status.timestamp <= stamp && stamp - status.timestamp < 0.3);
                received += 1;
            }
            while let Some(control) = controls.receive().unwrap() {
                assert_eq!(control.enabled, 0, "depth-free map cannot enable output");
                disabled = true;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}

#[test]
#[ignore = "requires release planner and exclusive Odometry/Trajectory/Reference IPC topics"]
#[allow(clippy::too_many_lines)]
fn independent_output_samples_position_yaw_and_finishes_stop_after_heartbeat_loss() {
    let node = create_node().unwrap();
    let odometry = Publisher::<OdomMessage>::with_topic(&node, ODOM_TOPIC).unwrap();
    let control =
        Publisher::<TrajectoryControlMessage>::with_topic(&node, TRAJECTORY_CONTROL_TOPIC).unwrap();
    let heartbeat =
        Publisher::<PlannerHeartbeatMessage>::with_topic(&node, PLANNER_HEARTBEAT_TOPIC).unwrap();
    let trajectory =
        Publisher::<[TrajectoryPieceMessage]>::with_slice_topic(&node, TRAJECTORY_TOPIC).unwrap();
    let references =
        Subscriber::<ReferenceMessage>::with_topic_and_buffer(&node, REFERENCE_TOPIC, 32).unwrap();
    let binary =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/release/planner");
    let mut process = OutputProcess(Command::new(binary).arg("--traj-server").spawn().unwrap());
    let origin = Instant::now();
    let start = Endpoint {
        position: Vector3::new(0., 2., 3.),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let end = Endpoint {
        position: Vector3::new(1., 2., 3.),
        ..start
    };
    let position = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&[Point3::new(0.5, 2., 3.)], &[1., 1.])
        .unwrap()
        .solve()
        .unwrap();
    let mut command_start = 1.;
    let mut first_reference = false;
    let mut continued_after_loss = false;
    let mut quiet_after_heartbeat = false;
    let mut resumed = false;
    loop {
        let elapsed = origin.elapsed().as_secs_f64();
        assert!(elapsed < 6., "output IPC deadline");
        assert!(
            process.0.try_wait().unwrap().is_none(),
            "output process exited"
        );
        odometry
            .publish(OdomMessage {
                timestamp: 1. + elapsed,
                is_initialized: true,
                ..Default::default()
            })
            .unwrap();
        control
            .publish(TrajectoryControlMessage { enabled: 1 })
            .unwrap();
        if elapsed < 0.5 || elapsed > 2.2 {
            heartbeat
                .publish(PlannerHeartbeatMessage { sequence: 1 })
                .unwrap();
        }
        let restart = elapsed > 2.6 && command_start == 1.;
        if restart {
            command_start = 1. + elapsed;
        }
        if (!first_reference && elapsed > 0.15) || restart {
            trajectory
                .publish_slice_with(2, |piece| TrajectoryPieceMessage {
                    start_time: command_start,
                    goal_timestamp: 123.,
                    traj_id: 1,
                    duration: 1.,
                    coefficients: std::array::from_fn(|axis| {
                        std::array::from_fn(|power| {
                            position.coefficients()[(piece * 6 + power, axis)]
                        })
                    }),
                    yaw_coefficients: std::array::from_fn(|power| {
                        position.coefficients()[(piece * 6 + power, 0)]
                    }),
                })
                .unwrap();
        }
        let mut received = false;
        while let Some(message) = references.receive().unwrap() {
            received = true;
            assert!(
                elapsed > 0.15,
                "heartbeat alone must not publish references"
            );
            let sample = position.eval((message.timestamp - command_start).clamp(0., 2.));
            assert!((message.position_x - sample.position.x).abs() < 1e-9);
            assert!((message.velocity_x - sample.velocity.x).abs() < 1e-9);
            assert!((message.acceleration_x - sample.acceleration.x).abs() < 1e-9);
            assert!((message.yaw - sample.position.x).abs() < 1e-9);
            assert!((message.yaw_dot - sample.velocity.x).abs() < 1e-9);
            assert!((message.position_y - 2.).abs() < 1e-9);
            assert!((message.position_z - 3.).abs() < 1e-9);
            assert!((message.goal_timestamp - 123.).abs() < 1e-9);
            first_reference = true;
            continued_after_loss |= elapsed > 1.2 && elapsed < 1.9;
            resumed |= command_start > 1.;
        }
        if elapsed > 2.3 && elapsed < 2.55 {
            assert!(
                !received,
                "heartbeat alone must not restart a completed stopping tail"
            );
            quiet_after_heartbeat = true;
        }
        if resumed {
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(first_reference && continued_after_loss && quiet_after_heartbeat && resumed);
    // A live global navigation stream cannot keep output alive after local odometry stops.
    let navigation = Publisher::<NavigationMessage>::with_topic(&node, NAVIGATION_TOPIC).unwrap();
    let stopped_at = Instant::now();
    let mut last_reference = Instant::now();
    while stopped_at.elapsed() < Duration::from_millis(900) {
        navigation
            .publish(NavigationMessage {
                odom: OdomMessage {
                    timestamp: 10. + stopped_at.elapsed().as_secs_f64(),
                    is_initialized: true,
                    ..Default::default()
                },
                rotation: [0., 0., 0., 1.],
                ..Default::default()
            })
            .unwrap();
        heartbeat
            .publish(PlannerHeartbeatMessage { sequence: 3 })
            .unwrap();
        while references.receive().unwrap().is_some() {
            last_reference = Instant::now();
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(last_reference.duration_since(stopped_at) < Duration::from_millis(650));
}
