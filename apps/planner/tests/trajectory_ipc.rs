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
use std::{
    process::{Child, Command},
    time::{Duration, Instant},
};

struct OutputProcess(Child);

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
        let _ = Command::new("kill")
            .args(["-INT", &self.0.id().to_string()])
            .status();
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
fn independent_output_preserves_coefficients_and_upstream_timeout_publish_order() {
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
    let mut first_reference = None;
    let mut timeout_pair = None;
    let mut previous: Option<ReferenceMessage> = None;
    let mut references_after_timeout = 0;
    let mut resumed = false;
    let mut quiet_after_heartbeat = false;
    let mut first_resumed_at = None;
    loop {
        assert!(
            origin.elapsed() < Duration::from_secs(6),
            "output IPC deadline"
        );
        assert!(
            process.0.try_wait().unwrap().is_none(),
            "output process exited"
        );
        let elapsed = origin.elapsed().as_secs_f64();
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
        let lost_for = first_reference.map_or(0., |t: f64| elapsed - t);
        if first_reference.is_none() || lost_for < 0.1 || lost_for > 0.9 {
            heartbeat
                .publish(PlannerHeartbeatMessage { sequence: 1 })
                .unwrap();
        }
        // 两段 p(t)=(t,2,3)，完整系数直接跨进程传输。
        if (first_reference.is_none() && elapsed > 0.15) || lost_for > 1.05 {
            trajectory
                .publish_slice_with(2, |piece| TrajectoryPieceMessage {
                    start_time: 1.,
                    goal_timestamp: 123.,
                    traj_id: 1,
                    duration: 10.,
                    coefficients: [
                        [piece as f64 * 10., 1., 0., 0., 0., 0.],
                        [2., 0., 0., 0., 0., 0.],
                        [3., 0., 0., 0., 0., 0.],
                    ],
                })
                .unwrap();
        }
        let mut received = false;
        while let Some(message) = references.receive().unwrap() {
            assert!(
                elapsed > 0.15,
                "control and heartbeat alone must not publish a reference"
            );
            received = true;
            if message.velocity_x == 0. {
                previous = Some(*message);
                continue;
            }
            assert!((message.position_x - (message.timestamp - 1.)).abs() < 1e-10);
            assert_eq!(message.position_y, 2.);
            assert_eq!(message.position_z, 3.);
            assert_eq!(message.velocity_x, 1.);
            assert_eq!(message.goal_timestamp, 123.);
            assert_eq!(message.acceleration_x, 0.);
            first_reference.get_or_insert(elapsed);
            if let Some(hold) = previous.take() {
                assert_eq!(
                    hold.timestamp, message.timestamp,
                    "hold must precede tracking in same callback"
                );
                timeout_pair = Some(elapsed);
            } else if timeout_pair.is_some() && lost_for < 1.05 {
                references_after_timeout += 1;
            }
            if lost_for > 1.05 {
                resumed = true;
                first_resumed_at = Some(elapsed);
            }
        }
        if lost_for > 0.95 && lost_for < 1.05 {
            assert!(
                !received,
                "heartbeats alone must not resume an expired trajectory"
            );
            quiet_after_heartbeat = true;
        }
        if resumed && first_resumed_at.is_some() {
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        timeout_pair.is_some(),
        "expected hold then tracking on timeout callback"
    );
    assert_eq!(
        references_after_timeout, 0,
        "output must stop after timeout callback"
    );
    assert!(quiet_after_heartbeat && resumed);
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
