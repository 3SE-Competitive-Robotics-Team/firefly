//! 独立 release 输出进程的真实 IPC 验证，需要独占规划相关话题。
use firefly_pubsub::{
    navigation::{NAVIGATION_TOPIC, NavigationMessage},
    node::create_node,
    odom::OdomMessage,
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
#[ignore = "requires release planner and exclusive Navigation/OccupancyMap/TrajectoryControl IPC topics"]
fn shared_map_revision_and_fusion_freshness_gate_planning() {
    use firefly_pubsub::map::{MAP_REGION_TOPIC, MAP_TOPIC, MapElement, MapRegion};
    let node = create_node().unwrap();
    let navigation = Publisher::<NavigationMessage>::with_topic(&node, NAVIGATION_TOPIC).unwrap();
    let maps = Publisher::<[MapElement]>::with_slice_topic(&node, MAP_TOPIC).unwrap();
    let requests = Subscriber::<MapRegion>::with_topic(&node, MAP_REGION_TOPIC).unwrap();
    let mut region = MapRegion {
        min: [-100.; 3],
        max: [100.; 3],
    };
    let controls =
        Subscriber::<TrajectoryControlMessage>::with_topic(&node, TRAJECTORY_CONTROL_TOPIC)
            .unwrap();
    let trajectories =
        Subscriber::<[TrajectoryPieceMessage]>::with_slice_topic(&node, TRAJECTORY_TOPIC).unwrap();
    let binary =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/release/planner");
    let mut process = OutputProcess(
        Command::new(binary)
            .args(["--goal", "4", "0", "1"])
            .spawn()
            .unwrap(),
    );
    let mut timestamp = 1.0;
    let mut sequence = 0;
    // Fresh map, new Navigation awaiting its map revision, corrected map, then
    // repeated complete snapshots whose fusion timestamp never advances.
    for (revision, map_revision, fresh, expected) in [
        (0, 0, true, 1),
        (1, 0, true, 0),
        (1, 1, true, 1),
        (1, 1, false, 0),
    ] {
        let start = Instant::now();
        let frozen = timestamp;
        let mut matching = 0;
        let mut planned = false;
        while matching < 5 || (expected == 1 && !planned) {
            assert!(
                start.elapsed() < Duration::from_secs(4),
                "map gate did not settle to {expected}"
            );
            assert!(process.0.try_wait().unwrap().is_none());
            timestamp += 0.01;
            sequence += 1;
            while let Some(request) = requests.receive().unwrap() {
                region = *request;
            }
            navigation
                .publish(NavigationMessage {
                    odom: OdomMessage {
                        timestamp,
                        is_initialized: true,
                        ..Default::default()
                    },
                    revision,
                    rotation: [0., 0., 0., 1.],
                    ..Default::default()
                })
                .unwrap();
            maps.publish_slice_with(1, |_| MapElement {
                session: 1,
                sequence,
                revision: map_revision,
                timestamp: if fresh { timestamp } else { frozen },
                min: region.min,
                max: region.max,
                rotation: [0., 0., 0., 1.],
                ..Default::default()
            })
            .unwrap();
            while let Some(control) = controls.receive().unwrap() {
                if control.enabled == expected {
                    matching += 1;
                } else {
                    matching = 0;
                }
            }
            while let Some(trajectory) = trajectories.receive().unwrap() {
                if expected == 1 && trajectory[0].revision == revision {
                    assert!(trajectory[0].start_time <= timestamp);
                    assert!(timestamp - trajectory[0].start_time < 1.0);
                    planned = true;
                }
            }
            std::thread::sleep(Duration::from_millis(10));
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
#[ignore = "requires release planner and exclusive Navigation/PlannerStatus/TrajectoryControl IPC topics"]
fn planner_status_continues_during_map_warmup_and_coordinate_revision() {
    let node = create_node().unwrap();
    let navigation = Publisher::<NavigationMessage>::with_topic(&node, NAVIGATION_TOPIC).unwrap();
    let statuses =
        Subscriber::<PlannerStatusMessage>::with_topic(&node, PLANNER_STATUS_TOPIC).unwrap();
    let controls =
        Subscriber::<TrajectoryControlMessage>::with_topic(&node, TRAJECTORY_CONTROL_TOPIC)
            .unwrap();
    let binary =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/release/planner");
    let mut process = OutputProcess(Command::new(binary).spawn().unwrap());
    let mut stamp = 1.;
    for revision in [0, 1] {
        let start = Instant::now();
        let mut received = 0;
        let mut disabled = false;
        while received < 5 || !disabled {
            assert!(
                start.elapsed() < Duration::from_secs(3),
                "status stopped during map warmup, revision={revision}"
            );
            assert!(process.0.try_wait().unwrap().is_none());
            stamp += 0.01;
            navigation
                .publish(NavigationMessage {
                    odom: OdomMessage {
                        timestamp: stamp,
                        is_initialized: true,
                        ..OdomMessage::default()
                    },
                    revision,
                    rotation: [0., 0., 0., 1.],
                    ..NavigationMessage::default()
                })
                .unwrap();
            while let Some(status) = statuses.receive().unwrap() {
                assert_eq!(status.finished, 0);
                assert!(status.timestamp <= stamp && stamp - status.timestamp < 0.3);
                received += 1;
            }
            while let Some(control) = controls.receive().unwrap() {
                if control.revision == revision {
                    assert_eq!(control.enabled, 0, "depth-free map cannot enable output");
                    disabled = true;
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}

#[test]
#[ignore = "requires release planner and exclusive Navigation/Trajectory/Reference IPC topics"]
#[allow(clippy::too_many_lines)]
fn independent_output_preserves_coefficients_and_upstream_timeout_publish_order() {
    let node = create_node().unwrap();
    let navigation = Publisher::<NavigationMessage>::with_topic(&node, NAVIGATION_TOPIC).unwrap();
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
        navigation
            .publish(NavigationMessage {
                odom: OdomMessage {
                    timestamp: 1. + elapsed,
                    is_initialized: true,
                    ..OdomMessage::default()
                },
                rotation: [0., 0., 0., 1.],
                ..NavigationMessage::default()
            })
            .unwrap();
        control
            .publish(TrajectoryControlMessage {
                enabled: 1,
                ..TrajectoryControlMessage::default()
            })
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
                    revision: 0,
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
    // 新版本轨迹可以先于门控消息到达；匹配版本前静默，匹配后执行该轨迹。
    while references.receive().unwrap().is_some() {}
    trajectory
        .publish_slice_with(1, |_| TrajectoryPieceMessage {
            start_time: 1.,
            goal_timestamp: 124.,
            traj_id: 2,
            revision: 1,
            duration: 10.,
            coefficients: [
                [100., 2., 0., 0., 0., 0.],
                [2., 0., 0., 0., 0., 0.],
                [3., 0., 0., 0., 0., 0.],
            ],
        })
        .unwrap();
    let revision_start = Instant::now();
    loop {
        let now = origin.elapsed().as_secs_f64();
        navigation
            .publish(NavigationMessage {
                odom: OdomMessage {
                    timestamp: 1. + now,
                    is_initialized: true,
                    ..OdomMessage::default()
                },
                rotation: [0., 0., 0., 1.],
                revision: 1,
                ..NavigationMessage::default()
            })
            .unwrap();
        heartbeat
            .publish(PlannerHeartbeatMessage { sequence: 2 })
            .unwrap();
        if revision_start.elapsed() > Duration::from_millis(50) {
            control
                .publish(TrajectoryControlMessage {
                    revision: 1,
                    enabled: 1,
                })
                .unwrap();
            if let Some(reference) = references.receive().unwrap() {
                assert_eq!(reference.goal_timestamp, 124.);
                assert!(
                    (reference.position_x - (100. + 2. * (reference.timestamp - 1.))).abs() < 1e-10
                );
                break;
            }
        } else if revision_start.elapsed() > Duration::from_millis(20) {
            assert!(
                references.receive().unwrap().is_none(),
                "coordinate versions must match"
            );
        } else {
            while references.receive().unwrap().is_some() {}
        }
        assert!(revision_start.elapsed() < Duration::from_secs(1));
        std::thread::sleep(Duration::from_millis(5));
    }
}
