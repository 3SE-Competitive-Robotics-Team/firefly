use super::*;
use crate::map_input::{Bounds, LocalSnapshot};
use firefly_pubsub::navigation::{NAVIGATION_TOPIC, NavigationMessage};
use firefly_pubsub::odom::{ODOM_TOPIC, OdomMessage};

fn publish_odometry(publisher: &Publisher<OdomMessage>, timestamp: f64, shift: f64) {
    publisher
        .publish(OdomMessage {
            timestamp,
            position_x: shift,
            is_initialized: true,
            ..Default::default()
        })
        .unwrap();
}

#[test]
#[ignore = "requires exclusive planner IPC topics"]
fn local_depth_source_reaches_planner_without_an_occupancy_publisher() {
    use firefly_pubsub::camera::{DEPTH_TOPIC, DepthImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH};
    let node = create_node().unwrap();
    let odometry = Publisher::<OdomMessage>::with_topic(&node, ODOM_TOPIC).unwrap();
    let depth = Publisher::<DepthImageMessage>::with_topic(&node, DEPTH_TOPIC).unwrap();
    let mut app = App::new(
        PlannerConfig {
            obstacle_clearance: 0.25,
            ..Default::default()
        },
        config::MappingOptions {
            resolution: 0.25,
            range: [2.; 3],
            depth: config::DepthOptions {
                translation: [0.; 3],
                rotation: [0., 0., 0., 1.],
                use_depth_filter: false,
                p_hit: 0.9,
                p_occ: 0.5,
                ..Default::default()
            },
        },
        ManagerOptions::default(),
        [0.; 3],
        None,
    )
    .unwrap();
    let correction = Publisher::<NavigationMessage>::with_topic(&node, NAVIGATION_TOPIC).unwrap();
    let epoch = Instant::now();
    for revision in [0, 1] {
        let started = Instant::now();
        let mut mapped = false;
        while started.elapsed() < Duration::from_secs(2) {
            let stamp = 1. + epoch.elapsed().as_secs_f64();
            publish_odometry(&odometry, stamp, 0.);
            if revision == 1 {
                correction
                    .publish(NavigationMessage {
                        odom: OdomMessage {
                            timestamp: stamp,
                            position_x: 100.,
                            is_initialized: true,
                            ..Default::default()
                        },
                        translation: [100., 0., 0.],
                        rotation: [0., 0., 0., 1.],
                        revision,
                        ..Default::default()
                    })
                    .unwrap();
            }
            depth
                .publish_with(|image| {
                    image.timestamp = stamp;
                    image.sensor_id = 0;
                    image.width = IMAGE_WIDTH as u32;
                    image.height = IMAGE_HEIGHT as u32;
                    image.data.fill(1.);
                })
                .unwrap();
            std::thread::sleep(Duration::from_millis(20));
            app.poll_sensors().unwrap();
            if let Some(nav) = app.state_input.odometry()
                && app.synchronize_map(&nav).unwrap()
                && app
                    .fsm
                    .manager()
                    .map()
                    .is_occupied_inflated(Vector3::new(0.125, 0.125, 1.125))
            {
                mapped = true;
                break;
            }
        }
        assert!(
            mapped,
            "Depth + Odometry must create the collision grid from its own depth fusion"
        );
    }
    // Continue valid odometry while withholding depth: the independently published map
    // cannot stay ready merely because execution callbacks are still running.
    let lost_depth = Instant::now();
    loop {
        assert!(
            lost_depth.elapsed() < Duration::from_secs(2),
            "stale depth must disable planning"
        );
        publish_odometry(&odometry, 1. + epoch.elapsed().as_secs_f64(), 0.);
        std::thread::sleep(Duration::from_millis(20));
        app.poll_odometry().unwrap();
        let nav = app.state_input.odometry().unwrap();
        if !app
            .map_input
            .ready(&nav, Instant::now(), Duration::from_secs(1))
        {
            break;
        }
    }
    drop(app); // Both receiving workers must join on ordinary teardown.
}

#[test]
fn dense_display_preserves_all_occupied_volume_within_message_budget() {
    let fine: Vec<_> = (-40..=40)
        .flat_map(|x| (-40..=40).flat_map(move |y| (-40..=40).map(move |z| [x, y, z])))
        .collect();
    let mut coarse = fine.clone();
    let size = App::coarsen_display_voxels(&mut coarse, [0.1; 3]);
    assert!(coarse.len() <= VOXELS_MAX);
    let scale = (size[0] / 0.1).round() as i32;
    assert!(scale > 1);
    for cell in fine {
        assert!(
            coarse
                .binary_search(&cell.map(|v| v.div_euclid(scale)))
                .is_ok()
        );
    }
}

#[test]
#[ignore = "requires exclusive planner IPC topics"]
fn safety_callback_applies_new_obstacles_and_deletions_without_exec_tick() {
    let node = create_node().unwrap();
    let odometry = Publisher::<OdomMessage>::with_topic(&node, ODOM_TOPIC).unwrap();
    let mut app = App::new(
        PlannerConfig::default(),
        config::MappingOptions {
            resolution: 0.5,
            range: [4.; 3],
            ..Default::default()
        },
        ManagerOptions::default(),
        [0.; 3],
        None,
    )
    .unwrap();
    publish_odometry(&odometry, 1., 0.);
    let start = Instant::now();
    while app.state_input.odometry().is_none() {
        assert!(start.elapsed() < Duration::from_secs(1));
        app.poll_odometry().unwrap();
        std::thread::sleep(Duration::from_millis(2));
    }
    let snapshot = |sequence, cells| LocalSnapshot {
        sequence,
        cells,
        timestamp: 1.,
        fused_at: Instant::now(),
        bounds: Bounds {
            min: [-5.; 3],
            max: [5.; 3],
        },
    };
    app.map_input.observe_local(snapshot(1, vec![])).unwrap();
    app.exec_step().unwrap();
    app.map_input
        .observe_local(snapshot(
            2,
            vec![Bounds {
                min: [1.; 3],
                max: [1.5; 3],
            }],
        ))
        .unwrap();
    app.safety_step().unwrap();
    let point = Vector3::repeat(1.25);
    assert!(
        app.fsm.manager().map().is_occupied(point),
        "safety must apply the accepted depth snapshot before checking collision"
    );
    app.map_input.observe_local(snapshot(3, vec![])).unwrap();
    app.safety_step().unwrap();
    assert!(!app.fsm.manager().map().is_occupied_inflated(point));

    let measured = app.state_input.current(Instant::now()).unwrap().state;
    app.fsm
        .set_goal(app.t_sim, Some(measured), Vector3::new(2., 0., 1.))
        .unwrap();
    for _ in 0..4 {
        let _ = app.fsm.exec_tick(app.t_sim, Some(measured));
    }
    assert!(app.fsm.manager().local().is_some());
    let mut uncovered = snapshot(4, vec![]);
    uncovered.bounds = Bounds {
        min: [-0.1; 3],
        max: [0.1; 3],
    };
    app.map_input.observe_local(uncovered).unwrap();
    app.safety_step().unwrap();
    assert!(
        !app.fsm.is_emergency(),
        "temporary coverage must not latch a timeout"
    );
    app.map_input.observe_local(snapshot(5, vec![])).unwrap();
    app.safety_step().unwrap();
    assert!(!app.fsm.is_emergency());
    let mut expired = snapshot(6, vec![]);
    expired.fused_at -= Duration::from_secs(2);
    app.map_input.observe_local(expired).unwrap();
    app.safety_step().unwrap();
    assert!(
        app.fsm.is_emergency(),
        "actual depth expiry still latches emergency stop"
    );
}
