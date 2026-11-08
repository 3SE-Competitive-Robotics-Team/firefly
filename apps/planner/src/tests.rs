use super::*;
use crate::map_input::{Bounds, LocalSnapshot};
use firefly_pubsub::navigation::{NAVIGATION_TOPIC, NavigationMessage};
use firefly_pubsub::odom::OdomMessage;

#[test]
#[ignore = "requires exclusive planner IPC topics"]
fn local_depth_source_reaches_planner_without_an_occupancy_publisher() {
    use firefly_pubsub::camera::{DEPTH_TOPIC, DepthImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH};
    let node = create_node().unwrap();
    let navigation = Publisher::<NavigationMessage>::with_topic(&node, NAVIGATION_TOPIC).unwrap();
    let depth = Publisher::<DepthImageMessage>::with_topic(&node, DEPTH_TOPIC).unwrap();
    let mut app = App::new(
        None,
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
    let epoch = Instant::now();
    let mut mapped = false;
    while epoch.elapsed() < Duration::from_secs(2) {
        let stamp = 1. + epoch.elapsed().as_secs_f64();
        navigation
            .publish(NavigationMessage {
                odom: OdomMessage {
                    timestamp: stamp,
                    is_initialized: true,
                    ..Default::default()
                },
                rotation: [0., 0., 0., 1.],
                ..Default::default()
            })
            .unwrap();
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
        if let Some(nav) = app.state_input.navigation()
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
        "Depth + Navigation must create the collision grid from its own depth fusion"
    );
    // Continue valid odometry while withholding depth: the independently published map
    // cannot stay ready merely because execution callbacks are still running.
    let lost_depth = Instant::now();
    loop {
        assert!(
            lost_depth.elapsed() < Duration::from_secs(2),
            "stale depth must disable planning"
        );
        navigation
            .publish(NavigationMessage {
                odom: OdomMessage {
                    timestamp: 1. + epoch.elapsed().as_secs_f64(),
                    is_initialized: true,
                    ..Default::default()
                },
                rotation: [0., 0., 0., 1.],
                ..Default::default()
            })
            .unwrap();
        std::thread::sleep(Duration::from_millis(20));
        app.poll_navigation().unwrap();
        let nav = app.state_input.navigation().unwrap();
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
    let navigation = Publisher::<NavigationMessage>::with_topic(&node, NAVIGATION_TOPIC).unwrap();
    let mut app = App::new(
        None,
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
    navigation
        .publish(NavigationMessage {
            odom: OdomMessage {
                timestamp: 1.,
                is_initialized: true,
                ..Default::default()
            },
            rotation: [0., 0., 0., 1.],
            ..Default::default()
        })
        .unwrap();
    let start = Instant::now();
    while app.state_input.navigation().is_none() {
        assert!(start.elapsed() < Duration::from_secs(1));
        app.poll_navigation().unwrap();
        std::thread::sleep(Duration::from_millis(2));
    }
    let snapshot = |sequence, cells| LocalSnapshot {
        sequence,
        cells,
        revision: 0,
        timestamp: 1.,
        fused_at: Instant::now(),
        translation: [0.; 3],
        rotation: [0., 0., 0., 1.],
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
}
