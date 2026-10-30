use super::*;
use firefly_pubsub::navigation::{NAVIGATION_TOPIC, NavigationMessage};
use firefly_pubsub::odom::OdomMessage;

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
    let maps = Publisher::<[MapElement]>::with_slice_topic(&node, MAP_TOPIC).unwrap();
    let mut app = App::new(
        None,
        PlannerConfig::default(),
        config::MappingOptions {
            resolution: 0.5,
            range: [4.; 3],
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
    let mut header = MapElement {
        session: 1,
        sequence: 1,
        timestamp: 1.,
        min: [-5.; 3],
        max: [5.; 3],
        rotation: [0., 0., 0., 1.],
        ..Default::default()
    };
    maps.publish_slice_with(1, |_| header).unwrap();
    app.exec_step().unwrap();
    let obstacle = MapElement {
        kind: 1,
        min: [1.; 3],
        max: [1.5; 3],
        ..Default::default()
    };
    header.sequence = 2;
    maps.publish_slice_with(2, |i| if i == 0 { header } else { obstacle })
        .unwrap();
    app.safety_step().unwrap();
    let point = Vector3::repeat(1.25);
    assert!(
        app.fsm.manager().map().is_occupied(point),
        "safety must use the received snapshot, not the previous execution tick's grid"
    );
    header.sequence = 3;
    maps.publish_slice_with(1, |_| header).unwrap();
    app.safety_step().unwrap();
    assert!(!app.fsm.manager().map().is_occupied_inflated(point));
}
