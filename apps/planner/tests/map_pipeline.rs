//! Real depth integration, regional export, IPC and planner-grid lifecycle.
#[path = "../src/map_input.rs"]
#[allow(dead_code)]
mod map_input;

use firefly_map::GridMapBuilder;
use firefly_okvis_x::{
    application::live::map_output::RegionIndex,
    kinematics::transformation::Transformation,
    mapping::{
        object_mapping::OccupancyMapBackend,
        submapping_interface::{DepthIntegration, PinholeSensor, SupereightBackend},
    },
};
use firefly_pubsub::{
    map::MapElement, navigation::NavigationMessage, odom::OdomMessage, publish::Publisher,
    subscriber::Subscriber,
};
use firefly_supereight2::{data::DataConfig, map::MapConfig};
use jiff::Timestamp;
use nalgebra::{Matrix4, Quaternion, Vector3};
use purecv::core::Matrix;
use std::{
    collections::BTreeMap,
    time::{Duration, Instant},
};

#[test]
#[allow(clippy::too_many_lines)]
fn fused_obstacles_survive_loop_and_window_roundtrip_then_clear_from_free_evidence() {
    let mut backend = OccupancyMapBackend::new(
        MapConfig {
            dim: Vector3::repeat(8.),
            res: 0.25,
            t_mw: nalgebra::IsometryMatrix3::translation(4., 4., 4.),
        },
        DataConfig::default(),
    );
    backend.create_map(1).unwrap();
    let sensor = PinholeSensor {
        fx: 64.,
        fy: 63.,
        cx: 31.5,
        cy: 23.5,
        width: 64,
        height: 48,
        near: 0.1,
        far: 8.,
        t_sc: Transformation::identity(),
    };
    let integrate = |backend: &mut OccupancyMapBackend, counter, depth| {
        let image = Matrix::from_vec(48, 64, 1, vec![depth; 64 * 48]);
        backend
            .integrate_depth(
                1,
                counter,
                DepthIntegration {
                    depth: &image,
                    sigma: None,
                    sensor: &sensor,
                    t_md: Matrix4::identity(),
                    colour: None,
                    segments: None,
                },
            )
            .unwrap();
    };
    for i in 0..5 {
        integrate(&mut backend, i, 2.);
    }
    let mut poses = BTreeMap::from([(1, Transformation::identity())]);
    let mut index = RegionIndex::default();
    index.update(&[1], &poses, &backend).unwrap();
    let mut grid = GridMapBuilder::new(0.25, [64; 3])
        .with_obstacles_inflation(0.25)
        .build()
        .unwrap();
    let mut input = map_input::MapInput::default();
    let mut nav = NavigationMessage {
        odom: OdomMessage {
            timestamp: 3.,
            is_initialized: true,
            ..Default::default()
        },
        rotation: [0., 0., 0., 1.],
        ..Default::default()
    };
    let node = firefly_pubsub::node::create_node().unwrap();
    let topic = format!("Firefly/MapPipeline{}", std::process::id());
    let publisher = Publisher::<[MapElement]>::with_slice_topic(&node, &topic).unwrap();
    let subscriber = Subscriber::<[MapElement]>::with_slice_topic(&node, &topic).unwrap();
    let now = Instant::now();
    let timeout = Duration::from_secs(1);
    let mut sequence = 0;
    let mut publish = |input: &mut map_input::MapInput,
                       grid: &mut firefly_map::GridMap,
                       nav: &NavigationMessage,
                       index: &RegionIndex,
                       poses: &BTreeMap<u64, Transformation>,
                       backend: &OccupancyMapBackend,
                       fused: i64| {
        let region = input.prepare_region(grid, nav, true).unwrap();
        sequence += 1;
        let header = MapElement {
            session: 1,
            sequence,
            revision: nav.revision,
            min: region.min,
            max: region.max,
            translation: nav.translation,
            rotation: nav.rotation,
            ..Default::default()
        };
        let snapshot = index
            .snapshot(
                nav.revision,
                Timestamp::from_second(fused).unwrap(),
                poses,
                backend,
                header,
            )
            .unwrap();
        publisher
            .publish_slice_with(snapshot.cells.len(), |i| snapshot.cells[i])
            .unwrap();
        let sample = subscriber.receive().unwrap().unwrap();
        input.observe(&sample, now).unwrap();
    };
    publish(&mut input, &mut grid, &nav, &index, &poses, &backend, 3);
    assert!(input.ready(&nav, now, timeout));
    input.apply(&mut grid, &nav, true).unwrap();
    let old = Vector3::new(0.125, 0.125, 2.125);
    assert!(grid.is_occupied(old));

    // A graph correction changes both map geometry and map<-odom atomically.
    nav.revision = 1;
    nav.translation = [3., 0., 0.];
    nav.odom.position_x = 3.;
    input.prepare_region(&mut grid, &nav, true).unwrap();
    assert!(!input.ready(&nav, now, timeout));
    poses.insert(
        1,
        Transformation::new(Vector3::new(3., 0., 0.), Quaternion::identity()),
    );
    index.update(&[1], &poses, &backend).unwrap();
    publish(&mut input, &mut grid, &nav, &index, &poses, &backend, 3);
    assert!(input.ready(&nav, now, timeout));
    input.apply(&mut grid, &nav, true).unwrap();
    let corrected = old + Vector3::new(3., 0., 0.);
    assert!(!grid.is_occupied_inflated(old));
    assert!(grid.is_occupied(corrected));

    nav.odom.position_x = 30.;
    input.prepare_region(&mut grid, &nav, true).unwrap();
    assert!(!input.ready(&nav, now, timeout));
    publish(&mut input, &mut grid, &nav, &index, &poses, &backend, 3);
    assert!(input.ready(&nav, now, timeout));
    input.apply(&mut grid, &nav, true).unwrap();
    nav.odom.position_x = 3.;
    input.prepare_region(&mut grid, &nav, true).unwrap();
    assert!(!input.ready(&nav, now, timeout));
    publish(&mut input, &mut grid, &nav, &index, &poses, &backend, 3);
    input.apply(&mut grid, &nav, true).unwrap();
    assert!(grid.is_occupied(corrected));
    assert!(!input.ready(&nav, now + Duration::from_secs(2), timeout));

    // Actual farther depth rays replace the near wall with free evidence.
    for i in 5..35 {
        integrate(&mut backend, i, 6.);
    }
    nav.odom.timestamp = 4.;
    publish(&mut input, &mut grid, &nav, &index, &poses, &backend, 4);
    assert!(input.ready(&nav, now, timeout));
    input.apply(&mut grid, &nav, true).unwrap();
    assert!(!grid.is_occupied_inflated(corrected));
}
