//! Full observed-map export versus indexed occupied-only export, including real IPC and grid derivation.
#[path = "../../okvis2-x/benches/common/mod.rs"]
mod common;
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
    map::{MapElement, MapRegion},
    navigation::NavigationMessage,
    publish::Publisher,
    subscriber::Subscriber,
};
use firefly_supereight2::{
    data::DataConfig,
    iterator::{OctreeIterator, Selection},
    map::MapConfig,
    octant::Octant,
};
use jiff::Timestamp;
use nalgebra::{Isometry3, Vector3};
use purecv::core::Matrix;
use std::{
    collections::BTreeMap,
    time::{Duration, Instant},
};

#[allow(clippy::too_many_lines)]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let timed = std::env::args().any(|arg| arg == "--bench");
    let moving = std::env::args().any(|arg| arg == "--moving");
    let history: u64 = std::env::args()
        .find_map(|arg| arg.strip_prefix("--history=").map(str::to_owned))
        .map_or(Ok(0), |arg| arg.parse())?;
    let node = firefly_pubsub::node::create_node()?;
    firefly_observability::init();
    let ipc = firefly_observability::init_ipc(&node, "mapping-bench");
    let topic = format!("Firefly/MappingBench{}", std::process::id());
    let publisher = Publisher::<[MapElement]>::with_slice_topic(&node, &topic)?;
    let subscriber = Subscriber::<[MapElement]>::with_slice_topic(&node, &topic)?;
    let request_topic = format!("{topic}/Region");
    let requests = Publisher::<MapRegion>::with_topic(&node, &request_topic)?;
    let regions = Subscriber::<MapRegion>::with_topic(&node, &request_topic)?;
    let sensor = PinholeSensor {
        fx: 168.,
        fy: 168.,
        cx: 159.5,
        cy: 119.5,
        width: 320,
        height: 240,
        near: 0.2,
        far: 8.,
        t_sc: Transformation::identity(),
    };
    let image = Matrix::from_vec(240, 320, 1, vec![3_f32; 320 * 240]);
    let mut backend = OccupancyMapBackend::new(
        MapConfig {
            dim: Vector3::repeat(16.),
            res: 0.25,
            t_mw: nalgebra::IsometryMatrix3::translation(8., 8., 8.),
        },
        DataConfig::default(),
    );
    backend.create_map(1)?;
    let mut poses = BTreeMap::from([(1, Transformation::identity())]);
    for counter in 0..5 {
        integrate(
            &mut backend,
            counter,
            &image,
            &sensor,
            &Isometry3::identity(),
        )?;
    }
    // Frozen historical geometry is shared read-only; each submap has its own world pose.
    let frozen = backend.maps[&1].snapshot()?;
    for id in 2..history + 2 {
        backend.maps.insert(id, frozen.clone());
        poses.insert(
            id,
            Transformation::new(
                Vector3::new(id as f64 * 32., 0., 0.),
                nalgebra::Quaternion::identity(),
            ),
        );
    }
    let mut index = RegionIndex::default();
    index.update(&poses.keys().copied().collect::<Vec<_>>(), &poses, &backend)?;
    let mut full_grid = GridMapBuilder::new(0.1, [160, 160, 80])
        .with_origin(Vector3::new(-8., -8., -4.))
        .with_obstacles_inflation(0.4)
        .build()?;
    let mut local_grid = full_grid.clone();
    let mut full_input = map_input::MapInput::default();
    let mut local_input = map_input::MapInput::default();
    let mut local_buffer = Vec::new();
    let count: u32 = if timed { 60 } else { 2 };
    if timed {
        log::info!("begin history={history} moving={moving}");
        common::host_snapshot("idle_start");
        std::thread::sleep(Duration::from_secs(2));
        common::host_snapshot("idle_end");
    }
    for iteration in 0..count {
        let time = Timestamp::from_nanosecond(i128::from(
            1_000_000_000 + u64::from(iteration) * 50_000_000,
        ))?;
        let pose = Isometry3::translation(
            if moving {
                f64::from(iteration) * 0.05
            } else {
                0.
            },
            0.,
            0.,
        );
        let begin = Instant::now();
        integrate(
            &mut backend,
            u64::from(iteration) + 5,
            &image,
            &sensor,
            &pose,
        )?;
        let fusion = begin.elapsed();
        let nav = NavigationMessage {
            rotation: [0., 0., 0., 1.],
            odom: firefly_pubsub::odom::OdomMessage {
                timestamp: time.as_nanosecond() as f64 * 1e-9,
                position_x: pose.translation.x,
                is_initialized: true,
                ..Default::default()
            },
            ..Default::default()
        };
        let region = local_input.prepare_region(&mut local_grid, &nav, true)?;
        full_input.prepare_region(&mut full_grid, &nav, true)?;
        let header = MapElement {
            session: 1,
            sequence: u64::from(iteration) + 1,
            timestamp: nav.odom.timestamp,
            min: region.min,
            max: region.max,
            rotation: [0., 0., 0., 1.],
            ..Default::default()
        };
        let mut full = || -> Result<_, Box<dyn std::error::Error>> {
            let start = Instant::now();
            let cells = full_export(&poses, &backend, header)?;
            publisher.publish_slice_with(cells.len(), |i| cells[i])?;
            let sample = subscriber.receive()?.ok_or("full IPC missing")?;
            full_input.observe(&sample, Instant::now())?;
            assert!(full_input.ready(&nav, Instant::now(), Duration::from_secs(1)));
            full_input.apply(&mut full_grid, &nav, true)?;
            Ok((start.elapsed(), cells))
        };
        let mut local = || -> Result<_, Box<dyn std::error::Error>> {
            let start = Instant::now();
            requests.publish(region)?;
            let request = regions.receive()?.ok_or("region IPC missing")?;
            let header = MapElement {
                min: request.min,
                max: request.max,
                ..header
            };
            index.update(&[1], &poses, &backend)?;
            let snapshot = index.snapshot_with_buffer(
                0,
                time,
                &poses,
                &backend,
                header,
                std::mem::take(&mut local_buffer),
            )?;
            assert_eq!(snapshot.visited_submaps, 1);
            let cells = snapshot.cells;
            publisher.publish_slice_with(cells.len(), |i| cells[i])?;
            let sample = subscriber.receive()?.ok_or("local IPC missing")?;
            local_input.observe(&sample, Instant::now())?;
            assert!(local_input.ready(&nav, Instant::now(), Duration::from_secs(1)));
            local_input.apply(&mut local_grid, &nav, true)?;
            Ok((start.elapsed(), cells))
        };
        let (old, new) = if iteration % 2 == 0 {
            let old = full()?;
            (old, local()?)
        } else {
            let new = local()?;
            (full()?, new)
        };
        // Independent full traversal is the oracle; filtering happens outside the timed paths.
        let expected: Vec<_> = old
            .1
            .iter()
            .skip(1)
            .filter(|cell| cell.kind == 1)
            .filter(|cell| {
                (0..3).all(|i| cell.min[i] < region.max[i] && cell.max[i] > region.min[i])
            })
            .copied()
            .collect();
        assert_eq!(expected, new.1[1..]);
        assert!(local_grid.is_occupied(Vector3::new(pose.translation.x + 0.125, 0.125, 3.125)));
        if timed && iteration >= 10 {
            log::info!(
                "history={history} moving={moving} sample={iteration} fusion_ms={} full_ms={} local_ms={} old_total_ms={} new_total_ms={} full_cells={} local_cells={}",
                fusion.as_secs_f64() * 1e3,
                old.0.as_secs_f64() * 1e3,
                new.0.as_secs_f64() * 1e3,
                (fusion + old.0).as_secs_f64() * 1e3,
                (fusion + new.0).as_secs_f64() * 1e3,
                old.1.len(),
                new.1.len()
            );
            firefly_observability::pump_log_ipc(&ipc);
        }
        local_buffer = new.1;
    }
    if timed {
        common::host_snapshot("end");
        firefly_observability::pump_log_ipc(&ipc);
    }
    firefly_observability::flush();
    Ok(())
}

fn integrate(
    backend: &mut OccupancyMapBackend,
    counter: u64,
    image: &Matrix<f32>,
    sensor: &PinholeSensor,
    pose: &Isometry3<f64>,
) -> std::io::Result<()> {
    backend.integrate_depth(
        1,
        counter,
        DepthIntegration {
            depth: image,
            sigma: None,
            sensor,
            t_md: pose.to_homogeneous().cast(),
            colour: None,
            segments: None,
        },
    )
}

/// Full traversal reference for the pre-region transport; benchmark only.
fn full_export(
    poses: &BTreeMap<u64, Transformation>,
    backend: &OccupancyMapBackend,
    header: MapElement,
) -> firefly_error::Result<Vec<MapElement>> {
    let mut cells = vec![header];
    for (id, handle) in &backend.maps {
        let map = handle.read()?;
        let transform = poses[id].matrix() * map.t_wm.to_homogeneous().cast::<f64>();
        let rotation = transform.fixed_view::<3, 3>(0, 0);
        for id in OctreeIterator::new(&map.octree, Selection::Leaves) {
            let octant = map.octree.get(id)?;
            let singleton = [octant.data()];
            let (scale, data) = match octant {
                Octant::Block(block) => (
                    block.current_scale,
                    block.data_at_scale(block.current_scale)?,
                ),
                Octant::Node(node) => (node.base.scale(), singleton.as_slice()),
            };
            let stride = 1_i32 << scale;
            let n = (octant.base().size / stride) as usize;
            for (i, value) in data.iter().enumerate().filter(|(_, value)| value.valid()) {
                let offset = Vector3::new((i % n) as i32, (i / n % n) as i32, (i / (n * n)) as i32);
                let low = (octant.base().coord + offset * stride).cast::<f64>()
                    * f64::from(map.resolution);
                let half = Vector3::repeat(f64::from(stride) * f64::from(map.resolution) * 0.5);
                let center = rotation * (low + half) + transform.fixed_view::<3, 1>(0, 3);
                let extent = rotation.map(f64::abs) * half;
                cells.push(MapElement {
                    kind: if value.is_inside() { 1 } else { 2 },
                    min: (center - extent).into(),
                    max: (center + extent).into(),
                    ..Default::default()
                });
            }
        }
    }
    Ok(cells)
}
