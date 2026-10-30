//! Complete regional occupancy snapshots from committed submaps.
use super::config::invalid;
use firefly_error::Result;
use firefly_pubsub::map::{MAP_REGION_TOPIC, MapElement, MapRegion};
use firefly_pubsub::subscriber::Subscriber;
use std::sync::{Arc, Mutex};

pub struct Snapshot {
    pub graph_revision: u64,
    pub cells: Vec<MapElement>,
    pub visited_submaps: usize,
    pub visited_octants: usize,
}
mod region;
pub use region::RegionIndex;

#[derive(Default)]
pub(super) struct Exchange {
    pub pending: Option<Snapshot>,
    pub recycled: Option<Vec<MapElement>>,
    pub region: Option<MapRegion>,
    pub metadata: (u64, MapElement),
}
pub(super) type Mailbox = Arc<Mutex<Exchange>>;

/// IPC serialization runs outside sensor reception and map integration.
pub(super) struct PublisherWorker {
    mailbox: Mailbox,
    stop: Arc<std::sync::atomic::AtomicBool>,
    worker: Option<std::thread::JoinHandle<Result<()>>>,
}
impl PublisherWorker {
    pub fn start(
        mailbox: Mailbox,
        process_stop: Arc<std::sync::atomic::AtomicBool>,
    ) -> Result<Self> {
        use std::sync::atomic::{AtomicBool, Ordering};
        let stop = Arc::new(AtomicBool::new(false));
        let shared = mailbox.clone();
        let stopping = stop.clone();
        let (ready, startup) = std::sync::mpsc::sync_channel(1);
        let worker = std::thread::Builder::new()
            .name("okvis-map-publisher".into())
            .spawn(move || {
                let result = (|| -> Result<()> {
                    let node = firefly_pubsub::node::create_node()?;
                    let publisher =
                        firefly_pubsub::publish::Publisher::<[MapElement]>::with_slice_topic(
                            &node,
                            firefly_pubsub::map::MAP_TOPIC,
                        )?;
                    let regions = Subscriber::<MapRegion>::with_topic(&node, MAP_REGION_TOPIC)?;
                    let _ = ready.send(Ok(()));
                    let mut sequence = 0_u64;
                    while !stopping.load(Ordering::Acquire) && !process_stop.load(Ordering::Acquire)
                    {
                        if node.wait(std::time::Duration::from_millis(5)).is_err() {
                            process_stop.store(true, Ordering::Release);
                            break;
                        }
                        let (revision, header, snapshot) = {
                            let mut exchange =
                                shared.lock().map_err(|_| invalid("map mailbox poisoned"))?;
                            while let Some(region) = regions.receive()? {
                                if !region.valid() {
                                    return Err(invalid("invalid planning map region"));
                                }
                                exchange.region = Some(*region);
                            }
                            let (revision, header) = exchange.metadata;
                            let snapshot = if exchange
                                .pending
                                .as_ref()
                                .is_some_and(|s| s.graph_revision <= revision)
                            {
                                exchange.pending.take()
                            } else {
                                None
                            };
                            (revision, header, snapshot)
                        };
                        if let Some(mut snapshot) = snapshot
                            && snapshot.graph_revision == revision
                        {
                            sequence = sequence
                                .checked_add(1)
                                .ok_or_else(|| invalid("map sequence exhausted"))?;
                            let timestamp = snapshot.cells[0].timestamp;
                            let min = snapshot.cells[0].min;
                            let max = snapshot.cells[0].max;
                            snapshot.cells[0] = MapElement {
                                timestamp,
                                sequence,
                                min,
                                max,
                                ..header
                            };
                            publisher
                                .publish_slice_with(snapshot.cells.len(), |i| snapshot.cells[i])?;
                            snapshot.cells.clear();
                            shared
                                .lock()
                                .map_err(|_| invalid("map mailbox poisoned"))?
                                .recycled = Some(snapshot.cells);
                        }
                    }
                    Ok(())
                })();
                if let Err(error) = &result {
                    let _ = ready.send(Err(error.to_string()));
                    process_stop.store(true, Ordering::Release);
                }
                result
            })
            .map_err(invalid)?;
        let mut this = Self {
            mailbox,
            stop,
            worker: Some(worker),
        };
        if let Err(error) = startup
            .recv()
            .map_err(|error| error.to_string())
            .and_then(|result| result)
        {
            this.finish()?;
            return Err(invalid(error));
        }
        Ok(this)
    }
    pub fn update(&self, output: &super::output::Output) -> Result<()> {
        let mut header = MapElement::default();
        output.stamp_map(&mut header, 0);
        self.mailbox
            .lock()
            .map_err(|_| invalid("map mailbox poisoned"))?
            .metadata = (output.graph_revision(), header);
        Ok(())
    }
    pub fn finish(&mut self) -> Result<()> {
        self.stop.store(true, std::sync::atomic::Ordering::Release);
        if let Some(worker) = self.worker.take() {
            worker
                .join()
                .map_err(|_| invalid("map publisher panicked"))??;
        }
        Ok(())
    }
}
impl Drop for PublisherWorker {
    fn drop(&mut self) {
        if let Err(error) = self.finish() {
            log::error!("map publisher shutdown: {error}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapping::submapping_interface::{
        DepthIntegration, PinholeSensor, SupereightBackend,
    };
    use crate::{
        kinematics::transformation::Transformation, mapping::object_mapping::OccupancyMapBackend,
    };
    use firefly_supereight2::{data::DataConfig, map::MapConfig};
    use jiff::Timestamp;
    use nalgebra::Vector3;
    use nalgebra::{Matrix4, Quaternion};
    use purecv::core::Matrix;
    use std::collections::BTreeMap;

    #[test]
    #[ignore = "requires exclusive MapRegion/OccupancyMap IPC topics"]
    fn region_request_and_snapshot_metadata_roundtrip() {
        use firefly_pubsub::{node::create_node, publish::Publisher};
        use std::time::{Duration, Instant};
        let node = create_node().unwrap();
        let mailbox = Mailbox::default();
        let mut worker = PublisherWorker::start(mailbox.clone(), Arc::default()).unwrap();
        let requests = Publisher::<MapRegion>::with_topic(&node, MAP_REGION_TOPIC).unwrap();
        let maps =
            Subscriber::<[MapElement]>::with_slice_topic(&node, firefly_pubsub::map::MAP_TOPIC)
                .unwrap();
        let region = MapRegion {
            min: [-2.; 3],
            max: [3.; 3],
        };
        requests.publish(region).unwrap();
        let start = Instant::now();
        while mailbox.lock().unwrap().region != Some(region) {
            assert!(start.elapsed() < Duration::from_secs(2));
            std::thread::sleep(Duration::from_millis(5));
        }
        {
            let mut exchange = mailbox.lock().unwrap();
            exchange.metadata = (
                4,
                MapElement {
                    session: 7,
                    revision: 9,
                    rotation: [0., 0., 0., 1.],
                    ..Default::default()
                },
            );
            exchange.pending = Some(Snapshot {
                graph_revision: 5,
                cells: vec![MapElement {
                    timestamp: 2.,
                    min: region.min,
                    max: region.max,
                    ..Default::default()
                }],
                visited_submaps: 0,
                visited_octants: 0,
            });
        }
        std::thread::sleep(Duration::from_millis(20));
        assert!(maps.receive().unwrap().is_none());
        // A future map cannot be published under the previous navigation correction.
        {
            let mut exchange = mailbox.lock().unwrap();
            exchange.metadata.0 = 5;
            exchange.metadata.1.revision = 10;
        }
        let start = Instant::now();
        let sample = loop {
            if let Some(sample) = maps.receive().unwrap() {
                break sample;
            }
            assert!(start.elapsed() < Duration::from_secs(2));
            std::thread::sleep(Duration::from_millis(5));
        };
        assert_eq!(sample[0].session, 7);
        assert_eq!(sample[0].revision, 10);
        assert_eq!(sample[0].sequence, 1);
        assert_eq!(sample[0].timestamp, 2.);
        assert_eq!(sample[0].min, region.min);
        assert_eq!(sample[0].max, region.max);
        worker.finish().unwrap();
    }

    #[test]
    fn recycled_export_buffer_clears_geometry_when_region_becomes_empty() {
        let (backend, poses) = fixture();
        let mut index = RegionIndex::default();
        index.update(&[1, 2], &poses, &backend).unwrap();
        let header = MapElement {
            min: [-16.; 3],
            max: [16.; 3],
            rotation: [0., 0., 0., 1.],
            ..Default::default()
        };
        let time = Timestamp::from_second(2).unwrap();
        let first = index.snapshot(0, time, &poses, &backend, header).unwrap();
        assert!(first.cells.len() > 1);
        let allocation = first.cells.as_ptr();
        let empty = index
            .snapshot_with_buffer(
                0,
                time,
                &poses,
                &backend,
                MapElement {
                    min: [100.; 3],
                    max: [101.; 3],
                    ..header
                },
                first.cells,
            )
            .unwrap();
        assert_eq!(empty.cells.len(), 1);
        assert_eq!(empty.cells.as_ptr(), allocation);
        let restored = index
            .snapshot_with_buffer(0, time, &poses, &backend, header, empty.cells)
            .unwrap();
        assert!(restored.cells.len() > 1);
        assert_eq!(restored.cells.as_ptr(), allocation);
    }

    fn fixture() -> (OccupancyMapBackend, BTreeMap<u64, Transformation>) {
        let mut backend = OccupancyMapBackend::new(
            MapConfig {
                dim: Vector3::repeat(8.),
                res: 0.25,
                t_mw: nalgebra::IsometryMatrix3::translation(4., 4., 4.),
            },
            DataConfig::default(),
        );
        let sensor = PinholeSensor {
            fx: 64.,
            fy: 63.,
            cx: 31.5,
            cy: 23.5,
            width: 64,
            height: 48,
            near: 0.1,
            far: 4.,
            t_sc: Transformation::identity(),
        };
        let depth = Matrix::from_vec(48, 64, 1, vec![2_f32; 64 * 48]);
        for id in [1, 2] {
            backend.create_map(id).unwrap();
            for counter in 0..5 {
                backend
                    .integrate_depth(
                        id,
                        counter,
                        DepthIntegration {
                            depth: &depth,
                            sigma: None,
                            sensor: &sensor,
                            t_md: Matrix4::identity(),
                            colour: None,
                            segments: None,
                        },
                    )
                    .unwrap();
            }
        }
        let poses = BTreeMap::from([
            (1, Transformation::identity()),
            (2, Transformation::identity()),
        ]);
        (backend, poses)
    }

    #[test]
    fn occupancy_pruning_preserves_every_obstacle_from_exhaustive_leaves() {
        use firefly_supereight2::{
            iterator::{OctreeIterator, Selection},
            octant::Octant,
        };
        let (mut backend, mut poses) = fixture();
        // A plane beyond this submap's root produces a fully free observed subtree.
        backend.create_map(3).unwrap();
        poses.insert(3, Transformation::identity());
        let depth = Matrix::from_vec(48, 64, 1, vec![6_f32; 64 * 48]);
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
        backend
            .integrate_depth(
                3,
                0,
                DepthIntegration {
                    depth: &depth,
                    sigma: None,
                    sensor: &sensor,
                    t_md: Matrix4::identity(),
                    colour: None,
                    segments: None,
                },
            )
            .unwrap();
        let mut expected = 0;
        let mut free = 0;
        let mut octants = 0;
        for handle in backend.maps.values() {
            let map = handle.read().unwrap();
            octants += OctreeIterator::new(&map.octree, Selection::All).count();
            for id in OctreeIterator::new(&map.octree, Selection::Leaves) {
                let octant = map.octree.get(id).unwrap();
                let singleton = [octant.data()];
                let data = match octant {
                    Octant::Block(block) => block.data_at_scale(block.current_scale).unwrap(),
                    Octant::Node(_) => &singleton,
                };
                for value in data.iter().filter(|v| v.valid()) {
                    if value.is_inside() {
                        expected += 1;
                    } else {
                        free += 1;
                    }
                }
            }
        }
        assert!(expected > 0 && free > 0);
        let mut index = RegionIndex::default();
        index.update(&[1, 2, 3], &poses, &backend).unwrap();
        let result = index
            .snapshot(
                0,
                Timestamp::from_second(2).unwrap(),
                &poses,
                &backend,
                MapElement {
                    min: [-64.; 3],
                    max: [64.; 3],
                    rotation: [0., 0., 0., 1.],
                    ..Default::default()
                },
            )
            .unwrap();
        assert_eq!(result.cells.len(), expected + 1);
        assert!(result.cells[1..].iter().all(|cell| cell.kind == 1));
        assert!(result.visited_octants < octants);
    }

    #[test]
    fn loop_revision_moves_each_submap_and_preserves_fusion_time() {
        let (backend, mut poses) = fixture();
        let time = Timestamp::from_second(2).unwrap();
        let mut index = RegionIndex::default();
        index.update(&[1, 2], &poses, &backend).unwrap();
        let header = MapElement {
            min: [-8.; 3],
            max: [8.; 3],
            rotation: [0., 0., 0., 1.],
            ..Default::default()
        };
        let old = index.snapshot(0, time, &poses, &backend, header).unwrap();
        assert!(old.cells.iter().any(|cell| cell.kind == 1));
        assert!(old.cells[1..].iter().all(|cell| cell.kind == 1));
        poses.insert(
            2,
            Transformation::new(Vector3::new(3., -1., 0.), Quaternion::identity()),
        );
        index.update(&[2], &poses, &backend).unwrap();
        let corrected = index.snapshot(1, time, &poses, &backend, header).unwrap();
        assert_eq!(corrected.graph_revision, 1);
        assert_eq!(corrected.cells[0].timestamp, old.cells[0].timestamp);
        let half = (old.cells.len() - 1) / 2;
        assert_eq!(&corrected.cells[1..=half], &old.cells[1..=half]);
        for (before, after) in old.cells[half + 1..]
            .iter()
            .zip(&corrected.cells[half + 1..])
        {
            for (axis, shift) in [3., -1., 0.].into_iter().enumerate() {
                assert!((after.min[axis] - before.min[axis] - shift).abs() < 1e-12);
                assert!((after.max[axis] - before.max[axis] - shift).abs() < 1e-12);
            }
        }
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn regional_pruning_matches_full_geometry_after_motion_rotation_and_loop() {
        let (mut backend, mut poses) = fixture();
        backend.maps.insert(3, backend.maps[&1].clone());
        poses.insert(
            3,
            Transformation::new(Vector3::new(320., 0., 0.), Quaternion::identity()),
        );
        let time = Timestamp::from_second(2).unwrap();
        let mut index = RegionIndex::default();
        index.update(&[1, 2, 3], &poses, &backend).unwrap();
        let full_header = MapElement {
            min: [-64.; 3],
            max: [64.; 3],
            rotation: [0., 0., 0., 1.],
            ..Default::default()
        };
        for revision in 0..3 {
            if revision == 1 {
                // A loop moves this submap completely outside the requested neighbourhood.
                poses.insert(
                    2,
                    Transformation::new(Vector3::new(40., 0., 0.), Quaternion::identity()),
                );
                index.update(&[2], &poses, &backend).unwrap();
            }
            let full = index
                .snapshot(revision, time, &poses, &backend, full_header)
                .unwrap();
            let header = MapElement {
                min: [0., 0., 1.],
                max: [1., 2., 3.],
                rotation: if revision == 2 {
                    [
                        0.,
                        0.,
                        std::f64::consts::FRAC_1_SQRT_2,
                        std::f64::consts::FRAC_1_SQRT_2,
                    ]
                } else {
                    full_header.rotation
                },
                ..full_header
            };
            let local = index
                .snapshot(revision, time, &poses, &backend, header)
                .unwrap();
            let (min, max) = if revision == 2 {
                ([-2., 0., 1.], [0., 1., 3.])
            } else {
                (header.min, header.max)
            };
            let expected: Vec<_> = full
                .cells
                .iter()
                .skip(1)
                .filter(|cell| (0..3).all(|i| cell.min[i] < max[i] && cell.max[i] > min[i]))
                .copied()
                .collect();
            assert_ne!(expected.len(), 0);
            // Rounded quaternion rotation can conservatively include boundary-touching voxels.
            for cell in expected {
                assert!(local.cells.contains(&cell));
            }
            if revision != 2 {
                assert_eq!(
                    local.cells.len(),
                    full.cells
                        .iter()
                        .skip(1)
                        .filter(|cell| (0..3).all(|i| cell.min[i] < max[i] && cell.max[i] > min[i]))
                        .count()
                        + 1
                );
            }
            assert_eq!(local.visited_submaps, if revision == 0 { 2 } else { 1 });
            assert!(local.visited_octants < full.visited_octants);
            assert_eq!(local.cells[0].timestamp, 2.);
            assert_eq!(local.cells[0].min, header.min);
        }
        let empty = index
            .snapshot(
                3,
                time,
                &poses,
                &backend,
                MapElement {
                    min: [100.; 3],
                    max: [101.; 3],
                    ..full_header
                },
            )
            .unwrap();
        assert_eq!(empty.cells.len(), 1);
        assert_eq!(empty.visited_submaps, 0);
        assert_eq!(empty.visited_octants, 0);
        assert!(
            index
                .snapshot(
                    3,
                    time,
                    &poses,
                    &backend,
                    MapElement {
                        min: [-1e100; 3],
                        max: [1e100; 3],
                        ..full_header
                    }
                )
                .is_err()
        );
    }
}
