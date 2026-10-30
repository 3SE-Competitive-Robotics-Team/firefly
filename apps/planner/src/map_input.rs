//! Derive a collision grid from a complete, versioned regional occupancy snapshot.
use firefly_base::{FrameId, RigidTransform};
use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{GridMap, VoxelState};
use firefly_pubsub::{
    map::{MAX_MAP_CELLS, MapElement, MapRegion},
    navigation::NavigationMessage,
};
use nalgebra::{Point3, Vector3};
use std::{
    collections::{HashMap, HashSet},
    time::{Duration, Instant},
};

#[derive(Default)]
pub struct MapInput {
    required: Option<MapRegion>,
    requested: Option<MapRegion>,
    cells: Vec<MapElement>,
    fused_at: Option<Instant>,
    fault: bool,
    occupied: HashMap<[i64; 3], u32>,
    boxes: HashSet<([i64; 3], [i64; 3])>,
    next_boxes: HashSet<([i64; 3], [i64; 3])>,
    applied: Option<(u64, u64, [i64; 3])>,
    geometry_version: u64,
}
impl MapInput {
    pub fn observe(&mut self, cells: &[MapElement], now: Instant) -> Result<()> {
        let result = self.accept(cells, now);
        self.fault |= result.is_err();
        result
    }
    fn accept(&mut self, cells: &[MapElement], now: Instant) -> Result<()> {
        let header = cells.first().ok_or_else(|| invalid("empty map packet"))?;
        if cells.len() > MAX_MAP_CELLS + 1
            || header.kind != 0
            || header.session == 0
            || !header.timestamp.is_finite()
            || header.timestamp < 0.0
            || !(MapRegion {
                min: header.min,
                max: header.max,
            })
            .valid()
        {
            return Err(invalid("invalid map header"));
        }
        source_alignment(header)?;
        for cell in &cells[1..] {
            if !(1..=2).contains(&cell.kind)
                || (0..3).any(|i| {
                    !cell.min[i].is_finite()
                        || !cell.max[i].is_finite()
                        || cell.min[i] >= cell.max[i]
                })
            {
                return Err(invalid("invalid map voxel bounds"));
            }
        }
        if let Some(old) = self.cells.first() {
            if header.session != old.session {
                return Err(invalid("map session changed; restart planner"));
            }
            if header.sequence <= old.sequence
                || header.revision < old.revision
                || header.timestamp < old.timestamp
            {
                return Ok(());
            }
            if header.timestamp > old.timestamp {
                self.fused_at = Some(now);
            }
            if header.revision == old.revision
                && (header.translation != old.translation || header.rotation != old.rotation)
            {
                return Err(invalid("unversioned source map alignment change"));
            }
        } else {
            self.fused_at = Some(now);
        }
        let cells_changed = self.cells.get(1..) != Some(&cells[1..]);
        if cells_changed
            || self.cells.first().is_none_or(|old| {
                old.translation != header.translation || old.rotation != header.rotation
            })
        {
            self.geometry_version = self.geometry_version.wrapping_add(1);
        }
        if cells_changed {
            self.cells.clear();
            self.cells.extend_from_slice(cells);
        } else {
            // Coverage and fusion time may advance without copying unchanged geometry.
            self.cells[0] = *header;
        }
        Ok(())
    }
    pub fn ready(&self, navigation: &NavigationMessage, now: Instant, timeout: Duration) -> bool {
        !self.fault
            && self.cells.first().is_some_and(|header| {
                self.required.is_some_and(|required| {
                    (MapRegion {
                        min: header.min,
                        max: header.max,
                    })
                    .contains(&required)
                }) && (navigation.external_map == 1
                    || (header.revision == navigation.revision
                        && header.translation == navigation.translation
                        && header.rotation == navigation.rotation))
                    && header.timestamp <= navigation.odom.timestamp
                    && navigation.odom.timestamp - header.timestamp <= timeout.as_secs_f64()
                    && self
                        .fused_at
                        .is_some_and(|received| now.saturating_duration_since(received) <= timeout)
            })
    }

    /// Move the planning window before requesting coverage; no uncovered grid may be queried.
    pub fn prepare_region(
        &mut self,
        map: &mut GridMap,
        navigation: &NavigationMessage,
        rolling: bool,
    ) -> Result<MapRegion> {
        if rolling {
            map.move_ring_buffer(Vector3::new(
                navigation.odom.position_x,
                navigation.odom.position_y,
                navigation.odom.position_z,
            ));
        }
        let low = map.origin();
        let high = low + Vector3::from(map.dims().map(|d| d as f64 * map.resolution()));
        let transform = navigation.alignment()?.isometry().inverse();
        let center = transform * Point3::from((low + high) * 0.5);
        let extent = transform
            .rotation
            .to_rotation_matrix()
            .matrix()
            .map(f64::abs)
            * ((high - low) * 0.5);
        let required = MapRegion {
            min: (center.coords - extent).into(),
            max: (center.coords + extent).into(),
        };
        self.required = Some(required);
        let prefetch = MapRegion {
            min: required.min.map(|v| v - 0.5),
            max: required.max.map(|v| v + 0.5),
        };
        if self
            .requested
            .is_none_or(|region| !region.contains(&prefetch))
        {
            // One metre of prefetch coverage avoids a new response for every voxel shift.
            self.requested = Some(MapRegion {
                min: required.min.map(|v| v - 1.),
                max: required.max.map(|v| v + 1.),
            });
        }
        Ok(self.requested.unwrap())
    }
    pub fn invalidate(&mut self) {
        self.applied = None;
        self.occupied.clear();
        self.boxes.clear();
    }

    pub fn observed_state(&self, global: [i64; 3]) -> VoxelState {
        if self.occupied.contains_key(&global) {
            VoxelState::Occupied
        } else {
            VoxelState::Unknown
        }
    }

    #[fastrace::trace]
    pub fn apply(
        &mut self,
        map: &mut GridMap,
        navigation: &NavigationMessage,
        rolling: bool,
    ) -> Result<()> {
        self.prepare_region(map, navigation, rolling)?;
        if !self.cells.first().is_some_and(|header| {
            (MapRegion {
                min: header.min,
                max: header.max,
            })
            .contains(&self.required.unwrap())
        }) {
            return Err(invalid("planning window exceeds map coverage"));
        }
        let header = self
            .cells
            .first()
            .ok_or_else(|| invalid("map snapshot absent"))?;
        let key = (
            self.geometry_version,
            navigation.revision,
            map.global_index([0; 3]),
        );
        if self.applied == Some(key) {
            return Ok(());
        }
        let transform =
            navigation.alignment()?.isometry() * source_alignment(header)?.isometry().inverse();
        let absolute_rotation = transform
            .rotation
            .to_rotation_matrix()
            .matrix()
            .map(f64::abs);
        let origin = map.origin();
        let resolution = map.resolution();
        let dims = map.dims();
        self.next_boxes.clear();
        let boxes = &mut self.next_boxes;
        for cell in self.cells.iter().skip(1).filter(|cell| cell.kind == 1) {
            let low = Vector3::from(cell.min);
            let high = Vector3::from(cell.max);
            let center = transform * Point3::from((low + high) * 0.5);
            let extent = absolute_rotation * ((high - low) * 0.5);
            let low = center.coords - extent;
            let high = center.coords + extent;
            let start: [usize; 3] = std::array::from_fn(|i| {
                ((low[i] - origin[i]) / resolution)
                    .floor()
                    .clamp(0., dims[i] as f64) as usize
            });
            let end: [usize; 3] = std::array::from_fn(|i| {
                ((high[i] - origin[i]) / resolution)
                    .ceil()
                    .clamp(0., dims[i] as f64) as usize
            });
            if (0..3).all(|i| start[i] < end[i]) {
                boxes.insert((map.global_index(start), map.global_index(end)));
            }
        }
        for bounds in self.boxes.difference(boxes) {
            for global in voxel_indices(*bounds) {
                let count = self
                    .occupied
                    .get_mut(&global)
                    .expect("source contribution exists");
                *count -= 1;
                if *count == 0 {
                    self.occupied.remove(&global);
                    if let Some(index) = map.window_index(global) {
                        map.set_state(index, VoxelState::Free);
                    }
                }
            }
        }
        for bounds in boxes.difference(&self.boxes) {
            for global in voxel_indices(*bounds) {
                let count = self.occupied.entry(global).or_default();
                *count += 1;
                if *count == 1
                    && let Some(index) = map.window_index(global)
                {
                    map.set_state(index, VoxelState::Occupied);
                }
            }
        }
        // Moving the ring buffer clears boundary strips, including retained source cells.
        if self.applied.is_some_and(|old| old.2 != key.2) {
            for global in self.occupied.keys() {
                if let Some(index) = map.window_index(*global) {
                    map.set_state(index, VoxelState::Occupied);
                }
            }
        }
        map.inflate_obstacles();
        std::mem::swap(&mut self.boxes, &mut self.next_boxes);
        self.applied = Some(key);
        Ok(())
    }
}
fn voxel_indices((start, end): ([i64; 3], [i64; 3])) -> impl Iterator<Item = [i64; 3]> {
    (start[0]..end[0]).flat_map(move |x| {
        (start[1]..end[1]).flat_map(move |y| (start[2]..end[2]).map(move |z| [x, y, z]))
    })
}
fn source_alignment(header: &MapElement) -> Result<RigidTransform> {
    RigidTransform::from_parts(
        FrameId::MAP,
        FrameId::ODOM,
        header.translation,
        header.rotation,
    )
}
fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("planner map snapshot")
}

#[cfg(test)]
mod tests {
    use super::*;
    fn nav(time: f64, revision: u64) -> NavigationMessage {
        NavigationMessage {
            revision,
            rotation: [0., 0., 0., 1.],
            odom: firefly_pubsub::odom::OdomMessage {
                timestamp: time,
                is_initialized: true,
                ..Default::default()
            },
            ..Default::default()
        }
    }
    fn packet(sequence: u64, revision: u64, time: f64, x: f64) -> Vec<MapElement> {
        vec![
            MapElement {
                sequence,
                revision,
                timestamp: time,
                session: 1,
                min: [-100.; 3],
                max: [100.; 3],
                rotation: [0., 0., 0., 1.],
                ..Default::default()
            },
            MapElement {
                kind: 1,
                min: [x, 2., 2.],
                max: [x + 0.5, 2.5, 2.5],
                ..Default::default()
            },
        ]
    }
    fn test_input() -> MapInput {
        MapInput {
            required: Some(MapRegion {
                min: [0.; 3],
                max: [8.; 3],
            }),
            ..Default::default()
        }
    }
    #[test]
    fn identical_geometry_updates_metadata_and_preserves_applied_cache() {
        let mut input = test_input();
        let now = Instant::now();
        let mut grid = firefly_map::GridMapBuilder::new(0.5, [16; 3])
            .build()
            .unwrap();
        input.observe(&packet(1, 0, 1., 1.), now).unwrap();
        input.apply(&mut grid, &nav(1., 0), false).unwrap();
        let applied = input.applied;
        let allocation = input.cells.as_ptr();
        let mut next = packet(2, 0, 2., 1.);
        next[0].min = [0.; 3];
        next[0].max = [8.; 3];
        input.observe(&next, now + Duration::from_secs(1)).unwrap();
        input.apply(&mut grid, &nav(2., 0), false).unwrap();
        assert_eq!(input.cells[0], next[0]);
        assert_eq!(input.cells.as_ptr(), allocation);
        assert_eq!(input.applied, applied);
        assert!(input.ready(
            &nav(2., 0),
            now + Duration::from_secs(1),
            Duration::from_secs(1)
        ));
        next[0].sequence = 3;
        next[0].max = [7.; 3];
        input.observe(&next, now + Duration::from_secs(1)).unwrap();
        assert!(!input.ready(
            &nav(2., 0),
            now + Duration::from_secs(1),
            Duration::from_secs(1)
        ));
    }

    #[test]
    fn fresh_publications_do_not_renew_fusion_time() {
        let mut input = test_input();
        let start = Instant::now();
        let timeout = Duration::from_secs(1);
        assert!(!input.ready(&nav(1., 0), start, timeout));
        input.observe(&packet(1, 0, 1., 1.), start).unwrap();
        assert!(input.ready(&nav(1., 0), start, timeout));
        input
            .observe(&packet(2, 0, 1., 1.), start + Duration::from_secs(2))
            .unwrap();
        assert!(!input.ready(&nav(1., 0), start + Duration::from_secs(2), timeout));
        input
            .observe(&packet(3, 0, 3., 1.), start + Duration::from_secs(2))
            .unwrap();
        assert!(input.ready(&nav(3., 0), start + Duration::from_secs(2), timeout));
        assert!(!input.ready(&nav(5., 0), start + Duration::from_secs(2), timeout));
        assert!(!input.ready(&nav(2., 0), start + Duration::from_secs(2), timeout));
    }
    #[test]
    fn correction_and_empty_snapshot_remove_old_contributions() {
        let mut input = test_input();
        let start = Instant::now();
        let mut grid = firefly_map::GridMapBuilder::new(0.5, [16, 16, 16])
            .with_obstacles_inflation(0.5)
            .build()
            .unwrap();
        grid.set_prior_occupied([12, 4, 4]);
        input.observe(&packet(1, 0, 1., 1.), start).unwrap();
        input.apply(&mut grid, &nav(1., 0), false).unwrap();
        assert!(grid.is_occupied(Vector3::new(1.25, 2.25, 2.25)));
        input.observe(&packet(2, 1, 1., 4.), start).unwrap();
        assert!(!input.ready(&nav(1., 0), start, Duration::from_secs(1)));
        assert!(input.ready(&nav(1., 1), start, Duration::from_secs(1)));
        input.apply(&mut grid, &nav(1., 1), false).unwrap();
        assert!(!grid.is_occupied_inflated(Vector3::new(1.25, 2.25, 2.25)));
        assert!(grid.is_occupied(Vector3::new(4.25, 2.25, 2.25)));
        let empty = packet(3, 1, 1.1, 4.);
        input.observe(&empty[..1], start).unwrap();
        input.apply(&mut grid, &nav(1.1, 1), false).unwrap();
        assert!(!grid.is_occupied_inflated(Vector3::new(4.25, 2.25, 2.25)));
        assert!(grid.is_occupied(Vector3::new(6.25, 2.25, 2.25)));
    }
    #[test]
    fn overlapping_sources_and_rolling_window_keep_obstacles() {
        let mut input = test_input();
        let mut grid = firefly_map::GridMapBuilder::new(0.5, [16, 16, 16])
            .with_obstacles_inflation(0.5)
            .build()
            .unwrap();
        let mut cells = packet(1, 0, 1., 2.);
        cells[1].max[0] = 3.;
        let mut overlapping = cells[1];
        overlapping.min[0] = 2.5;
        overlapping.max[0] = 3.5;
        cells.push(overlapping);
        input.observe(&cells, Instant::now()).unwrap();
        input.apply(&mut grid, &nav(1., 0), false).unwrap();
        cells.remove(1);
        cells[0].sequence = 2;
        input.observe(&cells, Instant::now()).unwrap();
        input.apply(&mut grid, &nav(1., 0), false).unwrap();
        assert!(!grid.is_occupied(Vector3::new(2.25, 2.25, 2.25)));
        assert!(grid.is_occupied(Vector3::new(2.75, 2.25, 2.25)));
        let mut moving = nav(1., 0);
        moving.odom.position_x = 4.5;
        moving.odom.position_y = 4.;
        moving.odom.position_z = 4.;
        input.apply(&mut grid, &moving, true).unwrap();
        assert!(grid.is_occupied(Vector3::new(2.75, 2.25, 2.25)));
        assert!(grid.is_occupied_inflated(Vector3::new(3.75, 2.25, 2.25)));
        assert_eq!(
            input.observed_state(grid.pos_to_global_index(Vector3::new(2.75, 2.25, 2.25))),
            VoxelState::Occupied
        );
    }

    #[test]
    fn external_alignment_transforms_geometry_and_internal_alignment_must_match() {
        let mut input = test_input();
        let mut grid = firefly_map::GridMapBuilder::new(0.5, [16, 16, 16])
            .build()
            .unwrap();
        let now = Instant::now();
        input.observe(&packet(1, 0, 1., 1.), now).unwrap();
        let mut navigation = nav(1., 0);
        navigation.translation = [2., 0., 0.];
        assert!(!input.ready(&navigation, now, Duration::from_secs(1)));
        navigation.external_map = 1;
        assert!(input.ready(&navigation, now, Duration::from_secs(1)));
        input.apply(&mut grid, &navigation, false).unwrap();
        assert!(grid.is_occupied(Vector3::new(3.25, 2.25, 2.25)));
        assert!(!grid.is_occupied(Vector3::new(1.25, 2.25, 2.25)));
        navigation.revision = 1;
        navigation.translation = [4., 0., 0.];
        navigation.rotation = [
            0.,
            0.,
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
        ];
        input.apply(&mut grid, &navigation, false).unwrap();
        // A 90-degree Z rotation maps [1,1.5] x [2,2.5] to [-2.5,-2] x [1,1.5].
        assert!(grid.is_occupied(Vector3::new(1.75, 1.25, 2.25)));
        assert!(!grid.is_occupied(Vector3::new(3.25, 2.25, 2.25)));
    }

    #[test]
    fn moving_window_requires_coverage_and_does_not_refresh_fusion_age() {
        let mut input = MapInput::default();
        let mut grid = firefly_map::GridMapBuilder::new(0.5, [16, 16, 16])
            .build()
            .unwrap();
        let start = Instant::now();
        let mut navigation = nav(1., 0);
        let request = input.prepare_region(&mut grid, &navigation, true).unwrap();
        let mut cells = packet(1, 0, 1., 1.);
        cells[0].min = request.min;
        cells[0].max = request.max;
        input.observe(&cells, start).unwrap();
        assert!(input.ready(&navigation, start, Duration::from_secs(1)));
        input.apply(&mut grid, &navigation, true).unwrap();
        navigation.odom.position_x = 1.;
        let prefetched = input.prepare_region(&mut grid, &navigation, true).unwrap();
        assert_ne!(prefetched, request);
        // Request the next window while the current response still covers planning.
        assert!(input.ready(&navigation, start, Duration::from_secs(1)));
        navigation.odom.position_x = 20.;
        let moved = input.prepare_region(&mut grid, &navigation, true).unwrap();
        assert!(!input.ready(&navigation, start, Duration::from_secs(1)));
        assert!(input.apply(&mut grid, &navigation, true).is_err());
        cells[0].sequence = 2;
        cells[0].min = moved.min;
        cells[0].max = moved.max;
        input
            .observe(&cells[..1], start + Duration::from_secs(2))
            .unwrap();
        assert!(!input.ready(
            &navigation,
            start + Duration::from_secs(2),
            Duration::from_secs(1)
        ));
        cells[0].sequence = 3;
        cells[0].timestamp = 3.;
        navigation.odom.timestamp = 3.;
        input
            .observe(&cells[..1], start + Duration::from_secs(2))
            .unwrap();
        assert!(input.ready(
            &navigation,
            start + Duration::from_secs(2),
            Duration::from_secs(1)
        ));
        input.apply(&mut grid, &navigation, true).unwrap();
        navigation.odom.position_x = 0.;
        input.prepare_region(&mut grid, &navigation, true).unwrap();
        assert!(!input.ready(
            &navigation,
            start + Duration::from_secs(2),
            Duration::from_secs(1)
        ));
    }

    #[test]
    fn session_change_and_invalid_cells_fail_closed() {
        let mut input = test_input();
        let now = Instant::now();
        input.observe(&packet(1, 0, 1., 1.), now).unwrap();
        let mut changed = packet(2, 0, 1.1, 1.);
        changed[0].session = 2;
        assert!(input.observe(&changed, now).is_err());
        assert!(!input.ready(&nav(1., 0), now, Duration::from_secs(1)));
        let mut input = test_input();
        changed[1].min[0] = f64::NAN;
        assert!(input.observe(&changed, now).is_err());
    }
}
