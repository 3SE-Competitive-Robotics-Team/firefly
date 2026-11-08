//! Derive a collision grid from the planner-owned, already inflated depth map.
use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{GridMap, VoxelState};
use firefly_pubsub::navigation::NavigationMessage;
use nalgebra::Vector3;
use std::{
    collections::{HashMap, HashSet},
    time::{Duration, Instant},
};

/// Half-open metric bounds in the navigation map frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub min: [f64; 3],
    pub max: [f64; 3],
}
impl Bounds {
    pub fn valid(&self) -> bool {
        (0..3).all(|i| {
            self.min[i].is_finite() && self.max[i].is_finite() && self.min[i] < self.max[i]
        })
    }
    pub fn contains(&self, other: &Self) -> bool {
        (0..3).all(|i| self.min[i] <= other.min[i] && self.max[i] >= other.max[i])
    }
}

/// Complete collision geometry passed between threads in this planner process.
pub struct LocalSnapshot {
    pub cells: Vec<Bounds>,
    pub bounds: Bounds,
    pub sequence: u64,
    pub revision: u64,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
    pub timestamp: f64,
    pub fused_at: Instant,
}

impl LocalSnapshot {
    fn matches(&self, navigation: &NavigationMessage) -> bool {
        self.revision == navigation.revision
            && self.translation == navigation.translation
            && self.rotation == navigation.rotation
    }
}

#[derive(Default)]
pub struct MapInput {
    required: Option<Bounds>,
    snapshot: Option<LocalSnapshot>,
    fault: bool,
    occupied: HashMap<[i64; 3], u32>,
    boxes: HashSet<([i64; 3], [i64; 3])>,
    next_boxes: HashSet<([i64; 3], [i64; 3])>,
    applied: Option<(u64, u64, [i64; 3])>,
    geometry_version: u64,
    window_moved: bool,
}
impl MapInput {
    pub fn observe_local(&mut self, snapshot: LocalSnapshot) -> Result<()> {
        let result = self.accept(snapshot);
        self.fault |= result.is_err();
        result
    }
    fn accept(&mut self, snapshot: LocalSnapshot) -> Result<()> {
        if !snapshot.timestamp.is_finite()
            || snapshot.timestamp < 0.
            || !snapshot.bounds.valid()
            || snapshot.cells.iter().any(|cell| !cell.valid())
        {
            return Err(invalid("invalid local collision snapshot"));
        }
        firefly_base::RigidTransform::from_parts(
            firefly_base::FrameId::MAP,
            firefly_base::FrameId::ODOM,
            snapshot.translation,
            snapshot.rotation,
        )?;
        if let Some(old) = &self.snapshot {
            if snapshot.sequence <= old.sequence
                || snapshot.revision < old.revision
                || snapshot.timestamp < old.timestamp
            {
                return Ok(());
            }
            if snapshot.revision == old.revision
                && (snapshot.translation != old.translation || snapshot.rotation != old.rotation)
            {
                return Err(invalid("unversioned depth map alignment change"));
            }
        }
        if self
            .snapshot
            .as_ref()
            .is_none_or(|old| old.cells != snapshot.cells)
        {
            self.geometry_version = self.geometry_version.wrapping_add(1);
        }
        self.snapshot = Some(snapshot);
        Ok(())
    }
    pub fn ready(&self, navigation: &NavigationMessage, now: Instant, timeout: Duration) -> bool {
        !self.expired(navigation, now, timeout)
            && self.snapshot.as_ref().is_some_and(|snapshot| {
                snapshot.matches(navigation)
                    && self
                        .required
                        .is_some_and(|required| snapshot.bounds.contains(&required))
                    && snapshot.timestamp <= navigation.odom.timestamp
            })
    }

    /// Coverage and revision handoffs can wait for the next fused snapshot.
    /// Only actual age expiry or a latched input fault disables recovery.
    pub fn expired(&self, navigation: &NavigationMessage, now: Instant, timeout: Duration) -> bool {
        self.fault
            || self.snapshot.as_ref().is_some_and(|snapshot| {
                navigation.odom.timestamp - snapshot.timestamp > timeout.as_secs_f64()
                    || now.saturating_duration_since(snapshot.fused_at) > timeout
            })
    }

    /// Move the planning window before checking depth-map coverage.
    pub fn prepare_region(
        &mut self,
        map: &mut GridMap,
        navigation: &NavigationMessage,
        rolling: bool,
    ) -> Result<Bounds> {
        navigation.alignment()?;
        if rolling {
            let before = map.global_index([0; 3]);
            map.move_ring_buffer(Vector3::new(
                navigation.odom.position_x,
                navigation.odom.position_y,
                navigation.odom.position_z,
            ));
            self.window_moved |= map.global_index([0; 3]) != before;
        }
        let low = map.origin();
        let high = low + Vector3::from(map.dims().map(|d| d as f64 * map.resolution()));
        let required = Bounds {
            min: low.into(),
            max: high.into(),
        };
        self.required = Some(required);
        Ok(required)
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
        let required = self.prepare_region(map, navigation, rolling)?;
        let snapshot = self
            .snapshot
            .as_ref()
            .ok_or_else(|| invalid("depth map absent"))?;
        if self.fault || !snapshot.matches(navigation) || !snapshot.bounds.contains(&required) {
            return Err(invalid("depth map version or coverage mismatch"));
        }
        let key = (
            self.geometry_version,
            navigation.revision,
            map.global_index([0; 3]),
        );
        if self.applied == Some(key) && !self.window_moved {
            return Ok(());
        }
        let origin = map.origin();
        let resolution = map.resolution();
        let dims = map.dims();
        self.next_boxes.clear();
        let boxes = &mut self.next_boxes;
        for cell in &snapshot.cells {
            let start: [usize; 3] = std::array::from_fn(|i| {
                lattice_coordinate((cell.min[i] - origin[i]) / resolution)
                    .floor()
                    .clamp(0., dims[i] as f64) as usize
            });
            let end: [usize; 3] = std::array::from_fn(|i| {
                lattice_coordinate((cell.max[i] - origin[i]) / resolution)
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
        // A ring shift clears boundary strips even when a source box survives.
        if self.window_moved {
            for global in self.occupied.keys() {
                if let Some(index) = map.window_index(*global) {
                    map.set_state(index, VoxelState::Occupied);
                }
            }
        }
        map.inflate_obstacles();
        std::mem::swap(&mut self.boxes, &mut self.next_boxes);
        self.applied = Some(key);
        self.window_moved = false;
        Ok(())
    }
}
// Source cells use an exact lattice; round-off must not expand them by one voxel.
fn lattice_coordinate(value: f64) -> f64 {
    if (value - value.round()).abs() <= 1e-8 {
        value.round()
    } else {
        value
    }
}
fn voxel_indices((start, end): ([i64; 3], [i64; 3])) -> impl Iterator<Item = [i64; 3]> {
    (start[0]..end[0]).flat_map(move |x| {
        (start[1]..end[1]).flat_map(move |y| (start[2]..end[2]).map(move |z| [x, y, z]))
    })
}
fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message)
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_map::GridMapBuilder;
    use firefly_pubsub::odom::OdomMessage;

    fn navigation() -> NavigationMessage {
        NavigationMessage {
            odom: OdomMessage {
                timestamp: 1.,
                position_x: 2.,
                position_y: 2.,
                position_z: 2.,
                is_initialized: true,
                ..Default::default()
            },
            rotation: [0., 0., 0., 1.],
            ..Default::default()
        }
    }

    fn snapshot(sequence: u64, cells: Vec<Bounds>) -> LocalSnapshot {
        LocalSnapshot {
            cells,
            bounds: Bounds {
                min: [0.; 3],
                max: [4.; 3],
            },
            sequence,
            revision: 0,
            translation: [0.; 3],
            rotation: [0., 0., 0., 1.],
            timestamp: 1.,
            fused_at: Instant::now(),
        }
    }

    #[test]
    fn returning_from_an_uncovered_window_restores_boundary_obstacles() {
        let mut map = GridMapBuilder::new(1., [4; 3])
            .with_obstacles_inflation(0.)
            .build()
            .unwrap();
        let mut input = MapInput::default();
        let nav = navigation();
        input
            .observe_local(snapshot(
                1,
                vec![Bounds {
                    min: [0., 1., 1.],
                    max: [1., 2., 2.],
                }],
            ))
            .unwrap();
        input.apply(&mut map, &nav, true).unwrap();
        let point = Vector3::new(0.5, 1.5, 1.5);
        assert!(map.is_occupied_inflated(point));
        let mut away = nav;
        away.odom.position_x = 6.;
        input.prepare_region(&mut map, &away, true).unwrap();
        assert!(!input.ready(&away, Instant::now(), Duration::from_secs(1)));
        assert!(!input.expired(&away, Instant::now(), Duration::from_secs(1)));
        input.prepare_region(&mut map, &nav, true).unwrap();
        assert!(!map.is_occupied_inflated(point));
        input.apply(&mut map, &nav, true).unwrap();
        assert!(map.is_occupied_inflated(point));
    }

    #[test]
    fn removing_overlapping_observations_preserves_remaining_obstacles_and_priors() {
        let mut map = GridMapBuilder::new(1., [4; 3])
            .with_obstacles_inflation(0.)
            .build()
            .unwrap();
        map.set_prior_occupied([1, 1, 1]);
        let mut input = MapInput::default();
        let nav = navigation();
        let a = Bounds {
            min: [0., 1., 1.],
            max: [2., 2., 2.],
        };
        let b = Bounds {
            min: [1., 1., 1.],
            max: [3., 2., 2.],
        };
        input.observe_local(snapshot(1, vec![a, b])).unwrap();
        input.apply(&mut map, &nav, true).unwrap();
        input.observe_local(snapshot(2, vec![b])).unwrap();
        input.apply(&mut map, &nav, true).unwrap();
        assert!(!map.is_occupied_inflated(Vector3::new(0.5, 1.5, 1.5)));
        assert!(map.is_occupied_inflated(Vector3::new(2.5, 1.5, 1.5)));
        input.observe_local(snapshot(3, vec![])).unwrap();
        input.apply(&mut map, &nav, true).unwrap();
        assert!(!map.is_occupied_inflated(Vector3::new(2.5, 1.5, 1.5)));
        assert!(map.is_occupied_inflated(Vector3::new(1.5, 1.5, 1.5)));
    }

    #[test]
    fn duplicate_snapshots_cannot_refresh_age_and_invalid_alignment_latches_fault() {
        let mut input = MapInput::default();
        let mut map = GridMapBuilder::new(1., [4; 3]).build().unwrap();
        let nav = navigation();
        let mut stale = snapshot(1, vec![]);
        stale.fused_at -= Duration::from_secs(2);
        input.observe_local(stale).unwrap();
        input.prepare_region(&mut map, &nav, true).unwrap();
        input.observe_local(snapshot(1, vec![])).unwrap();
        assert!(!input.ready(&nav, Instant::now(), Duration::from_secs(1)));
        assert!(input.expired(&nav, Instant::now(), Duration::from_secs(1)));
        input.observe_local(snapshot(2, vec![])).unwrap();
        assert!(input.ready(&nav, Instant::now(), Duration::from_secs(1)));
        let mut invalid = snapshot(3, vec![]);
        invalid.translation[0] = 0.01;
        assert!(input.observe_local(invalid).is_err());
        input.observe_local(snapshot(4, vec![])).unwrap();
        assert!(!input.ready(&nav, Instant::now(), Duration::from_secs(1)));
    }
}
