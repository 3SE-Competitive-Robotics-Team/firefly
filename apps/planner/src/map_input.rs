//! Derive a collision grid from the planner-owned, already inflated depth map.
use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{GridMap, VoxelState};
use firefly_pubsub::odom::OdomMessage;
use nalgebra::Vector3;
use std::{
    collections::{HashMap, HashSet},
    time::{Duration, Instant},
};

/// Half-open metric bounds in the local odom frame.
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
    pub observed: firefly_map::ObservedSpace,
    pub bounds: Bounds,
    pub sequence: u64,
    pub timestamp: f64,
    pub fused_at: Instant,
}

#[derive(Default)]
pub struct MapInput {
    required: Option<Bounds>,
    snapshot: Option<LocalSnapshot>,
    fault: Option<String>,
    occupied: HashMap<[i64; 3], u32>,
    boxes: HashSet<([i64; 3], [i64; 3])>,
    next_boxes: HashSet<([i64; 3], [i64; 3])>,
    applied: Option<(u64, [i64; 3])>,
    geometry_version: u64,
    window_moved: bool,
}
impl MapInput {
    pub fn observed(&self) -> Option<firefly_map::ObservedSpace> {
        self.snapshot.as_ref().map(|s| s.observed.clone())
    }
    pub fn observe_local(&mut self, snapshot: LocalSnapshot) -> Result<()> {
        let result = self.accept(snapshot);
        if let Err(error) = &result {
            self.fault.get_or_insert_with(|| error.to_string());
        }
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
        if let Some(old) = &self.snapshot
            && (snapshot.sequence <= old.sequence || snapshot.timestamp < old.timestamp)
        {
            return Ok(());
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
    pub fn ready(&self, odometry: &OdomMessage, now: Instant, timeout: Duration) -> bool {
        !self.expired(odometry, now, timeout)
            && self.snapshot.as_ref().is_some_and(|snapshot| {
                self.required
                    .is_some_and(|required| snapshot.bounds.contains(&required))
                    && snapshot.timestamp <= odometry.timestamp
            })
    }

    /// Coverage gaps can wait for the next fused snapshot.
    /// Only actual age expiry or a latched input fault disables recovery.
    pub fn expired(&self, odometry: &OdomMessage, now: Instant, timeout: Duration) -> bool {
        self.fault.is_some()
            || self.snapshot.as_ref().is_some_and(|snapshot| {
                odometry.timestamp - snapshot.timestamp > timeout.as_secs_f64()
                    || now.saturating_duration_since(snapshot.fused_at) > timeout
            })
    }

    /// Move the planning window before checking depth-map coverage.
    pub fn prepare_region(
        &mut self,
        map: &mut GridMap,
        odometry: &OdomMessage,
        rolling: bool,
    ) -> Bounds {
        if rolling {
            let before = map.global_index([0; 3]);
            map.move_ring_buffer(Vector3::new(
                odometry.position_x,
                odometry.position_y,
                odometry.position_z,
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
        required
    }
    #[fastrace::trace]
    pub fn apply(
        &mut self,
        map: &mut GridMap,
        odometry: &OdomMessage,
        rolling: bool,
    ) -> Result<()> {
        let required = self.prepare_region(map, odometry, rolling);
        if let Some(reason) = &self.fault {
            return Err(invalid("depth map input fault").with_context("reason", reason.clone()));
        }
        let snapshot = self
            .snapshot
            .as_ref()
            .ok_or_else(|| invalid("depth map absent"))?;
        if !snapshot.bounds.contains(&required) {
            return Err(invalid("depth map coverage mismatch"));
        }
        let key = (self.geometry_version, map.global_index([0; 3]));
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

    fn odometry() -> OdomMessage {
        OdomMessage {
            timestamp: 1.,
            position_x: 2.,
            position_y: 2.,
            position_z: 2.,
            is_initialized: true,
            ..Default::default()
        }
    }

    fn snapshot(sequence: u64, cells: Vec<Bounds>) -> LocalSnapshot {
        LocalSnapshot {
            cells,
            observed: firefly_map::ObservedSpace::from_buffer(
                1.,
                [0; 3],
                [4; 3],
                [0; 3],
                &[false; 64],
            ),
            bounds: Bounds {
                min: [0.; 3],
                max: [4.; 3],
            },
            sequence,
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
        let nav = odometry();
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
        away.position_x = 6.;
        input.prepare_region(&mut map, &away, true);
        assert!(!input.ready(&away, Instant::now(), Duration::from_secs(1)));
        assert!(!input.expired(&away, Instant::now(), Duration::from_secs(1)));
        input.prepare_region(&mut map, &nav, true);
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
        let nav = odometry();
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
    fn duplicate_snapshots_cannot_refresh_age_and_invalid_geometry_latches_fault() {
        let mut input = MapInput::default();
        let mut map = GridMapBuilder::new(1., [4; 3]).build().unwrap();
        let nav = odometry();
        let mut stale = snapshot(1, vec![]);
        stale.fused_at -= Duration::from_secs(2);
        input.observe_local(stale).unwrap();
        input.prepare_region(&mut map, &nav, true);
        input.observe_local(snapshot(1, vec![])).unwrap();
        assert!(!input.ready(&nav, Instant::now(), Duration::from_secs(1)));
        assert!(input.expired(&nav, Instant::now(), Duration::from_secs(1)));
        input.observe_local(snapshot(2, vec![])).unwrap();
        assert!(input.ready(&nav, Instant::now(), Duration::from_secs(1)));
        let mut invalid = snapshot(3, vec![]);
        invalid.bounds.min[0] = f64::NAN;
        assert!(input.observe_local(invalid).is_err());
        input.observe_local(snapshot(4, vec![])).unwrap();
        assert!(!input.ready(&nav, Instant::now(), Duration::from_secs(1)));
    }
}
