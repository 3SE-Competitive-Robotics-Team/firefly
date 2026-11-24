use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{GridMap, depth::DepthSnapshot};
use firefly_pubsub::odom::OdomMessage;
use nalgebra::Vector3;
use std::time::{Duration, Instant};

pub use firefly_map::depth::Bounds;

/// Complete collision geometry passed between threads in this planner process.
pub struct LocalSnapshot {
    pub map: DepthSnapshot,
    pub sequence: u64,
    pub last_observed_at: f64,
    pub fused_at: Instant,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapStatus {
    Waiting,
    Ready,
    Unavailable,
}

#[derive(Default)]
pub struct MapInput {
    snapshot: Option<LocalSnapshot>,
    faulted: bool,
}
impl MapInput {
    pub fn observe_local(&mut self, snapshot: LocalSnapshot) -> Result<()> {
        let result = self.accept(snapshot);
        self.faulted |= result.is_err();
        result
    }
    fn accept(&mut self, snapshot: LocalSnapshot) -> Result<()> {
        if !snapshot.last_observed_at.is_finite()
            || snapshot.last_observed_at < 0.
            || !snapshot.map.measured_at.is_finite()
            || snapshot.last_observed_at > snapshot.map.measured_at
            || !snapshot.map.bounds.valid()
        {
            return Err(invalid("invalid local collision snapshot"));
        }
        if let Some(old) = &self.snapshot
            && (snapshot.sequence <= old.sequence
                || snapshot.map.measured_at < old.map.measured_at
                || snapshot.last_observed_at < old.last_observed_at)
        {
            return Ok(());
        }
        self.snapshot = Some(snapshot);
        Ok(())
    }
    /// Move the view and install the latest fully covering snapshot using the same odometry.
    #[fastrace::trace]
    pub fn synchronize(
        &self,
        map: &mut GridMap,
        odometry: &OdomMessage,
        now: Instant,
        timeout: Duration,
    ) -> MapStatus {
        map.move_ring_buffer(Vector3::new(
            odometry.position_x,
            odometry.position_y,
            odometry.position_z,
        ));
        if self.faulted {
            return MapStatus::Unavailable;
        }
        let Some(snapshot) = &self.snapshot else {
            return MapStatus::Waiting;
        };
        if odometry.timestamp - snapshot.last_observed_at > timeout.as_secs_f64()
            || now.saturating_duration_since(snapshot.fused_at) > timeout
        {
            return MapStatus::Unavailable;
        }
        let low = map.origin();
        let required = Bounds {
            min: low.into(),
            max: (low + map.size()).into(),
        };
        if snapshot.map.measured_at > odometry.timestamp || !snapshot.map.bounds.contains(&required)
        {
            return MapStatus::Waiting;
        }
        map.set_online_snapshot(
            snapshot.map.collision.clone(),
            snapshot.map.observed.clone(),
        );
        MapStatus::Ready
    }
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
            map: DepthSnapshot {
                measured_at: 1.,
                collision: firefly_map::CollisionSnapshot::from_runs(
                    [0; 3],
                    [4; 3],
                    cells
                        .into_iter()
                        .map(|cell| cell.voxel_bounds(Vector3::zeros(), 1.)),
                ),
                observed: firefly_map::ObservedSpace::from_voxels(
                    1.,
                    firefly_map::VoxelBox::centered([2; 3], [4; 3]).unwrap(),
                    [],
                )
                .unwrap(),
                bounds: Bounds {
                    min: [0.; 3],
                    max: [4.; 3],
                },
            },
            sequence,
            last_observed_at: 1.,
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
        assert_eq!(
            input.synchronize(&mut map, &nav, Instant::now(), Duration::from_secs(1)),
            MapStatus::Ready
        );
        let point = Vector3::new(0.5, 1.5, 1.5);
        assert!(map.is_occupied_inflated(point));
        let mut away = nav;
        away.position_x = 6.;
        assert_eq!(
            input.synchronize(&mut map, &away, Instant::now(), Duration::from_secs(1)),
            MapStatus::Waiting
        );
        assert_eq!(
            input.synchronize(&mut map, &nav, Instant::now(), Duration::from_secs(1)),
            MapStatus::Ready
        );
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
        assert_eq!(
            input.synchronize(&mut map, &nav, Instant::now(), Duration::from_secs(1)),
            MapStatus::Ready
        );
        input.observe_local(snapshot(2, vec![b])).unwrap();
        assert_eq!(
            input.synchronize(&mut map, &nav, Instant::now(), Duration::from_secs(1)),
            MapStatus::Ready
        );
        assert!(!map.is_occupied_inflated(Vector3::new(0.5, 1.5, 1.5)));
        assert!(map.is_occupied_inflated(Vector3::new(2.5, 1.5, 1.5)));
        input.observe_local(snapshot(3, vec![])).unwrap();
        assert_eq!(
            input.synchronize(&mut map, &nav, Instant::now(), Duration::from_secs(1)),
            MapStatus::Ready
        );
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
        input.observe_local(snapshot(1, vec![])).unwrap();
        assert_eq!(
            input.synchronize(&mut map, &nav, Instant::now(), Duration::from_secs(1)),
            MapStatus::Unavailable
        );
        input.observe_local(snapshot(2, vec![])).unwrap();
        assert_eq!(
            input.synchronize(&mut map, &nav, Instant::now(), Duration::from_secs(1)),
            MapStatus::Ready
        );
        let mut invalid = snapshot(3, vec![]);
        invalid.map.bounds.min[0] = f64::NAN;
        assert!(input.observe_local(invalid).is_err());
        input.observe_local(snapshot(4, vec![])).unwrap();
        assert_eq!(
            input.synchronize(&mut map, &nav, Instant::now(), Duration::from_secs(1)),
            MapStatus::Unavailable
        );
    }
}
