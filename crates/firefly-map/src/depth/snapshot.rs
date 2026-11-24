use super::{DepthMapper, config::invalid};
use crate::{CollisionSnapshot, ObservedSpace};
use firefly_error::Result;
use nalgebra::Vector3;

const MAX_COLLISION_RUNS: usize = 1_000_000;

/// Half-open metric bounds in the local odom frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub min: [f64; 3],
    pub max: [f64; 3],
}
impl Bounds {
    #[must_use]
    pub fn valid(&self) -> bool {
        (0..3).all(|i| {
            self.min[i].is_finite() && self.max[i].is_finite() && self.min[i] < self.max[i]
        })
    }
    #[must_use]
    pub fn voxel_bounds(&self, origin: Vector3<f64>, resolution: f64) -> ([i64; 3], [i64; 3]) {
        let low = std::array::from_fn(|i| {
            lattice_coordinate((self.min[i] - origin[i]) / resolution).floor() as i64
        });
        let high = std::array::from_fn(|i| {
            lattice_coordinate((self.max[i] - origin[i]) / resolution).ceil() as i64
        });
        (low, high)
    }
    #[must_use]
    pub fn contains(&self, other: &Self) -> bool {
        (0..3).all(|i| self.min[i] <= other.min[i] && self.max[i] >= other.max[i])
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

/// Collision geometry and time-stamped evidence produced in the same mapping window.
pub struct DepthSnapshot {
    /// Latest accepted frame, including empty frames that moved the window.
    pub measured_at: f64,
    pub collision: CollisionSnapshot,
    pub observed: ObservedSpace,
    pub bounds: Bounds,
}

impl DepthMapper {
    /// Capture the current window. A fresh mapper has no window until it integrates a frame.
    /// # Errors
    /// Returns an error when the collision geometry exceeds the publishing budget.
    pub fn snapshot(&mut self) -> Result<Option<DepthSnapshot>> {
        let (Some(frame), Some(observed)) = (self.last_frame, self.grid.observations()) else {
            return Ok(None);
        };
        let resolution = self.grid.resolution();
        let bounds = Bounds {
            min: observed.bounds.low().map(|i| i as f64 * resolution),
            max: observed.bounds.end().map(|i| i as f64 * resolution),
        };
        let collision = if self.collision_origin == Vector3::zeros()
            && self.collision_resolution == resolution
        {
            self.grid.collision_snapshot(observed.bounds)
        } else {
            self.collision_at(bounds)?
        };
        Ok(Some(DepthSnapshot {
            measured_at: frame.measured_at,
            collision,
            observed,
            bounds,
        }))
    }

    fn collision_at(&mut self, bounds: Bounds) -> Result<CollisionSnapshot> {
        let revision = self.grid.collision_revision();
        if let Some((cached, collision)) = &self.collision_cache
            && *cached == revision
        {
            return Ok(collision.clone());
        }
        let next = self.build_collision(bounds)?;
        // Retain buffer identity when intervening updates cancel before publication.
        let collision = match &self.collision_cache {
            Some((_, old)) if *old == next => old.clone(),
            _ => next,
        };
        self.collision_cache = Some((revision, collision.clone()));
        Ok(collision)
    }

    fn build_collision(&self, bounds: Bounds) -> Result<CollisionSnapshot> {
        let resolution = self.grid.resolution();
        let (low, high) = bounds.voxel_bounds(self.collision_origin, self.collision_resolution);
        let mut dims = [0; 3];
        for (i, size) in dims.iter_mut().enumerate() {
            *size = high[i]
                .checked_sub(low[i])
                .and_then(|n| usize::try_from(n).ok())
                .filter(|&n| n > 0)
                .ok_or_else(|| invalid("unrepresentable depth snapshot bounds"))?;
        }
        dims.iter()
            .try_fold(1usize, |len, &size| len.checked_mul(size))
            .ok_or_else(|| invalid("depth snapshot dimensions overflow"))?;
        let mut count = 0;
        let runs = self
            .grid
            .inflated_runs()
            .take(MAX_COLLISION_RUNS + 1)
            .map(|(start, end)| {
                count += 1;
                Bounds {
                    min: start.map(|v| f64::from(v) * resolution),
                    max: end.map(|v| f64::from(v) * resolution),
                }
                .voxel_bounds(self.collision_origin, self.collision_resolution)
            });
        let snapshot = CollisionSnapshot::from_runs(low, dims, runs);
        if count > MAX_COLLISION_RUNS {
            return Err(invalid("local collision snapshot capacity exceeded"));
        }
        Ok(snapshot)
    }
}
