use super::GridMap;
use crate::OnlineState;
use nalgebra::Vector3;
use std::time::Duration;

#[derive(Clone, Copy)]
pub enum SegmentRule {
    KnownObstacles,
    RecentlyFree { measured_at: f64, max_age: Duration },
}

impl GridMap {
    /// Check both endpoints and the cells between them in the local planning window.
    /// Collision uses configured clearance. `RecentlyFree` also requires fresh ray evidence
    /// throughout the corresponding cubical neighborhood on the sensor lattice.
    #[must_use]
    pub fn segment_is_clear(&self, a: Vector3<f64>, b: Vector3<f64>, rule: SegmentRule) -> bool {
        if !a.iter().chain(b.iter()).all(|v| v.is_finite())
            || self.window_global(a).is_none()
            || self.window_global(b).is_none()
            || self
                .virtual_wall
                .is_some_and(|w| w.blocks(a.z) || w.blocks(b.z))
        {
            return false;
        }
        let local = |p| (p - self.world_origin) / self.resolution;
        if !traverse(local(a), local(b), |g| {
            !self.external_blocked(g) && self.inflate[self.inf_addr(g)] == 0
        }) {
            return false;
        }
        let SegmentRule::RecentlyFree {
            measured_at,
            max_age,
        } = rule
        else {
            return true;
        };
        let Some(obs) = &self.observations else {
            return false;
        };
        let clearance = self.resolution * f64::from(self.inf_grid);
        let margin = Vector3::repeat(clearance);
        if self.window_global(a.inf(&b) - margin).is_none()
            || self.window_global(a.sup(&b) + margin).is_none()
        {
            return false;
        }
        let r = (clearance / obs.resolution).ceil() as i64;
        let lo = (a.inf(&b) / obs.resolution).map(|v| v.floor() as i64);
        let hi = (a.sup(&b) / obs.resolution).map(|v| v.floor() as i64);
        if (0..3).any(|i| {
            lo[i].checked_sub(r).is_none_or(|v| v < obs.bounds.low()[i])
                || hi[i]
                    .checked_add(r)
                    .is_none_or(|v| v >= obs.bounds.end()[i])
        }) {
            return false;
        }
        traverse(a / obs.resolution, b / obs.resolution, |g| {
            (-r..=r).all(|z| {
                (-r..=r).all(|y| {
                    (-r..=r).all(|x| {
                        matches!(
                            obs.state_at_index(
                                [g[0] + x, g[1] + y, g[2] + z],
                                measured_at,
                                max_age
                            ),
                            OnlineState::Free(_)
                        )
                    })
                })
            })
        })
    }
}

// Voxel DDA. At a tied crossing, the half-open boundary point belongs to the
// positive-side cell on every axis, including when the ray has mixed signs.
fn traverse(a: Vector3<f64>, b: Vector3<f64>, mut clear: impl FnMut([i64; 3]) -> bool) -> bool {
    let mut cell = a.map(|v| v.floor() as i64);
    let end = b.map(|v| v.floor() as i64);
    let delta = b - a;
    loop {
        if !clear(cell.into()) {
            return false;
        }
        if cell == end {
            return true;
        }
        let crossing: Vector3<f64> = Vector3::from_fn(|i, _| {
            if cell[i] == end[i] {
                f64::INFINITY
            } else {
                ((cell[i] + i64::from(delta[i] > 0.)) as f64 - a[i]) / delta[i]
            }
        });
        let t = crossing.min();
        let mut boundary = cell;
        let before = cell;
        for i in 0..3 {
            if crossing[i] == t {
                if delta[i] > 0. {
                    cell[i] += 1;
                    boundary[i] = cell[i];
                } else {
                    cell[i] -= 1;
                }
            }
        }
        if boundary != before && boundary != cell && !clear(boundary.into()) {
            return false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{GridMapBuilder, VoxelState};

    #[test]
    fn clearance_needs_fresh_evidence_off_the_centerline() {
        let mut map = GridMapBuilder::new(1., [7; 3])
            .with_obstacles_inflation(1.)
            .build()
            .unwrap();
        let bounds = crate::VoxelBox::centered([3; 3], [7; 3]).unwrap();
        let free = (0..7).flat_map(|x| {
            (0..7).flat_map(move |y| {
                (0..7).map(move |z| {
                    let cell = [x, y, z];
                    (
                        cell,
                        crate::Evidence::Free(if cell == [3, 4, 3] { 1. } else { 2. }),
                    )
                })
            })
        });
        map.set_online_snapshot(
            crate::CollisionSnapshot::from_runs([0; 3], [7; 3], []),
            crate::ObservedSpace::from_voxels(1., bounds, free).unwrap(),
        );
        let a = Vector3::new(2.5, 3.5, 3.5);
        let b = Vector3::new(4.5, 3.5, 3.5);
        let fresh = |at| SegmentRule::RecentlyFree {
            measured_at: at,
            max_age: Duration::from_secs(1),
        };
        assert!(map.segment_is_clear(a, b, fresh(2.)));
        assert!(!map.segment_is_clear(a, b, fresh(2.1)));
        assert!(map.segment_is_clear(a, b, SegmentRule::KnownObstacles));
        assert!(!map.segment_is_clear(a, Vector3::new(6.5, 3.5, 3.5), fresh(2.)));
    }

    #[test]
    fn diagonal_touch_and_intervening_short_cell_block_both_directions() {
        let mut map = GridMapBuilder::new(1., [4; 3])
            .with_obstacles_inflation(0.)
            .build()
            .unwrap();
        map.set_state([1, 1, 0], VoxelState::Occupied);
        map.inflate_obstacles();
        let a = Vector3::new(0.5, 1.5, 0.5);
        for b in [Vector3::new(1.5, 0.5, 0.5), Vector3::new(1.5, 0.51, 0.5)] {
            assert!(!map.segment_is_clear(a, b, SegmentRule::KnownObstacles));
            assert!(!map.segment_is_clear(b, a, SegmentRule::KnownObstacles));
        }
    }
}
