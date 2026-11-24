//! Time-stamped ray evidence, shared by the fusion writer and immutable map readers.
use crate::{VoxelBox, grid::buffer::Buffer};
use nalgebra::Vector3;
use std::time::Duration;

/// The last measurement supporting the cell's current classification (seconds).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Evidence {
    Unknown,
    Free(f64),
    Occupied(f64),
}

/// Age and spatial coverage are evaluated at the caller's measurement time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OnlineState {
    OutsideCoverage,
    Unknown,
    Free(f64),
    Occupied(f64),
    Expired(Evidence),
}

/// Fixed geometry, online surface evidence and collision clearance from one map view.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CellSample {
    pub fixed_occupied: bool,
    pub online: OnlineState,
    pub collision: bool,
}

/// A writer owns the ring slots. Published views detach touched pages on subsequent writes.
#[derive(Clone, Debug)]
pub(crate) struct EvidenceBuffer(Buffer<Evidence>);
impl EvidenceBuffer {
    #[must_use]
    pub(crate) fn new(cells: usize) -> Self {
        Self(Buffer::new(Evidence::Unknown, cells))
    }
    pub(crate) fn set(&mut self, address: usize, evidence: Evidence) {
        if self.0[address] != evidence {
            self.0[address] = evidence;
        }
    }
    #[cfg(feature = "depth")]
    pub(crate) fn mark_free(&mut self, mask: &[u64], measured_at: f64) {
        self.0.fill_masked(mask, Evidence::Free(measured_at));
    }
    #[must_use]
    pub(crate) fn get(&self, address: usize) -> Evidence {
        self.0[address]
    }
    #[cfg(feature = "depth")]
    pub(crate) fn clear(&mut self) {
        self.0.fill(Evidence::Unknown);
    }

    /// Capture x-fastest ring data anchored on global multiples of the dimensions.
    /// # Panics
    /// The dimensions must match the buffer and the metric resolution must be positive.
    #[must_use]
    pub(crate) fn snapshot(&self, resolution: f64, bounds: VoxelBox) -> ObservedSpace {
        let dims = bounds.dimensions();
        assert!(resolution.is_finite() && resolution > 0.);
        assert_eq!(
            dims.into_iter()
                .try_fold(1usize, |len, n| len.checked_mul(usize::try_from(n).ok()?)),
            Some(self.0.len())
        );
        ObservedSpace {
            resolution,
            bounds,
            dims,
            cells: self.clone(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct ObservedSpace {
    pub(crate) resolution: f64,
    pub(crate) bounds: VoxelBox,
    dims: [i64; 3],
    cells: EvidenceBuffer,
}

impl ObservedSpace {
    /// Build an immutable observation layer from global voxel coordinates. Omitted cells
    /// are unknown. A repeated coordinate takes its last value in input order.
    /// # Errors
    /// Rejects invalid resolution, overflowing dimensions, out-of-window cells and invalid stamps.
    pub fn from_voxels(
        resolution: f64,
        bounds: VoxelBox,
        voxels: impl IntoIterator<Item = ([i64; 3], Evidence)>,
    ) -> firefly_error::Result<Self> {
        let invalid =
            |msg| firefly_error::Error::new(firefly_error::ErrorKind::InvalidArgument, msg);
        if !resolution.is_finite() || resolution <= 0. {
            return Err(invalid("invalid observation resolution"));
        }
        let dims: [i64; 3] = bounds.dimensions();
        let len = dims
            .into_iter()
            .try_fold(1usize, |n, d| n.checked_mul(usize::try_from(d).ok()?))
            .ok_or_else(|| invalid("observation dimensions overflow"))?;
        let mut cells = EvidenceBuffer::new(len);
        for (g, evidence) in voxels {
            if !bounds.contains(g) {
                return Err(invalid("evidence outside observation window"));
            }
            if let Evidence::Free(t) | Evidence::Occupied(t) = evidence
                && (!t.is_finite() || t < 0.)
            {
                return Err(invalid("invalid observation time"));
            }
            cells.set(ring_address(g, dims), evidence);
        }
        Ok(cells.snapshot(resolution, bounds))
    }
    #[must_use]
    pub fn sample(&self, point: Vector3<f64>, at: f64, max_age: Duration) -> OnlineState {
        if !point.iter().all(|v| v.is_finite()) {
            return OnlineState::OutsideCoverage;
        }
        let global: [i64; 3] = point.map(|v| (v / self.resolution).floor() as i64).into();
        self.state_at_index(global, at, max_age)
    }

    pub(crate) fn state_at_index(
        &self,
        global: [i64; 3],
        at: f64,
        max_age: Duration,
    ) -> OnlineState {
        if !self.bounds.contains(global) {
            return OnlineState::OutsideCoverage;
        }
        let evidence = self.cells.get(ring_address(global, self.dims));
        let (stamp, state) = match evidence {
            Evidence::Unknown => return OnlineState::Unknown,
            Evidence::Free(stamp) => (stamp, OnlineState::Free(stamp)),
            Evidence::Occupied(stamp) => (stamp, OnlineState::Occupied(stamp)),
        };
        if !at.is_finite() || !stamp.is_finite() || stamp < 0. || at < stamp {
            OnlineState::Unknown
        } else if at - stamp > max_age.as_secs_f64() {
            OnlineState::Expired(evidence)
        } else {
            state
        }
    }
}

fn ring_address(global: [i64; 3], dims: [i64; 3]) -> usize {
    let [x, y, z]: [usize; 3] = std::array::from_fn(|i| global[i].rem_euclid(dims[i]) as usize);
    (z * dims[1] as usize + y) * dims[0] as usize + x
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn imported_evidence_uses_world_coordinates_across_the_ring_seam() {
        let bounds = VoxelBox::centered([-2, 1, 0], [4, 3, 2]).unwrap();
        let cell = [-3, 1, -1];
        let evidence = [(cell, Evidence::Free(3.)), (cell, Evidence::Occupied(4.))];
        let view = ObservedSpace::from_voxels(1., bounds, evidence).unwrap();
        let center = Vector3::from(cell).cast::<f64>() + Vector3::repeat(0.5);
        assert_eq!(
            view.sample(center, 4., Duration::ZERO),
            OnlineState::Occupied(4.)
        );
        assert!(
            ObservedSpace::from_voxels(1., bounds, [(bounds.end(), Evidence::Unknown)]).is_err()
        );
        assert!(
            ObservedSpace::from_voxels(1., bounds, [(cell, Evidence::Free(f64::NAN))]).is_err()
        );
    }

    #[test]
    fn evidence_is_frozen_at_publication_and_expires_at_read_time() {
        let mut writer = EvidenceBuffer::new(64);
        writer.set(0, Evidence::Free(0.));
        let map = writer.snapshot(1., VoxelBox::centered([0; 3], [4; 3]).unwrap());
        writer.set(0, Evidence::Occupied(2.));
        let age = Duration::from_secs(1);
        assert_eq!(
            map.sample(Vector3::repeat(0.5), 1., age),
            OnlineState::Free(0.)
        );
        assert_eq!(
            map.sample(Vector3::repeat(0.5), 2., age),
            OnlineState::Expired(Evidence::Free(0.))
        );
        assert_eq!(
            map.sample(Vector3::repeat(-0.5), 2., age),
            OnlineState::Unknown
        );
        assert_eq!(
            map.sample(Vector3::repeat(2.), 2., age),
            OnlineState::OutsideCoverage
        );
    }
}
