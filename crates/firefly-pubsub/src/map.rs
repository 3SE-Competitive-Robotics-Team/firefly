//! Atomic complete regional occupancy snapshots. Element zero is the header; remaining elements
//! are observed voxel bounds in the source map frame (1 occupied, 2 free).
//! Absence means unknown, never an incremental deletion. Consumers replace the snapshot.
use iceoryx2::prelude::*;

pub const MAP_TOPIC: &str = "Firefly/OccupancyMap";
pub const MAP_REGION_TOPIC: &str = "Firefly/MapRegion";

/// Requested coverage in the continuous odom frame, including planning margins.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, ZeroCopySend)]
#[type_name("FireflyMapRegion")]
pub struct MapRegion {
    pub min: [f64; 3],
    pub max: [f64; 3],
}
impl MapRegion {
    #[must_use]
    pub fn valid(&self) -> bool {
        (0..3).all(|i| {
            self.min[i].is_finite() && self.max[i].is_finite() && self.min[i] < self.max[i]
        })
    }
    #[must_use]
    pub fn contains(&self, other: &Self) -> bool {
        (0..3).all(|i| self.min[i] <= other.min[i] && self.max[i] >= other.max[i])
    }
}
/// Bound both transport allocation and consumer work; exceeding this fails publication.
pub const MAX_MAP_CELLS: usize = 1_000_000;

#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, ZeroCopySend)]
#[type_name("FireflyMapElement")]
pub struct MapElement {
    pub kind: u64,
    pub session: u64,
    pub sequence: u64,
    pub revision: u64,
    /// Last successfully fused nonempty depth measurement, not publication time.
    pub timestamp: f64,
    /// Header: complete coverage in odom. Cells: bounds in source map.
    pub min: [f64; 3],
    pub max: [f64; 3],
    /// Header only: source map <- odom.
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}
