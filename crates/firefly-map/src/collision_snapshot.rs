//! Immutable occupancy in the owning `GridMap`'s global voxel lattice.
use std::sync::Arc;

#[derive(Clone, Debug)]
pub struct CollisionSnapshot {
    low: [i64; 3],
    dims: [usize; 3],
    words: Words,
}

#[derive(Clone, Debug)]
enum Words {
    Dense(Arc<[u64]>),
    #[cfg(feature = "depth")]
    Ring {
        dims: [usize; 3],
        start: [usize; 3],
        bits: crate::grid::buffer::Buffer<u64>,
    },
}

impl CollisionSnapshot {
    /// Pack half-open occupied boxes as a union, with x-contiguous rows.
    /// Overlap is idempotent; a new snapshot fully replaces previous observations.
    ///
    /// # Panics
    /// Dimensions must be positive and their product must fit in usize.
    #[must_use]
    pub fn from_runs(
        low: [i64; 3],
        dims: [usize; 3],
        runs: impl IntoIterator<Item = ([i64; 3], [i64; 3])>,
    ) -> Self {
        assert!(dims.iter().all(|n| *n > 0));
        let len = dims
            .into_iter()
            .try_fold(1usize, usize::checked_mul)
            .expect("collision dimensions overflow");
        let mut words = vec![0u64; len.div_ceil(64)];
        for (start, end) in runs {
            let begin: [usize; 3] =
                std::array::from_fn(|i| (start[i] - low[i]).clamp(0, dims[i] as i64) as usize);
            let end: [usize; 3] =
                std::array::from_fn(|i| (end[i] - low[i]).clamp(0, dims[i] as i64) as usize);
            if (0..3).any(|i| begin[i] >= end[i]) {
                continue;
            }
            for z in begin[2]..end[2] {
                for y in begin[1]..end[1] {
                    let row = (z * dims[1] + y) * dims[0];
                    let mut at = row + begin[0];
                    let stop = row + end[0];
                    while at < stop {
                        let count = (64 - at % 64).min(stop - at);
                        let mask = u64::MAX >> (64 - count);
                        words[at / 64] |= mask << (at % 64);
                        at += count;
                    }
                }
            }
        }
        Self {
            low,
            dims,
            words: Words::Dense(words.into()),
        }
    }

    #[cfg(feature = "depth")]
    pub(crate) fn from_ring(
        bounds: crate::VoxelBox,
        dims: [usize; 3],
        bits: &crate::grid::buffer::Buffer<u64>,
    ) -> Self {
        let start = std::array::from_fn(|i| bounds.low()[i].rem_euclid(dims[i] as i64) as usize);
        Self {
            low: bounds.low(),
            dims: bounds.dimensions().map(|v| v as usize),
            words: Words::Ring {
                dims,
                start,
                bits: bits.clone(),
            },
        }
    }

    // Address of the first bit and number of contiguous cells before the ring seam.
    fn row_address(&self, local: [usize; 3]) -> (usize, usize) {
        let (dims, [x, y, z]) = match &self.words {
            Words::Dense(_) => (self.dims, local),
            #[cfg(feature = "depth")]
            Words::Ring { dims, start, .. } => (
                *dims,
                std::array::from_fn(|i| {
                    let at = start[i] + local[i];
                    if at >= dims[i] { at - dims[i] } else { at }
                }),
            ),
        };
        ((z * dims[1] + y) * dims[0] + x, dims[0] - x)
    }

    fn word(&self, index: usize) -> u64 {
        match &self.words {
            Words::Dense(words) => words[index],
            #[cfg(feature = "depth")]
            Words::Ring { bits, .. } => bits[index],
        }
    }

    #[must_use]
    pub fn is_occupied(&self, global: [i64; 3]) -> bool {
        let local: [i64; 3] = std::array::from_fn(|i| global[i].saturating_sub(self.low[i]));
        if (0..3).any(|i| local[i] < 0 || local[i] >= self.dims[i] as i64) {
            return false;
        }
        let (address, _) = self.row_address(local.map(|i| i as usize));
        self.word(address / 64) & (1 << (address % 64)) != 0
    }

    /// Visit set bits inside the intersection with a half-open global box.
    pub(crate) fn visit_occupied(
        &self,
        low: [i64; 3],
        end: [i64; 3],
        mut visit: impl FnMut([i64; 3]),
    ) {
        let clip = |at: [i64; 3]| -> [usize; 3] {
            std::array::from_fn(|i| {
                at[i]
                    .saturating_sub(self.low[i])
                    .clamp(0, self.dims[i] as i64) as usize
            })
        };
        let low = clip(low);
        let end = clip(end);
        if (0..3).any(|i| low[i] >= end[i]) {
            return;
        }
        for z in low[2]..end[2] {
            for y in low[1]..end[1] {
                let mut x = low[0];
                while x < end[0] {
                    let (at, contiguous) = self.row_address([x, y, z]);
                    let count = (64 - at % 64).min(end[0] - x).min(contiguous);
                    let mut bits = (self.word(at / 64) >> (at % 64)) & (u64::MAX >> (64 - count));
                    while bits != 0 {
                        visit([
                            self.low[0] + x as i64 + i64::from(bits.trailing_zeros()),
                            self.low[1] + y as i64,
                            self.low[2] + z as i64,
                        ]);
                        bits &= bits - 1;
                    }
                    x += count;
                }
            }
        }
    }

    pub(crate) fn shares_data(&self, other: &Self) -> bool {
        self.low == other.low
            && self.dims == other.dims
            && match (&self.words, &other.words) {
                (Words::Dense(a), Words::Dense(b)) => Arc::ptr_eq(a, b),
                #[cfg(feature = "depth")]
                (
                    Words::Ring {
                        dims: da, bits: a, ..
                    },
                    Words::Ring {
                        dims: db, bits: b, ..
                    },
                ) => da == db && a.shares_data(b),
                #[cfg(feature = "depth")]
                _ => false,
            }
    }
}

impl PartialEq for CollisionSnapshot {
    fn eq(&self, other: &Self) -> bool {
        if self.low != other.low || self.dims != other.dims {
            return false;
        }
        match (&self.words, &other.words) {
            (Words::Dense(a), Words::Dense(b)) => a == b,
            #[cfg(feature = "depth")]
            _ => {
                self.shares_data(other)
                    || (0..self.dims[2]).all(|z| {
                        (0..self.dims[1]).all(|y| {
                            (0..self.dims[0]).all(|x| {
                                let g = std::array::from_fn(|i| self.low[i] + [x, y, z][i] as i64);
                                self.is_occupied(g) == other.is_occupied(g)
                            })
                        })
                    })
            }
        }
    }
}
impl Eq for CollisionSnapshot {}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "depth")]
    #[test]
    fn ring_region_matches_dense_after_writer_reuses_its_pages() {
        let dims = [73, 7, 5];
        let bounds = crate::VoxelBox::centered([1, -2, 1], [69, 5, 4]).unwrap();
        let mut bits = crate::grid::buffer::Buffer::new(0_u64, (73_usize * 7 * 5).div_ceil(64));
        let mut runs = Vec::new();
        let low = bounds.low();
        let end = bounds.end();
        for z in low[2]..end[2] {
            for y in low[1]..end[1] {
                for x in low[0]..end[0] {
                    if (x + 3 * y - z).rem_euclid(5) == 0 {
                        let [i, j, k]: [usize; 3] = std::array::from_fn(|a| {
                            [x, y, z][a].rem_euclid(dims[a] as i64) as usize
                        });
                        let bit = (k * dims[1] + j) * dims[0] + i;
                        bits[bit / 64] |= 1 << (bit % 64);
                        runs.push(([x, y, z], [x + 1, y + 1, z + 1]));
                    }
                }
            }
        }
        let ring = CollisionSnapshot::from_ring(bounds, dims, &bits);
        let dense = CollisionSnapshot::from_runs(low, [69, 5, 4], runs);
        bits.fill(0);
        assert_eq!(ring, dense);
        for (a, b) in [(low, end), ([-7, -3, 0], [19, 1, 2])] {
            let mut expected = Vec::new();
            let mut actual = Vec::new();
            dense.visit_occupied(a, b, |g| expected.push(g));
            ring.visit_occupied(a, b, |g| actual.push(g));
            assert_eq!(actual, expected);
        }
    }

    #[test]
    fn packed_union_matches_boxes_across_words_rows_and_clipped_boundaries() {
        let low = [-7, 3, -2];
        let dims = [137, 5, 4];
        let runs = [([-20, 3, -2], [70, 6, 0]), ([55, 4, -1], [155, 9, 4])];
        let mask = CollisionSnapshot::from_runs(low, dims, runs);
        for z in -3..4 {
            for y in 2..10 {
                for x in -20..155 {
                    let at = [x, y, z];
                    let inside = (0..3).all(|i| at[i] >= low[i] && at[i] < low[i] + dims[i] as i64);
                    let occupied = runs
                        .iter()
                        .any(|(a, b)| (0..3).all(|i| at[i] >= a[i] && at[i] < b[i]));
                    assert_eq!(mask.is_occupied(at), inside && occupied, "{at:?}");
                }
            }
        }
        assert!(mask.shares_data(&mask.clone()));
    }
}
