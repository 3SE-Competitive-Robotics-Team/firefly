//! Immutable occupancy in the owning `GridMap`'s global voxel lattice.
use std::sync::Arc;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CollisionSnapshot {
    low: [i64; 3],
    dims: [usize; 3],
    words: Arc<[u64]>,
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
            words: words.into(),
        }
    }

    #[must_use]
    pub fn is_occupied(&self, global: [i64; 3]) -> bool {
        let local: [i64; 3] = std::array::from_fn(|i| global[i] - self.low[i]);
        if (0..3).any(|i| local[i] < 0 || local[i] >= self.dims[i] as i64) {
            return false;
        }
        let address = (local[2] as usize * self.dims[1] + local[1] as usize) * self.dims[0]
            + local[0] as usize;
        self.words[address / 64] & (1 << (address % 64)) != 0
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
                let row = (z * self.dims[1] + y) * self.dims[0];
                let mut x = low[0];
                while x < end[0] {
                    let at = row + x;
                    let count = (64 - at % 64).min(end[0] - x);
                    let mut bits = (self.words[at / 64] >> (at % 64)) & (u64::MAX >> (64 - count));
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
        self.low == other.low && self.dims == other.dims && Arc::ptr_eq(&self.words, &other.words)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
