//! Immutable ray-observation mask in the depth mapper's world-anchored lattice.
use std::sync::Arc;

use nalgebra::Vector3;

/// Free evidence is separate from obstacle inflation. Unseen and evicted cells are false.
#[derive(Clone, Debug)]
pub struct ObservedSpace {
    resolution: f64,
    low: [i32; 3],
    dims: [i32; 3],
    origin: [i32; 3],
    words: Arc<[u64]>,
}

impl ObservedSpace {
    /// Import the mapper's x-fastest ring buffer without expanding it into point objects.
    ///
    /// # Panics
    /// Panics if resolution or dimensions are nonpositive, or the buffer size disagrees.
    #[must_use]
    pub fn from_buffer(
        resolution: f64,
        low: [i32; 3],
        dims: [i32; 3],
        origin: [i32; 3],
        free: &[bool],
    ) -> Self {
        assert!(resolution.is_finite() && resolution > 0.);
        assert!(dims.iter().all(|n| *n > 0));
        assert_eq!(
            dims.iter().map(|n| *n as usize).product::<usize>(),
            free.len()
        );
        let words = free
            .chunks(64)
            .map(|chunk| {
                chunk
                    .iter()
                    .enumerate()
                    .fold(0, |bits, (i, value)| bits | (u64::from(*value) << i))
            })
            .collect();
        Self {
            resolution,
            low,
            dims,
            origin,
            words,
        }
    }

    #[must_use]
    pub fn is_free(&self, point: Vector3<f64>) -> bool {
        if point.iter().any(|v| !v.is_finite()) {
            return false;
        }
        let index = point.map(|v| (v / self.resolution).floor() as i64);
        if (0..3).any(|i| {
            index[i] < i64::from(self.low[i])
                || index[i] >= i64::from(self.low[i]) + i64::from(self.dims[i])
        }) {
            return false;
        }
        let local: [usize; 3] = std::array::from_fn(|i| {
            (index[i] - i64::from(self.origin[i])).rem_euclid(i64::from(self.dims[i])) as usize
        });
        let address =
            (local[2] * self.dims[1] as usize + local[1]) * self.dims[0] as usize + local[0];
        self.words[address / 64] & (1 << (address % 64)) != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_addressing_keeps_unknown_outside_and_distinguishes_free() {
        let mut free = vec![false; 64];
        free[0] = true;
        let mask = ObservedSpace::from_buffer(1., [-2; 3], [4; 3], [0; 3], &free);
        assert!(mask.is_free(Vector3::repeat(0.5)));
        assert!(!mask.is_free(Vector3::repeat(-0.5)));
        assert!(!mask.is_free(Vector3::repeat(4.5)));
        free[0] = false;
        assert!(mask.is_free(Vector3::repeat(0.5)));
    }
}
