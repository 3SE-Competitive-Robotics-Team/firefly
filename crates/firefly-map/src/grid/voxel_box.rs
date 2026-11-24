/// Half-open bounds in the world-anchored integer lattice.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VoxelBox {
    pub(crate) low: [i64; 3],
    pub(crate) end: [i64; 3],
}

impl VoxelBox {
    /// Center a nonempty window of the given dimensions on a global voxel.
    #[must_use]
    pub fn centered(center: [i64; 3], dims: [usize; 3]) -> Option<Self> {
        let mut bounds = Self {
            low: center,
            end: center,
        };
        for (axis, dim) in dims.into_iter().enumerate() {
            let width = i64::try_from(dim).ok().filter(|&n| n > 0)?;
            bounds.low[axis] = center[axis].checked_sub(width / 2)?;
            bounds.end[axis] = bounds.low[axis].checked_add(width)?;
        }
        Some(bounds)
    }

    /// Inclusive lower corner in global voxel coordinates.
    #[must_use]
    pub const fn low(self) -> [i64; 3] {
        self.low
    }

    /// Exclusive upper corner in global voxel coordinates.
    #[must_use]
    pub const fn end(self) -> [i64; 3] {
        self.end
    }

    pub(crate) fn dimensions(self) -> [i64; 3] {
        std::array::from_fn(|i| self.end[i] - self.low[i])
    }

    #[must_use]
    pub fn contains(self, point: [i64; 3]) -> bool {
        (0..3).all(|i| (self.low[i]..self.end[i]).contains(&point[i]))
    }

    #[must_use]
    pub fn intersection(self, other: Self) -> Option<Self> {
        let low = std::array::from_fn(|i| self.low[i].max(other.low[i]));
        let end = std::array::from_fn(|i| self.end[i].min(other.end[i]));
        (0..3).all(|i| low[i] < end[i]).then_some(Self { low, end })
    }

    /// Peel off disjoint slabs, leaving the shared intersection in the middle.
    pub(crate) fn visit_difference(
        mut self,
        intersection: Option<Self>,
        mut visit: impl FnMut(Self),
    ) {
        let Some(shared) = intersection else {
            visit(self);
            return;
        };
        for axis in 0..3 {
            if self.low[axis] < shared.low[axis] {
                let mut slab = self;
                slab.end[axis] = shared.low[axis];
                visit(slab);
                self.low[axis] = shared.low[axis];
            }
            if shared.end[axis] < self.end[axis] {
                let mut slab = self;
                slab.low[axis] = shared.end[axis];
                visit(slab);
                self.end[axis] = shared.end[axis];
            }
        }
    }

    /// The unique multiple of each ring dimension within the corresponding axis.
    #[must_use]
    pub(crate) fn ring_origin(self) -> [i64; 3] {
        std::array::from_fn(|i| {
            let width = self.end[i] - self.low[i];
            let last = self.end[i] - 1;
            last - last.rem_euclid(width)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::VoxelBox;
    use std::collections::HashSet;

    fn cells(bounds: VoxelBox) -> impl Iterator<Item = [i64; 3]> {
        (bounds.low[0]..bounds.end[0]).flat_map(move |x| {
            (bounds.low[1]..bounds.end[1])
                .flat_map(move |y| (bounds.low[2]..bounds.end[2]).map(move |z| [x, y, z]))
        })
    }

    #[test]
    fn partition_visits_every_changed_cell_exactly_once() {
        let before = VoxelBox {
            low: [0; 3],
            end: [8, 5, 3],
        };
        let original: HashSet<_> = cells(before).collect();
        for x in [-8, -3, 0, 3, 8] {
            for y in [-5, -2, 0, 2, 5] {
                for z in [-3, -1, 0, 1, 3] {
                    let after = VoxelBox {
                        low: [x, y, z],
                        end: [x + 8, y + 5, z + 3],
                    };
                    let moved: HashSet<_> = cells(after).collect();
                    let shared = before.intersection(after);
                    let retained: HashSet<_> = shared.into_iter().flat_map(cells).collect();
                    assert_eq!(retained, original.intersection(&moved).copied().collect());
                    for (bounds, source, other) in
                        [(before, &original, &moved), (after, &moved, &original)]
                    {
                        let mut visited = HashSet::new();
                        bounds.visit_difference(shared, |slab| {
                            for cell in cells(slab) {
                                assert!(visited.insert(cell), "duplicate voxel {cell:?}");
                            }
                        });
                        assert_eq!(visited, source.difference(other).copied().collect());
                    }
                }
            }
        }
    }
}
