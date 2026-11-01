// SPDX-License-Identifier: MPL-2.0
// Copyright (c) 2016 Pierre MOULON, 2018 Nikolaus DEMMEL.
// Source: VladyslavUsenko/basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc,
// include/basalt/utils/union_find.h (adapted from OpenMVG).
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. https://mozilla.org/MPL/2.0/

/// Disjoint sets with union by rank and recursive path compression.
#[derive(Clone, Debug, Default)]
pub struct UnionFind {
    pub cc_parent: Vec<u32>,
    pub cc_rank: Vec<u32>,
    pub cc_size: Vec<u32>,
}

impl UnionFind {
    #[must_use]
    pub const fn invalid_index() -> u32 {
        u32::MAX
    }

    /// Resizes the arrays; existing size and rank entries retain their values.
    pub fn init_sets(&mut self, count: u32) {
        self.cc_size.resize(count as usize, 1);
        self.cc_parent.resize(count as usize, 0);
        for (index, parent) in self.cc_parent.iter_mut().enumerate() {
            *parent = index as u32;
        }
        self.cc_rank.resize(count as usize, 0);
    }

    #[must_use]
    pub fn num_nodes(&self) -> usize {
        self.cc_size.len()
    }

    /// Returns the representative or the invalid marker of a rejected set.
    /// # Panics
    /// Panics when `index` or a stored parent is outside the initialized forest.
    pub fn find(&mut self, index: u32) -> u32 {
        let slot = index as usize;
        if self.cc_parent[slot] != index && self.cc_parent[slot] != Self::invalid_index() {
            self.cc_parent[slot] = self.find(self.cc_parent[slot]);
        }
        self.cc_parent[slot]
    }

    /// Joins valid sets, preserving the first root on equal rank.
    /// # Panics
    /// Panics for out-of-range nodes or distinct sets marked invalid.
    pub fn union(&mut self, left: u32, right: u32) {
        let left = self.find(left) as usize;
        let right = self.find(right) as usize;
        if left == right {
            return;
        }
        if self.cc_rank[left] < self.cc_rank[right] {
            self.cc_parent[left] = right as u32;
            self.cc_size[right] += self.cc_size[left];
        } else {
            self.cc_parent[right] = left as u32;
            self.cc_size[left] += self.cc_size[right];
            if self.cc_rank[left] == self.cc_rank[right] {
                self.cc_rank[left] += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn matches_cpp_oracle() {
        let expected: serde_json::Value =
            serde_json::from_str(include_str!("../../tests/reference/utils.json")).unwrap();
        let state = |sets: &UnionFind| serde_json::json!({"parent": sets.cc_parent, "rank": sets.cc_rank, "size": sets.cc_size});
        let mut sets = UnionFind::default();
        sets.init_sets(8);
        assert_eq!(state(&sets), expected["union_find"][0]);
        for (index, (a, b)) in [
            (0, 1),
            (2, 3),
            (1, 3),
            (4, 5),
            (6, 7),
            (5, 7),
            (3, 7),
            (0, 7),
        ]
        .into_iter()
        .enumerate()
        {
            sets.union(a, b);
            assert_eq!(state(&sets), expected["union_find"][index + 1]);
        }
        for i in 0..8 {
            sets.find(i);
        }
        assert_eq!(state(&sets), expected["compressed"]);
        sets.init_sets(4);
        assert_eq!(state(&sets), expected["reinitialized"]);
    }
    #[test]
    fn ranks_sizes_compression_and_reinitialization_match_source() {
        let mut sets = UnionFind::default();
        sets.init_sets(5);
        sets.union(0, 1);
        sets.union(2, 3);
        sets.union(0, 2);
        assert_eq!(sets.find(3), 0);
        assert_eq!(sets.cc_parent, vec![0, 0, 0, 0, 4]);
        assert_eq!(sets.cc_rank, vec![2, 0, 1, 0, 0]);
        assert_eq!(sets.cc_size, vec![4, 1, 2, 1, 1]);
        sets.init_sets(6);
        assert_eq!(sets.cc_parent, vec![0, 1, 2, 3, 4, 5]);
        assert_eq!(sets.cc_rank, vec![2, 0, 1, 0, 0, 0]);
        assert_eq!(sets.cc_size, vec![4, 1, 2, 1, 1, 1]);
        sets.cc_parent[4] = UnionFind::invalid_index();
        assert_eq!(sets.find(4), UnionFind::invalid_index());
    }
}
