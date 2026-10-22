/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENSE file for details
 */
//! Kruskal minimum spanning forest corresponding to `MstGraph.hpp`.
//! The source file's title says maximum; its ascending edge sort computes minimum.
#[derive(Clone, Debug)]
pub struct MstGraph {
    pub vertices: usize,
    pub expected_edges: usize,
    pub edges: Vec<(i32, (usize, usize))>,
}
impl MstGraph {
    #[must_use]
    pub fn new(vertices: usize, expected_edges: usize) -> Self {
        Self {
            vertices,
            expected_edges,
            edges: Vec::new(),
        }
    }
    pub fn add_edge(&mut self, u: usize, v: usize, weight: i32) {
        self.edges.push((weight, (u, v)));
    }
    /// Appends to `output`; disconnected graphs produce a forest. Vertex V is
    /// accepted because the source allocates and initializes V+1 entries.
    pub fn kruskal_mst(&mut self, output: &mut Vec<(usize, usize)>) -> Option<i32> {
        if self
            .edges
            .iter()
            .any(|&(_, (u, v))| u > self.vertices || v > self.vertices)
        {
            return None;
        }
        self.edges.sort_unstable();
        let mut sets = DisjointSets::new(self.vertices);
        let mut weight = 0i32;
        for &(cost, (u, v)) in &self.edges {
            let a = sets.find(u);
            let b = sets.find(v);
            if a != b {
                weight = weight.checked_add(cost)?;
                output.push((u, v));
                sets.merge(a, b);
            }
        }
        Some(weight)
    }
}
#[derive(Clone, Debug)]
pub struct DisjointSets {
    pub parent: Vec<usize>,
    pub rank: Vec<usize>,
    pub n: usize,
}
impl DisjointSets {
    #[must_use]
    pub fn new(n: usize) -> Self {
        Self {
            parent: (0..=n).collect(),
            rank: vec![0; n + 1],
            n,
        }
    }
    pub fn find(&mut self, u: usize) -> usize {
        if self.parent[u] != u {
            self.parent[u] = self.find(self.parent[u]);
        }
        self.parent[u]
    }
    pub fn merge(&mut self, x: usize, y: usize) {
        let x = self.find(x);
        let y = self.find(y);
        if self.rank[x] > self.rank[y] {
            self.parent[y] = x;
        } else {
            self.parent[x] = y;
        }
        if self.rank[x] == self.rank[y] {
            self.rank[y] += 1;
        }
    }
}
