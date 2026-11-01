// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/linearization/block_diagonal.hpp`,
//! original Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector};
use std::collections::{HashMap, hash_map::Entry};
pub type IndexedBlocks<S> = HashMap<usize, DMatrix<S>>;

/// Apply diagonal Jacobian scaling as D H D.
/// # Panics
/// Requires consistent block and vector dimensions.
pub fn scale_jacobians<S: BasaltScalar>(
    blocks: &mut IndexedBlocks<S>,
    num_blocks: usize,
    block_size: usize,
    scaling: &DVector<S>,
) {
    assert_eq!(num_blocks * block_size, scaling.len());
    for (&index, block) in blocks {
        assert!(index < num_blocks);
        assert_eq!(block.shape(), (block_size, block_size));
        for row in 0..block_size {
            for col in 0..block_size {
                block[(row, col)] = scaling[index * block_size + row]
                    * block[(row, col)]
                    * scaling[index * block_size + col];
            }
        }
    }
}
/// Add a full diagonal, creating every absent block.
/// # Panics
/// Requires consistent block and vector dimensions.
pub fn add_diagonal<S: BasaltScalar>(
    blocks: &mut IndexedBlocks<S>,
    num_blocks: usize,
    block_size: usize,
    diagonal: &DVector<S>,
) {
    assert_eq!(num_blocks * block_size, diagonal.len());
    for index in 0..num_blocks {
        let block = blocks
            .entry(index)
            .or_insert_with(|| DMatrix::zeros(block_size, block_size));
        assert_eq!(block.shape(), (block_size, block_size));
        for i in 0..block_size {
            block[(i, i)] += diagonal[index * block_size + i];
        }
    }
}
#[derive(Clone, Debug)]
pub struct BlockDiagonalAccumulator<S: BasaltScalar> {
    pub block_diagonal: IndexedBlocks<S>,
}
impl<S: BasaltScalar> Default for BlockDiagonalAccumulator<S> {
    fn default() -> Self {
        Self {
            block_diagonal: HashMap::new(),
        }
    }
}
impl<S: BasaltScalar> BlockDiagonalAccumulator<S> {
    pub fn add(&mut self, index: usize, block: DMatrix<S>) {
        match self.block_diagonal.entry(index) {
            Entry::Vacant(entry) => {
                entry.insert(block);
            }
            Entry::Occupied(mut entry) => {
                *entry.get_mut() += block;
            }
        }
    }
    pub fn add_diag(&mut self, num_blocks: usize, block_size: usize, diagonal: &DVector<S>) {
        add_diagonal(&mut self.block_diagonal, num_blocks, block_size, diagonal);
    }
    /// Merge blocks; newly inserted entries move out of the source accumulator.
    pub fn join(&mut self, other: &mut Self) {
        for (&index, block) in &mut other.block_diagonal {
            match self.block_diagonal.entry(index) {
                Entry::Vacant(entry) => {
                    entry.insert(std::mem::take(block));
                }
                Entry::Occupied(mut entry) => {
                    *entry.get_mut() += &*block;
                }
            }
        }
    }
}
