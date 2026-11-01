// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// See docs/licenses/README.md.
//! Dense accumulator from `include/basalt/optimization/accumulator.h`,
//! original Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

use crate::utils::ldlt::Ldlt;
use crate::utils::sparse_ldlt::{SparseLdlt, SparseMatrix, conjugate_gradient};
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector, SMatrix, SVector};
use std::{collections::HashMap, time::Instant};

/// Statically sized block insertion shared by the upstream dense and sparse accumulators.
pub trait BlockAccumulator<S: BasaltScalar> {
    fn add_h<const R: usize, const C: usize>(
        &mut self,
        row: usize,
        col: usize,
        block: &SMatrix<S, R, C>,
    );
    fn add_b<const R: usize>(&mut self, row: usize, block: &SVector<S, R>);
}

/// Reduction and solve operations used by the original calibration templates.
pub trait SolverAccumulator<S: BasaltScalar>: BlockAccumulator<S> + Default + Send {
    fn reset(&mut self, size: usize);
    fn join(&mut self, other: &Self);
    fn setup_solver(&mut self);
    fn h_diagonal(&self) -> DVector<S>;
    fn b(&self) -> &DVector<S>;
    fn solve(&self, diagonal: Option<&DVector<S>>) -> DVector<S>;
}
macro_rules! solver_accumulator {
    ($kind:ident) => {
        impl<S: BasaltScalar> SolverAccumulator<S> for $kind<S> {
            fn reset(&mut self, size: usize) {
                Self::reset(self, size);
            }
            fn join(&mut self, other: &Self) {
                Self::join(self, other);
            }
            fn setup_solver(&mut self) {
                Self::setup_solver(self);
            }
            fn h_diagonal(&self) -> DVector<S> {
                Self::h_diagonal(self)
            }
            fn b(&self) -> &DVector<S> {
                Self::b(self)
            }
            fn solve(&self, diagonal: Option<&DVector<S>>) -> DVector<S> {
                Self::solve(self, diagonal)
            }
        }
    };
}
solver_accumulator!(DenseAccumulator);
solver_accumulator!(SparseHashAccumulator);
impl<S: BasaltScalar> BlockAccumulator<S> for DenseAccumulator<S> {
    fn add_h<const R: usize, const C: usize>(
        &mut self,
        row: usize,
        col: usize,
        block: &SMatrix<S, R, C>,
    ) {
        Self::add_h(self, row, col, block);
    }
    fn add_b<const R: usize>(&mut self, row: usize, block: &SVector<S, R>) {
        Self::add_b(self, row, block);
    }
}
impl<S: BasaltScalar> BlockAccumulator<S> for SparseHashAccumulator<S> {
    fn add_h<const R: usize, const C: usize>(
        &mut self,
        row: usize,
        col: usize,
        block: &SMatrix<S, R, C>,
    ) {
        Self::add_h(self, row, col, block);
    }
    fn add_b<const R: usize>(&mut self, row: usize, block: &SVector<S, R>) {
        Self::add_b(self, row, block);
    }
}

#[derive(Clone, Debug)]
pub struct DenseAccumulator<S: BasaltScalar = f64> {
    h: DMatrix<S>,
    b: DVector<S>,
}
impl<S: BasaltScalar> Default for DenseAccumulator<S> {
    fn default() -> Self {
        Self {
            h: DMatrix::zeros(0, 0),
            b: DVector::zeros(0),
        }
    }
}
impl<S: BasaltScalar> DenseAccumulator<S> {
    pub fn reset(&mut self, size: usize) {
        self.h = DMatrix::zeros(size, size);
        self.b = DVector::zeros(size);
    }
    /// # Panics
    /// Requires the block to lie inside the accumulated matrix.
    pub fn add_h<const R: usize, const C: usize>(
        &mut self,
        row: usize,
        column: usize,
        block: &SMatrix<S, R, C>,
    ) {
        assert!(row <= self.h.nrows() && R <= self.h.nrows() - row);
        assert!(column <= self.h.ncols() && C <= self.h.ncols() - column);
        for r in 0..R {
            for c in 0..C {
                self.h[(row + r, column + c)] += block[(r, c)];
            }
        }
    }
    /// # Panics
    /// Requires the block to lie inside the accumulated vector.
    pub fn add_b<const R: usize>(&mut self, row: usize, block: &SVector<S, R>) {
        assert!(row <= self.b.len() && R <= self.b.len() - row);
        for r in 0..R {
            self.b[row + r] += block[r];
        }
    }
    /// Solves `(H + diag(damping)) x = b`, preserving the source sign convention.
    /// # Panics
    /// Requires a damping vector matching the system dimension.
    #[must_use]
    pub fn solve(&self, diagonal: Option<&DVector<S>>) -> DVector<S> {
        let mut matrix = self.h.clone();
        if let Some(diagonal) = diagonal {
            assert_eq!(diagonal.len(), self.h.nrows());
            for i in 0..diagonal.len() {
                matrix[(i, i)] += diagonal[i];
            }
        }
        Ldlt::new(matrix).solve_vector(&self.b)
    }
    /// # Panics
    /// Requires identically sized systems.
    pub fn join(&mut self, other: &Self) {
        self.h += &other.h;
        self.b += &other.b;
    }
    pub fn setup_solver(&mut self) {}
    #[must_use]
    pub fn h_diagonal(&self) -> DVector<S> {
        self.h.diagonal()
    }
    #[must_use]
    pub fn h(&self) -> &DMatrix<S> {
        &self.h
    }
    pub fn h_mut(&mut self) -> &mut DMatrix<S> {
        &mut self.h
    }
    #[must_use]
    pub fn b(&self) -> &DVector<S> {
        &self.b
    }
    pub fn b_mut(&mut self) -> &mut DVector<S> {
        &mut self.b
    }
    pub fn print(&self) {
        log::info!("H\n{:.2}\nb\n{:.2}", self.h, self.b.transpose());
    }
}

/// Hash-map block assembly followed by sparse AMD/LDLT or diagonal-preconditioned CG.
/// The original CG branch hard-codes double precision; the generic Rust branch
/// extends that same algorithm to f32 without an implicit dense conversion.
pub struct SparseHashAccumulator<S: BasaltScalar> {
    blocks: HashMap<[usize; 4], DMatrix<S>>,
    b: DVector<S>,
    matrix: SparseMatrix<S>,
    pub tolerance: f64,
    pub iterative_solver: bool,
    pub print_info: bool,
}
impl<S: BasaltScalar> Default for SparseHashAccumulator<S> {
    fn default() -> Self {
        Self {
            blocks: HashMap::new(),
            b: DVector::zeros(0),
            matrix: SparseMatrix::zeros(0),
            tolerance: 1e-4,
            iterative_solver: false,
            print_info: false,
        }
    }
}
impl<S: BasaltScalar> SparseHashAccumulator<S> {
    pub fn add_h<const R: usize, const C: usize>(
        &mut self,
        row: usize,
        col: usize,
        block: &SMatrix<S, R, C>,
    ) {
        let entry = self
            .blocks
            .entry([row, col, R, C])
            .or_insert_with(|| DMatrix::zeros(R, C));
        *entry += block;
    }
    /// # Panics
    /// Requires the residual block to fit the optimization dimension.
    pub fn add_b<const R: usize>(&mut self, row: usize, block: &SVector<S, R>) {
        for i in 0..R {
            self.b[row + i] += block[i];
        }
    }
    pub fn reset(&mut self, size: usize) {
        self.blocks.clear();
        self.b = DVector::zeros(size);
    }
    pub fn join(&mut self, other: &Self) {
        for (&key, block) in &other.blocks {
            *self
                .blocks
                .entry(key)
                .or_insert_with(|| DMatrix::zeros(block.nrows(), block.ncols())) += block;
        }
        self.b += &other.b;
    }
    /// Assemble all stored entries, including structural zero entries and diagonal sentinels.
    /// # Panics
    /// Requires every accumulated block to fit the optimization dimension.
    pub fn setup_solver(&mut self) {
        let mut matrix = SparseMatrix::zeros(self.b.len());
        for (&[row, col, rows, cols], block) in &self.blocks {
            for r in 0..rows {
                for c in 0..cols {
                    matrix.add(row + r, col + c, block[(r, c)]);
                }
            }
        }
        for i in 0..self.b.len() {
            matrix.add(i, i, S::from_f64_lossy(f64::MIN_POSITIVE));
        }
        self.matrix = matrix;
    }
    #[must_use]
    pub fn h_diagonal(&self) -> DVector<S> {
        self.matrix.diagonal()
    }
    #[must_use]
    pub fn b(&self) -> &DVector<S> {
        &self.b
    }
    pub fn b_mut(&mut self) -> &mut DVector<S> {
        &mut self.b
    }
    #[must_use]
    pub fn sparse_matrix(&self) -> &SparseMatrix<S> {
        &self.matrix
    }
    /// # Panics
    /// Requires setup, a matching damping vector, and nonzero sparse LDLT pivots.
    #[must_use]
    pub fn solve(&self, diagonal: Option<&DVector<S>>) -> DVector<S> {
        let start = Instant::now();
        assert_eq!(self.matrix.size, self.b.len());
        let mut matrix = self.matrix.clone();
        if let Some(diagonal) = diagonal {
            assert_eq!(diagonal.len(), matrix.size);
            for i in 0..matrix.size {
                matrix.add(i, i, diagonal[i]);
            }
        }
        let solution = if self.iterative_solver {
            conjugate_gradient(&matrix, &self.b, S::from_f64_lossy(self.tolerance))
        } else {
            SparseLdlt::new(&matrix).solve(&self.b)
        };
        if self.print_info {
            log::info!(
                "Solving sparse linear system: {}s",
                start.elapsed().as_secs_f64()
            );
        }
        solution
    }
}
