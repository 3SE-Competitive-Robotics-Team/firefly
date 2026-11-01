// Copyright (C) 2008-2011 Gael Guennebaud <gael.guennebaud@inria.fr>.
// Copyright (C) 2009 Keir Mierle <mierle@gmail.com>.
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>.
// Copyright (C) 2011 Timothy E. Holy <tim.holy@gmail.com>.
// SPDX-License-Identifier: MPL-2.0
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain
// one at https://mozilla.org/MPL/2.0/.
//! Real dynamic specialization of Eigen 5.0.1 lower-triangle pivoted LDLT.
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector};

#[derive(Clone, Debug)]
pub struct Ldlt<S: BasaltScalar> {
    matrix: DMatrix<S>,
    transpositions: Vec<usize>,
}
impl<S: BasaltScalar> Ldlt<S> {
    /// Factor using only the lower triangle and diagonal, as Eigen does.
    /// # Panics
    /// Panics if the matrix is not square.
    #[must_use]
    pub fn new(mut matrix: DMatrix<S>) -> Self {
        assert!(matrix.is_square());
        let size = matrix.nrows();
        let mut transpositions: Vec<_> = (0..size).collect();
        for k in 0..size {
            let mut pivot = k;
            for i in k + 1..size {
                if matrix[(i, i)].abs() > matrix[(pivot, pivot)].abs() {
                    pivot = i;
                }
            }
            transpositions[k] = pivot;
            if k != pivot {
                for j in 0..k {
                    matrix.swap((k, j), (pivot, j));
                }
                for i in pivot + 1..size {
                    matrix.swap((i, k), (i, pivot));
                }
                matrix.swap((k, k), (pivot, pivot));
                for i in k + 1..pivot {
                    matrix.swap((i, k), (pivot, i));
                }
            }
            let temp = DVector::from_fn(k, |j, _| matrix[(j, j)] * matrix[(k, j)]);
            let mut update = S::zero();
            for j in 0..k {
                update += matrix[(k, j)] * temp[j];
            }
            matrix[(k, k)] -= update;
            for i in k + 1..size {
                let mut update = S::zero();
                for j in 0..k {
                    update += matrix[(i, j)] * temp[j];
                }
                matrix[(i, k)] -= update;
            }
            let diagonal = matrix[(k, k)];
            let valid = diagonal.abs() > S::zero();
            if k == 0 && !valid {
                transpositions = (0..size).collect();
                break;
            }
            if valid {
                for i in k + 1..size {
                    matrix[(i, k)] /= diagonal;
                }
            }
        }
        Self {
            matrix,
            transpositions,
        }
    }
    #[must_use]
    pub fn diagonal(&self) -> DVector<S> {
        self.matrix.diagonal()
    }
    #[must_use]
    pub fn lower(&self) -> DMatrix<S> {
        DMatrix::from_fn(self.matrix.nrows(), self.matrix.ncols(), |r, c| {
            match r.cmp(&c) {
                std::cmp::Ordering::Greater => self.matrix[(r, c)],
                std::cmp::Ordering::Equal => S::one(),
                std::cmp::Ordering::Less => S::zero(),
            }
        })
    }
    /// Apply the pivot transpositions P on the left.
    /// # Panics
    /// Panics if the matrix row count differs from the decomposition size.
    pub fn permute_rows(&self, rhs: &mut DMatrix<S>) {
        assert_eq!(rhs.nrows(), self.matrix.nrows());
        for (row, &pivot) in self.transpositions.iter().enumerate() {
            rhs.swap_rows(row, pivot);
        }
    }
    /// Solve L X = B for the unit lower-triangular factor.
    /// # Panics
    /// Panics if the matrix row count differs from the decomposition size.
    pub fn solve_lower(&self, rhs: &mut DMatrix<S>) {
        assert_eq!(rhs.nrows(), self.matrix.nrows());
        for row in 0..rhs.nrows() {
            for col in 0..rhs.ncols() {
                let mut update = S::zero();
                for j in 0..row {
                    update += self.matrix[(row, j)] * rhs[(j, col)];
                }
                rhs[(row, col)] -= update;
            }
        }
    }
    /// Solve using Eigen's diagonal pseudo-inverse cutoff at the smallest normal value.
    /// # Panics
    /// Panics if the right-hand side row count differs from the decomposition size.
    #[must_use]
    pub fn solve(&self, rhs: &DMatrix<S>) -> DMatrix<S> {
        let mut result = rhs.clone();
        self.permute_rows(&mut result);
        if result.ncols() == 1 {
            self.solve_lower(&mut result);
        } else {
            // Eigen's column-major multiple-RHS triangular kernel updates each
            // remaining entry after each pivot, rather than accumulating a dot.
            let panel = if std::mem::size_of::<S>() == 4 { 8 } else { 4 };
            for start in (0..result.nrows()).step_by(panel) {
                let end = (start + panel).min(result.nrows());
                for pivot in start..end {
                    for col in 0..result.ncols() {
                        let value = result[(pivot, col)];
                        for row in pivot + 1..end {
                            result[(row, col)] -= value * self.matrix[(row, pivot)];
                        }
                    }
                }
                for row in end..result.nrows() {
                    for col in 0..result.ncols() {
                        let mut update = S::zero();
                        for pivot in start..end {
                            update += self.matrix[(row, pivot)] * result[(pivot, col)];
                        }
                        result[(row, col)] -= update;
                    }
                }
            }
        }
        for row in 0..result.nrows() {
            let diagonal = self.matrix[(row, row)];
            for col in 0..result.ncols() {
                result[(row, col)] = if diagonal.abs() > S::min_positive_value() {
                    result[(row, col)] / diagonal
                } else {
                    S::zero()
                };
            }
        }
        for row in (0..result.nrows()).rev() {
            for col in 0..result.ncols() {
                let mut update = S::zero();
                for j in row + 1..result.nrows() {
                    update += self.matrix[(j, row)] * result[(j, col)];
                }
                result[(row, col)] -= update;
            }
        }
        for (row, &pivot) in self.transpositions.iter().enumerate().rev() {
            result.swap_rows(row, pivot);
        }
        result
    }
    /// Vector specialization of [`Self::solve`].
    /// # Panics
    /// Panics if the vector length differs from the decomposition size.
    #[must_use]
    pub fn solve_vector(&self, rhs: &DVector<S>) -> DVector<S> {
        self.solve(&DMatrix::from_column_slice(rhs.len(), 1, rhs.as_slice()))
            .column(0)
            .into_owned()
    }
}
