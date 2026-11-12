// Copyright (C) 2016 Rasmus Munk Larsen <rmlarsen@google.com>.
// Copyright (C) 2010 Gael Guennebaud <gael.guennebaud@inria.fr>.
// SPDX-License-Identifier: MPL-2.0
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain
// one at https://mozilla.org/MPL/2.0/.
//! Real dynamic specialization of Eigen 5.0.1 `CompleteOrthogonalDecomposition`,
//! `ColPivHouseholderQR` and Householder operations, preserving rank thresholds.
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector, Dim, Matrix, StorageMut};

pub(crate) fn machine_epsilon<S: BasaltScalar>() -> S {
    S::from_f64_lossy(if size_of::<S>() == 4 {
        f64::from(f32::EPSILON)
    } else {
        f64::EPSILON
    })
}

/// Store the essential Householder vector in the tail; leave the first entry.
pub(crate) fn make_householder<S: BasaltScalar>(vector: &mut DVector<S>) -> (S, S) {
    assert!(!vector.is_empty());
    let mut tail_squared = S::zero();
    for &value in vector.iter().skip(1) {
        tail_squared += value * value;
    }
    let first = vector[0];
    if tail_squared <= S::min_positive_value() {
        for value in vector.iter_mut().skip(1) {
            *value = S::zero();
        }
        (S::zero(), first)
    } else {
        let mut beta = (first * first + tail_squared).sqrt();
        if first >= S::zero() {
            beta = -beta;
        }
        for value in vector.iter_mut().skip(1) {
            *value /= first - beta;
        }
        ((beta - first) / beta, beta)
    }
}

pub(crate) fn apply_householder_left<S: BasaltScalar, R: Dim, C: Dim, M: StorageMut<S, R, C>>(
    matrix: &mut Matrix<S, R, C, M>,
    row: usize,
    column: usize,
    essential: &DVector<S>,
    tau: S,
) {
    assert_eq!(matrix.nrows() - row, essential.len() + 1);
    if essential.is_empty() {
        for c in column..matrix.ncols() {
            matrix[(row, c)] *= S::one() - tau;
        }
    } else if tau != S::zero() {
        for c in column..matrix.ncols() {
            let mut temporary = S::zero();
            for r in 0..essential.len() {
                temporary += essential[r] * matrix[(row + 1 + r, c)];
            }
            temporary += matrix[(row, c)];
            matrix[(row, c)] -= tau * temporary;
            for r in 0..essential.len() {
                matrix[(row + 1 + r, c)] -= tau * essential[r] * temporary;
            }
        }
    }
}

/// Column-pivoted QR followed by right Householder completion.
#[derive(Clone, Debug)]
pub struct CompleteOrthogonalDecomposition<S: BasaltScalar> {
    matrix: DMatrix<S>,
    q_coefficients: DVector<S>,
    z_coefficients: DVector<S>,
    permutation: Vec<usize>,
    rank: usize,
}
impl<S: BasaltScalar> CompleteOrthogonalDecomposition<S> {
    /// Use Eigen's default `epsilon * min(rows,cols) * max_pivot` rank cutoff.
    #[must_use]
    #[expect(
        clippy::too_many_lines,
        reason = "Preserves the upstream CPQR and right-completion sequence"
    )]
    #[expect(
        clippy::missing_panics_doc,
        reason = "The sealed f32/f64 scalars represent every usize as a finite value"
    )]
    pub fn new(mut matrix: DMatrix<S>) -> Self {
        let (rows, cols) = (matrix.nrows(), matrix.ncols());
        let size = rows.min(cols);
        let mut q_coefficients = DVector::zeros(size);
        let mut z_coefficients = DVector::zeros(size);
        let mut permutation: Vec<_> = (0..cols).collect();
        if size == 0 {
            return Self {
                matrix,
                q_coefficients,
                z_coefficients,
                permutation,
                rank: 0,
            };
        }
        let mut direct = DVector::<S>::from_fn(cols, |c, _| matrix.column(c).norm());
        let mut updated = direct.clone();
        let epsilon = machine_epsilon::<S>();
        let maximum = updated
            .iter()
            .copied()
            .fold(S::zero(), |a, b| if a < b { b } else { a });
        let threshold_helper =
            (maximum * epsilon) * (maximum * epsilon) / S::from_usize(rows).unwrap();
        let downdate_threshold = epsilon.sqrt();
        let mut nonzero_pivots = size;
        let mut max_pivot = S::zero();
        for k in 0..size {
            let mut pivot = k;
            for c in k + 1..cols {
                if updated[c] > updated[pivot] {
                    pivot = c;
                }
            }
            if nonzero_pivots == size
                && updated[pivot] * updated[pivot]
                    < threshold_helper * S::from_usize(rows - k).unwrap()
            {
                nonzero_pivots = k;
            }
            if pivot != k {
                matrix.swap_columns(k, pivot);
                updated.swap_rows(k, pivot);
                direct.swap_rows(k, pivot);
                permutation.swap(k, pivot);
            }
            let mut reflector = matrix.view((k, k), (rows - k, 1)).column(0).into_owned();
            let (tau, beta) = make_householder(&mut reflector);
            q_coefficients[k] = tau;
            matrix[(k, k)] = beta;
            for r in 1..reflector.len() {
                matrix[(k + r, k)] = reflector[r];
            }
            if beta.abs() > max_pivot {
                max_pivot = beta.abs();
            }
            apply_householder_left(
                &mut matrix,
                k,
                k + 1,
                &reflector.rows(1, reflector.len() - 1).into_owned(),
                tau,
            );
            for c in k + 1..cols {
                if updated[c] != S::zero() {
                    let relative = matrix[(k, c)].abs() / updated[c];
                    let mut temp = (S::one() + relative) * (S::one() - relative);
                    if temp < S::zero() {
                        temp = S::zero();
                    }
                    let ratio = updated[c] / direct[c];
                    let temp2 = temp * (ratio * ratio);
                    if temp2 <= downdate_threshold {
                        direct[c] = matrix.view((k + 1, c), (rows - k - 1, 1)).norm();
                        updated[c] = direct[c];
                    } else {
                        updated[c] *= temp.sqrt();
                    }
                }
            }
        }
        let rank_threshold = max_pivot * (epsilon * S::from_usize(size).unwrap());
        let rank = (0..nonzero_pivots)
            .filter(|&i| matrix[(i, i)].abs() > rank_threshold)
            .count();
        if rank < cols {
            for k in (0..rank).rev() {
                if k != rank - 1 {
                    for r in 0..=k {
                        matrix.swap((r, k), (r, rank - 1));
                    }
                }
                let mut reflector =
                    DVector::from_fn(cols - rank + 1, |c, _| matrix[(k, rank - 1 + c)]);
                let (tau, beta) = make_householder(&mut reflector);
                z_coefficients[k] = tau;
                matrix[(k, rank - 1)] = beta;
                for c in 1..reflector.len() {
                    matrix[(k, rank - 1 + c)] = reflector[c];
                }
                // Apply Z(k) from the right to the first k rows.
                if tau != S::zero() {
                    for r in 0..k {
                        let mut temporary = S::zero();
                        for c in 1..reflector.len() {
                            temporary += matrix[(r, rank - 1 + c)] * reflector[c];
                        }
                        temporary += matrix[(r, rank - 1)];
                        matrix[(r, rank - 1)] -= tau * temporary;
                        for c in 1..reflector.len() {
                            matrix[(r, rank - 1 + c)] -= tau * temporary * reflector[c];
                        }
                    }
                }
                if k != rank - 1 {
                    for r in 0..=k {
                        matrix.swap((r, k), (r, rank - 1));
                    }
                }
            }
        }
        Self {
            matrix,
            q_coefficients,
            z_coefficients,
            permutation,
            rank,
        }
    }
    #[must_use]
    pub fn rank(&self) -> usize {
        self.rank
    }
    /// Recover the CPQR preconditioner when right completion is unnecessary.
    /// Used by the original full-column-rank polynomial Jacobi SVD.
    pub(crate) fn full_rank_qr_factors(&self) -> (DMatrix<S>, DMatrix<S>, Vec<usize>) {
        let (rows, cols) = self.matrix.shape();
        assert!(rows >= cols && self.rank == cols);
        let r = DMatrix::from_fn(cols, cols, |i, j| {
            if i <= j {
                self.matrix[(i, j)]
            } else {
                S::zero()
            }
        });
        let mut q = DMatrix::<S>::identity(rows, cols);
        for k in (0..cols).rev() {
            let essential = self
                .matrix
                .view((k + 1, k), (rows - k - 1, 1))
                .column(0)
                .into_owned();
            apply_householder_left(&mut q, k, 0, &essential, self.q_coefficients[k]);
        }
        (q, r, self.permutation.clone())
    }
    /// Minimum-norm solution using the complete orthogonal factors.
    /// # Panics
    /// Panics if right-hand-side rows do not match the decomposed matrix.
    #[must_use]
    pub fn solve(&self, rhs: &DMatrix<S>) -> DMatrix<S> {
        assert_eq!(rhs.nrows(), self.matrix.nrows());
        let cols = self.matrix.ncols();
        let rank = self.rank;
        let mut solution = DMatrix::<S>::zeros(cols, rhs.ncols());
        if rank == 0 {
            return solution;
        }
        let mut transformed = rhs.clone();
        for k in 0..rank {
            let essential = self
                .matrix
                .view((k + 1, k), (self.matrix.nrows() - k - 1, 1))
                .column(0)
                .into_owned();
            apply_householder_left(&mut transformed, k, 0, &essential, self.q_coefficients[k]);
        }
        for column in 0..rhs.ncols() {
            for row in (0..rank).rev() {
                let mut sum = S::zero();
                for j in row + 1..rank {
                    sum += self.matrix[(row, j)] * solution[(j, column)];
                }
                solution[(row, column)] =
                    (transformed[(row, column)] - sum) / self.matrix[(row, row)];
            }
        }
        if rank < cols {
            for k in 0..rank {
                if k != rank - 1 {
                    solution.swap_rows(k, rank - 1);
                }
                let essential = DVector::from_fn(cols - rank, |i, _| self.matrix[(k, rank + i)]);
                apply_householder_left(
                    &mut solution,
                    rank - 1,
                    0,
                    &essential,
                    self.z_coefficients[k],
                );
                if k != rank - 1 {
                    solution.swap_rows(k, rank - 1);
                }
            }
        }
        let mut result = DMatrix::zeros(cols, rhs.ncols());
        for i in 0..cols {
            result
                .row_mut(self.permutation[i])
                .copy_from(&solution.row(i));
        }
        result
    }
    #[must_use]
    pub fn pseudo_inverse(&self) -> DMatrix<S> {
        self.solve(&DMatrix::identity(self.matrix.nrows(), self.matrix.nrows()))
    }
}
