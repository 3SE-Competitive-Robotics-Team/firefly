// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `vi_estimator/marg_helper.h` and its `.cpp` implementation,
//! original Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
use crate::utils::{
    complete_orthogonal_decomposition::{
        CompleteOrthogonalDecomposition, apply_householder_left, machine_epsilon, make_householder,
    },
    ldlt::Ldlt,
};
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector};
use std::{collections::BTreeSet, marker::PhantomData};

/// Marginalization in normal-equation and square-root representations.
pub struct MargHelper<S: BasaltScalar>(PhantomData<S>);
impl<S: BasaltScalar> MargHelper<S> {
    fn validate(columns: usize, keep: &BTreeSet<usize>, marg: &BTreeSet<usize>) {
        assert_eq!(keep.len() + marg.len(), columns);
        assert!(keep.is_disjoint(marg));
        assert!(keep.iter().chain(marg).all(|&i| i < columns));
    }
    /// Schur elimination with the original complete-orthogonal pseudoinverse.
    /// Consumes the numeric contents of `h` and `b`, leaving empty inputs.
    /// # Panics
    /// Requires a square system and disjoint index sets partitioning all columns.
    pub fn marginalize_sq_to_sq(
        h: &mut DMatrix<S>,
        b: &mut DVector<S>,
        keep: &BTreeSet<usize>,
        marg: &BTreeSet<usize>,
    ) -> (DMatrix<S>, DVector<S>) {
        Self::validate(h.ncols(), keep, marg);
        assert!(h.is_square());
        assert_eq!(h.nrows(), b.len());
        let indices: Vec<_> = keep.iter().chain(marg).copied().collect();
        let k = keep.len();
        let m = marg.len();
        let permuted = DMatrix::from_fn(h.nrows(), h.ncols(), |r, c| h[(indices[r], indices[c])]);
        let rhs = DVector::from_fn(b.len(), |r, _| b[indices[r]]);
        let eliminated = permuted.view((k, k), (m, m)).into_owned();
        let inverse = CompleteOrthogonalDecomposition::new(eliminated).pseudo_inverse();
        let cross = permuted.view((0, k), (k, m)) * inverse;
        let result_h = permuted.view((0, 0), (k, k)) - &cross * permuted.view((k, 0), (m, k));
        let result_b = rhs.rows(0, k) - cross * rhs.rows(k, m);
        *h = DMatrix::zeros(0, 0);
        *b = DVector::zeros(0);
        (result_h, result_b)
    }
    /// Schur elimination followed by the original pivoted-LDLT square root.
    /// Negative LDLT pivots are clamped to zero before taking their square root.
    /// # Panics
    /// Requires the same partition and dimensions as [`Self::marginalize_sq_to_sq`].
    pub fn marginalize_sq_to_sqrt(
        h: &mut DMatrix<S>,
        b: &mut DVector<S>,
        keep: &BTreeSet<usize>,
        marg: &BTreeSet<usize>,
    ) -> (DMatrix<S>, DVector<S>) {
        let (reduced_h, reduced_b) = Self::marginalize_sq_to_sq(h, b, keep, marg);
        let size = keep.len();
        let ldlt = Ldlt::new(reduced_h);
        let diagonal = ldlt
            .diagonal()
            .map(|v| if v < S::zero() { S::zero() } else { v }.sqrt());
        let mut root = DMatrix::identity(size, size);
        ldlt.permute_rows(&mut root);
        root = ldlt.lower().transpose() * root;
        for row in 0..size {
            root.row_mut(row).scale_mut(diagonal[row]);
        }
        let mut rhs = DMatrix::from_column_slice(size, 1, reduced_b.as_slice());
        ldlt.permute_rows(&mut rhs);
        ldlt.solve_lower(&mut rhs);
        let minimum = S::min_positive_value().sqrt();
        for row in 0..size {
            rhs[(row, 0)] = if diagonal[row] > minimum {
                rhs[(row, 0)] / diagonal[row]
            } else {
                S::zero()
            };
        }
        (root, rhs.column(0).into_owned())
    }
    /// Ordered Householder elimination with the upstream absolute rank threshold.
    /// The residual retains at least one row, even for a rank-zero result.
    /// # Panics
    /// Requires matching nonempty row dimensions and a complete column partition.
    pub fn marginalize_sqrt_to_sqrt(
        jacobian: &mut DMatrix<S>,
        residual: &mut DVector<S>,
        keep: &BTreeSet<usize>,
        marg: &BTreeSet<usize>,
    ) -> (DMatrix<S>, DVector<S>) {
        Self::validate(jacobian.ncols(), keep, marg);
        assert_eq!(jacobian.nrows(), residual.len());
        assert!(jacobian.nrows() > 0);
        let indices: Vec<_> = marg.iter().chain(keep).copied().collect();
        let rows = jacobian.nrows();
        let cols = jacobian.ncols();
        let mut matrix = DMatrix::from_fn(rows, cols, |r, c| jacobian[(r, indices[c])]);
        let mut rhs = DMatrix::from_column_slice(rows, 1, residual.as_slice());
        let mut marg_rank = 0;
        let mut total_rank = 0;
        let threshold = machine_epsilon::<S>().sqrt();
        for col in 0..cols {
            if total_rank >= rows {
                break;
            }
            let remaining = rows - total_rank;
            let mut reflector = matrix
                .view((total_rank, col), (remaining, 1))
                .column(0)
                .into_owned();
            let (tau, beta) = make_householder(&mut reflector);
            if beta.abs() > threshold {
                matrix[(total_rank, col)] = beta;
                let essential = reflector.rows(1, remaining - 1).into_owned();
                apply_householder_left(&mut matrix, total_rank, col + 1, &essential, tau);
                apply_householder_left(&mut rhs, total_rank, 0, &essential, tau);
                total_rank += 1;
            } else {
                matrix[(total_rank, col)] = S::zero();
            }
            // Reflector vectors are overwritten starting after the original pivot row.
            let pivot_row = rows - remaining;
            for row in pivot_row + 1..rows {
                matrix[(row, col)] = S::zero();
            }
            if col + 1 == marg.len() {
                marg_rank = total_rank;
            }
        }
        let valid_rows = (total_rank - marg_rank).max(1);
        assert!(marg_rank + valid_rows <= rows);
        let root = matrix
            .view((marg_rank, marg.len()), (valid_rows, keep.len()))
            .into_owned();
        let rhs = rhs
            .view((marg_rank, 0), (valid_rows, 1))
            .column(0)
            .into_owned();
        *jacobian = DMatrix::zeros(0, 0);
        *residual = DVector::zeros(0);
        (root, rhs)
    }
}
