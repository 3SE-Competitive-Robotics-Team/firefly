// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
// SPDX-License-Identifier: MPL-2.0
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. https://mozilla.org/MPL/2.0/
//! Eigen 5.0.1 `FullPivHouseholderQR`: real decomposition, rank and solve used by NFR.
use super::complete_orthogonal_decomposition::{apply_householder_left, make_householder};
use nalgebra::{DMatrix, DVector};

pub struct FullPivHouseholderQr {
    qr: DMatrix<f64>,
    coefficients: Vec<f64>,
    row_swaps: Vec<usize>,
    permutation: Vec<usize>,
    rank: usize,
}
impl FullPivHouseholderQr {
    #[must_use]
    pub fn new(mut matrix: DMatrix<f64>) -> Self {
        let (rows, columns) = matrix.shape();
        let size = rows.min(columns);
        let precision = f64::EPSILON * size as f64;
        let mut coefficients = vec![0.; size];
        let mut row_swaps: Vec<_> = (0..size).collect();
        let mut permutation: Vec<_> = (0..columns).collect();
        let mut first_pivot = 0.;
        let mut max_pivot = 0.0_f64;
        let mut nonzero = size;
        for k in 0..size {
            let (mut pivot_row, mut pivot_column, mut biggest) = (k, k, matrix[(k, k)].abs());
            for c in k..columns {
                for r in k..rows {
                    if matrix[(r, c)].abs() > biggest {
                        (pivot_row, pivot_column, biggest) = (r, c, matrix[(r, c)].abs());
                    }
                }
            }
            if k == 0 {
                first_pivot = biggest;
            }
            if biggest <= first_pivot * precision {
                nonzero = k;
                break;
            }
            row_swaps[k] = pivot_row;
            for c in k..columns {
                matrix.swap((k, c), (pivot_row, c));
            }
            matrix.swap_columns(k, pivot_column);
            permutation.swap(k, pivot_column);
            let mut reflector = matrix.view((k, k), (rows - k, 1)).column(0).into_owned();
            let (tau, beta) = make_householder(&mut reflector);
            coefficients[k] = tau;
            for r in k + 1..rows {
                matrix[(r, k)] = reflector[r - k];
            }
            matrix[(k, k)] = beta;
            max_pivot = max_pivot.max(beta.abs());
            let essential = reflector.rows(1, reflector.len() - 1).into_owned();
            apply_householder_left(&mut matrix, k, k + 1, &essential, tau);
        }
        let rank = (0..nonzero)
            .filter(|&k| matrix[(k, k)].abs() > max_pivot * precision)
            .count();
        Self {
            qr: matrix,
            coefficients,
            row_swaps,
            permutation,
            rank,
        }
    }
    #[must_use]
    pub fn rank(&self) -> usize {
        self.rank
    }
    /// # Panics
    /// Panics if the right hand side row count differs from the matrix.
    #[must_use]
    pub fn solve(&self, rhs: &DMatrix<f64>) -> DMatrix<f64> {
        assert_eq!(rhs.nrows(), self.qr.nrows());
        let mut output = DMatrix::zeros(self.qr.ncols(), rhs.ncols());
        if self.rank == 0 {
            return output;
        }
        let mut transformed = rhs.clone();
        for k in 0..self.rank {
            transformed.swap_rows(k, self.row_swaps[k]);
            let essential = DVector::from_iterator(
                self.qr.nrows() - k - 1,
                (k + 1..self.qr.nrows()).map(|r| self.qr[(r, k)]),
            );
            apply_householder_left(&mut transformed, k, 0, &essential, self.coefficients[k]);
        }
        for c in 0..rhs.ncols() {
            for r in (0..self.rank).rev() {
                let mut value = transformed[(r, c)];
                for j in r + 1..self.rank {
                    value -= self.qr[(r, j)] * transformed[(j, c)];
                }
                transformed[(r, c)] = value / self.qr[(r, r)];
            }
        }
        for r in 0..self.rank {
            output
                .row_mut(self.permutation[r])
                .copy_from(&transformed.row(r));
        }
        output
    }
}
