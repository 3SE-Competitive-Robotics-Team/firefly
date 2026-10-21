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

//! Spectral inverse and square-root factors corresponding to `PseudoInverse.hpp`.

use nalgebra::{DMatrix, DVector, SymmetricEigen};

#[derive(Clone, Copy, Debug, Default)]
pub struct PseudoInverse;

impl PseudoInverse {
    pub const DEFAULT_EPSILON: f64 = f64::EPSILON;

    /// Symmetric PSD input; the lower triangle determines the eigendecomposition.
    /// Eigenvalues at or below tolerance receive weight `1 / tolerance`.
    pub fn symm(
        a: &DMatrix<f64>,
        result: &mut DMatrix<f64>,
        epsilon: f64,
        rank: Option<&mut usize>,
    ) -> bool {
        let (vectors, inverse_values) = spectral_inverse(a, epsilon, rank);
        *result = &vectors * DMatrix::from_diagonal(&inverse_values) * vectors.transpose();
        true
    }

    /// Returns `U * diag(sqrt(inverse_values))`.
    pub fn symm_sqrt(
        a: &DMatrix<f64>,
        result: &mut DMatrix<f64>,
        epsilon: f64,
        rank: Option<&mut usize>,
    ) -> bool {
        let (vectors, inverse_values) = spectral_inverse(a, epsilon, rank);
        *result = vectors * DMatrix::from_diagonal(&inverse_values.map(f64::sqrt));
        true
    }

    /// Returns `diag(sqrt(inverse_values)) * U.transpose()`.
    pub fn symm_sqrt_u(
        a: &DMatrix<f64>,
        result: &mut DMatrix<f64>,
        epsilon: f64,
        rank: Option<&mut usize>,
    ) -> bool {
        let (vectors, inverse_values) = spectral_inverse(a, epsilon, rank);
        *result = DMatrix::from_diagonal(&inverse_values.map(f64::sqrt)) * vectors.transpose();
        true
    }
}

fn spectral_inverse(
    a: &DMatrix<f64>,
    epsilon: f64,
    rank: Option<&mut usize>,
) -> (DMatrix<f64>, DVector<f64>) {
    debug_assert_eq!(a.nrows(), a.ncols(), "matrix supplied is not square");
    let decomposition = SymmetricEigen::new(a.clone());
    let mut order: Vec<_> = (0..a.ncols()).collect();
    order.sort_by(|&left, &right| {
        decomposition.eigenvalues[left].total_cmp(&decomposition.eigenvalues[right])
    });
    let values = DVector::from_iterator(
        a.ncols(),
        order.iter().map(|&i| decomposition.eigenvalues[i]),
    );
    let vectors = DMatrix::from_fn(a.nrows(), a.ncols(), |row, column| {
        decomposition.eigenvectors[(row, order[column])]
    });
    let max_value = values[values.len() - 1];
    let scaled_epsilon = epsilon * a.ncols() as f64 * max_value;
    let tolerance = if epsilon < scaled_epsilon {
        scaled_epsilon
    } else {
        epsilon
    };
    if let Some(rank) = rank {
        *rank = values.iter().filter(|&&value| value > tolerance).count();
    }
    let inverse_values = values.map(|value| {
        if value > tolerance {
            1.0 / value
        } else {
            1.0 / tolerance
        }
    });
    (vectors, inverse_values)
}
