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
        if result.shape() != a.shape() {
            result.resize_mut(a.nrows(), a.ncols(), 0.0);
        }
        result.fill(0.0);
        for (column, weight) in vectors.column_iter().zip(inverse_values.iter()) {
            result.ger(*weight, &column, &column, 1.0);
        }
        true
    }

    /// Returns `U * diag(sqrt(inverse_values))`.
    pub fn symm_sqrt(
        a: &DMatrix<f64>,
        result: &mut DMatrix<f64>,
        epsilon: f64,
        rank: Option<&mut usize>,
    ) -> bool {
        let (mut vectors, inverse_values) = spectral_inverse(a, epsilon, rank);
        for (mut column, weight) in vectors.column_iter_mut().zip(inverse_values.iter()) {
            column.scale_mut(weight.sqrt());
        }
        *result = vectors;
        true
    }

    /// Returns `diag(sqrt(inverse_values)) * U.transpose()`.
    pub fn symm_sqrt_u(
        a: &DMatrix<f64>,
        result: &mut DMatrix<f64>,
        epsilon: f64,
        rank: Option<&mut usize>,
    ) -> bool {
        let (mut vectors, inverse_values) = spectral_inverse(a, epsilon, rank);
        for (mut column, weight) in vectors.column_iter_mut().zip(inverse_values.iter()) {
            column.scale_mut(weight.sqrt());
        }
        vectors.transpose_mut();
        *result = vectors;
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spectral_factors_match_dense_reference_through_imu_dimension() {
        for size in [1, 3, 6, 15] {
            let basis = DMatrix::from_fn(size, size, |row, col| {
                ((row + 1) as f64 * (col + 3) as f64).sin() + if row == col { 2. } else { 0. }
            });
            for scale in [1e-12, 1., 1e12] {
                let input = basis.tr_mul(&basis) * scale;
                for epsilon in [f64::EPSILON, 1e-6] {
                    let (vectors, weights) = spectral_inverse(&input, epsilon, None);
                    let expected_inverse =
                        &vectors * DMatrix::from_diagonal(&weights) * vectors.transpose();
                    let expected_root = &vectors * DMatrix::from_diagonal(&weights.map(f64::sqrt));
                    let mut inverse = DMatrix::zeros(0, 0);
                    let mut root = DMatrix::zeros(0, 0);
                    let mut upper = DMatrix::zeros(0, 0);
                    PseudoInverse::symm(&input, &mut inverse, epsilon, None);
                    PseudoInverse::symm_sqrt(&input, &mut root, epsilon, None);
                    PseudoInverse::symm_sqrt_u(&input, &mut upper, epsilon, None);
                    let bound = 2e-14 * size as f64;
                    assert!(
                        (&inverse - &expected_inverse).amax() <= bound * expected_inverse.amax()
                    );
                    assert!((&root - &expected_root).amax() <= bound * expected_root.amax());
                    assert!(
                        (upper - expected_root.transpose()).amax() <= bound * expected_root.amax()
                    );
                }
            }
        }
    }

    #[test]
    fn spectral_factors_match_rotated_analytic_inverse_with_eigenvalue_flooring() {
        let epsilon = 1e-3_f64;
        let tolerance = epsilon * 3. * 8.;
        for eigenvalue in [1., 0., -1., 0.5 * tolerance, 2. * tolerance] {
            let input = DMatrix::from_row_slice(
                3,
                3,
                &[
                    eigenvalue.midpoint(2.),
                    (eigenvalue - 2.) / 2.,
                    0.,
                    (eigenvalue - 2.) / 2.,
                    eigenvalue.midpoint(2.),
                    0.,
                    0.,
                    0.,
                    8.,
                ],
            );
            let weight = if eigenvalue > tolerance {
                1. / eigenvalue
            } else {
                1. / tolerance
            };
            let expected = DMatrix::from_row_slice(
                3,
                3,
                &[
                    weight.midpoint(0.5),
                    (weight - 0.5) / 2.,
                    0.,
                    (weight - 0.5) / 2.,
                    weight.midpoint(0.5),
                    0.,
                    0.,
                    0.,
                    0.125,
                ],
            );
            let expected_rank = if eigenvalue > tolerance { 3 } else { 2 };
            let mut rank = 0;
            let mut inverse = DMatrix::repeat(3, 3, 99.);
            let pointer = inverse.as_ptr();
            assert!(PseudoInverse::symm(
                &input,
                &mut inverse,
                epsilon,
                Some(&mut rank)
            ));
            assert_eq!(rank, expected_rank);
            assert_eq!(inverse.as_ptr(), pointer);
            assert!((&inverse - &expected).amax() < 1e-11);
            assert!(PseudoInverse::symm(&input, &mut inverse, epsilon, None));
            assert!((&inverse - &expected).amax() < 1e-11);
            let mut root = DMatrix::zeros(1, 1);
            assert!(PseudoInverse::symm_sqrt(
                &input,
                &mut root,
                epsilon,
                Some(&mut rank)
            ));
            assert_eq!(rank, expected_rank);
            assert!((&root * root.transpose() - &expected).amax() < 1e-11);
            let mut upper = DMatrix::zeros(1, 1);
            assert!(PseudoInverse::symm_sqrt_u(
                &input,
                &mut upper,
                epsilon,
                Some(&mut rank)
            ));
            assert_eq!(rank, expected_rank);
            assert!((upper.tr_mul(&upper) - &expected).amax() < 1e-11);
            assert!((upper - root.transpose()).amax() < 1e-12);
        }
    }

    #[test]
    fn spectral_inverse_preserves_lower_triangle_and_strict_threshold_semantics() {
        let mut input = DMatrix::from_diagonal(&DVector::from_row_slice(&[0.024, 2., 8.]));
        input[(0, 1)] = 100.;
        let mut output = DMatrix::zeros(1, 1);
        let mut rank = 0;
        PseudoInverse::symm(&input, &mut output, 1e-3, Some(&mut rank));
        assert_eq!(rank, 2);
        let expected = DMatrix::from_diagonal(&DVector::from_row_slice(&[1. / 0.024, 0.5, 0.125]));
        assert!((output - expected).amax() < 1e-12);
    }
}
