// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/spline/spline_common.h` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
use crate::scalar::BasaltScalar;
use nalgebra::{SMatrix, SVector};

#[must_use]
pub const fn binomial_coefficient(mut n: u64, k: u64) -> u64 {
    if k > n {
        return 0;
    }
    let mut result = 1_u64;
    let mut d = 1;
    while d <= k {
        result = result.wrapping_mul(n);
        n = n.wrapping_sub(1);
        result /= d;
        d += 1;
    }
    result
}
/// Uniform B-spline blending matrix, optionally with cumulative rows.
/// # Panics
/// Panics for zero order.
#[must_use]
pub fn compute_blending_matrix<S: BasaltScalar, const N: usize, const CUMULATIVE: bool>()
-> SMatrix<S, N, N> {
    assert!(N > 0);
    let mut m = SMatrix::<f64, N, N>::zeros();
    for i in 0..N {
        for j in 0..N {
            let mut sum = 0.;
            for s in j..N {
                sum += (-1.0_f64).powf((s - j) as f64)
                    * binomial_coefficient(N as u64, (s - j) as u64) as f64
                    * (N as f64 - s as f64 - 1.).powf(N as f64 - 1. - i as f64);
            }
            m[(j, i)] = binomial_coefficient((N - 1) as u64, (N - 1 - i) as u64) as f64 * sum;
        }
    }
    if CUMULATIVE {
        for i in 0..N {
            for j in i + 1..N {
                for c in 0..N {
                    m[(i, c)] += m[(j, c)];
                }
            }
        }
    }
    let mut factorial = 1_u64;
    for i in 2..N {
        factorial = factorial.wrapping_mul(i as u64);
    }
    (m / factorial as f64).map(S::from_f64_lossy)
}
/// Uniform Bezier blending matrix.
/// # Panics
/// Panics for zero order.
#[must_use]
pub fn compute_blending_matrix_bezier<S: BasaltScalar, const N: usize>() -> SMatrix<S, N, N> {
    assert!(N > 0);
    SMatrix::from_fn(|j, i| {
        if i >= j {
            S::from_f64_lossy(
                (-1.0_f64).powf((i - j) as f64)
                    * binomial_coefficient((N - 1) as u64, j as u64) as f64
                    * binomial_coefficient((N - 1 - j) as u64, (i - j) as u64) as f64,
            )
        } else {
            S::zero()
        }
    })
}
/// Derivative coefficients of monomials in increasing power order.
/// # Panics
/// Panics for zero order.
#[must_use]
pub fn compute_base_coefficients<S: BasaltScalar, const N: usize>() -> SMatrix<S, N, N> {
    assert!(N > 0);
    let mut base = SMatrix::<f64, N, N>::zeros();
    base.row_mut(0).fill(1.);
    for derivative in 1..N {
        for i in derivative - 1..N {
            base[(derivative, i)] = (i + 1 - derivative) as f64 * base[(derivative - 1, i)];
        }
    }
    base.map(S::from_f64_lossy)
}
/// Quadratic forms integrating twice the squared monomial derivative.
/// # Panics
/// Panics for zero order.
#[must_use]
pub fn compute_quadratic_coefficients<S: BasaltScalar, const N: usize>() -> [SMatrix<S, N, N>; N] {
    let base = compute_base_coefficients::<S, N>();
    std::array::from_fn(|derivative| {
        let mut matrix = SMatrix::zeros();
        for col in 0..N - derivative {
            for row in 0..N - derivative {
                let exponent = S::from_f64_lossy(((N - 1 - derivative) * 2 + 1 - row - col) as f64);
                matrix[(N - 1 - row, N - 1 - col)] = base[(derivative, N - 1 - row)]
                    * base[(derivative, N - 1 - col)]
                    * S::from_f64_lossy(2.)
                    / exponent;
            }
        }
        matrix
    })
}
/// Sparse knot Jacobian; each scalar multiplies the identity in value space.
#[derive(Clone, Debug)]
pub struct Jacobian<S: BasaltScalar, const N: usize> {
    pub start_idx: usize,
    pub d_val_d_knot: [S; N],
}
impl<S: BasaltScalar, const N: usize> Default for Jacobian<S, N> {
    fn default() -> Self {
        Self {
            start_idx: 0,
            d_val_d_knot: [S::zero(); N],
        }
    }
}
pub(super) fn powers<S: BasaltScalar, const N: usize>(dt: i64) -> [S; N] {
    assert!(N >= 2);
    let mut powers = [S::zero(); N];
    powers[0] = S::one();
    powers[1] = S::from_f64_lossy(1e9) / S::from_f64_lossy(dt as f64);
    for i in 2..N {
        powers[i] = powers[i - 1] * powers[1];
    }
    powers
}
pub fn polynomial<S: BasaltScalar, const N: usize, const D: usize>(time: S) -> SVector<S, N> {
    let base = compute_base_coefficients::<S, N>();
    let mut result = SVector::zeros();
    if D < N {
        result[D] = base[(D, D)];
        let mut power = time;
        for j in D + 1..N {
            result[j] = base[(D, j)] * power;
            power *= time;
        }
    }
    result
}
