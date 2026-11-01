// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original `utils/test_utils.h`: central-difference Jacobian diagnostics.
//! Diagnostics are returned to the caller instead of writing to stderr.
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector};

#[derive(Clone, Copy, Debug)]
pub struct JacobianOptions {
    pub eps: f64,
    pub max_norm: f64,
}
impl Default for JacobianOptions {
    fn default() -> Self {
        Self {
            eps: 1e-8,
            max_norm: 1e-4,
        }
    }
}
#[derive(Clone, Debug)]
pub struct JacobianDiagnostic<S: BasaltScalar> {
    pub name: String,
    pub numerical: DMatrix<S>,
    pub difference_norm: S,
    pub reported: bool,
}

/// Preserve the source's reporting predicate, including its NaN behavior.
/// # Panics
/// The input length and function output length must match the Jacobian dimensions.
pub fn test_jacobian_code<S: BasaltScalar>(
    name: &str,
    analytic: &DMatrix<S>,
    func: impl Fn(&DVector<S>) -> DVector<S>,
    initial: &DVector<S>,
    options: JacobianOptions,
) -> JacobianDiagnostic<S> {
    assert_eq!(initial.len(), analytic.ncols());
    let mut numerical = DMatrix::zeros(analytic.nrows(), analytic.ncols());
    for column in 0..analytic.ncols() {
        let mut increment = DVector::zeros(initial.len());
        increment[column] = S::from_f64_lossy(options.eps);
        let positive = func(&(initial + &increment));
        let negative = func(&(initial - &increment));
        numerical.set_column(
            column,
            &((positive - negative) / S::from_f64_lossy(2. * options.eps)),
        );
    }
    let difference_norm = (analytic - &numerical).norm();
    let reported = difference_norm.as_f64() > options.max_norm
        || !analytic.iter().all(nalgebra::ComplexField::is_finite);
    JacobianDiagnostic {
        name: name.to_owned(),
        numerical,
        difference_norm,
        reported,
    }
}

/// # Panics
/// The input and function output dimensions must match `analytic`.
pub fn test_jacobian<S: BasaltScalar>(
    name: &str,
    analytic: &DMatrix<S>,
    func: impl Fn(&DVector<S>) -> DVector<S>,
    initial: &DVector<S>,
) -> JacobianDiagnostic<S> {
    test_jacobian_code(name, analytic, func, initial, JacobianOptions::default())
}
