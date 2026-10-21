/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

use nalgebra::{DMatrix, DVector};
use num_complex::Complex64;
use std::marker::PhantomData;

/// Dimensions of a residual functor; -1 denotes a dynamic compile-time size.
#[derive(Clone, Copy, Debug)]
pub struct OptimizationFunctor<Scalar, const NX: isize = -1, const NY: isize = -1> {
    inputs: isize,
    values: isize,
    scalar: PhantomData<Scalar>,
}
impl<Scalar, const NX: isize, const NY: isize> Default for OptimizationFunctor<Scalar, NX, NY> {
    fn default() -> Self {
        Self::new(NX, NY)
    }
}
impl<Scalar, const NX: isize, const NY: isize> OptimizationFunctor<Scalar, NX, NY> {
    pub const INPUTS_AT_COMPILE_TIME: isize = NX;
    pub const VALUES_AT_COMPILE_TIME: isize = NY;
    #[must_use]
    pub fn new(inputs: isize, values: isize) -> Self {
        Self {
            inputs,
            values,
            scalar: PhantomData,
        }
    }
    #[must_use]
    pub fn inputs(&self) -> isize {
        self.inputs
    }
    #[must_use]
    pub fn values(&self) -> isize {
        self.values
    }
}

/// Parameters of Eigen's forward-difference Levenberg–Marquardt solver.
#[derive(Clone, Copy, Debug)]
pub struct LmParameters {
    pub factor: f64,
    pub maxfev: usize,
    pub ftol: f64,
    pub xtol: f64,
    pub gtol: f64,
    pub epsfcn: f64,
}
impl Default for LmParameters {
    fn default() -> Self {
        Self {
            factor: 100.0,
            maxfev: 400,
            ftol: f64::EPSILON.sqrt(),
            xtol: f64::EPSILON.sqrt(),
            gtol: 0.0,
            epsfcn: 0.0,
        }
    }
}
/// Fills residuals for the current estimate, returning `None` if evaluation fails.
pub type ResidualFunction<'a> = dyn FnMut(&DVector<f64>, &mut DVector<f64>) -> Option<()> + 'a;

/// Numerical decompositions and minimization supplied by a numerical library.
/// Eigenvectors are columns and correspond in order to the returned eigenvalues.
/// The minimizer uses forward numerical differences with the stated parameters.
pub trait NumericalBackend {
    /// Full orthogonal U and V, including the nullspace columns for rectangular input.
    fn full_svd(&self, matrix: &DMatrix<f64>)
    -> Option<(DMatrix<f64>, DVector<f64>, DMatrix<f64>)>;
    fn eigenpairs(&self, matrix: &DMatrix<f64>)
    -> Option<(DVector<Complex64>, DMatrix<Complex64>)>;
    fn levenberg_marquardt(
        &self,
        x: &mut DVector<f64>,
        values: usize,
        residual: &mut ResidualFunction<'_>,
        parameters: LmParameters,
    ) -> Option<()>;
}

/// Pure Rust dense decompositions and MINPACK-compatible forward-difference LM.
#[derive(Clone, Copy, Debug, Default)]
pub struct RustNumericalBackend;

impl NumericalBackend for RustNumericalBackend {
    fn full_svd(
        &self,
        matrix: &DMatrix<f64>,
    ) -> Option<(DMatrix<f64>, DVector<f64>, DMatrix<f64>)> {
        if matrix.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let (m, n) = matrix.shape();
        if m == 0 || n == 0 {
            return Some((
                DMatrix::identity(m, m),
                DVector::zeros(m.min(n)),
                DMatrix::identity(n, n),
            ));
        }
        let input = faer::Mat::from_fn(m, n, |i, j| matrix[(i, j)]);
        let decomposition = input.svd().ok()?;
        Some((
            DMatrix::from_fn(m, m, |i, j| decomposition.U()[(i, j)]),
            DVector::from_fn(m.min(n), |i, _| decomposition.S()[i]),
            DMatrix::from_fn(n, n, |i, j| decomposition.V()[(i, j)]),
        ))
    }
    fn eigenpairs(
        &self,
        matrix: &DMatrix<f64>,
    ) -> Option<(DVector<Complex64>, DMatrix<Complex64>)> {
        if !matrix.is_square() || matrix.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let n = matrix.nrows();
        if n == 0 {
            return Some((DVector::zeros(0), DMatrix::zeros(0, 0)));
        }
        let input = faer::Mat::from_fn(n, n, |i, j| matrix[(i, j)]);
        let decomposition = input.eigen().ok()?;
        let mut vectors = DMatrix::from_fn(n, n, |i, j| decomposition.U()[(i, j)]);
        // Eigen's EigenSolver exposes unit-length complex eigenvectors.
        for j in 0..n {
            let norm = vectors.column(j).norm();
            if norm != 0.0 {
                vectors.column_mut(j).unscale_mut(norm);
            }
        }
        Some((DVector::from_fn(n, |i, _| decomposition.S()[i]), vectors))
    }
    fn levenberg_marquardt(
        &self,
        x: &mut DVector<f64>,
        values: usize,
        residual: &mut ResidualFunction<'_>,
        parameters: LmParameters,
    ) -> Option<()> {
        minpack_lm::minimize(x, values, residual, parameters)
    }
}
mod minpack_lm;
