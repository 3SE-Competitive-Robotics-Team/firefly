// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/spline/rd_bezier.h` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
use super::spline_common::{self, Jacobian};
use crate::scalar::BasaltScalar;
use nalgebra::{SMatrix, SVector};

#[derive(Clone, Debug)]
pub struct IntegralJacobian<S: BasaltScalar, const DIM: usize, const N: usize> {
    pub d_int_d_knot: [SVector<S, DIM>; N],
}
impl<S: BasaltScalar, const DIM: usize, const N: usize> Default for IntegralJacobian<S, DIM, N> {
    fn default() -> Self {
        Self {
            d_int_d_knot: std::array::from_fn(|_| SVector::zeros()),
        }
    }
}
/// One Euclidean Bezier segment, timed in integer nanoseconds.
#[derive(Clone, Debug)]
pub struct RdBezier<S: BasaltScalar, const DIM: usize, const N: usize> {
    pub(super) knots: [SVector<S, DIM>; N],
    pub(super) dt_ns: i64,
    pub(super) start_t_ns: i64,
    pub(super) pow_inv_dt: [S; N],
    blending: SMatrix<S, N, N>,
}
impl<S: BasaltScalar, const DIM: usize, const N: usize> Default for RdBezier<S, DIM, N> {
    fn default() -> Self {
        Self {
            knots: std::array::from_fn(|_| SVector::zeros()),
            dt_ns: 0,
            start_t_ns: 0,
            pow_inv_dt: [S::zero(); N],
            blending: spline_common::compute_blending_matrix_bezier(),
        }
    }
}
impl<S: BasaltScalar, const DIM: usize, const N: usize> RdBezier<S, DIM, N> {
    pub const N: usize = N;
    pub const DIM: usize = DIM;
    pub const DEG: usize = N - 1;
    pub const NS_TO_S: f64 = 1e-9;
    pub const S_TO_NS: f64 = 1e9;
    /// # Panics
    /// Panics if the order is less than two.
    #[must_use]
    pub fn new(time_interval_ns: i64, start_time_ns: i64) -> Self {
        Self {
            dt_ns: time_interval_ns,
            start_t_ns: start_time_ns,
            pow_inv_dt: spline_common::powers(time_interval_ns),
            ..Self::default()
        }
    }
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> RdBezier<T, DIM, N> {
        RdBezier {
            knots: self.knots.map(|v| v.map(|s| T::from_f64_lossy(s.as_f64()))),
            dt_ns: self.dt_ns,
            start_t_ns: self.start_t_ns,
            pow_inv_dt: self.pow_inv_dt.map(|s| T::from_f64_lossy(s.as_f64())),
            ..RdBezier::default()
        }
    }
    pub fn set_start_time_ns(&mut self, time: i64) {
        self.start_t_ns = time;
    }
    #[must_use]
    pub const fn min_time_ns(&self) -> i64 {
        self.start_t_ns
    }
    #[must_use]
    pub const fn max_time_ns(&self) -> i64 {
        self.start_t_ns.wrapping_add(self.dt_ns).wrapping_sub(1)
    }
    #[must_use]
    pub const fn time_interval_ns(&self) -> i64 {
        self.dt_ns
    }
    /// Generates knots from successive Eigen-style uniform samples in [-1,1].
    /// The caller owns the RNG; the static initialization branch scales by five.
    pub fn gen_random_trajectory(&mut self, static_init: bool, mut sample: impl FnMut() -> S) {
        if static_init {
            let knot = SVector::from_fn(|_, _| sample()) * S::from_f64_lossy(5.);
            self.knots.fill(knot);
        } else {
            for knot in &mut self.knots {
                *knot = SVector::from_fn(|_, _| sample());
            }
        }
    }
    /// # Panics
    /// Panics for a zero-order curve.
    #[must_use]
    pub fn knots_front(&self) -> &SVector<S, DIM> {
        &self.knots[0]
    }
    /// # Panics
    /// Panics for an invalid knot index.
    #[must_use]
    pub fn knot(&self, index: usize) -> &SVector<S, DIM> {
        &self.knots[index]
    }
    /// # Panics
    /// Panics for an invalid knot index.
    pub fn knot_mut(&mut self, index: usize) -> &mut SVector<S, DIM> {
        &mut self.knots[index]
    }
    /// Evaluates a derivative and the corresponding sparse knot Jacobian.
    /// # Panics
    /// Panics for a nonpositive interval, out-of-domain time, or derivative >= order.
    #[must_use]
    pub fn evaluate<const D: usize>(
        &self,
        time_ns: i64,
        jacobian: Option<&mut Jacobian<S, N>>,
    ) -> SVector<S, DIM> {
        assert!(self.dt_ns > 0 && D < N);
        let elapsed = time_ns - self.start_t_ns;
        assert!(elapsed >= 0);
        let segment = elapsed / self.dt_ns;
        assert_eq!(segment, 0);
        let time = S::from_f64_lossy((elapsed % self.dt_ns) as f64 / self.dt_ns as f64);
        let coeff =
            (self.blending * self.pow_inv_dt[D]) * spline_common::polynomial::<S, N, D>(time);
        let mut result = SVector::zeros();
        for i in 0..N {
            result += self.knots[i] * coeff[i];
        }
        if let Some(j) = jacobian {
            j.start_idx = 0;
            j.d_val_d_knot = std::array::from_fn(|i| coeff[i]);
        }
        result
    }
    /// # Panics
    /// Has the same time-domain requirements as `evaluate`.
    #[must_use]
    pub fn velocity(&self, time: i64, j: Option<&mut Jacobian<S, N>>) -> SVector<S, DIM> {
        self.evaluate::<1>(time, j)
    }
    /// # Panics
    /// Has the same time-domain requirements as `evaluate`.
    #[must_use]
    pub fn acceleration(&self, time: i64, j: Option<&mut Jacobian<S, N>>) -> SVector<S, DIM> {
        self.evaluate::<2>(time, j)
    }
    /// Integral of the squared value/derivative over the entire segment.
    /// Derivatives at or above the order return zero without touching the Jacobian.
    /// # Panics
    /// Panics for order smaller than two.
    #[must_use]
    pub fn evaluate_integral_squared<const D: usize>(
        &self,
        jacobian: Option<&mut IntegralJacobian<S, DIM, N>>,
    ) -> S {
        if D >= N {
            return S::zero();
        }
        let knots = SMatrix::<S, DIM, N>::from_fn(|r, c| self.knots[c][r]);
        let q = spline_common::compute_quadratic_coefficients::<S, N>();
        let tmp = knots * self.blending * q[D] * self.blending.transpose();
        let scaling = if D == 0 {
            S::one() / self.pow_inv_dt[1]
        } else {
            self.pow_inv_dt[D] * self.pow_inv_dt[D - 1]
        };
        if let Some(j) = jacobian {
            for (i, slot) in j.d_int_d_knot.iter_mut().enumerate() {
                *slot = tmp.column(i).into_owned() * scaling;
            }
        }
        S::from_f64_lossy(0.5) * scaling * tmp.component_mul(&knots).sum()
    }
}
