// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/spline/rd_spline.h` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
use super::{
    rd_bezier::RdBezier,
    spline_common::{self, Jacobian},
};
use crate::scalar::BasaltScalar;
use nalgebra::{DMatrix, SMatrix, SVector};
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub struct IntegralJacobian<S: BasaltScalar, const DIM: usize> {
    pub d_int_d_knot: VecDeque<SVector<S, DIM>>,
}
impl<S: BasaltScalar, const DIM: usize> Default for IntegralJacobian<S, DIM> {
    fn default() -> Self {
        Self {
            d_int_d_knot: VecDeque::new(),
        }
    }
}
/// Uniform Euclidean B-spline with local support on N consecutive knots.
#[derive(Clone, Debug)]
pub struct RdSpline<S: BasaltScalar, const DIM: usize, const N: usize> {
    knots: VecDeque<SVector<S, DIM>>,
    dt_ns: i64,
    start_t_ns: i64,
    pow_inv_dt: [S; N],
    blending: SMatrix<S, N, N>,
}
impl<S: BasaltScalar, const DIM: usize, const N: usize> Default for RdSpline<S, DIM, N> {
    fn default() -> Self {
        Self {
            knots: VecDeque::new(),
            dt_ns: 0,
            start_t_ns: 0,
            pow_inv_dt: [S::zero(); N],
            blending: spline_common::compute_blending_matrix::<S, N, false>(),
        }
    }
}
impl<S: BasaltScalar, const DIM: usize, const N: usize> RdSpline<S, DIM, N> {
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
    pub fn cast<T: BasaltScalar>(&self) -> RdSpline<T, DIM, N> {
        RdSpline {
            knots: self
                .knots
                .iter()
                .map(|v| v.map(|s| T::from_f64_lossy(s.as_f64())))
                .collect(),
            dt_ns: self.dt_ns,
            start_t_ns: self.start_t_ns,
            pow_inv_dt: self.pow_inv_dt.map(|s| T::from_f64_lossy(s.as_f64())),
            ..RdSpline::default()
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
    pub fn max_time_ns(&self) -> i64 {
        self.start_t_ns
            .wrapping_add(
                (self.knots.len().wrapping_sub(N).wrapping_add(1) as i64).wrapping_mul(self.dt_ns),
            )
            .wrapping_sub(1)
    }
    #[must_use]
    pub const fn time_interval_ns(&self) -> i64 {
        self.dt_ns
    }
    /// Appends random knots using caller-provided Eigen-style samples in [-1,1].
    /// Static initialization appends N identical knots even if count is smaller.
    pub fn gen_random_trajectory(
        &mut self,
        count: i32,
        static_init: bool,
        mut sample: impl FnMut() -> S,
    ) {
        let mut knot = || SVector::from_fn(|_, _| sample()) * S::from_f64_lossy(5.);
        if static_init {
            let first = knot();
            for _ in 0..N {
                self.knots.push_back(first);
            }
            for _ in 0..i64::from(count) - N as i64 {
                self.knots.push_back(knot());
            }
        } else {
            for _ in 0..count {
                self.knots.push_back(knot());
            }
        }
    }
    pub fn knots_push_back(&mut self, knot: SVector<S, DIM>) {
        self.knots.push_back(knot);
    }
    /// # Panics
    /// Panics if no knots exist.
    pub fn knots_pop_back(&mut self) {
        self.knots.pop_back().expect("empty spline");
    }
    /// # Panics
    /// Panics if no knots exist.
    #[must_use]
    pub fn knots_front(&self) -> &SVector<S, DIM> {
        self.knots.front().expect("empty spline")
    }
    /// Removes the first knot and advances the start time by one interval.
    /// # Panics
    /// Panics if no knots exist or the new start time overflows.
    pub fn knots_pop_front(&mut self) {
        self.start_t_ns += self.dt_ns;
        self.knots.pop_front().expect("empty spline");
    }
    pub fn resize(&mut self, size: usize) {
        self.knots.resize(size, SVector::zeros());
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
    #[must_use]
    pub fn num_knots(&self) -> usize {
        self.knots.len()
    }
    #[must_use]
    pub const fn knots(&self) -> &VecDeque<SVector<S, DIM>> {
        &self.knots
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
        let segment = (elapsed / self.dt_ns) as usize;
        assert!(segment + N <= self.knots.len());
        let time = S::from_f64_lossy((elapsed % self.dt_ns) as f64 / self.dt_ns as f64);
        let coeff =
            (self.blending * self.pow_inv_dt[D]) * spline_common::polynomial::<S, N, D>(time);
        let mut result = SVector::zeros();
        for i in 0..N {
            result += self.knots[segment + i] * coeff[i];
        }
        if let Some(j) = jacobian {
            j.start_idx = segment;
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
    /// Integrates the squared derivative over every complete segment.
    /// # Panics
    /// Panics if the order is less than two.
    #[must_use]
    pub fn evaluate_integral_squared<const D: usize>(
        &self,
        mut jacobian: Option<&mut IntegralJacobian<S, DIM>>,
    ) -> S {
        if let Some(j) = jacobian.as_deref_mut() {
            j.d_int_d_knot.resize(self.knots.len(), SVector::zeros());
            for knot in &mut j.d_int_d_knot {
                knot.fill(S::zero());
            }
        }
        if D >= N {
            return S::zero();
        }
        let q = spline_common::compute_quadratic_coefficients::<S, N>();
        let integral = self.blending * q[D] * self.blending.transpose();
        let scaling = if D == 0 {
            S::one() / self.pow_inv_dt[1]
        } else {
            self.pow_inv_dt[D] * self.pow_inv_dt[D - 1]
        };
        let mut result = S::zero();
        for start in 0..self.knots.len().checked_sub(N).map_or(0, |v| v + 1) {
            let knots = SMatrix::<S, DIM, N>::from_fn(|r, c| self.knots[start + c][r]);
            let tmp = knots * integral;
            if let Some(j) = jacobian.as_deref_mut() {
                for i in 0..N {
                    j.d_int_d_knot[start + i] += tmp.column(i) * scaling;
                }
            }
            result += S::from_f64_lossy(0.5) * scaling * tmp.component_mul(&knots).sum();
        }
        result
    }
    /// Converts one local segment to its equivalent Bezier control points.
    /// # Panics
    /// Panics for an invalid segment or a singular blending matrix.
    #[must_use]
    pub fn segment_bezier_curve(&self, start: usize) -> RdBezier<S, DIM, N> {
        assert!(start + N <= self.knots.len());
        let basis = spline_common::compute_blending_matrix_bezier::<S, N>();
        let inverse = DMatrix::from_column_slice(N, N, basis.as_slice())
            .try_inverse()
            .expect("singular Bezier blending matrix");
        let inverse = SMatrix::<S, N, N>::from_fn(|r, c| inverse[(r, c)]);
        let transform = self.blending * inverse;
        let knots = SMatrix::<S, DIM, N>::from_fn(|r, c| self.knots[start + c][r]) * transform;
        let mut result = RdBezier::new(self.dt_ns, self.start_t_ns + start as i64 * self.dt_ns);
        for i in 0..N {
            *result.knot_mut(i) = knots.column(i).into_owned();
        }
        result
    }
}
