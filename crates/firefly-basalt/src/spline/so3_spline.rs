// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `spline/so3_spline.h` at headers revision
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
use crate::utils::sophus_utils::{
    hat, left_jacobian_inv_so3, left_jacobian_so3, right_jacobian_inv_so3, right_jacobian_so3,
    so3_cast, so3_exp, so3_inverse, so3_log, so3_matrix, so3_product,
};
use firefly_calibration::scalar::BasaltScalar;
use firefly_calibration::spline::spline_common::{compute_blending_matrix, polynomial};
use nalgebra::{Matrix3, SVector, UnitQuaternion, Vector3};
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub struct Jacobian<S: BasaltScalar, const N: usize> {
    pub start_idx: usize,
    pub d_val_d_knot: [Matrix3<S>; N],
}
impl<S: BasaltScalar, const N: usize> Default for Jacobian<S, N> {
    fn default() -> Self {
        Self {
            start_idx: 0,
            d_val_d_knot: std::array::from_fn(|_| Matrix3::zeros()),
        }
    }
}

/// Cumulative uniform SO(3) B-spline, with left knot perturbations.
#[derive(Clone, Debug)]
pub struct So3Spline<S: BasaltScalar, const N: usize> {
    knots: VecDeque<UnitQuaternion<S>>,
    dt_ns: i64,
    start_t_ns: i64,
    pow_inv_dt: [S; 4],
}
impl<S: BasaltScalar, const N: usize> So3Spline<S, N> {
    #[must_use]
    /// # Panics
    /// Requires order at least two and a positive knot interval.
    pub fn new(time_interval_ns: i64, start_time_ns: i64) -> Self {
        assert!(N >= 2 && time_interval_ns > 0);
        let inverse = S::from_f64_lossy(1e9) / S::from_i64_lossy(time_interval_ns);
        Self {
            knots: VecDeque::new(),
            dt_ns: time_interval_ns,
            start_t_ns: start_time_ns,
            pow_inv_dt: [
                S::one(),
                inverse,
                inverse * inverse,
                inverse * inverse * inverse,
            ],
        }
    }
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> So3Spline<T, N> {
        let mut result = So3Spline::new(self.dt_ns, self.start_t_ns);
        result.knots = self.knots.iter().map(so3_cast).collect();
        result
    }
    #[must_use]
    /// # Panics
    /// Requires at least N knots and representable nanosecond bounds.
    pub fn max_time_ns(&self) -> i64 {
        assert!(self.knots.len() >= N);
        self.start_t_ns + i64::try_from(self.knots.len() - N + 1).unwrap() * self.dt_ns - 1
    }
    #[must_use]
    pub const fn min_time_ns(&self) -> i64 {
        self.start_t_ns
    }
    #[must_use]
    pub const fn time_interval_ns(&self) -> i64 {
        self.dt_ns
    }
    pub fn set_start_time_ns(&mut self, time: i64) {
        self.start_t_ns = time;
    }
    pub fn knots_push_back(&mut self, knot: UnitQuaternion<S>) {
        self.knots.push_back(knot);
    }
    /// # Panics
    /// Requires a nonempty knot sequence.
    pub fn knots_pop_back(&mut self) {
        self.knots.pop_back().expect("empty spline");
    }
    /// # Panics
    /// Requires a nonempty sequence and representable start time.
    pub fn knots_pop_front(&mut self) {
        self.knots.pop_front().expect("empty spline");
        self.start_t_ns += self.dt_ns;
    }
    #[must_use]
    /// # Panics
    /// Requires a nonempty knot sequence.
    pub fn knots_front(&self) -> &UnitQuaternion<S> {
        self.knots.front().expect("empty spline")
    }
    pub fn resize(&mut self, size: usize) {
        self.knots.resize(size, UnitQuaternion::identity());
    }
    #[must_use]
    pub fn knot(&self, index: usize) -> &UnitQuaternion<S> {
        &self.knots[index]
    }
    pub fn knot_mut(&mut self, index: usize) -> &mut UnitQuaternion<S> {
        &mut self.knots[index]
    }
    #[must_use]
    pub fn knots(&self) -> &VecDeque<UnitQuaternion<S>> {
        &self.knots
    }
    #[must_use]
    pub fn num_knots(&self) -> usize {
        self.knots.len()
    }
    /// `random` supplies the upstream uniform samples in [-1,1].
    /// # Panics
    /// Requires a positive spline order.
    pub fn gen_random_trajectory(
        &mut self,
        count: usize,
        static_init: bool,
        mut random: impl FnMut() -> Vector3<S>,
    ) {
        let pi = S::from_f64_lossy(std::f64::consts::PI);
        let initial = so3_exp(&(random() * pi));
        let repeated = if static_init { N } else { 1 };
        for _ in 0..repeated {
            self.knots.push_back(initial);
        }
        for _ in repeated..count {
            self.knots.push_back(so3_product(
                self.knots.back().unwrap(),
                &so3_exp(&(random() * (pi / S::from_f64_lossy(2.)))),
            ));
        }
    }
    fn segment(&self, time: i64) -> (usize, S) {
        let elapsed = time - self.start_t_ns;
        assert!(elapsed >= 0);
        let segment = usize::try_from(elapsed / self.dt_ns).unwrap();
        assert!(segment + N <= self.knots.len());
        (
            segment,
            S::from_f64_lossy((elapsed % self.dt_ns) as f64 / self.dt_ns as f64),
        )
    }
    fn coefficients<const D: usize>(&self, time: S) -> SVector<S, N> {
        (compute_blending_matrix::<S, N, true>() * self.pow_inv_dt[D]) * polynomial::<S, N, D>(time)
    }
    fn delta(&self, index: usize) -> Vector3<S> {
        so3_log(&so3_product(
            &so3_inverse(&self.knots[index]),
            &self.knots[index + 1],
        ))
    }
    #[must_use]
    /// # Panics
    /// Requires a supported timestamp and sufficient knots.
    pub fn evaluate(
        &self,
        time: i64,
        mut jacobian: Option<&mut Jacobian<S, N>>,
    ) -> UnitQuaternion<S> {
        let (start, u) = self.segment(time);
        let coefficients = self.coefficients::<0>(u);
        let mut result = self.knots[start];
        let mut helper = Matrix3::identity();
        if let Some(j) = jacobian.as_deref_mut() {
            j.start_idx = start;
        }
        for i in 0..N - 1 {
            let delta = self.delta(start + i);
            let scaled = delta * coefficients[i + 1];
            if let Some(j) = jacobian.as_deref_mut() {
                j.d_val_d_knot[i] = helper;
                helper = so3_matrix(&result)
                    * coefficients[i + 1]
                    * left_jacobian_so3(&scaled)
                    * left_jacobian_inv_so3(&delta)
                    * so3_matrix(&so3_inverse(&self.knots[start + i]));
                j.d_val_d_knot[i] -= helper;
            }
            result = so3_product(&result, &so3_exp(&scaled));
        }
        if let Some(j) = jacobian {
            j.d_val_d_knot[N - 1] = helper;
        }
        result
    }
    #[must_use]
    /// # Panics
    /// Requires a supported timestamp and sufficient knots.
    pub fn velocity_body(&self, time: i64, jacobian: Option<&mut Jacobian<S, N>>) -> Vector3<S> {
        let (start, u) = self.segment(time);
        let coefficients = self.coefficients::<0>(u);
        let derivative = self.coefficients::<1>(u);
        if jacobian.is_none() {
            let mut velocity = Vector3::zeros();
            for i in 0..N - 1 {
                let delta = self.delta(start + i);
                velocity = so3_exp(&(-delta * coefficients[i + 1])) * velocity;
                velocity += delta * derivative[i + 1];
            }
            return velocity;
        }
        let degree = N - 1;
        let mut delta = vec![Vector3::zeros(); degree];
        let mut rotations = vec![UnitQuaternion::identity(); degree];
        let mut accumulated = UnitQuaternion::identity();
        let mut temporary = vec![Matrix3::zeros(); degree];
        let mut inverse = vec![Matrix3::zeros(); degree];
        let mut right = vec![Matrix3::zeros(); degree];
        for i in (0..degree).rev() {
            delta[i] = self.delta(start + i);
            inverse[i] = right_jacobian_inv_so3(&delta[i])
                * so3_matrix(&so3_inverse(&self.knots[start + i + 1]));
            let scaled = delta[i] * coefficients[i + 1];
            right[i] = right_jacobian_so3(&(-scaled));
            temporary[i] = so3_matrix(&accumulated);
            rotations[i] = so3_exp(&(-scaled));
            accumulated = so3_product(&accumulated, &rotations[i]);
        }
        let mut derivatives = vec![Matrix3::zeros(); degree];
        derivatives[0] = (temporary[0] * derivative[1]) * inverse[0];
        let mut velocity = delta[0] * derivative[1];
        for i in 1..degree {
            derivatives[i] = (temporary[i - 1] * hat(&velocity) * right[i] * coefficients[i + 1]
                + temporary[i] * derivative[i + 1])
                * inverse[i];
            velocity = rotations[i] * velocity + delta[i] * derivative[i + 1];
        }
        let jacobian = jacobian.unwrap();
        jacobian.start_idx = start;
        jacobian.d_val_d_knot.fill(Matrix3::zeros());
        for (i, value) in derivatives.into_iter().enumerate() {
            jacobian.d_val_d_knot[i] -= value;
            jacobian.d_val_d_knot[i + 1] += value;
        }
        velocity
    }
    #[must_use]
    /// # Panics
    /// Requires a supported timestamp; acceleration Jacobians require order at least three.
    pub fn acceleration_body(
        &self,
        time: i64,
        acceleration_jacobian: Option<&mut Jacobian<S, N>>,
        velocity_output: Option<&mut Vector3<S>>,
        velocity_jacobian: Option<&mut Jacobian<S, N>>,
    ) -> Vector3<S> {
        let (start, u) = self.segment(time);
        let coefficients = self.coefficients::<0>(u);
        let first = self.coefficients::<1>(u);
        let second = self.coefficients::<2>(u);
        let degree = N - 1;
        let mut deltas = vec![Vector3::zeros(); degree];
        let mut rotations = vec![Matrix3::identity(); degree];
        let mut inverse = vec![Matrix3::zeros(); degree];
        let mut right = vec![Matrix3::zeros(); degree];
        let mut velocities = vec![Vector3::zeros(); degree];
        let mut accelerations = vec![Vector3::zeros(); degree];
        let mut velocity = Vector3::zeros();
        let mut acceleration = Vector3::zeros();
        for i in 0..degree {
            let delta = self.delta(start + i);
            deltas[i] = delta;
            let scaled = delta * coefficients[i + 1];
            let rotation = so3_exp(&(-scaled));
            rotations[i] = so3_matrix(&rotation);
            if acceleration_jacobian.is_some() {
                inverse[i] = right_jacobian_inv_so3(&delta)
                    * so3_matrix(&so3_inverse(&self.knots[start + i + 1]));
                right[i] = right_jacobian_so3(&(-scaled));
                velocity = rotations[i] * velocity;
                acceleration = rotations[i] * acceleration;
            } else {
                velocity = rotation * velocity;
                acceleration = rotation * acceleration;
            }
            let current = delta * first[i + 1];
            velocity += current;
            acceleration += delta * second[i + 1] + velocity.cross(&current);
            velocities[i] = velocity;
            accelerations[i] = acceleration;
        }
        if let Some(jacobian) = acceleration_jacobian {
            assert!(N >= 3);
            let last = degree - 1;
            let mut velocity_delta = vec![Matrix3::zeros(); degree];
            let mut acceleration_delta = vec![Matrix3::zeros(); degree];
            velocity_delta[last] =
                (rotations[last] * coefficients[degree]) * hat(&velocities[last - 1]) * right[last]
                    + Matrix3::identity() * first[degree];
            acceleration_delta[last] = (rotations[last] * coefficients[degree])
                * hat(&accelerations[last - 1])
                * right[last]
                + Matrix3::identity() * second[degree]
                + (hat(&velocities[last]) - hat(&deltas[last]) * velocity_delta[last])
                    * first[degree];
            let mut product = Matrix3::identity();
            let mut sum = Vector3::zeros();
            for i in (0..last).rev() {
                sum += (product * first[i + 2]) * deltas[i + 1];
                product *= rotations[i + 1];
                velocity_delta[i] = Matrix3::identity() * first[i + 1];
                if i >= 1 {
                    velocity_delta[i] +=
                        (rotations[i] * coefficients[i + 1]) * hat(&velocities[i - 1]) * right[i];
                }
                acceleration_delta[i] = Matrix3::identity() * second[i + 1]
                    + (hat(&velocities[i]) - hat(&deltas[i]) * velocity_delta[i]) * first[i + 1];
                if i >= 1 {
                    acceleration_delta[i] += (rotations[i] * coefficients[i + 1])
                        * hat(&accelerations[i - 1])
                        * right[i];
                }
                velocity_delta[i] = product * velocity_delta[i];
                acceleration_delta[i] =
                    product * acceleration_delta[i] - hat(&sum) * velocity_delta[i];
            }
            if let Some(j) = velocity_jacobian {
                j.start_idx = start;
                j.d_val_d_knot.fill(Matrix3::zeros());
                for i in 0..degree {
                    let value = velocity_delta[i] * inverse[i];
                    j.d_val_d_knot[i] -= value;
                    j.d_val_d_knot[i + 1] += value;
                }
            }
            jacobian.start_idx = start;
            jacobian.d_val_d_knot.fill(Matrix3::zeros());
            for i in 0..degree {
                let value = acceleration_delta[i] * inverse[i];
                jacobian.d_val_d_knot[i] -= value;
                jacobian.d_val_d_knot[i + 1] += value;
            }
        }
        if let Some(output) = velocity_output {
            *output = velocity;
        }
        acceleration
    }
    #[must_use]
    /// # Panics
    /// Requires a supported timestamp and sufficient knots.
    pub fn jerk_body(
        &self,
        time: i64,
        velocity_output: Option<&mut Vector3<S>>,
        acceleration_output: Option<&mut Vector3<S>>,
    ) -> Vector3<S> {
        let (start, u) = self.segment(time);
        let coefficients = self.coefficients::<0>(u);
        let first = self.coefficients::<1>(u);
        let second = self.coefficients::<2>(u);
        let third = self.coefficients::<3>(u);
        let mut velocity = Vector3::zeros();
        let mut acceleration = Vector3::zeros();
        let mut jerk = Vector3::zeros();
        for i in 0..N - 1 {
            let delta = self.delta(start + i);
            let rotation = so3_exp(&(-delta * coefficients[i + 1]));
            velocity = rotation * velocity;
            let current = delta * first[i + 1];
            velocity += current;
            acceleration = rotation * acceleration;
            let cross = velocity.cross(&current);
            acceleration += delta * second[i + 1] + cross;
            jerk = rotation * jerk;
            jerk += delta * third[i + 1]
                + (velocity * second[i + 1]
                    + acceleration * (S::from_f64_lossy(2.) * first[i + 1])
                    - cross * first[i + 1])
                    .cross(&delta);
        }
        if let Some(output) = velocity_output {
            *output = velocity;
        }
        if let Some(output) = acceleration_output {
            *output = acceleration;
        }
        jerk
    }
}
