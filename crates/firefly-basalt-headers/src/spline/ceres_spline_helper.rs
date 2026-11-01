// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Sophus SE(3) adapter: Copyright 2011-2017 Hauke Strasdat,
// 2012-2017 Steven Lovegrove. See the MIT notice in ../../LICENSE.
//! Typed-knot translation of `spline/ceres_spline_helper.h` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
use super::spline_common::{compute_blending_matrix, polynomial};
use crate::{
    scalar::BasaltScalar,
    utils::sophus_utils::{hat, so3_exp, so3_inverse, so3_log, so3_matrix, so3_product},
};
use nalgebra::{Isometry3, SMatrix, SVector, Translation3, UnitQuaternion};

// Sophus SO3::leftJacobian uses epsilon squared, unlike Basalt's free helper.
fn translation_jacobian<S: BasaltScalar>(
    omega: &SVector<S, 3>,
    theta: S,
    inverse: bool,
) -> SMatrix<S, 3, 3> {
    let squared = theta * theta;
    let skew = hat(omega);
    let skew_squared = skew * skew;
    let half = S::from_f64_lossy(0.5);
    if inverse {
        let result = SMatrix::identity() - skew * half;
        if squared < S::epsilon() * S::epsilon() {
            result + skew_squared * S::from_f64_lossy(1. / 12.)
        } else {
            let half_theta = half * theta;
            result
                + skew_squared
                    * ((S::one() - half * theta * half_theta.cos() / half_theta.sin())
                        / (theta * theta))
        }
    } else if squared < S::epsilon() * S::epsilon() {
        SMatrix::identity() + skew * half
    } else {
        SMatrix::identity()
            + skew * ((S::one() - theta.cos()) / squared)
            + skew_squared * ((theta - theta.sin()) / (squared * theta))
    }
}

/// Operations required by the original helper's `GroupT` template.
pub trait SplineLieGroup<S: BasaltScalar, const D: usize>: Clone {
    #[must_use]
    fn inverse(&self) -> Self;
    #[must_use]
    fn product(&self, other: &Self) -> Self;
    fn log(&self) -> SVector<S, D>;
    fn exp(tangent: &SVector<S, D>) -> Self;
    fn adjoint(&self) -> SMatrix<S, D, D>;
    fn lie_bracket(left: &SVector<S, D>, right: &SVector<S, D>) -> SVector<S, D>;
}
impl<S: BasaltScalar> SplineLieGroup<S, 3> for UnitQuaternion<S> {
    fn inverse(&self) -> Self {
        so3_inverse(self)
    }
    fn product(&self, other: &Self) -> Self {
        so3_product(self, other)
    }
    fn log(&self) -> SVector<S, 3> {
        so3_log(self)
    }
    fn exp(tangent: &SVector<S, 3>) -> Self {
        so3_exp(tangent)
    }
    fn adjoint(&self) -> SMatrix<S, 3, 3> {
        so3_matrix(self)
    }
    fn lie_bracket(left: &SVector<S, 3>, right: &SVector<S, 3>) -> SVector<S, 3> {
        left.cross(right)
    }
}
impl<S: BasaltScalar> SplineLieGroup<S, 6> for Isometry3<S> {
    fn inverse(&self) -> Self {
        let rotation = so3_inverse(&self.rotation);
        Self::from_parts(
            Translation3::from(rotation * (-self.translation.vector)),
            rotation,
        )
    }
    fn product(&self, other: &Self) -> Self {
        Self::from_parts(
            Translation3::from(self.translation.vector + self.rotation * other.translation.vector),
            so3_product(&self.rotation, &other.rotation),
        )
    }
    fn log(&self) -> SVector<S, 6> {
        let rotation = so3_log(&self.rotation);
        let quaternion = self.rotation.quaternion();
        let squared = quaternion.vector().norm_squared();
        let theta = if squared < S::epsilon() * S::epsilon() {
            S::from_f64_lossy(2.) * squared / quaternion.w
        } else {
            let norm = squared.sqrt();
            let angle = if quaternion.w < S::zero() {
                (-norm).atan2(-quaternion.w)
            } else {
                norm.atan2(quaternion.w)
            };
            (S::from_f64_lossy(2.) * angle / norm) * norm
        };
        let mut result = SVector::<S, 6>::zeros();
        result
            .fixed_rows_mut::<3>(0)
            .copy_from(&(translation_jacobian(&rotation, theta, true) * self.translation.vector));
        result.fixed_rows_mut::<3>(3).copy_from(&rotation);
        result
    }
    fn exp(tangent: &SVector<S, 6>) -> Self {
        let rotation = tangent.fixed_rows::<3>(3).into_owned();
        let squared = rotation.norm_squared();
        let theta = if squared < S::epsilon() * S::epsilon() {
            S::zero()
        } else {
            squared.sqrt()
        };
        Self::from_parts(
            Translation3::from(
                translation_jacobian(&rotation, theta, false) * tangent.fixed_rows::<3>(0),
            ),
            so3_exp(&rotation),
        )
    }
    fn adjoint(&self) -> SMatrix<S, 6, 6> {
        let rotation = so3_matrix(&self.rotation);
        let mut result = SMatrix::<S, 6, 6>::zeros();
        result.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
        result.fixed_view_mut::<3, 3>(3, 3).copy_from(&rotation);
        result
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(hat(&self.translation.vector) * rotation));
        result
    }
    fn lie_bracket(left: &SVector<S, 6>, right: &SVector<S, 6>) -> SVector<S, 6> {
        let mut result = SVector::<S, 6>::zeros();
        result.fixed_rows_mut::<3>(0).copy_from(
            &(left.fixed_rows::<3>(3).cross(&right.fixed_rows::<3>(0))
                + left.fixed_rows::<3>(0).cross(&right.fixed_rows::<3>(3))),
        );
        result
            .fixed_rows_mut::<3>(3)
            .copy_from(&left.fixed_rows::<3>(3).cross(&right.fixed_rows::<3>(3)));
        result
    }
}

pub struct CeresSplineHelper<const N: usize>;
impl<const N: usize> CeresSplineHelper<N> {
    #[must_use]
    pub fn base_coefficients_with_time<const D: usize>(time: f64) -> SVector<f64, N> {
        polynomial::<f64, N, D>(time)
    }
    /// # Panics
    /// Requires a positive order and a derivative order representable as i32.
    pub fn evaluate<S: BasaltScalar, const DIM: usize, const D: usize>(
        knots: &[SVector<S, DIM>; N],
        u: f64,
        inverse_dt: f64,
        output: Option<&mut SVector<S, DIM>>,
    ) {
        let Some(output) = output else {
            return;
        };
        let coefficients = (compute_blending_matrix::<f64, N, false>()
            * inverse_dt.powi(i32::try_from(D).unwrap()))
            * polynomial::<f64, N, D>(u);
        output.fill(S::zero());
        for i in 0..N {
            *output += knots[i] * S::from_f64_lossy(coefficients[i]);
        }
    }
    /// # Panics
    /// Requires at least one knot.
    pub fn evaluate_lie<S: BasaltScalar, G: SplineLieGroup<S, D>, const D: usize>(
        knots: &[G; N],
        u: f64,
        inverse_dt: f64,
        mut transform: Option<&mut G>,
        velocity_output: Option<&mut SVector<S, D>>,
        acceleration_output: Option<&mut SVector<S, D>>,
        jerk_output: Option<&mut SVector<S, D>>,
    ) {
        assert!(N > 0);
        let blending = compute_blending_matrix::<f64, N, true>();
        let coefficients = blending * polynomial::<f64, N, 0>(u);
        let first = if velocity_output.is_some()
            || acceleration_output.is_some()
            || jerk_output.is_some()
        {
            (blending * inverse_dt) * polynomial::<f64, N, 1>(u)
        } else {
            SVector::zeros()
        };
        let second = if acceleration_output.is_some() || jerk_output.is_some() {
            (blending * (inverse_dt * inverse_dt)) * polynomial::<f64, N, 2>(u)
        } else {
            SVector::zeros()
        };
        let third = if jerk_output.is_some() {
            (blending * (inverse_dt * inverse_dt * inverse_dt)) * polynomial::<f64, N, 3>(u)
        } else {
            SVector::zeros()
        };
        if let Some(output) = transform.as_deref_mut() {
            *output = knots[0].clone();
        }
        let mut velocity = SVector::zeros();
        let mut acceleration = SVector::zeros();
        let mut jerk = SVector::zeros();
        for i in 0..N - 1 {
            let delta = knots[i].inverse().product(&knots[i + 1]).log();
            let exp = G::exp(&(delta * S::from_f64_lossy(coefficients[i + 1])));
            if let Some(output) = transform.as_deref_mut() {
                *output = output.product(&exp);
            }
            if velocity_output.is_some() || acceleration_output.is_some() || jerk_output.is_some() {
                let adjoint = exp.inverse().adjoint();
                velocity = adjoint * velocity;
                let current = delta * S::from_f64_lossy(first[i + 1]);
                velocity += current;
                if acceleration_output.is_some() || jerk_output.is_some() {
                    acceleration = adjoint * acceleration;
                    let bracket = G::lie_bracket(&velocity, &current);
                    acceleration += delta * S::from_f64_lossy(second[i + 1]) + bracket;
                    if jerk_output.is_some() {
                        jerk = adjoint * jerk;
                        jerk += delta * S::from_f64_lossy(third[i + 1])
                            + G::lie_bracket(
                                &(velocity * S::from_f64_lossy(second[i + 1])
                                    + acceleration * S::from_f64_lossy(2. * first[i + 1])
                                    - bracket * S::from_f64_lossy(first[i + 1])),
                                &delta,
                            );
                    }
                }
            }
        }
        if let Some(output) = velocity_output {
            *output = velocity;
        }
        if let Some(output) = acceleration_output {
            *output = acceleration;
        }
        if let Some(output) = jerk_output {
            *output = jerk;
        }
    }
}
