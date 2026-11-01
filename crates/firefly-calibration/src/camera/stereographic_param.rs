// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are indexed in docs/licenses/README.md.
//! Translation of `include/basalt/camera/stereographic_param.hpp` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.

use crate::scalar::BasaltScalar;
use nalgebra::{SMatrix, Vector2, Vector4};
use std::marker::PhantomData;

/// Minimal coordinates of a direction, excluding the negative z axis.
pub struct StereographicParam<S: BasaltScalar = f64>(PhantomData<S>);

impl<S: BasaltScalar> StereographicParam<S> {
    /// Projects a homogeneous direction. Its fourth coordinate is ignored.
    /// The projection pole and zero vector retain the source's IEEE results.
    #[must_use]
    pub fn project(point: &Vector4<S>, jacobian: Option<&mut SMatrix<S, 2, 4>>) -> Vector2<S> {
        let length = point.fixed_rows::<3>(0).norm();
        let denominator = point[2] + length;
        let inverse = S::one() / denominator;
        let result = Vector2::new(point[0] * inverse, point[1] * inverse);
        if let Some(jacobian) = jacobian {
            let inverse_squared = inverse * inverse;
            let coefficient = -inverse_squared / length;
            jacobian.fill(S::zero());
            jacobian[(0, 0)] = inverse + point[0] * point[0] * coefficient;
            jacobian[(1, 0)] = point[0] * point[1] * coefficient;
            jacobian[(1, 1)] = inverse + point[1] * point[1] * coefficient;
            jacobian[(0, 1)] = point[0] * point[1] * coefficient;
            jacobian[(0, 2)] = point[0] * denominator * coefficient;
            jacobian[(1, 2)] = point[1] * denominator * coefficient;
        }
        result
    }

    /// Unprojects to a unit direction with fourth coordinate zero.
    #[must_use]
    pub fn unproject(
        projection: &Vector2<S>,
        jacobian: Option<&mut SMatrix<S, 4, 2>>,
    ) -> Vector4<S> {
        let x_squared = projection[0] * projection[0];
        let y_squared = projection[1] * projection[1];
        let radius_squared = x_squared + y_squared;
        let inverse = (S::one() + S::one()) / (S::one() + radius_squared);
        let result = Vector4::new(
            projection[0] * inverse,
            projection[1] * inverse,
            inverse - S::one(),
            S::zero(),
        );
        if let Some(jacobian) = jacobian {
            let inverse_squared = inverse * inverse;
            let product = projection[0] * projection[1];
            jacobian[(0, 0)] = inverse - x_squared * inverse_squared;
            jacobian[(0, 1)] = -product * inverse_squared;
            jacobian[(1, 0)] = -product * inverse_squared;
            jacobian[(1, 1)] = inverse - y_squared * inverse_squared;
            jacobian[(2, 0)] = -projection[0] * inverse_squared;
            jacobian[(2, 1)] = -projection[1] * inverse_squared;
            jacobian[(3, 0)] = S::zero();
            jacobian[(3, 1)] = S::zero();
        }
        result
    }
}
