// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are in ../../LICENSE.
//! Translation of `include/basalt/camera/pinhole_camera.hpp` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.

use crate::scalar::BasaltScalar;
use nalgebra::{SMatrix, SVector, Vector2, Vector4};

/// Perspective camera with intrinsics `[fx, fy, cx, cy]` in pixels.
#[derive(Clone, Debug)]
pub struct PinholeCamera<S: BasaltScalar = f64> {
    param: Vector4<S>,
}

impl<S: BasaltScalar> Default for PinholeCamera<S> {
    fn default() -> Self {
        Self {
            param: Vector4::zeros(),
        }
    }
}

impl<S: BasaltScalar> PinholeCamera<S> {
    pub const NAME: &'static str = "pinhole";
    pub const N: usize = 4;

    #[must_use]
    pub fn new(param: Vector4<S>) -> Self {
        Self { param }
    }

    #[must_use]
    pub fn param(&self) -> &Vector4<S> {
        &self.param
    }

    pub fn set_from_init(&mut self, param: Vector4<S>) {
        self.param = param;
    }

    pub fn increment(&mut self, increment: &Vector4<S>) {
        self.param += increment;
    }

    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> PinholeCamera<T> {
        PinholeCamera::new(self.param.map(|value| T::from_f64_lossy(value.as_f64())))
    }

    /// Intrinsics supplied by the upstream camera-model tests.
    #[must_use]
    pub fn test_projections() -> [Self; 2] {
        [
            [
                460.764_846_515_664_68,
                459.405_101_804_948_3,
                365.893_716_130_961_5,
                249.334_998_697_524_45,
            ],
            [
                191.147_998_166_487_48,
                191.131_509_465_851_35,
                254.958_577_152_331_18,
                256.881_546_623_589_8,
            ],
        ]
        .map(|parameters| Self::new(Vector4::from(parameters.map(S::from_f64_lossy))))
    }

    #[must_use]
    pub const fn test_resolutions() -> [[usize; 2]; 2] {
        [[752, 480], [512, 512]]
    }

    /// Projects Euclidean or homogeneous points; the fourth coordinate is ignored.
    /// Invalid depth still produces the upstream arithmetic result and Jacobians.
    /// # Panics
    /// Panics for point dimensions other than three or four.
    pub fn project<const D: usize>(
        &self,
        point: &SVector<S, D>,
        projection: &mut Vector2<S>,
        point_jacobian: Option<&mut SMatrix<S, 2, D>>,
        param_jacobian: Option<&mut SMatrix<S, 2, 4>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let [fx, fy, cx, cy] = self.param.as_slice().try_into().unwrap();
        let x = point[0];
        let y = point[1];
        let z = point[2];
        projection[0] = fx * x / z + cx;
        projection[1] = fy * y / z + cy;
        let valid = z >= S::epsilon_sqrt();
        if let Some(jacobian) = point_jacobian {
            jacobian.fill(S::zero());
            let squared_depth = z * z;
            jacobian[(0, 0)] = fx / z;
            jacobian[(0, 2)] = -fx * x / squared_depth;
            jacobian[(1, 1)] = fy / z;
            jacobian[(1, 2)] = -fy * y / squared_depth;
        }
        if let Some(jacobian) = param_jacobian {
            jacobian.fill(S::zero());
            jacobian[(0, 0)] = x / z;
            jacobian[(0, 2)] = S::one();
            jacobian[(1, 1)] = y / z;
            jacobian[(1, 3)] = S::one();
        }
        valid
    }

    /// Returns a unit-length ray; a homogeneous output has fourth coordinate zero.
    /// Returns true as upstream does, including for zero focal lengths.
    /// # Panics
    /// Panics for point dimensions other than three or four.
    pub fn unproject<const D: usize>(
        &self,
        projection: &Vector2<S>,
        point: &mut SVector<S, D>,
        pixel_jacobian: Option<&mut SMatrix<S, D, 2>>,
        param_jacobian: Option<&mut SMatrix<S, D, 4>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let [fx, fy, cx, cy] = self.param.as_slice().try_into().unwrap();
        let mx = (projection[0] - cx) / fx;
        let my = (projection[1] - cy) / fy;
        let radius_squared = mx * mx + my * my;
        let norm = (S::one() + radius_squared).sqrt();
        let inverse_norm = S::one() / norm;
        point.fill(S::zero());
        point[0] = mx * inverse_norm;
        point[1] = my * inverse_norm;
        point[2] = inverse_norm;
        if pixel_jacobian.is_some() || param_jacobian.is_some() {
            let two = S::one() + S::one();
            let derivative = -(S::one() / two) * inverse_norm * inverse_norm * inverse_norm;
            let mut column_x = SVector::<S, D>::zeros();
            let mut column_y = SVector::<S, D>::zeros();
            column_x[0] = (inverse_norm + two * mx * mx * derivative) / fx;
            column_x[1] = (two * my * mx * derivative) / fx;
            column_x[2] = two * mx * derivative / fx;
            column_y[0] = (two * my * mx * derivative) / fy;
            column_y[1] = (inverse_norm + two * my * my * derivative) / fy;
            column_y[2] = two * my * derivative / fy;
            if let Some(jacobian) = pixel_jacobian {
                jacobian.set_column(0, &column_x);
                jacobian.set_column(1, &column_y);
            }
            if let Some(jacobian) = param_jacobian {
                jacobian.set_column(2, &(-column_x));
                jacobian.set_column(3, &(-column_y));
                jacobian.set_column(0, &(-column_x * mx));
                jacobian.set_column(1, &(-column_y * my));
            }
        }
        true
    }
}
