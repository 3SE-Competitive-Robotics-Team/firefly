// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/camera/bal_camera.hpp` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
#![allow(clippy::many_single_char_names, clippy::similar_names)]
use crate::scalar::BasaltScalar;
use nalgebra::{Matrix2, SMatrix, SVector, Vector2, Vector3, Vector4};

/// BAL radial model with `[f,k1,k2]` intrinsics and positive forward depth.
#[derive(Clone, Debug)]
pub struct BalCamera<S: BasaltScalar = f64> {
    param: Vector3<S>,
}
impl<S: BasaltScalar> Default for BalCamera<S> {
    fn default() -> Self {
        Self {
            param: Vector3::zeros(),
        }
    }
}
impl<S: BasaltScalar> BalCamera<S> {
    pub const NAME: &'static str = "bal";
    pub const N: usize = 3;
    #[must_use]
    pub fn new(param: Vector3<S>) -> Self {
        Self { param }
    }
    #[must_use]
    pub fn param(&self) -> &Vector3<S> {
        &self.param
    }
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> BalCamera<T> {
        BalCamera::new(self.param.map(|v| T::from_f64_lossy(v.as_f64())))
    }
    pub fn set_from_init(&mut self, init: Vector4<S>) {
        self.param = Vector3::new(init[0], S::zero(), S::zero());
    }
    pub fn increment(&mut self, inc: &Vector3<S>) {
        self.param += inc;
    }
    #[must_use]
    pub fn test_projections() -> [Self; 1] {
        [Self::new(Vector3::from(
            [399.752, -3.78048e-5, 5.37738e-7].map(S::from_f64_lossy),
        ))]
    }
    #[must_use]
    pub const fn test_resolutions() -> [[usize; 2]; 1] {
        [[640, 480]]
    }
    /// # Panics
    /// Panics unless the point dimension is three or four.
    pub fn project<const D: usize>(
        &self,
        point: &SVector<S, D>,
        proj: &mut Vector2<S>,
        point_jacobian: Option<&mut SMatrix<S, 2, D>>,
        param_jacobian: Option<&mut SMatrix<S, 2, 3>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let (f, k1, k2) = (self.param[0], self.param[1], self.param[2]);
        let (x, y, z) = (point[0], point[1], point[2]);
        let one = S::one();
        let two = one + one;
        let mx = x / z;
        let my = y / z;
        let mx2 = mx * mx;
        let my2 = my * my;
        let r2 = mx2 + my2;
        let r4 = r2 * r2;
        let rp = one + k1 * r2 + k2 * r4;
        proj[0] = f * mx * rp;
        proj[1] = f * my * rp;
        if let Some(j) = point_jacobian {
            j.fill(S::zero());
            let tmp = k1 + k2 * two * r2;
            j[(0, 0)] = f * (rp + two * mx2 * tmp) / z;
            j[(1, 1)] = f * (rp + two * my2 * tmp) / z;
            j[(0, 1)] = f * my * mx * two * tmp / z;
            j[(1, 0)] = j[(0, 1)];
            j[(0, 2)] = -f * mx * (rp + two * tmp * r2) / z;
            j[(1, 2)] = -f * my * (rp + two * tmp * r2) / z;
        }
        if let Some(j) = param_jacobian {
            j.fill(S::zero());
            j[(0, 0)] = mx * rp;
            j[(1, 0)] = my * rp;
            j[(0, 1)] = f * mx * r2;
            j[(1, 1)] = f * my * r2;
            j[(0, 2)] = f * mx * r4;
            j[(1, 2)] = f * my * r4;
        }
        z >= S::epsilon_sqrt()
    }
    /// Three Gauss–Newton steps, using the same explicit normal-equation inverse.
    /// # Panics
    /// Panics unless the point dimension is three or four, or if a Jacobian is
    /// requested: the upstream BAL unprojection does not implement Jacobians.
    #[allow(clippy::needless_pass_by_value)] // Keep the common camera API for unsupported Jacobians.
    pub fn unproject<const D: usize>(
        &self,
        proj: &Vector2<S>,
        point: &mut SVector<S, D>,
        pixel_jacobian: Option<&mut SMatrix<S, D, 2>>,
        param_jacobian: Option<&mut SMatrix<S, D, 3>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let (f, k1, k2) = (self.param[0], self.param[1], self.param[2]);
        let one = S::one();
        let two = one + one;
        let pp = Vector2::new(proj[0] / f, proj[1] / f);
        let mut p = pp;
        for _ in 0..3 {
            let r2 = p.dot(&p);
            let rp = one + k1 * r2 + k2 * r2 * r2;
            let pp_computed = p * rp;
            let tmp = k1 + k2 * two * r2;
            let mut j = Matrix2::zeros();
            j[(0, 0)] = rp + two * p[0] * p[0] * tmp;
            j[(1, 1)] = rp + two * p[1] * p[1] * tmp;
            j[(0, 1)] = p[0] * p[1] * two * tmp;
            j[(1, 0)] = j[(0, 1)];
            let h = j.transpose() * j;
            let inverse_det = one / (h[(0, 0)] * h[(1, 1)] - h[(1, 0)] * h[(0, 1)]);
            let inverse = Matrix2::new(
                h[(1, 1)] * inverse_det,
                -h[(0, 1)] * inverse_det,
                -h[(1, 0)] * inverse_det,
                h[(0, 0)] * inverse_det,
            );
            let dp = inverse * j.transpose() * (pp_computed - pp);
            p -= dp;
        }
        point.fill(S::zero());
        point[0] = p[0];
        point[1] = p[1];
        point[2] = one;
        point.normalize_mut();
        assert!(
            pixel_jacobian.is_none() && param_jacobian.is_none(),
            "Jacobians for unprojection are not implemented"
        );
        true
    }
}
