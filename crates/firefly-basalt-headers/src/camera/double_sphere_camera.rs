// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/camera/double_sphere_camera.hpp` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
// Preserve the upstream mathematical names and arithmetic order for comparison.
#![allow(clippy::many_single_char_names, clippy::similar_names)]
use super::clamp;
use crate::scalar::BasaltScalar;
use nalgebra::{SMatrix, SVector, Vector2, Vector4};

/// Double sphere model with `[fx,fy,cx,cy,xi,alpha]` intrinsics.
#[derive(Clone, Debug)]
pub struct DoubleSphereCamera<S: BasaltScalar = f64> {
    param: SVector<S, 6>,
}
impl<S: BasaltScalar> Default for DoubleSphereCamera<S> {
    fn default() -> Self {
        Self {
            param: SVector::zeros(),
        }
    }
}
impl<S: BasaltScalar> DoubleSphereCamera<S> {
    pub const NAME: &'static str = "ds";
    pub const N: usize = 6;
    #[must_use]
    pub fn new(param: SVector<S, 6>) -> Self {
        Self { param }
    }
    #[must_use]
    pub fn param(&self) -> &SVector<S, 6> {
        &self.param
    }
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> DoubleSphereCamera<T> {
        DoubleSphereCamera::new(self.param.map(|v| T::from_f64_lossy(v.as_f64())))
    }
    pub fn set_from_init(&mut self, init: Vector4<S>) {
        self.param.fixed_rows_mut::<4>(0).copy_from(&init);
        self.param[4] = S::zero();
        self.param[5] = S::from_f64_lossy(0.5);
    }
    pub fn increment(&mut self, inc: &SVector<S, 6>) {
        self.param += inc;
        self.param[4] = clamp(self.param[4], -S::one(), S::one());
        self.param[5] = clamp(self.param[5], S::zero(), S::one());
    }
    #[must_use]
    pub fn test_projections() -> [Self; 1] {
        [Self::new(SVector::from(
            [402.5, 400., 505., 509., -0.075_347, 0.743_925].map(S::from_f64_lossy),
        ))]
    }
    /// Project a 3D point, ignoring a fourth homogeneous coordinate.
    /// Invalid points retain the upstream arithmetic outputs and Jacobians.
    /// # Panics
    /// Panics unless the point dimension is three or four.
    pub fn project<const D: usize>(
        &self,
        point: &SVector<S, D>,
        proj: &mut Vector2<S>,
        point_jacobian: Option<&mut SMatrix<S, 2, D>>,
        param_jacobian: Option<&mut SMatrix<S, 2, 6>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let [fx, fy, cx, cy, xi, alpha] = self.param.as_slice().try_into().unwrap();
        let (x, y, z) = (point[0], point[1], point[2]);
        let one = S::one();
        let two = one + one;
        let half = S::from_f64_lossy(0.5);
        let (xx, yy, zz) = (x * x, y * y, z * z);
        let r2 = xx + yy;
        let d1 = (r2 + zz).sqrt();
        let w1 = if alpha > half {
            (one - alpha) / alpha
        } else {
            alpha / (one - alpha)
        };
        let w2 = (w1 + xi) / (two * w1 * xi + xi * xi + one).sqrt();
        let valid = z > -w2 * d1;
        let k = xi * d1 + z;
        let d2 = (r2 + k * k).sqrt();
        let norm = alpha * d2 + (one - alpha) * k;
        let mx = x / norm;
        let my = y / norm;
        proj[0] = fx * mx + cx;
        proj[1] = fy * my + cy;
        if let Some(j) = point_jacobian {
            let norm2 = norm * norm;
            let xy = x * y;
            let tt2 = xi * z / d1 + one;
            let d_norm_d_r2 = (xi * (one - alpha) / d1 + alpha * (xi * k / d1 + one) / d2) / norm2;
            let tmp2 = ((one - alpha) * tt2 + alpha * k * tt2 / d2) / norm2;
            j.fill(S::zero());
            j[(0, 0)] = fx * (one / norm - xx * d_norm_d_r2);
            j[(1, 0)] = -fy * xy * d_norm_d_r2;
            j[(0, 1)] = -fx * xy * d_norm_d_r2;
            j[(1, 1)] = fy * (one / norm - yy * d_norm_d_r2);
            j[(0, 2)] = -fx * x * tmp2;
            j[(1, 2)] = -fy * y * tmp2;
        }
        if let Some(j) = param_jacobian {
            let norm2 = norm * norm;
            j.fill(S::zero());
            j[(0, 0)] = mx;
            j[(0, 2)] = one;
            j[(1, 1)] = my;
            j[(1, 3)] = one;
            let tmp4 = (alpha - one - alpha * k / d2) * d1 / norm2;
            let tmp5 = (k - d2) / norm2;
            j[(0, 4)] = fx * x * tmp4;
            j[(1, 4)] = fy * y * tmp4;
            j[(0, 5)] = fx * x * tmp5;
            j[(1, 5)] = fy * y * tmp5;
        }
        valid
    }
    /// Unproject into a unit ray; homogeneous fourth coordinates are zero.
    /// Validity follows the upstream strict radial bound, including NaN behavior.
    /// # Panics
    /// Panics unless the point dimension is three or four.
    pub fn unproject<const D: usize>(
        &self,
        proj: &Vector2<S>,
        point: &mut SVector<S, D>,
        pixel_jacobian: Option<&mut SMatrix<S, D, 2>>,
        param_jacobian: Option<&mut SMatrix<S, D, 6>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let [fx, fy, cx, cy, xi, alpha] = self.param.as_slice().try_into().unwrap();
        let one = S::one();
        let two = one + one;
        let half = S::from_f64_lossy(0.5);
        let mx = (proj[0] - cx) / fx;
        let my = (proj[1] - cy) / fy;
        let r2 = mx * mx + my * my;
        let valid = !(alpha > half && r2 >= one / (two * alpha - one));
        let xi2_2 = alpha * alpha;
        let xi1_2 = xi * xi;
        let sqrt2 = (one - (two * alpha - one) * r2).sqrt();
        let norm2 = alpha * sqrt2 + one - alpha;
        let mz = (one - xi2_2 * r2) / norm2;
        let mz2 = mz * mz;
        let norm1 = mz2 + r2;
        let sqrt1 = (mz2 + (one - xi1_2) * r2).sqrt();
        let k = (mz * xi + sqrt1) / norm1;
        point.fill(S::zero());
        point[0] = k * mx;
        point[1] = k * my;
        point[2] = k * mz - xi;
        if pixel_jacobian.is_some() || param_jacobian.is_some() {
            let norm2_2 = norm2 * norm2;
            let norm1_2 = norm1 * norm1;
            let d_mz_d_r2 =
                (half * alpha - xi2_2) * (r2 * xi2_2 - one) / (sqrt2 * norm2_2) - xi2_2 / norm2;
            let d_mz_d_mx = two * mx * d_mz_d_r2;
            let d_mz_d_my = two * my * d_mz_d_r2;
            let d_k_d_mz = (norm1 * (xi * sqrt1 + mz) - two * mz * (mz * xi + sqrt1) * sqrt1)
                / (norm1_2 * sqrt1);
            let d_k_d_r2 = (xi * d_mz_d_r2 + half / sqrt1 * (two * mz * d_mz_d_r2 + one - xi1_2))
                / norm1
                - (mz * xi + sqrt1) * (two * mz * d_mz_d_r2 + one) / norm1_2;
            let d_k_d_mx = d_k_d_r2 * two * mx;
            let d_k_d_my = d_k_d_r2 * two * my;
            let mut c0 = SVector::<S, D>::zeros();
            let mut c1 = SVector::<S, D>::zeros();
            c0[0] = mx * d_k_d_mx + k;
            c0[1] = my * d_k_d_mx;
            c0[2] = mz * d_k_d_mx + k * d_mz_d_mx;
            c0 /= fx;
            c1[0] = mx * d_k_d_my;
            c1[1] = my * d_k_d_my + k;
            c1[2] = mz * d_k_d_my + k * d_mz_d_my;
            c1 /= fy;
            if let Some(j) = pixel_jacobian {
                j.set_column(0, &c0);
                j.set_column(1, &c1);
            }
            if let Some(j) = param_jacobian {
                let d_k_d_xi1 = (mz * sqrt1 - xi * r2) / (sqrt1 * norm1);
                let d_mz_d_xi2 = ((one - r2 * xi2_2) * (r2 * alpha / sqrt2 - sqrt2 + one) / norm2
                    - two * r2 * alpha)
                    / norm2;
                let d_k_d_xi2 = d_k_d_mz * d_mz_d_xi2;
                j.fill(S::zero());
                j.set_column(0, &(-c0 * mx));
                j.set_column(1, &(-c1 * my));
                j.set_column(2, &(-c0));
                j.set_column(3, &(-c1));
                j[(0, 4)] = mx * d_k_d_xi1;
                j[(1, 4)] = my * d_k_d_xi1;
                j[(2, 4)] = mz * d_k_d_xi1 - one;
                j[(0, 5)] = mx * d_k_d_xi2;
                j[(1, 5)] = my * d_k_d_xi2;
                j[(2, 5)] = mz * d_k_d_xi2 + k * d_mz_d_xi2;
            }
        }
        valid
    }
}
