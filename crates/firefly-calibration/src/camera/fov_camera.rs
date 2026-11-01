// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/camera/fov_camera.hpp` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
// Preserve upstream branch thresholds and mathematical operation order.
#![allow(clippy::many_single_char_names, clippy::similar_names)]
use crate::scalar::BasaltScalar;
use nalgebra::{SMatrix, SVector, Vector2, Vector4};

/// Field-of-view camera with `[fx,fy,cx,cy,w]` intrinsics.
#[derive(Clone, Debug)]
pub struct FovCamera<S: BasaltScalar = f64> {
    param: SVector<S, 5>,
}
impl<S: BasaltScalar> Default for FovCamera<S> {
    fn default() -> Self {
        Self {
            param: SVector::zeros(),
        }
    }
}
impl<S: BasaltScalar> FovCamera<S> {
    pub const NAME: &'static str = "fov";
    pub const N: usize = 5;
    #[must_use]
    pub fn new(param: SVector<S, 5>) -> Self {
        Self { param }
    }
    #[must_use]
    pub fn param(&self) -> &SVector<S, 5> {
        &self.param
    }
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> FovCamera<T> {
        FovCamera::new(self.param.map(|v| T::from_f64_lossy(v.as_f64())))
    }
    pub fn set_from_init(&mut self, init: Vector4<S>) {
        self.param.fixed_rows_mut::<4>(0).copy_from(&init);
        self.param[4] = S::one();
    }
    pub fn increment(&mut self, inc: &SVector<S, 5>) {
        self.param += inc;
    }
    #[must_use]
    pub fn test_projections() -> [Self; 1] {
        [Self::new(SVector::from(
            [379.045, 379.008, 505.512, 509.969, 0.925_948_750_190_569_7].map(S::from_f64_lossy),
        ))]
    }
    #[must_use]
    pub const fn test_resolutions() -> [[usize; 2]; 1] {
        [[752, 480]]
    }
    /// Retains the source's small-radius and small-w branches.
    /// # Panics
    /// Panics unless the point dimension is three or four.
    pub fn project<const D: usize>(
        &self,
        point: &SVector<S, D>,
        proj: &mut Vector2<S>,
        point_jacobian: Option<&mut SMatrix<S, 2, D>>,
        param_jacobian: Option<&mut SMatrix<S, 2, 5>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let [fx, fy, cx, cy, w] = self.param.as_slice().try_into().unwrap();
        let (x, y, z) = (point[0], point[1], point[2]);
        let one = S::one();
        let two = one + one;
        let half = S::from_f64_lossy(0.5);
        let r2 = x * x + y * y;
        let r = r2.sqrt();
        let z2 = z * z;
        let tanwhalf = (w / two).tan();
        let atan_wrd = (two * tanwhalf * r).atan2(z);
        let mut rd = one;
        let mut d_rd_d_w = S::zero();
        let mut d_rd_d_x = S::zero();
        let mut d_rd_d_y = S::zero();
        let mut d_rd_d_z = S::zero();
        let tmp1 = one / (w / two).cos();
        let d_tanwhalf_d_w = half * tmp1 * tmp1;
        let tmp = z2 + S::from_f64_lossy(4.) * tanwhalf * tanwhalf * r2;
        let d_atan_wrd_d_w = two * r * d_tanwhalf_d_w * z / tmp;
        let mut valid = true;
        if w > S::epsilon_sqrt() {
            if r2 < S::epsilon_sqrt() {
                if z < S::epsilon_sqrt() {
                    valid = false;
                }
                rd = two * tanwhalf / w;
                d_rd_d_w = two * (d_tanwhalf_d_w * w - tanwhalf) / (w * w);
            } else {
                rd = atan_wrd / (r * w);
                d_rd_d_w = (d_atan_wrd_d_w * w - atan_wrd) / (r * w * w);
                let d_r_d_x = x / r;
                let d_r_d_y = y / r;
                let d_atan_wrd_d_x = two * tanwhalf * d_r_d_x * z / tmp;
                let d_atan_wrd_d_y = two * tanwhalf * d_r_d_y * z / tmp;
                let d_atan_wrd_d_z = -two * tanwhalf * r / tmp;
                d_rd_d_x = (d_atan_wrd_d_x * r - d_r_d_x * atan_wrd) / (r * r * w);
                d_rd_d_y = (d_atan_wrd_d_y * r - d_r_d_y * atan_wrd) / (r * r * w);
                d_rd_d_z = d_atan_wrd_d_z / (r * w);
            }
        }
        let mx = x * rd;
        let my = y * rd;
        proj[0] = fx * mx + cx;
        proj[1] = fy * my + cy;
        if let Some(j) = point_jacobian {
            j.fill(S::zero());
            j[(0, 0)] = fx * (d_rd_d_x * x + rd);
            j[(0, 1)] = fx * d_rd_d_y * x;
            j[(0, 2)] = fx * d_rd_d_z * x;
            j[(1, 0)] = fy * d_rd_d_x * y;
            j[(1, 1)] = fy * (d_rd_d_y * y + rd);
            j[(1, 2)] = fy * d_rd_d_z * y;
        }
        if let Some(j) = param_jacobian {
            j.fill(S::zero());
            j[(0, 0)] = mx;
            j[(0, 2)] = one;
            j[(1, 1)] = my;
            j[(1, 3)] = one;
            j[(0, 4)] = fx * x * d_rd_d_w;
            j[(1, 4)] = fy * y * d_rd_d_w;
        }
        valid
    }
    /// Unproject and normalize the ray, including normalization Jacobians.
    /// # Panics
    /// Panics unless the point dimension is three or four.
    pub fn unproject<const D: usize>(
        &self,
        proj: &Vector2<S>,
        point: &mut SVector<S, D>,
        mut pixel_jacobian: Option<&mut SMatrix<S, D, 2>>,
        mut param_jacobian: Option<&mut SMatrix<S, D, 5>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let [fx, fy, cx, cy, w] = self.param.as_slice().try_into().unwrap();
        let one = S::one();
        let two = one + one;
        let tan_w_2 = (w / two).tan();
        let mul2tanwby2 = tan_w_2 * two;
        let mx = (proj[0] - cx) / fx;
        let my = (proj[1] - cy) / fy;
        let rd = (mx * mx + my * my).sqrt();
        let mut ru = one;
        let mut sin_rd_w = S::zero();
        let mut cos_rd_w = one;
        let mut d_ru_d_rd = S::zero();
        let mut rd_inv = one;
        if mul2tanwby2 > S::epsilon_sqrt() && rd > S::epsilon_sqrt() {
            sin_rd_w = (rd * w).sin();
            cos_rd_w = (rd * w).cos();
            ru = sin_rd_w / (rd * mul2tanwby2);
            rd_inv = one / rd;
            d_ru_d_rd = (w * cos_rd_w * rd - sin_rd_w) * rd_inv * rd_inv / mul2tanwby2;
        }
        point.fill(S::zero());
        point[0] = mx * ru;
        point[1] = my * ru;
        point[2] = cos_rd_w;
        if pixel_jacobian.is_some() || param_jacobian.is_some() {
            let mut c0 = SVector::<S, D>::zeros();
            let mut c1 = SVector::<S, D>::zeros();
            c0[0] = (ru + mx * d_ru_d_rd * mx * rd_inv) / fx;
            c0[1] = my * d_ru_d_rd * mx * rd_inv / fx;
            c0[2] = -sin_rd_w * w * mx * rd_inv / fx;
            c1[0] = my * d_ru_d_rd * mx * rd_inv / fy;
            c1[1] = (ru + my * d_ru_d_rd * my * rd_inv) / fy;
            c1[2] = -sin_rd_w * w * my * rd_inv / fy;
            if let Some(j) = pixel_jacobian.as_deref_mut() {
                j.set_column(0, &c0);
                j.set_column(1, &c1);
            }
            if let Some(j) = param_jacobian.as_deref_mut() {
                j.fill(S::zero());
                j.set_column(2, &(-c0));
                j.set_column(3, &(-c1));
                j.set_column(0, &(-c0 * mx));
                j.set_column(1, &(-c1 * my));
                let tmp = (cos_rd_w
                    - (tan_w_2 * tan_w_2 + one) * sin_rd_w * rd_inv / (two * tan_w_2))
                    / mul2tanwby2;
                j[(0, 4)] = mx * tmp;
                j[(1, 4)] = my * tmp;
                j[(2, 4)] = -sin_rd_w * rd;
            }
            let norm = point.norm();
            let norm_inv = one / norm;
            let norm_inv2 = norm_inv * norm_inv;
            let norm_inv3 = norm_inv2 * norm_inv;
            let mut normalization = SMatrix::<S, D, D>::zeros();
            normalization[(0, 0)] = norm_inv * (one - point[0] * point[0] * norm_inv2);
            normalization[(1, 0)] = -point[1] * point[0] * norm_inv3;
            normalization[(2, 0)] = -point[2] * point[0] * norm_inv3;
            normalization[(0, 1)] = -point[1] * point[0] * norm_inv3;
            normalization[(1, 1)] = norm_inv * (one - point[1] * point[1] * norm_inv2);
            normalization[(2, 1)] = -point[1] * point[2] * norm_inv3;
            normalization[(0, 2)] = -point[2] * point[0] * norm_inv3;
            normalization[(1, 2)] = -point[2] * point[1] * norm_inv3;
            normalization[(2, 2)] = norm_inv * (one - point[2] * point[2] * norm_inv2);
            if let Some(j) = pixel_jacobian {
                *j = normalization * *j;
            }
            if let Some(j) = param_jacobian {
                *j = normalization * *j;
            }
        }
        *point /= point.norm();
        true
    }
}
