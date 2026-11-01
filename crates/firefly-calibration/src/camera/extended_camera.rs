// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/camera/extended_camera.hpp` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
// Preserve upstream mathematical symbols for source comparison.
#![allow(clippy::many_single_char_names, clippy::similar_names)]
use super::clamp;
use crate::scalar::BasaltScalar;
use nalgebra::{SMatrix, SVector, Vector2, Vector4};

/// Extended unified model with `[fx,fy,cx,cy,alpha,beta]` intrinsics.
#[derive(Clone, Debug)]
pub struct ExtendedUnifiedCamera<S: BasaltScalar = f64> {
    param: SVector<S, 6>,
}
impl<S: BasaltScalar> Default for ExtendedUnifiedCamera<S> {
    fn default() -> Self {
        Self {
            param: SVector::zeros(),
        }
    }
}
impl<S: BasaltScalar> ExtendedUnifiedCamera<S> {
    pub const NAME: &'static str = "eucm";
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
    pub fn cast<T: BasaltScalar>(&self) -> ExtendedUnifiedCamera<T> {
        ExtendedUnifiedCamera::new(self.param.map(|v| T::from_f64_lossy(v.as_f64())))
    }
    pub fn set_from_init(&mut self, init: Vector4<S>) {
        self.param.fixed_rows_mut::<4>(0).copy_from(&init);
        self.param[4] = S::from_f64_lossy(0.5);
        self.param[5] = S::one();
    }
    pub fn increment(&mut self, inc: &SVector<S, 6>) {
        self.param += inc;
        self.param[4] = clamp(self.param[4], S::zero(), S::one());
        if self.param[5] < S::epsilon_sqrt() {
            self.param[5] = S::epsilon_sqrt();
        }
    }
    #[must_use]
    pub fn test_projections() -> [Self; 2] {
        [
            [
                460.764_846_515_664_68,
                459.405_101_804_948_3,
                365.893_716_130_961_5,
                249.334_998_697_524_45,
                0.590_336_591_522_714_3,
                1.127_468_196_965_374,
            ],
            [
                191.147_998_166_487_48,
                191.131_509_465_851_35,
                254.958_577_152_331_18,
                256.881_546_623_589_8,
                0.629_106_087_116_184_2,
                1.041_806_740_313_969_3,
            ],
        ]
        .map(|p| Self::new(SVector::from(p.map(S::from_f64_lossy))))
    }
    #[must_use]
    pub const fn test_resolutions() -> [[usize; 2]; 2] {
        [[752, 480], [512, 512]]
    }
    /// Projects a point and computes requested Jacobians even when the validity test fails.
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
        let [fx, fy, cx, cy, alpha, beta] = self.param.as_slice().try_into().unwrap();
        let (x, y, z) = (point[0], point[1], point[2]);
        let one = S::one();
        let half = S::from_f64_lossy(0.5);
        let r2 = x * x + y * y;
        let rho2 = beta * r2 + z * z;
        let rho = rho2.sqrt();
        let norm = alpha * rho + (one - alpha) * z;
        let mx = x / norm;
        let my = y / norm;
        *proj = Vector2::new(fx * mx + cx, fy * my + cy);
        let w = if alpha > half {
            (one - alpha) / alpha
        } else {
            alpha / (one - alpha)
        };
        let valid = z > -w * rho;
        if let Some(j) = point_jacobian {
            let denom = norm * norm * rho;
            let mid = -(alpha * beta * x * y);
            let add = norm * rho;
            let addz = alpha * z + (one - alpha) * rho;
            j.fill(S::zero());
            j[(0, 0)] = fx * (add - x * x * alpha * beta);
            j[(1, 0)] = fy * mid;
            j[(0, 1)] = fx * mid;
            j[(1, 1)] = fy * (add - y * y * alpha * beta);
            j[(0, 2)] = -fx * x * addz;
            j[(1, 2)] = -fy * y * addz;
            *j /= denom;
        }
        if let Some(j) = param_jacobian {
            let norm2 = norm * norm;
            j.fill(S::zero());
            j[(0, 0)] = mx;
            j[(0, 2)] = one;
            j[(1, 1)] = my;
            j[(1, 3)] = one;
            let tmp_x = -fx * x / norm2;
            let tmp_y = -fy * y / norm2;
            let tmp4 = rho - z;
            j[(0, 4)] = tmp_x * tmp4;
            j[(1, 4)] = tmp_y * tmp4;
            let tmp5 = half * alpha * r2 / rho;
            j[(0, 5)] = tmp_x * tmp5;
            j[(1, 5)] = tmp_y * tmp5;
        }
        valid
    }
    /// Unprojects pixels into unit rays, setting the fourth homogeneous coordinate to zero.
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
        let [fx, fy, cx, cy, alpha, beta] = self.param.as_slice().try_into().unwrap();
        let one = S::one();
        let two = one + one;
        let half = S::from_f64_lossy(0.5);
        let mx = (proj[0] - cx) / fx;
        let my = (proj[1] - cy) / fy;
        let r2 = mx * mx + my * my;
        let gamma = one - alpha;
        let valid = !(alpha > half && r2 >= one / ((alpha - gamma) * beta));
        let tmp1 = one - alpha * alpha * beta * r2;
        let tmp_sqrt = (one - (alpha - gamma) * beta * r2).sqrt();
        let tmp2 = alpha * tmp_sqrt + gamma;
        let k = tmp1 / tmp2;
        let norm = (r2 + k * k).sqrt();
        point.fill(S::zero());
        point[0] = mx / norm;
        point[1] = my / norm;
        point[2] = k / norm;
        if pixel_jacobian.is_some() || param_jacobian.is_some() {
            let norm2 = norm * norm;
            let tmp2_2 = tmp2 * tmp2;
            let d_k_d_r2 =
                half * alpha * beta * (-two * alpha * tmp2 + tmp1 * (alpha - gamma) / tmp_sqrt)
                    / tmp2_2;
            let d_norm_inv_d_r2 = -half * (one + two * k * d_k_d_r2) / norm2;
            let mut c0 = SVector::<S, D>::zeros();
            c0[0] = one + two * mx * mx * d_norm_inv_d_r2;
            c0[1] = two * my * mx * d_norm_inv_d_r2;
            c0[2] = two * mx * (k * d_norm_inv_d_r2 + d_k_d_r2);
            c0 /= fx * norm;
            let mut c1 = SVector::<S, D>::zeros();
            c1[0] = two * my * mx * d_norm_inv_d_r2;
            c1[1] = one + two * my * my * d_norm_inv_d_r2;
            c1[2] = two * my * (k * d_norm_inv_d_r2 + d_k_d_r2);
            c1 /= fy * norm;
            if let Some(j) = pixel_jacobian {
                j.set_column(0, &c0);
                j.set_column(1, &c1);
            }
            if let Some(j) = param_jacobian {
                j.fill(S::zero());
                j.set_column(0, &(-c0 * mx));
                j.set_column(1, &(-c1 * my));
                j.set_column(2, &(-c0));
                j.set_column(3, &(-c1));
                let d_k_d_alpha = (-two * alpha * beta * r2 * tmp2
                    - (tmp_sqrt - alpha * beta * r2 / tmp_sqrt - one) * tmp1)
                    / tmp2_2;
                let d_k_d_beta =
                    alpha * r2 * (half * tmp1 * (alpha - gamma) / tmp_sqrt - alpha * tmp2) / tmp2_2;
                let d_norm_inv_d_k = -k / norm2;
                j[(0, 4)] = mx * d_norm_inv_d_k * d_k_d_alpha;
                j[(1, 4)] = my * d_norm_inv_d_k * d_k_d_alpha;
                j[(2, 4)] = (k * d_norm_inv_d_k + one) * d_k_d_alpha;
                j.column_mut(4).scale_mut(one / norm);
                j[(0, 5)] = mx * d_norm_inv_d_k * d_k_d_beta;
                j[(1, 5)] = my * d_norm_inv_d_k * d_k_d_beta;
                j[(2, 5)] = (k * d_norm_inv_d_k + one) * d_k_d_beta;
                j.column_mut(5).scale_mut(one / norm);
            }
        }
        valid
    }
}
