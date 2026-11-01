// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/camera/unified_camera.hpp` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
// Preserve the upstream mathematical names and arithmetic order for comparison.
#![allow(clippy::many_single_char_names, clippy::similar_names)]
use super::clamp;
use crate::scalar::BasaltScalar;
use nalgebra::{SMatrix, SVector, Vector2, Vector4};

/// Unified camera model with `[fx,fy,cx,cy,alpha]` intrinsics.
#[derive(Clone, Debug)]
pub struct UnifiedCamera<S: BasaltScalar = f64> {
    param: SVector<S, 5>,
}
impl<S: BasaltScalar> Default for UnifiedCamera<S> {
    fn default() -> Self {
        Self {
            param: SVector::zeros(),
        }
    }
}
impl<S: BasaltScalar> UnifiedCamera<S> {
    pub const NAME: &'static str = "ucm";
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
    pub fn cast<T: BasaltScalar>(&self) -> UnifiedCamera<T> {
        UnifiedCamera::new(self.param.map(|v| T::from_f64_lossy(v.as_f64())))
    }
    pub fn set_from_init(&mut self, init: Vector4<S>) {
        self.param.fixed_rows_mut::<4>(0).copy_from(&init);
        self.param[4] = S::from_f64_lossy(0.5);
    }
    pub fn increment(&mut self, inc: &SVector<S, 5>) {
        self.param += inc;
        self.param[4] = clamp(self.param[4], S::zero(), S::one());
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
            ],
            [
                191.147_998_166_487_48,
                191.131_509_465_851_35,
                254.958_577_152_331_18,
                256.881_546_623_589_8,
                0.629_106_087_116_184_2,
            ],
        ]
        .map(|p| Self::new(SVector::from(p.map(S::from_f64_lossy))))
    }
    #[must_use]
    pub const fn test_resolutions() -> [[usize; 2]; 2] {
        [[752, 480], [512, 512]]
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
        param_jacobian: Option<&mut SMatrix<S, 2, 5>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let [fx, fy, cx, cy, alpha] = self.param.as_slice().try_into().unwrap();
        let (x, y, z) = (point[0], point[1], point[2]);
        let one = S::one();
        let half = S::from_f64_lossy(0.5);
        let r2 = x * x + y * y;
        let rho = (r2 + z * z).sqrt();
        let norm = alpha * rho + (one - alpha) * z;
        let mx = x / norm;
        let my = y / norm;
        proj[0] = fx * mx + cx;
        proj[1] = fy * my + cy;
        let w = if alpha > half {
            (one - alpha) / alpha
        } else {
            alpha / (one - alpha)
        };
        let valid = z > -w * rho;
        if let Some(j) = point_jacobian {
            let denom = norm * norm * rho;
            let mid = -(alpha * x * y);
            let add = norm * rho;
            let addz = alpha * z + (one - alpha) * rho;
            j.fill(S::zero());
            j[(0, 0)] = fx * (add - x * x * alpha);
            j[(1, 0)] = fy * mid;
            j[(0, 1)] = fx * mid;
            j[(1, 1)] = fy * (add - y * y * alpha);
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
        }
        valid
    }
    /// Unproject into a unit ray; homogeneous fourth coordinates are zero.
    /// Preserves the upstream validity test independently of nonfinite results.
    /// # Panics
    /// Panics unless the point dimension is three or four.
    pub fn unproject<const D: usize>(
        &self,
        proj: &Vector2<S>,
        point: &mut SVector<S, D>,
        pixel_jacobian: Option<&mut SMatrix<S, D, 2>>,
        param_jacobian: Option<&mut SMatrix<S, D, 5>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let [fx, fy, cx, cy, alpha] = self.param.as_slice().try_into().unwrap();
        let one = S::one();
        let two = one + one;
        let half = S::from_f64_lossy(0.5);
        let xi = alpha / (one - alpha);
        let mxx = (proj[0] - cx) / fx;
        let myy = (proj[1] - cy) / fy;
        let mx = (one - alpha) * mxx;
        let my = (one - alpha) * myy;
        let r2 = mx * mx + my * my;
        let valid = !(alpha > half && r2 >= one / (two * alpha - one));
        let xi2 = xi * xi;
        let n = (one + (one - xi2) * r2).sqrt();
        let m = one + r2;
        let k = (xi + n) / m;
        point.fill(S::zero());
        point[0] = k * mx;
        point[1] = k * my;
        point[2] = k - xi;
        if pixel_jacobian.is_some() || param_jacobian.is_some() {
            let dk_dmx = -two * mx * (n + xi) / (m * m) + mx * (one - xi2) / (n * m);
            let dk_dmy = -two * my * (n + xi) / (m * m) + my * (one - xi2) / (n * m);
            let mut c0 = SVector::<S, D>::zeros();
            let mut c1 = SVector::<S, D>::zeros();
            c0[0] = (dk_dmx * mx + k) / fx;
            c0[1] = dk_dmx * my / fx;
            c0[2] = dk_dmx / fx;
            c1[0] = dk_dmy * mx / fy;
            c1[1] = (dk_dmy * my + k) / fy;
            c1[2] = dk_dmy / fy;
            c0 *= one - alpha;
            c1 *= one - alpha;
            if let Some(j) = pixel_jacobian {
                j.set_column(0, &c0);
                j.set_column(1, &c1);
            }
            if let Some(j) = param_jacobian {
                let d_xi_d_alpha = one / ((one - alpha) * (one - alpha));
                let d_m_d_alpha = -two * (one - alpha) * (mxx * mxx + myy * myy);
                let d_n_d_alpha = -(mxx * mxx + myy * myy) / n;
                let dk_d_alpha =
                    ((d_xi_d_alpha + d_n_d_alpha) * m - d_m_d_alpha * (xi + n)) / (m * m);
                j.fill(S::zero());
                j.set_column(0, &(c0 * (-mxx)));
                j.set_column(1, &(c1 * (-myy)));
                j.set_column(2, &(-c0));
                j.set_column(3, &(-c1));
                j[(0, 4)] = dk_d_alpha * mx - k * mxx;
                j[(1, 4)] = dk_d_alpha * my - k * myy;
                j[(2, 4)] = dk_d_alpha - d_xi_d_alpha;
            }
        }
        valid
    }
}
