// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/camera/kannala_brandt_camera4.hpp` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
// Preserve the upstream mathematical names and arithmetic order for comparison.
#![allow(clippy::many_single_char_names, clippy::similar_names)]
use crate::scalar::BasaltScalar;
use nalgebra::{SMatrix, SVector, Vector2, Vector4};

/// Kannala–Brandt model with `[fx,fy,cx,cy,k1,k2,k3,k4]` intrinsics.
#[derive(Clone, Debug)]
pub struct KannalaBrandtCamera4<S: BasaltScalar = f64> {
    param: SVector<S, 8>,
}
impl<S: BasaltScalar> Default for KannalaBrandtCamera4<S> {
    fn default() -> Self {
        Self {
            param: SVector::zeros(),
        }
    }
}
impl<S: BasaltScalar> KannalaBrandtCamera4<S> {
    pub const NAME: &'static str = "kb4";
    pub const N: usize = 8;
    #[must_use]
    pub fn new(param: SVector<S, 8>) -> Self {
        Self { param }
    }
    #[must_use]
    pub fn param(&self) -> &SVector<S, 8> {
        &self.param
    }
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> KannalaBrandtCamera4<T> {
        KannalaBrandtCamera4::new(self.param.map(|v| T::from_f64_lossy(v.as_f64())))
    }
    pub fn set_from_init(&mut self, init: Vector4<S>) {
        self.param.fill(S::zero());
        self.param.fixed_rows_mut::<4>(0).copy_from(&init);
    }
    pub fn increment(&mut self, inc: &SVector<S, 8>) {
        self.param += inc;
    }
    #[must_use]
    pub fn test_projections() -> [Self; 1] {
        [Self::new(SVector::from(
            [
                379.045,
                379.008,
                505.512,
                509.969,
                0.006_930_23,
                -0.001_382_8,
                -0.000_272_596,
                -0.000_452_646,
            ]
            .map(S::from_f64_lossy),
        ))]
    }
    fn radial_polynomial(&self, theta: S) -> S {
        let theta2 = theta * theta;
        let mut result = self.param[7] * theta2;
        result += self.param[6];
        result *= theta2;
        result += self.param[5];
        result *= theta2;
        result += self.param[4];
        result *= theta2;
        result += S::one();
        result *= theta;
        result
    }
    fn radial_derivative(&self, theta2: S) -> S {
        let mut result = S::from_f64_lossy(9.) * self.param[7] * theta2;
        result += S::from_f64_lossy(7.) * self.param[6];
        result *= theta2;
        result += S::from_f64_lossy(5.) * self.param[5];
        result *= theta2;
        result += S::from_f64_lossy(3.) * self.param[4];
        result *= theta2;
        result += S::one();
        result
    }
    /// Exactly `ITER` Newton steps; derivative is from the final pre-update iterate.
    /// With zero iterations, the caller's derivative is left unchanged.
    #[must_use]
    pub fn solve_theta<const ITER: usize>(&self, r_theta: S, derivative: &mut S) -> S {
        let mut theta = r_theta;
        for _ in 0..ITER {
            let func = self.radial_polynomial(theta);
            *derivative = self.radial_derivative(theta * theta);
            theta += (r_theta - func) / *derivative;
        }
        theta
    }
    /// Project with the upstream optical-axis pinhole fallback.
    /// # Panics
    /// Panics unless the point dimension is three or four.
    pub fn project<const D: usize>(
        &self,
        point: &SVector<S, D>,
        proj: &mut Vector2<S>,
        point_jacobian: Option<&mut SMatrix<S, 2, D>>,
        param_jacobian: Option<&mut SMatrix<S, 2, 8>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let (fx, fy, cx, cy) = (self.param[0], self.param[1], self.param[2], self.param[3]);
        let (x, y, z) = (point[0], point[1], point[2]);
        let r2 = x * x + y * y;
        let r = r2.sqrt();
        if r > S::epsilon_sqrt() {
            let theta = r.atan2(z);
            let theta2 = theta * theta;
            let r_theta = self.radial_polynomial(theta);
            let mx = x * r_theta / r;
            let my = y * r_theta / r;
            proj[0] = fx * mx + cx;
            proj[1] = fy * my + cy;
            if let Some(j) = point_jacobian {
                let d_r_d_x = x / r;
                let d_r_d_y = y / r;
                let tmp = z * z + r2;
                let d_theta_d_x = d_r_d_x * z / tmp;
                let d_theta_d_y = d_r_d_y * z / tmp;
                let d_theta_d_z = -r / tmp;
                let derivative = self.radial_derivative(theta2);
                j.fill(S::zero());
                j[(0, 0)] = fx
                    * (r_theta * r + x * r * derivative * d_theta_d_x - x * x * r_theta / r)
                    / r2;
                j[(1, 0)] = fy * y * (derivative * d_theta_d_x * r - x * r_theta / r) / r2;
                j[(0, 1)] = fx * x * (derivative * d_theta_d_y * r - y * r_theta / r) / r2;
                j[(1, 1)] = fy
                    * (r_theta * r + y * r * derivative * d_theta_d_y - y * y * r_theta / r)
                    / r2;
                j[(0, 2)] = fx * x * derivative * d_theta_d_z / r;
                j[(1, 2)] = fy * y * derivative * d_theta_d_z / r;
            }
            if let Some(j) = param_jacobian {
                j.fill(S::zero());
                j[(0, 0)] = mx;
                j[(0, 2)] = S::one();
                j[(1, 1)] = my;
                j[(1, 3)] = S::one();
                j[(0, 4)] = fx * x * theta * theta2 / r;
                j[(1, 4)] = fy * y * theta * theta2 / r;
                for col in 5..8 {
                    for row in 0..2 {
                        j[(row, col)] = j[(row, col - 1)] * theta2;
                    }
                }
            }
            true
        } else {
            // The source comparison intentionally returns true for a NaN z.
            let valid = z.partial_cmp(&S::epsilon_sqrt()) != Some(std::cmp::Ordering::Less);
            proj[0] = fx * x / z + cx;
            proj[1] = fy * y / z + cy;
            if let Some(j) = point_jacobian {
                j.fill(S::zero());
                let z2 = z * z;
                j[(0, 0)] = fx / z;
                j[(0, 2)] = -fx * x / z2;
                j[(1, 1)] = fy / z;
                j[(1, 2)] = -fy * y / z2;
            }
            if let Some(j) = param_jacobian {
                j.fill(S::zero());
                j[(0, 0)] = x / z;
                j[(0, 2)] = S::one();
                j[(1, 1)] = y / z;
                j[(1, 3)] = S::one();
            }
            valid
        }
    }
    /// Unproject using exactly three Newton steps and the upstream axis fallback.
    /// Returns true even if the resulting arithmetic is nonfinite.
    /// # Panics
    /// Panics unless the point dimension is three or four.
    pub fn unproject<const D: usize>(
        &self,
        proj: &Vector2<S>,
        point: &mut SVector<S, D>,
        pixel_jacobian: Option<&mut SMatrix<S, D, 2>>,
        param_jacobian: Option<&mut SMatrix<S, D, 8>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let (fx, fy, cx, cy) = (self.param[0], self.param[1], self.param[2], self.param[3]);
        let mx = (proj[0] - cx) / fx;
        let my = (proj[1] - cy) / fy;
        let mut theta = S::zero();
        let mut sin_theta = S::zero();
        let mut cos_theta = S::one();
        let thetad = (mx * mx + my * my).sqrt();
        let mut scaling = S::one();
        let mut derivative = S::zero();
        if thetad > S::epsilon_sqrt() {
            theta = self.solve_theta::<3>(thetad, &mut derivative);
            sin_theta = theta.sin();
            cos_theta = theta.cos();
            scaling = sin_theta / thetad;
        }
        point.fill(S::zero());
        point[0] = mx * scaling;
        point[1] = my * scaling;
        point[2] = cos_theta;
        if pixel_jacobian.is_some() || param_jacobian.is_some() {
            let mut d_thetad_d_mx = S::zero();
            let mut d_thetad_d_my = S::zero();
            let mut d_scaling_d_thetad = S::zero();
            let mut d_cos_d_thetad = S::zero();
            let mut d_scaling_d_k1 = S::zero();
            let mut d_cos_d_k1 = S::zero();
            let mut theta2 = S::zero();
            if thetad > S::epsilon_sqrt() {
                d_thetad_d_mx = mx / thetad;
                d_thetad_d_my = my / thetad;
                theta2 = theta * theta;
                d_scaling_d_thetad =
                    (thetad * cos_theta / derivative - sin_theta) / (thetad * thetad);
                d_cos_d_thetad = sin_theta / derivative;
                d_scaling_d_k1 = -cos_theta * theta * theta2 / (derivative * thetad);
                d_cos_d_k1 = d_cos_d_thetad * theta * theta2;
            }
            let mut c0 = SVector::<S, D>::zeros();
            let mut c1 = SVector::<S, D>::zeros();
            c0[0] = (scaling + mx * d_scaling_d_thetad * d_thetad_d_mx) / fx;
            c0[1] = my * d_scaling_d_thetad * d_thetad_d_mx / fx;
            c0[2] = -d_cos_d_thetad * d_thetad_d_mx / fx;
            c1[0] = mx * d_scaling_d_thetad * d_thetad_d_my / fy;
            c1[1] = (scaling + my * d_scaling_d_thetad * d_thetad_d_my) / fy;
            c1[2] = -d_cos_d_thetad * d_thetad_d_my / fy;
            if let Some(j) = pixel_jacobian {
                j.set_column(0, &c0);
                j.set_column(1, &c1);
            }
            if let Some(j) = param_jacobian {
                j.fill(S::zero());
                j.set_column(2, &(-c0));
                j.set_column(3, &(-c1));
                j.set_column(0, &(-c0 * mx));
                j.set_column(1, &(-c1 * my));
                j[(0, 4)] = mx * d_scaling_d_k1;
                j[(1, 4)] = my * d_scaling_d_k1;
                j[(2, 4)] = d_cos_d_k1;
                for col in 5..8 {
                    for row in 0..D {
                        j[(row, col)] = j[(row, col - 1)] * theta2;
                    }
                }
            }
        }
        true
    }
}
