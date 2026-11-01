// Copyright (c) 2021-2022 Collabora Ltd.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/camera/pinhole_radtan8_camera.hpp` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
// Preserve upstream symbolic Jacobian expressions and arithmetic order.
#![allow(clippy::many_single_char_names, clippy::similar_names)]
use crate::scalar::BasaltScalar;
use nalgebra::{Matrix2, SMatrix, SVector, Vector2, Vector3, Vector4};

/// Rational radial-tangential camera with eight distortion coefficients.
#[derive(Clone, Debug)]
pub struct PinholeRadtan8Camera<S: BasaltScalar = f64> {
    param: SVector<S, 12>,
    rpmax: S,
}
impl<S: BasaltScalar> Default for PinholeRadtan8Camera<S> {
    fn default() -> Self {
        Self {
            param: SVector::zeros(),
            rpmax: S::zero(),
        }
    }
}
impl<S: BasaltScalar> PinholeRadtan8Camera<S> {
    pub const NAME: &'static str = "pinhole-radtan8";
    pub const N: usize = 12;
    /// Estimates the injective radius using the upstream gradient ascent.
    /// # Panics
    /// Panics if the near-center starting pixel cannot be unprojected.
    #[must_use]
    pub fn new(param: SVector<S, 12>) -> Self {
        Self::with_rpmax(param, -S::one())
    }
    /// A radius of -1 estimates the domain; zero disables radius rejection.
    /// # Panics
    /// Panics if radius estimation's near-center unprojection fails.
    #[must_use]
    pub fn with_rpmax(param: SVector<S, 12>, rpmax: S) -> Self {
        let mut result = Self {
            param,
            rpmax: S::zero(),
        };
        result.rpmax = if rpmax == -S::one() {
            result.compute_rpmax()
        } else {
            rpmax
        };
        result
    }
    #[must_use]
    pub fn param(&self) -> &SVector<S, 12> {
        &self.param
    }
    #[must_use]
    pub const fn rpmax(&self) -> S {
        self.rpmax
    }
    /// Recomputes the radius after casting, matching the upstream constructor.
    /// # Panics
    /// Panics if radius estimation's near-center unprojection fails.
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> PinholeRadtan8Camera<T> {
        PinholeRadtan8Camera::new(self.param.map(|v| T::from_f64_lossy(v.as_f64())))
    }
    pub fn set_from_init(&mut self, init: Vector4<S>) {
        self.param.fill(S::zero());
        self.param.fixed_rows_mut::<4>(0).copy_from(&init);
        self.rpmax = S::zero();
    }
    /// # Panics
    /// Panics if radius estimation's near-center unprojection fails.
    pub fn increment(&mut self, inc: &SVector<S, 12>) {
        self.param += inc;
        self.rpmax = self.compute_rpmax();
    }
    #[must_use]
    pub fn test_projections() -> [Self; 1] {
        [Self::new(SVector::from(
            [
                269.060_077_667_236_3,
                269.167_985_916_137_7,
                324.333_305_358_886_7,
                245.226_745_605_468_75,
                0.625_731_945_037_841_8,
                0.466_120_362_281_799_3,
                -0.000_185_023_993_253_707_89,
                -4.288_297_350_285_575e-5,
                0.004_179_582_931_101_322,
                0.894_319_355_487_823_5,
                0.542_539_775_371_551_5,
                0.066_212_147_474_288_9,
            ]
            .map(S::from_f64_lossy),
        ))]
    }
    #[must_use]
    pub const fn test_resolutions() -> [[usize; 2]; 1] {
        [[640, 480]]
    }
    /// Projects a point and optionally evaluates the upstream analytic Jacobians.
    /// # Panics
    /// Panics unless the point dimension is three or four.
    #[expect(
        clippy::too_many_lines,
        reason = "Preserve upstream symbolic Jacobians in their source function"
    )]
    pub fn project<const D: usize>(
        &self,
        point: &SVector<S, D>,
        proj: &mut Vector2<S>,
        point_jacobian: Option<&mut SMatrix<S, 2, D>>,
        param_jacobian: Option<&mut SMatrix<S, 2, 12>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let [fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6] =
            self.param.as_slice().try_into().unwrap();
        let zero = S::zero();
        let one = S::one();
        let two = one + one;
        let three = two + one;
        let (x, y, z) = (point[0], point[1], point[2]);
        let xp = x / z;
        let yp = y / z;
        let rp2 = xp * xp + yp * yp;
        let cdist =
            (one + rp2 * (k1 + rp2 * (k2 + rp2 * k3))) / (one + rp2 * (k4 + rp2 * (k5 + rp2 * k6)));
        let delta_x = two * p1 * xp * yp + p2 * (rp2 + two * xp * xp);
        let delta_y = two * p2 * xp * yp + p1 * (rp2 + two * yp * yp);
        let xpp = xp * cdist + delta_x;
        let ypp = yp * cdist + delta_y;
        *proj = Vector2::new(fx * xpp + cx, fy * ypp + cy);
        let valid =
            z >= S::epsilon_sqrt() && (self.rpmax == zero || rp2 <= self.rpmax * self.rpmax);
        if let Some(j) = point_jacobian {
            j.fill(zero);
            let v0 = p1 * y;
            let v1 = p2 * x;
            let v2 = z * z * z * z * z * z;
            let v3 = x * x;
            let v4 = y * y;
            let v5 = v3 + v4;
            let v6 = z * z * z * z;
            let v7 = z * z;
            let v8 = k5 * v7;
            let v9 = k6 * v5;
            let v10 = k4 * v6 + v5 * (v8 + v9);
            let v11 = v10 * v5 + v2;
            let v12 = v11 * v11;
            let v13 = two * v12;
            let v14 = k2 * v7;
            let v15 = k3 * v5;
            let v16 = k1 * v6 + v5 * (v14 + v15);
            let v17 = v16 * v5 + v2;
            let v18 = v17 * z * (v10 + v5 * (v8 + two * v9));
            let v19 = two * v18;
            let v20 = v16 + v5 * (v14 + two * v15);
            let v21 = two * v20;
            let v22 = v11 * z;
            let v23 = one / v7;
            let v24 = one / v12;
            let v25 = fx * v24;
            let v26 = v23 * v25;
            let v27 = p2 * y;
            let v28 = x * y;
            let v29 = two * v12 * (p1 * x + v27) - two * v18 * v28 + two * v20 * v22 * v28;
            let v30 = one / (z * z * z);
            let v31 = two * x;
            let v32 = v22 * (v17 + v21 * v5);
            let v33 = fy * v24;
            let v34 = v23 * v33;

            let du_dx = v26 * (v13 * (v0 + three * v1) - v19 * v3 + v22 * (v17 + v21 * v3));
            let du_dy = v26 * v29;
            let du_dz =
                -v25 * v30 * (v13 * (p2 * (three * v3 + v4) + v0 * v31) - v18 * v31 * v5 + v32 * x);
            let dv_dx = v29 * v34;
            let dv_dy = v34 * (v13 * (three * v0 + v1) - v19 * v4 + v22 * (v17 + v21 * v4));
            let dv_dz =
                -v30 * v33 * (v13 * (p1 * (v3 + three * v4) + v27 * v31) - v19 * v5 * y + v32 * y);

            j[(0, 0)] = du_dx;
            j[(0, 1)] = du_dy;
            j[(0, 2)] = du_dz;
            j[(1, 0)] = dv_dx;
            j[(1, 1)] = dv_dy;
            j[(1, 2)] = dv_dz;
        }
        if let Some(j) = param_jacobian {
            j.fill(zero);
            let w0 = z * z * z * z * z * z;
            let w1 = x * x;
            let w2 = y * y;
            let w3 = w1 + w2;
            let w4 = z * z * z * z;
            let w5 = z * z;
            let w6 = w0 + w3 * (k1 * w4 + w3 * (k2 * w5 + k3 * w3));
            let w7 = w6 * z;
            let w8 = w7 * x;
            let w9 = two * x * y;
            let w10 = three * w1 + w2;
            let w11 = w0 + w3 * (k4 * w4 + w3 * (k5 * w5 + k6 * w3));
            let w12 = one / w5;
            let w13 = one / w11;
            let w14 = w12 * w13;
            let w15 = w3 * (z * z * z);
            let w16 = fx * x;
            let w17 = w13 * w16;
            let w18 = w3 * w3;
            let w19 = w18 * z;
            let w20 = fx * w12;
            let w21 = (w3 * w3 * w3) * one / z;
            let w22 = one / (w11 * w11);
            let w23 = w22 * w6;
            let w24 = w16 * w23;
            let w25 = w18 * w22;
            let w26 = w7 * y;
            let w27 = w1 + three * w2;
            let w28 = fy * y;
            let w29 = w13 * w28;
            let w30 = fy * w12;
            let w31 = w23 * w28;

            let du_fx = w14 * (w11 * (p1 * w9 + p2 * w10) + w8);
            let du_fy = zero;
            let du_cx = one;
            let du_cy = zero;
            let du_k1 = w15 * w17;
            let du_k2 = w17 * w19;
            let du_p1 = w20 * w9;
            let du_p2 = w10 * w20;
            let du_k3 = w17 * w21;
            let du_k4 = -w15 * w24;
            let du_k5 = -fx * w25 * w8;
            let du_k6 = -w21 * w24;
            let dv_fx = zero;
            let dv_fy = w14 * (w11 * (p1 * w27 + p2 * w9) + w26);
            let dv_cx = zero;
            let dv_cy = one;
            let dv_k1 = w15 * w29;
            let dv_k2 = w19 * w29;
            let dv_p1 = w27 * w30;
            let dv_p2 = w30 * w9;
            let dv_k3 = w21 * w29;
            let dv_k4 = -w15 * w31;
            let dv_k5 = -fy * w25 * w26;
            let dv_k6 = -w21 * w31;

            j[(0, 0)] = du_fx;
            j[(0, 1)] = du_fy;
            j[(0, 2)] = du_cx;
            j[(0, 3)] = du_cy;
            j[(0, 4)] = du_k1;
            j[(0, 5)] = du_k2;
            j[(0, 6)] = du_p1;
            j[(0, 7)] = du_p2;
            j[(0, 8)] = du_k3;
            j[(0, 9)] = du_k4;
            j[(0, 10)] = du_k5;
            j[(0, 11)] = du_k6;
            j[(1, 0)] = dv_fx;
            j[(1, 1)] = dv_fy;
            j[(1, 2)] = dv_cx;
            j[(1, 3)] = dv_cy;
            j[(1, 4)] = dv_k1;
            j[(1, 5)] = dv_k2;
            j[(1, 6)] = dv_p1;
            j[(1, 7)] = dv_p2;
            j[(1, 8)] = dv_k3;
            j[(1, 9)] = dv_k4;
            j[(1, 10)] = dv_k5;
            j[(1, 11)] = dv_k6;
        }
        valid
    }
    /// Distorts a normalized point and optionally differentiates the distortion.
    pub fn distort(
        &self,
        undist: &Vector2<S>,
        dist: &mut Vector2<S>,
        jacobian: Option<&mut Matrix2<S>>,
    ) {
        let [_, _, _, _, k1, k2, p1, p2, k3, k4, k5, k6]: [S; 12] =
            std::array::from_fn(|i| self.param[i]);
        let one = S::one();
        let two = one + one;
        let three = two + one;
        let xp = undist.x;
        let yp = undist.y;
        let rp2 = xp * xp + yp * yp;
        let cdist =
            (one + rp2 * (k1 + rp2 * (k2 + rp2 * k3))) / (one + rp2 * (k4 + rp2 * (k5 + rp2 * k6)));
        let delta_x = two * p1 * xp * yp + p2 * (rp2 + two * xp * xp);
        let delta_y = two * p2 * xp * yp + p1 * (rp2 + two * yp * yp);
        dist.x = xp * cdist + delta_x;
        dist.y = yp * cdist + delta_y;
        if let Some(j) = jacobian {
            let v0 = xp * xp;
            let v1 = yp * yp;
            let v2 = v0 + v1;
            let v3 = k6 * v2;
            let v4 = k4 + v2 * (k5 + v3);
            let v5 = v2 * v4 + one;
            let v6 = v5 * v5;
            let v7 = one / v6;
            let v8 = p1 * yp;
            let v9 = p2 * xp;
            let v10 = two * v6;
            let v11 = k3 * v2;
            let v12 = k1 + v2 * (k2 + v11);
            let v13 = v12 * v2 + one;
            let v14 = v13 * (v2 * (k5 + two * v3) + v4);
            let v15 = two * v14;
            let v16 = v12 + v2 * (k2 + two * v11);
            let v17 = two * v16;
            let v18 = xp * yp;
            let v19 = two * v7 * (-v14 * v18 + v16 * v18 * v5 + v6 * (p1 * xp + p2 * yp));

            let dxpp_dxp = v7 * (-v0 * v15 + v10 * (v8 + three * v9) + v5 * (v0 * v17 + v13));
            let dxpp_dyp = v19;
            let dypp_dxp = v19;
            let dypp_dyp = v7 * (-v1 * v15 + v10 * (three * v8 + v9) + v5 * (v1 * v17 + v13));

            j[(0, 0)] = dxpp_dxp;
            j[(0, 1)] = dxpp_dyp;
            j[(1, 0)] = dypp_dxp;
            j[(1, 1)] = dypp_dyp;
        }
    }
    /// Newton unprojection with five updates and the upstream radius validity test.
    /// Unprojection Jacobians are unsupported in the upstream model.
    /// # Panics
    /// Panics for dimensions other than three/four or if a Jacobian is requested.
    pub fn unproject<const D: usize>(
        &self,
        proj: &Vector2<S>,
        point: &mut SVector<S, D>,
        pixel_jacobian: Option<&mut SMatrix<S, D, 2>>,
        param_jacobian: Option<&mut SMatrix<S, D, 12>>,
    ) -> bool {
        assert!(D == 3 || D == 4);
        let dist = Vector2::new(
            (proj[0] - self.param[2]) / self.param[0],
            (proj[1] - self.param[3]) / self.param[1],
        );
        let mut undist = dist;
        for _ in 0..5 {
            let mut j = Matrix2::zeros();
            let mut fundist = Vector2::zeros();
            self.distort(&undist, &mut fundist, Some(&mut j));
            let residual = fundist - dist;
            let invdet = S::one() / (j[(0, 0)] * j[(1, 1)] - j[(0, 1)] * j[(1, 0)]);
            let inverse = Matrix2::new(
                j[(1, 1)] * invdet,
                -j[(0, 1)] * invdet,
                -j[(1, 0)] * invdet,
                j[(0, 0)] * invdet,
            );
            undist -= inverse * residual;
            if residual.norm() < S::epsilon_sqrt() {
                break;
            }
        }
        let (xp, yp) = (undist.x, undist.y);
        let norm_inv = S::one() / (xp * xp + yp * yp + S::one()).sqrt();
        point.fill(S::zero());
        point[0] = xp * norm_inv;
        point[1] = yp * norm_inv;
        point[2] = norm_inv;
        assert!(
            matches!((pixel_jacobian, param_jacobian), (None, None)),
            "upstream radtan8 unprojection Jacobians are unsupported"
        );
        let rp2 = xp * xp + yp * yp;
        self.rpmax == S::zero() || rp2 <= self.rpmax * self.rpmax
    }
    /// Estimates the injective domain radius without changing the stored radius.
    /// # Panics
    /// Panics if the near-center starting pixel cannot be unprojected.
    pub fn compute_rpmax(&mut self) -> S {
        let backup = self.rpmax;
        self.rpmax = S::zero();
        let one = S::one();
        let two = one + one;
        let step = S::from_f64_lossy(0.1);
        let initial = Vector2::new(
            step * self.param[0] + self.param[2],
            step * self.param[1] + self.param[3],
        );
        let mut ray = Vector3::zeros();
        assert!(self.unproject(&initial, &mut ray, None, None));
        ray /= ray.z;
        let mut x = Vector2::new(ray.x, ray.y);
        let rpp2 = |xy: &Vector2<S>| {
            let mut result = Vector2::zeros();
            self.distort(xy, &mut result, None);
            result.norm_squared()
        };
        let derivative = |xy: &Vector2<S>| {
            let eps = S::epsilon_sqrt();
            let hx = eps
                * (if xy.x < S::zero() { -one } else { one })
                * (if one < xy.x { xy.x } else { one });
            let hy = eps
                * (if xy.y < S::zero() { -one } else { one })
                * (if one < xy.y { xy.y } else { one });
            let f = rpp2(xy);
            Vector2::new(
                (rpp2(&(xy + Vector2::new(hx, S::zero()))) - f) / hx,
                (rpp2(&(xy + Vector2::new(S::zero(), hy))) - f) / hy,
            )
        };
        let w = two * self.param[2];
        let h = two * self.param[3];
        let mut corners_maxrp2 = -one;
        for (u, v) in [
            (S::zero(), S::zero()),
            (w, S::zero()),
            (S::zero(), h),
            (w, h),
        ] {
            let mut corner = Vector3::zeros();
            self.unproject(&Vector2::new(u, v), &mut corner, None, None);
            corner /= corner.z;
            let radius = corner.x * corner.x + corner.y * corner.y;
            if radius > corners_maxrp2 {
                corners_maxrp2 = radius;
            }
        }
        let domain_bound = corners_maxrp2 * S::from_f64_lossy(1.5);
        let mut rpp2_x = rpp2(&x);
        let mut diverged = false;
        for _ in 1..1000 {
            x += derivative(&x) * step;
            if x.norm_squared() > domain_bound {
                diverged = true;
                break;
            }
            let old = rpp2_x;
            rpp2_x = rpp2(&x);
            if (rpp2_x - old).abs() < S::from_f64_lossy(0.0001) * old {
                break;
            }
        }
        let result = if diverged {
            S::zero()
        } else {
            S::from_f64_lossy(0.85) * x.norm()
        };
        self.rpmax = backup;
        result
    }
}
