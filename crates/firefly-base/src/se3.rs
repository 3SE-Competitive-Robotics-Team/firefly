//! SO(3)/SE(3) 共用李群运算。
//!
//! Hamilton 主动旋转，扭量按 `[rotation, translation]` 排列。
//! 指数映射与 Sophus / `OpenVINS` 的数学定义一致，小角使用 Taylor 展开；
//! JPL 四元数表示及其乘法由 `firefly-vio-types::quat_ops` 处理。

use nalgebra::{Matrix3, Matrix4, Quaternion, Rotation3, UnitQuaternion, Vector3, Vector6};

/// 反对称矩阵（对照 `skew`）。
///
/// `skew(x) · y = x × y`。
#[must_use]
pub fn skew(x: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -x[2], x[1], //
        x[2], 0.0, -x[0], //
        -x[1], x[0], 0.0, //
    )
}

/// SO(3) expmap（对照 `so3_exp`）。
///
/// 输入 `omega = [rx, ry, rz]`（旋转向量，模长即转角），输出单位四元数。
/// 小角用 Taylor 展开避免除零，大角用 `sin(θ/2)/θ` 形式。
#[must_use]
pub fn so3_exp(omega: &Vector3<f64>) -> UnitQuaternion<f64> {
    let theta_sq = omega.dot(omega);

    let (imag_factor, real_factor) = if theta_sq < 1e-10 {
        let theta_quad = theta_sq * theta_sq;
        let imag_factor = 0.5 - theta_sq / 48.0 + theta_quad / 3840.0;
        let real_factor = 1.0 - theta_sq / 8.0 + theta_quad / 384.0;
        (imag_factor, real_factor)
    } else {
        let theta = theta_sq.sqrt();
        let half = 0.5 * theta;
        let imag_factor = half.sin() / theta;
        let real_factor = half.cos();
        (imag_factor, real_factor)
    };

    let q = Quaternion::new(
        real_factor,
        imag_factor * omega.x,
        imag_factor * omega.y,
        imag_factor * omega.z,
    );
    UnitQuaternion::new_unchecked(q)
}

/// SE(3) expmap，旋转优先（对照 `se3_exp`）。
///
/// 输入扭曲向量 `a = [rx, ry, rz, tx, ty, tz]`，输出 4×4 齐次变换矩阵。
#[must_use]
pub fn se3_exp(a: &Vector6<f64>) -> Matrix4<f64> {
    let omega = a.fixed_rows::<3>(0).into_owned();
    let v = a.fixed_rows::<3>(3).into_owned();
    let mut se3 = Matrix4::identity();
    se3.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&so3_exp_matrix(&omega));
    se3.fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&(jl_so3(&omega) * v));
    se3
}

/// SO(3) logmap（对照 Sophus `log`）。
///
/// 输出主值 `omega`，模长落在 `[0, π]`。零旋转返回零向量；π 处的轴符号不唯一。
#[must_use]
pub fn so3_log(q: &UnitQuaternion<f64>) -> Vector3<f64> {
    let mut v = q.vector().into_owned();
    let mut w = q.scalar();
    // 取反四元数（同一旋转）使夹角落在 [0, π]
    if w < 0.0 {
        v = -v;
        w = -w;
    }
    let norm = v.norm();
    if norm < 1e-10 {
        return v * 2.0;
    }
    let angle = 2.0 * norm.atan2(w);
    v * (angle / norm)
}

/// SO(3) 矩阵指数；与四元数形式共享同一个指数映射。
#[must_use]
pub fn so3_exp_matrix(omega: &Vector3<f64>) -> Matrix3<f64> {
    so3_exp(omega).to_rotation_matrix().into_inner()
}

/// SO(3) 矩阵对数；输入必须是有效旋转矩阵，输出转角主值不超过 π。
#[must_use]
pub fn log_so3(rotation: &Matrix3<f64>) -> Vector3<f64> {
    so3_log(&UnitQuaternion::from_rotation_matrix(
        &Rotation3::from_matrix_unchecked(*rotation),
    ))
}

/// SO(3) 左雅可比（Barfoot Eq.7.77b），也是 SE(3) 指数映射的平移矩阵。
#[must_use]
pub fn jl_so3(omega: &Vector3<f64>) -> Matrix3<f64> {
    let theta_sq = omega.norm_squared();
    let (b, c) = if theta_sq < 1e-8 {
        (
            0.5 - theta_sq / 24.0 + theta_sq * theta_sq / 720.0,
            1.0 / 6.0 - theta_sq / 120.0 + theta_sq * theta_sq / 5040.0,
        )
    } else {
        let theta = theta_sq.sqrt();
        (
            (1.0 - theta.cos()) / theta_sq,
            (theta - theta.sin()) / (theta_sq * theta),
        )
    };
    let w = skew(omega);
    Matrix3::identity() + b * w + c * w * w
}

/// SO(3) 右雅可比 `Jr(ω) = Jl(−ω)`。
#[must_use]
pub fn jr_so3(omega: &Vector3<f64>) -> Matrix3<f64> {
    jl_so3(&-omega)
}

/// SE(3) 对数映射，输出 `[rotation, translation]`，旋转取主值。
/// 输入必须是有效 SE(3) 齐次变换。
#[must_use]
pub fn se3_log(t: &Matrix4<f64>) -> Vector6<f64> {
    let omega = log_so3(&t.fixed_view::<3, 3>(0, 0).into_owned());
    let translation = t.fixed_view::<3, 1>(0, 3).into_owned();
    let theta_sq = omega.norm_squared();
    let coefficient = if theta_sq < 1e-8 {
        1.0 / 12.0 + theta_sq / 720.0 + theta_sq * theta_sq / 30240.0
    } else {
        let half_theta = 0.5 * theta_sq.sqrt();
        (1.0 - half_theta / half_theta.tan()) / theta_sq
    };
    let w = skew(&omega);
    let v = (Matrix3::identity() - 0.5 * w + coefficient * w * w) * translation;
    let mut a = Vector6::zeros();
    a.fixed_rows_mut::<3>(0).copy_from(&omega);
    a.fixed_rows_mut::<3>(3).copy_from(&v);
    a
}

/// SE(3) 右雅可比；`Exp(x+dx) ≈ Exp(x) Exp(Jr(x) dx)`。
/// 用于右误差注入后的协方差重置，扭量顺序 `[rotation, translation]`。
#[must_use]
pub fn right_jacobian(x: &Vector6<f64>) -> nalgebra::Matrix6<f64> {
    let w = skew(&x.fixed_rows::<3>(0).into_owned());
    let v = skew(&x.fixed_rows::<3>(3).into_owned());
    let mut ad = nalgebra::Matrix6::zeros();
    ad.fixed_view_mut::<3, 3>(0, 0).copy_from(&w);
    ad.fixed_view_mut::<3, 3>(3, 3).copy_from(&w);
    ad.fixed_view_mut::<3, 3>(3, 0).copy_from(&v);
    let mut term = nalgebra::Matrix6::identity();
    let mut sum = term;
    for n in 1..=48 {
        term = term * (-ad) / f64::from(n + 1);
        sum += term;
        if term.norm() < 1e-15 {
            break;
        }
    }
    sum
}

/// 反对称矩阵 → 向量部分。
#[must_use]
pub fn vee(w_x: &Matrix3<f64>) -> Vector3<f64> {
    Vector3::new(w_x[(2, 1)], w_x[(0, 2)], w_x[(1, 0)])
}

/// `R6 -> se(3)` hat 算子。
#[must_use]
pub fn hat_se3(vec: &Vector6<f64>) -> Matrix4<f64> {
    let w = Vector3::new(vec[0], vec[1], vec[2]);
    let u = Vector3::new(vec[3], vec[4], vec[5]);
    let mut mat = Matrix4::<f64>::zeros();
    mat.fixed_view_mut::<3, 3>(0, 0).copy_from(&skew(&w));
    mat.fixed_view_mut::<3, 1>(0, 3).copy_from(&u);
    mat
}

/// `SE(3)` 解析逆（避免数值逆）。
#[must_use]
pub fn inv_se3(t: &Matrix4<f64>) -> Matrix4<f64> {
    let mut tinv = Matrix4::<f64>::identity();
    let r = t.fixed_view::<3, 3>(0, 0).into_owned();
    let p = Vector3::new(t[(0, 3)], t[(1, 3)], t[(2, 3)]);
    let r_t = r.transpose();
    tinv.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_t);
    tinv.fixed_view_mut::<3, 1>(0, 3).copy_from(&(-r_t * p));
    tinv
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close_scalar(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() <= eps
    }

    fn close_vec3(a: &Vector3<f64>, b: &Vector3<f64>, eps: f64) -> bool {
        (a - b).norm() <= eps
    }

    fn close_mat3(a: &Matrix3<f64>, b: &Matrix3<f64>, eps: f64) -> bool {
        (a - b).norm() <= eps
    }

    #[test]
    fn so3_exp_identity() {
        let q = so3_exp(&Vector3::zeros());
        assert!(close_scalar(q.scalar(), 1.0, 1e-12));
        assert!(close_vec3(
            &q.vector().into_owned(),
            &Vector3::zeros(),
            1e-12
        ));
    }

    #[test]
    fn so3_exp_90deg_z() {
        let q = so3_exp(&Vector3::new(0.0, 0.0, std::f64::consts::FRAC_PI_2));
        assert!(close_scalar(
            q.scalar(),
            (std::f64::consts::FRAC_PI_4).cos(),
            1e-12
        ));
        assert!(close_vec3(
            &q.vector().into_owned(),
            &Vector3::new(0.0, 0.0, (std::f64::consts::FRAC_PI_4).sin()),
            1e-12
        ));
    }

    #[test]
    fn so3_exp_log_roundtrip() {
        let mut rng = 0xABCD_u64;
        for _ in 0..50 {
            let omega = rand_vec(&mut rng, 2.5); // 模长 < π，主值可还原
            let q = so3_exp(&omega);
            let rec = so3_log(&q);
            assert!(
                close_vec3(&rec, &omega, 1e-9),
                "omega={omega:?} rec={rec:?}"
            );
        }
    }

    #[test]
    fn se3_exp_pure_translation() {
        let a = Vector6::new(0.0, 0.0, 0.0, 1.0, 2.0, 3.0);
        let t = se3_exp(&a);
        let rot = t.fixed_view::<3, 3>(0, 0).into_owned();
        let trans = t.fixed_view::<3, 1>(0, 3).into_owned();
        assert!(close_mat3(&rot, &Matrix3::identity(), 1e-12));
        assert!(close_vec3(&trans, &Vector3::new(1.0, 2.0, 3.0), 1e-12));
    }

    #[test]
    fn se3_exp_rotation_only() {
        let a = Vector6::new(0.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0);
        let t = se3_exp(&a);
        let expected = so3_exp(&Vector3::new(0.0, 0.0, std::f64::consts::FRAC_PI_2))
            .to_rotation_matrix()
            .matrix()
            .into_owned();
        let rot = t.fixed_view::<3, 3>(0, 0).into_owned();
        let trans = t.fixed_view::<3, 1>(0, 3).into_owned();
        assert!(close_mat3(&rot, &expected, 1e-12));
        assert!(close_vec3(&trans, &Vector3::zeros(), 1e-12));
    }

    #[test]
    fn se3_exp_log_roundtrip() {
        let mut rng = 0xDCBA_u64;
        for _ in 0..50 {
            let mut a = rand_vec6(&mut rng);
            a.fixed_rows_mut::<3>(0).copy_from(&rand_vec(&mut rng, 2.5));
            let t = se3_exp(&a);
            let rec = se3_log(&t);
            let mut diff = 0.0f64;
            for k in 0..6 {
                diff = diff.max((rec[k] - a[k]).abs());
            }
            assert!(diff <= 1e-8, "a={a:?} rec={rec:?} diff={diff}");
        }
    }

    #[test]
    fn analytic_rotation_and_translation_at_small_angles_and_pi() {
        for angle in [
            0.0,
            1e-12,
            1e-8,
            1e-4,
            0.7,
            std::f64::consts::PI - 1e-9,
            std::f64::consts::PI,
        ] {
            let (sin, cos) = angle.sin_cos();
            let expected = Matrix3::new(cos, -sin, 0.0, sin, cos, 0.0, 0.0, 0.0, 1.0);
            let omega = Vector3::z() * angle;
            assert!((so3_exp_matrix(&omega) - expected).norm() < 1e-14);
            assert!((log_so3(&expected) - omega).norm() < 1e-14);
            assert!(
                (so3_log(&UnitQuaternion::new_unchecked(
                    -so3_exp(&omega).into_inner()
                )) - omega)
                    .norm()
                    < 1e-14
            );

            // 绕 z 的单位 x 平移扭量：积分 [cos(tθ), sin(tθ), 0], t∈[0,1]。
            let (x, y) = if angle == 0.0 {
                (1.0, 0.0)
            } else {
                (sin / angle, 2.0 * (angle / 2.0).sin().powi(2) / angle)
            };
            let mut transform = Matrix4::identity();
            transform.fixed_view_mut::<3, 3>(0, 0).copy_from(&expected);
            transform
                .fixed_view_mut::<3, 1>(0, 3)
                .copy_from(&Vector3::new(x, y, 0.0));
            let tangent = Vector6::new(0.0, 0.0, angle, 1.0, 0.0, 0.0);
            assert!((se3_exp(&tangent) - transform).norm() < 1e-12);
            assert!((se3_log(&transform) - tangent).norm() < 1e-12);
        }
        for axis in [
            Vector3::x(),
            Vector3::y(),
            Vector3::new(1.0, -2.0, 3.0).normalize(),
        ] {
            // R(π) = 2aaᵀ - I，π 处只比较旋转，轴的正负均有效。
            let expected = 2.0 * axis * axis.transpose() - Matrix3::identity();
            let recovered = log_so3(&expected);
            assert!((recovered.norm() - std::f64::consts::PI).abs() < 1e-14);
            assert!((so3_exp_matrix(&recovered) - expected).norm() < 1e-14);
        }
    }

    #[test]
    fn so3_jacobians_match_small_angle_terms_and_central_differences() {
        let tiny = Vector3::new(1e-8, -2e-8, 3e-8);
        let expected = Matrix3::identity() + skew(&tiny) * 0.5 + skew(&tiny) * skew(&tiny) / 6.0;
        assert!((jl_so3(&tiny) - expected).norm() < 1e-16);
        for omega in [Vector3::zeros(), tiny, Vector3::new(0.4, -0.7, 1.2)] {
            let rotation = so3_exp_matrix(&omega);
            let eps = 1e-6;
            for column in 0..3 {
                let mut delta = Vector3::zeros();
                delta[column] = eps;
                let derivative = (so3_exp_matrix(&(omega + delta))
                    - so3_exp_matrix(&(omega - delta)))
                    / (2.0 * eps);
                let left = vee(&(derivative * rotation.transpose()));
                let right = vee(&(rotation.transpose() * derivative));
                assert!((left - jl_so3(&omega).column(column)).norm() < 1e-9);
                assert!((right - jr_so3(&omega).column(column)).norm() < 1e-9);
            }
        }
    }

    fn rand_vec(rng: &mut u64, scale: f64) -> Vector3<f64> {
        let u = splitmix64(rng);
        let v = splitmix64(rng);
        let w = splitmix64(rng);
        let to = |x: u64| -> f64 { (x as f64 / u64::MAX as f64) * 2.0 - 1.0 };
        let v3 = Vector3::new(to(u), to(v), to(w));
        v3.normalize() * (scale * (splitmix64(rng) as f64 / u64::MAX as f64))
    }

    fn rand_vec6(rng: &mut u64) -> Vector6<f64> {
        let mut a = Vector6::zeros();
        for k in 0..6 {
            a[k] = (splitmix64(rng) as f64 / u64::MAX as f64) * 2.0 - 1.0;
        }
        a
    }

    fn splitmix64(state: &mut u64) -> u64 {
        *state = state.wrapping_add(0x9E37_9B97_F4A7_C15B);
        let mut z = *state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
}
