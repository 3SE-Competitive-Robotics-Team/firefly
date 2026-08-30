//! SO(3)/SE(3) 李代数运算（对照 official `util/lie.hpp`）。
//!
//! `so3_exp` 取自 Sophus 的 SO(3) expmap（小角展开与 Eigen 四元数约定一致），
//! `se3_exp` 为「旋转优先」的 SE(3) expmap。数值定义与官方逐行对齐。

use nalgebra::{Matrix3, Matrix4, Quaternion, Rotation3, UnitQuaternion, Vector3, Vector6};

/// 反对称矩阵（对照 `skew`）。
///
/// `skew(x) · y = x × y`。仅用于 se3_exp 的 V 矩阵构造。
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
pub fn se3_exp(a: &Vector6<f64>) -> Matrix4<f64> {
    let omega = a.fixed_rows::<3>(0).into_owned();
    let theta_sq = omega.dot(&omega);
    let theta = theta_sq.sqrt();

    let mut se3 = Matrix4::identity();
    let r = so3_exp(&omega);
    se3.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(r.to_rotation_matrix().matrix());

    let v = a.fixed_rows::<3>(3).into_owned();
    if theta < 1e-10 {
        // 小角：V ≈ I，平移 = R · v（该展开在数值上精确）
        se3.fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&(r.to_rotation_matrix().matrix() * v));
    } else {
        let omega_skew = skew(&omega);
        let v_mat = Matrix3::identity()
            + ((1.0 - theta.cos()) / theta_sq) * omega_skew
            + ((theta - theta.sin()) / (theta_sq * theta)) * (omega_skew * omega_skew);
        se3.fixed_view_mut::<3, 1>(0, 3).copy_from(&(v_mat * v));
    }

    se3
}

/// SO(3) logmap（用于测试与逆向校验，对照 Sophus `log`）。
///
/// 输出主值 `omega`，模长落在 `[0, π]`。零旋转返回零向量。
pub fn so3_log(q: &UnitQuaternion<f64>) -> Vector3<f64> {
    let mut v = q.vector().into_owned();
    let mut w = q.scalar();
    // 取反四元数（同一旋转）使夹角落在 [0, π]，保证主值唯一
    if w < 0.0 {
        v = -v;
        w = -w;
    }
    let norm = v.norm();
    if norm < 1e-10 {
        return Vector3::zeros();
    }
    let angle = 2.0 * norm.atan2(w);
    v * (angle / norm)
}

/// SE(3) logmap（用于测试与逆向校验）。
///
/// 由齐次变换矩阵反解扭曲向量 `[omega; v]`，`omega` 取主值。
pub fn se3_log(t: &Matrix4<f64>) -> Vector6<f64> {
    let r = Rotation3::from_matrix(&t.fixed_view::<3, 3>(0, 0).into_owned().into_owned());
    let omega = so3_log(&UnitQuaternion::from_rotation_matrix(&r));
    let translation = t.fixed_view::<3, 1>(0, 3).into_owned();

    let theta = omega.norm();
    let v = if theta < 1e-10 {
        translation
    } else {
        let omega_skew = skew(&omega);
        let theta_sq = theta * theta;
        let v_mat = Matrix3::identity()
            + ((1.0 - theta.cos()) / theta_sq) * omega_skew
            + ((theta - theta.sin()) / (theta_sq * theta)) * (omega_skew * omega_skew);
        v_mat.try_inverse().expect("V 矩阵非奇异（theta ≠ 0）") * translation
    };

    let mut a = Vector6::zeros();
    a.fixed_rows_mut::<3>(0).copy_from(&omega);
    a.fixed_rows_mut::<3>(3).copy_from(&v);
    a
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
