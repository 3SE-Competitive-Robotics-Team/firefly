//! JPL 四元数运算（对照 `OpenVINS` `ov_core/quat_ops.h`）。
//! 通用李群运算由 `firefly-base` 提供；此处沿用 `OpenVINS` 的函数名。
//!
//! JPL 惯例：四元数 `[x, y, z, w]`（标量在最后），与 Hamilton 相反；
//! 旋转合成 `q ⊗ p` 采用 JPL 乘法（`quat_multiply`）。

use nalgebra::{Matrix3, Matrix4, Vector3, Vector4};

pub use firefly_base::se3::{
    hat_se3, inv_se3, jl_so3, jr_so3, log_so3, se3_exp as exp_se3, se3_log as log_se3,
    skew as skew_x, so3_exp_matrix as exp_so3, vee,
};

/// 旋转矩阵 → JPL 四元数（Trawny 论文 Eq.74，按最大对角元分支避免除零）。
#[must_use]
pub fn rot_2_quat(rot: &Matrix3<f64>) -> Vector4<f64> {
    let mut q = Vector4::<f64>::zeros();
    let t = rot.trace();
    if rot[(0, 0)] >= t && rot[(0, 0)] >= rot[(1, 1)] && rot[(0, 0)] >= rot[(2, 2)] {
        q[0] = ((1.0 + (2.0 * rot[(0, 0)]) - t) / 4.0).sqrt();
        q[1] = (1.0 / (4.0 * q[0])) * (rot[(0, 1)] + rot[(1, 0)]);
        q[2] = (1.0 / (4.0 * q[0])) * (rot[(0, 2)] + rot[(2, 0)]);
        q[3] = (1.0 / (4.0 * q[0])) * (rot[(1, 2)] - rot[(2, 1)]);
    } else if rot[(1, 1)] >= t && rot[(1, 1)] >= rot[(0, 0)] && rot[(1, 1)] >= rot[(2, 2)] {
        q[1] = ((1.0 + (2.0 * rot[(1, 1)]) - t) / 4.0).sqrt();
        q[0] = (1.0 / (4.0 * q[1])) * (rot[(0, 1)] + rot[(1, 0)]);
        q[2] = (1.0 / (4.0 * q[1])) * (rot[(1, 2)] + rot[(2, 1)]);
        q[3] = (1.0 / (4.0 * q[1])) * (rot[(2, 0)] - rot[(0, 2)]);
    } else if rot[(2, 2)] >= t && rot[(2, 2)] >= rot[(0, 0)] && rot[(2, 2)] >= rot[(1, 1)] {
        q[2] = ((1.0 + (2.0 * rot[(2, 2)]) - t) / 4.0).sqrt();
        q[0] = (1.0 / (4.0 * q[2])) * (rot[(0, 2)] + rot[(2, 0)]);
        q[1] = (1.0 / (4.0 * q[2])) * (rot[(1, 2)] + rot[(2, 1)]);
        q[3] = (1.0 / (4.0 * q[2])) * (rot[(0, 1)] - rot[(1, 0)]);
    } else {
        q[3] = ((1.0 + t) / 4.0).sqrt();
        q[0] = (1.0 / (4.0 * q[3])) * (rot[(1, 2)] - rot[(2, 1)]);
        q[1] = (1.0 / (4.0 * q[3])) * (rot[(2, 0)] - rot[(0, 2)]);
        q[2] = (1.0 / (4.0 * q[3])) * (rot[(0, 1)] - rot[(1, 0)]);
    }
    if q[3] < 0.0 {
        q = -q;
    }
    quatnorm(q)
}

/// JPL 四元数 → SO(3) 旋转矩阵（Trawny Eq.62：`R = (2q₄²−1)I − 2q₄⌊q×⌋ + 2qqᵀ`）。
#[must_use]
pub fn quat_2_rot(q: &Vector4<f64>) -> Matrix3<f64> {
    let q_vec = Vector3::new(q[0], q[1], q[2]);
    let q_x = skew_x(&q_vec);
    (2.0 * q[3] * q[3] - 1.0) * Matrix3::identity() - 2.0 * q[3] * q_x
        + 2.0 * q_vec * q_vec.transpose()
}

/// JPL 四元数乘法 `q ⊗ p`（Trawny Eq.9，`q₄ > 0` 保证唯一性）。
#[must_use]
pub fn quat_multiply(q: &Vector4<f64>, p: &Vector4<f64>) -> Vector4<f64> {
    let q_vec = Vector3::new(q[0], q[1], q[2]);
    let mut qm = Matrix4::<f64>::zeros();
    qm.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(q[3] * Matrix3::identity() - skew_x(&q_vec)));
    qm.fixed_view_mut::<3, 1>(0, 3).copy_from(&q_vec);
    qm.fixed_view_mut::<1, 3>(3, 0)
        .copy_from(&(-q_vec.transpose()));
    qm[(3, 3)] = q[3];
    let q_t = qm * p;
    quatnorm(q_t)
}

/// JPL 四元数逆 `q⁻¹ = [−q_vec; q₄]`。
#[must_use]
pub fn inv_quat(q: &Vector4<f64>) -> Vector4<f64> {
    Vector4::new(-q[0], -q[1], -q[2], q[3])
}

/// 角速度积分矩阵 `Ω(ω)`（Trawny Eq.48）。
#[must_use]
pub fn omega(w: &Vector3<f64>) -> Matrix4<f64> {
    let mut mat = Matrix4::<f64>::zeros();
    mat.fixed_view_mut::<3, 3>(0, 0).copy_from(&(-skew_x(w)));
    mat.fixed_view_mut::<1, 3>(3, 0)
        .copy_from(&(-w.transpose()));
    mat.fixed_view_mut::<3, 1>(0, 3).copy_from(w);
    mat
}

/// 四元数归一化（强制 `q₄ > 0` 保证唯一性）。
#[must_use]
pub fn quatnorm(q_t: Vector4<f64>) -> Vector4<f64> {
    let q = if q_t[3] < 0.0 { -q_t } else { q_t };
    q / q.norm()
}

/// 旋转矩阵 → roll/pitch/yaw（`R = Rz(yaw)·Ry(pitch)·Rx(roll)`）。
#[must_use]
pub fn rot2rpy(rot: &Matrix3<f64>) -> Vector3<f64> {
    let mut rpy = Vector3::zeros();
    rpy[1] = (-rot[(2, 0)]).atan2((rot[(0, 0)] * rot[(0, 0)] + rot[(1, 0)] * rot[(1, 0)]).sqrt());
    if rpy[1].cos().abs() > 1.0e-12 {
        rpy[2] = (rot[(1, 0)] / rpy[1].cos()).atan2(rot[(0, 0)] / rpy[1].cos());
        rpy[0] = (rot[(2, 1)] / rpy[1].cos()).atan2(rot[(2, 2)] / rpy[1].cos());
    } else {
        rpy[2] = 0.0;
        rpy[0] = rot[(0, 1)].atan2(rot[(1, 1)]);
    }
    rpy
}

/// roll 旋转矩阵。
#[must_use]
pub fn rot_x(t: f64) -> Matrix3<f64> {
    let (ct, st) = (t.cos(), t.sin());
    Matrix3::new(1.0, 0.0, 0.0, 0.0, ct, -st, 0.0, st, ct)
}

/// pitch 旋转矩阵。
#[must_use]
pub fn rot_y(t: f64) -> Matrix3<f64> {
    let (ct, st) = (t.cos(), t.sin());
    Matrix3::new(ct, 0.0, st, 0.0, 1.0, 0.0, -st, 0.0, ct)
}

/// yaw 旋转矩阵。
#[must_use]
pub fn rot_z(t: f64) -> Matrix3<f64> {
    let (ct, st) = (t.cos(), t.sin());
    Matrix3::new(ct, -st, 0.0, st, ct, 0.0, 0.0, 0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector6;

    fn assert_close(a: f64, b: f64, eps: f64) {
        assert!((a - b).abs() < eps, "{a} != {b}");
    }

    fn assert_mat_close(a: &Matrix3<f64>, b: &Matrix3<f64>, eps: f64) {
        for i in 0..3 {
            for j in 0..3 {
                assert_close(a[(i, j)], b[(i, j)], eps);
            }
        }
    }

    #[test]
    fn jpl_increment_has_opposite_matrix_rotation_to_hamilton() {
        let angle = 0.4_f64;
        let jpl = Vector4::new(0.0, 0.0, (angle / 2.0).sin(), (angle / 2.0).cos());
        let expected = Matrix3::new(
            angle.cos(),
            angle.sin(),
            0.0,
            -angle.sin(),
            angle.cos(),
            0.0,
            0.0,
            0.0,
            1.0,
        );
        assert_mat_close(&quat_2_rot(&jpl), &expected, 1e-14);
        assert_mat_close(&exp_so3(&Vector3::new(0.0, 0.0, -angle)), &expected, 1e-14);
        assert!((log_so3(&expected) - Vector3::new(0.0, 0.0, -angle)).norm() < 1e-14);
    }

    #[test]
    fn quat_rot_roundtrip() {
        // 绕 x/y/z 复合旋转：R → q → R 往返
        let r = rot_z(0.4) * rot_y(-0.3) * rot_x(0.2);
        let q = rot_2_quat(&r);
        assert_close(q.norm(), 1.0, 1e-12);
        assert_mat_close(&quat_2_rot(&q), &r, 1e-12);
    }

    #[test]
    fn quat_multiply_matches_rotation_composition() {
        let q1 = rot_2_quat(&(rot_z(0.5) * rot_x(0.3)));
        let q2 = rot_2_quat(&rot_y(-0.4));
        let q3 = quat_multiply(&q1, &q2);
        // JPL 乘法：R(q1⊗q2) = R(q1)·R(q2)（先应用 q2 再 q1）
        let r_composed = rot_z(0.5) * rot_x(0.3) * rot_y(-0.4);
        assert_mat_close(&quat_2_rot(&q3), &r_composed, 1e-10);
    }

    #[test]
    fn skew_vee_roundtrip() {
        let w = Vector3::new(0.3, -1.2, 0.7);
        assert_close((vee(&skew_x(&w)) - w).norm(), 0.0, 1e-12);
    }

    #[test]
    fn exp_log_so3_roundtrip() {
        let w = Vector3::new(0.2, -0.5, 0.8);
        let r = exp_so3(&w);
        assert_close((log_so3(&r) - w).norm(), 0.0, 1e-10);
        // 大角度
        let w2 = Vector3::new(1.5, 2.0, -0.7);
        let r2 = exp_so3(&w2);
        assert_close((log_so3(&r2) - w2).norm(), 0.0, 1e-8);
    }

    #[test]
    fn exp_log_se3_roundtrip() {
        let v = Vector6::new(0.2, -0.3, 0.5, 1.0, -0.5, 2.0);
        let t = exp_se3(&v);
        assert_close((log_se3(&t) - v).norm(), 0.0, 1e-8);
    }

    #[test]
    fn inv_se3_is_analytical_inverse() {
        let v = Vector6::new(0.1, 0.2, -0.3, 1.0, 2.0, 3.0);
        let t = exp_se3(&v);
        let tinv = inv_se3(&t);
        let identity = t * tinv;
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_close(identity[(i, j)], expected, 1e-10);
            }
        }
    }

    #[test]
    fn quat_inverse_and_identity() {
        let q = rot_2_quat(&rot_z(0.6));
        let qinv = inv_quat(&q);
        let identity = quat_multiply(&q, &qinv);
        assert_close(identity[3], 1.0, 1e-10);
        assert_close(identity[0].abs(), 0.0, 1e-10);
        assert_close(identity[1].abs(), 0.0, 1e-10);
        assert_close(identity[2].abs(), 0.0, 1e-10);
    }

    #[test]
    fn jacobians_small_angle() {
        let w_small = Vector3::new(1e-8, 0.0, 0.0);
        assert_mat_close(&jl_so3(&w_small), &Matrix3::identity(), 1e-6);
        let w = Vector3::new(0.3, -0.2, 0.5);
        assert_mat_close(&jr_so3(&w), &jl_so3(&-w), 1e-12);
    }

    #[test]
    fn rpy_roundtrip() {
        let r = rot_z(0.4) * rot_y(-0.3) * rot_x(0.2);
        let rpy = rot2rpy(&r);
        assert_close(rpy[0], 0.2, 1e-10);
        assert_close(rpy[1], -0.3, 1e-10);
        assert_close(rpy[2], 0.4, 1e-10);
    }

    #[test]
    fn omega_matrix_form() {
        let w = Vector3::new(0.1, 0.2, 0.3);
        let om = omega(&w);
        // Ω 的右上块是 w，左上块是 -⌊w×⌋
        assert_close(om[(0, 3)], 0.1, 1e-12);
        assert_close(om[(1, 3)], 0.2, 1e-12);
        assert_close(om[(2, 3)], 0.3, 1e-12);
        assert_close(om[(0, 1)], 0.3, 1e-12);
        assert_close(om[(3, 3)], 0.0, 1e-12);
    }
}
