//! `JPL` 四元数与 `Lie` 群运算（对照 `OpenVINS` `ov_core/quat_ops.h` 逐行翻译）。
//!
//! JPL 惯例：四元数 `[x, y, z, w]`（标量在最后），与 Hamilton 相反；
//! 旋转合成 `q ⊗ p` 采用 JPL 乘法（`quat_multiply`）。

use nalgebra::{Matrix3, Matrix4, Vector3, Vector4, Vector6};

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

/// 反对称矩阵 `⌊w×⌋`。
#[must_use]
pub fn skew_x(w: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -w[2], w[1], w[2], 0.0, -w[0], -w[1], w[0], 0.0)
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

/// 反对称矩阵 → 向量部分。
#[must_use]
pub fn vee(w_x: &Matrix3<f64>) -> Vector3<f64> {
    Vector3::new(w_x[(2, 1)], w_x[(0, 2)], w_x[(1, 0)])
}

/// `SO(3)` 矩阵指数（Rodrigues，Eade Eq.15）。
#[must_use]
pub fn exp_so3(w: &Vector3<f64>) -> Matrix3<f64> {
    let w_x = skew_x(w);
    let theta = w.norm();
    let (a, b) = if theta < 1e-7 {
        (1.0, 0.5)
    } else {
        (theta.sin() / theta, (1.0 - theta.cos()) / (theta * theta))
    };
    if theta == 0.0 {
        Matrix3::identity()
    } else {
        Matrix3::identity() + a * w_x + b * (w_x * w_x)
    }
}

/// `SO(3)` 矩阵对数（GTSAM 稳定版，含 π 边界处理）。
#[must_use]
pub fn log_so3(rot: &Matrix3<f64>) -> Vector3<f64> {
    let r11 = rot[(0, 0)];
    let r12 = rot[(0, 1)];
    let r13 = rot[(0, 2)];
    let r21 = rot[(1, 0)];
    let r22 = rot[(1, 1)];
    let r23 = rot[(1, 2)];
    let r31 = rot[(2, 0)];
    let r32 = rot[(2, 1)];
    let r33 = rot[(2, 2)];
    let tr = rot.trace();
    if tr + 1.0 < 1e-10 {
        // θ = ±π, ±3π, ...：特殊处理
        if (r33 + 1.0).abs() > 1e-5 {
            (std::f64::consts::PI / (2.0 + 2.0 * r33).sqrt()) * Vector3::new(r13, r23, 1.0 + r33)
        } else if (r22 + 1.0).abs() > 1e-5 {
            (std::f64::consts::PI / (2.0 + 2.0 * r22).sqrt()) * Vector3::new(r12, 1.0 + r22, r32)
        } else {
            (std::f64::consts::PI / (2.0 + 2.0 * r11).sqrt()) * Vector3::new(1.0 + r11, r21, r31)
        }
    } else {
        let tr_3 = tr - 3.0;
        let magnitude = if tr_3 < -1e-7 {
            let theta = ((tr - 1.0) / 2.0).acos();
            theta / (2.0 * theta.sin())
        } else {
            // θ 接近 0：泰勒展开
            0.5 - tr_3 / 12.0
        };
        magnitude * Vector3::new(r32 - r23, r13 - r31, r21 - r12)
    }
}

/// `SE(3)` 矩阵指数（Eade Eq.19-21）。
#[must_use]
pub fn exp_se3(vec: &Vector6<f64>) -> Matrix4<f64> {
    let w = Vector3::new(vec[0], vec[1], vec[2]);
    let u = Vector3::new(vec[3], vec[4], vec[5]);
    let theta = w.norm();
    let w_skew = skew_x(&w);
    let (coef_a, coef_b, coef_c) = if theta < 1e-7 {
        (1.0, 0.5, 1.0 / 6.0)
    } else {
        (
            theta.sin() / theta,
            (1.0 - theta.cos()) / (theta * theta),
            (1.0 - theta.sin() / theta) / (theta * theta),
        )
    };
    let i33 = Matrix3::identity();
    let v = i33 + coef_b * w_skew + coef_c * (w_skew * w_skew);
    let mut mat = Matrix4::<f64>::zeros();
    mat.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(i33 + coef_a * w_skew + coef_b * (w_skew * w_skew)));
    mat.fixed_view_mut::<3, 1>(0, 3).copy_from(&(v * u));
    mat[(3, 3)] = 1.0;
    mat
}

/// `SE(3)` 矩阵对数（GTSAM 稳定版，Agrawal06iros Eq.14）。
#[must_use]
pub fn log_se3(mat: &Matrix4<f64>) -> Vector6<f64> {
    let w = log_so3(&mat.fixed_view::<3, 3>(0, 0).into_owned());
    let t = Vector3::new(mat[(0, 3)], mat[(1, 3)], mat[(2, 3)]);
    let norm = w.norm();
    if norm < 1e-10 {
        return Vector6::new(w[0], w[1], w[2], t[0], t[1], t[2]);
    }
    let w_hat = skew_x(&(w / norm));
    let tan = (0.5 * norm).tan();
    let wt = w_hat * t;
    let u = t - (0.5 * norm) * wt + (1.0 - norm / (2.0 * tan)) * (w_hat * wt);
    Vector6::new(w[0], w[1], w[2], u[0], u[1], u[2])
}

/// `R6 -> se(3)` hat 算子。
#[must_use]
pub fn hat_se3(vec: &Vector6<f64>) -> Matrix4<f64> {
    let w = Vector3::new(vec[0], vec[1], vec[2]);
    let u = Vector3::new(vec[3], vec[4], vec[5]);
    let mut mat = Matrix4::<f64>::zeros();
    mat.fixed_view_mut::<3, 3>(0, 0).copy_from(&skew_x(&w));
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

/// `SO(3)` 左雅可比（Barfoot Eq.7.77b）。
#[must_use]
pub fn jl_so3(w: &Vector3<f64>) -> Matrix3<f64> {
    let theta = w.norm();
    if theta < 1e-6 {
        Matrix3::identity()
    } else {
        let axis = w / theta;
        theta.sin() / theta * Matrix3::identity()
            + (1.0 - theta.sin() / theta) * (axis * axis.transpose())
            + ((1.0 - theta.cos()) / theta) * skew_x(&axis)
    }
}

/// `SO(3)` 右雅可比 `Jr(w) = Jl(−w)`。
#[must_use]
pub fn jr_so3(w: &Vector3<f64>) -> Matrix3<f64> {
    jl_so3(&-w)
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
