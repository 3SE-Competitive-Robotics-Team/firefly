//! SO(3) 指数/对数与雅可比（对照 `include/utils/so3_math.h` 逐行翻译）。
//!
//! 记号同官方：`Exp(ω)` 为右乘扰动的 SO(3) 指数，`Log(R)` 为其逆。
//! 本模块是 `State` boxplus/boxminus 的旋转分量基础。

use nalgebra::{Matrix3, Vector3};

/// SO(3) 指数映射 `Exp(ω)`（Rodrigues，对照 `so3_math.h:9` `Exp(rvalue)`）。
///
/// 模长阈值 `1e-7` 与官方一致，小于阈值时返回单位阵。
#[must_use]
pub fn exp(ang: Vector3<f64>) -> Matrix3<f64> {
    let ang_norm = ang.norm();
    if ang_norm > 1e-7 {
        let axis = ang / ang_norm;
        let k = skew(&axis);
        Matrix3::identity() + ang_norm.sin() * k + (1.0 - ang_norm.cos()) * (k * k)
    } else {
        Matrix3::identity()
    }
}

/// 角速度×时间间隔的指数映射 `Exp(ω·dt)`（对照 `so3_math.h:24`）。
///
/// 用于离散化旋转传播：`R ← R · Exp(ω_avr·dt)`。
#[must_use]
pub fn exp_dt(ang_vel: Vector3<f64>, dt: f64) -> Matrix3<f64> {
    let ang_norm = ang_vel.norm();
    if ang_norm > 1e-7 {
        let axis = ang_vel / ang_norm;
        let k = skew(&axis);
        let ang = ang_norm * dt;
        Matrix3::identity() + ang.sin() * k + (1.0 - ang.cos()) * (k * k)
    } else {
        Matrix3::identity()
    }
}

/// SO(3) 对数映射 `Log(R)`（对照 `so3_math.h:61`）。
///
/// 小角度分支 `θ<1e-3` 用 `0.5·vee(R−Rᵀ)` 近似，避免除零。
#[must_use]
pub fn log(rot: &Matrix3<f64>) -> Vector3<f64> {
    let theta = if rot.trace() > 3.0 - 1e-6 {
        0.0
    } else {
        (0.5 * (rot.trace() - 1.0)).acos()
    };
    let k = Vector3::new(
        rot[(2, 1)] - rot[(1, 2)],
        rot[(0, 2)] - rot[(2, 0)],
        rot[(1, 0)] - rot[(0, 1)],
    );
    if theta.abs() < 0.001 {
        0.5 * k
    } else {
        0.5 * theta / theta.sin() * k
    }
}

/// 反对称矩阵 `⌊v×⌋`（对照 `so3_math.h:7` 的 `SKEW_SYM_MATRX`）。
#[must_use]
pub fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v[2], v[1], //
        v[2], 0.0, -v[0], //
        -v[1], v[0], 0.0,
    )
}

/// SO(3) 左雅可比 `J_l(ω)`（FAST-LIVO2 `Exp` 的解析导数）。
///
/// 用于协方差传播中旋转分量的线性化，见论文 (1) 式。
#[must_use]
pub fn left_jacobian(ang: Vector3<f64>) -> Matrix3<f64> {
    let theta = ang.norm();
    if theta < 1e-6 {
        Matrix3::identity()
    } else {
        let axis = ang / theta;
        let k = skew(&axis);
        theta.sin() / theta * Matrix3::identity()
            + (1.0 - theta.sin() / theta) * (axis * axis.transpose())
            + (1.0 - theta.cos()) / theta * k
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(a: f64, b: f64, eps: f64) {
        assert!((a - b).abs() < eps, "{a} != {b}");
    }

    #[test]
    fn exp_log_roundtrip() {
        // 小角度与大角度往返（对照 so3_math.h 的 Exp/Log 数值行为；
        // 官方 Log 的 acos 分支在 θ→π 附近精度退化，测试取远离 π 的角度）
        for w in [
            Vector3::new(1e-8, 0.0, 0.0),
            Vector3::new(0.2, -0.5, 0.8),
            Vector3::new(1.2, 1.5, -0.6),
        ] {
            let r = exp(w);
            let back = log(&r);
            let err = (back - w).norm();
            assert!(err < 1e-7, "roundtrip err={err}");
        }
    }

    #[test]
    fn exp_dt_matches_exp_scaled() {
        let w = Vector3::new(0.3, -0.2, 0.5);
        let dt = 0.1;
        let a = exp_dt(w, dt);
        let b = exp(w * dt);
        for i in 0..3 {
            for j in 0..3 {
                assert_close(a[(i, j)], b[(i, j)], 1e-12);
            }
        }
    }

    #[test]
    fn skew_vee_identity() {
        let v = Vector3::new(0.3, -1.2, 0.7);
        let s = skew(&v);
        // 反对称矩阵的非对角元编码 v
        assert_close(s[(0, 1)], -v[2], 1e-15);
        assert_close(s[(1, 0)], v[2], 1e-15);
        assert_close(s[(0, 2)], v[1], 1e-15);
        assert_close(s[(2, 0)], -v[1], 1e-15);
        assert_close(s[(1, 2)], -v[0], 1e-15);
        assert_close(s[(2, 1)], v[0], 1e-15);
    }

    #[test]
    fn left_jacobian_small_angle() {
        // θ→0 时 J_l→I
        let j = left_jacobian(Vector3::new(1e-9, 0.0, 0.0));
        for i in 0..3 {
            for jj in 0..3 {
                assert_close(j[(i, jj)], if i == jj { 1.0 } else { 0.0 }, 1e-6);
            }
        }
        // 非零角度的解析形式（Barfoot 7.77b）
        let w = Vector3::new(0.4, -0.2, 0.6);
        let theta: f64 = w.norm();
        let axis = w / theta;
        let k = skew(&axis);
        let expect = theta.sin() / theta * Matrix3::identity()
            + (1.0 - theta.sin() / theta) * (axis * axis.transpose())
            + (1.0 - theta.cos()) / theta * k;
        let j = left_jacobian(w);
        for i in 0..3 {
            for jj in 0..3 {
                assert_close(j[(i, jj)], expect[(i, jj)], 1e-12);
            }
        }
    }
}
