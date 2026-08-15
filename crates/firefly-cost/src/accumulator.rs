//! 采样点梯度累加器。
//!
//! 每个采样点贡献状态梯度 (∂J/∂p, ∂J/∂v, ∂J/∂a, ∂J/∂j)，累加为：
//! - ∂F/∂c：经 β 基函数（自变量为段内真实时间 t = τ·Tᵢ）
//! - ∂F/∂T 显式部分：采样点段内位置 τ 固定，t = t_{i-1} + `τ·T_i`，
//!   前面段时长变化被完全抵消，仅本段贡献：
//!   ∂p/∂Tᵢ = v·τ，∂v/∂Tᵢ = a·τ，∂a/∂Tᵢ = j·τ，∂j/∂Tᵢ = snap·τ
//!
//! 隐式部分（∂c/∂T）由 `Minco::propagate_gradient` 处理。

use firefly_trajectory::Sample;
use nalgebra::{DMatrix, DVector, Vector3};

pub struct Accumulator {
    pub d_f_d_c: DMatrix<f64>,
    pub d_f_d_t: DVector<f64>,
}

impl Accumulator {
    #[must_use]
    pub fn new(pieces: usize) -> Self {
        Self {
            d_f_d_c: DMatrix::zeros(6 * pieces, 3),
            d_f_d_t: DVector::zeros(pieces),
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn add(
        &mut self,
        piece: usize,
        tau: f64,
        duration: f64,
        s: &Sample,
        d_p: Vector3<f64>,
        d_v: Vector3<f64>,
        d_a: Vector3<f64>,
        d_j: Vector3<f64>,
    ) {
        let t_local = tau * duration;
        let beta0 = beta_derivative(0, t_local);
        let beta1 = beta_derivative(1, t_local);
        let beta2 = beta_derivative(2, t_local);
        let beta3 = beta_derivative(3, t_local);

        let row = piece * 6;
        for k in 0..6 {
            for dim in 0..3 {
                self.d_f_d_c[(row + k, dim)] += beta0[k] * d_p[dim]
                    + beta1[k] * d_v[dim]
                    + beta2[k] * d_a[dim]
                    + beta3[k] * d_j[dim];
            }
        }

        self.d_f_d_t[piece] +=
            (d_p.dot(&s.velocity) + d_v.dot(&s.acceleration) + d_a.dot(&s.jerk) + d_j.dot(&s.snap))
                * tau;
    }

    /// 绝对时间采样模式（集群避碰/队形用）。
    ///
    /// 采样点固定在绝对时间 `t_abs` = Σ_{l<piece} `T_l` + `τ·T_piece`，
    /// 因此前面所有段时长变化都会移动采样点（论文 Eq. S28）：
    /// ∂`t_abs/∂T_l` = 1 (l < `piece)，∂t_abs/∂T_piece` = τ。
    /// 参考轨迹（peer/guide）也在同一绝对时刻求值，时间移动用
    /// **相对速度**（本机 − 参考）：`reference_velocity` 为参考轨迹速度。
    #[allow(clippy::too_many_arguments)]
    pub fn add_absolute(
        &mut self,
        piece: usize,
        tau: f64,
        duration: f64,
        s: &Sample,
        reference_velocity: Vector3<f64>,
        d_p: Vector3<f64>,
        d_v: Vector3<f64>,
        d_a: Vector3<f64>,
        d_j: Vector3<f64>,
    ) {
        let t_local = tau * duration;
        let beta0 = beta_derivative(0, t_local);
        let beta1 = beta_derivative(1, t_local);
        let beta2 = beta_derivative(2, t_local);
        let beta3 = beta_derivative(3, t_local);

        let row = piece * 6;
        for k in 0..6 {
            for dim in 0..3 {
                self.d_f_d_c[(row + k, dim)] += beta0[k] * d_p[dim]
                    + beta1[k] * d_v[dim]
                    + beta2[k] * d_a[dim]
                    + beta3[k] * d_j[dim];
            }
        }

        // 绝对时间移动（论文 Eq. S28）分两部分：
        // - 本段（τ 项）：采样点段内时间随 T_piece 变，本机与参考都移动，
        //   用相对速度：∂t/∂T_piece = τ
        // - 前面段（系数 1）：采样点绝对时间移动，但**本机段内位置不变**
        //   （段内时间 = τ·T_i 与前面段无关），只有参考轨迹（绝对时刻
        //   求值的 peer/guide）移动：∂f/∂p_ref·v_ref = −d_p·v_ref
        let time_derivative_local = d_p.dot(&(s.velocity - reference_velocity))
            + d_v.dot(&s.acceleration)
            + d_a.dot(&s.jerk)
            + d_j.dot(&s.snap);
        self.d_f_d_t[piece] += time_derivative_local * tau;
        let reference_only = -d_p.dot(&reference_velocity);
        for l in 0..piece {
            self.d_f_d_t[l] += reference_only;
        }
    }
}

fn beta_derivative(order: usize, t: f64) -> [f64; 6] {
    let mut v = [0.0; 6];
    for (j, slot) in v.iter_mut().enumerate().skip(order) {
        let mut coeff = 1.0;
        for k in 0..order {
            coeff *= (j - k) as f64;
        }
        *slot = coeff * t.powi((j - order) as i32);
    }
    v
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn gradient_mapping() {
        // 零状态梯度 → 无效果
        let mut acc = Accumulator::new(1);
        let zero = Sample {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            jerk: Vector3::zeros(),
            snap: Vector3::zeros(),
        };
        acc.add(
            0,
            0.0,
            2.0,
            &zero,
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        assert_eq!(acc.d_f_d_c[(0, 0)], 0.0);
        // 位置梯度经基映射到系数：τ=0 时 β(0)=[1,0,0,0,0,0]，梯度只在 (0,0) 为 1
        let mut acc = Accumulator::new(1);
        acc.add(
            0,
            0.0,
            2.0,
            &zero,
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        assert_eq!(acc.d_f_d_c[(0, 0)], 1.0);
        assert_eq!(acc.d_f_d_c[(1, 0)], 0.0);
    }
}
