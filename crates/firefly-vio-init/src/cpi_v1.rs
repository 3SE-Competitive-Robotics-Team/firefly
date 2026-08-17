//! 连续预积分 Model 1（对照 `ov_core::CpiV1` + `ov_core::CpiBase` 逐行翻译）。
//!
//! "piecewise constant measurement assumption"（分段常值测量假设），源于：
//! > Eckenhoff, Geneva, Huang. "High-accuracy preintegration for visual inertial
//! > navigation." WAFR 2016；理论见 IJRR "Continuous Preintegration Theory
//! > for Graph-based Visual-Inertial Navigation"。
//!
//! 使用步骤（与 C++ 注释对应）：
//! 1. `CpiV1::new` 构造（设置噪声 σ）；
//! 2. `set_linearization_points` 设置偏置线性化点；
//! 3. `feed_imu` 依次喂入要预积分的 IMU 测量；
//! 4. 读取公有成员获得均值、雅可比与测量协方差。

#![allow(clippy::many_single_char_names, clippy::similar_names)]

use firefly_vio_types::quat_ops::{rot_2_quat, skew_x};
use nalgebra::{DMatrix, Matrix3, Vector3, Vector4};
use std::ops::AddAssign;

/// 连续预积分 Model 1（对照 `ov_core::CpiV1`）。
///
/// 成员与 `CpiBase` 提供的公有状态一一对应：
/// 测量均值 / 偏置雅可比（`J_q`/`J_a`/`J_b`/`H_a`/`H_b`）/ 线性化点 /
/// 连续测量噪声 `Q_c`（12×12，由 σ 构造）与最终协方差 `P_meas`（15×15）。
#[derive(Debug, Clone)]
pub struct CpiV1 {
    /// 是否对相邻 IMU 测量取平均（对照 `imu_avg`；IJRR 论文默认不取平均）。
    pub imu_avg: bool,
    /// 累计积分时间（对照 `DT`）。
    pub dt: f64,
    /// 位移测量均值（对照 `alpha_tau`）。
    pub alpha_tau: Vector3<f64>,
    /// 速度测量均值（对照 `beta_tau`）。
    pub beta_tau: Vector3<f64>,
    /// 姿态测量均值（JPL 四元数；对照 `q_k2tau`）。
    pub q_k2tau: Vector4<f64>,
    /// 姿态测量均值（旋转矩阵；对照 `R_k2tau`）。
    pub r_k2tau: Matrix3<f64>,
    /// 姿态 wrt 陀螺偏置雅可比（对照 `J_q`）。
    pub j_q: Matrix3<f64>,
    /// 位移 wrt 陀螺偏置雅可比（对照 `J_a`）。
    pub j_a: Matrix3<f64>,
    /// 速度 wrt 陀螺偏置雅可比（对照 `J_b`）。
    pub j_b: Matrix3<f64>,
    /// 位移 wrt 加速度偏置雅可比（对照 `H_a`）。
    pub h_a: Matrix3<f64>,
    /// 速度 wrt 加速度偏置雅可比（对照 `H_b`）。
    pub h_b: Matrix3<f64>,
    /// 陀螺偏置线性化点（对照 `b_w_lin`）。
    pub b_w_lin: Vector3<f64>,
    /// 加速度偏置线性化点（对照 `b_a_lin`）。
    pub b_a_lin: Vector3<f64>,
    /// 重力（对照 `grav`；Model 1 不使用，保留与基类一致）。
    pub grav: Vector3<f64>,
    /// 连续测量噪声矩阵（12×12；对照 `Q_c`，由构造 σ 计算对角块）。
    pub q_c: DMatrix<f64>,
    /// 测量协方差（15×15；对照 `P_meas`）。
    pub p_meas: DMatrix<f64>,
}

impl CpiV1 {
    /// 构造 Model 1 预积分器（对照 `CpiV1::CpiV1` 与 `CpiBase::CpiBase`）。
    ///
    /// - `sigma_w` 陀螺白噪声密度（rad/s/sqrt(hz)）；
    /// - `sigma_wb` 陀螺随机游走（rad/s^2/sqrt(hz)）；
    /// - `sigma_a` 加速度计白噪声密度（m/s^2/sqrt(hz)）；
    /// - `sigma_ab` 加速度计随机游走（m/s^3/sqrt(hz)）；
    /// - `imu_avg` 是否取平均（IJRR 论文未取平均）。
    #[must_use]
    pub fn new(sigma_w: f64, sigma_wb: f64, sigma_a: f64, sigma_ab: f64, imu_avg: bool) -> Self {
        // 计算协方差矩阵：Q_c 的 4 个 3×3 对角块（对照 CpiBase 构造）。
        let eye = Matrix3::<f64>::identity();
        let mut q_c = DMatrix::<f64>::zeros(12, 12);
        for i in 0..3 {
            for j in 0..3 {
                q_c[(i, j)] = sigma_w * sigma_w * eye[(i, j)];
                q_c[(3 + i, 3 + j)] = sigma_wb * sigma_wb * eye[(i, j)];
                q_c[(6 + i, 6 + j)] = sigma_a * sigma_a * eye[(i, j)];
                q_c[(9 + i, 9 + j)] = sigma_ab * sigma_ab * eye[(i, j)];
            }
        }

        // C++ 中 `q_k2tau` 未在构造时显式赋值；R_k2tau = I。此处按
        // `R_k2tau = I` 推导 `q_k2tau = rot_2_quat(I)`，保证结构一致。
        let r_k2tau = Matrix3::<f64>::identity();
        let q_k2tau = rot_2_quat(&r_k2tau);

        Self {
            imu_avg,
            dt: 0.0,
            alpha_tau: Vector3::zeros(),
            beta_tau: Vector3::zeros(),
            q_k2tau,
            r_k2tau,
            j_q: Matrix3::zeros(),
            j_a: Matrix3::zeros(),
            j_b: Matrix3::zeros(),
            h_a: Matrix3::zeros(),
            h_b: Matrix3::zeros(),
            b_w_lin: Vector3::zeros(),
            b_a_lin: Vector3::zeros(),
            grav: Vector3::zeros(),
            q_c,
            p_meas: DMatrix::<f64>::zeros(15, 15),
        }
    }

    /// 设置陀螺/加速度偏置的线性化点（对照 `setLinearizationPoints`；
    /// Model 1 不使用 `q_k_lin` 与 `grav`）。
    pub fn set_linearization_points(&mut self, b_w_lin: Vector3<f64>, b_a_lin: Vector3<f64>) {
        self.b_w_lin = b_w_lin;
        self.b_a_lin = b_a_lin;
    }

    /// 预积分一次 IMU 测量段 `[t_0, t_1]`（对照 `CpiV1::feed_IMU`）。
    ///
    /// 先解析积分测量均值与偏置雅可比，再对测量协方差做数值积分（RK4）。
    /// C++ 的默认参数 `w_m_1`/`a_m_1`（零向量）由调用方显式传入——本契约
    /// 强制 6 参，与 C++ 全部调用点（均显式传参）一致。
    // 对照 C++ 的完整 RK4 协方差积分，函数天然冗长（逐行照搬）。
    #[allow(clippy::too_many_lines)]
    pub fn feed_imu(
        &mut self,
        t_0: f64,
        t_1: f64,
        w_m_0: &Vector3<f64>,
        a_m_0: &Vector3<f64>,
        w_m_1: &Vector3<f64>,
        a_m_1: &Vector3<f64>,
    ) {
        // 时间差
        let delta_t = t_1 - t_0;
        self.dt += delta_t;

        // 若没有时间经过则什么都不做
        if delta_t == 0.0 {
            return;
        }

        // 估计的 IMU 读数
        let mut w_hat = *w_m_0 - self.b_w_lin;
        let mut a_hat = *a_m_0 - self.b_a_lin;

        // 如果要取平均，平均处理
        if self.imu_avg {
            w_hat += *w_m_1 - self.b_w_lin;
            w_hat *= 0.5;
            a_hat += *a_m_1 - self.b_a_lin;
            a_hat *= 0.5;
        }

        // 角增量 w*dt
        let w_hatdt = w_hat * delta_t;

        // w_hat 的各分量
        let w_1 = w_hat[0];
        let w_2 = w_hat[1];
        let w_3 = w_hat[2];

        // w 与 wdt 的大小
        let mag_w = w_hat.norm();
        let w_dt = mag_w * delta_t;

        // 判断方程是否会不稳定的阈值
        let small_w = mag_w < 0.008_726_646;

        // 预积分方程中用到的一些变量
        let dt_2 = delta_t.powi(2);
        let cos_wt = w_dt.cos();
        let sin_wt = w_dt.sin();

        let eye3 = Matrix3::<f64>::identity();
        let w_x = skew_x(&w_hat);
        let a_x = skew_x(&a_hat);
        let w_tx = skew_x(&w_hatdt);
        let w_x_2 = w_x * w_x;

        //==========================================================================
        // 测量均值
        //==========================================================================

        // 相对旋转
        let r_tau2tau1 = if small_w {
            eye3 - delta_t * w_x + (delta_t.powi(2) / 2.0) * w_x_2
        } else {
            eye3 - (sin_wt / mag_w) * w_x + ((1.0 - cos_wt) / (mag_w.powi(2))) * w_x_2
        };

        // 更新后的旋转及其转置
        let r_k2tau1 = r_tau2tau1 * self.r_k2tau;
        let r_tau12k = r_k2tau1.transpose();

        // 用于评估测量/偏置雅可比更新的中间变量
        let (f_1, f_2, f_3, f_4) = if small_w {
            (
                -(delta_t.powi(3) / 3.0),
                delta_t.powi(4) / 8.0,
                -(delta_t.powi(2) / 2.0),
                delta_t.powi(3) / 6.0,
            )
        } else {
            (
                (w_dt * cos_wt - sin_wt) / mag_w.powi(3),
                (w_dt.powi(2) - 2.0 * cos_wt - 2.0 * w_dt * sin_wt + 2.0) / (2.0 * mag_w.powi(4)),
                -(1.0 - cos_wt) / mag_w.powi(2),
                (w_dt - sin_wt) / mag_w.powi(3),
            )
        };

        // 解析均值的主体部分
        let alpha_arg = (dt_2 / 2.0) * eye3 + f_1 * w_x + f_2 * w_x_2;
        let beta_arg = delta_t * eye3 + f_3 * w_x + f_4 * w_x_2;

        // 更新表达式中乘 a_hat 的矩阵
        let h_al = r_tau12k * alpha_arg;
        let h_be = r_tau12k * beta_arg;

        // 更新测量均值
        self.alpha_tau += self.beta_tau * delta_t + h_al * a_hat;
        self.beta_tau += h_be * a_hat;

        //==========================================================================
        // 偏置雅可比（解析）
        //==========================================================================

        // 右雅可比
        let j_r_tau1 = if small_w {
            eye3 - 0.5 * w_tx + (1.0 / 6.0) * w_tx * w_tx
        } else {
            eye3 - ((1.0 - cos_wt) / w_dt.powi(2)) * w_tx
                + ((w_dt - sin_wt) / w_dt.powi(3)) * w_tx * w_tx
        };

        // 更新姿态 wrt 陀螺偏置雅可比
        self.j_q = r_tau2tau1 * self.j_q + j_r_tau1 * delta_t;

        // 更新 alpha/beta wrt 加速度偏置雅可比
        self.h_a -= h_al;
        self.h_a += delta_t * self.h_b;
        self.h_b -= h_be;

        // 单位向量及其反对称（C++ 中是 CpiBase 成员；Rust 内按需构造）
        let e_1 = Vector3::new(1.0, 0.0, 0.0);
        let e_2 = Vector3::new(0.0, 1.0, 0.0);
        let e_3 = Vector3::new(0.0, 0.0, 1.0);
        let e_1x = skew_x(&e_1);
        let e_2x = skew_x(&e_2);
        let e_3x = skew_x(&e_3);

        // R_tau12k 对 bias_w 各分量的导数
        let d_r_bw_1 = -r_tau12k * skew_x(&(self.j_q * e_1));
        let d_r_bw_2 = -r_tau12k * skew_x(&(self.j_q * e_2));
        let d_r_bw_3 = -r_tau12k * skew_x(&(self.j_q * e_3));

        // 陀螺偏置雅可比项
        let (df_1_dbw, df_2_dbw, df_3_dbw, df_4_dbw) = if small_w {
            let df_1_dw_mag = -(delta_t.powi(5) / 15.0);
            let df_2_dw_mag = delta_t.powi(6) / 72.0;
            let df_3_dw_mag = -(delta_t.powi(4) / 12.0);
            let df_4_dw_mag = delta_t.powi(5) / 60.0;
            (
                [w_1 * df_1_dw_mag, w_2 * df_1_dw_mag, w_3 * df_1_dw_mag],
                [w_1 * df_2_dw_mag, w_2 * df_2_dw_mag, w_3 * df_2_dw_mag],
                [w_1 * df_3_dw_mag, w_2 * df_3_dw_mag, w_3 * df_3_dw_mag],
                [w_1 * df_4_dw_mag, w_2 * df_4_dw_mag, w_3 * df_4_dw_mag],
            )
        } else {
            let df_1_dw_mag =
                (w_dt.powi(2) * sin_wt - 3.0 * sin_wt + 3.0 * w_dt * cos_wt) / mag_w.powi(5);
            let df_2_dw_mag =
                (w_dt.powi(2) - 4.0 * cos_wt - 4.0 * w_dt * sin_wt + w_dt.powi(2) * cos_wt + 4.0)
                    / mag_w.powi(6);
            let df_3_dw_mag = (2.0 * (cos_wt - 1.0) + w_dt * sin_wt) / mag_w.powi(4);
            let df_4_dw_mag = (2.0 * w_dt + w_dt * cos_wt - 3.0 * sin_wt) / mag_w.powi(5);
            (
                [w_1 * df_1_dw_mag, w_2 * df_1_dw_mag, w_3 * df_1_dw_mag],
                [w_1 * df_2_dw_mag, w_2 * df_2_dw_mag, w_3 * df_2_dw_mag],
                [w_1 * df_3_dw_mag, w_2 * df_3_dw_mag, w_3 * df_3_dw_mag],
                [w_1 * df_4_dw_mag, w_2 * df_4_dw_mag, w_3 * df_4_dw_mag],
            )
        };

        // 更新 alpha/beta 的陀螺偏置雅可比
        // （C++ 中分为 6 个 block 子块赋值，逐一照搬）
        self.j_a += self.j_b * delta_t;

        // 逐列：df_*_dbw_{col} 对应 e_{col+1} 与 d_R_bw_{col+1}
        let col_terms = [
            (&d_r_bw_1, &e_1x, &df_1_dbw, &df_2_dbw, &df_3_dbw, &df_4_dbw),
            (&d_r_bw_2, &e_2x, &df_1_dbw, &df_2_dbw, &df_3_dbw, &df_4_dbw),
            (&d_r_bw_3, &e_3x, &df_1_dbw, &df_2_dbw, &df_3_dbw, &df_4_dbw),
        ];
        for (col, (d_r, e_x, a1, a2, b1, b2)) in col_terms.into_iter().enumerate() {
            // J_a 第 col 列
            let alpha_term = d_r * alpha_arg
                + r_tau12k
                    * (a1[col] * w_x - f_1 * e_x + a2[col] * w_x_2 - f_2 * (e_x * w_x + w_x * e_x));
            self.j_a
                .fixed_view_mut::<3, 1>(0, col)
                .add_assign(&(alpha_term * a_hat));
            // J_b 第 col 列
            let beta_term = d_r * beta_arg
                + r_tau12k
                    * (b1[col] * w_x - f_3 * e_x + b2[col] * w_x_2 - f_4 * (e_x * w_x + w_x * e_x));
            self.j_b
                .fixed_view_mut::<3, 1>(0, col)
                .add_assign(&(beta_term * a_hat));
        }

        //==========================================================================
        // 测量协方差
        //==========================================================================

        // 需要中间时刻（即 .5*dt）的姿态
        let mut r_mid = if small_w {
            eye3 - 0.5 * delta_t * w_x + ((0.5 * delta_t).powi(2) / 2.0) * w_x_2
        } else {
            eye3 - ((mag_w * 0.5 * delta_t).sin() / mag_w) * w_x
                + ((1.0 - (mag_w * 0.5 * delta_t).cos()) / mag_w.powi(2)) * w_x_2
        };
        r_mid *= self.r_k2tau;

        // 计算协方差（本实现使用 RK4）
        // k1---------------------------------------------------------------------------------------

        // 状态雅可比
        let mut f_k1 = DMatrix::<f64>::zeros(15, 15);
        for i in 0..3 {
            for j in 0..3 {
                f_k1[(i, j)] = -w_x[(i, j)];
                f_k1[(i, 3 + j)] = -eye3[(i, j)];
                f_k1[(6 + i, j)] = -(self.r_k2tau.transpose() * a_x)[(i, j)];
                f_k1[(6 + i, 9 + j)] = -(self.r_k2tau.transpose())[(i, j)];
                f_k1[(12 + i, 6 + j)] = eye3[(i, j)];
            }
        }

        // 噪声雅可比
        let mut g_k1 = DMatrix::<f64>::zeros(15, 12);
        for i in 0..3 {
            for j in 0..3 {
                g_k1[(i, j)] = -eye3[(i, j)];
                g_k1[(3 + i, 3 + j)] = eye3[(i, j)];
                g_k1[(6 + i, 6 + j)] = -(self.r_k2tau.transpose())[(i, j)];
                g_k1[(9 + i, 9 + j)] = eye3[(i, j)];
            }
        }

        // 协方差导数
        let p_dot_k1 = &f_k1 * &self.p_meas
            + &self.p_meas * f_k1.transpose()
            + &g_k1 * &self.q_c * &g_k1.transpose();

        // k2---------------------------------------------------------------------------------------

        // 状态雅可比
        let mut f_k2 = DMatrix::<f64>::zeros(15, 15);
        for i in 0..3 {
            for j in 0..3 {
                f_k2[(i, j)] = -w_x[(i, j)];
                f_k2[(i, 3 + j)] = -eye3[(i, j)];
                f_k2[(6 + i, j)] = -(r_mid.transpose() * a_x)[(i, j)];
                f_k2[(6 + i, 9 + j)] = -(r_mid.transpose())[(i, j)];
                f_k2[(12 + i, 6 + j)] = eye3[(i, j)];
            }
        }

        // 噪声雅可比
        let mut g_k2 = DMatrix::<f64>::zeros(15, 12);
        for i in 0..3 {
            for j in 0..3 {
                g_k2[(i, j)] = -eye3[(i, j)];
                g_k2[(3 + i, 3 + j)] = eye3[(i, j)];
                g_k2[(6 + i, 6 + j)] = -(r_mid.transpose())[(i, j)];
                g_k2[(9 + i, 9 + j)] = eye3[(i, j)];
            }
        }

        // 协方差导数
        let p_k2 = &self.p_meas + &p_dot_k1 * (delta_t / 2.0);
        let p_dot_k2 =
            &f_k2 * &p_k2 + &p_k2 * f_k2.transpose() + &g_k2 * &self.q_c * &g_k2.transpose();

        // k3---------------------------------------------------------------------------------------

        // 状态与噪声雅可比与 k2 相同
        let f_k3 = f_k2;
        let g_k3 = g_k2;

        // 协方差导数
        let p_k3 = &self.p_meas + &p_dot_k2 * (delta_t / 2.0);
        let p_dot_k3 =
            &f_k3 * &p_k3 + &p_k3 * f_k3.transpose() + &g_k3 * &self.q_c * &g_k3.transpose();

        // k4---------------------------------------------------------------------------------------

        // 状态雅可比
        let mut f_k4 = DMatrix::<f64>::zeros(15, 15);
        for i in 0..3 {
            for j in 0..3 {
                f_k4[(i, j)] = -w_x[(i, j)];
                f_k4[(i, 3 + j)] = -eye3[(i, j)];
                f_k4[(6 + i, j)] = -(r_k2tau1.transpose() * a_x)[(i, j)];
                f_k4[(6 + i, 9 + j)] = -(r_k2tau1.transpose())[(i, j)];
                f_k4[(12 + i, 6 + j)] = eye3[(i, j)];
            }
        }

        // 噪声雅可比
        let mut g_k4 = DMatrix::<f64>::zeros(15, 12);
        for i in 0..3 {
            for j in 0..3 {
                g_k4[(i, j)] = -eye3[(i, j)];
                g_k4[(3 + i, 3 + j)] = eye3[(i, j)];
                g_k4[(6 + i, 6 + j)] = -(r_k2tau1.transpose())[(i, j)];
                g_k4[(9 + i, 9 + j)] = eye3[(i, j)];
            }
        }

        // 协方差导数
        let p_k4 = &self.p_meas + &p_dot_k3 * delta_t;
        let p_dot_k4 =
            &f_k4 * &p_k4 + &p_k4 * f_k4.transpose() + &g_k4 * &self.q_c * &g_k4.transpose();

        // done----------------------------------------------------------------------------------------

        // 收集协方差解，确保正定
        self.p_meas +=
            (delta_t / 6.0) * (&p_dot_k1 + 2.0 * &p_dot_k2 + 2.0 * &p_dot_k3 + &p_dot_k4);
        self.p_meas = 0.5 * (&self.p_meas + self.p_meas.transpose());

        // 更新旋转均值
        // 注意必须等到这里才更新，因为协方差计算中使用了旧姿态
        self.r_k2tau = r_k2tau1;
        self.q_k2tau = rot_2_quat(&self.r_k2tau);
    }
}

#[cfg(test)]
mod tests {
    // 测试内浮点严格比较与数学标识符（`P_meas`/`R_k2tau` 等）在 doc 中的
    // 反引号提示，均属测试断言意图，予以允许。
    #![allow(clippy::float_cmp, clippy::doc_markdown)]
    use super::*;
    use firefly_vio_types::quat_ops::quat_2_rot;

    fn assert_close(a: f64, b: f64, eps: f64) {
        assert!((a - b).abs() < eps, "{a} != {b} (eps={eps})");
    }

    /// 小角度恒定角速度/加速度单步：w=(0,0,0.1), a=(0,0,9.81), dt=0.1。
    /// R 绕 z 转约 0.01 rad（OpenVINS 约定 `R[(1,0)] = -sin(w*dt)`）；
    /// alpha == 0.5*a*dt²、beta == a*dt（a 平行于旋转轴 z，严格成立）。
    #[test]
    fn single_step_small_angle_matches_analytic() {
        let mut cpi = CpiV1::new(1.7e-4, 1.9e-5, 2.0e-3, 3.0e-3, false);
        let w = Vector3::new(0.0, 0.0, 0.1);
        let a = Vector3::new(0.0, 0.0, 9.81);
        let dt = 0.1;

        cpi.feed_imu(0.0, dt, &w, &a, &w, &a);

        // 角度增量 0.1*0.1 = 0.01 rad，绕 z（C++ 公式第一阶即 I - dt*w_x）
        assert_close(cpi.r_k2tau[(0, 0)], 0.01_f64.cos(), 1e-6);
        assert_close(cpi.r_k2tau[(1, 0)], -0.01_f64.sin(), 1e-6);
        assert_close(cpi.r_k2tau[(0, 1)], 0.01_f64.sin(), 1e-6);
        assert_close(cpi.r_k2tau[(1, 1)], 0.01_f64.cos(), 1e-6);
        assert_close(cpi.r_k2tau[(2, 2)], 1.0, 1e-6);
        // q 与 R 一致
        assert_close(quat_2_rot(&cpi.q_k2tau)[(1, 0)], cpi.r_k2tau[(1, 0)], 1e-6);

        // w_x*a = w×a = 0 且 w_x²*a = 0，而绕 z 旋转保持 a 不变：
        // alpha_tau == 0.5*a*dt²，beta_tau == a*dt（解析严格成立）
        assert_close(cpi.beta_tau[2], 9.81 * dt, 1e-9);
        assert_close(cpi.alpha_tau[2], 0.5 * 9.81 * dt * dt, 1e-9);
        // 其余方向为零（旋转轴平行于 a，无横向分量）
        assert_close(cpi.alpha_tau[0].abs(), 0.0, 1e-12);
        assert_close(cpi.alpha_tau[1].abs(), 0.0, 1e-12);
        assert_close(cpi.beta_tau[0].abs(), 0.0, 1e-12);
        assert_close(cpi.beta_tau[1].abs(), 0.0, 1e-12);
    }

    /// 零 dt 不改变任何状态。
    #[test]
    fn zero_dt_does_not_change_state() {
        let mut cpi = CpiV1::new(1.7e-4, 1.9e-5, 2.0e-3, 3.0e-3, false);
        let before_r = cpi.r_k2tau;
        let before_q = cpi.q_k2tau;
        let before_p = cpi.p_meas.clone();
        let before_alpha = cpi.alpha_tau;

        let w = Vector3::new(0.1, 0.2, 0.3);
        let a = Vector3::new(1.0, 2.0, 3.0);
        cpi.feed_imu(1.0, 1.0, &w, &a, &w, &a);

        assert!(before_r == cpi.r_k2tau, "R_k2tau should not change");
        assert!(before_q == cpi.q_k2tau, "q_k2tau should not change");
        assert!(before_alpha == cpi.alpha_tau, "alpha_tau should not change");
        assert!(before_p == cpi.p_meas, "P_meas should not change");
        // DT 仍累加（对照 C++：DT += delta_t 在 zero-dt 检查之前），此处 delta_t=0
        assert!(
            (cpi.dt - 0.0).abs() < 1e-15,
            "dt should stay ~0, got {}",
            cpi.dt
        );
    }

    /// P_meas 对称且半正定（特征值 ≥ 0）。
    #[test]
    fn p_meas_symmetric_psd() {
        let mut cpi = CpiV1::new(1.7e-4, 1.9e-5, 2.0e-3, 3.0e-3, false);
        let w = Vector3::new(0.1, -0.2, 0.05);
        let a = Vector3::new(0.0, 0.0, 9.81);
        for k in 0..10 {
            let t0 = f64::from(k) * 0.01;
            cpi.feed_imu(t0, t0 + 0.01, &w, &a, &w, &a);
        }

        // 对称（喂入即做 0.5*(P+P^T)，检查对称性）
        for i in 0..15 {
            for j in 0..15 {
                assert_close(cpi.p_meas[(i, j)], cpi.p_meas[(j, i)], 1e-12);
            }
        }
        // 半正定：检查所有主子阵行列式/特征值。直接用特征值。
        let p = cpi.p_meas.clone();
        let e = nalgebra::SymmetricEigen::new(p).eigenvalues;
        for v in &e {
            assert!(*v >= -1e-9, "eigenvalue {v} should be >= 0");
        }
    }
}
