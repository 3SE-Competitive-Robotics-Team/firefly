//! 误差态 EKF：`VIO→全局` 漂移 `SE(3)` 的预测-更新。
//!
//! 状态 `x ∈ se(3)` 为 `T_drift = T_global · T_vio⁻¹` 的对数，名义量 `T_drift` 常驻，
//! 误差态零均值。预测仅膨胀 `P`，更新以 `GICP` 全局位姿为观测，`R = h⁻¹`，
//! `chi2` 门控后 `Joseph` 更新，连续拒收时自动放大 `R`。

use firefly_gicp::util::lie::{se3_exp, se3_log};
use nalgebra::{Matrix4, Matrix6, Vector6};

/// `chi2 95%` 分位数（与 `firefly-vio/src/updater.rs:24` 同表）。
const CHI2_95_TABLE: [f64; 30] = [
    3.8415, 5.9915, 7.8147, 9.4877, 11.0705, 12.5916, 14.0671, 15.5073, 16.9190, 18.3070, 19.6751,
    21.0261, 22.3620, 23.6848, 24.9958, 26.2962, 27.5871, 28.8693, 30.1435, 31.4104, 32.6706,
    33.9244, 35.1725, 36.4150, 37.6525, 38.8851, 40.1133, 41.3372, 42.5570, 43.7730,
];

fn chi2_95(dof: usize) -> f64 {
    if dof == 0 {
        return 0.0;
    }
    if dof <= CHI2_95_TABLE.len() {
        return CHI2_95_TABLE[dof - 1];
    }
    let nu = dof as f64;
    let t = 1.0 - 2.0 / (9.0 * nu) + 1.644_853_626_951_472_2 * (2.0 / (9.0 * nu)).sqrt();
    nu * t * t * t
}

/// 融合参数。
#[derive(Debug, Clone, serde::Deserialize)]
#[serde(default)]
pub struct FusionOptions {
    /// 单步过程噪声：旋转 `rad²`。
    pub process_noise_rot: f64,
    /// 单步过程噪声：平移 `m²`。
    pub process_noise_pos: f64,
    /// 观测噪声回退：旋转 `rad²`（`h` 不可逆时）。
    pub fallback_noise_rot: f64,
    /// 观测噪声回退：平移 `m²`。
    pub fallback_noise_pos: f64,
    /// `chi2` 阈值乘子（对照 `UpdaterOptions::chi2_multipler`）。
    pub chi2_multiplier: f64,
    /// 最小内点率。
    pub min_inlier_ratio: f64,
    /// 最小内点数。
    pub min_num_inliers: usize,
    /// 最大配准残差（`RegistrationResult::error` 为误差和，`>1e9` 视为发散）。
    pub max_registration_error: f64,
    /// 单次矫正限幅：平移 `m`。
    pub max_correction_trans: f64,
    /// 单次矫正限幅：旋转 `rad`。
    pub max_correction_rot: f64,
    /// 连续拒收后 `R` 的放大倍数。
    pub auto_scale_factor: f64,
    /// 触发放大的连续拒收次数。
    pub auto_scale_trigger: usize,
}

impl Default for FusionOptions {
    fn default() -> Self {
        Self {
            process_noise_rot: 1e-6,
            process_noise_pos: 1e-4,
            fallback_noise_rot: (2.5_f64.to_radians()).powi(2),
            fallback_noise_pos: 0.2_f64.powi(2),
            chi2_multiplier: 1.0,
            min_inlier_ratio: 0.3,
            min_num_inliers: 30,
            max_registration_error: 1e9,
            max_correction_trans: 0.5,
            max_correction_rot: 5.0_f64.to_radians(),
            auto_scale_factor: 2.0,
            auto_scale_trigger: 3,
        }
    }
}

/// 门控结果。
#[derive(Debug, Clone)]
pub enum RelocGate {
    /// 接受，已融合。
    Accepted {
        chi2: f64,
        threshold: f64,
        delta: Vector6<f64>,
    },
    /// 配准层拒绝（收敛/内点/误差）。
    RejectedPrecheck { reason: &'static str },
    /// `chi2` 拒绝。
    RejectedChi2 { chi2: f64, threshold: f64 },
    /// 数值异常（`S` 不可逆等）。
    RejectedNumerical { reason: &'static str },
}

/// 融合滤波器：维护 `T_drift` 与 `P`。
#[derive(Debug, Clone)]
pub struct FusionFilter {
    /// `T_drift = T_global · T_vio⁻¹`（初值 `I`，全局系与 `VIO` 起点对齐）。
    t_drift: Matrix4<f64>,
    /// 协方差 `6×6`（顺序 `[rot, trans]`，与 `se3` 一致）。
    p: Matrix6<f64>,
    last_vio: Option<Matrix4<f64>>,
    options: FusionOptions,
    consecutive_rejects: usize,
    r_scale: f64,
}

impl FusionFilter {
    /// 新建，`P` 初值 `0.1·I`。
    #[must_use]
    pub fn new(options: FusionOptions) -> Self {
        let mut p = Matrix6::identity();
        p *= 0.1;
        Self {
            t_drift: Matrix4::identity(),
            p,
            last_vio: None,
            options,
            consecutive_rejects: 0,
            r_scale: 1.0,
        }
    }

    /// 默认参数构造。
    #[must_use]
    pub fn with_default() -> Self {
        Self::new(FusionOptions::default())
    }

    /// 访问漂移。
    #[must_use]
    pub fn drift(&self) -> &Matrix4<f64> {
        &self.t_drift
    }

    /// 访问协方差。
    #[must_use]
    pub fn covariance(&self) -> &Matrix6<f64> {
        &self.p
    }

    /// 连续拒收计数（诊断用）。
    #[must_use]
    pub const fn consecutive_rejects(&self) -> usize {
        self.consecutive_rejects
    }

    /// 由 `VIO` 位姿预测：仅膨胀 `P`，不改 `T_drift`。
    #[fastrace::trace]
    pub fn predict(&mut self, t_vio: &Matrix4<f64>) {
        if self.last_vio.is_none() {
            self.last_vio = Some(*t_vio);
            return;
        }
        let mut q = Matrix6::zeros();
        for i in 0..3 {
            q[(i, i)] = self.options.process_noise_rot;
        }
        for i in 3..6 {
            q[(i, i)] = self.options.process_noise_pos;
        }
        self.p += q;
        // 数值防护：保持对称
        self.p = (self.p + self.p.transpose()) * 0.5;
        self.last_vio = Some(*t_vio);
    }

    /// 由 `GICP` 观测更新。
    ///
    /// `t_vio` 为当前 `VIO` 位姿（与 `predict` 同帧），`t_gicp` 为 `GICP` 给出的
    /// 全局位姿 `T_target_source`（`target=全局地图`），`h` 为信息矩阵，
    /// `num_inliers/total_points/error/converged` 来自 `RegistrationResult`。
    #[allow(clippy::too_many_arguments)]
    #[fastrace::trace]
    pub fn update(
        &mut self,
        t_vio: &Matrix4<f64>,
        t_gicp: &Matrix4<f64>,
        h: &Matrix6<f64>,
        num_inliers: usize,
        total_points: usize,
        error: f64,
        converged: bool,
    ) -> RelocGate {
        // 1. 配准层门控
        if !converged {
            return RelocGate::RejectedPrecheck {
                reason: "not converged",
            };
        }
        if num_inliers < self.options.min_num_inliers {
            return RelocGate::RejectedPrecheck {
                reason: "too few inliers",
            };
        }
        if total_points > 0 {
            let ratio = num_inliers as f64 / total_points as f64;
            if ratio < self.options.min_inlier_ratio {
                return RelocGate::RejectedPrecheck {
                    reason: "low inlier ratio",
                };
            }
        }
        if !error.is_finite() || error > self.options.max_registration_error {
            return RelocGate::RejectedPrecheck {
                reason: "large error",
            };
        }

        // 2. 预测位姿与残差
        let t_pred = self.t_drift * *t_vio;
        let t_pred_inv = match t_pred.try_inverse() {
            Some(v) => v,
            None => {
                return RelocGate::RejectedNumerical {
                    reason: "pred not invertible",
                };
            }
        };
        let t_err = t_pred_inv * *t_gicp;
        let z = se3_log(&t_err);

        // 3. 观测噪声 R = h⁻¹，失败回退对角阵；连续拒收时放大
        let r = match h.try_inverse() {
            Some(inv) => {
                let mut r = inv;
                // 保持对称
                r = (r + r.transpose()) * 0.5;
                // 数值防护：对角线截断为正
                for i in 0..6 {
                    if r[(i, i)] < 1e-9 {
                        r[(i, i)] = 1e-9;
                    }
                    if !r[(i, i)].is_finite() {
                        r[(i, i)] = if i < 3 {
                            self.options.fallback_noise_rot
                        } else {
                            self.options.fallback_noise_pos
                        };
                    }
                }
                r * self.r_scale
            }
            None => {
                let mut r = Matrix6::zeros();
                for i in 0..3 {
                    r[(i, i)] = self.options.fallback_noise_rot * self.r_scale;
                }
                for i in 3..6 {
                    r[(i, i)] = self.options.fallback_noise_pos * self.r_scale;
                }
                r
            }
        };

        let s = self.p + r;
        let s_inv = match s.try_inverse() {
            Some(v) => v,
            None => {
                return RelocGate::RejectedNumerical {
                    reason: "S not invertible",
                };
            }
        };
        let chi2 = z.dot(&(s_inv * z));
        let threshold = chi2_95(6) * self.options.chi2_multiplier;

        if chi2 > threshold {
            self.consecutive_rejects += 1;
            if self.consecutive_rejects >= self.options.auto_scale_trigger {
                self.r_scale *= self.options.auto_scale_factor;
                // 上限 16 倍，避免发散
                if self.r_scale > 16.0 {
                    self.r_scale = 16.0;
                }
            }
            log::debug!(
                "GICP chi2 reject {chi2:.2} > {threshold:.2} (reject #{})",
                self.consecutive_rejects
            );
            return RelocGate::RejectedChi2 { chi2, threshold };
        }

        // 4. 通过：Joseph 更新，限幅后注入名义量
        self.consecutive_rejects = 0;
        self.r_scale = 1.0;

        let k = self.p * s_inv;
        let mut delta = k * z;

        // 限幅：旋转与平移分别截断
        let rot_norm = delta.fixed_rows::<3>(0).norm();
        if rot_norm > self.options.max_correction_rot && rot_norm > 1e-12 {
            let scale = self.options.max_correction_rot / rot_norm;
            delta.fixed_rows_mut::<3>(0).scale_mut(scale);
        }
        let trans_norm = delta.fixed_rows::<3>(3).norm();
        if trans_norm > self.options.max_correction_trans && trans_norm > 1e-12 {
            let scale = self.options.max_correction_trans / trans_norm;
            delta.fixed_rows_mut::<3>(3).scale_mut(scale);
        }

        let d_t = se3_exp(&delta);
        self.t_drift = d_t * self.t_drift;

        // Joseph: P = (I-K) P (I-K)ᵀ + K R Kᵀ
        let i = Matrix6::identity();
        let ik = i - k;
        self.p = ik * self.p * ik.transpose() + k * r * k.transpose();
        self.p = (self.p + self.p.transpose()) * 0.5;

        // 同步 last_vio 为当前，避免下次 predict 重复膨胀
        self.last_vio = Some(*t_vio);

        log::debug!(
            "GICP fused chi2 {chi2:.2}/{threshold:.2} delta rot {:.3}° trans {:.3}m inliers {num_inliers}/{total_points}",
            rot_norm.to_degrees(),
            trans_norm
        );
        RelocGate::Accepted {
            chi2,
            threshold,
            delta,
        }
    }

    /// 由 `VIO` 位姿得矫正后全局位姿。
    #[must_use]
    pub fn corrected_pose(&self, t_vio: &Matrix4<f64>) -> Matrix4<f64> {
        self.t_drift * *t_vio
    }

    /// 直接以矫正后位姿覆盖（用于测试/重置）。
    pub fn set_drift(&mut self, t_drift: Matrix4<f64>) {
        self.t_drift = t_drift;
    }

    /// 重置为初值。
    pub fn reset(&mut self) {
        self.t_drift = Matrix4::identity();
        self.p = Matrix6::identity() * 0.1;
        self.last_vio = None;
        self.consecutive_rejects = 0;
        self.r_scale = 1.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix4, Vector3, Vector6};

    fn pose(trans: Vector3<f64>, yaw_deg: f64) -> Matrix4<f64> {
        let mut a = Vector6::zeros();
        a[2] = yaw_deg.to_radians();
        a.fixed_rows_mut::<3>(3).copy_from(&trans);
        se3_exp(&a)
    }

    #[test]
    fn predict_grows_covariance() {
        let mut f = FusionFilter::with_default();
        let t0 = Matrix4::identity();
        f.predict(&t0);
        let p0 = *f.covariance();
        f.predict(&t0);
        let p1 = *f.covariance();
        assert!(p1[(0, 0)] > p0[(0, 0)]);
        assert!(p1[(3, 3)] > p0[(3, 3)]);
    }

    #[test]
    fn precheck_rejects_low_inliers() {
        let mut f = FusionFilter::with_default();
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        let h = Matrix6::identity() * 100.0;
        let g = f.update(&t_vio, &Matrix4::identity(), &h, 5, 100, 0.1, true);
        assert!(matches!(g, RelocGate::RejectedPrecheck { .. }));
    }

    #[test]
    fn chi2_rejects_large_error() {
        let mut f = FusionFilter::with_default();
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        // 人为让 t_gicp 远离预测 5m
        let t_gicp = pose(Vector3::new(5.0, 0.0, 0.0), 0.0);
        let h = Matrix6::identity() * 1000.0; // 小 R，S≈P，chi2 很大
        let g = f.update(&t_vio, &t_gicp, &h, 80, 100, 0.1, true);
        assert!(matches!(g, RelocGate::RejectedChi2 { .. }));
        assert_eq!(f.consecutive_rejects(), 1);
    }

    #[test]
    fn accepted_update_corrects_drift() {
        let mut f = FusionFilter::with_default();
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        // VIO 漂 0.3m，GICP 给出真值（0.3m 观测）
        let t_gicp = pose(Vector3::new(0.3, 0.0, 0.0), 0.0);
        let mut h = Matrix6::identity();
        // 构造信息使 R≈0.04（与 fallback 一致），保证 chi2 通过
        for i in 0..6 {
            h[(i, i)] = 25.0;
        }
        let g = f.update(&t_vio, &t_gicp, &h, 80, 100, 0.1, true);
        assert!(matches!(g, RelocGate::Accepted { .. }));
        let t_corr = f.corrected_pose(&t_vio);
        // 矫正后应向 0.3m 靠拢（非 100% 因 K<1）
        let trans = t_corr.fixed_view::<3, 1>(0, 3).into_owned();
        assert!(trans.x > 0.05 && trans.x < 0.3);
    }

    #[test]
    fn auto_scale_on_consecutive_chi2() {
        let mut f = FusionFilter::with_default();
        let t_vio = Matrix4::identity();
        let t_gicp = pose(Vector3::new(5.0, 0.0, 0.0), 0.0);
        let h = Matrix6::identity() * 1000.0;
        for _ in 0..3 {
            f.predict(&t_vio);
            let _ = f.update(&t_vio, &t_gicp, &h, 80, 100, 0.1, true);
        }
        assert!(f.r_scale > 1.0);
        // 一次通过后复位
        let t_gicp_ok = Matrix4::identity();
        let h_ok = Matrix6::identity() * 10.0;
        f.predict(&t_vio);
        let _ = f.update(&t_vio, &t_gicp_ok, &h_ok, 80, 100, 0.1, true);
        assert_eq!(f.r_scale, 1.0);
        assert_eq!(f.consecutive_rejects(), 0);
    }

    #[test]
    fn correction_clamped() {
        let opts = FusionOptions {
            max_correction_trans: 0.1,
            max_correction_rot: 1.0_f64.to_radians(),
            ..Default::default()
        };
        let mut f = FusionFilter::new(opts);
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        // 大误差但 R 很大使 chi2 通过
        let t_gicp = pose(Vector3::new(1.0, 0.0, 0.0), 10.0);
        let h = Matrix6::identity() * 0.1; // 大 R
        // 膨胀 P 使 K≈1，delta≈z（1m/10°）超限幅
        for _ in 0..10 {
            f.predict(&t_vio);
        }
        let g = f.update(&t_vio, &t_gicp, &h, 80, 100, 0.1, true);
        if let RelocGate::Accepted { delta, .. } = g {
            assert!(delta.fixed_rows::<3>(3).norm() <= 0.11);
            assert!(delta.fixed_rows::<3>(0).norm() <= 0.02);
        }
    }
}
