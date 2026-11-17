//! 误差态 EKF：`VIO→全局` 漂移 `SE(3)` 的预测-更新。
//!
//! 名义量 `D = T_map_odom`，右误差定义为 `D_true = D Exp(e_odom)`。
//! 观测 `D_obs = T_map_body T_odom_body⁻¹`，残差 `Log(D⁻¹ D_obs)`。
//! body 右扰动协方差经 `Ad(T_odom_body)` 换基；Joseph 更新后按右雅可比重置。

use firefly_base::se3::{right_jacobian, se3_exp, se3_log};
use firefly_base::{FrameId, RigidTransform};
use nalgebra::{Matrix4, Matrix6, Vector6};

use firefly_base::stats::chi2_95;

/// 新息门决定接受资格，修正限幅限制单步注入量。
#[derive(Debug, Clone, Copy, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct GateProfile {
    /// 新息限幅：平移 `m`（超过视为误锁拒收）。
    pub max_innovation_trans: f64,
    /// 新息限幅：旋转 `rad`。
    pub max_innovation_rot: f64,
    /// 单次修正限幅：平移 `m`（接受后注入名义量的钳制，防单步跳变）。
    pub max_correction_trans: f64,
    /// 单次修正限幅：旋转 `rad`。
    pub max_correction_rot: f64,
}

impl GateProfile {
    #[must_use]
    pub fn tight() -> Self {
        Self {
            max_innovation_trans: 0.3,
            max_innovation_rot: 5.0_f64.to_radians(),
            max_correction_trans: 0.5,
            max_correction_rot: 5.0_f64.to_radians(),
        }
    }

    /// 松门用于已通过 RANSAC、重投影误差和先验几何验证的观测；此处仅做
    /// VINS `20m/30°` 新息检查。接受后应用完整后验，限幅只拦截超限修正。
    #[must_use]
    pub fn loose() -> Self {
        Self {
            max_innovation_trans: 20.0,
            max_innovation_rot: 30.0_f64.to_radians(),
            max_correction_trans: 20.0,
            max_correction_rot: 30.0_f64.to_radians(),
        }
    }
}

impl Default for GateProfile {
    fn default() -> Self {
        Self::tight()
    }
}

#[derive(Debug, Clone, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FusionOptions {
    /// 视觉观测门控；部署配置可使用经过特征验证后的宽门限。
    pub visual: GateProfile,
    /// 单步过程噪声：旋转 `rad²`（每 odom 步，100Hz 下按秒 ≈1e-5）。
    pub process_noise_rot: f64,
    /// 单步过程噪声：平移 `m²`（每 odom 步，100Hz 下按秒 ≈1e-3；覆盖悬停
    /// 随机游走）。
    pub process_noise_pos: f64,
    /// 里程过程噪声：平移 `m²/m`，每行驶 1 米增加的方差；悬停时此项为零。
    pub process_noise_pos_per_m: f64,
    /// 观测噪声回退：旋转 `rad²`（`h` 不可逆时）。
    pub fallback_noise_rot: f64,
    /// 观测噪声回退：平移 `m²`。
    pub fallback_noise_pos: f64,
    /// `chi2` 阈值乘子（对照 `UpdaterOptions::chi2_multipler`；取 1 即
    /// `chi2_95` 原教旨名义 5% 误拒率，乘子下调等价于收紧置信度）。
    pub chi2_multiplier: f64,
    pub min_inlier_ratio: f64,
    pub min_num_inliers: usize,
    /// 最大观测误差；视觉观测为平均重投影误差，单位像素。
    pub max_registration_error: f64,
    /// 观测噪声下限：旋转 `rad²`（`R=h⁻¹` 失配时防过自信；离线标定：
    /// 好修正旋转噪声中位 ~1°，下限取 `(1°)²≈3e-4`）。
    pub r_floor_rot: f64,
    /// 观测噪声下限：平移 `m²`（好修正噪声中位 ~0.07m，下限取 `5e-3`）。
    pub r_floor_pos: f64,
    /// `P` 初值：旋转不确定度，单位 `rad²`，默认 `(3°)²`。
    pub p_init_rot: f64,
    /// `P` 初值：局部平移的不确定度，单位 `m²`，默认 `(0.1m)²`。
    pub p_init_pos: f64,
    /// 平移协方差下限，单位 `m²`，预测后钳制；为未建模系统误差保留余量。
    /// 观测仍须通过创新门控，旋转协方差不设此下限。
    pub p_min_pos: f64,
    /// 连续拒收后 `R` 的放大倍数。
    pub auto_scale_factor: f64,
    /// 触发放大的连续拒收次数。
    pub auto_scale_trigger: usize,
}

impl Default for FusionOptions {
    fn default() -> Self {
        Self {
            process_noise_rot: 1e-7,
            process_noise_pos: 1e-5,
            process_noise_pos_per_m: 0.04,
            fallback_noise_rot: (2.5_f64.to_radians()).powi(2),
            fallback_noise_pos: 0.2_f64.powi(2),
            chi2_multiplier: 1.0,
            min_inlier_ratio: 0.3,
            min_num_inliers: 30,
            max_registration_error: 1e9,
            visual: GateProfile::tight(),
            r_floor_rot: 3.0e-4,
            r_floor_pos: 5.0e-3,
            p_init_rot: 2.7e-3,
            p_init_pos: 1.0e-2,
            p_min_pos: 0.25,
            auto_scale_factor: 2.0,
            auto_scale_trigger: 3,
        }
    }
}

#[derive(Debug, Clone)]
pub enum RelocGate {
    Accepted {
        chi2: f64,
        threshold: f64,
        delta: Vector6<f64>,
    },
    /// 观测质量拒绝（收敛/内点/误差）。
    RejectedPrecheck {
        reason: &'static str,
    },
    RejectedChi2 {
        chi2: f64,
        threshold: f64,
    },
    /// 新息超限（局部跟踪档：平移/旋转跳变过大，疑似别名误锁）。
    RejectedInnovation {
        trans: f64,
        rot_deg: f64,
    },
    /// 数值异常（`S` 不可逆等）。
    RejectedNumerical {
        reason: &'static str,
    },
}

/// 单次视觉位姿观测：地图位姿、body 右扰动信息与质量字段。
/// 两个更新入口共用 `FusionOptions::visual` 门控。
#[derive(Debug, Clone, Copy)]
pub struct Observation {
    /// 全局位姿观测 `T_target_source`（`target=全局地图`）。
    pub t_global: Matrix4<f64>,
    /// body 右扰动信息矩阵，顺序 `[rot, trans]`；观测噪声 `R = h⁻¹`。
    pub h: Matrix6<f64>,
    pub num_inliers: usize,
    /// 总点数（内点率分母；未知传 0 跳过比率门）。
    pub total_points: usize,
    /// 平均重投影误差，单位像素。
    pub error: f64,
    pub converged: bool,
}

#[derive(Debug, Clone)]
pub struct FusionFilter {
    /// `T_drift = T_map_body · T_odom_body⁻¹`；初始值由部署先验指定。
    t_drift: Matrix4<f64>,
    initial_drift: Matrix4<f64>,
    /// odom 系右误差协方差 `6×6`，顺序 `[rot, trans]`。
    p: Matrix6<f64>,
    /// 仅由 predict 推进；延迟观测更新不得回拨里程基准。
    last_vio: Option<Matrix4<f64>>,
    options: FusionOptions,
    consecutive_rejects: usize,
    r_scale: f64,
}

impl FusionFilter {
    #[must_use]
    pub fn new(options: FusionOptions) -> Self {
        let mut p = Matrix6::zeros();
        for i in 0..3 {
            p[(i, i)] = options.p_init_rot;
        }
        for i in 3..6 {
            p[(i, i)] = options.p_init_pos;
        }
        Self {
            t_drift: Matrix4::identity(),
            initial_drift: Matrix4::identity(),
            p,
            last_vio: None,
            options,
            consecutive_rejects: 0,
            r_scale: 1.0,
        }
    }

    #[must_use]
    pub fn with_default() -> Self {
        Self::new(FusionOptions::default())
    }

    #[must_use]
    pub fn drift(&self) -> &Matrix4<f64> {
        &self.t_drift
    }

    #[must_use]
    pub fn covariance(&self) -> &Matrix6<f64> {
        &self.p
    }

    #[must_use]
    pub const fn consecutive_rejects(&self) -> usize {
        self.consecutive_rejects
    }

    /// 由 `VIO` 位姿预测：仅膨胀 `P`，不改 `T_drift`。平移协方差按时间与行驶
    /// 距离扩散；旋转协方差只按时间扩散。
    #[fastrace::trace]
    pub fn predict(&mut self, t_vio: &Matrix4<f64>) {
        let dist = if let Some(last) = &self.last_vio {
            let d = t_vio.fixed_view::<3, 1>(0, 3) - last.fixed_view::<3, 1>(0, 3);
            let dist = d.norm();
            if dist.is_finite() { dist } else { 0.0 }
        } else {
            self.last_vio = Some(*t_vio);
            return;
        };
        let mut q = Matrix6::zeros();
        for i in 0..3 {
            q[(i, i)] = self.options.process_noise_rot;
        }
        for i in 3..6 {
            q[(i, i)] =
                self.options.process_noise_pos + self.options.process_noise_pos_per_m * dist;
        }
        self.p += q;
        self.p = (self.p + self.p.transpose()) * 0.5;
        for i in 3..6 {
            if self.p[(i, i)] < self.options.p_min_pos {
                self.p[(i, i)] = self.options.p_min_pos;
            }
        }
        self.last_vio = Some(*t_vio);
    }

    /// 由 body 右扰动信息矩阵与地图位姿更新，使用视觉门控。
    #[allow(clippy::too_many_lines)]
    #[fastrace::trace]
    pub fn update(&mut self, t_vio: &Matrix4<f64>, obs: &Observation) -> RelocGate {
        let profile = self.options.visual;
        self.update_inner(t_vio, obs, &profile)
    }

    #[allow(clippy::too_many_lines)]
    #[fastrace::trace]
    fn update_inner(
        &mut self,
        t_vio: &Matrix4<f64>,
        obs: &Observation,
        profile: &GateProfile,
    ) -> RelocGate {
        if !obs.converged {
            return RelocGate::RejectedPrecheck {
                reason: "not converged",
            };
        }
        if obs.num_inliers < self.options.min_num_inliers {
            return RelocGate::RejectedPrecheck {
                reason: "too few inliers",
            };
        }
        if obs.total_points > 0 {
            let ratio = obs.num_inliers as f64 / obs.total_points as f64;
            if ratio < self.options.min_inlier_ratio {
                return RelocGate::RejectedPrecheck {
                    reason: "low inlier ratio",
                };
            }
        }
        if !obs.error.is_finite() || obs.error > self.options.max_registration_error {
            return RelocGate::RejectedPrecheck {
                reason: "large error",
            };
        }

        let Ok(vio) = RigidTransform::from_matrix(FrameId::ODOM, FrameId::BODY, t_vio) else {
            return RelocGate::RejectedNumerical {
                reason: "invalid odometry pose",
            };
        };
        let Ok(observed) = RigidTransform::from_matrix(FrameId::MAP, FrameId::BODY, &obs.t_global)
        else {
            return RelocGate::RejectedNumerical {
                reason: "invalid observation pose",
            };
        };
        let Ok(drift) = RigidTransform::from_matrix(FrameId::MAP, FrameId::ODOM, &self.t_drift)
        else {
            return RelocGate::RejectedNumerical {
                reason: "invalid map alignment",
            };
        };
        let observed_drift = observed
            .compose(&vio.inverse())
            .expect("map-body-odom chain");
        let error = drift.inverse().matrix() * observed_drift.matrix();
        let z = se3_log(&error);

        let innov_rot = z.fixed_rows::<3>(0).norm();
        let innov_trans = z.fixed_rows::<3>(3).norm();
        if innov_trans > profile.max_innovation_trans || innov_rot > profile.max_innovation_rot {
            return RelocGate::RejectedInnovation {
                trans: innov_trans,
                rot_deg: innov_rot.to_degrees(),
            };
        }

        // 信息矩阵描述观测位姿的 body 右扰动；必须为有限对称半正定矩阵。
        if !obs.h.iter().all(|x| x.is_finite())
            || (obs.h - obs.h.transpose()).norm() > 1e-6 * obs.h.norm().max(1.0)
        {
            return RelocGate::RejectedNumerical {
                reason: "invalid observation information",
            };
        }
        let info = (obs.h + obs.h.transpose()) * 0.5;
        if info.symmetric_eigen().eigenvalues.min() < -1e-9 {
            return RelocGate::RejectedNumerical {
                reason: "indefinite observation information",
            };
        }
        let mut r = if let Some(chol) = info.cholesky() {
            chol.inverse() * self.r_scale
        } else {
            let mut fallback = Matrix6::zeros();
            for i in 0..6 {
                fallback[(i, i)] = if i < 3 {
                    self.options.fallback_noise_rot
                } else {
                    self.options.fallback_noise_pos
                };
            }
            fallback * self.r_scale
        };

        for i in 0..3 {
            if r[(i, i)] < self.options.r_floor_rot {
                r[(i, i)] = self.options.r_floor_rot;
            }
        }
        for i in 3..6 {
            if r[(i, i)] < self.options.r_floor_pos {
                r[(i, i)] = self.options.r_floor_pos;
            }
        }

        r = vio.tangent_covariance(&r);
        let s = self.p + r;
        let Some(s_chol) = s.cholesky() else {
            return RelocGate::RejectedNumerical {
                reason: "S not invertible",
            };
        };
        let s_inv = s_chol.inverse();
        let chi2 = z.dot(&(s_inv * z));
        if !chi2.is_finite() {
            return RelocGate::RejectedNumerical {
                reason: "non-finite innovation",
            };
        }
        let threshold = chi2_95(6) * self.options.chi2_multiplier;

        if chi2 > threshold {
            self.consecutive_rejects += 1;
            if self.consecutive_rejects >= self.options.auto_scale_trigger {
                self.r_scale *= self.options.auto_scale_factor;
                if self.r_scale > 16.0 {
                    self.r_scale = 16.0;
                }
            }
            log::debug!(
                "视觉定位 chi2 reject {chi2:.2} > {threshold:.2} (reject #{})",
                self.consecutive_rejects
            );
            return RelocGate::RejectedChi2 { chi2, threshold };
        }

        self.consecutive_rejects = 0;
        self.r_scale = 1.0;

        let mut k = self.p * s_inv;
        let mut delta = k * z;

        let rot_norm = delta.fixed_rows::<3>(0).norm();
        if rot_norm > profile.max_correction_rot && rot_norm > 1e-12 {
            let scale = profile.max_correction_rot / rot_norm;
            delta.fixed_rows_mut::<3>(0).scale_mut(scale);
            k.fixed_rows_mut::<3>(0).scale_mut(scale);
        }
        let trans_norm = delta.fixed_rows::<3>(3).norm();
        if trans_norm > profile.max_correction_trans && trans_norm > 1e-12 {
            let scale = profile.max_correction_trans / trans_norm;
            delta.fixed_rows_mut::<3>(3).scale_mut(scale);
            k.fixed_rows_mut::<3>(3).scale_mut(scale);
        }

        let d_t = se3_exp(&delta);
        self.t_drift *= d_t;

        // Joseph: P = (I-K) P (I-K)ᵀ + K R Kᵀ
        let i = Matrix6::identity();
        let ik = i - k;
        let posterior = ik * self.p * ik.transpose() + k * r * k.transpose();
        let reset = right_jacobian(&delta);
        self.p = reset * posterior * reset.transpose();
        self.p = (self.p + self.p.transpose()) * 0.5;

        log::debug!(
            "视觉定位 fused chi2 {chi2:.2}/{threshold:.2} delta rot {:.3}° trans {:.3}m inliers {}/{}",
            rot_norm.to_degrees(),
            trans_norm,
            obs.num_inliers,
            obs.total_points,
        );
        RelocGate::Accepted {
            chi2,
            threshold,
            delta,
        }
    }

    /// 由 `Firefly/PoseObservation` 更新：位姿由 `pos/quat` 组装，
    /// 信息矩阵为 body 右扰动协方差的逆；使用视觉门控与统一更新内核。
    ///
    /// # Panics
    ///
    /// 观测协方差与其回退对角阵均不可逆时。
    #[fastrace::trace]
    pub fn update_with_observation(
        &mut self,
        t_vio: &Matrix4<f64>,
        obs: &firefly_pubsub::vision::PoseObservation,
    ) -> RelocGate {
        let q = nalgebra::Quaternion::new(obs.quat_w, obs.quat_x, obs.quat_y, obs.quat_z);
        let t_obs = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(obs.position_x, obs.position_y, obs.position_z),
            nalgebra::UnitQuaternion::from_quaternion(q),
        )
        .to_homogeneous();
        let r = Matrix6::from_row_slice(&obs.covariance);
        let h = r.try_inverse().unwrap_or_else(|| {
            let mut fb = Matrix6::zeros();
            for i in 0..3 {
                fb[(i, i)] = self.options.fallback_noise_rot;
            }
            for i in 3..6 {
                fb[(i, i)] = self.options.fallback_noise_pos;
            }
            fb.try_inverse()
                .expect("fallback 对角信息阵必须可逆（fallback_noise_* 为正）")
        });
        let profile = self.options.visual;
        self.update_inner(
            t_vio,
            &Observation {
                t_global: t_obs,
                h,
                num_inliers: obs.num_inliers as usize,
                total_points: obs.total_points as usize,
                error: obs.error,
                converged: obs.converged,
            },
            &profile,
        )
    }

    #[must_use]
    pub fn corrected_pose(&self, t_vio: &Matrix4<f64>) -> Matrix4<f64> {
        self.t_drift * *t_vio
    }

    /// 设置已知启动先验 map←odom；不会读取仿真真值。
    /// # Errors
    /// 输入坐标系不是 map←odom。
    pub fn set_alignment(&mut self, transform: RigidTransform) -> firefly_error::Result<()> {
        if transform.target() != FrameId::MAP || transform.source() != FrameId::ODOM {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "expected map<-odom alignment",
            ));
        }
        self.initial_drift = transform.matrix();
        self.reset();
        Ok(())
    }

    pub fn reset(&mut self) {
        self.t_drift = self.initial_drift;
        let mut p = Matrix6::zeros();
        for i in 0..3 {
            p[(i, i)] = self.options.p_init_rot;
        }
        for i in 3..6 {
            p[(i, i)] = self.options.p_init_pos;
        }
        self.p = p;
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

    fn obs(t_global: &Matrix4<f64>, h: &Matrix6<f64>) -> Observation {
        Observation {
            t_global: *t_global,
            h: *h,
            num_inliers: 80,
            total_points: 100,
            error: 0.1,
            converged: true,
        }
    }

    #[test]
    fn world_translation_is_corrected_independently_of_body_heading() {
        for heading in [0.0, 90.0, -90.0, 170.0] {
            let mut f = FusionFilter::with_default();
            let vio = pose(Vector3::zeros(), heading);
            f.predict(&vio);
            let mut observed = vio;
            observed[(0, 3)] = 0.1;
            let gate = f.update(&vio, &obs(&observed, &(Matrix6::identity() * 1000.0)));
            assert!(matches!(gate, RelocGate::Accepted { .. }));
            let corrected = f.corrected_pose(&vio);
            assert!(corrected[(0, 3)] > 0.05);
            assert!(corrected[(1, 3)].abs() < 1e-10);
            assert!(
                (corrected.fixed_view::<3, 1>(0, 3) - observed.fixed_view::<3, 1>(0, 3)).norm()
                    < 0.05
            );
        }
    }

    #[test]
    fn changing_map_coordinates_preserves_filter_result() {
        let mut a = FusionFilter::with_default();
        let mut b = FusionFilter::with_default();
        let shift = RigidTransform::from_matrix(
            FrameId::MAP,
            FrameId::ODOM,
            &se3_exp(&Vector6::new(0.1, -0.2, 0.3, 5.0, -3.0, 2.0)),
        )
        .unwrap();
        b.set_alignment(shift).unwrap();
        let vio = pose(Vector3::new(1.0, 0.3, 0.2), 60.0);
        a.predict(&vio);
        b.predict(&vio);
        let observed = se3_exp(&Vector6::new(0.001, -0.002, 0.003, 0.02, -0.01, 0.03)) * vio;
        let info = Matrix6::from_diagonal(&Vector6::new(100.0, 200.0, 300.0, 400.0, 500.0, 600.0));
        assert!(matches!(
            a.update(&vio, &obs(&observed, &info)),
            RelocGate::Accepted { .. }
        ));
        assert!(matches!(
            b.update(&vio, &obs(&(shift.matrix() * observed), &info)),
            RelocGate::Accepted { .. }
        ));
        assert!((b.corrected_pose(&vio) - shift.matrix() * a.corrected_pose(&vio)).norm() < 1e-10);
        assert!((a.covariance() - b.covariance()).norm() < 1e-10);
        assert!(a.covariance().cholesky().is_some());
    }

    #[test]
    fn invalid_information_does_not_change_nominal_or_covariance() {
        let mut f = FusionFilter::with_default();
        let p = *f.covariance();
        for bad in [f64::NAN, -1.0] {
            let mut h = Matrix6::identity();
            h[(0, 0)] = bad;
            assert!(matches!(
                f.update(&Matrix4::identity(), &obs(&Matrix4::identity(), &h)),
                RelocGate::RejectedNumerical { .. }
            ));
            assert!((f.covariance() - p).norm() < 1e-12);
            assert!((f.drift() - Matrix4::identity()).norm() < 1e-12);
        }
    }

    #[test]
    fn predict_enforces_p_floor() {
        let mut f = FusionFilter::with_default();
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        let h_tight = Matrix6::identity() * 1000.0;
        let g = f.update(&t_vio, &obs(&Matrix4::identity(), &h_tight));
        assert!(matches!(g, RelocGate::Accepted { .. }));
        assert!(f.covariance()[(3, 3)] < 0.25);
        f.predict(&t_vio);
        assert!((f.covariance()[(3, 3)] - 0.25).abs() < 1e-9);
        assert!(f.covariance()[(0, 0)] < 0.01);
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
    fn predict_grows_with_distance() {
        let opts = FusionOptions {
            process_noise_pos: 0.0,
            process_noise_pos_per_m: 0.04,
            ..Default::default()
        };
        let mut f = FusionFilter::new(opts);
        f.predict(&Matrix4::identity());
        f.predict(&pose(Vector3::new(10.0, 0.0, 0.0), 0.0));
        let p = f.covariance()[(3, 3)];
        assert!((p - 0.41).abs() < 1e-9);
    }

    #[test]
    fn replay_far_end_hover_accepts() {
        use firefly_pubsub::vision::{OBS_SOURCE_VISUAL, PoseObservation};
        let opts = FusionOptions {
            visual: GateProfile::loose(),
            process_noise_pos: 0.0,
            process_noise_pos_per_m: 0.04,
            ..Default::default()
        };
        let mut f = FusionFilter::new(opts);
        f.predict(&Matrix4::identity());
        for k in 1..=200 {
            f.predict(&pose(Vector3::new(0.1 * f64::from(k), 0.0, 0.0), 0.0));
        }
        let mut cov = [0.0f64; 36];
        for i in 0..6 {
            cov[i * 6 + i] = 0.04;
        }
        let obs = PoseObservation {
            timestamp: 0.0,
            source: OBS_SOURCE_VISUAL,
            position_x: 22.3,
            position_y: 0.0,
            position_z: 0.0,
            quat_x: 0.0,
            quat_y: 0.0,
            quat_z: 0.0,
            quat_w: 1.0,
            covariance: cov,
            num_inliers: 500,
            total_points: 600,
            error: 0.1,
            converged: true,
        };
        let t_vio = pose(Vector3::new(20.0, 0.0, 0.0), 0.0);
        let g = f.update_with_observation(&t_vio, &obs);
        assert!(matches!(g, RelocGate::Accepted { .. }));
        let trans = f
            .corrected_pose(&t_vio)
            .fixed_view::<3, 1>(0, 3)
            .into_owned();
        assert!(trans.x > 22.1 && trans.x < 22.31);
    }

    #[test]
    fn precheck_rejects_low_inliers() {
        let mut f = FusionFilter::with_default();
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        let h = Matrix6::identity() * 100.0;
        let g = f.update(
            &t_vio,
            &Observation {
                num_inliers: 5,
                ..obs(&Matrix4::identity(), &h)
            },
        );
        assert!(matches!(g, RelocGate::RejectedPrecheck { .. }));
    }

    #[test]
    fn innovation_rejects_large_jump() {
        let mut f = FusionFilter::with_default();
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        let t_observed = pose(Vector3::new(5.0, 0.0, 0.0), 0.0);
        let h = Matrix6::identity() * 1000.0;
        let g = f.update(&t_vio, &obs(&t_observed, &h));
        assert!(matches!(g, RelocGate::RejectedInnovation { .. }));
    }

    /// 低 R 下限的测试配置：`R=h⁻¹` 不被下限托住、`P` 下限关闭，`chi2`
    /// 在新息门内可达（隔离 `chi2` 机制本身；`p_min` 行为另由专测覆盖）。
    fn low_floor_opts() -> FusionOptions {
        FusionOptions {
            r_floor_pos: 1e-4,
            r_floor_rot: 1e-5,
            p_min_pos: 0.0,
            ..Default::default()
        }
    }

    #[test]
    fn chi2_rejects_confident_but_wrong() {
        let mut f = FusionFilter::new(low_floor_opts());
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        let h_tight = Matrix6::identity() * 1000.0;
        let g0 = f.update(&t_vio, &obs(&Matrix4::identity(), &h_tight));
        assert!(matches!(g0, RelocGate::Accepted { .. }));
        f.predict(&t_vio);
        let t_observed = pose(Vector3::new(0.25, 0.0, 0.0), 0.0);
        let g = f.update(&t_vio, &obs(&t_observed, &h_tight));
        assert!(matches!(g, RelocGate::RejectedChi2 { .. }));
        assert_eq!(f.consecutive_rejects(), 1);
    }

    #[test]
    fn accepted_update_corrects_drift() {
        let mut f = FusionFilter::with_default();
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        let t_observed = pose(Vector3::new(0.3, 0.0, 0.0), 0.0);
        let mut h = Matrix6::identity();
        for i in 0..6 {
            h[(i, i)] = 25.0;
        }
        let g = f.update(&t_vio, &obs(&t_observed, &h));
        assert!(matches!(g, RelocGate::Accepted { .. }));
        let t_corr = f.corrected_pose(&t_vio);
        let trans = t_corr.fixed_view::<3, 1>(0, 3).into_owned();
        assert!(trans.x > 0.05 && trans.x < 0.3);
    }

    #[test]
    fn observation_update_matches_direct() {
        use firefly_pubsub::vision::{OBS_SOURCE_VISUAL, PoseObservation};
        let t_vio = Matrix4::identity();
        let mut cov = [0.0f64; 36];
        for i in 0..6 {
            cov[i * 6 + i] = 0.04;
        }
        let obs = PoseObservation {
            timestamp: 0.0,
            source: OBS_SOURCE_VISUAL,
            position_x: 0.3,
            position_y: 0.0,
            position_z: 0.0,
            quat_x: 0.0,
            quat_y: 0.0,
            quat_z: 0.0,
            quat_w: 1.0,
            covariance: cov,
            num_inliers: 80,
            total_points: 100,
            error: 0.1,
            converged: true,
        };
        let mut f = FusionFilter::with_default();
        f.predict(&t_vio);
        let g = f.update_with_observation(&t_vio, &obs);
        assert!(matches!(g, RelocGate::Accepted { .. }));
        let trans = f
            .corrected_pose(&t_vio)
            .fixed_view::<3, 1>(0, 3)
            .into_owned();
        assert!(trans.x > 0.05 && trans.x < 0.3);
    }

    /// 0.5m 偏差的镜像对比（chi2≈5 < 12.6 两边都过，只差新息画像）：
    /// 视觉松画像接受，几何紧画像拒收。
    #[test]
    fn visual_wide_gate_accepts_half_meter() {
        use firefly_pubsub::vision::{OBS_SOURCE_VISUAL, PoseObservation};
        let opts = FusionOptions {
            visual: GateProfile::loose(),
            ..Default::default()
        };
        let mut f = FusionFilter::new(opts);
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        let mut cov = [0.0f64; 36];
        for i in 0..6 {
            cov[i * 6 + i] = 0.04;
        }
        let obs = PoseObservation {
            timestamp: 0.0,
            source: OBS_SOURCE_VISUAL,
            position_x: 0.5,
            position_y: 0.0,
            position_z: 0.0,
            quat_x: 0.0,
            quat_y: 0.0,
            quat_z: 0.0,
            quat_w: 1.0,
            covariance: cov,
            num_inliers: 80,
            total_points: 100,
            error: 0.1,
            converged: true,
        };
        let g = f.update_with_observation(&t_vio, &obs);
        assert!(matches!(g, RelocGate::Accepted { .. }));
        let trans = f
            .corrected_pose(&t_vio)
            .fixed_view::<3, 1>(0, 3)
            .into_owned();
        assert!(trans.x > 0.0 && trans.x < 0.5);
    }

    #[test]
    fn default_tight_gate_rejects_half_meter() {
        let mut f = FusionFilter::with_default();
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        let t_observed = pose(Vector3::new(0.5, 0.0, 0.0), 0.0);
        let h = Matrix6::identity() * 25.0;
        let g = f.update(&t_vio, &obs(&t_observed, &h));
        assert!(matches!(g, RelocGate::RejectedInnovation { .. }));
        assert_eq!(f.consecutive_rejects(), 0);
    }

    #[test]
    fn auto_scale_on_consecutive_chi2() {
        let mut f = FusionFilter::new(low_floor_opts());
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        let h_tight = Matrix6::identity() * 1000.0;
        let g0 = f.update(&t_vio, &obs(&Matrix4::identity(), &h_tight));
        assert!(matches!(g0, RelocGate::Accepted { .. }));
        let t_observed = pose(Vector3::new(0.25, 0.0, 0.0), 0.0);
        for _ in 0..3 {
            f.predict(&t_vio);
            let _ = f.update(&t_vio, &obs(&t_observed, &h_tight));
        }
        assert!(f.r_scale > 1.0);
        let t_observed_ok = Matrix4::identity();
        let h_ok = Matrix6::identity() * 10.0;
        f.predict(&t_vio);
        let _ = f.update(&t_vio, &obs(&t_observed_ok, &h_ok));
        assert_eq!(f.r_scale, 1.0);
        assert_eq!(f.consecutive_rejects(), 0);
    }

    #[test]
    fn correction_clamped() {
        let visual = GateProfile {
            max_correction_trans: 0.1,
            max_correction_rot: 1.0_f64.to_radians(),
            ..GateProfile::tight()
        };
        let opts = FusionOptions {
            visual,
            ..Default::default()
        };
        let mut f = FusionFilter::new(opts);
        let t_vio = Matrix4::identity();
        f.predict(&t_vio);
        let t_observed = pose(Vector3::new(0.25, 0.0, 0.0), 4.0);
        let h = Matrix6::identity() * 1.0e6; // R→下限，K≈0.7/0.9
        let g = f.update(&t_vio, &obs(&t_observed, &h));
        if let RelocGate::Accepted { delta, .. } = g {
            assert!(delta.fixed_rows::<3>(3).norm() <= 0.11);
            assert!(delta.fixed_rows::<3>(0).norm() <= 0.02);
            assert!(delta.fixed_rows::<3>(3).norm() > 0.05);
        } else {
            panic!("expected accept, got {g:?}");
        }
    }
}
