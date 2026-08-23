//! 零速更新（对照 `OpenVINS` `ov_msckf/update/UpdaterZeroVelocity.cpp/.h`）。
//!
//! [`UpdaterZeroVelocity::try_update`]：当 IMU 测量表明系统静止（角速度与
//! 比力残差接近零）且图像视差足够小时，用零速约束更新状态——等效于在
//! 相机时刻做一次"零位移"观测，抑制漂移。
//!
//! 裁剪（对照 C++ 默认分支）：`integrated_accel_constraint = false`、
//! `model_time_varying_bias = true`、`override_with_disparity_check = true`、
//! `explicitly_enforce_zero_motion = false`；其余分支标注 TODO。

use firefly_vio_core::feat::FeatureDatabase;
use firefly_vio_core::noise::ImuNoise;
use firefly_vio_core::propagation::Propagator;
use firefly_vio_types::quat_ops::skew_x;
use firefly_vio_types::var::Variable;
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

use crate::options::UpdaterOptions;
use crate::state::State;
use crate::state_helper::{ekf_propagation, ekf_update, get_marginal_covariance};
use crate::updater::chi2_95;
use crate::updater_helper::measurement_compress_inplace;

/// 计算两时刻间的特征平均视差（对照 `FeatureHelper::compute_disparity`）。
///
/// 返回 `(平均视差, 方差, 特征数)`；无共同特征时返回 `None`。
#[must_use]
pub fn compute_disparity(
    db: &mut FeatureDatabase,
    time0: f64,
    time1: f64,
) -> Option<(f64, f64, usize)> {
    let feats0 = db.features_containing(time0, false, true);
    let mut disparities: Vec<f64> = Vec::new();
    for feat in &feats0 {
        for (cam_id, times) in &feat.timestamps {
            // 该相机在 time0/time1 都有测量（精确相等；对照 C++ 的 find）
            let idx0 = times.iter().position(|t| t.total_cmp(&time0).is_eq())?;
            let idx1 = times.iter().position(|t| t.total_cmp(&time1).is_eq())?;
            let uv0 = feat.uvs[cam_id][idx0];
            let uv1 = feat.uvs[cam_id][idx1];
            let d = (uv1 - uv0).norm();
            disparities.push(f64::from(d));
        }
    }
    if disparities.is_empty() {
        return None;
    }
    let n = disparities.len() as f64;
    let mean = disparities.iter().sum::<f64>() / n;
    let var = disparities
        .iter()
        .map(|d| (d - mean) * (d - mean))
        .sum::<f64>()
        / n;
    Some((mean, var, disparities.len()))
}

/// 零速更新器（对照 `UpdaterZeroVelocity`）。
#[derive(Debug, Clone)]
pub struct UpdaterZeroVelocity {
    /// 更新选项（chi2 乘子）。
    pub options: UpdaterOptions,
    /// IMU 噪声（白化残差用）。
    pub noises: ImuNoise,
    /// 速度上限：超过即拒绝（`zupt_max_velocity`）。
    pub zupt_max_velocity: f64,
    /// 噪声放大乘子（`zupt_noise_multiplier`）。
    pub zupt_noise_multiplier: f64,
    /// 视差上限（`zupt_max_disparity`）。
    pub zupt_max_disparity: f64,
    /// 积分加速度约束开关（对照 C++ `integrated_accel_constraint`）。
    pub integrated_accel_constraint: bool,
    /// 上次 ZUPT 的状态时刻（清理特征用）。
    last_zupt_state_timestamp: f64,
    /// 连续 ZUPT 计数（对照 `last_zupt_count`）。
    last_zupt_count: usize,
    /// 上次传播时间偏移（对照 `last_prop_time_offset`）。
    last_prop_time_offset: f64,
    /// 是否已设置时间偏移。
    have_last_prop_time_offset: bool,
}

impl UpdaterZeroVelocity {
    /// 构造（对照 `UpdaterZeroVelocity` 构造函数）。
    #[must_use]
    pub fn new(
        options: UpdaterOptions,
        noises: ImuNoise,
        zupt_max_velocity: f64,
        zupt_noise_multiplier: f64,
        zupt_max_disparity: f64,
        integrated_accel_constraint: bool,
    ) -> Self {
        Self {
            options,
            noises,
            zupt_max_velocity,
            zupt_noise_multiplier,
            zupt_max_disparity,
            integrated_accel_constraint,
            last_zupt_state_timestamp: 0.0,
            last_zupt_count: 0,
            last_prop_time_offset: 0.0,
            have_last_prop_time_offset: false,
        }
    }

    /// 尝试零速更新（对照 `UpdaterZeroVelocity::try_update`）。
    ///
    /// 返回 `true` 表示接受了 ZUPT（状态时间被推进到 `timestamp`，且不会
    /// 在该时刻增广克隆——调用方需据此跳过传播/克隆逻辑）。
    // 与 C++ 1:1 移植的长流程函数，拆分会破坏对照可审计性。
    #[allow(clippy::too_many_lines)]
    pub fn try_update(
        &mut self,
        state: &mut State,
        timestamp: f64,
        db: &mut FeatureDatabase,
        prop: &Propagator,
    ) -> bool {
        // 无 IMU 数据或状态已到目标时刻 → 拒绝
        if prop.imu_data_len() == 0 {
            self.last_zupt_state_timestamp = 0.0;
            return false;
        }
        if state.timestamp.total_cmp(&timestamp).is_eq() {
            self.last_zupt_state_timestamp = 0.0;
            return false;
        }
        if !self.have_last_prop_time_offset {
            self.last_prop_time_offset = state
                .calib_dt_cam_to_imu
                .as_ref()
                .map_or(0.0, |dt| dt.vec()[0]);
            self.have_last_prop_time_offset = true;
        }

        let t_off_new = state
            .calib_dt_cam_to_imu
            .as_ref()
            .map_or(0.0, |dt| dt.vec()[0]);
        let time0 = state.timestamp + self.last_prop_time_offset;
        let time1 = timestamp + t_off_new;
        let imu_data = prop.imu_data_snapshot();
        let Some(imu_recent) = Propagator::select_imu_readings(&imu_data, time0, time1, true)
        else {
            self.last_zupt_state_timestamp = 0.0;
            return false;
        };
        self.last_prop_time_offset = t_off_new;
        if imu_recent.len() < 2 {
            log::warn!("[ZUPT] 无足够 IMU 数据检查零速");
            self.last_zupt_state_timestamp = 0.0;
            return false;
        }

        // 状态顺序：[q_GtoI, bg, ba, (v)]（integrated_accel_constraint 开启
        // 时含速度，12 维）
        let mut hx_order = vec![
            (state.imu.pose().q().id(), 3),
            (state.imu.bg().id(), 3),
            (state.imu.ba().id(), 3),
        ];
        if self.integrated_accel_constraint {
            hx_order.push((state.imu.v().id(), 3));
        }
        let h_size = if self.integrated_accel_constraint {
            12
        } else {
            9
        };
        let m_size = 6 * (imu_recent.len() - 1);
        let mut h = DMatrix::<f64>::zeros(m_size, h_size);
        let mut res = DVector::<f64>::zeros(m_size);

        let calib = state.imu_calibration();
        let r_gtoi_jacob = if state.options.do_fej {
            state.imu.pose().rot_fej()
        } else {
            state.imu.pose().rot()
        };
        let gravity = Vector3::new(0.0, 0.0, 9.81);

        let mut dt_summed = 0.0f64;
        for i in 0..imu_recent.len() - 1 {
            let dt = imu_recent[i + 1].timestamp - imu_recent[i].timestamp;
            let am = imu_recent[i].am;
            let wm = imu_recent[i].wm;
            let a_hat = calib.r_acc_to_imu * calib.da * (am - calib.bias_a);
            let w_hat = calib.r_gyro_to_imu * calib.dw * (wm - calib.bias_g - calib.tg * a_hat);

            // 白化（对照 C++ 的 w_omega/w_accel；integrated 用 w_accel_v）
            let w_omega = dt.sqrt() / self.noises.sigma_w;
            let w_accel = dt.sqrt() / self.noises.sigma_a;
            let w_accel_v = 1.0 / (dt.sqrt() * self.noises.sigma_a);

            // 残差（真值为零；integrated 时加速度约束为 v − g·dt + Rᵀ·a·dt）
            res.rows_range_mut(6 * i..6 * i + 3)
                .copy_from(&(-w_omega * w_hat));
            if self.integrated_accel_constraint {
                let vel = state.imu.vel();
                res.rows_range_mut(6 * i + 3..6 * i + 6).copy_from(
                    &(-w_accel_v * (vel - gravity * dt + r_gtoi_jacob.transpose() * a_hat * dt)),
                );
            } else {
                res.rows_range_mut(6 * i + 3..6 * i + 6)
                    .copy_from(&(-w_accel * (a_hat - r_gtoi_jacob * gravity)));
            }

            // 雅可比（对照 C++ 的 H.block 三式）
            h.view_mut((6 * i, 3), (3, 3))
                .copy_from(&(-w_omega * Matrix3::identity()));
            if self.integrated_accel_constraint {
                h.view_mut((6 * i + 3, 0), (3, 3))
                    .copy_from(&(-w_accel_v * r_gtoi_jacob.transpose() * skew_x(&a_hat) * dt));
                h.view_mut((6 * i + 3, 6), (3, 3))
                    .copy_from(&(-w_accel_v * r_gtoi_jacob.transpose() * dt));
                h.view_mut((6 * i + 3, 9), (3, 3))
                    .copy_from(&(w_accel_v * Matrix3::identity()));
            } else {
                h.view_mut((6 * i + 3, 0), (3, 3))
                    .copy_from(&(-w_accel * skew_x(&(r_gtoi_jacob * gravity))));
                h.view_mut((6 * i + 3, 6), (3, 3))
                    .copy_from(&(-w_accel * Matrix3::identity()));
            }
            dt_summed += dt;
        }

        // 压缩（超定系统）
        measurement_compress_inplace(&mut h, &mut res);
        if h.nrows() < 1 {
            return false;
        }

        // 噪声放大（避免过自信）
        let r = self.zupt_noise_multiplier * DMatrix::<f64>::identity(res.len(), res.len());

        // bias 随时间演化（G·Qd·Gᵀ = dt·Qc）
        let mut q_bias = DMatrix::<f64>::identity(6, 6);
        q_bias
            .view_mut((0, 0), (3, 3))
            .copy_from(&(dt_summed * self.noises.sigma_wb_2 * Matrix3::identity()));
        q_bias
            .view_mut((3, 3), (3, 3))
            .copy_from(&(dt_summed * self.noises.sigma_ab_2 * Matrix3::identity()));

        // chi2 检验（含 bias 演化噪声）
        let mut p_marg = get_marginal_covariance(state, &hx_order);
        let bias_block =
            p_marg.view((3, 3), (6, 6)).into_owned() + q_bias.view((0, 0), (6, 6)).into_owned();
        p_marg.view_mut((3, 3), (6, 6)).copy_from(&bias_block);
        let s = &h * p_marg * h.transpose() + &r;
        let chi2 = match s.clone().cholesky() {
            Some(chol) => res.dot(&chol.solve(&res)),
            None => f64::INFINITY,
        };
        let chi2_check = chi2_95(res.len());

        // 视差检查（对照 C++ 的 override_with_disparity_check）
        let mut disparity_passed = false;
        if let Some((disp_avg, _, num_features)) = compute_disparity(db, state.timestamp, timestamp)
        {
            disparity_passed = disp_avg < self.zupt_max_disparity && num_features > 20;
        }

        // 拒绝条件（对照 C++：视差不过 + (chi2 超限 或 速度超限)）
        let vel = state.imu.vel().norm();
        if !disparity_passed
            && (chi2 > self.options.chi2_multipler * chi2_check || vel > self.zupt_max_velocity)
        {
            self.last_zupt_state_timestamp = 0.0;
            self.last_zupt_count = 0;
            log::debug!(
                "[ZUPT] 拒绝 |v|={vel:.3} chi2={chi2:.3} > {:.3}",
                self.options.chi2_multipler * chi2_check
            );
            return false;
        }
        log::info!("[ZUPT] 接受 |v|={vel:.3} chi2={chi2:.3}");

        // 连续 ZUPT 后清理旧时刻特征（对照 C++：不在此刻增广克隆）
        if self.last_zupt_count >= 2 {
            db.cleanup_measurements_exact(self.last_zupt_state_timestamp);
        }

        // bias 传播（model_time_varying_bias）
        let phi_bias = DMatrix::<f64>::identity(6, 6);
        let bias_order = [(state.imu.bg().id(), 3), (state.imu.ba().id(), 3)];
        ekf_propagation(state, &bias_order, &bias_order, &phi_bias, &q_bias);

        // ZUPT 更新（对照 C++：EKFUpdate + timestamp 推进）
        ekf_update(state, &hx_order, &h, &res, &r, f64::INFINITY);
        state.timestamp = timestamp;

        self.last_zupt_state_timestamp = timestamp;
        self.last_zupt_count += 1;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::StateOptions;
    use crate::state::State;
    use crate::state_helper::augment_clone;
    use firefly_vio_core::sensor::ImuData;

    fn zupt() -> UpdaterZeroVelocity {
        UpdaterZeroVelocity::new(
            UpdaterOptions::default(),
            ImuNoise::default(),
            1.0,
            1.0,
            1.0,
            false,
        )
    }

    fn state_with_zero_motion() -> State {
        let mut s = State::new(StateOptions::default());
        s.imu.set_value(
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        s
    }

    #[test]
    fn zupt_accepts_zero_motion() {
        let mut s = state_with_zero_motion();
        s.timestamp = 1.0;
        augment_clone(&mut s, &Vector3::zeros());
        let prop = Propagator::new(ImuNoise::default());
        // 静止 IMU：零角速度 + 比力 = +g
        for i in 0..11 {
            prop.feed_imu(
                ImuData {
                    timestamp: 1.0 + 0.02 * f64::from(i),
                    wm: Vector3::zeros(),
                    am: Vector3::new(0.0, 0.0, 9.81),
                },
                -1.0,
            );
        }
        let mut db = FeatureDatabase::new();
        let mut u = zupt();
        // 无特征 → 视差检查失败（num_features=0）→ 视差不过；
        // 但速度 0 + chi2 小 → 仍接受（视差不过时用 chi2/速度判据）
        let ok = u.try_update(&mut s, 1.2, &mut db, &prop);
        assert!(ok, "静止 ZUPT 应接受");
        assert!((s.timestamp - 1.2).abs() < 1e-12);
    }

    #[test]
    fn zupt_integrated_accel_accepts_zero_motion() {
        // 开启积分加速度约束：静止场景仍应接受（v − g·dt + Rᵀ·a·dt ≈ 0）
        let mut s = state_with_zero_motion();
        s.timestamp = 1.0;
        augment_clone(&mut s, &Vector3::zeros());
        let prop = Propagator::new(ImuNoise::default());
        for i in 0..11 {
            prop.feed_imu(
                ImuData {
                    timestamp: 1.0 + 0.02 * f64::from(i),
                    wm: Vector3::zeros(),
                    am: Vector3::new(0.0, 0.0, 9.81),
                },
                -1.0,
            );
        }
        let mut db = FeatureDatabase::new();
        let mut u = UpdaterZeroVelocity::new(
            UpdaterOptions::default(),
            ImuNoise::default(),
            1.0,
            1.0,
            1.0,
            true,
        );
        let ok = u.try_update(&mut s, 1.2, &mut db, &prop);
        assert!(ok, "integrated_accel 静止 ZUPT 应接受");
    }

    #[test]
    fn zupt_rejects_moving() {
        let mut s = state_with_zero_motion();
        s.timestamp = 1.0;
        let prop = Propagator::new(ImuNoise::default());
        // 明显运动：恒定角速度 + 加速度
        for i in 0..11 {
            prop.feed_imu(
                ImuData {
                    timestamp: 1.0 + 0.02 * f64::from(i),
                    wm: Vector3::new(1.0, 2.0, 3.0),
                    am: Vector3::new(0.0, 0.0, 0.0),
                },
                -1.0,
            );
        }
        let mut db = FeatureDatabase::new();
        let mut u = zupt();
        let ok = u.try_update(&mut s, 1.2, &mut db, &prop);
        assert!(!ok, "运动 ZUPT 应拒绝");
    }
}
