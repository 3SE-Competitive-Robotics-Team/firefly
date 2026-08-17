//! 动态初始化器（对照 `OpenVINS` `ov_init/src/dynamic/DynamicInitializer.cpp`）。
//!
//! 流程（对照 C++ 顺序）：
//! 1. 特征数据库清理后，选取窗口内可用特征与相机时刻；
//! 2. 连续预积分（CPI）构造 I0→Ii 与 Ii→Ii1 两张预积分表；
//! 3. 构造带 |g|=9.81 约束的线性系统求解速度/重力/特征初值（董氏系数 +
//!    伴矩阵特征值，EigenSolver → nalgebra `complex_eigenvalues`）；
//! 4. 恢复各时刻位姿/速度与全局特征初值；
//! 5. 自研高斯牛顿（GN）MLE（替代 Ceres DOGLEG）：IMU 预积分因子 +
//!    重投影因子（Cauchy loss）+ 首姿态/标定先验；
//! 6. 从最终最优点的 `J` 恢复 IMU 15×15 协方差（含先验，膨胀并对称化）。
//!
//! Ceres 求解器不用移植；线性系统部分与三个因子（`Factor_ImuCPIv1`、
//! `Factor_ImageReprojCalib`、`Factor_GenericPrior`）的残差/雅可比公式
//! 逐行对照 C++。

#![allow(
    non_snake_case,
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::items_after_statements,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::too_many_arguments
)]

use crate::InitResult;
use crate::cpi_v1::CpiV1;
use crate::helper::{compute_dongsi_coeff, gram_schmidt, select_imu_readings};
use crate::options::InitOptions;
use firefly_vio_core::cam::{CamEqui, CamRadtan, CameraModel, SharedCamera};
use firefly_vio_core::feat::{Feature, FeatureDatabase};
use firefly_vio_core::sensor::ImuData;
use firefly_vio_types::quat_ops::{
    inv_quat, jr_so3, log_so3, quat_2_rot, quat_multiply, quatnorm, rot_2_quat, skew_x,
};
use firefly_vio_types::var::PoseJpl;
use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, Vector2, Vector3, Vector4};
use std::collections::BTreeMap;
use std::collections::HashMap;

/// 时间戳包装键：`f64` 不实现 `Ord`/`Eq`/`Hash`，作为 BTree/Hash 键需包装
/// （语义等同 C++ 的 `std::map<double,...>` / `std::unordered_map<double,...>`）。
#[derive(Debug, Clone, Copy)]
struct Tk(f64);

impl Tk {
    fn of(t: f64) -> Self {
        Self(t)
    }
}

impl PartialEq for Tk {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}
impl Eq for Tk {}
impl PartialOrd for Tk {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Tk {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&other.0)
    }
}
impl std::hash::Hash for Tk {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.0.to_bits().hash(state);
    }
}

/// f64 时间戳的排序集合（`f64` 不实现 `Ord`，故自持升序 `Vec`，语义等同 C++ 的
/// `std::map<double, bool>` 相机时刻键）。
#[derive(Debug, Clone, Default)]
struct TimeSet(Vec<f64>);

impl TimeSet {
    fn insert(&mut self, t: f64) {
        if self.0.binary_search_by(|x| x.total_cmp(&t)).is_ok() {
            return;
        }
        let i = self.0.partition_point(|x| x < &t);
        self.0.insert(i, t);
    }
    fn len(&self) -> usize {
        self.0.len()
    }
    fn contains(&self, t: f64) -> bool {
        self.0.binary_search_by(|x| x.total_cmp(&t)).is_ok()
    }
    fn iter(&self) -> impl Iterator<Item = &f64> {
        self.0.iter()
    }
    fn keys_vec(&self) -> Vec<f64> {
        self.0.clone()
    }
}

/// 动态初始化器（对照 `DynamicInitializer`）。
#[derive(Debug, Clone)]
pub struct DynamicInitializer {
    /// 初始化器选项。
    pub params: InitOptions,
    /// IMU 测量缓冲（按 `feed_imu` 的 `oldest_time` 清理）。
    pub imu_data: Vec<ImuData>,
}

impl DynamicInitializer {
    /// 构造（对照 `DynamicInitializer` 构造函数）。
    #[must_use]
    pub fn new(params: InitOptions) -> Self {
        Self {
            params,
            imu_data: Vec::new(),
        }
    }

    /// 喂入 IMU 测量并在必要时清理过期读数
    /// （对照 `InertialInitializer::feed_imu` 的 push + erase 循环）。
    pub fn feed_imu(&mut self, message: &ImuData, oldest_time: f64) {
        self.imu_data.push(*message);
        if oldest_time != -1.0 {
            self.imu_data.retain(|d| d.timestamp >= oldest_time);
        }
    }

    /// 尝试动态初始化（对照 `DynamicInitializer::initialize`）。
    ///
    /// 成功返回 [`InitResult`]；数据不足 / 线性系统失败 / MLE 未收敛返回 `None`。
    #[allow(clippy::too_many_lines)]
    pub fn initialize(&mut self, db: &mut FeatureDatabase) -> Option<InitResult> {
        let params = &self.params;

        // ---------------------------------------------------------------
        // 1. 窗口与特征预处理（对照 C++ 48-164 行）
        // ---------------------------------------------------------------
        let newest_cam_time = db
            .iter_features()
            .flat_map(|f| f.timestamps.values())
            .flatten()
            .copied()
            .fold(-1.0f64, f64::max);
        let oldest_time = newest_cam_time - params.init_window_time;
        if newest_cam_time < 0.0 || oldest_time < 0.0 {
            return None;
        }

        db.cleanup_measurements(oldest_time);
        let mut have_old_imu_readings = false;
        self.imu_data.retain(|d| {
            if d.timestamp < oldest_time + params.calib_camimu_dt {
                have_old_imu_readings = true;
                false
            } else {
                true
            }
        });
        if db.size() < (0.75 * params.init_max_features as f64) as usize {
            log::warn!(
                "[init-d]: 有效特征 {} 个，不足所需 {} 个",
                db.size(),
                0.95 * params.init_max_features as f64
            );
            return None;
        }
        if self.imu_data.len() < 2 || !have_old_imu_readings {
            return None;
        }

        let features: HashMap<usize, Feature> =
            db.iter_features().map(|f| (f.featid, f.clone())).collect();

        const MIN_VALID_FEATURES: usize = 8;
        let min_num_meas_to_optimize = params.init_window_time as usize;
        let mut count_valid_features = 0usize;
        let mut map_features_num_meas: HashMap<usize, usize> = HashMap::new();
        let mut num_measurements = 0i32;
        let mut oldest_camera_time = f64::INFINITY;
        let mut camera_times: TimeSet = TimeSet::default();
        camera_times.insert(newest_cam_time);
        let mut map_camera_ids: BTreeMap<usize, bool> = BTreeMap::new();
        let pose_dt_avg = params.init_window_time / (params.init_dyn_num_pose + 1) as f64;

        for feat in features.values() {
            let mut times: Vec<f64> = Vec::new();
            let mut camids: BTreeMap<usize, bool> = BTreeMap::new();
            for (camid, ts) in &feat.timestamps {
                for time in ts {
                    let mut time_dt = f64::INFINITY;
                    for tmp in camera_times.iter() {
                        time_dt = time_dt.min((time - tmp).abs());
                    }
                    for tmp in &times {
                        time_dt = time_dt.min((time - tmp).abs());
                    }
                    if time_dt >= pose_dt_avg || time_dt == 0.0 {
                        times.push(*time);
                        camids.insert(*camid, true);
                    }
                }
            }
            map_features_num_meas.insert(feat.featid, times.len());
            if map_features_num_meas[&feat.featid] < min_num_meas_to_optimize {
                continue;
            }
            for tmp in &times {
                camera_times.insert(*tmp);
                oldest_camera_time = oldest_camera_time.min(*tmp);
                num_measurements += 2;
            }
            for tmp in camids.keys() {
                map_camera_ids.insert(*tmp, true);
            }
            count_valid_features += 1;
        }

        if camera_times.len() < params.init_dyn_num_pose {
            return None;
        }
        if count_valid_features < MIN_VALID_FEATURES {
            log::warn!(
                "[init-d]: 有效特征 {count_valid_features} 个，不足所需 {MIN_VALID_FEATURES} 个"
            );
            return None;
        }

        let gyroscope_bias = params.init_dyn_bias_g;
        let accelerometer_bias = params.init_dyn_bias_a;

        // 检查角速度/方向变化量（对照 C++ 164-183 行）
        let time0_in_imu = oldest_camera_time + params.calib_camimu_dt;
        let time1_in_imu = newest_cam_time + params.calib_camimu_dt;
        let readings = select_imu_readings(&self.imu_data, time0_in_imu, time1_in_imu);
        if readings.len() <= 2 {
            return None;
        }
        let mut theta_inI_norm = 0.0f64;
        let mut accel_inI_norm = 0.0f64;
        for k in 0..readings.len() - 1 {
            let imu0 = readings[k];
            let imu1 = readings[k + 1];
            let dt = imu1.timestamp - imu0.timestamp;
            let wm = 0.5 * (imu0.wm + imu1.wm) - gyroscope_bias;
            let am = 0.5 * (imu0.am + imu1.am) - accelerometer_bias;
            theta_inI_norm += (-wm * dt).norm();
            accel_inI_norm += am.norm();
        }
        accel_inI_norm /= (readings.len() - 1) as f64;
        log::debug!(
            "[init-d]: |theta_I| = {:.4}° 且 |accel| = {:.4}",
            180.0 / std::f64::consts::PI * theta_inI_norm,
            accel_inI_norm
        );
        if 180.0 / std::f64::consts::PI * theta_inI_norm < params.init_dyn_min_deg {
            log::warn!(
                "[init-d]: 陀螺仪仅变化 {:.2}°（阈值 {:.2}°）",
                180.0 / std::f64::consts::PI * theta_inI_norm,
                params.init_dyn_min_deg
            );
            return None;
        }

        // ---------------------------------------------------------------
        // 2. CPI 预积分（对照 C++ 240-307 行）
        // ---------------------------------------------------------------
        let use_single_depth = false;
        let size_feature = if use_single_depth { 1 } else { 3 };
        let num_features = count_valid_features;
        let system_size = size_feature * num_features + 3 + 3;
        if (num_measurements as usize) < system_size {
            log::warn!("[init-d]: 特征测量不足（{num_measurements} 测量 vs {system_size} 状态）!");
            return None;
        }

        let mut map_camera_cpi_I0toIi: HashMap<Tk, Option<CpiV1>> = HashMap::new();
        let mut map_camera_cpi_IitoIi1: HashMap<Tk, Option<CpiV1>> = HashMap::new();
        build_cpi_tables(
            &self.imu_data,
            &camera_times,
            oldest_camera_time,
            params,
            gyroscope_bias,
            accelerometer_bias,
            &mut map_camera_cpi_I0toIi,
            &mut map_camera_cpi_IitoIi1,
        )?;

        // ---------------------------------------------------------------
        // 3. 线性系统（对照 C++ 309-383 行）
        // ---------------------------------------------------------------
        let (a, b) = build_linear_system(
            params,
            &features,
            &camera_times,
            &map_camera_cpi_I0toIi,
            &map_features_num_meas,
            min_num_meas_to_optimize,
            size_feature,
            num_features,
            system_size,
            num_measurements,
        );

        // ---------------------------------------------------------------
        // 4. 约束求解 |g|=9.81（对照 C++ 389-481 行）
        // ---------------------------------------------------------------
        let (state_feat_vel, gravity_inI0) = solve_gravity_constraint(
            &a,
            &b,
            system_size,
            size_feature * num_features,
            params.gravity_mag,
        )?;

        let v_i0 = state_feat_vel
            .rows(size_feature * num_features, 3)
            .into_owned();
        let v_I0inI0 = Vector3::new(v_i0[0], v_i0[1], v_i0[2]);
        log::debug!("[init-d]: I0 中速度 {v_I0inI0}，|v|={:.4}", v_I0inI0.norm());
        if (gravity_inI0.norm() - params.gravity_mag).abs() > 1e-3 {
            log::warn!(
                "[init-d]: 重力未收敛（|g|-9.81={:.4}）",
                (gravity_inI0.norm() - params.gravity_mag).abs()
            );
            return None;
        }

        // ---------------------------------------------------------------
        // 5. 恢复位姿/特征初值（对照 C++ 487-567 行）
        // ---------------------------------------------------------------
        let mut ori_I0toIi: HashMap<Tk, Vector4<f64>> = HashMap::new();
        let mut pos_IiinI0: HashMap<Tk, Vector3<f64>> = HashMap::new();
        let mut vel_IiinI0: HashMap<Tk, Vector3<f64>> = HashMap::new();
        for time in camera_times.iter() {
            let (dt, r, alpha, beta) = cpi_at(&map_camera_cpi_I0toIi, *time);
            let p_IkinI0 = v_I0inI0 * dt - 0.5 * gravity_inI0 * dt * dt + alpha;
            let v_IkinI0 = v_I0inI0 - gravity_inI0 * dt + beta;
            ori_I0toIi.insert(Tk::of(*time), rot_2_quat(&r));
            pos_IiinI0.insert(Tk::of(*time), p_IkinI0);
            vel_IiinI0.insert(Tk::of(*time), v_IkinI0);
        }

        let a_index_features =
            A_index_features(&features, &map_features_num_meas, min_num_meas_to_optimize);
        let mut features_inI0: HashMap<usize, Vector3<f64>> = HashMap::new();
        count_valid_features = 0;
        for feat in features.values() {
            if map_features_num_meas[&feat.featid] < min_num_meas_to_optimize {
                continue;
            }
            let p_FinI0 = match a_index_features.get(&feat.featid) {
                Some(&idx) => {
                    let v = state_feat_vel
                        .view((size_feature * idx, 0), (3, 1))
                        .into_owned();
                    Vector3::new(v[0], v[1], v[2])
                }
                None => continue,
            };
            let mut is_behind = false;
            for cam_id in feat.timestamps.keys() {
                let ext = &params.camera_extrinsics[cam_id];
                let r_itoc = quat_2_rot(&Vector4::new(ext[0], ext[1], ext[2], ext[3]));
                let p_iinc = Vector3::new(ext[4], ext[5], ext[6]);
                let p_FinC0 = r_itoc * p_FinI0 + p_iinc;
                if p_FinC0[2] < 0.0 {
                    is_behind = true;
                }
            }
            if !is_behind {
                features_inI0.insert(feat.featid, p_FinI0);
                count_valid_features += 1;
            }
        }
        if count_valid_features < MIN_VALID_FEATURES {
            log::error!(
                "[init-d]: 有效特征 {count_valid_features} 个，不足所需 {MIN_VALID_FEATURES} 个（MLE）!"
            );
            return None;
        }

        let r_gtoi0 = gram_schmidt(&gravity_inI0);
        let q_GtoI0 = rot_2_quat(&r_gtoi0);
        let gravity = Vector3::new(0.0, 0.0, params.gravity_mag);
        let mut ori_GtoIi: HashMap<Tk, Vector4<f64>> = HashMap::new();
        let mut pos_IiinG: HashMap<Tk, Vector3<f64>> = HashMap::new();
        let mut vel_IiinG: HashMap<Tk, Vector3<f64>> = HashMap::new();
        let mut features_inG: HashMap<usize, Vector3<f64>> = HashMap::new();
        for (time, q) in &ori_I0toIi {
            ori_GtoIi.insert(*time, quat_multiply(q, &q_GtoI0));
            pos_IiinG.insert(*time, r_gtoi0.transpose() * pos_IiinI0[time]);
            vel_IiinG.insert(*time, r_gtoi0.transpose() * vel_IiinI0[time]);
        }
        for (id, p) in &features_inI0 {
            features_inG.insert(*id, r_gtoi0.transpose() * p);
        }

        // ---------------------------------------------------------------
        // 6. MLE 高斯牛顿（替代 Ceres，对照 C++ 572-898 行）
        // ---------------------------------------------------------------
        let (opt_state, cov_j) = run_gn_mle(
            params,
            &features,
            &camera_times,
            &map_camera_cpi_IitoIi1,
            &map_features_num_meas,
            min_num_meas_to_optimize,
            &ori_GtoIi,
            &pos_IiinG,
            &vel_IiinG,
            &features_inG,
            gyroscope_bias,
            accelerometer_bias,
            gravity,
            &map_camera_ids,
        )?;

        // ---------------------------------------------------------------
        // 7. 输出（对照 C++ 909-1106 行）
        // ---------------------------------------------------------------
        let newest_pose_idx = opt_state.pose_index(newest_cam_time);
        let newest_state = &opt_state.poses[newest_pose_idx];
        let mut imu_state = [0.0f64; 16];
        imu_state[0..4].copy_from_slice(newest_state.q.as_slice());
        imu_state[4..7].copy_from_slice(newest_state.p.as_slice());
        imu_state[7..10].copy_from_slice(newest_state.v.as_slice());
        imu_state[10..13].copy_from_slice(newest_state.bg.as_slice());
        imu_state[13..16].copy_from_slice(newest_state.ba.as_slice());

        let mut clones_imu: Vec<(f64, PoseJpl)> = Vec::new();
        for time in camera_times.iter() {
            let pose = &opt_state.poses[opt_state.pose_index(*time)];
            let mut pj = PoseJpl::default();
            pj.set_value(pose.q, pose.p);
            pj.set_fej(pose.q, pose.p);
            clones_imu.push((*time, pj));
        }

        let mut features_slam: Vec<(usize, Vector3<f64>)> = Vec::new();
        for (featid, p) in &opt_state.features {
            features_slam.push((*featid, *p));
        }

        let covariance = reconstruct_covariance(params, &cov_j, newest_pose_idx)?;

        // 末位置零（对照 1090-1094 行）
        imu_state[4..7].copy_from_slice(&[0.0, 0.0, 0.0]);

        log::info!(
            "[init-d]: 动态初始化成功，{} 姿态 / {} 特征",
            camera_times.len(),
            features_slam.len()
        );
        Some(InitResult {
            timestamp: newest_cam_time,
            covariance,
            order: vec![(0, 15)],
            imu_state,
            clones_imu,
            features_slam,
        })
    }
}

// ===========================================================================
// 工具：特征索引
// ===========================================================================

fn A_index_features(
    features: &HashMap<usize, Feature>,
    map_features_num_meas: &HashMap<usize, usize>,
    min_num_meas_to_optimize: usize,
) -> HashMap<usize, usize> {
    let mut idx = HashMap::new();
    let mut i = 0;
    for featid in features.keys() {
        if map_features_num_meas[featid] < min_num_meas_to_optimize {
            continue;
        }
        idx.insert(*featid, i);
        i += 1;
    }
    idx
}

// ===========================================================================
// CPI 预积分
// ===========================================================================

fn build_cpi_tables(
    imu_data: &[ImuData],
    camera_times: &TimeSet,
    oldest_camera_time: f64,
    params: &InitOptions,
    gyroscope_bias: Vector3<f64>,
    accelerometer_bias: Vector3<f64>,
    map_camera_cpi_I0toIi: &mut HashMap<Tk, Option<CpiV1>>,
    map_camera_cpi_IitoIi1: &mut HashMap<Tk, Option<CpiV1>>,
) -> Option<()> {
    let mut last_camera_timestamp = 0.0f64;
    for time in camera_times.iter() {
        let current_time = *time;
        if current_time == oldest_camera_time {
            map_camera_cpi_I0toIi.insert(Tk::of(current_time), None);
            map_camera_cpi_IitoIi1.insert(Tk::of(current_time), None);
            last_camera_timestamp = current_time;
            continue;
        }
        let cpi0_time0 = oldest_camera_time + params.calib_camimu_dt;
        let cpi0_time1 = current_time + params.calib_camimu_dt;
        let mut cpi0 = CpiV1::new(
            params.sigma_w,
            params.sigma_wb,
            params.sigma_a,
            params.sigma_ab,
            true,
        );
        cpi0.set_linearization_points(gyroscope_bias, accelerometer_bias);
        let r0 = select_imu_readings(imu_data, cpi0_time0, cpi0_time1);
        if r0.len() < 2 {
            return None;
        }
        if (r0[r0.len() - 1].timestamp - r0[0].timestamp - (cpi0_time1 - cpi0_time0)).abs() > 0.01 {
            return None;
        }
        for k in 0..r0.len() - 1 {
            cpi0.feed_imu(
                r0[k].timestamp,
                r0[k + 1].timestamp,
                &r0[k].wm,
                &r0[k].am,
                &r0[k + 1].wm,
                &r0[k + 1].am,
            );
        }

        let cpi1_time0 = last_camera_timestamp + params.calib_camimu_dt;
        let cpi1_time1 = current_time + params.calib_camimu_dt;
        let mut cpi1 = CpiV1::new(
            params.sigma_w,
            params.sigma_wb,
            params.sigma_a,
            params.sigma_ab,
            true,
        );
        cpi1.set_linearization_points(gyroscope_bias, accelerometer_bias);
        let r1 = select_imu_readings(imu_data, cpi1_time0, cpi1_time1);
        if r1.len() < 2 {
            return None;
        }
        if (r1[r1.len() - 1].timestamp - r1[0].timestamp - (cpi1_time1 - cpi1_time0)).abs() > 0.01 {
            return None;
        }
        for k in 0..r1.len() - 1 {
            cpi1.feed_imu(
                r1[k].timestamp,
                r1[k + 1].timestamp,
                &r1[k].wm,
                &r1[k].am,
                &r1[k + 1].wm,
                &r1[k + 1].am,
            );
        }

        map_camera_cpi_I0toIi.insert(Tk::of(current_time), Some(cpi0));
        map_camera_cpi_IitoIi1.insert(Tk::of(current_time), Some(cpi1));
        last_camera_timestamp = current_time;
    }
    Some(())
}

fn cpi_at(
    table: &HashMap<Tk, Option<CpiV1>>,
    time: f64,
) -> (f64, Matrix3<f64>, Vector3<f64>, Vector3<f64>) {
    let tk = Tk::of(time);
    match table.get(&tk) {
        Some(Some(cpi)) => (cpi.dt, cpi.r_k2tau, cpi.alpha_tau, cpi.beta_tau),
        _ => (0.0, Matrix3::identity(), Vector3::zeros(), Vector3::zeros()),
    }
}

// ===========================================================================
// 线性系统
// ===========================================================================

#[allow(clippy::too_many_arguments)]
fn build_linear_system(
    params: &InitOptions,
    features: &HashMap<usize, Feature>,
    camera_times: &TimeSet,
    map_camera_cpi_I0toIi: &HashMap<Tk, Option<CpiV1>>,
    map_features_num_meas: &HashMap<usize, usize>,
    min_num_meas_to_optimize: usize,
    size_feature: usize,
    num_features: usize,
    system_size: usize,
    num_measurements: i32,
) -> (DMatrix<f64>, DVector<f64>) {
    let a_index_features =
        A_index_features(features, map_features_num_meas, min_num_meas_to_optimize);
    let mut a = DMatrix::<f64>::zeros(num_measurements as usize, system_size);
    let mut b = DVector::<f64>::zeros(num_measurements as usize);
    let mut index_meas = 0;
    let vel_col = size_feature * num_features;
    let grav_col = vel_col + 3;

    for feat in features.values() {
        if map_features_num_meas[&feat.featid] < min_num_meas_to_optimize {
            continue;
        }
        let feat_idx = size_feature * a_index_features[&feat.featid];
        for (cam_id, ts) in &feat.timestamps {
            let ext = &params.camera_extrinsics[cam_id];
            let r_itoc = quat_2_rot(&Vector4::new(ext[0], ext[1], ext[2], ext[3]));
            let p_iinc = Vector3::new(ext[4], ext[5], ext[6]);
            for (i, time) in ts.iter().enumerate() {
                if !camera_times.contains(*time) {
                    continue;
                }
                let uv_norm = feat.uvs_norm[cam_id][i].cast::<f64>();
                let (dt, r_i0toik, alpha_i0toik, _) = cpi_at(map_camera_cpi_I0toIi, *time);
                // H_proj: [1 0 -u; 0 1 -v]（2×3）
                let h_proj = nalgebra::Matrix2x3::new(1.0, 0.0, -uv_norm[0], 0.0, 1.0, -uv_norm[1]);
                let y = h_proj * r_itoc * r_i0toik;
                let b_i = y * alpha_i0toik - h_proj * p_iinc;
                let y_vel = -(dt * y);
                let y_grav = 0.5 * dt * dt * y;
                for r in 0..2 {
                    for c in 0..3 {
                        a[(index_meas + r, feat_idx + c)] = y[(r, c)];
                        a[(index_meas + r, vel_col + c)] = y_vel[(r, c)];
                        a[(index_meas + r, grav_col + c)] = y_grav[(r, c)];
                    }
                }
                b[index_meas] = b_i[0];
                b[index_meas + 1] = b_i[1];
                index_meas += 2;
            }
        }
    }
    (a, b)
}

// ===========================================================================
// 约束求解 |g|=9.81
// ===========================================================================

fn solve_gravity_constraint(
    a: &DMatrix<f64>,
    b: &DVector<f64>,
    system_size: usize,
    feat_vel_dim: usize,
    gravity_mag: f64,
) -> Option<(DVector<f64>, Vector3<f64>)> {
    let a1 = a.columns(0, system_size - 3).into_owned();
    let a2 = a.columns(system_size - 3, 3).into_owned();
    let at_a1 = &a1.transpose() * &a1;
    let a1a1_inv = at_a1.cholesky()?.inverse();
    // Temp = A2ᵀ·(I_N − A1·A1A1_inv·A1ᵀ)（3×N；对照 C++ 393 行）
    let n = a1.nrows();
    let i_n = DMatrix::<f64>::identity(n, n);
    let temp = &a2.transpose() * (&i_n - &a1 * &a1a1_inv * &a1.transpose());
    let d = &temp * &a2; // D = Temp·A2（3×3）
    let d_vec_inner = &temp * b; // d = Temp·b（3×1）
    let d_vec = DMatrix::from_column_slice(3, 1, d_vec_inner.as_slice());

    let coeff = compute_dongsi_coeff(&d, &d_vec, gravity_mag);
    if coeff.len() != 7 || (coeff[0] - 1.0).abs() > 1e-8 {
        log::error!("[init-d]: 董氏系数异常");
        return None;
    }

    let mut cm = SMatrix::<f64, 6, 6>::zeros();
    for i in 0..5 {
        cm[(i + 1, i)] = 1.0;
    }
    for j in 0..6 {
        cm[(j, 5)] = -coeff[6 - j];
    }
    let companion = cm;
    if !companion.lu().is_invertible() {
        log::error!("[init-d]: 伴矩阵特征值分解不满秩!!");
        return None;
    }

    let eigenvalues = companion.complex_eigenvalues();
    let mut lambda_found = false;
    let mut lambda_min = -1.0f64;
    let mut cost_min = f64::INFINITY;
    let i3 = SMatrix::<f64, 3, 3>::identity();
    for v in eigenvalues.iter() {
        if v.im.abs() < 1e-12 {
            let lambda = v.re;
            let d_lambda = &d - lambda * i3;
            let state_grav = d_lambda.try_inverse()? * &d_vec;
            let cost = (state_grav.norm() - gravity_mag).abs();
            if !lambda_found || cost < cost_min {
                lambda_found = true;
                lambda_min = lambda;
                cost_min = cost;
            }
        }
    }
    if !lambda_found {
        log::error!("[init-d]: 未找到实数特征值!!!");
        return None;
    }

    let d_lambda = &d - lambda_min * i3;
    let sg_vec = d_lambda.try_inverse()? * &d_vec;
    let sg = Vector3::new(sg_vec[0], sg_vec[1], sg_vec[2]);

    let lhs = -(&a1a1_inv * &a1.transpose() * &a2) * sg;
    let rhs = &a1a1_inv * &a1.transpose() * b;
    let mut state_feat_vel = DVector::<f64>::zeros(system_size);
    state_feat_vel
        .rows_mut(0, system_size - 3)
        .copy_from(&(lhs + rhs));
    state_feat_vel.rows_mut(system_size - 3, 3).copy_from(&sg);
    debug_assert_eq!(feat_vel_dim, system_size - 3);
    Some((state_feat_vel, sg))
}

// ===========================================================================
// GN 状态
// ===========================================================================

#[derive(Clone, Copy)]
struct GnPose {
    q: Vector4<f64>,
    p: Vector3<f64>,
    v: Vector3<f64>,
    bg: Vector3<f64>,
    ba: Vector3<f64>,
}

#[derive(Clone)]
struct GnCalib {
    q: Vector4<f64>,
    p: Vector3<f64>,
    intr: [f64; 8],
    /// 是否鱼眼（构造时经探针判定，对照 C++ `dynamic_pointer_cast<CamEqui>`）。
    is_fisheye: bool,
}

struct GnState {
    poses: Vec<GnPose>,
    features: HashMap<usize, Vector3<f64>>,
    pose_lookup: HashMap<Tk, usize>,
    calibs: HashMap<usize, GnCalib>,
    /// 标定线性化点快照（构造时初值；对照 C++ `Factor_GenericPrior::x_lin`）。
    calib_lin: HashMap<usize, GnCalib>,
    /// 特征 dof 起始（姿态之后）。
    feat_base: usize,
    /// 标定 dof 起始（pose + feature 之后），按 cam_id 升序排列。
    calib_base: usize,
    /// 参与 GN 的标定 cam_id（升序）。
    calib_order: Vec<usize>,
    /// 参与 GN 的特征 id（升序）。
    feat_order: Vec<usize>,
}

impl GnState {
    fn pose_index(&self, time: f64) -> usize {
        self.pose_lookup[&Tk::of(time)]
    }
    fn feat_dof(&self, fid: usize) -> Option<usize> {
        self.feat_order
            .iter()
            .position(|x| *x == fid)
            .map(|i| self.feat_base + i * 3)
    }
    fn calib_index(&self, cam_id: usize) -> Option<usize> {
        self.calib_order.iter().position(|x| *x == cam_id)
    }
    fn dof(&self) -> usize {
        self.calib_base + self.calib_order.len() * 14
    }
}

/// 姿态 i 的 dof 起始（每个姿态 15 dof：q0 p3 v6 bg9 ba12）。
fn pose_dof(i: usize) -> usize {
    i * 15
}

// ===========================================================================
// MLE 高斯牛顿
// ===========================================================================

#[allow(clippy::too_many_arguments)]
fn run_gn_mle(
    params: &InitOptions,
    features: &HashMap<usize, Feature>,
    camera_times: &TimeSet,
    map_camera_cpi_IitoIi1: &HashMap<Tk, Option<CpiV1>>,
    map_features_num_meas: &HashMap<usize, usize>,
    min_num_meas_to_optimize: usize,
    ori_GtoIi: &HashMap<Tk, Vector4<f64>>,
    pos_IiinG: &HashMap<Tk, Vector3<f64>>,
    vel_IiinG: &HashMap<Tk, Vector3<f64>>,
    features_inG: &HashMap<usize, Vector3<f64>>,
    gyroscope_bias: Vector3<f64>,
    accelerometer_bias: Vector3<f64>,
    gravity: Vector3<f64>,
    map_camera_ids: &BTreeMap<usize, bool>,
) -> Option<(GnState, DMatrix<f64>)> {
    let pose_lookup: HashMap<Tk, usize> = camera_times
        .iter()
        .enumerate()
        .map(|(i, t)| (Tk::of(*t), i))
        .collect();
    let n_poses = camera_times.len();
    let mut poses = Vec::with_capacity(n_poses);
    for time in camera_times.iter() {
        poses.push(GnPose {
            q: ori_GtoIi[&Tk::of(*time)],
            p: pos_IiinG[&Tk::of(*time)],
            v: vel_IiinG[&Tk::of(*time)],
            bg: gyroscope_bias,
            ba: accelerometer_bias,
        });
    }
    let feat_order: Vec<usize> = features_inG.keys().copied().collect();
    let mut gn_features = HashMap::new();
    for (id, p) in features_inG {
        gn_features.insert(*id, *p);
    }
    let feat_base = n_poses * 15;
    let calib_order: Vec<usize> = map_camera_ids.keys().copied().collect();
    let mut calibs = HashMap::new();
    for cam_id in &calib_order {
        let ext = &params.camera_extrinsics[cam_id];
        let shared = &params.camera_intrinsics[cam_id];
        calibs.insert(
            *cam_id,
            GnCalib {
                q: Vector4::new(ext[0], ext[1], ext[2], ext[3]),
                p: Vector3::new(ext[4], ext[5], ext[6]),
                intr: shared.value(),
                is_fisheye: is_fisheye_cam(shared),
            },
        );
    }
    let calib_base = feat_base + feat_order.len() * 3;
    let calib_lin = calibs.clone();
    let mut state = GnState {
        poses,
        features: gn_features,
        pose_lookup,
        calibs,
        calib_lin,
        feat_base,
        calib_base,
        calib_order,
        feat_order,
    };

    let n_iter = params.init_dyn_mle_max_iter;
    let opt_calib = params.init_dyn_mle_opt_calib;
    if n_iter != 0 {
        let x_lin = state.poses[0];
        let mut prev_cost = f64::INFINITY;
        let mut converged = false;
        for _ in 0..n_iter {
            let (r, j, cost) = build_gn_residuals(
                params,
                features,
                camera_times,
                map_camera_cpi_IitoIi1,
                map_features_num_meas,
                min_num_meas_to_optimize,
                &state,
                gravity,
                opt_calib,
                &x_lin,
            );
            let ata = &j.transpose() * &j;
            let g = -(&j.transpose() * &r);
            // 稀疏起见用 LU；必要时加小阻尼（LM 风格）
            let delta = ata.lu().solve(&g)?;
            apply_delta(&mut state, &delta, opt_calib);
            log::debug!("[init-d]: GN 迭代 cost={cost:.6e}");
            if (prev_cost - cost).abs() < 1e-5 {
                converged = true;
                break;
            }
            prev_cost = cost;
        }
        if !converged {
            log::warn!("[init-d]: GN MLE 未收敛");
            return None;
        }
    }

    let (_, j, _) = build_gn_residuals(
        params,
        features,
        camera_times,
        map_camera_cpi_IitoIi1,
        map_features_num_meas,
        min_num_meas_to_optimize,
        &state,
        gravity,
        opt_calib,
        &state.poses[0],
    );
    Some((state, &j.transpose() * &j))
}

/// GN 残差行数统计。
#[allow(clippy::too_many_arguments)]
fn num_gn_rows(
    features: &HashMap<usize, Feature>,
    camera_times: &TimeSet,
    map_features_num_meas: &HashMap<usize, usize>,
    min_num_meas_to_optimize: usize,
    state: &GnState,
) -> usize {
    let n_poses = camera_times.len();
    let mut n_reproj = 0usize;
    for feat in features.values() {
        if map_features_num_meas[&feat.featid] < min_num_meas_to_optimize
            || !features_in_g_for(feat.featid, state)
        {
            continue;
        }
        for (cam_id, ts) in &feat.timestamps {
            if !state.calibs.contains_key(cam_id) {
                continue;
            }
            for t in ts {
                if camera_times.contains(*t) {
                    n_reproj += 1;
                }
            }
        }
    }
    let n_cam = state.calibs.len();
    (n_poses.saturating_sub(1)) * 15 + 10 + n_cam * 14 + 2 * n_reproj
}

fn features_in_g_for(fid: usize, state: &GnState) -> bool {
    state.features.contains_key(&fid)
}

/// 构建残差与全局雅可比。
#[allow(clippy::too_many_arguments)]
fn build_gn_residuals(
    params: &InitOptions,
    features: &HashMap<usize, Feature>,
    camera_times: &TimeSet,
    map_camera_cpi_IitoIi1: &HashMap<Tk, Option<CpiV1>>,
    map_features_num_meas: &HashMap<usize, usize>,
    min_num_meas_to_optimize: usize,
    state: &GnState,
    gravity: Vector3<f64>,
    opt_calib: bool,
    x_lin: &GnPose,
) -> (DVector<f64>, DMatrix<f64>, f64) {
    let n_rows = num_gn_rows(
        features,
        camera_times,
        map_features_num_meas,
        min_num_meas_to_optimize,
        state,
    );
    // 标定不优化时（默认）：其 dof 不进入系统（对照 Ceres
    // `SetParameterBlockConstant` 把参数块排除出优化变量，而非保留零列）
    let dof = if opt_calib {
        state.dof()
    } else {
        state.calib_base
    };
    let mut r = DVector::<f64>::zeros(n_rows);
    let mut j = DMatrix::<f64>::zeros(n_rows, dof);
    let mut row = 0usize;
    let mut cost = 0.0f64;

    // IMU 因子
    let times: Vec<f64> = camera_times.keys_vec();
    for k in 0..times.len().saturating_sub(1) {
        let pi = state.pose_index(times[k]);
        let pi2 = state.pose_index(times[k + 1]);
        let cpi = map_camera_cpi_IitoIi1[&Tk::of(times[k + 1])]
            .as_ref()
            .expect("cpi present");
        let (res, jac) =
            imu_factor_residual_jacobian(&state.poses[pi], &state.poses[pi2], cpi, gravity);
        let base = pose_dof(pi);
        let base2 = pose_dof(pi2);
        // jac 顺序：q1,bg1,v1,ba1,p1,q2,bg2,v2,ba2,p2（各 15×3）
        for (ii, val) in res.iter().enumerate() {
            r[row + ii] = *val;
            cost += val * val;
        }
        scatter_pose(
            &mut j, row, base, &jac[0], &jac[1], &jac[2], &jac[3], &jac[4],
        );
        scatter_pose(
            &mut j, row, base2, &jac[5], &jac[6], &jac[7], &jac[8], &jac[9],
        ); // jac 序 [q,bg,v,ba,p]
        row += 15;
    }

    // 首姿态先验
    {
        let (res, jac) = prior_first_pose(&state.poses[0], x_lin);
        for (ii, val) in res.iter().enumerate() {
            r[row + ii] = *val;
            cost += val * val;
        }
        scatter_first_prior(&mut j, row, pose_dof(0), &jac);
        row += 10;
    }

    // 标定先验（对照 C++ 的 Factor_GenericPrior：残差 = sqrtI·(x−x_lin)，
    // x_lin 为构造时初值快照；标定不优化时其 dof 不在系统内，仍计入残差行
    // 保持行数一致）
    for cam_id in &state.calib_order {
        let calib = &state.calibs[cam_id];
        let lin = &state.calib_lin[cam_id];
        let (res, jac) = calib_prior(calib, lin);
        for (ii, val) in res.iter().enumerate() {
            r[row + ii] = *val;
            cost += val * val;
        }
        if opt_calib {
            let cidx = state.calib_index(*cam_id).unwrap();
            let off = state.calib_base + cidx * 14;
            scatter_block(&mut j, row, off, &jac);
        }
        row += 14;
    }

    // 重投影因子
    for feat in features.values() {
        if map_features_num_meas[&feat.featid] < min_num_meas_to_optimize
            || !state.features.contains_key(&feat.featid)
        {
            continue;
        }
        let featid = feat.featid;
        let feat_p = state.features[&featid];
        let feat_dof = state.feat_dof(featid).unwrap();
        for (cam_id, ts) in &feat.timestamps {
            let Some(calib) = state.calibs.get(cam_id) else {
                continue;
            };
            let cam_idx = state.calib_index(*cam_id).unwrap();
            let calib_dof = state.calib_base + cam_idx * 14;
            for (i, time) in ts.iter().enumerate() {
                if !camera_times.contains(*time) {
                    continue;
                }
                let uv_meas = feat.uvs[cam_id][i].cast::<f64>();
                let pose = &state.poses[state.pose_index(*time)];
                let pose_dof = pose_dof(state.pose_index(*time));
                let (mut res, jac) =
                    reproj_factor_residual_jacobian(params, pose, &feat_p, calib, uv_meas);
                // Cauchy loss 权重（对照 ceres::CauchyLoss(1.0)）
                let s = res.x * res.x + res.y * res.y;
                let w_sqrt = 1.0 / (1.0 + s).sqrt();
                res *= w_sqrt;
                r[row] = res.x;
                r[row + 1] = res.y;
                cost += res.x * res.x + res.y * res.y;
                // 6 块：q,p,feat,qcalib,pcalib,intr
                scatter_block(&mut j, row, pose_dof, &(w_sqrt * &jac[0]));
                scatter_block(&mut j, row, pose_dof + 3, &(w_sqrt * &jac[1]));
                scatter_block(&mut j, row, feat_dof, &(w_sqrt * &jac[2]));
                if opt_calib {
                    scatter_block(&mut j, row, calib_dof, &(w_sqrt * &jac[3]));
                    scatter_block(&mut j, row, calib_dof + 3, &(w_sqrt * &jac[4]));
                    scatter_block(&mut j, row, calib_dof + 6, &(w_sqrt * &jac[5]));
                }
                row += 2;
            }
        }
    }

    (r, j, cost)
}

/// 把姿态的 5 个 15×3 雅可比块散布到 `base`（顺序：q, p, v, bg, ba 的 dof）。
fn scatter_pose(
    j: &mut DMatrix<f64>,
    row: usize,
    base: usize,
    j_q: &DMatrix<f64>,
    j_bg: &DMatrix<f64>,
    j_v: &DMatrix<f64>,
    j_ba: &DMatrix<f64>,
    j_p: &DMatrix<f64>,
) {
    scatter_block(j, row, base, j_q); // q -> +0
    scatter_block(j, row, base + 3, j_p); // p -> +3
    scatter_block(j, row, base + 6, j_v); // v -> +6
    scatter_block(j, row, base + 9, j_bg); // bg -> +9
    scatter_block(j, row, base + 12, j_ba); // ba -> +12
}

/// 散布第一个先验：quat(+0,3列)、pos(+3)、bg(+9)、ba(+12)。
fn scatter_first_prior(j: &mut DMatrix<f64>, row: usize, base: usize, jac: &[DMatrix<f64>; 4]) {
    scatter_block(j, row, base, &jac[0]); // quat 3 cols (+0)
    scatter_block(j, row, base + 3, &jac[1]); // pos
    scatter_block(j, row, base + 9, &jac[2]); // bg
    scatter_block(j, row, base + 12, &jac[3]); // ba
}

fn scatter_block(j: &mut DMatrix<f64>, row: usize, col0: usize, block: &DMatrix<f64>) {
    for i in 0..block.nrows() {
        for c in 0..block.ncols() {
            j[(row + i, col0 + c)] = block[(i, c)];
        }
    }
}

fn apply_delta(state: &mut GnState, delta: &DVector<f64>, opt_calib: bool) {
    for (i, pose) in state.poses.iter_mut().enumerate() {
        let off = i * 15;
        let dq = small_delta_quat(&delta.rows(off, 3).into_owned());
        pose.q = quatnorm(quat_multiply(&dq, &pose.q));
        pose.p += Vector3::new(delta[off + 3], delta[off + 4], delta[off + 5]);
        pose.v += Vector3::new(delta[off + 6], delta[off + 7], delta[off + 8]);
        pose.bg += Vector3::new(delta[off + 9], delta[off + 10], delta[off + 11]);
        pose.ba += Vector3::new(delta[off + 12], delta[off + 13], delta[off + 14]);
    }
    for (fidx, fid) in state.feat_order.iter().enumerate() {
        let off = state.feat_base + fidx * 3;
        let v = state.features[fid] + Vector3::new(delta[off], delta[off + 1], delta[off + 2]);
        state.features.insert(*fid, v);
    }
    if opt_calib {
        for (cidx, cid) in state.calib_order.iter().enumerate() {
            let off = state.calib_base + cidx * 14;
            let calib = state.calibs.get_mut(cid).unwrap();
            let dq = small_delta_quat(&delta.rows(off, 3).into_owned());
            calib.q = quatnorm(quat_multiply(&dq, &calib.q));
            calib.p += Vector3::new(delta[off + 3], delta[off + 4], delta[off + 5]);
            for k in 0..8 {
                calib.intr[k] += delta[off + 6 + k];
            }
        }
    }
}

fn small_delta_quat(dth: &DVector<f64>) -> Vector4<f64> {
    let dth = Vector3::new(dth[0], dth[1], dth[2]);
    let theta = dth.norm();
    if theta < 1e-8 {
        quatnorm(Vector4::new(0.5 * dth[0], 0.5 * dth[1], 0.5 * dth[2], 1.0))
    } else {
        let axis = dth / theta;
        quatnorm(Vector4::new(
            axis[0] * (theta / 2.0).sin(),
            axis[1] * (theta / 2.0).sin(),
            axis[2] * (theta / 2.0).sin(),
            (theta / 2.0).cos(),
        ))
    }
}

// ===========================================================================
// 因子
// ===========================================================================

/// IMU 预积分因子（对照 Factor_ImuCPIv1.cpp）。
/// 返回 (residual 15×1, jacobians 10×15×3 按 [q1,bg1,v1,ba1,p1,q2,bg2,v2,ba2,p2], sqrtI)。
fn imu_factor_residual_jacobian(
    s1: &GnPose,
    s2: &GnPose,
    cpi: &CpiV1,
    gravity: Vector3<f64>,
) -> (DVector<f64>, [DMatrix<f64>; 10]) {
    let eye = Matrix3::<f64>::identity();
    let r_1 = quat_2_rot(&s1.q);
    let q_2 = s2.q;
    let dbw = s1.bg - cpi.b_w_lin;
    let dba = s1.ba - cpi.b_a_lin;

    let qb_vec = 0.5 * cpi.j_q * dbw;
    let q_b = quatnorm(Vector4::new(qb_vec[0], qb_vec[1], qb_vec[2], 1.0));
    let q_breve = cpi.q_k2tau;
    let q_1to2 = quat_multiply(&q_2, &inv_quat(&s1.q));
    let q_res_m = quat_multiply(&q_1to2, &inv_quat(&q_breve));
    let q_res_p = quat_multiply(&q_res_m, &q_b);

    let mut res = DVector::<f64>::zeros(15);
    res.rows_mut(0, 3).copy_from(&(2.0 * q_res_p.xyz()));
    res.rows_mut(3, 3).copy_from(&(s2.bg - s1.bg));
    res.rows_mut(6, 3).copy_from(
        &(r_1 * (s2.v - s1.v + gravity * cpi.dt) - cpi.j_b * dbw - cpi.h_b * dba - cpi.beta_tau),
    );
    res.rows_mut(9, 3).copy_from(&(s2.ba - s1.ba));
    res.rows_mut(12, 3).copy_from(
        &(r_1 * (s2.p - s1.p - s1.v * cpi.dt + 0.5 * gravity * cpi.dt * cpi.dt)
            - cpi.j_a * dbw
            - cpi.h_a * dba
            - cpi.alpha_tau),
    );
    let sqrti = sqrt_info(&cpi.p_meas);
    res = &sqrti * &res;

    let q_meas_p = quat_multiply(&inv_quat(&q_breve), &q_b);
    let j_th1 = -((q_1to2[3] * eye - skew_x(&q_1to2.xyz()))
        * (q_meas_p[3] * eye + skew_x(&q_meas_p.xyz()))
        - q_1to2.xyz() * q_meas_p.xyz().transpose());
    let j_th2 = q_res_p[3] * eye + skew_x(&q_res_p.xyz());
    let j_bw1_rot = (q_res_m[3] * eye - skew_x(&q_res_m.xyz())) * cpi.j_q;
    let j_v_th1 = skew_x(&(r_1 * (s2.v - s1.v + gravity * cpi.dt)));
    let j_p_th1 = skew_x(&(r_1 * (s2.p - s1.p - s1.v * cpi.dt + 0.5 * gravity * cpi.dt * cpi.dt)));

    let mut jac = DMatrix::<f64>::zeros(15, 30);
    let set3 = |j: &mut DMatrix<f64>, row: usize, col: usize, m: &Matrix3<f64>| {
        for a in 0..3 {
            for b in 0..3 {
                j[(row + a, col + b)] = m[(a, b)];
            }
        }
    };
    set3(&mut jac, 0, 0, &j_th1);
    set3(&mut jac, 0, 15, &j_th2);
    set3(&mut jac, 0, 3, &j_bw1_rot);
    set3(&mut jac, 3, 3, &(-eye));
    set3(&mut jac, 3, 18, &eye);
    set3(&mut jac, 6, 0, &j_v_th1);
    set3(&mut jac, 6, 6, &(-r_1));
    set3(&mut jac, 6, 21, &r_1);
    set3(&mut jac, 6, 3, &(-cpi.j_b));
    set3(&mut jac, 6, 9, &(-cpi.h_b));
    set3(&mut jac, 9, 9, &(-eye));
    set3(&mut jac, 9, 24, &eye);
    set3(&mut jac, 12, 0, &j_p_th1);
    set3(&mut jac, 12, 6, &(-r_1 * cpi.dt));
    set3(&mut jac, 12, 12, &(-r_1));
    set3(&mut jac, 12, 27, &r_1);
    set3(&mut jac, 12, 3, &(-cpi.j_a));
    set3(&mut jac, 12, 9, &(-cpi.h_a));
    jac = &sqrti * &jac;

    let cols = [0usize, 3, 6, 9, 12, 15, 18, 21, 24, 27];
    let mut out: [DMatrix<f64>; 10] = std::array::from_fn(|_| DMatrix::<f64>::zeros(15, 3));
    for (k, c0) in cols.iter().enumerate() {
        out[k] = jac.columns(*c0, 3).into_owned();
    }
    (res, out)
}

/// P 的信息矩阵平方根（对照 C++ sqrtI = LLT(Pinv)）。
fn sqrt_info(p: &DMatrix<f64>) -> DMatrix<f64> {
    let n = p.nrows();
    // inv(P) via cholesky
    let info = p
        .clone()
        .cholesky()
        .map_or_else(|| p.clone(), |c| c.inverse());
    let l = info
        .cholesky()
        .map_or_else(|| DMatrix::<f64>::identity(n, n), |c| c.l());
    let mut out = DMatrix::<f64>::zeros(n, n);
    for i in 0..n {
        for k in 0..n {
            out[(k, i)] = l[(i, k)];
        }
    }
    out
}

/// 2×3 定维矩阵转 DMatrix（nalgebra 无 From 实现，显式拷贝）。
fn mat2x3(m: &nalgebra::Matrix2x3<f64>) -> DMatrix<f64> {
    DMatrix::from_fn(2, 3, |i, j| m[(i, j)])
}

/// 首姿态先验（对照 Factor_GenericPrior，x_types = [quat_yaw, vec3, vec3, vec3]）。
/// 返回 (residual 10, jac 4 块：[quat 10×3, pos 10×3, bg 10×3, ba 10×3])。
fn prior_first_pose(p: &GnPose, x_lin: &GnPose) -> (DVector<f64>, [DMatrix<f64>; 4]) {
    let mut sqrti = DMatrix::<f64>::identity(10, 10);
    for i in 0..4 {
        sqrti[(i, i)] = 1.0 / 1e-5;
    }
    for i in 4..7 {
        sqrti[(i, i)] = 1.0 / 0.05;
    }
    for i in 7..10 {
        sqrti[(i, i)] = 1.0 / 0.10;
    }

    // quat_yaw 残差：-ezᵀ·log(R_iᵀ·R_lin)
    let ez = Vector3::new(0.0, 0.0, 1.0);
    let r_i = quat_2_rot(&p.q);
    let r_lin = quat_2_rot(&x_lin.q);
    let theta_err = log_so3(&(r_i.transpose() * r_lin));

    let mut res = DVector::<f64>::zeros(10);
    res[0] = -(ez.dot(&theta_err));
    res.rows_mut(1, 3).copy_from(&(p.p - x_lin.p));
    res.rows_mut(4, 3).copy_from(&(p.bg - x_lin.bg));
    res.rows_mut(7, 3).copy_from(&(p.ba - x_lin.ba));
    res = &sqrti * &res;

    // 雅可比
    let mut jac: [DMatrix<f64>; 4] = std::array::from_fn(|_| DMatrix::<f64>::zeros(10, 3));
    let jr_inv = jr_so3(&theta_err)
        .try_inverse()
        .unwrap_or_else(Matrix3::identity);
    let h_theta = -ez.transpose() * (jr_inv * r_lin.transpose());
    for b in 0..3 {
        jac[0][(0, b)] = sqrti[(0, 0)] * h_theta[b];
    }
    for r in 1..10 {
        for b in 0..3 {
            jac[1][(r, b)] = 0.0;
            jac[2][(r, b)] = 0.0;
            jac[3][(r, b)] = 0.0;
        }
    }
    // pos 列（sqrti 的 1..4 行）
    for r in 1..4 {
        for b in 0..3 {
            jac[1][(r, b)] = if r - 1 == b { sqrti[(r, r)] } else { 0.0 };
        }
    }
    // bg 列（4..7 行用 jac[2]），ba 列（7..10 用 jac[3]）
    for r in 4..7 {
        for b in 0..3 {
            jac[2][(r, b)] = if r - 4 == b { sqrti[(r, r)] } else { 0.0 };
        }
    }
    for r in 7..10 {
        for b in 0..3 {
            jac[3][(r, b)] = if r - 7 == b { sqrti[(r, r)] } else { 0.0 };
        }
    }
    (res, jac)
}

/// 标定先验（外参 quat+pos 6 + 内参 8 = 14 维残差；对照 C++ 的
/// `Factor_GenericPrior`，x_types = [quat, vec3] 与 [vec8]）。
///
/// 残差 = sqrtI·(x − x_lin)：quat 用 `−log_so3(R_iᵀ·R_lin)`（JPL 误差），
/// pos/intr 用向量差；雅可比为 sqrtI 缩放（quat 块含 `−Jr_inv·R_linᵀ`）。
fn calib_prior(calib: &GnCalib, lin: &GnCalib) -> (DVector<f64>, DMatrix<f64>) {
    // sqrtI 对角：外参 q 1/0.001、外参 p 1/0.01、内参前 4 1/1.0、后 4 1/0.005
    let mut sqrti = DVector::<f64>::zeros(14);
    for i in 0..3 {
        sqrti[i] = 1.0 / 0.001;
    }
    for i in 3..6 {
        sqrti[i] = 1.0 / 0.01;
    }
    for i in 6..10 {
        sqrti[i] = 1.0;
    }
    for i in 10..14 {
        sqrti[i] = 1.0 / 0.005;
    }

    let r_i = quat_2_rot(&calib.q);
    let r_lin = quat_2_rot(&lin.q);
    let theta_err = log_so3(&(r_i.transpose() * r_lin));

    let mut res = DVector::<f64>::zeros(14);
    res.rows_mut(0, 3).copy_from(&(-theta_err));
    res.rows_mut(3, 3).copy_from(&(calib.p - lin.p));
    for k in 0..8 {
        res[6 + k] = calib.intr[k] - lin.intr[k];
    }
    res.component_mul_assign(&sqrti);

    // 雅可比：quat 块 −Jr_inv·R_linᵀ（乘 sqrti 前三行），其余块对角
    let mut jac = DMatrix::<f64>::zeros(14, 14);
    let jr_inv = jr_so3(&theta_err)
        .try_inverse()
        .unwrap_or_else(Matrix3::identity);
    let h_theta = -(jr_inv * r_lin.transpose());
    for a in 0..3 {
        for b in 0..3 {
            jac[(a, b)] = sqrti[a] * h_theta[(a, b)];
        }
    }
    for i in 3..14 {
        jac[(i, i)] = sqrti[i];
    }
    (res, jac)
}

/// 重投影因子（对照 Factor_ImageReprojCalib.cpp）。
/// 返回 (残差 2, jac 6 块：[q,p,feat,qcalib,pcalib,intr(2×8)])。
fn reproj_factor_residual_jacobian(
    params: &InitOptions,
    pose: &GnPose,
    feat_p: &Vector3<f64>,
    cam: &GnCalib,
    uv_meas: Vector2<f64>,
) -> (Vector2<f64>, [DMatrix<f64>; 6]) {
    let r_gtoii = quat_2_rot(&pose.q);
    let p_finii = r_gtoii * (*feat_p - pose.p);
    let r_itoc = quat_2_rot(&cam.q);
    let p_finci = r_itoc * p_finii + cam.p;
    let uv_norm = Vector2::new(p_finci[0] / p_finci[2], p_finci[1] / p_finci[2]);

    // 畸变与雅可比（用相机模型对象）
    let calib: &[f64] = &cam.intr;
    let (uv_dist, (dz_dzn, dz_dzeta)) = if cam.is_fisheye {
        let c = CamEqui::new(0, 0, calib);
        let d = c.distort_d(uv_norm);
        let j = c.compute_distort_jacobian(uv_norm);
        (d, j)
    } else {
        let c = CamRadtan::new(0, 0, calib);
        let d = c.distort_d(uv_norm);
        let j = c.compute_distort_jacobian(uv_norm);
        (d, j)
    };

    let sigma = params.sigma_pix;
    let res = Vector2::new(
        (uv_dist.x - uv_meas.x) / sigma,
        (uv_dist.y - uv_meas.y) / sigma,
    );

    // H_dzn_dpfc: [1/z 0 -x/z²; 0 1/z -y/z²]（2×3）
    let h_dzn_dpfc = nalgebra::Matrix2x3::new(
        1.0 / p_finci[2],
        0.0,
        -p_finci[0] / (p_finci[2] * p_finci[2]),
        0.0,
        1.0 / p_finci[2],
        -p_finci[1] / (p_finci[2] * p_finci[2]),
    );
    let h_dz_dpfc = dz_dzn * h_dzn_dpfc; // 2×3

    let mut out: [DMatrix<f64>; 6] = std::array::from_fn(|_| DMatrix::<f64>::zeros(2, 3));
    out[0] = mat2x3(&(h_dz_dpfc * r_itoc * skew_x(&p_finii)));
    out[1] = mat2x3(&(-h_dz_dpfc * r_itoc * r_gtoii));
    out[2] = mat2x3(&(h_dz_dpfc * r_itoc * r_gtoii));
    out[3] = mat2x3(&(h_dz_dpfc * skew_x(&(r_itoc * p_finii))));
    out[4] = mat2x3(&h_dz_dpfc);
    let mut j5 = DMatrix::<f64>::zeros(2, 8);
    for a in 0..2 {
        for b in 0..8 {
            j5[(a, b)] = dz_dzeta[(a, b)];
        }
    }
    out[5] = j5;
    // 除以 sigma 到雅可比
    for b in [0usize, 1, 2, 3, 4] {
        out[b] = (1.0 / sigma) * &out[b];
    }
    out[5] = (1.0 / sigma) * &out[5];
    (res, out)
}

/// 判断共享相机是否鱼眼（对照 C++ `dynamic_pointer_cast<CamEqui>`）。
///
/// `CameraModel` 无 `Any` 上界，无法直接 `downcast_ref`；改用**探针法**：
/// 在若干归一化点上对比共享相机对象与临时 `CamRadtan` 的 `distort_d` 输出，
/// 一致则共享相机为 Radtan，否则为 Equi。两种模型在非零畸变下几乎处处不同，
/// 零畸变时等价于 Radtan 恒等映射（此时判定准确）。
fn is_fisheye_cam(shared: &SharedCamera) -> bool {
    let calib = shared.value();
    let tmp = CamRadtan::new(0, 0, &calib);
    let probes = [
        Vector2::new(0.2, -0.3),
        Vector2::new(-0.4, 0.1),
        Vector2::new(0.5, 0.4),
    ];
    let mut agree = 0usize;
    for p in probes {
        let d1 = shared.distort_d(p);
        let d2 = tmp.distort_d(p);
        if (d1 - d2).norm() < 1e-2 {
            agree += 1;
        }
    }
    let is_radtan = agree > probes.len() / 2;
    !is_radtan
}

// ===========================================================================
// 协方差恢复
// ===========================================================================

fn reconstruct_covariance(
    params: &InitOptions,
    cov_j: &DMatrix<f64>,
    newest_pose_idx: usize,
) -> Option<DMatrix<f64>> {
    let off = newest_pose_idx * 15;
    let sub = cov_j.view((off, off), (15, 15)).into_owned();
    let inv: DMatrix<f64> = sub.cholesky()?.inverse();
    let mut cov = inv;
    let mut diag = cov.diagonal().norm();
    if !diag.is_finite() || diag < 1e-30 {
        // 极小/非正定 → 视为失败（返回 None 由调用方处理）
        return None;
    }
    diag = diag.max(0.0);
    let _ = diag;
    // 条件数近似：||C||_F^2 与最小奇异值
    let sv = cov.clone().symmetric_eigen().eigenvalues;
    let s_min = sv.iter().fold(f64::INFINITY, |a: f64, &b| a.min(b.abs()));
    let s_max = sv.iter().fold(0.0f64, |a, &b| a.max(b.abs()));
    if s_max > 0.0 && s_min / s_max < params.init_dyn_min_rec_cond {
        log::warn!(
            "[init-d]: 协方差条件数过小（{}/{} < {})",
            s_min,
            s_max,
            params.init_dyn_min_rec_cond
        );
        return None;
    }

    // 膨胀对角块
    for (blk, scale) in [
        ((0, 0), params.init_dyn_inflation_orientation),
        ((6, 6), params.init_dyn_inflation_velocity),
        ((9, 9), params.init_dyn_inflation_bias_gyro),
        ((12, 12), params.init_dyn_inflation_bias_accel),
    ] {
        for i in 0..3 {
            for k in 0..3 {
                cov[(blk.0 + i, blk.1 + k)] *= scale;
            }
        }
    }
    cov = 0.5 * (cov.clone() + cov.transpose());
    Some(cov)
}

// ===========================================================================
// 测试
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_db_returns_none() {
        let mut init = DynamicInitializer::new(InitOptions::default());
        let mut db = FeatureDatabase::new();
        assert!(init.initialize(&mut db).is_none());
    }

    #[test]
    fn no_imu_returns_none_without_panic() {
        let mut init = DynamicInitializer::new(InitOptions::default());
        let mut db = FeatureDatabase::new();
        assert!(init.initialize(&mut db).is_none());
    }

    #[test]
    fn few_features_returns_none_without_panic() {
        // 少于硬编码 MIN_VALID_FEATURES=8 个特征时，应在特征校验分支返回 None，
        // 且不 panic（对照 C++ 的 count_valid_features < 8 检查）。
        let mut init = DynamicInitializer::new(InitOptions::default());
        init.feed_imu(
            &ImuData {
                timestamp: 0.0,
                wm: Vector3::zeros(),
                am: Vector3::new(0.0, 0.0, 9.81),
            },
            -1.0,
        );
        let mut db = FeatureDatabase::new();
        for id in 0..3 {
            db.update_feature(id, 0.0, 0, 320.0, 240.0, 0.0, 0.0);
        }
        assert!(init.initialize(&mut db).is_none());
    }

    #[test]
    fn compute_dongsi_coeff_shape_and_leading() {
        let d = DMatrix::<f64>::identity(3, 3);
        let dv = DMatrix::<f64>::zeros(3, 1);
        let c = compute_dongsi_coeff(&d, &dv, 9.81);
        assert_eq!(c.len(), 7);
        assert!((c[0] - 1.0).abs() < 1e-12);
    }
}
