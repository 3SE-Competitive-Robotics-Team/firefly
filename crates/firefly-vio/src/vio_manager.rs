//! VIO 管理器（对照 `OpenVINS` `ov_msckf/core/VioManager.cpp/.h`）。
//!
//! 编排 IMU/相机输入：
//! - [`VioManager::feed_measurement_imu`]：喂给传播器与初始化器缓冲；
//! - [`VioManager::feed_measurement_camera`]：跟踪 → 初始化/传播+增广 →
//!   MSCKF/SLAM 更新；
//! - [`VioManager::initialize_with_gt`]：真值初始化（调试/仿真用）；
//! - [`VioManager::try_to_initialize`]：静态/动态初始化（`firefly-vio-init`）。
//!
//! 裁剪（对照 C++ 超出范围的部分）：ARUCO 跟踪器（`max_aruco_features=0`）、
//! 降采样、统计文件与可视化——后续按需移植。

use firefly_vio_core::feat::Feature;
use firefly_vio_core::imu_model::ImuCalibration;
use firefly_vio_core::propagation::{LinearizationPoint, MeanState, Propagator};
use firefly_vio_core::sensor::{CameraData, ImuData};
use firefly_vio_core::track::TrackKlt;
use firefly_vio_types::var::Variable as _;
use nalgebra::{DMatrix, Vector3};
use std::collections::BTreeMap;

use crate::options::VioManagerOptions;
use crate::state::State;
use crate::state_helper::{
    augment_clone, ekf_propagation, marginalize_old_clone, marginalize_slam,
};
use crate::updater::UpdaterMsckf;
use crate::updater_slam::UpdaterSlam;
use crate::updater_zero_velocity::UpdaterZeroVelocity;

/// VIO 管理器（对照 `VioManager`）。
#[derive(Debug)]
pub struct VioManager {
    /// 管理器参数。
    pub params: VioManagerOptions,
    /// 滤波器状态。
    pub state: State,
    /// IMU 传播器（持有 IMU 缓冲）。
    pub propagator: Propagator,
    /// 稀疏特征跟踪器（KLT）。
    pub track_feats: TrackKlt,
    /// MSCKF 特征更新器。
    pub updater_msckf: UpdaterMsckf,
    /// SLAM 特征更新器。
    pub updater_slam: UpdaterSlam,
    /// 初始化器（对照 C++ 的 `initializer`）。
    pub initializer: firefly_vio_init::inertial_init::InertialInitializer,
    /// 零速更新器（`try_zero_velocity` 开启时使用）。
    pub updater_zero_velocity: Option<UpdaterZeroVelocity>,
    /// 自上次零速更新后是否移动过（`only_at_beginning` 用）。
    pub has_moved_since_zero_vel: bool,
    /// 是否已初始化。
    pub is_initialized_vio: bool,
    /// 初始化时刻。
    pub startup_time: f64,
    /// 上次更新时间（相机时钟系）。
    pub timelastupdate: f64,
    /// 上次传播的时间偏移（`Propagator::last_prop_time_offset`）。
    last_prop_time_offset: f64,
}

impl VioManager {
    /// 构造（对照 `VioManager` 构造函数）。
    ///
    /// `cameras`：相机 id → 畸变模型对象（由应用层从标定加载）；
    /// `tracker`：已配置的 KLT 跟踪器。
    #[must_use]
    pub fn new(
        params: VioManagerOptions,
        cameras: BTreeMap<usize, firefly_vio_core::cam::SharedCamera>,
        tracker: TrackKlt,
    ) -> Self {
        let state = State::new(params.state_options.clone());
        let noise = params.imu_noises.clone();
        let propagator = Propagator::new(noise);
        let updater_msckf =
            UpdaterMsckf::new(params.msckf_options.clone(), params.triangulation_options);
        let updater_slam = UpdaterSlam::new(
            params.slam_options.clone(),
            params.state_options.feat_rep_slam,
            params.triangulation_options,
        );
        let init_options = params.init_options.clone();
        let updater_zero_velocity = if params.zero_velocity_options.try_zero_velocity {
            Some(UpdaterZeroVelocity::new(
                params.msckf_options.clone(),
                params.imu_noises.clone(),
                params.zero_velocity_options.max_velocity,
                params.zero_velocity_options.noise_multiplier,
                params.zero_velocity_options.max_disparity,
                params.zero_velocity_options.integrated_accel_constraint,
            ))
        } else {
            None
        };
        let mut mgr = Self {
            params,
            state,
            propagator,
            track_feats: tracker,
            updater_msckf,
            updater_slam,
            initializer: firefly_vio_init::inertial_init::InertialInitializer::new(init_options),
            updater_zero_velocity,
            has_moved_since_zero_vel: false,
            is_initialized_vio: false,
            startup_time: -1.0,
            timelastupdate: -1.0,
            last_prop_time_offset: 0.0,
        };
        mgr.state.cameras = cameras;

        // 同步相机标定到初始化器选项（对照 C++ 的 init_options.camera_*）
        for (id, cam) in &mgr.state.cameras {
            mgr.params
                .init_options
                .camera_intrinsics
                .insert(*id, cam.clone());
        }
        for (id, calib) in &mgr.state.calib_imu_to_cam {
            let q = calib.quat();
            let p = calib.pos();
            let mut ext = nalgebra::SVector::<f64, 7>::zeros();
            ext.fixed_view_mut::<4, 1>(0, 0).copy_from(&q);
            ext.fixed_view_mut::<3, 1>(4, 0).copy_from(&p);
            mgr.params.init_options.camera_extrinsics.insert(*id, ext);
        }
        let t_off = mgr.time_offset();
        mgr.params.init_options.calib_camimu_dt = t_off;
        mgr
    }

    /// 是否已初始化（对照 `VioManager::initialized`）。
    #[must_use]
    pub fn initialized(&self) -> bool {
        // timelastupdate 与 -1.0 的浮点比较：VIO 时间戳为单调时钟值，
        // 语义等价 C++ 的 `timelastupdate != -1`（无 ±0.0/NaN 场景）。
        self.is_initialized_vio && (self.timelastupdate - (-1.0)).abs() > f64::EPSILON
    }

    /// 高频位姿预测（对照 `Propagator::fast_state_propagate`）。
    ///
    /// 不修改 `State`；首次调用（或缓存被传播/更新失效后）从当前状态组装
    /// 缓存。返回局部系速度/角速度与 12×12 协方差，供控制环使用。
    #[must_use]
    pub fn fast_state_propagate(
        &mut self,
        timestamp: f64,
    ) -> Option<firefly_vio_core::propagation::FastState> {
        let calib = self.state.imu_calibration();
        let initial = if self.propagator.cache_valid() {
            None
        } else {
            let q = self.state.imu.quat();
            let p = self.state.imu.pos();
            let v = self.state.imu.vel();
            let bg = self.state.imu.bias_g();
            let ba = self.state.imu.bias_a();
            let mut est = [0.0f64; 16];
            est[0..4].copy_from_slice(q.as_slice());
            est[4..7].copy_from_slice(p.as_slice());
            est[7..10].copy_from_slice(v.as_slice());
            est[10..13].copy_from_slice(bg.as_slice());
            est[13..16].copy_from_slice(ba.as_slice());
            let cov = crate::state_helper::get_marginal_covariance(&self.state, &[(0, 15)]);
            Some(firefly_vio_core::propagation::FastInit {
                time: self.state.timestamp,
                t_off: self.time_offset(),
                est,
                covariance: cov,
            })
        };
        self.propagator
            .fast_state_propagate(initial.as_ref(), timestamp, &calib)
    }

    /// 纯 IMU 传播到指定时刻（无相机更新；对应 C++ `fast_state_propagate`
    /// 的用途：两次更新之间的高频位姿输出）。
    pub fn propagate_to(&mut self, timestamp: f64) {
        if timestamp <= self.state.timestamp {
            return;
        }
        // 只传播均值/协方差，**不增广克隆**——克隆仅在相机时刻由
        // `propagate_and_clone` 增广（对照 open_vins：odom 输出路径不产生克隆）。
        self.propagate_impl(timestamp, false);
        self.timelastupdate = timestamp;
    }

    /// IMU 输入（对照 `VioManager::feed_measurement_imu`）。
    #[fastrace::trace]
    pub fn feed_measurement_imu(&mut self, message: &ImuData) {
        // 最老需要保留的 IMU 时刻：上次边缘化克隆时刻之前（对照 C++）
        let mut oldest_time = self.state.marg_timestep();
        if oldest_time > self.state.timestamp {
            oldest_time = -1.0;
        }
        if !self.is_initialized_vio {
            // 未初始化：保留初始化窗口内的数据（对照 C++ 的 init_window_time）
            oldest_time = message.timestamp - self.params.init_options.init_window_time
                + self.time_offset()
                - 0.10;
        }
        self.propagator.feed_imu(*message, oldest_time);

        // 喂给初始化器（对照 C++：未初始化时）
        if !self.is_initialized_vio {
            self.initializer.feed_imu(message, oldest_time);
        }
    }

    /// 相机输入：跟踪 → 传播/更新（对照 `VioManager::feed_measurement_camera`）。
    ///
    /// # Panics
    /// 消息的 `sensor_ids` 为空或与 `images` 长度不一致（对照 C++ 的 assert）。
    #[fastrace::trace]
    pub fn feed_measurement_camera(&mut self, message: &CameraData) {
        assert!(!message.sensor_ids.is_empty(), "相机消息必须含传感器 id");
        assert_eq!(
            message.sensor_ids.len(),
            message.images.len(),
            "sensor_ids 与 images 长度必须一致"
        );

        // 特征跟踪（对照 C++：trackFEATS->feed_new_camera）
        self.track_feats.feed_new_camera(message);

        // 零速更新分支（对照 C++：初始化后尝试；成功则跳过传播/克隆）
        if self.is_initialized_vio
            && let Some(zero_vel) = &mut self.updater_zero_velocity
            && (!self.params.zero_velocity_options.only_at_beginning
                || !self.has_moved_since_zero_vel)
        {
            let did_zero_vel_update = if self.state.timestamp.total_cmp(&message.timestamp).is_eq()
            {
                false
            } else {
                zero_vel.try_update(
                    &mut self.state,
                    message.timestamp,
                    self.track_feats.database_mut(),
                    &self.propagator,
                )
            };
            if did_zero_vel_update {
                self.propagator.invalidate_cache();
                return;
            }
        }

        // 未初始化 → 尝试初始化（对照 C++ 的 try_to_initialize）
        if !self.is_initialized_vio {
            self.is_initialized_vio = self.try_to_initialize(message);
            if !self.is_initialized_vio {
                return;
            }
        }

        self.do_feature_propagate_update(message);
    }

    /// 真值初始化（对照 `VioManager::initialize_with_gt`）。
    ///
    /// `imustate` 为 `[time, q_GtoI(4), p_IinG(3), v_IinG(3), bg(3), ba(3)]`
    /// （共 17 维，MSCKF 状态序）。
    pub fn initialize_with_gt(&mut self, imustate: &[f64; 17]) {
        self.state.timestamp = imustate[0];
        self.startup_time = imustate[0];
        let q = nalgebra::Vector4::new(imustate[1], imustate[2], imustate[3], imustate[4]);
        let p = Vector3::new(imustate[5], imustate[6], imustate[7]);
        let v = Vector3::new(imustate[8], imustate[9], imustate[10]);
        let bg = Vector3::new(imustate[11], imustate[12], imustate[13]);
        let ba = Vector3::new(imustate[14], imustate[15], imustate[16]);
        self.state.imu.set_value(q, p, v, bg, ba);
        // FEJ 同步为首估计（对照 C++ 的 set_value + set_fej）
        self.state.imu.set_fej(q, p, v, bg, ba);

        // IMU 协方差块重写（对照 C++ initialize_with_gt 的
        // StateHelper::set_initial_covariance 段）：`State::new` 的 1e-6
        // 单位阵等价于"全知"先验——σ_ba=1mm/s² 使加速度零偏不可观测，
        // 有偏场景视觉更新无法修正状态而发散。诚实先验：基础 σ=20mm/s
        // （覆盖 bg/ba），姿态 1.7°，位置 5cm，速度 1cm/s。
        let id = self.state.imu.id() as usize;
        let base = 0.02_f64 * 0.02;
        let mut cov = base * DMatrix::<f64>::identity(15, 15);
        for (blk, sigma) in [(0usize, 0.017f64), (3, 0.05), (6, 0.01)] {
            for r in 0..3 {
                cov[(blk + r, blk + r)] = sigma * sigma;
            }
        }
        // bg/ba 先验 σ 由参数控制（默认 0.02；MuJoCo 无偏置场景应用层调小）：
        // 视觉会把 KLT 亚像素偏置误学成 bg/ba，σ 大 → bg 学到 -0.03 rad/s
        // → roll 线性漂 → 重力投影错 → 位置二次发散。
        let bias_sigma = self.params.init_bias_sigma;
        for blk in [9usize, 12] {
            for r in 0..3 {
                cov[(blk + r, blk + r)] = bias_sigma * bias_sigma;
            }
        }
        self.state.cov.view_mut((id, id), (15, 15)).copy_from(&cov);

        self.is_initialized_vio = true;
    }

    /// 相机-IMU 时间偏移（对照 `_calib_dt_CAMtoIMU->value()(0)`）。
    fn time_offset(&self) -> f64 {
        self.state
            .calib_dt_cam_to_imu
            .as_ref()
            .map_or(0.0, |dt| dt.vec()[0])
    }

    /// 尝试初始化（对照 `VioManager::try_to_initialize`）。
    ///
    /// 单线程实现：直接调用初始化器（对照 C++ 的 `use_multi_threading_subs`
    /// 关闭时的同步路径）。`wait_for_jerk` 取决于是否启用零速更新（对照 C++）。
    ///
    /// 成功后：设置协方差（[`crate::state_helper::set_initial_covariance`]）、
    /// 状态时间与启动时刻、清理过旧特征、恢复跟踪特征数。
    fn try_to_initialize(&mut self, _message: &CameraData) -> bool {
        let wait_for_jerk = self.updater_zero_velocity.is_none();
        let Some(result) = self
            .initializer
            .initialize(self.track_feats.database_mut(), wait_for_jerk)
        else {
            return false;
        };

        // 设置协方差（对照 C++ 的 set_initial_covariance）
        crate::state_helper::set_initial_covariance(
            &mut self.state,
            &result.covariance,
            &result.order,
        );

        // 设置 IMU 状态与 FEJ（对照 C++ 的 t_imu->set_value/set_fej）
        let s16 = result.imu_state;
        let q = nalgebra::Vector4::new(s16[0], s16[1], s16[2], s16[3]);
        let p = Vector3::new(s16[4], s16[5], s16[6]);
        let v = Vector3::new(s16[7], s16[8], s16[9]);
        let bg = Vector3::new(s16[10], s16[11], s16[12]);
        let ba = Vector3::new(s16[13], s16[14], s16[15]);
        self.state.imu.set_value(q, p, v, bg, ba);
        self.state.imu.set_fej(q, p, v, bg, ba);

        // 设置状态时间与启动时刻（对照 C++）
        self.state.timestamp = result.timestamp;
        self.startup_time = result.timestamp;

        // 清理过旧特征并恢复跟踪特征数（对照 C++）
        self.track_feats
            .database_mut()
            .cleanup_measurements(self.state.timestamp);
        let num_pts = self.params.init_options.init_max_features;
        let num_cam = self.params.state_options.num_cameras.max(1);
        self.track_feats
            .set_num_features((num_pts as f64 / num_cam as f64).floor() as usize);

        // 若移动中则禁用零速更新（对照 C++ 的 has_moved_since_zupt）
        if self.state.imu.vel().norm() > self.params.zero_velocity_options.max_velocity {
            self.has_moved_since_zero_vel = true;
        }

        log::info!("[init]: successful initialization (q={q:?}, bg={bg:?}, ba={ba:?}, v={v:?})");
        self.is_initialized_vio = true;
        true
    }

    /// 传播 + 增广 + MSCKF/SLAM 更新（对照 `VioManager::do_feature_propagate_update`，
    /// SLAM 分支完整移植；ARUCO 分支裁剪——无 aruco 跟踪器）。
    // 与 C++ 1:1 移植的编排长流程，拆分会破坏对照可审计性。
    #[allow(clippy::too_many_lines)]
    #[fastrace::trace]
    fn do_feature_propagate_update(&mut self, message: &CameraData) {
        // 乱序相机消息直接忽略（对照 C++）
        if self.state.timestamp > message.timestamp {
            log::warn!(
                "图像乱序：state={:.3} > 消息={:.3}，跳过",
                self.state.timestamp,
                message.timestamp
            );
            return;
        }

        // 传播到当前时刻并增广克隆（对照 C++ 的 propagate_and_clone）
        // 时间戳精确相等检查（时间对齐语义，对照 C++ 的 `!=`；VIO 时钟为
        // 单调浮点值，total_cmp 保持精确语义）
        if !self.state.timestamp.total_cmp(&message.timestamp).is_eq() {
            self.propagate_and_clone(message.timestamp);
        }

        // 克隆不足（min(max_clone_size, 5)）时等待（对照 C++）
        let min_clones = self.state.options.max_clone_size.min(5);
        if self.state.clones_imu.len() < min_clones {
            log::debug!(
                "等待克隆数达到 {min_clones}（当前 {}）",
                self.state.clones_imu.len()
            );
            return;
        }
        if !self.state.timestamp.total_cmp(&message.timestamp).is_eq() {
            log::warn!("传播未能推进到消息时刻");
            return;
        }
        self.timelastupdate = message.timestamp;
        self.has_moved_since_zero_vel = true;

        //=====================================================================
        // MSCKF 特征与 SLAM 特征收集（对照 C++ 第 362-495 行）
        //=====================================================================

        // 2. 取丢失特征（新帧未跟踪到；对照 C++ 的 feats_lost）
        let mut feats_lost = self
            .track_feats
            .database_mut()
            .features_not_containing_newer(self.state.timestamp, false, true);

        // 3. 只保留与当前消息相机相关的特征（对照 C++ 的 camid 过滤）
        feats_lost.retain(|feat| {
            feat.timestamps
                .keys()
                .any(|cam| message.sensor_ids.contains(&(*cam as i32)))
        });

        // 4. 边缘化时刻特征（对照 C++ 的 feats_marg）
        let mut feats_marg: Vec<Feature> = Vec::new();
        if self.state.clones_imu.len() > self.state.options.max_clone_size
            || self.state.clones_imu.len() > 5
        {
            let marg_time = self.state.marg_timestep();
            feats_marg = self
                .track_feats
                .database_mut()
                .features_containing(marg_time, false, true);
        }

        // 5. 去重：feats_lost 中不允许包含 feats_marg（对照 C++ 第二段循环）
        let marg_ids: std::collections::HashSet<usize> =
            feats_marg.iter().map(|f| f.featid).collect();
        feats_lost.retain(|f| !marg_ids.contains(&f.featid));

        // 6. 从 feats_marg 中挑出达到最大跟踪长度的长轨迹（SLAM 候选）
        let mut feats_maxtracks: Vec<Feature> = Vec::new();
        feats_marg.retain_mut(|feat| {
            let reached_max = feat
                .timestamps
                .values()
                .any(|ts| ts.len() > self.state.options.max_clone_size);
            if reached_max {
                feats_maxtracks.push(feat.clone());
                false
            } else {
                true
            }
        });

        // 7. 现有 SLAM 特征数（无 aruco → 全部按特征 id 判断时不进位减；
        //    max_aruco_features=0 → curr_aruco_tags 恒为 0）
        let curr_aruco_tags = 0;

        // 8. 新增 SLAM 特征：若还有余量且过了延迟期，取若干长轨迹（对照 C++）
        let mut feats_slam: Vec<Feature> = Vec::new();
        let elapsed = message.timestamp - self.startup_time;
        let max_slam = self.state.options.max_slam_features;
        let capacity = max_slam + curr_aruco_tags;
        if max_slam > 0
            && elapsed >= self.params.dt_slam_delay
            && self.state.features_slam.len() < capacity
        {
            let amount_to_add = capacity - self.state.features_slam.len();
            let valid_amount = amount_to_add.min(feats_maxtracks.len());
            if valid_amount > 0 {
                let start = feats_maxtracks.len() - valid_amount;
                let tail: Vec<Feature> = feats_maxtracks.split_off(start);
                feats_slam.extend(tail);
            }
        }

        // 9. 遍历现有 SLAM 特征，取回它们当前帧的跟踪（对照 C++ 循环）
        // 仍在跟踪 → 进入更新；丢失或失败多次 → 标记边缘化
        let current_slam_ids: Vec<usize> = self.state.features_slam.keys().copied().collect();
        for featid in current_slam_ids {
            // 取回特征库中的跟踪（remove=false，返回克隆；对照 C++ 的 feat2）
            let feat2 = self.track_feats.database_mut().get_feature(featid, false);
            if let Some(f2) = &feat2 {
                feats_slam.push(f2.clone());
            }
            // 当前消息是否为该特征的首相机（对照 C++ 的 current_unique_cam）
            let current_unique_cam = self
                .state
                .features_slam
                .get(&featid)
                .is_some_and(|l| message.sensor_ids.contains(&l.unique_camera_id));
            // 在首相机上丢失 → 无后续跟踪，标记边缘化
            if feat2.is_none()
                && current_unique_cam
                && let Some(l) = self.state.features_slam.get_mut(&featid)
            {
                l.should_marg = true;
            }
            // 连续更新失败 → 标记边缘化（对照 C++：update_fail_count > 1）
            if self
                .state
                .features_slam
                .get(&featid)
                .is_some_and(|l| l.update_fail_count > 1)
                && let Some(l) = self.state.features_slam.get_mut(&featid)
            {
                l.should_marg = true;
            }
        }

        // 10. 边缘化所有标记 should_marg 的 SLAM 特征（对照 C++）
        marginalize_slam(&mut self.state);

        // 11. 分离为新特征（延迟初始化）与老特征（SLAM 更新）（对照 C++）
        let mut feats_slam_delayed: Vec<Feature> = Vec::new();
        let mut feats_slam_update: Vec<Feature> = Vec::new();
        for feat in feats_slam {
            if self.state.features_slam.contains_key(&feat.featid) {
                feats_slam_update.push(feat);
            } else {
                feats_slam_delayed.push(feat);
            }
        }

        // 12. MSCKF 更新用的特征 = lost + marg + maxtracks 剩余（对照 C++）
        let mut featsup_msckf = feats_lost;
        featsup_msckf.append(&mut feats_marg);
        featsup_msckf.append(&mut feats_maxtracks);

        // 13. 按跟踪长度升序，只保留最长的 max_msckf_in_update（对照 C++ sort/truncate）
        let track_len = |f: &Feature| -> usize { f.timestamps.values().map(Vec::len).sum() };
        featsup_msckf.sort_by_key(track_len);
        if featsup_msckf.len() > self.state.options.max_msckf_in_update {
            let keep = self.state.options.max_msckf_in_update;
            let drop = featsup_msckf.len() - keep;
            featsup_msckf.drain(..drop);
        }

        //=====================================================================
        // 更新：先 MSCKF，再批量 SLAM 更新，最后延迟初始化（对照 C++ 505-548）
        //=====================================================================
        let msckf_ids: Vec<usize> = featsup_msckf.iter().map(|f| f.featid).collect();
        self.updater_msckf
            .update(&mut self.state, &mut featsup_msckf);
        self.propagator.invalidate_cache();
        // MSCKF 用过的特征全部删除（对照 C++ 末尾的 to_delete 标记段）
        self.track_feats.database_mut().mark_deleted(msckf_ids);

        // 14. SLAM 更新（分批 max_slam_in_update；对照 C++：循环内 erase 前缀、
        // 列表递减可终止，处理后不回填）。SLAM 特征是持久路标——**不标记删除**
        //（C++ 的 `to_delete = true` 只作用于 MSCKF 特征段）。
        let max_slam_in_update = self.state.options.max_slam_in_update;
        while !feats_slam_update.is_empty() {
            let take = max_slam_in_update.min(feats_slam_update.len());
            let mut batch: Vec<Feature> = feats_slam_update.drain(..take).collect();
            self.updater_slam.update(&mut self.state, &mut batch);
            self.propagator.invalidate_cache();
        }

        // 15. SLAM 延迟初始化（对照 C++）
        let delayed_ids: Vec<usize> = feats_slam_delayed.iter().map(|f| f.featid).collect();
        if !feats_slam_delayed.is_empty() {
            self.updater_slam
                .delayed_init(&mut self.state, &mut feats_slam_delayed);
            self.propagator.invalidate_cache();
            self.track_feats.database_mut().mark_deleted(delayed_ids);
        }

        //=====================================================================
        // 清理与边缘化（对照 C++ 552-596）
        //=====================================================================

        // 16. 清理（对照 C++ 末尾：cleanup + 边缘化旧克隆）
        self.track_feats.database_mut().cleanup();
        // 17. 锚点切换（锚定表示用；当前 GLOBAL_3D 为 no-op）
        self.updater_slam.change_anchors(&mut self.state);
        if self.state.clones_imu.len() > self.state.options.max_clone_size {
            let marg_time = self.state.marg_timestep();
            self.track_feats
                .database_mut()
                .cleanup_measurements(marg_time);
        }
        marginalize_old_clone(&mut self.state);
    }

    /// 传播到指定相机时刻并增广克隆（对照
    /// `Propagator::propagate_and_clone`）。
    ///
    /// 多段合成：`Phi_summed = Φ_i·...·Φ_0`、`Qd_summed` 按
    /// `Q ← Φ·Q·Φᵀ + Qd_i` 累积，最后 `EKFPropagation` 写回协方差并增广克隆。
    #[fastrace::trace]
    fn propagate_and_clone(&mut self, timestamp: f64) {
        self.propagate_impl(timestamp, true);
    }

    /// 传播主体（`augment` 决定是否增广克隆）。
    ///
    /// 多段合成：`Phi_summed = Φ_i·...·Φ_0`、`Qd_summed` 按
    /// `Q ← Φ·Q·Φᵀ + Qd_i` 累积，最后 `EKFPropagation` 写回协方差，
    /// `augment` 时增广克隆（对照 `Propagator::propagate_and_clone`）。
    #[fastrace::trace]
    fn propagate_impl(&mut self, timestamp: f64, augment: bool) {
        let t_off_new = self.time_offset();
        let time0 = self.state.timestamp + self.last_prop_time_offset;
        let time1 = timestamp + t_off_new;

        let imu_data = self.propagator.imu_data_snapshot();
        let Some(prop_data) = Propagator::select_imu_readings(&imu_data, time0, time1, false)
        else {
            log::warn!(
                "IMU 测量不足，无法传播（time0={time0:.4} time1={time1:.4} state={:.4} off={:.6} imu_n={}）",
                self.state.timestamp,
                self.last_prop_time_offset,
                imu_data.len()
            );
            return;
        };

        let dim = 15 + self.state.options.imu_intrinsic_size();
        let mut phi_summed = DMatrix::<f64>::identity(dim, dim);
        let mut qd_summed = DMatrix::<f64>::zeros(dim, dim);
        let opts = self.state.options.to_propagation_options();

        for i in 0..prop_data.len() - 1 {
            let calib: ImuCalibration = self.state.imu_calibration();
            let input = MeanState::new(
                self.state.imu.quat(),
                self.state.imu.pos(),
                self.state.imu.vel(),
            );
            // FEJ 线性化点：开启时用首估计，否则当前均值
            let lin = if opts.do_fej {
                LinearizationPoint::new(
                    self.state.imu.pose().rot_fej(),
                    self.state.imu.vel_fej(),
                    self.state.imu.pose().pos_fej(),
                )
            } else {
                LinearizationPoint::from_state(&input)
            };

            let prop = self.propagator.predict_and_compute(
                &opts,
                &calib,
                &prop_data[i],
                &prop_data[i + 1],
                &input,
                &lin,
            );

            // 更新均值（bg/ba 不变）—— 同步更新 FEJ（对照
            // `Propagator::predict_and_compute` 末尾 `set_value`+`set_fej`）
            let bg = self.state.imu.bias_g();
            let ba = self.state.imu.bias_a();
            self.state.imu.set_value(prop.q, prop.p, prop.v, bg, ba);
            self.state.imu.set_fej(prop.q, prop.p, prop.v, bg, ba);

            // 合成状态转移与噪声（对照 C++：Phi_summed = F·Phi_summed）
            phi_summed = &prop.f * &phi_summed;
            qd_summed = &prop.f * qd_summed * prop.f.transpose() + prop.qd;
            qd_summed = 0.5 * (&qd_summed + qd_summed.transpose());
        }

        // 最后角速度（时间偏移标定的克隆增广用；对照 C++ 的 last_w）
        let last_w = {
            let last = prop_data.last().expect("prop_data 非空");
            let calib: ImuCalibration = self.state.imu_calibration();
            let a_hat = calib.r_acc_to_imu * calib.da * (last.am - calib.bias_a);
            calib.r_gyro_to_imu * calib.dw * (last.wm - calib.bias_g - calib.tg * a_hat)
        };

        // 协方差传播（对照 C++：EKFPropagation(state, Phi_order, Phi_order, ...)）
        let order = self.state.variable_order();
        let order: Vec<(i32, usize)> = order
            .iter()
            .filter(|(id, _)| *id < 15 + dim as i32)
            .copied()
            .collect();
        // 只取 IMU + 标定块（克隆不参与传播）
        let order: Vec<(i32, usize)> = order
            .iter()
            .filter(|(id, _)| *id >= 0 && (*id as usize) < dim)
            .copied()
            .collect();
        ekf_propagation(&mut self.state, &order, &order, &phi_summed, &qd_summed);

        // 更新时间戳，必要时增广克隆（对照 C++ 末尾）
        self.state.timestamp = timestamp;
        if augment {
            augment_clone(&mut self.state, &last_w);
            // 可观测性：克隆窗口大小（诊断边缘化是否生效）
            log::debug!(
                "propagate_and_clone t={timestamp:.3} clones={}",
                self.state.clones_imu.len()
            );
        }
        self.last_prop_time_offset = t_off_new;
        log::debug!(
            "prop_impl {} t={timestamp:.3} p=({:.3},{:.3},{:.3})",
            if augment { "CLONE" } else { "odom " },
            self.state.imu.pos().x,
            self.state.imu.pos().y,
            self.state.imu.pos().z
        );
        // 状态已变化，fast-prop 缓存失效（对照 C++ 调用方的 invalidate_cache）
        self.propagator.invalidate_cache();
    }
}

/// 单相机参数（占位：`VioManagerOptions` 中与跟踪器相关的字段由应用层
/// 直接配置 `TrackKlt`，此处不重复）。
pub type CamParamsPlaceholder = ();

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::StateOptions;
    use firefly_vio_core::sensor::GrayImage;
    use firefly_vio_core::track::{HistogramMethod, TrackKlt};
    use std::collections::HashMap;

    fn test_manager() -> VioManager {
        let params = VioManagerOptions::default();
        let cameras = BTreeMap::new();
        let tracker = TrackKlt::new(
            HashMap::new(),
            200,
            0,
            false,
            HistogramMethod::None,
            10,
            5,
            5,
            15,
        );
        VioManager::new(params, cameras, tracker)
    }

    #[test]
    fn gt_initialization_sets_state() {
        let mut mgr = test_manager();
        let mut imustate = [0.0f64; 17];
        imustate[0] = 10.0;
        imustate[5] = 1.0;
        imustate[6] = 2.0;
        imustate[7] = 3.0;
        imustate[8] = 0.1;
        imustate[11] = 0.01;
        mgr.initialize_with_gt(&imustate);
        // initialized() 需要 timelastupdate != -1（首次更新后）；GT 初始化
        // 后 is_initialized_vio 即为 true（对照 C++ 语义）
        assert!(mgr.is_initialized_vio);
        assert!(!mgr.initialized());
        // FEJ 同步为首估计
        assert_eq!(mgr.state.imu.vel_fej(), Vector3::new(0.1, 0.0, 0.0));
        assert!((mgr.state.timestamp - 10.0).abs() < 1e-12);
        assert_eq!(mgr.state.imu.pos(), Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(mgr.state.imu.vel(), Vector3::new(0.1, 0.0, 0.0));
        assert_eq!(mgr.state.imu.bias_g(), Vector3::new(0.01, 0.0, 0.0));
    }

    #[test]
    fn imu_feed_buffers() {
        let mut mgr = test_manager();
        for t in 0..10 {
            mgr.feed_measurement_imu(&ImuData {
                timestamp: f64::from(t),
                wm: Vector3::zeros(),
                am: Vector3::zeros(),
            });
        }
        // feed_imu 会按 oldest_time（未初始化窗口 2.1s）清理旧测量：
        // 保留最后 ~2.1s 内的数据（对照 C++ 的 clean_old_imu_measurements）
        let n = mgr.propagator.imu_data_len();
        assert!((2..=10).contains(&n), "缓冲长度 {n} 应在清理窗口内");
    }

    #[test]
    fn propagate_and_clone_with_zero_imu() {
        let mut mgr = test_manager();
        mgr.state.timestamp = 0.0;
        mgr.state.imu.set_value(
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        // 恒定零 IMU 测量 → 传播后状态不变，克隆被增广
        for t in 0..20 {
            mgr.feed_measurement_imu(&ImuData {
                timestamp: 0.05 * f64::from(t),
                wm: Vector3::zeros(),
                am: Vector3::zeros(),
            });
        }
        mgr.propagate_and_clone(0.5);
        assert!((mgr.state.timestamp - 0.5).abs() < 1e-12);
        assert_eq!(mgr.state.clones_imu.len(), 1);
        assert_eq!(mgr.state.cov.nrows(), 21);
        // 零 IMU 测量但重力存在：p_z = −½·g·dt² = −1.22625（对照 C++ 行为）
        assert!((mgr.state.imu.pos().z - (-1.22625)).abs() < 1e-6);
        // 速度 v_z = −g·dt = −4.905
        assert!((mgr.state.imu.vel().z - (-4.905)).abs() < 1e-6);
        // 零角速度 → 姿态不变
        assert_eq!(
            mgr.state.imu.quat(),
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0)
        );
    }

    #[test]
    fn state_options_defaults() {
        let s = StateOptions::default();
        assert_eq!(s.max_clone_size, 11);
        assert_eq!(s.max_msckf_in_update, 1000);
    }

    #[test]
    fn gray_image_roundtrip() {
        let img = GrayImage {
            width: 4,
            height: 4,
            data: vec![0u8; 16],
        };
        assert_eq!(img.data.len(), 16);
    }
}
