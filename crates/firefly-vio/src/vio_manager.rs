//! VIO 管理器（对照 `OpenVINS` `ov_msckf/core/VioManager.cpp/.h`）。
//!
//! 编排 IMU/相机输入：
//! - [`VioManager::feed_measurement_imu`]：喂给传播器与初始化器缓冲；
//! - [`VioManager::feed_measurement_camera`]：跟踪 → 初始化/传播+增广 →
//!   MSCKF/SLAM 更新；
//! - [`VioManager::initialize_with_gt`]：真值初始化（仅供合成算法评测夹具）；
//! - [`VioManager::try_to_initialize`]：静态/动态初始化（`firefly-vio-init`）。
//!
//! 本模块不提供 ARUCO 跟踪、图像降采样、统计文件或可视化。

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
    last_prop_time_offset: Option<f64>,
    /// 最新只读地图关键帧；消费端取走，不保留无界队列。
    pub mapping_snapshot: Option<crate::mapping::Snapshot>,
    last_mapping_time: f64,
}

impl VioManager {
    /// 构造（对照 `VioManager` 构造函数）。
    ///
    /// `cameras`：相机 id → 畸变模型对象（由应用层从标定加载）；
    /// `tracker`：已配置的 KLT 跟踪器。
    ///
    /// # Panics
    /// 相机 id 不在 `0..num_cameras` 内。
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
        let init_options = firefly_vio_init::options::InitOptions {
            sigma_w: params.imu_noises.sigma_w,
            sigma_wb: params.imu_noises.sigma_wb,
            sigma_a: params.imu_noises.sigma_a,
            sigma_ab: params.imu_noises.sigma_ab,
            ..params.init_options.clone()
        };
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
            last_prop_time_offset: None,
            mapping_snapshot: None,
            last_mapping_time: f64::NEG_INFINITY,
        };
        mgr.state.cameras = cameras;

        // 同步相机标定到初始化器选项（对照 C++ 的 init_options.camera_*）
        for (id, cam) in &mgr.state.cameras {
            let intrinsics = mgr
                .state
                .cam_intrinsics
                .get_mut(id)
                .expect("相机 id 必须属于状态");
            let value = nalgebra::DVector::from_column_slice(&cam.value());
            intrinsics.set_value(value.clone());
            intrinsics.set_fej(value);
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

    /// 滤波状态实际积分到的 IMU 时刻；在线更新时延参数不能重贴状态时间戳。
    #[must_use]
    pub fn state_imu_timestamp(&self) -> f64 {
        self.state.timestamp
            + self
                .last_prop_time_offset
                .unwrap_or_else(|| self.time_offset())
    }

    /// 高频位姿预测，目标时间为 IMU 时钟秒（对照 `Propagator::fast_state_propagate`）。
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
            Some(self.fast_state_snapshot())
        };
        self.propagator
            .fast_state_propagate(initial.as_ref(), timestamp, &calib)
    }

    /// 供异步输出传播使用的完整 IMU 后验副本；时间固定在实际积分到的 IMU 时刻。
    #[must_use]
    pub fn fast_state_snapshot(&self) -> firefly_vio_core::propagation::FastInit {
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
        firefly_vio_core::propagation::FastInit {
            time: self.state_imu_timestamp(),
            t_off: 0.0,
            est,
            covariance: cov,
        }
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
        self.state.update_diagnostics.clear();
        self.state.slam_diagnostics.clear();
        self.state.update_source = "zero_velocity";
        assert!(!message.sensor_ids.is_empty(), "相机消息必须含传感器 id");
        assert_eq!(
            message.sensor_ids.len(),
            message.images.len(),
            "sensor_ids 与 images 长度必须一致"
        );

        if !message.is_valid() {
            return;
        }

        if self.state.options.do_calib_camera_intrinsics {
            for (&id, camera) in &self.state.cameras {
                self.track_feats.set_camera_calibration(id, camera.clone());
            }
        }
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

    /// 合成算法评测夹具初始化（对照 `VioManager::initialize_with_gt`）。
    /// 应用运行入口必须使用传感器初始化，禁止调用此接口。
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

        // 初始化协方差先验：姿态标准差 1.7°，位置 5 cm，速度 1 cm/s。
        let id = self.state.imu.id() as usize;
        let base = 0.02_f64 * 0.02;
        let mut cov = base * DMatrix::<f64>::identity(15, 15);
        for (blk, sigma) in [(0usize, 0.017f64), (3, 0.05), (6, 0.01)] {
            for r in 0..3 {
                cov[(blk + r, blk + r)] = sigma * sigma;
            }
        }
        // bg/ba 先验标准差由参数控制；无偏置仿真可由应用配置更小的值。
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
    pub fn time_offset(&self) -> f64 {
        self.state
            .calib_dt_cam_to_imu
            .as_ref()
            .map_or(0.0, |dt| dt.vec()[0])
    }

    /// 尝试初始化（对照 `VioManager::try_to_initialize`）。
    ///
    /// 单线程实现：直接调用初始化器（对照 C++ 的 `use_multi_threading_subs`
    /// 关闭时的同步路径）。静止上电允许不等待急动，运动初始化由选项控制。
    ///
    /// 成功后：设置协方差（[`crate::state_helper::set_initial_covariance`]）、
    /// 状态时间与启动时刻、清理过旧特征，保持应用配置的跟踪特征数。
    fn try_to_initialize(&mut self, _message: &CameraData) -> bool {
        self.params.init_options.camera_intrinsics = self.state.cameras.clone();
        for (id, calib) in &self.state.calib_imu_to_cam {
            let mut ext = nalgebra::SVector::<f64, 7>::zeros();
            ext.fixed_rows_mut::<4>(0).copy_from(&calib.quat());
            ext.fixed_rows_mut::<3>(4).copy_from(&calib.pos());
            self.params.init_options.camera_extrinsics.insert(*id, ext);
        }
        self.params.init_options.calib_camimu_dt = self.time_offset();
        self.initializer.configure(self.params.init_options.clone());
        let wait_for_jerk = self.params.init_wait_for_jerk && self.updater_zero_velocity.is_none();
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

        // 清理初始化时刻之前的观测，跟踪特征数由应用配置。
        self.track_feats
            .database_mut()
            .cleanup_measurements(self.state.timestamp);
        // 若移动中则禁用零速更新（对照 C++ 的 has_moved_since_zupt）
        if self.state.imu.vel().norm() > self.params.zero_velocity_options.max_velocity {
            self.has_moved_since_zero_vel = true;
        }

        log::info!("[init]: successful initialization (q={q:?}, bg={bg:?}, ba={ba:?}, v={v:?})");
        self.is_initialized_vio = true;
        true
    }

    /// 传播、克隆增广与 MSCKF/SLAM 更新；输入流程与 `VioManager::do_feature_propagate_update` 对应。
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
            !self.state.features_slam.contains_key(&feat.featid)
                && feat
                    .timestamps
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
            // 已有路标的测量只属于 SLAM，不能作为独立 MSCKF 特征消费。
            feats_marg.retain(|feat| !self.state.features_slam.contains_key(&feat.featid));
        }

        // 5. 去重：feats_lost 中不允许包含 feats_marg（对照 C++ 第二段循环）
        let marg_ids: std::collections::HashSet<usize> =
            feats_marg.iter().map(|f| f.featid).collect();
        feats_lost.retain(|f| !marg_ids.contains(&f.featid));

        // 6. 从 feats_marg 中挑出达到最大跟踪长度的长轨迹（SLAM 候选）
        let mut feats_maxtracks: Vec<Feature> = feats_marg
            .extract_if(.., |feat| {
                feat.timestamps
                    .values()
                    .any(|ts| ts.len() > self.state.options.max_clone_size)
            })
            .collect();

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

        // 9. SLAM 更新集来源（对照官方 `featureUpdate` 的路标循环）：遍历全部 SLAM
        // 路标。对每个路标：feature 在库则加入更新集；feature 缺失且其 host 相机在
        // 本帧 → 标记边缘化；`update_fail_count > 1` → 标记边缘化。标记只作用于
        // 被访问的路标，缺失但未标记的路标不因此被丢弃。
        for featid in self.state.features_slam.keys().copied().collect::<Vec<_>>() {
            let feat2 = self.track_feats.database_mut().get_feature(featid, false);
            let feature_missing = feat2.is_none();
            if let Some(f2) = feat2 {
                feats_slam.push(f2);
            }
            let Some(lm) = self.state.features_slam.get_mut(&featid) else {
                continue;
            };
            let host_cam_in_frame = message.sensor_ids.contains(&lm.unique_camera_id);
            if feature_missing && host_cam_in_frame {
                lm.should_marg = true;
            }
            if lm.update_fail_count > 1 {
                lm.should_marg = true;
            }
        }

        // 10. 边缘化所有标记 `should_marg` 的 SLAM 特征（对照官方
        // `marginalizeSlam`：在选点循环之后执行）。
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
        log::debug!(
            "SLAM 选点 t={:.2} 路标={} 更新={} 延迟={}",
            self.state.timestamp,
            self.state.features_slam.len(),
            feats_slam_update.len(),
            feats_slam_delayed.len()
        );
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
        self.state.update_source = "msckf";
        self.updater_msckf
            .update(&mut self.state, &mut featsup_msckf);
        self.propagator.invalidate_cache();
        // MSCKF 用过的特征全部删除（对照 C++ 末尾的 to_delete 标记段）
        self.track_feats.database_mut().mark_deleted(msckf_ids);

        // 14. SLAM 更新（分批 max_slam_in_update；对照 C++：循环内 erase 前缀、
        // 列表递减可终止，处理后不回填）。Rust 端 update 操作的是特征克隆，
        // 消费标记（to_delete）须按返回值写回数据库——对照 C++ 中
        // `UpdaterSLAM::update` 直接在库内特征上置位（「Delete it so we do
        // not reuse information」）：不标记则同一测量会被 marg/max-track
        // 查询再次取用（MSCKF 双重消费 + 同一更新批次内重复入列）。
        let max_slam_in_update = self.state.options.max_slam_in_update;
        self.state.update_source = "slam";
        while !feats_slam_update.is_empty() {
            let take = max_slam_in_update.min(feats_slam_update.len());
            let mut batch: Vec<Feature> = feats_slam_update.drain(..take).collect();
            let consumed = self.updater_slam.update(&mut self.state, &mut batch);
            self.propagator.invalidate_cache();
            self.track_feats.database_mut().mark_deleted(consumed);
        }

        // 15. SLAM 延迟初始化（对照 C++）
        let delayed_ids: Vec<usize> = feats_slam_delayed.iter().map(|f| f.featid).collect();
        self.state.update_source = "landmark_init";
        if !feats_slam_delayed.is_empty() {
            self.updater_slam
                .delayed_init(&mut self.state, &mut feats_slam_delayed);
            self.propagator.invalidate_cache();
            self.track_feats.database_mut().mark_deleted(delayed_ids);
        }

        //=====================================================================
        // 清理与边缘化（对照 C++ 552-596）
        //=====================================================================

        if self.state.timestamp - self.last_mapping_time >= 0.2 - 1e-9 {
            self.mapping_snapshot = crate::mapping::capture(self);
            self.last_mapping_time = self.state.timestamp;
        }
        // 17. 清理（对照 C++ 末尾：cleanup + 边缘化旧克隆）
        self.track_feats.database_mut().cleanup();
        // 18. 锚点切换（锚定表示用；当前 GLOBAL_3D 为 no-op）
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
        let time0 = self.state.timestamp + self.last_prop_time_offset.unwrap_or(t_off_new);
        let time1 = timestamp + t_off_new;

        let imu_data = self.propagator.imu_data_snapshot();
        let Some(prop_data) = Propagator::select_imu_readings(&imu_data, time0, time1, false)
        else {
            log::warn!(
                "IMU 测量不足，无法传播（time0={time0:.4} time1={time1:.4} state={:.4} off={:.6} imu_n={}）",
                self.state.timestamp,
                self.last_prop_time_offset.unwrap_or(t_off_new),
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
        self.last_prop_time_offset = Some(t_off_new);
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

/// 单相机参数；跟踪器选项由应用层直接传给 `TrackKlt`。
pub type CamParamsPlaceholder = ();

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::StateOptions;
    use firefly_vio_core::sensor::GrayImage;
    use firefly_vio_core::track::{HistogramMethod, TrackKlt};
    use nalgebra::Vector2;
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
    fn camera_intrinsics_initialize_and_update_projection() {
        use firefly_vio_core::cam::{CamRadtan, SharedCamera};
        use std::sync::Arc;
        let value = [170.0, 172.0, 160.0, 120.0, 0.01, 0.0, 0.0, 0.0];
        let camera: SharedCamera = Arc::new(CamRadtan::new(320, 240, &value));
        let mut params = VioManagerOptions::default();
        params.state_options.do_calib_camera_intrinsics = true;
        let tracker = TrackKlt::new(
            HashMap::from([(0, camera.clone())]),
            100,
            0,
            false,
            HistogramMethod::None,
            10,
            5,
            5,
            15,
        );
        let mut mgr = VioManager::new(params, BTreeMap::from([(0, camera)]), tracker);
        assert_eq!(mgr.state.cam_intrinsics[&0].vec().as_slice(), &value);
        let mut dx = nalgebra::DVector::zeros(mgr.state.cov.nrows());
        dx[mgr.state.cam_intrinsics[&0].id() as usize] = 2.0;
        mgr.state.update_all(&dx);
        assert!((mgr.state.cameras[&0].value()[0] - 172.0).abs() < 1e-12);
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

    /// SLAM 场景：相机 0 内参 `fx=fy=600`、`cx=320`、`cy=240`，外参为单位位姿
    /// （相机系 = IMU 系平移）。供整体输入输出测试注入跟踪观测与路标。
    fn slam_manager() -> VioManager {
        use firefly_vio_core::cam::{CamRadtan, SharedCamera};
        use std::sync::Arc;
        let mut params = VioManagerOptions::default();
        params.state_options.max_clone_size = 6;
        let cam: SharedCamera = Arc::new(CamRadtan::new(
            640,
            480,
            &[600.0, 600.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0],
        ));
        let tracker = TrackKlt::new(
            HashMap::from([(0usize, cam.clone())]),
            200,
            0,
            false,
            HistogramMethod::None,
            10,
            5,
            5,
            15,
        );
        VioManager::new(params, BTreeMap::from([(0usize, cam)]), tracker)
    }

    /// 推进一帧：设置 IMU 位姿并增广克隆。
    fn feed_frame(mgr: &mut VioManager, t: f64, p_imu: Vector3<f64>) {
        let q = nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0);
        mgr.state.timestamp = t;
        mgr.state.imu.set_value(
            q,
            p_imu,
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        mgr.state.imu.set_fej(
            q,
            p_imu,
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        crate::state_helper::augment_clone(&mut mgr.state, &Vector3::zeros());
    }

    #[test]
    fn mapping_snapshot_owns_geometry_without_mutating_filter_or_retiming_it() {
        let mut mgr = slam_manager();
        let point = Vector3::new(0.55, 0.25, 3.05);
        for (i, position) in four_views().into_iter().enumerate() {
            let timestamp = 1. + i as f64;
            feed_frame(&mut mgr, timestamp, position);
            let relative = point - position;
            let u = (relative.x / relative.z) as f32;
            let v = (relative.y / relative.z) as f32;
            mgr.track_feats.database_mut().update_feature(
                7,
                timestamp,
                0,
                600. * u + 320.,
                600. * v + 240.,
                u,
                v,
            );
        }
        mgr.last_prop_time_offset = Some(0.02);
        mgr.track_feats.database_mut().mark_deleted([7]);
        let covariance = mgr.state.cov.clone();
        let snapshot = crate::mapping::capture(&mgr).unwrap();
        assert_eq!(snapshot.camera_time, 4.);
        assert!((snapshot.imu_time - 4.02).abs() < 1e-12);
        assert_eq!(snapshot.observations.len(), 1);
        let actual = Vector3::from(snapshot.observations[0].point_body.unwrap());
        assert!((actual - (point - four_views()[3])).norm() < 1e-5);
        assert_eq!(mgr.state.cov, covariance);
        mgr.track_feats.database_mut().cleanup();
        assert!(mgr.track_feats.database().get_feature_clone(7).is_none());
        assert_eq!(snapshot.observations[0].id, 7);
    }

    /// 四个不共线视角（射线方向不退化）。
    fn four_views() -> [Vector3<f64>; 4] {
        [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.5, 0.0, 0.0),
            Vector3::new(0.5, 0.4, 0.0),
            Vector3::new(0.0, 0.4, 0.0),
        ]
    }

    /// 已有 SLAM 路标的测量只由批量 SLAM 更新消费一次：帧内更新路径的结果必须与
    /// 单独调用一次 `updater_slam.update` 完全一致，MSCKF 不得同时使用同一测量。
    #[test]
    fn deferred_slam_observations_are_consumed_once_by_slam_only() {
        use crate::{landmark::Landmark, options::FeatRepresentation};
        let mut mgr = slam_manager();
        // 克隆上限放到帧数之上：本测试只比较一次 SLAM 观测批次的消费结果，
        // 不掺入克隆边缘化带来的协方差维度变化。
        mgr.state.options.max_clone_size = 10;
        let point = Vector3::new(0.55, 0.25, 3.05);
        for i in 0..6 {
            let t = f64::from(i);
            let position = Vector3::new(0.1 * t, 0.0, 0.0);
            feed_frame(&mut mgr, t, position);
            let pc = point - position;
            let uv = Vector2::new(pc.x / pc.z, pc.y / pc.z);
            mgr.track_feats.database_mut().update_feature(
                100,
                t,
                0,
                (600. * uv.x + 320.) as f32,
                (600. * uv.y + 240.) as f32,
                uv.x as f32,
                uv.y as f32,
            );
        }
        let size = mgr.state.cov.nrows();
        let mut landmark = Landmark::new(FeatRepresentation::Global3D, 100);
        landmark.unique_camera_id = 0;
        landmark.set_from_xyz(&point, false);
        landmark.set_from_xyz(&point, true);
        landmark.set_local_id(size as i32);
        mgr.state.features_slam.insert(100, landmark);
        mgr.state.cov = mgr.state.cov.clone().resize(size + 3, size + 3, 0.);
        for i in size..size + 3 {
            mgr.state.cov[(i, i)] = 0.01;
        }
        let before = mgr.state.cov.clone();
        let feature = mgr
            .track_feats
            .database_mut()
            .get_feature(100, false)
            .unwrap();
        let mut once = mgr.state.clone();
        assert_eq!(
            mgr.updater_slam.update(&mut once, &mut vec![feature]),
            vec![100]
        );
        assert!(once.cov.trace() < before.trace());
        mgr.do_feature_propagate_update(&CameraData {
            timestamp: 5.,
            sensor_ids: vec![0],
            images: vec![],
            masks: vec![],
        });
        assert!(
            (mgr.state.cov.clone() - once.cov).amax() < 1e-12,
            "帧内 SLAM 更新必须等价于单次批量 SLAM 更新，同一测量不得再进 MSCKF"
        );
        assert!(
            mgr.track_feats
                .database_mut()
                .get_feature(100, false)
                .is_none()
        );
    }
}

#[cfg(test)]
mod analytic_contract;
