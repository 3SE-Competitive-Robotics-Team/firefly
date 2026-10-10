//! VIO 管理器（对照 `OpenVINS` `ov_msckf/core/VioManager.cpp/.h`）。
//!
//! 编排 IMU/相机输入：
//! - [`VioManager::feed_measurement_imu`]：喂给传播器与初始化器缓冲；
//! - [`VioManager::feed_measurement_camera`]：跟踪 → 初始化/传播+增广 →
//!   MSCKF/SLAM 更新；
//! - [`VioManager::initialize_with_gt`]：真值初始化（仅供合成算法评测夹具）；
//! - [`VioManager::try_to_initialize`]：静态/动态初始化（`firefly-vio-init`）。
//!
//! 裁剪（对照 C++ 超出范围的部分）：ARUCO 跟踪器（`max_aruco_features=0`）、
//! 降采样、统计文件与可视化。

use firefly_vio_core::feat::Feature;
use firefly_vio_core::imu_model::ImuCalibration;
use firefly_vio_core::propagation::{LinearizationPoint, MeanState, Propagator};
use firefly_vio_core::sensor::{CameraData, ImuData};
use firefly_vio_core::track::TrackKlt;
use firefly_vio_types::var::Variable as _;
use firefly_voxel_svio::{
    ActiveTrackTriangulator, IntersectOptions, RayObservation, VoxelKey, VoxelMap,
};
use nalgebra::{DMatrix, Vector2, Vector3};
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
    /// 体素地图（`voxel_options.enabled` 时与 `state.features_slam` 同步，
    /// 为 SLAM 更新做可见体素选点；关闭时全程空置）。
    pub voxel_map: VoxelMap,
    /// 上一帧活跃轨迹命中的可见体素（对照 `getRecentVoxel`：每帧末刷新，
    /// 供下一帧 SLAM 选点；仅在 `voxel_options.enabled` 时维护）。
    recent_voxels: Vec<VoxelKey>,
    /// 活跃轨迹跨帧射线交会累积器（对照 `active_feat_linsys_*`；仅在
    /// `voxel_options.enabled` 时使用）。
    active_tracks: ActiveTrackTriangulator,
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
        let init_options = params.init_options.clone();
        let voxel_map = VoxelMap::new(params.voxel_options.clone());
        let active_tracks = ActiveTrackTriangulator::new(IntersectOptions {
            max_cond_number: params.triangulation_options.max_cond_number,
            min_dist: params.triangulation_options.min_dist,
            max_dist: params.triangulation_options.max_dist,
        });
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
            voxel_map,
            recent_voxels: Vec::new(),
            active_tracks,
            initializer: firefly_vio_init::inertial_init::InertialInitializer::new(init_options),
            updater_zero_velocity,
            has_moved_since_zero_vel: false,
            is_initialized_vio: false,
            startup_time: -1.0,
            timelastupdate: -1.0,
            last_prop_time_offset: None,
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

        // 9. SLAM 更新集来源（对照官方 `featureUpdate` 的 voxel 循环）：开启体素
        // 选点 → 遍历上一帧算出的可见体素，每体素取块首 map point；关闭 → 遍历
        // 全部 SLAM 路标（官方无开关，此为 firefly 的关闭语义）。对每个被访问的
        // 路标：feature 在库则加入更新集；feature 缺失且其 host 相机在本帧 → 标记
        // 边缘化；`update_fail_count > 1` → 标记边缘化。标记只作用于被访问的路标，
        // 不可见的路标不因此被丢弃（官方语义）。
        let visited: Vec<usize> = if self.params.voxel_options.enabled {
            self.voxel_map.select(&self.recent_voxels)
        } else {
            self.state.features_slam.keys().copied().collect()
        };
        for featid in visited {
            let feat2 = self.track_feats.database_mut().get_feature(featid, false);
            if let Some(f2) = &feat2 {
                feats_slam.push(f2.clone());
            }
            let Some(lm) = self.state.features_slam.get_mut(&featid) else {
                continue;
            };
            let host_cam_in_frame = message.sensor_ids.contains(&lm.unique_camera_id);
            if feat2.is_none() && host_cam_in_frame {
                lm.should_marg = true;
            }
            if lm.update_fail_count > 1 {
                lm.should_marg = true;
            }
        }

        // 10. 边缘化所有标记 `should_marg` 的 SLAM 特征（对照官方
        // `marginalizeSlam`：在选点循环之后执行）。
        let marged = marginalize_slam(&mut self.state);
        if self.params.voxel_options.enabled {
            for id in marged {
                let _ = self.voxel_map.remove_point(id);
            }
        }

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
            "SLAM 选点 t={:.2} 路标={} 可见体素={} 更新={} 延迟={}",
            self.state.timestamp,
            self.state.features_slam.len(),
            self.recent_voxels.len(),
            feats_slam_update.len(),
            feats_slam_delayed.len()
        );

        // 11b. 体素选点（对照 Voxel-SVIO `featureUpdate` 的选点段；仅开启时）：
        // 对上一帧末算出的可见体素（`recent_voxels`）取每体素**块首**路标的 id，
        // 保留其中仍在待更新集里的特征；无可见体素则本帧无路标更新
        // （官方行为，无回落）。落选特征保留在库中延后更新（不标记删除）。
        if self.params.voxel_options.enabled && !feats_slam_update.is_empty() {
            let selected = self.voxel_map.select(&self.recent_voxels);
            log::debug!(
                "体素选点 t={:.2} 候选={} 可见体素={} 选中={} 索引={}点/{}体素",
                self.state.timestamp,
                feats_slam_update.len(),
                self.recent_voxels.len(),
                selected.len(),
                self.voxel_map.num_points(),
                self.voxel_map.num_voxels()
            );
            let keep: std::collections::HashSet<usize> = selected.into_iter().collect();
            feats_slam_update.retain(|f| keep.contains(&f.featid));
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
        // 列表递减可终止，处理后不回填）。Rust 端 update 操作的是特征克隆，
        // 消费标记（to_delete）须按返回值写回数据库——对照 C++ 中
        // `UpdaterSLAM::update` 直接在库内特征上置位（「Delete it so we do
        // not reuse information」）：不标记则同一测量会被 marg/max-track
        // 查询再次取用（MSCKF 双重消费 + 同一更新批次内重复入列）。
        let max_slam_in_update = self.state.options.max_slam_in_update;
        while !feats_slam_update.is_empty() {
            let take = max_slam_in_update.min(feats_slam_update.len());
            let mut batch: Vec<Feature> = feats_slam_update.drain(..take).collect();
            let consumed = self.updater_slam.update(&mut self.state, &mut batch);
            self.propagator.invalidate_cache();
            self.track_feats.database_mut().mark_deleted(consumed);
            // 对照官方：每批 SLAM 更新后把所有路标迁到当前体素
            // （`changeHostVoxel`）。
            self.sync_voxel_positions();
        }

        // 15. SLAM 延迟初始化（对照 C++）
        let delayed_ids: Vec<usize> = feats_slam_delayed.iter().map(|f| f.featid).collect();
        if !feats_slam_delayed.is_empty() {
            self.updater_slam
                .delayed_init(&mut self.state, &mut feats_slam_delayed);
            self.propagator.invalidate_cache();
            // 对照官方 `delayedInit`：新路标必须成功入体素图，否则丢弃该路标。
            // 官方此时已把变量加入协方差却不写 `map_points`（孤儿变量），这里改为
            // 回滚边缘化：语义等同且不留孤儿。
            if self.params.voxel_options.enabled {
                for id in &delayed_ids {
                    if let Some(lm) = self.state.features_slam.get(id) {
                        let p = lm.get_xyz(false);
                        if !self.voxel_map.add_point(*id, &p)
                            && let Some(lm) = self.state.features_slam.get_mut(id)
                        {
                            lm.should_marg = true;
                        }
                    }
                }
                for id in marginalize_slam(&mut self.state) {
                    let _ = self.voxel_map.remove_point(id);
                }
            }
            self.track_feats.database_mut().mark_deleted(delayed_ids);
        }

        //=====================================================================
        // 清理与边缘化（对照 C++ 552-596）
        //=====================================================================

        // 16. 刷新可见体素缓存（对照 `triangulateActiveTracks` + `getRecentVoxel`，
        // 官方在 delayedInit 之后、清理之前调用，且只在左目触发帧执行）：查询源是
        // tracker 的 last obs，不读特征库，故不受本帧清理影响。
        if message.sensor_ids.first() == Some(&0) {
            self.refresh_recent_voxels();
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

    /// 把 `state.features_slam` 中已在体素图里的路标迁到当前体素（对照官方
    /// `changeHostVoxel`：EKF 更新后路标会移动，跨体素自动搬家）。
    fn sync_voxel_positions(&mut self) {
        if !self.params.voxel_options.enabled {
            return;
        }
        for (id, lm) in &self.state.features_slam {
            if self.voxel_map.contains(*id) {
                let _ = self.voxel_map.update_point(*id, &lm.get_xyz(false));
            }
        }
    }

    /// 刷新可见体素缓存（对照 Voxel-SVIO `triangulateActiveTracks` +
    /// `getRecentVoxel`，`stereoVio.cpp:627-895`）。
    ///
    /// 用 tracker 的上一帧观测（`last_ids`/`last_points`，不经特征库，故不受
    /// 本帧清理影响）对非 SLAM 特征做跨帧射线交会（[`ActiveTrackTriangulator`]），
    /// 产出世界坐标；再用它们标记 ±`neighbor_radius` 内的可见体素，供下一帧
    /// 选点。仅在 `voxel_options.enabled` 时维护；关闭时缓存保持为空。
    #[fastrace::trace]
    fn refresh_recent_voxels(&mut self) {
        if !self.params.voxel_options.enabled {
            return;
        }
        let timestamp = self.state.timestamp;
        let clones_cam = crate::updater_slam::build_clones_cam(&self.state);
        let slam_ids: std::collections::HashSet<usize> =
            self.state.features_slam.keys().copied().collect();
        let mut observations: Vec<RayObservation> = Vec::new();
        for cam_id in self
            .state
            .calib_imu_to_cam
            .keys()
            .copied()
            .collect::<Vec<_>>()
        {
            let (Some(cam), Some(clones)) =
                (self.state.cameras.get(&cam_id), clones_cam.get(&cam_id))
            else {
                continue;
            };
            let Some(clone) = clones
                .iter()
                .find(|(t, _)| t.total_cmp(&timestamp).is_eq())
                .map(|(_, c)| c)
            else {
                continue;
            };
            let sensor = cam_id as i32;
            let ids = self.track_feats.last_ids(sensor);
            let pts = self.track_feats.last_points(sensor);
            for (id, kp) in ids.iter().zip(pts.iter()) {
                // 官方跳过 `map_points`（SLAM 路标不作为查询源）。
                if slam_ids.contains(id) {
                    continue;
                }
                let uv_norm = cam.undistort_d(Vector2::new(f64::from(kp.x), f64::from(kp.y)));
                observations.push(RayObservation {
                    featid: *id,
                    uv_norm,
                    rot_g_to_c: clone.rot,
                    pos_c_in_g: clone.pos,
                });
            }
        }
        let queries: Vec<Vector3<f64>> = self
            .active_tracks
            .update(&observations)
            .into_iter()
            .map(|(_, p_g)| p_g)
            .collect();
        self.recent_voxels = self.voxel_map.recent_voxels(&queries, timestamp);
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
    fn voxel_selection_defaults_off_and_wires() {
        // 默认关闭：行为与改动前一致，体素索引全程空置。
        let mgr = test_manager();
        assert!(!mgr.params.voxel_options.enabled);
        assert_eq!(mgr.voxel_map.num_points(), 0);
        // 开启后索引可用：收录 → 可见查询 → 每体素限量选点。
        let mut params = VioManagerOptions::default();
        params.voxel_options.enabled = true;
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
        let mut mgr = VioManager::new(params, cameras, tracker);
        assert!(mgr.voxel_map.add_point(1, &Vector3::new(0.05, 0.0, 3.0)));
        assert!(mgr.voxel_map.add_point(2, &Vector3::new(5.0, 0.0, 3.0)));
        let recent = mgr
            .voxel_map
            .recent_voxels(&[Vector3::new(0.05, 0.0, 3.0)], 1.0);
        assert_eq!(mgr.voxel_map.select(&recent), vec![1]);
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

    /// 体素场景：相机 0 内参 `fx=fy=600`、`cx=320`、`cy=240`，外参为单位位姿
    /// （相机系 = IMU 系平移）。供整体输入输出测试注入跟踪观测与路标。
    fn voxel_manager() -> VioManager {
        use firefly_vio_core::cam::{CamRadtan, SharedCamera};
        use std::sync::Arc;
        let mut params = VioManagerOptions::default();
        params.voxel_options.enabled = true;
        // 同格多路标场景需要允许同体素多点；解析夹具点距 0.5~1m，体素与邻域
        // 按官方 EuRoC/TUM-VI 调参（0.3 / ±2）。
        params.voxel_options.min_point_distance = 0.0;
        params.voxel_options.voxel_size = 0.3;
        params.voxel_options.neighbor_radius = 2;
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

    /// 推进一帧：设置 IMU 位姿、增广克隆、写入相机 0 的 last obs（世界点 `p_g`），
    /// 并刷新可见体素缓存（与流水线一致，逐帧累积）。累积器需覆盖 4 帧才产出
    /// （对照官方 `count > 3`）。
    fn feed_voxel_frame(
        mgr: &mut VioManager,
        t: f64,
        p_imu: Vector3<f64>,
        tracks: &[(usize, Vector3<f64>)],
    ) {
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
        let mut pts = Vec::new();
        let mut ids = Vec::new();
        for (id, p_g) in tracks {
            let pc = p_g - p_imu;
            let (un, vn) = (pc.x / pc.z, pc.y / pc.z);
            pts.push(firefly_vio_core::track::KeyPoint::new(
                (600.0 * un + 320.0) as f32,
                (600.0 * vn + 240.0) as f32,
            ));
            ids.push(*id);
        }
        mgr.track_feats.set_last_observations(0, pts, ids);
        mgr.refresh_recent_voxels();
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

    /// 体素选点整体输入输出（对照帧末 `triangulateActiveTracks` +
    /// `getRecentVoxel` → 下帧 `select`）：last obs → 跨帧交会 → 命中体素 → 块首选点。
    #[test]
    fn voxel_pipeline_marks_visible_voxel_and_selects_front_point() {
        let mut mgr = voxel_manager();
        let p_vis = Vector3::new(0.55, 0.25, 3.05);
        assert!(mgr.voxel_map.add_point(100, &p_vis));
        for (i, p_imu) in four_views().iter().enumerate() {
            feed_voxel_frame(&mut mgr, 1.0 + i as f64, *p_imu, &[(7, p_vis)]);
        }
        assert!(
            mgr.recent_voxels.contains(&mgr.voxel_map.key_of(&p_vis)),
            "可见体素未命中：{:?}",
            mgr.recent_voxels
        );
        assert_eq!(mgr.voxel_map.select(&mgr.recent_voxels), vec![100]);
    }

    /// 不足四帧不产出（对照官方 `count > 3`）：可见体素保持为空。
    #[test]
    fn voxel_pipeline_needs_four_frames() {
        let mut mgr = voxel_manager();
        let p = Vector3::new(0.55, 0.25, 3.05);
        assert!(mgr.voxel_map.add_point(100, &p));
        for (i, p_imu) in four_views().iter().take(3).enumerate() {
            feed_voxel_frame(&mut mgr, 1.0 + i as f64, *p_imu, &[(7, p)]);
        }
        assert_eq!(mgr.voxel_map.num_points(), 1);
        assert_eq!(mgr.recent_voxels, [] as [firefly_voxel_svio::VoxelKey; 0]);
    }

    /// 视野外路标不进入可见体素（查询源是活跃轨迹，不是地图点）。
    #[test]
    fn voxel_pipeline_excludes_out_of_view_landmark() {
        let mut mgr = voxel_manager();
        let p_vis = Vector3::new(0.55, 0.25, 3.05);
        let p_far = Vector3::new(50.05, 0.05, 3.05);
        assert!(mgr.voxel_map.add_point(100, &p_vis));
        assert!(mgr.voxel_map.add_point(200, &p_far));
        for (i, p_imu) in four_views().iter().enumerate() {
            feed_voxel_frame(&mut mgr, 1.0 + i as f64, *p_imu, &[(7, p_vis)]);
        }
        assert!(mgr.recent_voxels.contains(&mgr.voxel_map.key_of(&p_vis)));
        assert!(!mgr.recent_voxels.contains(&mgr.voxel_map.key_of(&p_far)));
    }

    /// 同体素多路标时只取块首（对照官方 `points.front()`）。
    #[test]
    fn voxel_pipeline_selects_front_point_per_voxel() {
        let mut mgr = voxel_manager();
        let p = Vector3::new(0.55, 0.25, 3.05);
        assert!(mgr.voxel_map.add_point(100, &p));
        assert!(mgr.voxel_map.add_point(101, &p));
        for (i, p_imu) in four_views().iter().enumerate() {
            feed_voxel_frame(&mut mgr, 1.0 + i as f64, *p_imu, &[(7, p)]);
        }
        assert_eq!(mgr.voxel_map.select(&mgr.recent_voxels), vec![100]);
    }

    /// 关闭时 `refresh_recent_voxels` 全程空转，缓存保持为空。
    #[test]
    fn voxel_pipeline_is_noop_when_disabled() {
        let mut mgr = test_manager();
        assert!(!mgr.params.voxel_options.enabled);
        mgr.refresh_recent_voxels();
        assert_eq!(mgr.recent_voxels, [] as [firefly_voxel_svio::VoxelKey; 0]);
    }
}

#[cfg(test)]
mod analytic_contract;
