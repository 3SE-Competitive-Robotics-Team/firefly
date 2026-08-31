//! DIVO 应用接口层：Odometry trait 与管线组装（P4）。
//!
//! 技术蓝本为 FAST-LIVO2 `src/LIVMapper.cpp` 的
//! `stateEstimationAndMapping` 主循环：scan 重组合 → 传播 → 深度更新 →
//! 视觉更新 → 建图（DESIGN.md §4）。本 crate 定义 [`Odometry`] trait 与
//! [`VoidOdometry`] 实现，供 `apps/void` 接线。
//!
//! 坐标约定：估计器运行在**虚拟针孔相机系**（`imu_to_cam_rot = I`）。
//! `MuJoCo` 深度相机为 OpenGL 系（前向 `-z_cam`、图 y 向上），左目为
//! `xyaxes="0 -1 0  0 0 1"`（前向 `+x_body`、上 `+z_body`、右 `-y_body`）；
//! 两步旋转合成 `R = R_cam_to_body · R_body_to_cam`（见
//! [`options::default_depth_ext`]）后，深度像素与左目针孔像素逐点重合，
//! 深度测量与视觉测量共用同一相机系（外参单位阵）。

use std::collections::VecDeque;

use firefly_void_esikf::propagator::Propagator;
use firefly_void_esikf::update::EskfUpdater;
use firefly_void_map::VoxelMap;
use firefly_void_map::options::VoxelMapOptions;
use firefly_void_measure::options::{DepthOptions, VisualOptions};
use firefly_void_measure::plane_update::DepthMeasurement;
use firefly_void_measure::visual_update::VisualMeasurement;
use firefly_void_types::sensor::{CameraFrame, DepthFrame, ImuSample};
use firefly_void_types::state::{DIM_STATE, State};
use firefly_void_types::visual::{GrayImage, Intrinsics, VisualState};
use nalgebra::{Isometry3, Matrix3, Rotation3, UnitQuaternion, Vector3};

pub mod options;

pub use options::{DepthConfig, MapConfig, PropagationNoiseConfig, VisualConfig, VoidOptions};

/// 传感器帧输入：深度帧与相机帧**同步到达**（同一时刻 10Hz 对齐，
/// 由接线层配对）。
pub struct FrameInput<'a> {
    /// 左目灰度帧（虚拟针孔相机系）。
    pub camera: &'a CameraFrame<'a>,
    /// 深度帧（OpenGL 系原始数据，内部反投影）。
    pub depth: &'a DepthFrame<'a>,
}

/// 单帧处理输出（P4 健康统计 / 各阶段耗时 trace 用）。
pub struct OdometryOutput {
    /// 帧时刻（仿真秒）。
    pub t: f64,
    /// 当前估计位姿（全局系 = 首帧 IMU 系）。
    pub pose: Isometry3<f64>,
    /// 当前估计速度（全局系，m/s）。
    pub velocity: Vector3<f64>,
    /// 状态协方差对角（19 维）。
    pub covariance_diag: nalgebra::SVector<f64, DIM_STATE>,
    /// 深度测量实际有效点（卡方/退化过滤后）。
    pub depth_inliers: usize,
    /// 深度测量迭代收敛迭代数（0 表示无有效测量）。
    pub depth_iterations: usize,
    /// 视觉测量迭代总迭代数（0 表示无可视点）。
    pub visual_iterations: usize,
    /// 视觉更新健康：有可见地图点且迭代收敛。
    pub visual_healthy: bool,
    /// 深度更新收敛与否。
    pub depth_converged: bool,
    /// 地图视觉点数（viz 采样用）。
    pub map_visual_points: usize,
    /// 本帧被门控拒绝的更新数（0/1/2，深度+视觉；健康统计 `rejected_updates`）。
    pub rejected_updates: usize,
    /// 累计被门控拒绝的更新数（进程生命周期）。
    pub rejected_total: usize,
    /// 各阶段耗时（秒）。
    pub timings: FrameTimings,
}

/// 单帧各阶段耗时（秒，trace/健康面板用）。
#[derive(Debug, Clone, Copy, Default)]
pub struct FrameTimings {
    /// IMU 前向传播。
    pub propagate: f64,
    /// 深度点云反投影 + 下采样。
    pub depth_preprocess: f64,
    /// 深度测量 ESIKF 更新。
    pub depth_update: f64,
    /// 视觉测量金字塔更新。
    pub visual_update: f64,
    /// 地图几何/视觉更新。
    pub map_update: f64,
}

/// 里程计抽象：算法实现与 iceoryx2 接线分离。
pub trait Odometry {
    /// 馈入一次 IMU 测量（内部缓存，帧到达时统一传播）。
    fn process_imu(&mut self, imu: &ImuSample);

    /// 馈入一帧深度+图像测量（传播后执行深度/视觉顺序更新）。
    ///
    /// # Errors
    /// 深度/视觉测量构造或更新失败（`InvalidArgument`/`Convergence`）。
    fn process_frame(&mut self, frame: &FrameInput<'_>) -> firefly_error::Result<OdometryOutput>;

    /// 当前估计状态。
    fn state(&self) -> &State;
}

/// DIVO 完整管线：ESIKF + 体素地图 + 两个测量模型。
pub struct VoidOdometry {
    esikf: Propagator,
    map: VoxelMap,
    options: VoidOptions,
    /// 当前状态（含协方差）。
    state: State,
    /// 待传播的 IMU 队列（`process_imu` 缓存，帧到达时按时间戳消费）。
    imu_queue: VecDeque<ImuSample>,
    /// 上一帧时刻（首帧初始化基准）。
    last_frame_t: Option<f64>,
    /// 帧计数器（视觉地图点增补判据）。
    frame_id: u32,
    /// 被门控拒绝的累计更新数（深度+视觉；健康统计）。
    rejected_updates: usize,
}

impl VoidOdometry {
    /// 构造管线。
    #[must_use]
    pub fn new(options: VoidOptions) -> Self {
        // 初始位置（全局系 = 首帧 IMU 系，`configs/void.toml` 的 t0）；
        // 重力初值（`MuJoCo` 世界系 `0 0 -9.81`，scene.py；重力估计开启时
        // 首帧前向传播即可收敛）
        let body_ext = options.body_ext_isometry();
        let r_wb0 = Rotation3::identity(); // 仿真初始水平（scene.py 单位四元数）
        // 初始姿态 `R_wv(0) = R_wb·R_bvᵀ`（世界 → 虚拟针孔系；imu 角速度/
        // 比力经 `R_bv` 转到虚拟系后进入传播）
        let rot0 = r_wb0 * body_ext.rotation.inverse().to_rotation_matrix();
        let state = State {
            pos: Vector3::new(options.t0[0], options.t0[1], options.t0[2]),
            rot: rot0,
            gravity: Vector3::new(0.0, 0.0, -9.81),
            ..State::default()
        };
        let mut esikf = Propagator::new().with_noise((&options.imu).into());
        // 仿真固定曝光：禁用 τ 估计（τ 恒 1；随机游走实测会把位置拉偏）
        if !options.estimate_exposure {
            esikf.disable_exposure_est();
        }
        let map = VoxelMap::new((&options.map).into());
        Self {
            esikf,
            map,
            options,
            state,
            imu_queue: VecDeque::new(),
            last_frame_t: None,
            frame_id: 0,
            rejected_updates: 0,
        }
    }

    /// 世界系 → 虚拟针孔相机系位姿（`^C T_G`，与 `VisualMeasurement` 的
    /// `cam_pose_from_state` 一致）。
    fn cam_pose(&self) -> Isometry3<f64> {
        let r_wi = self.state.rot.matrix();
        let p_wi = self.state.pos;
        let r_cw = r_wi.transpose();
        let p_cw = -r_cw * p_wi;
        Isometry3::from_parts(
            nalgebra::Translation3::from(p_cw),
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(r_cw)),
        )
    }

    /// 深度图反投影 + 体素下采样。
    ///
    /// 反投影用 OpenGL 系约定：`p_cam = (dx·z, dy·z, -z)`，
    /// `dx = (u−cx)/f`、`dy = (v−cy)/f`（对照 `firefly-map::DepthCamera`，
    /// 深度值为相机空间 Z）。随后经 [`VoidOptions::depth_ext_rot`] 转到
    /// **虚拟针孔 IMU 系**，协方差同系（`DepthNoise` 对旋转不变，取
    /// `R_glv·Σ·R_glvᵀ`）。输出即虚拟系点云——P3 的 [`DepthMeasurement`]
    /// 以单位外参消费（点已就位，避免双重旋转）。
    ///
    /// 下采样：`downsample_voxel`（默认 0.5m）体素网格内保留深度不确定度
    /// 最大（`σ_z` 最大 = 距离最远）的点，控制单帧点数（320×240 → 数百）。
    #[must_use]
    fn build_downsampled_cloud(
        &self,
        depth: &DepthFrame<'_>,
        intrinsics: Intrinsics,
    ) -> (Vec<Vector3<f64>>, Vec<Matrix3<f64>>) {
        let opts = &self.options.depth;
        let ext = self.options.depth_ext_isometry();
        let r_glv = ext.rotation.to_rotation_matrix().into_inner();
        let noise = firefly_void_measure::DepthNoise::from_intrinsics(
            &DepthOptions::from(opts),
            intrinsics.fx,
            intrinsics.fy,
        );
        // 体素网格：键 → (点, 协方差, 距离不确定度)
        let mut grid: std::collections::HashMap<[i64; 3], (Vector3<f64>, Matrix3<f64>, f64)> =
            std::collections::HashMap::new();
        let inv_cell = 1.0 / opts.downsample_voxel;
        let depth_data = depth.depth;
        let width = depth.width;
        let height = depth.height;
        // 深度边缘跳变阈值（m）：剔除仿真边缘膨胀 1px 的前景点
        // （深度不连续处前景像素被背景深度覆盖，系统偏大——边缘点会
        // 把平面拟合往外推，产生系统位置偏差）
        let edge_thresh = 0.15;
        for y in 0..height {
            for x in 0..width {
                let z = depth_data[y * width + x];
                if z <= 0.05 || z > opts.max_range || !z.is_finite() {
                    continue;
                }
                // 邻域深度跳变剔除（边缘膨胀点）
                let mut edge = false;
                for (du, dv) in [(-1i64, 0i64), (1, 0), (0, -1), (0, 1)] {
                    let (nx, ny) = (x as i64 + du, y as i64 + dv);
                    if nx < 0 || ny < 0 || nx >= width as i64 || ny >= height as i64 {
                        continue;
                    }
                    let nz = depth_data[ny as usize * width + nx as usize];
                    if nz > 0.05 && nz.is_finite() && (z - nz).abs() > edge_thresh {
                        edge = true;
                        break;
                    }
                }
                if edge {
                    continue;
                }
                let dx = (x as f64 - intrinsics.cx) / intrinsics.fx;
                let dy = (y as f64 - intrinsics.cy) / intrinsics.fy;
                // OpenGL 系（深度值为相机空间 Z）→ 虚拟针孔系
                let p_gl = Vector3::new(dx * z, dy * z, -z);
                let p_v = firefly_void_map::voxel::transform_point(&ext, &p_gl);
                let cov_gl = noise.point_covariance(&p_gl);
                let cov = r_glv * cov_gl * r_glv.transpose();
                let sigma_z = noise.range_sigma(p_gl.norm());
                let key = [
                    (p_v[0] * inv_cell).floor() as i64,
                    (p_v[1] * inv_cell).floor() as i64,
                    (p_v[2] * inv_cell).floor() as i64,
                ];
                match grid.get_mut(&key) {
                    Some(entry) => {
                        // 保留深度不确定度最小（近、σ∝z² 小）的点：远点
                        // σ 爆炸会主导平面拟合，产生随机系统偏差
                        if sigma_z < entry.2 {
                            *entry = (p_v, cov, sigma_z);
                        }
                    }
                    None => {
                        grid.insert(key, (p_v, cov, sigma_z));
                    }
                }
            }
        }
        let mut points = Vec::with_capacity(grid.len());
        let mut covs = Vec::with_capacity(grid.len());
        for (p, cov, _) in grid.into_values() {
            points.push(p);
            covs.push(cov);
        }
        (points, covs)
    }

    /// 地图视觉点采样（≤ `max_points`，viz `map_points` 实体用）。
    #[must_use]
    pub fn map_points(&self, max_points: usize) -> Vec<Vector3<f64>> {
        let pts = self.map.visual_points();
        let step = (pts.len().max(1)).div_ceil(max_points);
        pts.iter()
            .step_by(step)
            .take(max_points)
            .map(|vp| vp.pos)
            .collect()
    }

    /// 地图统计（viz/健康面板）。
    #[must_use]
    pub fn map_stats(&self) -> (usize, usize) {
        (self.map.root_count(), self.map.visual_point_count())
    }

    /// 真实机体 → 虚拟系旋转 `R_bv`（供接线层做 GT 姿态初始化）。
    #[must_use]
    pub fn body_ext(&self) -> Option<nalgebra::Rotation3<f64>> {
        let e = self.options.body_ext_isometry();
        Some(e.rotation.to_rotation_matrix())
    }

    /// 用真值覆盖初始位姿（仅启动时调用一次；此后估计器独立运行）。
    ///
    /// `rot` 为世界系 → 虚拟系的姿态（`R_wb·R_bvᵀ`）。位置与速度同时
    /// 归零（悬停起步）。
    pub fn set_initial_pose(
        &mut self,
        t: f64,
        x: f64,
        y: f64,
        z: f64,
        rot: nalgebra::Rotation3<f64>,
    ) {
        self.state.rot = rot;
        self.state.pos = Vector3::new(x, y, z);
        self.state.vel = Vector3::zeros();
        self.last_frame_t = Some(t);
    }
}

impl Odometry for VoidOdometry {
    fn process_imu(&mut self, imu: &ImuSample) {
        // 真实机体 → 虚拟针孔系：`ω_v = R_bv·ω_b`、`a_v = R_bv·a_b`
        // （P3 视觉模型与深度测量都在虚拟系，IMU 传播必须同系）
        let r_bv = self.options.body_ext_isometry().rotation;
        let omega_v = r_bv * imu.omega;
        let acc_v = r_bv * imu.acc;
        self.imu_queue.push_back(ImuSample {
            t: imu.t,
            omega: omega_v,
            acc: acc_v,
        });
    }

    // 帧处理编排（传播→深度→视觉→建图四阶段），结构由管线顺序驱动
    #[allow(clippy::too_many_lines)]
    fn process_frame(&mut self, frame: &FrameInput<'_>) -> firefly_error::Result<OdometryOutput> {
        let t0 = std::time::Instant::now();
        let frame_t = frame.depth.t.max(frame.camera.t);
        self.frame_id += 1;
        // 本帧被门控拒绝的更新数（深度+视觉，健康统计输出）
        let mut frame_rejected = 0usize;

        // 1. 前向传播：消费到帧时刻为止的 IMU（梯形积分，逐段传播）
        let propagate = {
            let mut t_imu = self.last_frame_t.unwrap_or(frame_t);
            let mut prev: Option<ImuSample> = None;
            while let Some(sample) = self.imu_queue.front() {
                if sample.t > frame_t + 1e-9 {
                    break;
                }
                let s = self.imu_queue.pop_front().expect("front checked");
                if let Some(p) = prev {
                    let dt = (s.t - p.t).max(1e-4);
                    let omega_avr = (p.omega + s.omega) * 0.5;
                    let acc_avr = (p.acc + s.acc) * 0.5;
                    self.esikf
                        .propagate(&mut self.state, omega_avr, acc_avr, dt);
                    t_imu = s.t;
                }
                prev = Some(s);
            }
            if let Some(p) = prev {
                // 传播到帧时刻（帧内最后一个 IMU 到帧时刻的残差段）
                let dt = (frame_t - t_imu).max(1e-4);
                if dt > 1e-3 {
                    self.esikf.propagate(&mut self.state, p.omega, p.acc, dt);
                }
            }
            t0.elapsed().as_secs_f64()
        };

        // 2. 深度点云反投影 + 下采样
        let depth_opts = DepthOptions::from(&self.options.depth);
        let vis_opts = VisualOptions::from(&self.options.visual);
        // 相机内参：MuJoCo fovy=70.88°、320×240 → f≈168.607（scene.py/env.py）
        let intrinsics = Intrinsics::new(168.607, 168.607, 160.0, 120.0);
        let t_pre = std::time::Instant::now();
        let (points_l, covs_l) = self.build_downsampled_cloud(frame.depth, intrinsics);
        let px_total = frame.depth.width * frame.depth.height;
        // 临时诊断：有效像素统计 + 采样 key 分布
        let valid_px = frame
            .depth
            .depth
            .iter()
            .filter(|&&z| z > 0.05 && z.is_finite())
            .count();
        log::debug!(
            "downsample: {} 点（有效像素 {valid_px}/{px_total}，voxel={}）",
            points_l.len(),
            self.options.depth.downsample_voxel
        );
        let depth_preprocess = t_pre.elapsed().as_secs_f64();

        // 3. 深度测量 ESIKF 更新（迭代收敛）
        // 点云已在虚拟针孔系：DepthMeasurement 以单位外参消费
        // （P3 内部 `p_b = ext·p_l`、`cov_w = R·cov·Rᵀ`，虚拟系下一致）
        let t_depth = std::time::Instant::now();
        let (depth_inliers, depth_iterations, depth_converged) = {
            let model = DepthMeasurement::new(
                &self.map,
                points_l.clone(),
                covs_l.clone(),
                nalgebra::Isometry3::identity(),
                depth_opts,
            );
            // 健康统计：update 前的有效点计数（传播先验下）
            let inliers = model.effective_count(&self.state);
            let before = self.state;
            let mut state = before;
            let mut updater = EskfUpdater::new(model, 5, 1.5e-4);
            let iterations = updater
                .update(&mut state)
                .map_err(|e| e.with_context("stage", "depth"))?;
            // 更新门控：单帧深度更新把状态打飞时（典型：新平面刚建立，
            // 初始点少导致平面参数不准，位置跳 0.5m）拒绝并回滚到传播
            // 先验——被拒后由 IMU 传播维持状态，平面随后续帧成熟后
            // 深度残差自然收敛，跳变帧不污染状态
            let gate = &self.options.update_gate;
            let dp = (state.pos - before.pos).norm();
            let drot = before.rot.rotation_to(&state.rot).angle();
            let accepted = dp < gate.max_pos_delta && drot < gate.max_rot_delta;
            if accepted {
                self.state = state;
            } else {
                self.rejected_updates += 1;
                frame_rejected += 1;
                log::warn!("深度更新拒绝：dp={dp:.3}m drot={drot:.3}rad（新平面未收敛/测量退化）");
            }
            (inliers, iterations, iterations < 5 && inliers > 0)
        };
        let depth_update = t_depth.elapsed().as_secs_f64();

        // 4. 视觉测量金字塔更新（预热期内只跑深度，状态稳定后再建视觉点）
        let t_visual = std::time::Instant::now();
        let warmup = self.frame_id <= self.options.visual_warmup_frames;
        let (visual_iterations, visual_healthy) = if warmup {
            (0usize, false)
        } else {
            // 可见视觉地图点（含光线投射补漏）
            let cam_pose = self.cam_pose();
            let mut points = self.map.visible_map_points(&cam_pose, &intrinsics, &[]);
            // 光线投射补漏（未占据网格；视觉点刚建立时可见数少）
            if points.len() < 60 {
                let (cols, rows) = VoxelMapOptions::from(&self.options.map).grid_dims(320, 240);
                let occupied = Self::occupied_grid(&points, cols, rows, &cam_pose, &intrinsics);
                self.map
                    .raycast_visual_points(&cam_pose, &intrinsics, &occupied, &mut points);
            }
            log::debug!(
                "visual: {} 可见点（vpts={}）",
                points.len(),
                self.map.visual_point_count()
            );
            if points.is_empty() {
                (0usize, false)
            } else {
                let image = GrayImage::new(
                    frame.camera.width,
                    frame.camera.height,
                    frame.camera.left_gray.to_vec(),
                );
                let depth = Some((frame.depth.depth, frame.depth.width, frame.depth.height));
                let before = self.state;
                let mut state = before;
                let iterations = VisualMeasurement::pyramid_update(
                    &image, &points, depth, intrinsics, vis_opts, &mut state,
                )
                .map_err(|e| e.with_context("stage", "visual"))?;
                // 仿真固定曝光：估计关闭时强制 τ = 1（视觉残差无曝光自由度）
                if !self.options.estimate_exposure {
                    state.inv_expo_time = 1.0;
                }
                // 更新门控：视觉参考补丁误匹配时会产生大状态跳变
                // （实测 22s 处位置瞬跳 0.4m + 速度爆），拒绝并回滚——
                // 深度+IMU 仍可维持，错误视觉更新丢弃（参数与深度门控
                // 对齐，另加速度通道；见 [`options::UpdateGateConfig`]）
                let gate = &self.options.update_gate;
                let dp = (state.pos - before.pos).norm();
                let dvel = (state.vel - before.vel).norm();
                let drot = before.rot.rotation_to(&state.rot).angle();
                let accepted = dp < gate.max_pos_delta
                    && dvel < gate.max_vel_delta
                    && drot < gate.max_rot_delta;
                if accepted {
                    self.state = state;
                    (iterations, iterations > 0)
                } else {
                    self.rejected_updates += 1;
                    frame_rejected += 1;
                    log::warn!(
                        "视觉更新拒绝：dp={dp:.3}m dvel={dvel:.3} drot={drot:.3}rad（参考补丁误匹配）"
                    );
                    (0usize, false)
                }
            }
        };
        let visual_update = t_visual.elapsed().as_secs_f64();

        // 5. 地图几何 + 视觉更新（建图预热期内跳过注册：前 N 帧纯 IMU
        // 传播——首帧位姿注册的平面若偏，会被深度残差固化成稳态偏置）
        let t_map = std::time::Instant::now();
        let map_warmup = self.frame_id <= self.options.map_warmup_frames;
        if !map_warmup {
            // 深度点云注册：虚拟系点云 → 世界系（更新后位姿）
            let rot = self.state.rot.matrix();
            let (points_w, covs_w): (Vec<_>, Vec<_>) = points_l
                .iter()
                .zip(&covs_l)
                .map(|(p, cov)| (rot * p + self.state.pos, rot * cov * rot.transpose()))
                .unzip();
            self.map.register_points(&points_w, &covs_w);
            // 视觉地图点更新（预热期后开启：参考补丁需在状态稳定时建立）
            if !warmup {
                let image = GrayImage::new(
                    frame.camera.width,
                    frame.camera.height,
                    frame.camera.left_gray.to_vec(),
                );
                let cam_pose = self.cam_pose();
                let vstate = VisualState::new(self.frame_id, self.state.inv_expo_time);
                self.map
                    .update_visual(&cam_pose, &image, &intrinsics, &vstate);
            }
            // 滑窗检查
            self.map.on_update_end(&self.state.pos);
        }
        let map_update = t_map.elapsed().as_secs_f64();

        self.last_frame_t = Some(frame_t);
        let pose = Isometry3::from_parts(
            nalgebra::Translation3::from(self.state.pos),
            UnitQuaternion::from_rotation_matrix(&self.state.rot),
        );
        let (map_roots, map_vpts) = self.map_stats();
        let map_planes = self.map.planes().count();
        log::debug!(
            "void frame t={frame_t:.2} inliers={depth_inliers} depth_it={depth_iterations} \
             visual_it={visual_iterations} map_roots={map_roots} planes={map_planes} vpts={map_vpts} \
             pos=({:.2},{:.2},{:.2})",
            self.state.pos[0],
            self.state.pos[1],
            self.state.pos[2],
        );

        Ok(OdometryOutput {
            t: frame_t,
            pose,
            velocity: self.state.vel,
            covariance_diag: self.state.cov.diagonal(),
            depth_inliers,
            depth_iterations,
            visual_iterations,
            visual_healthy,
            depth_converged,
            map_visual_points: map_vpts,
            rejected_updates: frame_rejected,
            rejected_total: self.rejected_updates,
            timings: FrameTimings {
                propagate,
                depth_preprocess,
                depth_update,
                visual_update,
                map_update,
            },
        })
    }

    fn state(&self) -> &State {
        &self.state
    }
}

impl VoidOdometry {
    /// 可见点投影占据网格（`raycast_visual_points` 的 `occupied_grid` 输入）。
    fn occupied_grid(
        points: &[firefly_void_map::visual_point::VisualPointView],
        cols: usize,
        rows: usize,
        cam_pose: &Isometry3<f64>,
        intrinsics: &Intrinsics,
    ) -> Vec<bool> {
        let mut occupied = vec![false; cols * rows];
        for p in points {
            let p_cam = firefly_void_map::voxel::transform_point(cam_pose, &p.pos);
            if p_cam[2] <= 0.0 {
                continue;
            }
            let Some(px) = intrinsics.project(&p_cam) else {
                continue;
            };
            if px[0] < 0.0 || px[1] < 0.0 {
                continue;
            }
            let col = (px[0] as usize / 30).min(cols - 1);
            let row = (px[1] as usize / 30).min(rows - 1);
            occupied[row * cols + col] = true;
        }
        occupied
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_void_types::visual::Intrinsics;
    use nalgebra::Vector3;

    /// 合成平面深度帧（z=2m 平面，像素逐点反投影）。
    fn plane_depth_frame(z: f64) -> (Vec<f64>, DepthFrame<'static>) {
        let width = 320;
        let height = 240;
        let mut depth = vec![0.0f64; width * height];
        for y in 0..height {
            for x in 0..width {
                depth[y * width + x] = z;
            }
        }
        let t = 1.0;
        let frame = DepthFrame {
            t,
            depth: Box::leak(depth.clone().into_boxed_slice()),
            width,
            height,
        };
        (depth, frame)
    }

    #[test]
    fn downsample_keeps_one_point_per_voxel() {
        let o = VoidOptions::default();
        let odom = VoidOdometry::new(o);
        let intrinsics = Intrinsics::new(168.607, 168.607, 160.0, 120.0);
        let (_depth, frame) = plane_depth_frame(2.0);
        let (points, covs) = odom.build_downsampled_cloud(&frame, intrinsics);
        assert_eq!(points.len(), covs.len());
        // 0.5m 体素 + 4m 上限：320×240 全有效 ≈ 76.8k 点 → 数十点
        assert!(
            points.len() < 500,
            "downsampled {} should be small",
            points.len()
        );
        assert!(
            points.len() > 30,
            "downsampled {} should be non-trivial",
            points.len()
        );
        // 全部点应在 z≈2 平面附近（OpenGL → 虚拟系转换后 z 为正）
        for p in &points {
            assert!((p[2] - 2.0).abs() < 0.2, "p={p}");
        }
        // 协方差对称正定
        for c in &covs {
            assert!((c - c.transpose()).norm() < 1e-10);
            assert!(c.determinant() > 0.0);
        }
    }

    #[test]
    fn depth_ext_rot_maps_opengl_to_pinhole() {
        let o = VoidOptions::default();
        let ext = o.depth_ext_isometry();
        // OpenGL 正前方 (0,0,-5) → 虚拟针孔前向 (0,0,5)
        let p = firefly_void_map::voxel::transform_point(&ext, &Vector3::new(0.0, 0.0, -5.0));
        assert!((p - Vector3::new(0.0, 0.0, 5.0)).norm() < 1e-12);
        // OpenGL 上方（图 y 向上）→ 针孔 y 向下
        let p2 = firefly_void_map::voxel::transform_point(&ext, &Vector3::new(0.1, 0.2, -1.0));
        assert!((p2 - Vector3::new(0.1, -0.2, 1.0)).norm() < 1e-12);
    }

    /// 悬停一致性：水平姿态（`R_wb=I`）下，比力 `a_b=(0,0,9.81)` 经 `R_bv`
    /// 转到虚拟系、再经 `R_wv(0)=R_bvᵀ` 回到世界系，与重力抵消 → 零加速度。
    #[test]
    fn hover_gravity_consistency() {
        let o = VoidOptions::default();
        let odom = VoidOdometry::new(o);
        // 初始姿态 R_wv(0) = R_bvᵀ（世界水平）
        let r_wv = odom.state.rot;
        let rot_bv = odom.options.body_ext_isometry().rotation;
        // 悬停比力（真实机体系 +z，抵消重力）
        let acc_b = Vector3::new(0.0, 0.0, 9.81);
        let acc_v = rot_bv * acc_b;
        let acc_world = r_wv * acc_v + odom.state.gravity;
        assert!(
            acc_world.norm() < 1e-9,
            "悬停加速度应为零：{acc_world}（r_wv={r_wv} rot_bv={rot_bv}）"
        );
        // 外参一致性：R_wv(0) = R_bvᵀ ⇒ R_wv · R_bv = I
        let chain = r_wv * rot_bv.to_rotation_matrix();
        assert!((chain.matrix() - nalgebra::Matrix3::identity()).norm() < 1e-9);
    }
}
