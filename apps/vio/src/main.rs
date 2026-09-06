//! VIO 进程：MSCKF 估计器（`firefly-vio`）+ iceoryx2 zero-copy 发布。
//!
//! 固定接入 `MuJoCo` 物理环境（Python `firefly-sim`）经 iceoryx2 发布的
//! IMU + 双目灰度（跑完整 MSCKF 视觉更新），输出 odom（估计位姿，100Hz）
//! 到 `Firefly/Odometry`。所有消息的 User Header 自动携带 fastrace trace
//! 上下文（跨进程 span 树可观测）。
//!
//! 运行：`cargo run -p vio`（配合 `uv run firefly-sim` 的 `MuJoCo` 物理环境）。
//! - 瘦版可视化：10Hz 位姿/轨迹（`vio/odom`+`vio/traj` 橙、真值
//!   `gt/pose`+`gt/traj` 蓝，统一 `sim_time` 时间轴），只记位姿不流图像。
//!   经 `Firefly/Viz` 话题发布，`firefly-viz` 进程统一写 rerun（计算线程零 IO）。
//! - GT 仅用于可视化，不进估计器；无 depth 订阅
//! - `node.wait(1ms)` 节拍尽快消费消息，Ctrl-C 优雅退出

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use fastrace::prelude::*;
use firefly_pubsub::event::{CAMERA_PAIR_TOPIC, TopicListener};
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::{GROUND_TRUTH_TOPIC, OdomMessage};
use firefly_pubsub::publish::{ODOM_TOPIC, OdomPublisher};
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::viz::{VizMessage, VizPublisher, kind};
use firefly_vio::options::VioManagerOptions;
use firefly_vio::vio_manager::VioManager;
use firefly_vio_core::cam::{CamRadtan, SharedCamera};
use firefly_vio_core::input::SensorInput;
use firefly_vio_core::track::{HistogramMethod, TrackKlt};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetRunResult;

mod config;
mod input;
use config::VioConfig;

use input::IceoryxInput;

/// `configs/vio.toml` 缺省路径（相对运行目录，通常为仓库根）。
const DEFAULT_CONFIG: &str = "configs/vio.toml";

/// odom 发布周期（秒）：100Hz（IMU propagation 速率，视觉仍 10Hz 修正）。
const ODOM_PERIOD: f64 = 0.01;
/// 可视化发布周期（秒）：10Hz（位姿/轨迹，防 Viz 话题洪泛）。
const VIZ_PERIOD: f64 = 0.1;
/// `MuJoCo` 场景无人机起点（= demo 地图 start；GT 先验）。
const SIM_START: [f64; 3] = [1.0, 4.0, 1.0];
/// rerun 图例颜色：真值=蓝、估计=橙。
const GT_COLOR: (u8, u8, u8) = (60, 120, 255);
const ODOM_COLOR: (u8, u8, u8) = (255, 140, 0);

/// 解析 `--config <path>`（缺省 [`DEFAULT_CONFIG`]）。
fn parse_config_path() -> Result<String, String> {
    let mut it = std::env::args().skip(1);
    let mut path = DEFAULT_CONFIG.to_owned();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--config" => {
                path = it
                    .next()
                    .ok_or_else(|| "missing --config value".to_owned())?;
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(path)
}

#[allow(clippy::too_many_lines)] // 启动编排（标定/初始化/订阅/主循环），结构由进程生命周期驱动
fn main() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    let config_path = parse_config_path().map_err(|e| {
        eprintln!("{e}\n用法：vio [--config configs/vio.toml]");
        std::process::exit(2);
    })?;
    let cfg = VioConfig::load(&config_path)?;
    log::info!(
        "VIO 进程启动：订阅 MuJoCo 物理环境（iceoryx2 输入 + trace 上下文），配置 {config_path}"
    );

    // 相机标定（内参来自配置：MuJoCo 双目 320×240、focal≈168.6、无畸变；
    // 真机换 D430 标定值即可，见 configs/vio.toml）。
    let intrinsics = [
        cfg.camera.focal,
        cfg.camera.focal,
        cfg.camera.width as f64 / 2.0,
        cfg.camera.height as f64 / 2.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ];
    let cam_left: SharedCamera = Arc::new(CamRadtan::new(
        cfg.camera.width,
        cfg.camera.height,
        &intrinsics,
    ));
    let cam_right: SharedCamera = Arc::new(CamRadtan::new(
        cfg.camera.width,
        cfg.camera.height,
        &intrinsics,
    ));

    // IMU 噪声来自配置（须与 sim 注入匹配：σ_gyro=0.002 rad/s、
    // σ_accel=0.02 m/s² 每采样高斯 @200Hz → 连续谱密度 ≈ ×√fs）。直接沿用
    // OpenVINS 真机默认会使 Q 偏小 ~1e4 倍——滤波器过度信任 IMU，静态悬停也
    // 恒加速漂移（实测 20s 内速度漂到 3m/s）。
    let mut params = VioManagerOptions {
        imu_noises: firefly_vio_core::noise::ImuNoise::new(
            cfg.imu.gyro_noise,
            cfg.imu.gyro_walk,
            cfg.imu.accel_noise,
            cfg.imu.accel_walk,
        ),
        // MuJoCo 无真实 IMU 偏置：收紧 bg/ba 先验防视觉误学（真机改回 0.02）
        init_bias_sigma: cfg.estimator.init_bias_sigma,
        ..VioManagerOptions::default()
    };
    params.state_options.num_cameras = 2;
    params.triangulation_options.max_baseline = cfg.estimator.max_baseline;
    // SLAM 默认开（OpenVINS 默认 25）：10 轨迹 bench 相对纯 MSCKF 均值 -8%
    //（8/10 路线占优；logs/bench suite_*_10x1x34s）。KLT 滞后偏置被持久路标
    // 吸收的机制仍在（单轮方差大），体素选点作 opt-in 优化（`[slam]` 开启）。
    params.state_options.max_slam_features = cfg.slam.max_slam_features;
    params.voxel_options.enabled = cfg.slam.voxel_selection;
    params.voxel_options.voxel_size = cfg.slam.voxel_size;
    params.voxel_options.max_points_per_voxel = cfg.slam.max_points_per_voxel;
    params.voxel_options.min_point_distance = cfg.slam.min_point_distance;
    // 零速更新保持 OpenVINS 默认关闭（官方参数 try_zupt=false）：慢速运动下
    // 加速度低于 IMU 噪声 σ 时 chi2 不超限 → 误接受 → 估计位置冻结；
    // 显式零运动分支（官方标注 untested）同样不属默认路径。
    let mut tracker_calib = std::collections::HashMap::new();
    tracker_calib.insert(0usize, cam_left.clone());
    tracker_calib.insert(1usize, cam_right.clone());
    let tracker = TrackKlt::new(
        tracker_calib,
        cfg.frontend.num_pts,
        0,
        true,
        HistogramMethod::None, // 全局直方图均衡实测引入亚像素偏置（残差 -1px → -0.4px）
        20,                    // fast_threshold: OpenVINS 默认值
        5,                     // grid_x: OpenVINS 默认值
        5,                     // grid_y: OpenVINS 默认值
        10,                    // min_px_dist: OpenVINS 默认值
    );
    let mut cameras = BTreeMap::new();
    cameras.insert(0usize, cam_left);
    cameras.insert(1usize, cam_right);
    let mut vio = VioManager::new(params, cameras, tracker);

    // 相机外参（IMU→cam）：R_ItoC + p_IinC（OpenVINS JPL 约定），见
    // [`mujoco_stereo_extrinsic`]。p_IinC = IMU 原点在相机系。
    let (q_ito_c, [p_left_in_c, p_right_in_c]) = mujoco_stereo_extrinsic(cfg.camera.baseline);
    for (cam_id, p_iin_c) in [(0usize, p_left_in_c), (1usize, p_right_in_c)] {
        let calib = vio
            .state
            .calib_imu_to_cam
            .get_mut(&cam_id)
            .expect("num_cameras=2 已建外参槽位");
        calib.set_value(q_ito_c, p_iin_c);
        calib.set_fej(q_ito_c, p_iin_c);
    }

    // 进程共享节点：所有端口由它派生；主循环以 node.wait 驱动，Ctrl-C 优雅
    // 退出并释放全部 IPC 资源（硬杀会留孤儿共享内存 + 幽灵端口注册）
    let node = create_node()?;
    log::info!("iceoryx2 节点已创建（进程共享，信号处理 = HandleTerminationRequests）");

    // odom 发布器（Trace 上下文由中间件在 publish 时自动注入）
    let odom_pub = OdomPublisher::new(&node)?;
    log::info!("已打开话题 {ODOM_TOPIC}");

    // 可视化发布器：经 Firefly/Viz 话题发布，firefly-viz 进程统一写 rerun
    // （计算线程零 IO；发布失败只降级 debug 日志，不影响估计）
    let viz_pub = VizPublisher::new(&node)?;
    log::info!(
        "已打开话题 {VIZ_TOPIC}",
        VIZ_TOPIC = firefly_pubsub::viz::VIZ_TOPIC
    );

    // 输入源：订阅 MuJoCo 物理环境（IMU/双目灰度），imu 话题由 MuJoCo 发布
    let mut input = IceoryxInput::new(&node)?;
    log::info!("输入源：订阅 MuJoCo 物理环境（IMU/双目灰度），imu 话题由 MuJoCo 发布");

    // 真值订阅（仅可视化对比，估计器不读）
    let gt_sub = match Subscriber::<OdomMessage>::with_topic(&node, GROUND_TRUTH_TOPIC) {
        Ok(s) => {
            log::info!("已订阅真值话题 {GROUND_TRUTH_TOPIC}（可视化对比）");
            Some(s)
        }
        Err(e) => {
            log::warn!("真值订阅不可用（无 GT 可视化）：{e}");
            None
        }
    };

    // 真值初始化：等待首条 GT（≤2s），用其位置/速度/姿态对齐真实起点。
    // 不假设 sim 静止起飞（demo 闭环）还是 --script 轨迹（起点速度非零）——
    // 硬编码静止先验会让 odom 与 GT 起点错位（实测 --script 起点 0.94m/s）。
    let mut imustate = [0.0f64; 17];
    imustate[4] = 1.0; // qw（回退：静止水平）
    imustate[5] = SIM_START[0];
    imustate[6] = SIM_START[1];
    imustate[7] = SIM_START[2];
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    let mut got_gt = false;
    while std::time::Instant::now() < deadline
        && let Some(s) = &gt_sub
    {
        let Ok(Some(sample)) = s.receive() else {
            std::thread::sleep(std::time::Duration::from_millis(10));
            continue;
        };
        let m = &*sample;
        imustate[0] = m.timestamp;
        imustate[1] = m.quat_x;
        imustate[2] = m.quat_y;
        imustate[3] = m.quat_z;
        imustate[4] = m.quat_w;
        imustate[5] = m.position_x;
        imustate[6] = m.position_y;
        imustate[7] = m.position_z;
        imustate[8] = m.velocity_x;
        imustate[9] = m.velocity_y;
        imustate[10] = m.velocity_z;
        got_gt = true;
        break;
    }
    if got_gt {
        log::info!(
            "真值初始化：t={:.2} p=({:.2},{:.2},{:.2}) v=({:.2},{:.2},{:.2})",
            imustate[0],
            imustate[5],
            imustate[6],
            imustate[7],
            imustate[8],
            imustate[9],
            imustate[10]
        );
    } else {
        log::info!("无真值，回退静止先验：t=0 ({SIM_START:?})");
    }
    vio.initialize_with_gt(&imustate);

    run_loop(
        &mut vio,
        &mut input,
        &odom_pub,
        &viz_pub,
        gt_sub.as_ref(),
        &node,
    )?;
    Ok(())
}

/// IMU 断流警戒阈值（正常 100Hz，超过视为断流）。
const IMU_STALL: Duration = Duration::from_millis(200);
/// 兜底心跳周期：无事件时也周期醒来（断流警戒/诊断打印/odom 节拍兜底）。
const HEARTBEAT: Duration = Duration::from_millis(100);
/// 诊断打印间隔（墙钟秒）。
const DIAG_PERIOD: f64 = 10.0;

/// 驱动循环（事件驱动）：WaitSet 监听 IMU/相机对事件即到即醒（数据率 ≈ 唤醒
/// 率），100ms 心跳兜底做断流警戒与诊断；SIGINT/SIGTERM 由
/// `WaitSet` 捕获返回 → 优雅退出，所有端口 Drop、IPC 资源释放。
// 输入分发 + 前端健康度统计的编排长流程（与 C++ run_loop 对照结构一致）。
#[allow(clippy::too_many_lines)]
fn run_loop(
    vio: &mut VioManager,
    input: &mut dyn SensorInput,
    odom_pub: &OdomPublisher,
    viz_pub: &VizPublisher,
    gt_sub: Option<&Subscriber<OdomMessage>>,
    node: &firefly_pubsub::node::IpcNode,
) -> Result<(), firefly_error::Error> {
    let mut t_sim = 0.0f64;
    let mut next_odom = 0.0f64;
    let mut next_viz = 0.0f64;
    let mut wake_count = 0u64;
    let mut est_prev: Option<[f64; 3]> = None;
    let mut gt_prev: Option<[f64; 3]> = None;
    let mut camera_count = 0u64;
    let mut imu_batch_count = 0u64;
    let t_wall_start = std::time::Instant::now();
    let mut next_diag_wall = DIAG_PERIOD;

    // 事件唤醒端：IMU + 相机对（notify 来自 sim；odom 由 OdomPublisher 自动通知）
    let imu_events = TopicListener::with_topic(node, firefly_pubsub::imu::IMU_TOPIC)?;
    let cam_events = TopicListener::with_topic(node, CAMERA_PAIR_TOPIC)?;
    let waitset = WaitSetBuilder::new()
        .create::<ipc::Service>()
        .map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::Internal,
                format!("创建 WaitSet 失败: {e:?}"),
            )
        })?;
    let _imu_guard = waitset.attach_notification(&imu_events).map_err(|e| {
        firefly_error::Error::new(
            firefly_error::ErrorKind::Internal,
            format!("挂载 IMU 事件监听失败: {e:?}"),
        )
    })?;
    let _cam_guard = waitset.attach_notification(&cam_events).map_err(|e| {
        firefly_error::Error::new(
            firefly_error::ErrorKind::Internal,
            format!("挂载相机对事件监听失败: {e:?}"),
        )
    })?;
    let tick_guard = waitset.attach_interval(HEARTBEAT).map_err(|e| {
        firefly_error::Error::new(
            firefly_error::ErrorKind::Internal,
            format!("挂载心跳定时器失败: {e:?}"),
        )
    })?;

    // IMU 新鲜度跟踪（断流警戒：告警一次，数据恢复后复位）
    let mut last_imu_wall = Option::<std::time::Instant>::None;
    let mut stall_warned = false;

    let on_event = |attachment_id: WaitSetAttachmentId<ipc::Service>| {
        let heartbeat = attachment_id.has_event_from(&tick_guard);
        // 官方纪律：唤醒后必须排空监听端，否则 fd 持续可读 → busy-loop
        let _ = imu_events.drain();
        let _ = cam_events.drain();
        wake_count += 1;

        // 推进输入源并消费到当前时刻
        input.advance();
        let now = input.now();

        // 帧 trace：续接最近收到的传感器 trace（每周期一条，跨进程同 trace）；
        // 无上游 trace（sim --no-trace / 独立运行）时用未采样 root，不产生
        // span 记录——否则 ConsoleReporter 每次唤醒打印一棵树
        let root = match input.last_trace() {
            Some((tid, sid, sampled)) => Span::root(
                "vio",
                SpanContext::new(TraceId(tid), SpanId(sid)).sampled(sampled),
            ),
            None => Span::root("vio", SpanContext::random().sampled(false)),
        };
        let guard = root.set_local_parent();

        let mut imu_batch = 0u32;
        while let Some(imu) = input.next_imu() {
            vio.feed_measurement_imu(&imu);
            imu_batch += 1;
        }
        if imu_batch > 0 {
            last_imu_wall = Some(std::time::Instant::now());
            stall_warned = false;
        }
        imu_batch_count += u64::from(imu_batch);

        // 相机（MSCKF 视觉更新）+ 前端健康度经 Firefly/Viz 发布
        if let Some(cam) = input.next_camera() {
            camera_count += 1;
            vio.feed_measurement_camera(&cam);
            log::debug!(
                "camera t={:.3} sensors={:?} imgs={}x{}",
                cam.timestamp,
                cam.sensor_ids,
                cam.images.first().map_or(0, |g| g.width),
                cam.images.first().map_or(0, |g| g.height),
            );
            // 前端健康度：统计保留在计算线程，结果经 Firefly/Viz 发布
            // （bar_chart/scalars），firefly-viz 进程统一写 rerun
            {
                let db = vio.track_feats.database();
                let mut hist = vec![0i64; 21];
                let mut total_len = 0usize;
                let mut n_feat = 0usize;
                for f in db.iter_features() {
                    let len: usize = f.timestamps.values().map(Vec::len).sum();
                    let bucket = len.min(hist.len() - 1);
                    hist[bucket] += 1;
                    total_len += len;
                    n_feat += 1;
                }
                let avg_len = if n_feat > 0 {
                    total_len as f64 / n_feat as f64
                } else {
                    0.0
                };
                // BarChart: x=track_length, y=count
                let mut hist_msg =
                    VizMessage::base(kind::BAR_CHART, t_sim, "vio/debug/track_length");
                for (i, &v) in hist.iter().enumerate() {
                    hist_msg.bins[i] = v as u64;
                }
                hist_msg.bin_count = hist.len() as u32;
                hist_msg.bin_start = 0;
                hist_msg.bin_width = 1;
                let _ = viz_pub.publish(hist_msg);

                // Scalars: db_size / avg_len 单值也走 scalars 消息
                for (entity, value) in [
                    ("vio/debug/db_size", db.size() as f64),
                    ("vio/debug/track_avg_len", avg_len),
                ] {
                    let mut msg = VizMessage::base(kind::SCALARS, t_sim, entity);
                    msg.scalars[0] = value;
                    msg.scalar_count = 1;
                    let _ = viz_pub.publish(msg);
                }
                log::debug!(
                    "frontend health t={:.2} db={} avg_len={:.1} hist={:?}",
                    t_sim,
                    db.size(),
                    avg_len,
                    hist
                );
            }
        }
        t_sim = t_sim.max(now);

        // 心跳分支：诊断打印 + IMU 断流警戒
        let wall_s = t_wall_start.elapsed().as_secs_f64();
        if heartbeat && wall_s >= next_diag_wall {
            next_diag_wall = wall_s + DIAG_PERIOD;
            log::info!(
                "[perf-diag] wall={wall_s:.1}s sim={t_sim:.2} wakes={wake_count} cameras={camera_count} imu_batches={imu_batch_count} sim_rate={:.2}x",
                if wall_s > 0.0 { t_sim / wall_s } else { 0.0 }
            );
        }
        if heartbeat && !stall_warned && last_imu_wall.is_some_and(|t| t.elapsed() > IMU_STALL) {
            stall_warned = true;
            log::warn!("IMU 断流 >{IMU_STALL:?}：滤波器停更，等待 sim 恢复");
        }

        // 按发布周期输出 odom（100Hz，经 IMU propagation；视觉仍 10Hz 修正）
        if t_sim + 1e-9 >= next_odom {
            let s = &vio.state;
            publish_odom(odom_pub, t_sim, s.timestamp, &s.imu, vio.initialized());
            next_odom += ODOM_PERIOD;
            // 落后超过一个周期（启动追赶 / 长阻塞后恢复）时重同步到当前时刻，
            // 避免按 100Hz 节奏洪泛补发积压的 odom
            if next_odom + ODOM_PERIOD < t_sim {
                next_odom = t_sim;
            }
        }

        // 瘦版可视化：位姿 + 轨迹折线 @10Hz（只记位姿不流图像）
        if t_sim + 1e-9 >= next_viz {
            let s = &vio.state;
            let p = s.imu.pos();
            let q = s.imu.quat();
            log_viz(
                viz_pub,
                t_sim,
                [p.x, p.y, p.z],
                [q[0], q[1], q[2], q[3]],
                gt_sub,
                &mut est_prev,
                &mut gt_prev,
            );
            next_viz += VIZ_PERIOD;
            if next_viz + VIZ_PERIOD < t_sim {
                next_viz = t_sim;
            }
        }

        // trace 只覆盖真实工作：先闭合本帧 span（时长不含下面的等待）
        drop(guard);
        drop(root);

        CallbackProgression::Continue
    };

    // 事件主循环：SIGINT/SIGTERM 由 WaitSet 捕获并返回
    match waitset.wait_and_process(on_event) {
        Ok(WaitSetRunResult::Interrupt | WaitSetRunResult::TerminationRequest) => {
            log::info!("收到终止信号，优雅退出（端口 Drop、IPC 资源释放）");
        }
        Ok(_) => {}
        Err(e) => {
            return Err(firefly_error::Error::temporary(
                firefly_error::ErrorKind::Internal,
                format!("WaitSet 事件等待失败: {e:?}"),
            ));
        }
    }
    Ok(())
}

/// 组装并发布一条 odom（100Hz，成功打 debug 行；100Hz info 会刷屏）。
fn publish_odom(
    odom_pub: &OdomPublisher,
    t_sim: f64,
    state_t: f64,
    imu: &firefly_vio_types::var::ImuState,
    is_initialized: bool,
) {
    log::debug!(
        "odom-publish state_t={state_t:.3} sim_t={t_sim:.3} pos=({:.3},{:.3},{:.3})",
        imu.pos().x,
        imu.pos().y,
        imu.pos().z,
    );
    let msg = firefly_pubsub::odom::OdomMessage {
        timestamp: t_sim,
        position_x: imu.pos().x,
        position_y: imu.pos().y,
        position_z: imu.pos().z,
        velocity_x: imu.vel().x,
        velocity_y: imu.vel().y,
        velocity_z: imu.vel().z,
        quat_x: imu.quat()[0],
        quat_y: imu.quat()[1],
        quat_z: imu.quat()[2],
        quat_w: imu.quat()[3],
        is_initialized,
    };
    match odom_pub.publish(msg) {
        Ok(ctx) => {
            log::debug!(
                "odom t={t_sim:.2} p=({:.2},{:.2},{:.2}) v=({:.3},{:.3},{:.3}) trace_id={:032x} sampled={}",
                imu.pos().x,
                imu.pos().y,
                imu.pos().z,
                imu.vel().x,
                imu.vel().y,
                imu.vel().z,
                ctx.trace_id(),
                ctx.sampled(),
            );
        }
        Err(e) => log::warn!("odom 发布失败（temporary 可重试）: {e}"),
    }
}

/// 瘦版可视化发布：估计位姿（橙）+ 最新真值样本（蓝），统一 `sim_time`
/// 时间轴，经 [`VIZ_TOPIC`] 由 `firefly-viz` 统一写 rerun。轨迹按官方增量段
/// 写法——同 entity 同一时间轴每次只发「上一帧→本帧」的两点段，由 rerun
/// 沿时间累积拼接成整条折线（存储 O(N)；每帧全量重写整条历史为 O(N²)）。
/// 发布失败只降级 debug 日志，不影响估计。
fn log_viz(
    viz_pub: &VizPublisher,
    t_sim: f64,
    pos: [f64; 3],
    quat_xyzw: [f64; 4],
    gt_sub: Option<&Subscriber<OdomMessage>>,
    est_prev: &mut Option<[f64; 3]>,
    gt_prev: &mut Option<[f64; 3]>,
) {
    let mut pose = VizMessage::base(kind::POSE, t_sim, "vio/odom");
    pose.color = [ODOM_COLOR.0, ODOM_COLOR.1, ODOM_COLOR.2];
    pose.xyz = pos;
    pose.quat_xyzw = quat_xyzw;
    if let Err(e) = viz_pub.publish(pose) {
        log::debug!("viz 发布 odom 位姿失败：{e}");
    }
    if let Some(prev) = *est_prev {
        let mut seg = VizMessage::base(kind::LINE_STRIP, t_sim, "vio/traj");
        seg.color = [ODOM_COLOR.0, ODOM_COLOR.1, ODOM_COLOR.2];
        seg.points[0] = prev;
        seg.points[1] = pos;
        seg.point_count = 2;
        if let Err(e) = viz_pub.publish(seg) {
            log::debug!("viz 发布 vio/traj 段失败：{e}");
        }
    }
    *est_prev = Some(pos);
    let Some(gt) = gt_sub else { return };
    let Ok(Some(sample)) = gt.receive() else {
        return;
    };
    let m = &*sample;
    let gpos = [m.position_x, m.position_y, m.position_z];
    let gquat = [m.quat_x, m.quat_y, m.quat_z, m.quat_w];
    let mut gpose = VizMessage::base(kind::POSE, t_sim, "gt/pose");
    gpose.color = [GT_COLOR.0, GT_COLOR.1, GT_COLOR.2];
    gpose.xyz = gpos;
    gpose.quat_xyzw = gquat;
    if let Err(e) = viz_pub.publish(gpose) {
        log::debug!("viz 发布 gt 位姿失败：{e}");
    }
    if let Some(prev) = *gt_prev {
        let mut seg = VizMessage::base(kind::LINE_STRIP, t_sim, "gt/traj");
        seg.color = [GT_COLOR.0, GT_COLOR.1, GT_COLOR.2];
        seg.points[0] = prev;
        seg.points[1] = gpos;
        seg.point_count = 2;
        if let Err(e) = viz_pub.publish(seg) {
            log::debug!("viz 发布 gt/traj 段失败：{e}");
        }
    }
    *gt_prev = Some(gpos);
}

/// `MuJoCo` 双目相机外参（OpenVINS JPL 约定），返回 `(q_ito_c, [p_left, p_right])`。
///
/// 物理相机（`scene.py` 的 `xyaxes="0 -1 0  0 0 1"`，与 `firefly-map::DepthCamera`
/// 投影一致实测校验）：**前向 = 机体 +x，上 = 机体 +z，右 = 机体 -y**。
/// VIO 三角化视线取 `(nx, ny, 1)`（标准 y-down / z-forward 相机系），故
/// body→camera 旋转 `R_ItoC = [[0,-1,0],[0,0,-1],[1,0,0]]`（列向量为相机系在 body 下的基）。
/// IMU 在两相机中点，基线 `baseline` 米。
///
/// 平移为 **`p_IinC`（IMU 原点在相机系下的坐标）**，非体系杆臂！由期望的
/// 体系横向安装位置 `p_CinI = (0, ±baseline/2, 0)` 换算：
/// `p_IinC = -R_ItoC · p_CinI` ⇒ `(±baseline/2, 0, 0)`。
/// 注意：误填体系值会经 R 旋成沿视线方向的纵向基线——立体视差恒为零，
/// 三角化全部退化。
#[must_use]
fn mujoco_stereo_extrinsic(baseline: f64) -> (nalgebra::Vector4<f64>, [nalgebra::Vector3<f64>; 2]) {
    use firefly_vio_types::quat_ops::rot_2_quat;
    use nalgebra::{Matrix3, Vector3};
    // R_ItoC: body -> camera
    let r_ito_c = Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
    // JPL 四元数 (x, y, z, w)。必须用项目的 rot_2_quat（Trawny Eq.74，与
    // `PoseJpl::rot` 的 quat_2_rot 互逆）——nalgebra 的 UnitQuaternion 是
    // Hamilton 约定，直接喂给 JPL 估计器等效于转置旋转（相机"侧装"、
    // 立体基线变纵向，视觉更新全部退化的根因）。
    let q_vec = rot_2_quat(&r_ito_c);
    // p_IinC = -R_ItoC · p_CinI；以 scene.py 物理安装为准：cam_left 在
    // body -y、cam_right 在 body +y 各 baseline/2 ⇒ p_IinC=(∓baseline/2,0,0)。
    let half = baseline / 2.0;
    let p_left_in_c = Vector3::new(-half, 0.0, 0.0);
    let p_right_in_c = Vector3::new(half, 0.0, 0.0);
    (q_vec, [p_left_in_c, p_right_in_c])
}

#[cfg(test)]
mod tests {
    use super::{DEFAULT_CONFIG, mujoco_stereo_extrinsic};
    use firefly_vio_types::quat_ops::quat_2_rot;
    use nalgebra::Vector3;

    /// 双目外参：body→camera 旋转应使相机前向 = body +x，上 = body +z。
    ///
    /// `quat_2_rot` 给出 body→cam（`v_cam = R·v_body`），故相机轴在体系下
    /// 取 `Rᵀ` 的列（即 R 的行）。
    #[test]
    fn camera_forward_is_body_x() {
        let (q, _) = mujoco_stereo_extrinsic(0.05);
        let r = quat_2_rot(&q);
        // camera z 轴（前向）在 body 下 = Rᵀ·e_z = R 第 2 行
        let cam_fwd = r.transpose() * Vector3::new(0.0, 0.0, 1.0);
        assert!((cam_fwd - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-9);
        // camera y 轴（图像向下）在 body 下 = Rᵀ·e_y = body -z
        let cam_down = r.transpose() * Vector3::new(0.0, 1.0, 0.0);
        assert!((cam_down - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-9);
    }

    /// 基线语义：经 `p_ciinG = p_IinG - R_GtoCi^T · p_IinC` 还原的世界系相机
    /// 位置差必须落在 **横向（body ±y / world ±y）**——纵向基线无立体视差，
    /// 三角化全部退化。
    #[test]
    fn stereo_baseline_is_lateral() {
        let baseline = 0.05;
        let (q_ito_c_vec, [p_left_in_c, p_right_in_c]) = mujoco_stereo_extrinsic(baseline);
        let r_ito_c = quat_2_rot(&q_ito_c_vec);
        // 水平姿态（R_GtoI = I）下的两相机世界位置
        let p_ciin_g = |p_iin_c: &nalgebra::Vector3<f64>| {
            nalgebra::Vector3::new(5.0, 7.0, 2.0) - r_ito_c.transpose() * p_iin_c
        };
        let delta = p_ciin_g(&p_right_in_c) - p_ciin_g(&p_left_in_c);
        assert!(delta.x.abs() < 1e-9, "基线不得有前向分量: {delta}");
        assert!(delta.z.abs() < 1e-9, "基线不得有竖直分量: {delta}");
        assert!(
            (delta.y.abs() - baseline).abs() < 1e-9,
            "基线长度应为 {baseline}m: {delta}"
        );
    }

    /// 缺省配置路径约定（configs/ 统一放仓库顶层）。
    #[test]
    fn default_config_path() {
        assert_eq!(DEFAULT_CONFIG, "configs/vio.toml");
    }
}
