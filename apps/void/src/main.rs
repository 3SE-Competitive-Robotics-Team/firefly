//! void 进程：DIVO 里程计（`firefly-void`）+ iceoryx2 zero-copy 发布。
//!
//! 固定接入 `MuJoCo` 物理环境（Python `firefly-sim`）经 iceoryx2 发布的
//! IMU + 左目灰度 + 深度（跑完整深度-惯性-视觉顺序更新），输出估计位姿
//! （10Hz）到 **`Firefly/VoidOdom`**（与现有 VIO 的 `Firefly/Odometry`
//! 分离，支持 A/B 对比）。所有消息 User Header 自动携带 fastrace trace
//! 上下文（跨进程 span 树可观测，照抄 vio 的 `TraceContext` 用法）。
//!
//! 运行：`cargo run -p void`（配合 `uv run firefly-sim` 的 `MuJoCo` 物理环境）。
//! - 可视化：10Hz 位姿/轨迹/地图点（`void/odom`+`void/traj` 橙、
//!   `void/map_points` 采样点、`void/health` 深度内点/视觉迭代标量），
//!   统一 `sim_time` 时间轴，经 `Firefly/Viz` 话题由 `firefly-viz` 进程
//!   统一写 rerun（计算线程零 IO）。
//! - 初始位姿：`configs/void.toml` 的 `t0`（缺省 `[1.0,4.0,1.0]`，与
//!   `SIM_START` 一致），启动时写入状态。
//! - `node.wait(1ms)` 节拍尽快消费消息，Ctrl-C 优雅退出（端口 Drop，
//!   iceoryx2 无幽灵服务残留）。

use std::time::Duration;

use fastrace::prelude::*;
use firefly_pubsub::event::{CAMERA_PAIR_TOPIC, TopicListener};
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::{GROUND_TRUTH_TOPIC, OdomMessage};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::viz::{POINTS_MAX, VizMessage, VizPublisher, kind};
use firefly_void::options::VoidOptions;
use firefly_void::{FrameInput, Odometry, VoidOdometry};
use firefly_void_types::sensor::{CameraFrame, DepthFrame};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetRunResult;

mod input;
use input::IceoryxInput;

/// `configs/void.toml` 缺省路径（相对运行目录，通常为仓库根）。
const DEFAULT_CONFIG: &str = "configs/void.toml";

/// void odom 发布话题（本地常量：与 VIO 的 `Firefly/Odometry` 分离，支持
/// A/B 对比；不扩 pubsub topic 常量文件）。
pub const VOID_ODOM_TOPIC: &str = "Firefly/VoidOdom";

/// odom 发布周期（秒）。
const ODOM_PERIOD: f64 = 0.1;
/// rerun 图例颜色：void=橙（与 vio 估计一致）、地图点=绿。
const ODOM_COLOR: (u8, u8, u8) = (255, 140, 0);
const MAP_COLOR: (u8, u8, u8) = (80, 220, 120);

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
        eprintln!("{e}\n用法：void [--config configs/void.toml]");
        std::process::exit(2);
    })?;
    let cfg = VoidOptions::load(&config_path)?;
    log::info!(
        "void 进程启动：DIVO 里程计（深度-惯性-视觉），配置 {config_path}，起点 ({:.2},{:.2},{:.2})",
        cfg.t0[0],
        cfg.t0[1],
        cfg.t0[2]
    );

    // 里程计管线（初始位姿来自配置 t0）
    let mut odom = VoidOdometry::new(cfg);

    // 进程共享节点：所有端口由它派生；主循环以 node.wait 驱动，Ctrl-C 优雅
    // 退出并释放全部 IPC 资源（硬杀会留孤儿共享内存 + 幽灵端口注册）
    let node = create_node()?;
    log::info!("iceoryx2 节点已创建（进程共享，信号处理 = HandleTerminationRequests）");

    // 真值订阅（仅启动初始化，估计器运行时不读）
    let gt_sub = match Subscriber::<OdomMessage>::with_topic(&node, GROUND_TRUTH_TOPIC) {
        Ok(s) => {
            log::info!("已订阅真值话题 {GROUND_TRUTH_TOPIC}（启动姿态初始化）");
            Some(s)
        }
        Err(e) => {
            log::warn!("真值订阅不可用（回退水平姿态先验）：{e}");
            None
        }
    };
    // 启动姿态初始化：等待首条 GT（≤2s），用真值姿态对齐世界系。
    // 仅靠水平先验 + t0 时，悬停无人机的微小初始倾斜会被深度/视觉残差
    // 吸收进 bias（bg 漂到 0.007 rad/s），位置随后被拉偏（实测随机 0.2~1.6m）。
    if let Some(gt) = &gt_sub {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while std::time::Instant::now() < deadline {
            let Ok(Some(sample)) = gt.receive() else {
                std::thread::sleep(std::time::Duration::from_millis(10));
                continue;
            };
            let m = &*sample;
            let q = nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                m.quat_w, m.quat_x, m.quat_y, m.quat_z,
            ));
            // 世界系 → 虚拟针孔系：R_wv = R_wb·R_bvᵀ（body_ext 返回 R_bv）
            let r_wb = q.to_rotation_matrix();
            let r_bv = odom
                .body_ext()
                .unwrap_or_else(nalgebra::Rotation3::identity);
            let rot = r_wb * r_bv.inverse();
            // GT 速度随位姿一起初始化：启动时轨迹已有速度，置零会留下
            // x/y 方向不可收敛的初始速度偏差（深度平面法向正交方向无约束）
            let vel = nalgebra::Vector3::new(m.velocity_x, m.velocity_y, m.velocity_z);
            odom.set_initial_pose(
                m.timestamp,
                m.position_x,
                m.position_y,
                m.position_z,
                vel,
                rot,
            );
            log::info!(
                "真值初始化：t={:.2} p=({:.2},{:.2},{:.2}) q=({:.3},{:.3},{:.3},{:.3})",
                m.timestamp,
                m.position_x,
                m.position_y,
                m.position_z,
                m.quat_x,
                m.quat_y,
                m.quat_z,
                m.quat_w
            );
            break;
        }
    }

    // void odom 发布器（带事件唤醒；Trace 上下文由中间件在 publish 时自动注入）
    let odom_pub: Publisher<OdomMessage> = Publisher::with_topic_notify(&node, VOID_ODOM_TOPIC)?;
    log::info!("已打开话题 {VOID_ODOM_TOPIC}");

    // 可视化发布器：经 Firefly/Viz 话题发布，firefly-viz 进程统一写 rerun
    // （计算线程零 IO；发布失败只降级 debug 日志，不影响估计）
    let viz_pub = VizPublisher::new(&node)?;
    log::info!(
        "已打开话题 {VIZ_TOPIC}",
        VIZ_TOPIC = firefly_pubsub::viz::VIZ_TOPIC
    );

    // 输入源：订阅 MuJoCo 物理环境（IMU/左目/深度），imu 话题由 MuJoCo 发布
    let mut input = IceoryxInput::new(&node)?;
    log::info!("输入源：订阅 MuJoCo 物理环境（IMU/左目灰度/深度）");

    run_loop(&mut odom, &mut input, &odom_pub, &viz_pub, &node)?;
    Ok(())
}

/// IMU 断流警戒阈值（正常 100Hz，超过视为断流）。
const IMU_STALL: Duration = Duration::from_millis(200);
/// 兜底心跳周期：无事件时也周期醒来（断流警戒/诊断打印/odom 节拍兜底）。
const HEARTBEAT: Duration = Duration::from_millis(100);
/// 诊断打印间隔（墙钟秒）。
const DIAG_PERIOD: f64 = 10.0;

/// 驱动循环（事件驱动）：WaitSet 监听 IMU/相机对事件即到即醒（数据率 ≈
/// 唤醒率），100ms 心跳兜底做断流警戒与诊断；SIGINT/SIGTERM 由
/// `WaitSet` 捕获返回 → 优雅退出，所有端口 Drop、IPC 资源释放。
// 输入分发 + 前端健康度统计的编排长流程（与 vio 的 run_loop 对照结构一致）。
#[allow(clippy::too_many_lines)]
fn run_loop(
    odom: &mut VoidOdometry,
    input: &mut IceoryxInput,
    odom_pub: &Publisher<OdomMessage>,
    viz_pub: &VizPublisher,
    node: &firefly_pubsub::node::IpcNode,
) -> Result<(), firefly_error::Error> {
    let mut t_sim = 0.0f64;
    let mut next_odom = 0.0f64;
    let mut wake_count = 0u64;
    let mut frame_count = 0u64;
    let mut odom_count = 0u64;
    let mut depth_ok_frames = 0u64;
    let mut visual_ok_frames = 0u64;
    // 先验批次诊断累计（P11.2；kept>0 帧数 + 残差均值/σ² 均值累计）
    let mut prior_active_frames = 0u64;
    let mut prior_kept_sum = 0u64;
    let mut prior_resid_sum = 0.0f64;
    let mut prior_sigma_sum = 0.0f64;
    let mut est_prev: Option<[f64; 3]> = None;
    let t_wall_start = std::time::Instant::now();
    let mut next_diag_wall = DIAG_PERIOD;

    // 事件唤醒端：IMU + 相机对（notify 来自 sim；VoidOdom 由发布器自动通知）
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
                "void",
                SpanContext::new(TraceId(tid), SpanId(sid)).sampled(sampled),
            ),
            None => Span::root("void", SpanContext::random().sampled(false)),
        };
        let guard = root.set_local_parent();

        let mut imu_batch = 0u32;
        while let Some(imu) = input.next_imu() {
            odom.process_imu(&imu);
            imu_batch += 1;
        }
        if imu_batch > 0 {
            last_imu_wall = Some(std::time::Instant::now());
            stall_warned = false;
        }

        // 深度+相机帧配对：同步到达时跑完整一帧（传播→深度→视觉→建图）
        if let Some((cam, dep)) = input.next_frame() {
            frame_count += 1;
            let camera = CameraFrame {
                t: cam.t,
                left_gray: &cam.left_gray,
                width: cam.width,
                height: cam.height,
            };
            let depth = DepthFrame {
                t: dep.t,
                depth: &dep.depth,
                width: dep.width,
                height: dep.height,
            };
            let frame = FrameInput {
                camera: &camera,
                depth: &depth,
            };
            match odom.process_frame(&frame) {
                Ok(out) => {
                    if out.depth_converged {
                        depth_ok_frames += 1;
                    }
                    if out.visual_healthy {
                        visual_ok_frames += 1;
                    }
                    if out.prior_inliers > 0 {
                        prior_active_frames += 1;
                        prior_kept_sum += out.prior_inliers as u64;
                        prior_resid_sum += out.prior_residual_mean;
                        prior_sigma_sum += out.prior_sigma_mean;
                    }
                    log::debug!(
                        "frame t={:.2} inliers={} prior_kept={} prior_resid={:.4} depth_it={} visual_it={} conv=[{} {}]",
                        out.t,
                        out.depth_inliers,
                        out.prior_inliers,
                        out.prior_residual_mean,
                        out.depth_iterations,
                        out.visual_iterations,
                        out.depth_converged,
                        out.visual_healthy
                    );
                    // 健康标量（深度内点数 / 先验内点数 / 视觉迭代数）
                    let mut health = VizMessage::base(kind::SCALARS, out.t, "void/health");
                    health.scalars[0] = out.depth_inliers as f64;
                    health.scalars[1] = out.prior_inliers as f64;
                    health.scalars[2] = out.visual_iterations as f64;
                    health.scalar_count = 3;
                    let _ = viz_pub.publish(health);
                }
                Err(e) => {
                    // 单帧失败不退出（发散/NaN 由 esikf 拦截），恢复下一帧
                    log::warn!("帧处理失败 t={:.2}: {e}", cam.t);
                }
            }
            t_sim = t_sim.max(now);
        }

        // 心跳分支：诊断打印 + IMU 断流警戒
        let wall_s = t_wall_start.elapsed().as_secs_f64();
        if heartbeat && wall_s >= next_diag_wall {
            next_diag_wall = wall_s + DIAG_PERIOD;
            let s = odom.state();
            let p = s.pos;
            let tau = s.inv_expo_time;
            let bg = s.bias_g;
            let ba = s.bias_a;
            let rate = if wall_s > 0.0 { t_sim / wall_s } else { 0.0 };
            // 先验批次平均 kept/残差/σ²（kept 才是物理有效指标，见
            // docs/void-motion-drift.md：depth_ok 是误导性指标）
            let (prior_avg_kept, prior_avg_resid, prior_avg_sigma) = if prior_active_frames > 0 {
                (
                    prior_kept_sum as f64 / prior_active_frames as f64,
                    prior_resid_sum / prior_active_frames as f64,
                    prior_sigma_sum / prior_active_frames as f64,
                )
            } else {
                (0.0, 0.0, 0.0)
            };
            log::info!(
                "[perf-diag] wall={wall_s:.1}s sim={t_sim:.2} wakes={wake_count} frames={frame_count} \
                 depth_ok={depth_ok_frames} visual_ok={visual_ok_frames} odom={odom_count} \
                 prior_kept_avg={prior_avg_kept:.0} prior_resid_avg={prior_avg_resid:.4} prior_sigma_avg={prior_avg_sigma:.5} \
                 pos=({:.3},{:.3},{:.3}) tau={tau:.3} bg=({:.4},{:.4},{:.4}) ba=({:.3},{:.3},{:.3}) \
                 sim_rate={rate:.2}x",
                p.x,
                p.y,
                p.z,
                bg.x,
                bg.y,
                bg.z,
                ba.x,
                ba.y,
                ba.z
            );
        }
        if heartbeat && !stall_warned && last_imu_wall.is_some_and(|t| t.elapsed() > IMU_STALL) {
            stall_warned = true;
            log::warn!("IMU 断流 >{IMU_STALL:?}：滤波器停更，等待 sim 恢复");
        }

        // 按发布周期输出 void odom
        if t_sim + 1e-9 >= next_odom {
            odom_count += 1;
            let s = odom.state();
            publish_odom(odom_pub, t_sim, s, odom);
            // 可视化：位姿 + 轨迹折线 + 地图点采样 @10Hz（位姿用机体系，
            // 与 odom 发布一致）
            {
                let p = s.pos;
                let r_bv = odom
                    .body_ext()
                    .unwrap_or_else(nalgebra::Rotation3::identity);
                let q = nalgebra::UnitQuaternion::from_rotation_matrix(&(s.rot * r_bv));
                log_viz(
                    viz_pub,
                    t_sim,
                    [p.x, p.y, p.z],
                    [q.i, q.j, q.k, q.w],
                    odom,
                    &mut est_prev,
                );
            }
            next_odom += ODOM_PERIOD;
            // 落后超过一个周期（启动追赶 / 长阻塞后恢复）时重同步到当前时刻，
            // 避免按 10Hz 节奏洪泛补发积压的 odom
            if next_odom + ODOM_PERIOD < t_sim {
                next_odom = t_sim;
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

/// 组装并发布一条 void odom（10Hz，`Firefly/VoidOdom`）。
///
/// 姿态转换到机体系：滤波器状态 `rot` 为虚拟针孔系（`R_wv`，见
/// `firefly-void/src/options.rs`），发布 `R_wb = R_wv·R_bv`；位置/速度
/// 已是世界系（与 `GroundTruth` 同框），直接发布。
fn publish_odom(
    odom_pub: &Publisher<OdomMessage>,
    t_sim: f64,
    state: &firefly_void_types::state::State,
    odom: &VoidOdometry,
) {
    let r_bv = odom
        .body_ext()
        .unwrap_or_else(nalgebra::Rotation3::identity);
    let q = nalgebra::UnitQuaternion::from_rotation_matrix(&(state.rot * r_bv));
    let msg = OdomMessage {
        timestamp: t_sim,
        position_x: state.pos.x,
        position_y: state.pos.y,
        position_z: state.pos.z,
        velocity_x: state.vel.x,
        velocity_y: state.vel.y,
        velocity_z: state.vel.z,
        quat_x: q.i,
        quat_y: q.j,
        quat_z: q.k,
        quat_w: q.w,
        is_initialized: true,
    };
    match odom_pub.publish(msg) {
        Ok(ctx) => {
            log::info!(
                "odom t={t_sim:.2} p=({:.2},{:.2},{:.2}) v=({:.3},{:.3},{:.3}) trace_id={:032x} sampled={}",
                state.pos.x,
                state.pos.y,
                state.pos.z,
                state.vel.x,
                state.vel.y,
                state.vel.z,
                ctx.trace_id(),
                ctx.sampled(),
            );
        }
        Err(e) => log::warn!("odom 发布失败（temporary 可重试）: {e}"),
    }
}

/// 可视化发布：估计位姿（橙）+ 轨迹折线 + 地图点采样（绿），统一
/// `sim_time` 时间轴，经 [`VIZ_TOPIC`] 由 `firefly-viz` 统一写 rerun。
/// 轨迹按官方增量段写法——同 entity 同一时间轴每次只发「上一帧→本帧」的
/// 两点段，由 rerun 沿时间累积拼接成整条折线。地图点为当前采样（≤512），
/// 点集用 `LINE_STRIP` 承载（viz kind 无 POINTS，LineStrips 渲染等价位姿点）。
/// 发布失败只降级 debug 日志，不影响估计。
#[allow(clippy::too_many_arguments)] // 位姿/四元数/里程计/历史状态的标准可视化参数集
fn log_viz(
    viz_pub: &VizPublisher,
    t_sim: f64,
    pos: [f64; 3],
    quat_xyzw: [f64; 4],
    odom: &VoidOdometry,
    est_prev: &mut Option<[f64; 3]>,
) {
    let mut pose = VizMessage::base(kind::POSE, t_sim, "void/odom");
    pose.color = [ODOM_COLOR.0, ODOM_COLOR.1, ODOM_COLOR.2];
    pose.xyz = pos;
    pose.quat_xyzw = quat_xyzw;
    if let Err(e) = viz_pub.publish(pose) {
        log::debug!("viz 发布 void/odom 位姿失败：{e}");
    }
    if let Some(prev) = *est_prev {
        let mut seg = VizMessage::base(kind::LINE_STRIP, t_sim, "void/traj");
        seg.color = [ODOM_COLOR.0, ODOM_COLOR.1, ODOM_COLOR.2];
        seg.points[0] = prev;
        seg.points[1] = pos;
        seg.point_count = 2;
        if let Err(e) = viz_pub.publish(seg) {
            log::debug!("viz 发布 void/traj 段失败：{e}");
        }
    }
    *est_prev = Some(pos);

    // 地图点采样（≤512，viz POINTS_MAX 上限）
    let pts = odom.map_points(POINTS_MAX);
    if !pts.is_empty() {
        let mut msg = VizMessage::base(kind::LINE_STRIP, t_sim, "void/map_points");
        msg.color = [MAP_COLOR.0, MAP_COLOR.1, MAP_COLOR.2];
        let n = pts.len().min(POINTS_MAX);
        for (i, p) in pts.iter().take(n).enumerate() {
            msg.points[i] = [p.x, p.y, p.z];
        }
        msg.point_count = n as u32;
        if let Err(e) = viz_pub.publish(msg) {
            log::debug!("viz 发布 void/map_points 失败：{e}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 缺省配置路径约定（configs/ 统一放仓库顶层）。
    #[test]
    fn default_config_path() {
        assert_eq!(DEFAULT_CONFIG, "configs/void.toml");
    }

    /// void odom 话题名与现有 VIO 的 `OdomTopic` 分离（A/B 对比前提）。
    #[test]
    fn void_odom_topic_is_distinct() {
        assert_eq!(VOID_ODOM_TOPIC, "Firefly/VoidOdom");
        assert_ne!(VOID_ODOM_TOPIC, firefly_pubsub::publish::ODOM_TOPIC);
    }
}
