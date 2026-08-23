//! planner 进程：任务执行（`firefly_planner::PlannerManager` 驱动）+ IPC
//! 接线 + rerun 可视化。
//!
//! 状态源 = **VIO odom**（`Firefly/Odometry`，新鲜度超时回退轨迹参考推进）；
//! 深度感知建图的位姿同源。真值不参与状态链路（vio 进程侧仅作对比可视化）。
//!
//! 运行：`cargo run -p planner`（配合 `uv run firefly-sim` + `cargo run -p vio`），
//! 或 `cargo run -p planner -- --map apps/planner/maps/gate.ffmap` 独立运行。
//!
//! rerun 实体约定（`sim_time` 时间轴）：`plan/global_path`、`plan/local_traj`
//! （+`velocity`）、`plan/planes`、`plan/drone`（位姿）、`plan/perceived`
//! （感知占据地图）、`plan/motions`（动态障碍）、`plan/map`+`plan/decor`
//! （静态先验，启动时一次性记录）。

mod scene;

use std::collections::HashSet;
use std::path::PathBuf;
use std::time::Duration;

use fastrace::prelude::*;
use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{DepthCamera, MapFile, VoxelState, update_from_depth};
use firefly_observability::init as init_observability;
use firefly_planner::{ManagerOptions, PlannerConfig, PlannerManager};
use firefly_pubsub::camera::{DEPTH_TOPIC, DepthImageMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::reference::{REFERENCE_TOPIC, ReferenceMessage};
use firefly_pubsub::subscriber::{OdomSubscriber, Subscriber};
use firefly_viewer::Viewer;
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetRunResult;
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};

use crate::scene::{human_voxels, mujoco_map_file, parse_vec3};

/// 主循环频率（官方 `exec_timer` 0.1s）。
const LOOP_PERIOD: Duration = Duration::from_millis(100);
/// odom 新鲜度阈值（秒）：超过该时长未收到 odom 则回退轨迹推进估计。
const ODOM_FRESH_TIMEOUT: f64 = 1.0;
/// 感知占据地图的 viewer 更新周期（帧；10Hz 循环下约每 2.5s）。
const PERCEIVED_PERIOD: usize = 25;

struct Args {
    map: Option<PathBuf>,
    save: Option<PathBuf>,
    start: [f64; 3],
    goal: [f64; 3],
    frame_offset: [f64; 3],
}

fn parse_args() -> Result<Args> {
    let mut it = std::env::args().skip(1);
    let mut args = Args {
        map: None,
        save: None,
        start: [1.0, 4.0, 1.0],
        goal: [7.0, 4.8, 1.0],
        frame_offset: [0.0, 0.0, 0.0],
    };
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--map" => {
                args.map = Some(PathBuf::from(it.next().ok_or_else(|| {
                    Error::new(ErrorKind::InvalidArgument, "missing --map value")
                })?));
            }
            "--save" => {
                args.save = Some(PathBuf::from(it.next().ok_or_else(|| {
                    Error::new(ErrorKind::InvalidArgument, "missing --save value")
                })?));
            }
            "--start" => args.start = parse_vec3(&mut it, "--start")?,
            "--goal" => args.goal = parse_vec3(&mut it, "--goal")?,
            "--frame-offset" => args.frame_offset = parse_vec3(&mut it, "--frame-offset")?,
            other => {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    format!("unknown argument {other}"),
                ));
            }
        }
    }
    Ok(args)
}

/// 最新里程计快照（地图系状态 + 姿态四元数，深度投影用）。
struct OdomSnapshot {
    state: firefly_planner::State,
    quat_xyzw: [f64; 4],
}

struct App {
    manager: PlannerManager,
    viewer: Viewer,
    map_file: MapFile,
    /// 静态占据体素（动态障碍不得清掉它们）。
    static_occupied: HashSet<[usize; 3]>,
    /// 上一帧动态障碍占据体素。
    prev_dyn: Vec<[usize; 3]>,
    /// 仿真时钟（秒）。**唯一权威 = 收到的 odom 时间戳**；无消息时按 tick
    /// 本地递增回退。所有计算/viewer 时间轴都用它，与 vio/仿真对齐。
    t_sim: f64,
    sensor_this_tick: bool,
    odom: Option<OdomSubscriber>,
    depth: Option<Subscriber<DepthImageMessage>>,
    latest_odom: Option<OdomSnapshot>,
    last_odom_recv: f64,
    /// 最新 odom 携带的 trace 上下文 `(trace_id, span_id, sampled)`（续接用）。
    odom_trace: Option<(u128, u64, bool)>,
    latest_depth: Option<DepthImageMessage>,
    ref_pub: Option<Publisher<ReferenceMessage>>,
    depth_cam: DepthCamera,
    frame_offset: Vector3<f64>,
    finished: bool,
}

impl App {
    /// 接线：地图 → planner → 管理器 → 订阅/发布端口。
    ///
    /// # Errors
    ///
    /// 地图体素化 / 全局路径搜索 / IPC 端口创建失败。
    fn new(
        map_file: MapFile,
        viewer: Viewer,
        config: PlannerConfig,
        start: [f64; 3],
        goal: [f64; 3],
        frame_offset: [f64; 3],
    ) -> Result<Self> {
        let grid = map_file.to_grid_map()?;
        let static_occupied = map_file
            .occupied
            .iter()
            .filter_map(|p| grid.index_of(Vector3::new(p[0], p[1], p[2])))
            .collect();
        let planner = firefly_planner::Planner::new(config, grid);
        let manager = PlannerManager::with_planner(
            planner,
            ManagerOptions::default(),
            Vector3::new(start[0], start[1], start[2]),
            Vector3::new(goal[0], goal[1], goal[2]),
        )?;
        // 进程共享节点：所有端口由它派生，进程退出时统一 Drop 释放 IPC 资源
        let node = create_node()?;
        // 订阅 VIO 输出（vio 进程未启动时降级为 None，保持独立运行）
        let odom = match OdomSubscriber::new(&node) {
            Ok(s) => {
                log::info!("已订阅 odom 话题（VIO 状态源）");
                Some(s)
            }
            Err(e) => {
                log::warn!("odom 订阅不可用，回退轨迹推进估计：{e}");
                None
            }
        };
        let depth = match Subscriber::<DepthImageMessage>::with_topic(&node, DEPTH_TOPIC) {
            Ok(s) => Some(s),
            Err(e) => {
                log::warn!("深度订阅不可用，感知建图停用：{e}");
                None
            }
        };
        let ref_pub = match Publisher::<ReferenceMessage>::with_topic(&node, REFERENCE_TOPIC) {
            Ok(p) => Some(p),
            Err(e) => {
                log::warn!("参考发布不可用：{e}");
                None
            }
        };
        log::info!("状态源：odom（新鲜度 {ODOM_FRESH_TIMEOUT}s）；真值不参与规划链路");
        Ok(Self {
            manager,
            viewer,
            static_occupied,
            prev_dyn: Vec::new(),
            map_file,
            t_sim: 0.0,
            sensor_this_tick: false,
            odom,
            depth,
            latest_odom: None,
            last_odom_recv: f64::NEG_INFINITY,
            odom_trace: None,
            latest_depth: None,
            ref_pub,
            depth_cam: DepthCamera::mujoco_default(),
            frame_offset: Vector3::new(frame_offset[0], frame_offset[1], frame_offset[2]),
            finished: false,
        })
    }

    /// 排空 odom/深度订阅：续接 trace span、锚定 sim 时钟、记录最新快照。
    fn poll_sensors(&mut self) -> Result<()> {
        if let Some(sub) = &self.odom {
            while let Some(sample) = sub.receive()? {
                let ctx = *sample.user_header();
                // 跨进程 trace 续接：本 span 的父即 vio 发布端 span
                let _span = ctx.continue_span("recv-odom");
                if ctx.is_traced() {
                    self.odom_trace = Some((ctx.trace_id(), ctx.span_id, ctx.sampled()));
                }
                let m: OdomMessage = *sample;
                // 锚定 sim 时钟到 odom 时间戳（vio 的 odom 用 MuJoCo sim 时钟）
                self.t_sim = self.t_sim.max(m.timestamp);
                self.last_odom_recv = m.timestamp;
                self.sensor_this_tick = true;
                let p = Vector3::new(m.position_x, m.position_y, m.position_z) + self.frame_offset;
                self.latest_odom = Some(OdomSnapshot {
                    state: firefly_planner::State {
                        position: Point3::from(p),
                        velocity: Vector3::new(m.velocity_x, m.velocity_y, m.velocity_z),
                        acceleration: Vector3::zeros(),
                    },
                    quat_xyzw: [m.quat_x, m.quat_y, m.quat_z, m.quat_w],
                });
            }
        }
        if let Some(sub) = &self.depth {
            while let Some(sample) = sub.receive()? {
                let ctx = *sample.user_header();
                let _span = ctx.continue_span("recv-depth");
                self.latest_depth = Some(*sample);
            }
        }
        Ok(())
    }

    /// 新鲜 odom 的规划系状态（超时返回 `None`，管理器回退轨迹推进估计）。
    fn measured(&self, now: f64) -> Option<firefly_planner::State> {
        if now - self.last_odom_recv >= ODOM_FRESH_TIMEOUT {
            return None;
        }
        self.latest_odom.as_ref().map(|o| o.state)
    }

    /// 深度 → 占据体素（感知建图）：位姿源与状态源同源（VIO odom）。
    fn update_map_from_depth(&mut self) {
        let (Some(depth), Some(odom)) = (&self.latest_depth, &self.latest_odom) else {
            return;
        };
        let pos = odom.state.position.coords;
        let q = odom.quat_xyzw;
        let quat = UnitQuaternion::from_quaternion(Quaternion::new(q[3], q[0], q[1], q[2]));
        let pose = Isometry3::from_parts(Translation3::new(pos.x, pos.y, pos.z), quat);
        update_from_depth(self.manager.map_mut(), &self.depth_cam, &pose, &depth.data);
    }

    /// 动态障碍按仿真时钟插值，增量更新占据地图（静态体素保护）。
    fn update_motion(&mut self) {
        if self.map_file.motions.is_empty() {
            return;
        }
        let dyn_voxels = self.map_file.motion_voxels(self.t_sim, self.manager.map());
        let map = self.manager.map_mut();
        for idx in &self.prev_dyn {
            if !self.static_occupied.contains(idx) {
                map.set_state(*idx, VoxelState::Unknown);
            }
        }
        for idx in &dyn_voxels {
            map.set_state(*idx, VoxelState::Occupied);
        }
        self.prev_dyn = dyn_voxels;
    }

    /// 主循环单步：传感器 → 感知/动态地图 → 管理器 tick → 参考发布 + 可视化。
    #[allow(clippy::too_many_lines)]
    fn step(&mut self) -> Result<()> {
        // 先消费传感器（更新权威仿真时钟），再取当帧 sim 时刻
        self.poll_sensors()?;
        let now = self.t_sim;
        self.sensor_this_tick = false;
        self.viewer.set_time(now);

        // 深度感知建图 + 动态障碍写入（规划地图更新先于重规划决策）
        self.update_map_from_depth();
        self.update_motion();

        let measured = self.measured(now);
        let report = self.manager.tick(now, measured);

        // 参考指令发布（闭环控制：MuJoCo 订阅后 PD 跟踪）
        if let Some(reference) = report.reference
            && let Some(pub_) = &self.ref_pub
            && let Err(e) = pub_.publish(ReferenceMessage {
                timestamp: now,
                position_x: reference.position.x,
                position_y: reference.position.y,
                position_z: reference.position.z,
                velocity_x: reference.velocity.x,
                velocity_y: reference.velocity.y,
                velocity_z: reference.velocity.z,
            })
        {
            log::warn!("参考状态发布失败: {e}");
        }

        // 可视化：新轨迹 / 无人机位姿 / 动态障碍
        if report.replanned
            && let Some(result) = self.manager.last_result()
        {
            self.viewer.log_path(
                "plan/global_path",
                self.manager.global_path(),
                (90, 235, 120),
            )?;
            self.viewer.log_trajectory(
                "plan/local_traj",
                &result.trajectory,
                (80, 160, 255),
                (255, 200, 80),
            )?;
            self.viewer.log_planes("plan/planes", &result.planes)?;
            log::info!(
                "replan #{} 完成，时长 {:.2}s",
                self.manager.replans(),
                result.trajectory.duration()
            );
        }
        if let Some(odom) = &self.latest_odom {
            self.viewer.log_pose(
                "plan/drone",
                [
                    odom.state.position.coords.x,
                    odom.state.position.coords.y,
                    odom.state.position.coords.z,
                ],
                odom.quat_xyzw,
            )?;
        }
        if !self.map_file.motions.is_empty() {
            let mut indices = Vec::new();
            for m in &self.map_file.motions {
                let p = m.position_at(now);
                indices.extend(human_voxels(p[0], p[1]));
            }
            self.viewer.log_voxel_grid(
                "plan/motions",
                &indices,
                [0.1, 0.1, 0.1],
                [0.0, 0.0, 0.0],
                (220, 60, 60),
            )?;
        }
        if report.finished {
            self.finished = true;
        }
        Ok(())
    }

    /// `WaitSet` 节拍驱动主循环：interval(10Hz)；SIGINT/SIGTERM → 优雅退出。
    fn run(&mut self) -> Result<()> {
        // 静态产物一次性记录（全局路径为 A* 简化缓存，不随 tick 重复写）
        self.viewer.log_path(
            "plan/global_path",
            self.manager.global_path(),
            (90, 235, 120),
        )?;
        log::info!(
            "主循环启动：10Hz，重规划阈值 {:.1}s，规划视界 {:.0}m",
            ManagerOptions::default().replan_thresh,
            ManagerOptions::default().planning_horizon,
        );
        let mut frame = 0usize;
        let waitset = WaitSetBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| Error::new(ErrorKind::Internal, format!("创建 WaitSet 失败: {e:?}")))?;
        let tick_guard = waitset
            .attach_interval(LOOP_PERIOD)
            .map_err(|e| Error::new(ErrorKind::Internal, format!("挂载节拍定时器失败: {e:?}")))?;
        let on_tick = |attachment_id: WaitSetAttachmentId<ipc::Service>| {
            if !attachment_id.has_event_from(&tick_guard) {
                return CallbackProgression::Continue;
            }
            if self.finished {
                return CallbackProgression::Stop;
            }
            // 每帧 trace 上下文：续接新鲜 odom 的 trace（跨进程同周期一条
            // trace），无新鲜 odom 时自建未采样 root（不产生 span 记录）
            let root = match self
                .odom_trace
                .filter(|_| self.t_sim - self.last_odom_recv < ODOM_FRESH_TIMEOUT)
            {
                Some((tid, sid, sampled)) => Span::root(
                    "planner",
                    SpanContext::new(TraceId(tid), SpanId(sid)).sampled(sampled),
                ),
                None => Span::root("planner", SpanContext::random().sampled(false)),
            };
            let guard = root.set_local_parent();
            let step = self.step();
            drop(guard);
            drop(root);
            match step {
                Ok(()) => {}
                Err(e) => {
                    // 感知/发布类瞬时错误不终止任务；管理器内部错误透传
                    log::warn!("tick 失败：{e}");
                }
            }
            frame += 1;
            // 感知占据地图全量记录（重内容，降频）
            if frame.is_multiple_of(PERCEIVED_PERIOD) {
                let map = self.manager.map();
                if let Err(e) = self.viewer.log_map("plan/perceived", map) {
                    log::debug!("感知地图记录失败：{e}");
                }
            }
            // 时钟推进：本 tick 收到带 sim 时间戳消息则由传感器锚定（已在
            // 轮询时更新 `t_sim`）；否则本地回退递增（独立运行无传感器时）。
            if !self.sensor_this_tick {
                self.t_sim += LOOP_PERIOD.as_secs_f64();
            }
            if self.finished {
                return CallbackProgression::Stop;
            }
            CallbackProgression::Continue
        };

        match waitset.wait_and_process(on_tick) {
            Ok(WaitSetRunResult::Interrupt | WaitSetRunResult::TerminationRequest) => {
                log::info!("收到终止信号，优雅退出");
            }
            Ok(_) => {}
            Err(e) => {
                return Err(Error::new(
                    ErrorKind::Internal,
                    format!("WaitSet 事件等待失败: {e:?}"),
                ));
            }
        }
        log::info!(
            "任务完成：{} 次重规划，总耗时 {:.1}s",
            self.manager.replans(),
            self.t_sim
        );
        Ok(())
    }
}

fn main() {
    init_observability();
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!(
                "{e}\n用法：planner [--map <map.ffmap>] [--save out.rrd] [--start x y z] [--goal x y z] [--frame-offset x y z]"
            );
            std::process::exit(2);
        }
    };
    let map_file = if let Some(p) = &args.map {
        match MapFile::from_file(p) {
            Ok(m) => m,
            Err(e) => {
                eprintln!("加载地图失败：{e}");
                std::process::exit(1);
            }
        }
    } else {
        log::info!("未指定 --map，加载 MuJoCo 默认场景静态地图（深度感知补充）");
        mujoco_map_file()
    };
    let viewer = match &args.save {
        Some(path) => Viewer::save(path),
        // 已有 rerun viewer 则共享（vio 同 viewer 同 recording），否则自动 spawn
        None => Viewer::connect_or_spawn(),
    };
    let viewer = match viewer {
        Ok(v) => v,
        Err(e) => {
            eprintln!("启动 viewer 失败：{e}");
            std::process::exit(1);
        }
    };
    if let Err(e) = viewer.send_default_blueprint() {
        log::warn!("默认布局发送失败（沿用 viewer 当前布局）：{e}");
    }
    match App::new(
        map_file,
        viewer,
        PlannerConfig::default(),
        args.start,
        args.goal,
        args.frame_offset,
    ) {
        Ok(mut app) => {
            // 静态先验一次性记录
            let grid = app.manager.map().clone();
            app.viewer.log_map("plan/map", &grid).expect("log map");
            if let Err(e) = app.run() {
                log::error!("planner 失败：{e}");
                firefly_observability::flush();
                std::process::exit(1);
            }
        }
        Err(e) => {
            eprintln!("初始化失败：{e}");
            firefly_observability::flush();
            std::process::exit(1);
        }
    }
    firefly_observability::flush();
}
