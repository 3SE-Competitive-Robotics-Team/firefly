//! planner 进程：任务执行（`firefly_planner::PlannerManager` 驱动）+ IPC
//! 接线 + 可视化发布（经 `Firefly/Viz` 话题，`firefly-viz` 进程统一写 rerun）。
//!
//! 状态源 = **VIO odom**（`Firefly/Odometry`，新鲜度超时回退轨迹参考推进）；
//! 深度感知建图的位姿同源，深度流超时（对照官方 `grid_map/odom_depth_timeout`）
//! 触发急停且禁用 fail-safe。真值不参与状态链路（vio 进程侧仅作对比可视化）。
//!
//! 运行：`cargo run -p planner`（配合 `uv run firefly-sim` + `cargo run -p vio`），
//! 或 `cargo run -p planner -- --map apps/planner/maps/gate.ffmap` 独立运行。
//!
//! 动态目标：订阅 `Firefly/Goal`（`uv run firefly-goal X Y Z` 发布），
//! 收到目标即重算全局路径并飞往该点；到达后悬停保持、进程保持运行等待
//! 新目标（`--goal` 仅为初始目标，可省略——缺省悬停在 `--start`）。
//!
//! 心跳门控（对照官方 `traj_server::cmdCallback`）：每 tick 即一次执行节拍
//! （官方 FSM 每拍发 `planning/heartbeat`）；从未收到节拍不发布参考，超过
//! `heartbeat_timeout` 无新节拍则降级零速悬停。强制停止：订阅
//! `Firefly/MandatoryStop`（对照官方 `mandatory_stop` topic），收到即进入
//! 急停且不自动恢复；`planner --mandatory-stop` 发一条指令后退出。
//!
//! viz 实体约定（`sim_time` 时间轴）：`plan/global_path`、`plan/local_traj`
//! （+`velocity`）、`plan/planes`、`plan/drone`（位姿）、`plan/perceived`
//! （感知占据地图）、`plan/motions`（动态障碍）、`plan/map`+`plan/decor`
//! （静态先验，启动时一次性记录）。

mod config;
mod scene;

use std::collections::HashSet;
use std::path::PathBuf;
use std::time::Duration;

use fastrace::prelude::*;
use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{
    DepthCamera, GridMap, MapFile, Plane, VirtualWall, VoxelState, update_from_depth,
};
use firefly_observability::init as init_observability;
use firefly_planner::{ManagerOptions, PlannerConfig, PlannerManager, Reference};
use firefly_pubsub::camera::{DEPTH_TOPIC, DepthImageMessage};
use firefly_pubsub::goal::{GOAL_TOPIC, GoalMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::reference::{REFERENCE_TOPIC, ReferenceMessage};
use firefly_pubsub::subscriber::{OdomSubscriber, Subscriber};
use firefly_pubsub::viz::{
    ARROWS_MAX, POINTS_MAX, VIZ_TOPIC, VOXELS_MAX, VizMessage, VizPublisher, kind,
};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetRunResult;
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};

use crate::scene::{human_voxels, mujoco_map_file, parse_vec3};

/// 主循环频率（官方 `exec_timer` 0.1s）。
const LOOP_PERIOD: Duration = Duration::from_millis(100);
/// odom 新鲜度阈值（秒）：超过该时长未收到 odom 则回退轨迹推进估计。
const ODOM_FRESH_TIMEOUT: f64 = 1.0;
/// 深度/odom 丢失阈值（秒），对照官方 `grid_map/odom_depth_timeout`
/// 默认值 1.0（`plan_env/src/grid_map.cpp`）。
const DEPTH_TIMEOUT: f64 = 1.0;
/// 感知占据地图的 viewer 更新周期（帧；10Hz 循环下约每 2.5s）。
const PERCEIVED_PERIOD: usize = 25;
/// 地图衰减周期（帧；10Hz 循环下 5 帧 = 0.5s，对照官方 `fading_timer` 2Hz）。
const FADE_TICKS: usize = 5;
/// `configs/planner.toml` 缺省路径（相对运行目录，通常为仓库根）。
const DEFAULT_CONFIG: &str = "configs/planner.toml";

/// 强制停止话题（外部工具 → 规划进程；对照官方 `mandatory_stop` topic）。
const MANDATORY_STOP_TOPIC: &str = "Firefly/MandatoryStop";

/// 强制停止消息：无载荷，收到即停（对照官方 `std_msgs::Empty`；timestamp
/// 仅诊断用）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyMandatoryStop")]
pub struct MandatoryStopMessage {
    /// 发送时刻（墙钟秒，仅诊断用）。
    pub timestamp: f64,
}

impl Default for MandatoryStopMessage {
    fn default() -> Self {
        Self { timestamp: -1.0 }
    }
}

/// 参考来源决策（对照官方 `traj_server::cmdCallback` 的发布前置条件）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RefSource {
    /// 从未收到执行节拍：不发布任何指令（官方 `heartbeat_time_ <= 1e-5`
    /// 直接返回）。
    Silent,
    /// 节拍超时：零速悬停（官方 `publish_cmd(last_pos_, 0,0,0, last_yaw_, 0)`）。
    Hover,
    /// 节拍新鲜：正常跟踪当前轨迹。
    Track,
}

/// 执行节拍监视（对照官方 `traj_server` 的 `heartbeat_time_`）：主循环每 tick
/// 记一次节拍，参考发布前检查新鲜度。单进程仿真中生产者/消费者同循环，
/// 超时只能由 sim 时钟跳变触发；结构上保留官方的门控语义。
struct HeartbeatGate {
    /// 是否已收到至少一次节拍（官方 `heartbeat_time_ > 1e-5`）。
    armed: bool,
    /// 最近一次节拍时刻（sim 时钟，秒）。
    last: f64,
}

impl HeartbeatGate {
    const fn new() -> Self {
        Self {
            armed: false,
            last: 0.0,
        }
    }

    /// 记录本拍：调用方在每拍的参考来源判定之后调用，使判定基于截至
    /// 上一拍的节拍历史（与官方「异步心跳回调 + 独立检查」的新鲜度语义一致）。
    fn observe(&mut self, now: f64) {
        self.armed = true;
        self.last = now;
    }

    /// 发布前置条件判定：未 armed → [`RefSource::Silent`]；超时 →
    /// [`RefSource::Hover`]；否则 [`RefSource::Track`]。
    fn decide(&self, now: f64, timeout: f64) -> RefSource {
        if !self.armed {
            return RefSource::Silent;
        }
        if now - self.last > timeout {
            RefSource::Hover
        } else {
            RefSource::Track
        }
    }
}

/// 零速悬停参考（对照官方 `publish_cmd(last_pos_, 0,0,0, last_yaw_, 0)`）：
/// 位置/偏航保持，速度为零（参考通道不建模加速度/加加速度，隐含为零）。
fn hover_reference(position: Vector3<f64>, yaw_state: (f64, f64)) -> Reference {
    Reference {
        position,
        velocity: Vector3::zeros(),
        yaw: yaw_state.0,
        // 官方超时悬停的 yaw_dot 恒为 0
        yaw_dot: 0.0,
    }
}

struct Args {
    map: Option<PathBuf>,
    config: PathBuf,
    start: [f64; 3],
    goal: Option<[f64; 3]>,
    frame_offset: [f64; 3],
    /// 仅向 `Firefly/MandatoryStop` 发一条强制停止指令后退出。
    mandatory_stop: bool,
}

fn parse_args() -> Result<Args> {
    let mut it = std::env::args().skip(1);
    let mut args = Args {
        map: None,
        config: PathBuf::from(DEFAULT_CONFIG),
        start: [1.0, 4.0, 1.0],
        // 初始目标缺省 = 起点：悬停等待外部 `Firefly/Goal` 目标
        goal: None,
        frame_offset: [0.0, 0.0, 0.0],
        mandatory_stop: false,
    };
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--map" => {
                args.map = Some(PathBuf::from(it.next().ok_or_else(|| {
                    Error::new(ErrorKind::InvalidArgument, "missing --map value")
                })?));
            }
            "--config" => {
                args.config = PathBuf::from(it.next().ok_or_else(|| {
                    Error::new(ErrorKind::InvalidArgument, "missing --config value")
                })?);
            }
            "--start" => args.start = parse_vec3(&mut it, "--start")?,
            "--goal" => args.goal = Some(parse_vec3(&mut it, "--goal")?),
            "--frame-offset" => args.frame_offset = parse_vec3(&mut it, "--frame-offset")?,
            "--mandatory-stop" => args.mandatory_stop = true,
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

/// 打开可选订阅器：失败降级为 `None` 并记录（辅助输入流可缺失，进程保持独立运行）。
fn open_sub<T: std::fmt::Debug + ZeroCopySend + 'static>(
    node: &firefly_pubsub::node::IpcNode,
    topic: &str,
    ok_msg: &str,
    err_msg: &str,
) -> Option<Subscriber<T>> {
    match Subscriber::<T>::with_topic(node, topic) {
        Ok(s) => {
            log::info!("{ok_msg}（topic {topic}）");
            Some(s)
        }
        Err(e) => {
            log::warn!("{err_msg}: {e}");
            None
        }
    }
}

/// 最新里程计快照（地图系状态 + 姿态四元数，深度投影用）。
struct OdomSnapshot {
    state: firefly_planner::State,
    quat_xyzw: [f64; 4],
}

/// 各布尔标志相互独立（传感器/停止/完成），非状态机编码。
#[allow(clippy::struct_excessive_bools)]
struct App {
    manager: PlannerManager,
    /// 管理器行为参数（来自配置，启动日志展示）。
    manager_options: ManagerOptions,
    /// 可视化发布端（经 `Firefly/Viz` 话题，`firefly-viz` 进程统一写 rerun）。
    viz_pub: Option<VizPublisher>,
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
    goal_sub: Option<Subscriber<GoalMessage>>,
    /// 强制停止订阅（对照官方 `mandatory_stop` topic；单进程仿真下由外部
    /// 工具经 `planner --mandatory-stop` 注入）。
    stop_sub: Option<Subscriber<MandatoryStopMessage>>,
    /// 收到的最近目标（本 tick 处理一次，处理完清空；快速连续发布取最新）。
    pending_goal: Option<GoalMessage>,
    /// 强制停止待处理标记（poll 置位，step 消费后清零）。
    pending_mandatory_stop: bool,
    /// 执行节拍监视（心跳门控，见 [`HeartbeatGate`]）。
    heartbeat: HeartbeatGate,
    /// 心跳超时阈值（秒，配置 `heartbeat_timeout`）。
    heartbeat_timeout: f64,
    /// 最近一次发布的参考位置（官方 `traj_server` 的 `last_pos_`；超时悬停落点）。
    last_ref_pos: Option<Vector3<f64>>,
    latest_odom: Option<OdomSnapshot>,
    last_odom_recv: f64,
    /// 最新 odom 携带的 trace 上下文 `(trace_id, span_id, sampled)`（续接用）。
    odom_trace: Option<(u128, u64, bool)>,
    latest_depth: Option<DepthImageMessage>,
    /// 深度流新鲜度监视（超时锁存触发急停）。
    depth_freshness: DepthFreshness,
    /// 深度丢失已报错标记（锁存期内不刷屏；新帧到达即复位）。
    depth_loss_reported: bool,
    ref_pub: Option<Publisher<ReferenceMessage>>,
    depth_cam: DepthCamera,
    frame_offset: Vector3<f64>,
    finished: bool,
    /// 衰减节拍计数（10Hz 累计，满 5 帧触发 2Hz `fade`）。
    fade_ticks: usize,
    corrected_odom: Option<firefly_pubsub::subscriber::CorrectedOdomSubscriber>,
    latest_corrected: Option<OdomSnapshot>,
    last_corrected_recv: f64,
}

impl App {
    /// 接线：地图 → planner → 管理器 → 订阅/发布端口。
    ///
    /// # Errors
    ///
    /// 地图体素化 / 全局路径搜索 / IPC 端口创建失败。
    #[allow(clippy::too_many_lines)]
    fn new(
        map_file: MapFile,
        config: PlannerConfig,
        manager_options: ManagerOptions,
        start: [f64; 3],
        goal: Option<[f64; 3]>,
        frame_offset: [f64; 3],
    ) -> Result<Self> {
        let mut grid = map_file.to_grid_map()?;
        // 心跳超时阈值先取（config 随后移入 Planner）
        let heartbeat_timeout = config.heartbeat_timeout;
        // 虚拟地面/天花板（对照官方 enable_virtual_wall）：任一配置存在即单独生效
        if config.virtual_ground.is_some() || config.virtual_ceiling.is_some() {
            grid.set_virtual_wall(VirtualWall {
                ground: config.virtual_ground.unwrap_or(f64::NEG_INFINITY),
                ceil: config.virtual_ceiling.unwrap_or(f64::INFINITY),
            });
        }
        let static_occupied = map_file
            .occupied
            .iter()
            .filter_map(|p| grid.index_of(Vector3::new(p[0], p[1], p[2])))
            .collect();
        let planner = firefly_planner::Planner::new(config, grid);
        // 初始目标缺省 = 起点：悬停等待外部 `Firefly/Goal` 目标
        let goal = goal.unwrap_or(start);
        let manager = PlannerManager::with_planner(
            planner,
            manager_options,
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
        let depth = open_sub::<DepthImageMessage>(
            &node,
            DEPTH_TOPIC,
            "已订阅深度话题（感知建图输入）",
            "深度订阅不可用，感知建图停用",
        );
        let goal_sub = open_sub::<GoalMessage>(
            &node,
            GOAL_TOPIC,
            "已订阅目标话题（`uv run firefly-goal X Y Z` 发布）",
            "目标订阅不可用",
        );
        let stop_sub = open_sub::<MandatoryStopMessage>(
            &node,
            MANDATORY_STOP_TOPIC,
            "已订阅强制停止话题（`planner --mandatory-stop` 发布）",
            "强制停止订阅不可用",
        );
        let ref_pub = match Publisher::<ReferenceMessage>::with_topic(&node, REFERENCE_TOPIC) {
            Ok(p) => Some(p),
            Err(e) => {
                log::warn!("参考发布不可用：{e}");
                None
            }
        };
        log::info!("状态源：odom（新鲜度 {ODOM_FRESH_TIMEOUT}s）；真值不参与规划链路");
        let corrected_odom = match firefly_pubsub::subscriber::CorrectedOdomSubscriber::new(&node) {
            Ok(s) => {
                log::info!("已订阅校正后里程计话题（GICP 融合输出，优先使用）");
                Some(s)
            }
            Err(e) => {
                log::warn!("校正后里程计订阅不可用，回退原始 odom：{e}");
                None
            }
        };
        // 可视化发布端：经 Firefly/Viz 话题发布，firefly-viz 进程统一写 rerun
        // （计算线程零 IO；创建失败只降级日志，规划链路不受影响）
        let viz_pub = match VizPublisher::new(&node) {
            Ok(p) => {
                log::info!("已打开话题 {VIZ_TOPIC}");
                Some(p)
            }
            Err(e) => {
                log::warn!("可视化发布不可用（跳过 viz 输出）：{e}");
                None
            }
        };
        Ok(Self {
            manager,
            manager_options,
            viz_pub,
            static_occupied,
            prev_dyn: Vec::new(),
            map_file,
            t_sim: 0.0,
            sensor_this_tick: false,
            odom,
            depth,
            goal_sub,
            stop_sub,
            pending_goal: None,
            pending_mandatory_stop: false,
            heartbeat: HeartbeatGate::new(),
            heartbeat_timeout,
            last_ref_pos: None,
            latest_odom: None,
            last_odom_recv: f64::NEG_INFINITY,
            odom_trace: None,
            latest_depth: None,
            depth_freshness: DepthFreshness::new(),
            depth_loss_reported: false,
            ref_pub,
            depth_cam: DepthCamera::mujoco_default(),
            frame_offset: Vector3::new(frame_offset[0], frame_offset[1], frame_offset[2]),
            finished: false,
            fade_ticks: 0,
            corrected_odom,
            latest_corrected: None,
            last_corrected_recv: f64::NEG_INFINITY,
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
        if let Some(sub) = &self.corrected_odom {
            while let Some(sample) = sub.receive()? {
                let ctx = *sample.user_header();
                let _span = ctx.continue_span("recv-corrected");
                if ctx.is_traced() {
                    self.odom_trace = Some((ctx.trace_id(), ctx.span_id, ctx.sampled()));
                }
                let m: OdomMessage = *sample;
                self.t_sim = self.t_sim.max(m.timestamp);
                self.last_corrected_recv = m.timestamp;
                self.sensor_this_tick = true;
                let p = Vector3::new(m.position_x, m.position_y, m.position_z) + self.frame_offset;
                self.latest_corrected = Some(OdomSnapshot {
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
                let m: DepthImageMessage = *sample;
                // 帧时间戳（传感器时钟 = 仿真时钟）锚定 sim 时钟，与 odom
                // 一致；新鲜度计时在 update_map_from_depth 实际吃进帧时推进
                self.t_sim = self.t_sim.max(m.timestamp);
                self.sensor_this_tick = true;
                self.depth_loss_reported = false;
                self.latest_depth = Some(m);
            }
        }
        if let Some(sub) = &self.goal_sub {
            while let Some(sample) = sub.receive()? {
                // 目标不参与 sim 时钟锚定（CLI 墙钟）；最新一条生效
                self.pending_goal = Some(*sample);
                log::info!(
                    "收到新目标 ({:.2},{:.2},{:.2})",
                    sample.position_x,
                    sample.position_y,
                    sample.position_z
                );
            }
        }
        if let Some(sub) = &self.stop_sub {
            while (sub.receive()?).is_some() {
                // 无载荷消息，收到即置位（对照官方 mandatoryStopCallback）
                self.pending_mandatory_stop = true;
            }
        }
        Ok(())
    }

    /// 零速悬停参考位置解析（官方超时分支的 `last_pos_` 取值）：优先最近一次
    /// 发布的参考位置，无则取实测位置；两者皆无时返回 `None`（无落点可不发布）。
    fn hover_fallback(&self, measured: Option<firefly_planner::State>) -> Option<Reference> {
        let position = self
            .last_ref_pos
            .or_else(|| measured.map(|m| m.position.coords))?;
        Some(hover_reference(position, self.manager.yaw_state()))
    }

    /// 新鲜 odom 的规划系状态（校正后优先，回退原始；超时返回 `None`）。
    fn measured(&self, now: f64) -> Option<firefly_planner::State> {
        if now - self.last_corrected_recv < ODOM_FRESH_TIMEOUT
            && let Some(snap) = &self.latest_corrected
        {
            return Some(snap.state);
        }
        if now - self.last_odom_recv >= ODOM_FRESH_TIMEOUT {
            return None;
        }
        self.latest_odom.as_ref().map(|o| o.state)
    }

    /// 深度 → 占据体素（感知建图）：位姿源与融合后状态同源（VIO 经 GICP 矫正）。
    /// 深度与位姿任一断流都会在此早退——管线饥饿由 [`Self::depth_freshness`]
    /// 的计时基准停止推进体现（对照官方 `last_occ_update_time_` 只在实际
    /// 更新占据栅格时推进，"odom or depth lost!" 任一丢失都算）。
    fn update_map_from_depth(&mut self) {
        let (Some(depth), Some(odom)) = (&self.latest_depth, &self.latest_odom) else {
            return;
        };
        // 位姿源与状态源同源：优先校正后位姿，保证地图与规划同系
        let snap = if self.last_corrected_recv > self.last_odom_recv {
            self.latest_corrected.as_ref().unwrap_or(odom)
        } else {
            odom
        };
        let pos = snap.state.position.coords;
        let q = snap.quat_xyzw;
        let quat = UnitQuaternion::from_quaternion(Quaternion::new(q[3], q[0], q[1], q[2]));
        let pose = Isometry3::from_parts(Translation3::new(pos.x, pos.y, pos.z), quat);
        update_from_depth(self.manager.map_mut(), &self.depth_cam, &pose, &depth.data);
        self.depth_freshness.observe(depth.timestamp);
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

        // 深度感知建图 + 动态障碍写入（规划地图更新先于重规划决策）
        self.update_map_from_depth();
        // 地图衰减（对照官方 `fadingCallback` 2Hz）：每 0.5s 调用一次固定 `fade()`，
        // 膨胀层在 `fade` 内部增量移除（计数缓冲，对照官方 `changeInfBuf`）。
        self.fade_ticks += 1;
        if self.fade_ticks >= FADE_TICKS {
            self.manager.map_mut().fade();
            self.fade_ticks = 0;
        }
        self.update_motion();

        let measured = self.measured(now);

        // 强制停止（官方 mandatoryStopCallback）：置锁存标志、关 fail-safe、
        // 进入急停——此后不自动恢复；先于其他分支处理。
        if self.pending_mandatory_stop {
            self.pending_mandatory_stop = false;
            self.manager.mandatory_stop(now, measured);
        }

        // 深度/odom 超时急停（官方 checkCollisionCallback：有可执行轨迹且
        // 未完成才监控深度丢失，命中则关 fail-safe 进急停、永不自动恢复）。
        // 锁存后重复命中不刷屏（监视结构锁存 + enter 幂等）。
        if self.manager.local().is_some()
            && !self.manager.is_finished()
            && self.depth_freshness.timed_out(now, DEPTH_TIMEOUT)
        {
            if !self.depth_loss_reported {
                log::error!("深度/里程计丢失！进入急停（fail-safe 已禁用）");
                self.depth_loss_reported = true;
            }
            self.manager
                .trigger_emergency_stop_disable_failsafe(now, measured);
        }

        // 动态目标：收到新目标即重目标（重算全局路径 + 重置状态机），
        // 下一 tick 重新规划飞往新目标
        if let Some(goal) = self.pending_goal.take() {
            let target =
                Vector3::new(goal.position_x, goal.position_y, goal.position_z) + self.frame_offset;
            match self.manager.set_goal(now, measured, target) {
                Ok(()) => log::info!(
                    "目标更新 ({:.1},{:.1},{:.1})，重新规划中",
                    target.x,
                    target.y,
                    target.z
                ),
                Err(e) => log::warn!("目标 ({target:?}) 不可达，忽略：{e}"),
            }
        }
        let report = self.manager.tick(now, measured);

        // 参考来源（对照官方 traj_server::cmdCallback 的心跳门控）：从未收到
        // 执行节拍不发布；节拍超时降级零速悬停；正常时跟踪轨迹，到达后以
        // 目标点为悬停参考（进程保持运行等待新目标）
        let tracked = if self.manager.is_finished() {
            let goal = self.manager.goal().coords;
            // 到达悬停保持最后朝向（无轨迹可前视，不再推进 yaw 状态）
            Some(hover_reference(goal, self.manager.yaw_state()))
        } else {
            report.reference
        };
        let reference = match self.heartbeat.decide(now, self.heartbeat_timeout) {
            RefSource::Silent => None,
            RefSource::Hover => {
                log::error!(
                    "执行节拍超时（>{:.1}s 无新节拍），降级零速悬停",
                    self.heartbeat_timeout
                );
                self.hover_fallback(measured)
            }
            RefSource::Track => tracked,
        };
        // 本 tick 节拍在判定后记录（与官方「异步心跳 + 独立检查」新鲜度语义一致）
        self.heartbeat.observe(now);
        if let Some(reference) = reference
            && let Some(pub_) = &self.ref_pub
        {
            match pub_.publish(ReferenceMessage {
                timestamp: now,
                position_x: reference.position.x,
                position_y: reference.position.y,
                position_z: reference.position.z,
                velocity_x: reference.velocity.x,
                velocity_y: reference.velocity.y,
                velocity_z: reference.velocity.z,
                yaw: reference.yaw,
                yaw_dot: reference.yaw_dot,
            }) {
                // 官方 last_pos_：超时悬停与跟踪共用最近指令位置
                Ok(_) => self.last_ref_pos = Some(reference.position),
                Err(e) => log::warn!("参考状态发布失败: {e}"),
            }
        }

        // 可视化：新轨迹 / 无人机位姿 / 动态障碍（经 Firefly/Viz 发布）
        if report.replanned
            && let Some(result) = self.manager.last_result()
        {
            self.log_line_strip(
                "plan/global_path",
                self.manager.global_path(),
                (90, 235, 120),
                now,
            );
            self.log_trajectory("plan/local_traj", &result.trajectory, now);
            self.log_planes("plan/planes", &result.planes, now);
            log::info!(
                "replan #{} 完成，时长 {:.2}s",
                self.manager.replans(),
                result.trajectory.duration()
            );
        }
        if let Some(odom) = &self.latest_odom {
            self.log_pose(
                "plan/drone",
                [
                    odom.state.position.coords.x,
                    odom.state.position.coords.y,
                    odom.state.position.coords.z,
                ],
                odom.quat_xyzw,
                now,
            );
        }
        if !self.map_file.motions.is_empty() {
            let mut indices = Vec::new();
            for m in &self.map_file.motions {
                let p = m.position_at(now);
                indices.extend(human_voxels(p[0], p[1]));
            }
            self.log_voxels("plan/motions", &indices, now);
        }
        if report.finished {
            self.finished = true;
        }
        Ok(())
    }

    /// `WaitSet` 节拍驱动主循环：interval(10Hz)；SIGINT/SIGTERM → 优雅退出。
    fn run(&mut self) -> Result<()> {
        // 静态产物一次性记录（全局路径为 A* 简化缓存，不随 tick 重复写）
        self.log_line_strip(
            "plan/global_path",
            self.manager.global_path(),
            (90, 235, 120),
            self.t_sim,
        );
        log::info!(
            "主循环启动：10Hz，重规划阈值 {:.1}s，规划视界 {:.0}m",
            self.manager_options.replan_thresh,
            self.manager_options.planning_horizon,
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
            // 动态目标工作流：到达后保持运行（悬停 + 等待新目标），
            // 不因 finished 退出进程（仅 SIGINT/SIGTERM 优雅退出）
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
                self.log_map("plan/perceived", map, self.t_sim);
            }
            // 时钟推进：本 tick 收到带 sim 时间戳消息则由传感器锚定（已在
            // 轮询时更新 `t_sim`）；否则本地回退递增（独立运行无传感器时）。
            if !self.sensor_this_tick {
                self.t_sim += LOOP_PERIOD.as_secs_f64();
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
            "进程退出：本会话 {} 次重规划，仿真时长 {:.1}s（悬停保持中，等待新目标可继续）",
            self.manager.replans(),
            self.t_sim
        );
        Ok(())
    }

    // --- 可视化发布（经 Firefly/Viz 话题，firefly-viz 进程统一写 rerun）---

    /// 发布一条可视化消息；无发布端或发布失败时静默降级（可视化不阻断规划）。
    fn viz_publish(&self, msg: &VizMessage) {
        if let Some(pub_) = &self.viz_pub
            && let Err(e) = pub_.publish(*msg)
        {
            log::debug!("viz 发布失败：{e}");
        }
    }

    /// 折线 → `line_strip` 消息。点数超上限 [`POINTS_MAX`] 时截断并告警一次。
    fn log_line_strip(&self, entity: &str, points: &[Vector3<f64>], rgb: (u8, u8, u8), t: f64) {
        let mut msg = VizMessage::base(kind::LINE_STRIP, t, entity);
        msg.color = [rgb.0, rgb.1, rgb.2];
        msg.point_count = points.len().min(POINTS_MAX) as u32;
        for (i, p) in points.iter().take(POINTS_MAX).enumerate() {
            msg.points[i] = [p.x, p.y, p.z];
        }
        if points.len() > POINTS_MAX {
            log::warn!("{entity} 点数 {} 超上限 {POINTS_MAX}，已截断", points.len());
        }
        self.viz_publish(&msg);
    }

    /// 轨迹 → 采样折线（位置）+ 速度箭头（位于对应采样点）。
    fn log_trajectory(&self, entity: &str, traj: &firefly_trajectory::Trajectory, t: f64) {
        const SAMPLES: usize = 100;
        let mut pts = Vec::with_capacity(SAMPLES);
        let mut arrows: Vec<[f64; 3]> = Vec::with_capacity(SAMPLES);
        for k in 0..SAMPLES {
            let s = traj.eval(traj.duration() * k as f64 / SAMPLES as f64);
            pts.push(s.position);
            arrows.push([s.velocity.x, s.velocity.y, s.velocity.z]);
        }
        self.log_line_strip(entity, &pts, (80, 160, 255), t);
        // 速度箭头挂在子实体，随轨迹同时间戳更新
        let mut msg = VizMessage::base(kind::ARROWS, t, &format!("{entity}/velocity"));
        msg.color = [255, 200, 80];
        msg.arrow_count = SAMPLES.min(ARROWS_MAX) as u32;
        for (i, (p, v)) in pts.iter().zip(&arrows).take(ARROWS_MAX).enumerate() {
            msg.arrow_origins[i] = [p.x, p.y, p.z];
            msg.arrow_vectors[i] = *v;
        }
        self.viz_publish(&msg);
    }

    /// 障碍平面（{s, v}）→ 法线向量（0.6m 长，黄）。
    fn log_planes(&self, entity: &str, planes: &[Plane], t: f64) {
        let mut msg = VizMessage::base(kind::ARROWS, t, entity);
        msg.color = [240, 200, 60];
        msg.arrow_count = planes.len().min(ARROWS_MAX) as u32;
        for (i, p) in planes.iter().take(ARROWS_MAX).enumerate() {
            let s = p.point();
            let v = p.normal() * 0.6;
            msg.arrow_origins[i] = [s.x, s.y, s.z];
            msg.arrow_vectors[i] = [v.x, v.y, v.z];
        }
        self.viz_publish(&msg);
    }

    /// 位姿 → pose 消息（`plan/drone` 等刚体变换实体）。
    fn log_pose(&self, entity: &str, pos: [f64; 3], quat_xyzw: [f64; 4], t: f64) {
        let mut msg = VizMessage::base(kind::POSE, t, entity);
        msg.color = [255, 255, 255];
        msg.xyz = pos;
        msg.quat_xyzw = quat_xyzw;
        self.viz_publish(&msg);
    }

    /// 占据体素索引 → voxels 消息（体素中心 = 原点 + (idx+0.5)·尺寸，
    /// Python 端 `VoxelGridMap` 直接镜像）。超限截断并告警一次（锁存防刷屏）。
    fn log_voxels(&self, entity: &str, indices: &[(i32, i32, i32)], t: f64) {
        let mut msg = VizMessage::base(kind::VOXELS, t, entity);
        msg.voxel_count = indices.len().min(VOXELS_MAX) as u32;
        for (i, &(x, y, z)) in indices.iter().take(VOXELS_MAX).enumerate() {
            msg.voxels[i] = [x, y, z];
        }
        msg.voxel_size = [0.1, 0.1, 0.1];
        msg.voxel_origin = [0.0, 0.0, 0.0];
        if indices.len() > VOXELS_MAX {
            log::warn!(
                "{entity} 体素数 {} 超上限 {VOXELS_MAX}，已截断",
                indices.len()
            );
        }
        self.viz_publish(&msg);
    }

    /// 占据栅格地图 → voxels 消息（收集 Occupied 索引，体素中心与
    /// `VoxelGridMap` 语义一致）。
    fn log_map(&self, entity: &str, map: &GridMap, t: f64) {
        let origin = map.origin();
        let dims = map.dims();
        let mut indices = Vec::new();
        for x in 0..dims[0] {
            for y in 0..dims[1] {
                for z in 0..dims[2] {
                    if map.state([x, y, z]) == VoxelState::Occupied {
                        indices.push((x as i32, y as i32, z as i32));
                    }
                }
            }
        }
        let mut msg = VizMessage::base(kind::VOXELS, t, entity);
        msg.voxel_count = indices.len().min(VOXELS_MAX) as u32;
        for (i, &(x, y, z)) in indices.iter().take(VOXELS_MAX).enumerate() {
            msg.voxels[i] = [x, y, z];
        }
        msg.voxel_size = [map.resolution() as f32; 3];
        msg.voxel_origin = [origin.x as f32, origin.y as f32, origin.z as f32];
        if indices.len() > VOXELS_MAX {
            log::warn!(
                "{entity} 体素数 {} 超上限 {VOXELS_MAX}，已截断",
                indices.len()
            );
        }
        self.viz_publish(&msg);
    }
}

/// `--mandatory-stop`：向 [`MANDATORY_STOP_TOPIC`] 发一条指令后返回（对照
/// 官方 `rostopic pub .../mandatory_stop std_msgs/Empty`）。
///
/// # Errors
///
/// 节点或发布器创建失败、消息发送失败。
fn emit_mandatory_stop() -> Result<()> {
    let node = create_node()?;
    let pub_ = Publisher::<MandatoryStopMessage>::with_topic(&node, MANDATORY_STOP_TOPIC)?;
    let timestamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(f64::NAN, |d| d.as_secs_f64());
    pub_.publish(MandatoryStopMessage { timestamp }).map(|_| ())
}

/// 感知建图管线新鲜度监视：对照官方 `flag_depth_odom_timeout_` 锁存语义——
/// 计时基准是管线**实际吃进一帧**（深度+位姿齐备并写入地图）的时间戳，
/// 深度或位姿任一断流即饥饿，超时置位后锁存，直到管线再次吃进新帧才复位。
struct DepthFreshness {
    /// 最近一次实际建图消费的帧时间戳（传感器时钟）；首帧之前为 `None`。
    last_frame_ts: Option<f64>,
    /// 超时锁存位（官方 `flag_depth_odom_timeout_`）。
    lost: bool,
}

impl DepthFreshness {
    const fn new() -> Self {
        Self {
            last_frame_ts: None,
            lost: false,
        }
    }

    /// 管线实际吃进一帧：记录时间戳并清除丢失锁存。
    fn observe(&mut self, ts: f64) {
        self.last_frame_ts = Some(self.last_frame_ts.map_or(ts, |prev| prev.max(ts)));
        self.lost = false;
    }

    /// 首帧之前恒 false；`now - 最新帧ts > timeout` 后置位并锁存
    /// （后续调用持续返回 true）。
    fn timed_out(&mut self, now: f64, timeout: f64) -> bool {
        if let Some(ts) = self.last_frame_ts
            && now - ts > timeout
        {
            self.lost = true;
        }
        self.lost
    }
}

fn main() {
    init_observability();
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!(
                "{e}\n用法：planner [--map <map.ffmap>] [--config configs/planner.toml] [--start x y z] [--goal x y z] [--frame-offset x y z] [--mandatory-stop]\n\n--goal 可省略（悬停等待 `uv run firefly-goal X Y Z` 动态目标）；--mandatory-stop 向 {MANDATORY_STOP_TOPIC} 发一条强制停止指令后退出"
            );
            std::process::exit(2);
        }
    };
    if args.mandatory_stop {
        if let Err(e) = emit_mandatory_stop() {
            log::error!("强制停止指令发布失败：{e}");
            firefly_observability::flush();
            std::process::exit(1);
        }
        log::info!("已发布强制停止指令到 {MANDATORY_STOP_TOPIC}");
        firefly_observability::flush();
        return;
    }
    let toml_cfg = match config::PlannerToml::load(&args.config) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("加载配置失败：{e}");
            std::process::exit(1);
        }
    };
    log::info!("已加载配置 {}", args.config.display());
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
    // 可视化发布端在 App::new 内随进程共享节点创建（经 Firefly/Viz 话题，
    // firefly-viz 进程统一写 rerun）
    match App::new(
        map_file,
        toml_cfg.config,
        toml_cfg.manager,
        args.start,
        args.goal,
        args.frame_offset,
    ) {
        Ok(mut app) => {
            // 静态先验一次性记录（体素索引收集在发布端完成）
            let grid = app.manager.map().clone();
            app.log_map("plan/map", &grid, 0.0);
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

#[cfg(test)]
mod tests {
    use super::*;

    /// 管线从未吃进帧（首帧之前）不触发。
    #[test]
    fn no_trigger_before_first_frame() {
        let mut f = DepthFreshness::new();
        assert!(!f.timed_out(100.0, DEPTH_TIMEOUT));
    }

    /// 管线再次吃进帧清除丢失锁存。
    #[test]
    fn new_frame_resets_latch() {
        let mut f = DepthFreshness::new();
        f.observe(10.0);
        assert!(f.timed_out(11.5, DEPTH_TIMEOUT));
        // 锁存期内持续命中
        assert!(f.timed_out(12.0, DEPTH_TIMEOUT));
        f.observe(11.8);
        assert!(!f.timed_out(12.0, DEPTH_TIMEOUT));
    }

    /// 超时置位并锁存。
    #[test]
    fn timeout_sets_and_latches() {
        let mut f = DepthFreshness::new();
        f.observe(10.0);
        assert!(!f.timed_out(11.0, DEPTH_TIMEOUT));
        assert!(f.timed_out(11.0 + 1e-3, DEPTH_TIMEOUT));
        assert!(f.timed_out(20.0, DEPTH_TIMEOUT));
    }

    /// 从未收到节拍不发布参考（官方 `heartbeat_time_ ≤ 1e-5` 直接返回）；
    /// 首拍之后转入正常跟踪。
    #[test]
    fn heartbeat_gate_blocks_until_first_tick() {
        let mut g = HeartbeatGate::new();
        assert_eq!(g.decide(0.0, 0.5), RefSource::Silent);
        g.observe(0.1);
        assert_eq!(g.decide(0.2, 0.5), RefSource::Track);
    }

    /// 节拍超时降级悬停，新节拍恢复跟踪（官方 0.5s 判据）。
    #[test]
    fn heartbeat_gate_times_out_to_hover() {
        let mut g = HeartbeatGate::new();
        g.observe(10.0);
        assert_eq!(g.decide(10.4, 0.5), RefSource::Track);
        assert_eq!(g.decide(10.6, 0.5), RefSource::Hover);
        g.observe(10.7);
        assert_eq!(g.decide(10.8, 0.5), RefSource::Track);
    }

    /// 零速悬停参考：位置/偏航保持，速度与角速度为零
    /// （官方 `publish_cmd(last_pos_, 0,0,0, last_yaw_, 0)`）。
    #[test]
    fn hover_reference_holds_position_and_yaw() {
        let r = hover_reference(Vector3::new(1.5, 2.5, 1.0), (0.3, -0.7));
        assert!((r.position - Vector3::new(1.5, 2.5, 1.0)).norm() < 1e-12);
        assert!(r.velocity.norm() < 1e-12);
        assert!((r.yaw - 0.3).abs() < 1e-12);
        assert!(r.yaw_dot.abs() < 1e-12);
    }
}
