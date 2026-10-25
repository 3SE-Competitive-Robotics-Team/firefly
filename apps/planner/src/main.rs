//! planner 进程接入层：Navigation/深度/目标输入 → FSM → 规划管理 → 完整轨迹。
//! FSM 为 100Hz，安全检查入口为 20Hz；独立轨迹输出线程为 100Hz，接收轨迹与心跳。
//! Navigation 失联或无效状态锁存停止输出，坐标修正使旧版本轨迹立即失效。
//! 深度超时触发急停并禁用 fail-safe；真值不进入规划状态链路。

mod config;
mod navigation;
mod output;
mod scene;
mod state_input;

use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use fastrace::prelude::*;
use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{
    DepthCamera, GridMap, MapFile, Plane, VirtualWall, VoxelState, update_from_depth,
};
use firefly_observability::init as init_observability;
use firefly_planner::{
    ManagerOptions, PlannerConfig, PlannerManager, Reference, ReplanFsm, TrajectoryCommand,
};
use firefly_pubsub::camera::{DEPTH_TOPIC, DepthImageMessage};
use firefly_pubsub::goal::{GOAL_TOPIC, GoalMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::{Received, Subscriber};

/// 深度样本句柄：零拷贝持有，tick 消费 `&data` 后释放，无 300KB 全拷贝。
type ReceivedDepth = Received<DepthImageMessage>;
use firefly_pubsub::viz::{
    ARROWS_MAX, POINTS_MAX, VIZ_TOPIC, VOXELS_MAX, VizMessage, VizPublisher, VoxelMessage, kind,
};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetRunResult;
use nalgebra::Vector3;

use crate::scene::{human_voxels, parse_vec3};
use crate::state_input::StateInput;

/// 主循环频率（官方 `exec_timer` 0.01s）。
const LOOP_PERIOD: Duration = Duration::from_millis(10);
/// 深度/odom 丢失阈值（秒），对照官方 `grid_map/odom_depth_timeout`
/// 默认值 1.0（`plan_env/src/grid_map.cpp`）。
const DEPTH_TIMEOUT: f64 = 1.0;
/// 体素层全量重同步周期（帧；10Hz 循环下 50 帧 = 5s）。增量协议下每拍只发
/// 变化量，周期性补发全量快照，任一增量消息丢失后消费端可自愈。
const VOXEL_RESYNC_TICKS: usize = 50;
/// 实时感知体素的显示半径（米）：以机体为中心的立方窗口。
///
/// 不用 `local_update_range`——它按整段局部更新范围，窗口里连整片地板一起算，
/// 占用格数仍超 `VOXELS_MAX`；4m @0.15m 约几千格，既不截断又跟得上机体。
const PERCEIVED_VIEW_RADIUS_M: f64 = 4.0;
/// `configs/planner.toml` 缺省路径（相对运行目录，通常为仓库根）。
/// 缺省配置：按安装布局解析，与启动目录无关。
fn default_config() -> PathBuf {
    firefly_config::paths::config("planner.toml")
}

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

/// 零速悬停参考（对照官方 `publish_cmd(last_pos_, 0,0,0, last_yaw_, 0)`）：
/// 位置/偏航保持，速度为零（参考通道不建模加速度/加加速度，隐含为零）。
fn hover_reference(position: Vector3<f64>, yaw_state: (f64, f64)) -> Reference {
    Reference {
        position,
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
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
    /// 仅向 `Firefly/MandatoryStop` 发一条强制停止指令后退出。
    mandatory_stop: bool,
}

fn parse_args() -> Result<Args> {
    let mut it = std::env::args().skip(1);
    let mut args = Args {
        map: None,
        config: default_config(),
        start: [0.0; 3],
        // 初始目标缺省 = 起点：悬停等待外部 `Firefly/Goal` 目标
        goal: None,
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

/// 打开可选订阅器：失败降级为 `None` 并记录（辅助输入流可缺失，状态门控仍生效）。
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

/// 各布尔标志相互独立（传感器/停止/完成），非状态机编码。
#[allow(clippy::struct_excessive_bools)]
struct App {
    fsm: ReplanFsm,
    output: output::OutputWorker,
    published_trajectory: u32,
    /// 管理器行为参数（来自配置，启动日志展示）。
    manager_options: ManagerOptions,
    /// 可视化发布端（经 `Firefly/Viz` 话题，`firefly-viz` 进程统一写 rerun）。
    viz_pub: Option<VizPublisher>,
    /// 日志聚合句柄（主循环作用域持有，每 tick 传给 `pump_log_ipc`；
    /// 发布端只活在创建线程，无跨线程触碰，见 `firefly-observability`）。
    log_ipc: firefly_observability::LogIpc,
    map_file: Option<MapFile>,
    /// 静态占据体素（动态障碍不得清掉它们）。
    /// 上一帧动态障碍占据体素。
    prev_dyn: Vec<[usize; 3]>,
    /// 当前机体位置（地图系，米）：感知地图只发它附近的局部窗口。
    perceived_center: Vector3<f64>,
    /// 各体素实体**上次发布的集合**：与当前集合做差得到增量（新增/删除）。
    voxel_state: HashMap<String, HashSet<[i32; 3]>>,
    voxel_geometry: HashMap<String, ([f32; 3], [f32; 3])>,
    /// 体素增量发布的复用缓冲（每 tick 都要用，避免反复分配）。
    voxel_current: HashSet<[i32; 3]>,
    voxel_added: Vec<[i32; 3]>,
    voxel_removed: Vec<[i32; 3]>,
    /// 待发布的占据索引缓冲（感知窗口 / 动态障碍共用各自的 Vec）。
    voxel_indices: Vec<[i32; 3]>,
    motion_indices: Vec<[i32; 3]>,
    /// 状态时钟（秒），只由通过校验的地图系里程计推进。
    t_sim: f64,
    wall_origin: Instant,
    odom: navigation::NavigationReceiver,
    mapped_frames: usize,
    state_input: StateInput,
    received_state: bool,
    state_loss_reported: bool,
    initial_goal: Option<[f64; 3]>,
    depth: Option<Subscriber<DepthImageMessage>>,
    goal_sub: Option<Subscriber<GoalMessage>>,
    /// 强制停止订阅（对照官方 `mandatory_stop` topic；单进程仿真下由外部
    /// 工具经 `planner --mandatory-stop` 注入）。
    stop_sub: Option<Subscriber<MandatoryStopMessage>>,
    /// 收到的最近目标（本 tick 处理一次，处理完清空；快速连续发布取最新）。
    pending_goal: Option<GoalMessage>,
    /// 强制停止待处理标记（poll 置位，step 消费后清零）。
    pending_mandatory_stop: bool,
    /// 最新 odom 携带的 trace 上下文 `(trace_id, span_id, sampled)`（续接用）。
    odom_trace: Option<(u128, u64, bool)>,
    latest_depth: Option<ReceivedDepth>,
    last_depth_timestamp: f64,
    /// 深度流新鲜度监视（超时锁存触发急停）。
    depth_freshness: DepthFreshness,
    /// 深度丢失已报错标记（锁存期内不刷屏；新帧到达即复位）。
    depth_loss_reported: bool,
    depth_cam: DepthCamera,
    finished: bool,
}

impl App {
    /// 接线：地图 → planner → 管理器 → 订阅/发布端口。
    ///
    /// # Errors
    ///
    /// 地图体素化 / 全局引导构造 / IPC 端口创建失败。
    #[allow(clippy::too_many_lines)]
    fn new(
        map_file: Option<MapFile>,
        config: PlannerConfig,
        mapping: config::MappingOptions,
        manager_options: ManagerOptions,
        start: [f64; 3],
        goal: Option<[f64; 3]>,
    ) -> Result<Self> {
        let mut grid = match &map_file {
            Some(map) => map.to_grid_map(config.obstacle_clearance)?,
            None => mapping.build(config.obstacle_clearance)?,
        };
        // 保存心跳超时阈值，供应用层发布门控使用。
        // 虚拟地面/天花板（对照官方 enable_virtual_wall）：任一配置存在即单独生效
        if config.virtual_ground.is_some() || config.virtual_ceiling.is_some() {
            grid.set_virtual_wall(VirtualWall {
                ground: config.virtual_ground.unwrap_or(f64::NEG_INFINITY),
                ceil: config.virtual_ceiling.unwrap_or(f64::INFINITY),
            });
        }
        let yaw_hold = config.yaw_hold;
        let heartbeat_timeout = config.heartbeat_timeout;
        let planner = firefly_planner::Planner::new(config, grid);
        // 初始目标缺省 = 起点：悬停等待外部 `Firefly/Goal` 目标
        let initial_goal = goal;
        let goal = start;
        let manager = PlannerManager::with_planner(
            planner,
            manager_options,
            Vector3::new(start[0], start[1], start[2]),
            Vector3::new(goal[0], goal[1], goal[2]),
        )?;
        // 进程共享节点：所有端口由它派生，进程退出时统一 Drop 释放 IPC 资源
        let node = create_node()?;
        let log_ipc = firefly_observability::init_ipc(&node, "planner");
        // 地图系输入是必需端口；局部 VIO 不能替代它。
        let odom = navigation::NavigationReceiver::start(map_file.is_some())
            .map_err(|e| e.with_operation("创建规划器地图系里程计订阅"))?;
        let depth = open_sub::<DepthImageMessage>(
            &node,
            DEPTH_TOPIC,
            "已订阅深度话题（感知建图输入）",
            "深度订阅不可用，感知建图停用",
        );
        let goal_sub = open_sub::<GoalMessage>(
            &node,
            GOAL_TOPIC,
            "已订阅目标话题（`ffctl planner goal X Y Z` 发布）",
            "目标订阅不可用",
        );
        let stop_sub = open_sub::<MandatoryStopMessage>(
            &node,
            MANDATORY_STOP_TOPIC,
            "已订阅强制停止话题（`planner --mandatory-stop` 发布）",
            "强制停止订阅不可用",
        );
        let output = output::OutputWorker::start(odom.reader(), yaw_hold, heartbeat_timeout)?;
        let viz_pub = match VizPublisher::with_voxels(&node) {
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
            fsm: ReplanFsm::new(manager),
            output,
            published_trajectory: 0,
            manager_options,
            viz_pub,
            log_ipc,
            prev_dyn: Vec::new(),
            perceived_center: Vector3::zeros(),
            voxel_state: HashMap::new(),
            voxel_geometry: HashMap::new(),
            voxel_current: HashSet::new(),
            voxel_added: Vec::new(),
            voxel_removed: Vec::new(),
            voxel_indices: Vec::new(),
            motion_indices: Vec::new(),
            map_file,
            t_sim: 0.0,
            wall_origin: Instant::now(),
            state_input: StateInput::default(),
            mapped_frames: 0,
            received_state: false,
            state_loss_reported: false,
            initial_goal,
            odom,
            depth,
            goal_sub,
            stop_sub,
            pending_goal: None,
            pending_mandatory_stop: false,
            odom_trace: None,
            latest_depth: None,
            last_depth_timestamp: f64::NEG_INFINITY,
            depth_freshness: DepthFreshness::new(),
            depth_loss_reported: false,
            depth_cam: DepthCamera::mujoco_default(),
            finished: false,
        })
    }

    fn poll_navigation(&mut self) -> Result<()> {
        let (state, ctx) = self.odom.snapshot()?;
        self.state_input.replace_snapshot(state);
        if let Some(navigation) = self.state_input.navigation() {
            self.t_sim = navigation.odom.timestamp;
            self.odom_trace = ctx
                .is_traced()
                .then(|| (ctx.trace_id(), ctx.span_id, ctx.sampled()));
        }
        Ok(())
    }

    /// 排空 odom/深度订阅：续接 trace span、锚定 sim 时钟、记录最新快照。
    #[fastrace::trace]
    fn poll_sensors(&mut self) -> Result<()> {
        self.poll_navigation()?;
        if let Some(sub) = &self.depth {
            while let Some(sample) = sub.receive()? {
                let ctx = *sample.user_header();
                let _span = ctx.continue_span("recv-depth");
                // 存样本句柄不拷贝 300KB payload；tick 内联消费后即释放。
                if sample.timestamp.is_finite()
                    && sample.timestamp >= 0.0
                    && sample.timestamp > self.last_depth_timestamp
                {
                    self.last_depth_timestamp = sample.timestamp;
                    self.latest_depth = Some(sample);
                }
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

    /// 深度帧只融合一次，位姿必须来自同一地图系状态源的时间插值。
    #[fastrace::trace]
    fn update_map_from_depth(&mut self, wall_time: f64) {
        if self
            .latest_depth
            .as_ref()
            .is_some_and(|depth| depth.timestamp > self.t_sim)
        {
            return;
        }
        let Some(depth) = self.latest_depth.take() else {
            return;
        };
        let Some(pose) = self.state_input.pose_at(depth.timestamp) else {
            return;
        };
        self.perceived_center = pose.translation.vector;
        if self.map_file.is_none() {
            self.fsm
                .manager_mut()
                .map_mut()
                .move_ring_buffer(pose.translation.vector);
        }
        if update_from_depth(
            self.fsm.manager_mut().map_mut(),
            &self.depth_cam,
            &pose,
            &depth.data,
        ) == 0
        {
            return;
        }
        self.mapped_frames = self.mapped_frames.saturating_add(1);
        self.depth_freshness.observe(wall_time);
        self.depth_loss_reported = false;
    }

    /// 动态障碍按仿真时钟插值，增量更新占据地图（静态体素保护）。
    fn update_motion(&mut self) {
        let Some(map_file) = &self.map_file else {
            return;
        };
        if map_file.motions.is_empty() {
            return;
        }
        let dyn_voxels = map_file.motion_voxels(self.t_sim, self.fsm.manager().map());
        let map = self.fsm.manager_mut().map_mut();
        for idx in &self.prev_dyn {
            map.set_state(*idx, VoxelState::Unknown);
        }
        for idx in &dyn_voxels {
            map.set_state(*idx, VoxelState::Occupied);
        }
        self.prev_dyn = dyn_voxels;
    }

    /// 执行回调：传感器 → 感知/动态地图 → 状态机 → 完整轨迹交付与可视化。
    #[fastrace::trace]
    #[allow(clippy::too_many_lines)]
    fn step(&mut self) -> Result<()> {
        self.output.heartbeat()?;
        // 先消费传感器（更新权威仿真时钟），再取当帧 sim 时刻
        self.poll_sensors()?;
        let now = self.t_sim;
        firefly_observability::set_sim_time(now);
        firefly_observability::pump_log_ipc(&self.log_ipc);

        let Some(snapshot) = self.state_input.current(Instant::now()) else {
            if self.state_input.stopped() && !self.state_loss_reported {
                log::error!("地图系里程计失联或无效，停止发布规划参考；恢复定位后须重启 planner");
                self.state_loss_reported = true;
            }
            self.output.control(None, false, None)?;
            return Ok(());
        };
        let measured = Some(snapshot.state);
        let revision = self
            .state_input
            .navigation()
            .map(|navigation| navigation.revision);
        if self.state_input.take_revision_change() {
            self.fsm.manager_mut().map_mut().clear_observations();
            self.prev_dyn.clear();
            self.fsm.invalidate_localization(now, snapshot.state)?;
            self.published_trajectory = 0;
            self.mapped_frames = 0;
            self.finished = false;
            log::info!("地图修正版本变更：清空局部融合并重新规划，任务目标保持地图坐标");
        }
        if !self.received_state {
            self.fsm
                .manager_mut()
                .set_goal(now, measured, snapshot.state.position.coords)?;
            if let Some(goal) = self.initial_goal {
                self.fsm.set_goal(now, measured, Vector3::from(goal))?;
            }
            self.received_state = true;
            log::info!("地图系状态就绪，规划从实测位置开始");
        }
        let wall_time = self.wall_origin.elapsed().as_secs_f64();

        // 深度感知建图 + 动态障碍写入（规划地图更新先于重规划决策）
        self.update_map_from_depth(wall_time);
        if self.fsm.manager().map().history_exhausted() {
            log::error!("会话障碍地图容量耗尽，停止规划；必须建立新会话");
            self.fsm
                .trigger_emergency_stop_disable_failsafe(now, measured);
            self.output.control(revision, false, None)?;
            return Ok(());
        }
        self.update_motion();

        // 强制停止（官方 mandatoryStopCallback）：置锁存标志、关 fail-safe、
        // 进入急停——此后不自动恢复；先于其他分支处理。
        if self.pending_mandatory_stop {
            self.pending_mandatory_stop = false;
            self.fsm.mandatory_stop(now, measured);
        }

        // 动态目标：收到新目标即重目标（重算全局路径 + 重置状态机），
        // 下一 tick 重新规划飞往新目标
        if let Some(goal) = self.pending_goal.take() {
            let target = Vector3::new(goal.position_x, goal.position_y, goal.position_z);
            match self.fsm.set_goal(now, measured, target) {
                Ok(()) => log::info!(
                    "目标更新 ({:.1},{:.1},{:.1})，重新规划中",
                    target.x,
                    target.y,
                    target.z
                ),
                Err(e) => log::warn!("目标 ({target:?}) 不可达，忽略：{e}"),
            }
        }
        // 至少四次命中才能从 clamp_min 达到占据阈值；初始化与坐标修正均须重新积累。
        if self.mapped_frames < 4 {
            let hold = hover_reference(snapshot.state.position.coords, (0.0, 0.0));
            self.output.control(revision, true, Some(hold))?;
            return Ok(());
        }
        let waiting = !self.fsm.has_target() && self.fsm.manager().local().is_none();
        let hold = waiting.then(|| hover_reference(snapshot.state.position.coords, (0.0, 0.0)));
        self.output.control(revision, true, hold)?;
        let report = self.fsm.exec_tick(now, measured);
        self.handoff_trajectory()?;

        // 可视化：新轨迹 / 无人机位姿 / 动态障碍（经 Firefly/Viz 发布）
        if report.replanned
            && let Some(result) = self.fsm.manager().last_result()
        {
            self.log_line_strip(
                "plan/global_path",
                self.fsm.manager().global_path(),
                (90, 235, 120),
                now,
            );
            self.log_trajectory("plan/local_traj", &result.trajectory, now);
            self.log_planes("plan/planes", &result.planes, now);
            log::info!(
                "replan #{} 完成，时长 {:.2}s",
                self.fsm.manager().replans(),
                result.trajectory.duration()
            );
        }
        let q = snapshot.orientation.quaternion();
        self.log_pose(
            "plan/drone",
            snapshot.state.position.coords.into(),
            [q.i, q.j, q.k, q.w],
            now,
        );
        if let Some(map_file) = &self.map_file {
            let mut indices = std::mem::take(&mut self.motion_indices);
            for m in &map_file.motions {
                let p = m.position_at(now);
                indices.extend(human_voxels(p[0], p[1]));
            }
            self.log_voxels("plan/motions", &indices, now);
            self.motion_indices = indices;
        }
        if report.finished {
            self.finished = true;
        }
        Ok(())
    }

    fn handoff_trajectory(&mut self) -> Result<()> {
        if let Some(local) = self.fsm.manager().local()
            && local.traj_id != self.published_trajectory
        {
            self.output.trajectory(TrajectoryCommand {
                trajectory: local.traj.clone(),
                start_time: local.start_time,
                traj_id: local.traj_id,
            })?;
            self.published_trajectory = local.traj_id;
        }

        Ok(())
    }

    /// 独立安全定时回调，与执行状态机共用单线程事件队列。
    fn safety_step(&mut self) -> Result<()> {
        self.poll_sensors()?;
        let Some(snapshot) = self.state_input.current(Instant::now()) else {
            self.output.control(None, false, None)?;
            return Ok(());
        };
        // 坐标修正和地图预热由执行回调完成，期间不检查旧轨迹。
        if self.state_input.has_revision_change() || self.mapped_frames < 4 {
            return Ok(());
        }
        let now = self.t_sim;
        let measured = Some(snapshot.state);
        let wall_time = self.wall_origin.elapsed().as_secs_f64();
        // 深度/odom 超时急停（官方 checkCollisionCallback：有可执行轨迹且
        // 未完成才监控深度丢失，命中则关 fail-safe 进急停、永不自动恢复）。
        // 锁存后重复命中不刷屏（监视结构锁存 + enter 幂等）。
        if self.fsm.manager().local().is_some()
            && !self.fsm.is_finished()
            && self.depth_freshness.timed_out(wall_time, DEPTH_TIMEOUT)
        {
            if !self.depth_loss_reported {
                log::error!("深度/里程计丢失！进入急停（fail-safe 已禁用）");
                self.depth_loss_reported = true;
            }
            self.fsm
                .trigger_emergency_stop_disable_failsafe(now, measured);
        }

        let _ = self.fsm.safety_tick(now, measured);
        self.handoff_trajectory()
    }

    /// `WaitSet` 分别驱动执行状态机 100Hz 与安全检查 20Hz。
    fn run(&mut self) -> Result<()> {
        // 静态产物一次性记录（全局任务航点不随 tick 重复写）
        self.log_line_strip(
            "plan/global_path",
            self.fsm.manager().global_path(),
            (90, 235, 120),
            self.t_sim,
        );
        log::info!(
            "主循环启动：状态机 100Hz / 安全检查 20Hz / 轨迹输出 100Hz，重规划阈值 {:.1}s，规划视界 {:.0}m",
            self.manager_options.replan_thresh,
            self.manager_options.planning_horizon,
        );
        let mut frame = 0usize;
        let mut last_visualization = Instant::now();
        let waitset = WaitSetBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| Error::new(ErrorKind::Internal, format!("创建 WaitSet 失败: {e:?}")))?;
        let tick_guard = waitset
            .attach_interval(LOOP_PERIOD)
            .map_err(|e| Error::new(ErrorKind::Internal, format!("挂载节拍定时器失败: {e:?}")))?;
        let safety_guard = waitset
            .attach_interval(Duration::from_millis(50))
            .map_err(|e| Error::new(ErrorKind::Internal, format!("挂载安全定时器失败: {e:?}")))?;
        let on_tick = |attachment_id: WaitSetAttachmentId<ipc::Service>| {
            if attachment_id.has_event_from(&safety_guard) {
                if let Err(error) = self.safety_step() {
                    log::warn!("安全检查失败：{error}");
                }
                return CallbackProgression::Continue;
            }
            if !attachment_id.has_event_from(&tick_guard) {
                return CallbackProgression::Continue;
            }
            // 接收线程可能先消费进程终止信号，主循环必须同步检查停止标志。
            if self.output.check().is_err() {
                return CallbackProgression::Stop;
            }
            if self.odom.terminated() {
                return CallbackProgression::Stop;
            }
            // 动态目标工作流：到达后保持运行（悬停 + 等待新目标），
            // 不因 finished 退出进程（仅 SIGINT/SIGTERM 优雅退出）
            // 每帧 trace 上下文：续接新鲜 odom 的 trace（跨进程同周期一条
            // trace），无新鲜 odom 时自建未采样 root（不产生 span 记录）
            let root = match self.odom_trace.filter(|_| !self.state_input.stopped()) {
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
            if last_visualization.elapsed() < Duration::from_millis(100) {
                return CallbackProgression::Continue;
            }
            last_visualization = Instant::now();
            frame += 1;
            // 感知占据地图：每拍只发变化量，周期性全量重同步
            let mut indices = std::mem::take(&mut self.voxel_indices);
            let (size, origin) = Self::occupied_indices(
                self.fsm.manager().map(),
                Some(self.perceived_center),
                &mut indices,
            );
            self.publish_voxel_delta(
                "plan/perceived",
                &indices,
                size,
                origin,
                self.t_sim,
                frame.is_multiple_of(VOXEL_RESYNC_TICKS),
            );
            self.voxel_indices = indices;
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
        self.output.check()?;
        self.odom.snapshot()?;
        log::info!(
            "进程退出：本会话 {} 次重规划，仿真时长 {:.1}s（悬停保持中，等待新目标可继续）",
            self.fsm.manager().replans(),
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
    /// Python 端在完整更新到达后构造 `VoxelGridMap`）。
    fn log_voxels(&mut self, entity: &str, indices: &[[i32; 3]], t: f64) {
        self.publish_voxel_delta(entity, indices, [0.1, 0.1, 0.1], [0.0, 0.0, 0.0], t, false);
    }

    /// 局部窗口内的占据体素索引（`center` 为空则全量栅格），附带网格尺寸与原点。
    ///
    /// 局部窗口以 `center` 为中心取 `local_update_range` 内的格子，限制传输量。
    fn occupied_indices(
        map: &GridMap,
        center: Option<Vector3<f64>>,
        indices: &mut Vec<[i32; 3]>,
    ) -> ([f32; 3], [f32; 3]) {
        let origin = map.origin();
        let dims = map.dims();
        let size = [map.resolution() as f32; 3];
        let at = [origin.x as f32, origin.y as f32, origin.z as f32];
        let (lo, hi) = match center {
            Some(c) => match Self::window_bounds(map, c) {
                Some(bounds) => bounds,
                None => return (size, at),
            },
            None => ([0, 0, 0], [dims[0] - 1, dims[1] - 1, dims[2] - 1]),
        };
        indices.clear();
        for x in lo[0]..=hi[0] {
            for y in lo[1]..=hi[1] {
                for z in lo[2]..=hi[2] {
                    if map.state([x, y, z]) == VoxelState::Occupied {
                        indices.push([x as i32, y as i32, z as i32]);
                    }
                }
            }
        }
        (size, at)
    }

    /// 体素层按**增量**发布：与上次发布的集合做差，只发新增/删除。首次发布、
    /// `force_full` 或差集超单条消息上限时发分块全量快照；消费端原子替换集合。
    fn publish_voxel_delta(
        &mut self,
        entity: &str,
        indices: &[[i32; 3]],
        size: [f32; 3],
        origin: [f32; 3],
        t: f64,
        force_full: bool,
    ) {
        let force_full = force_full
            || self
                .voxel_geometry
                .insert(entity.to_owned(), (size, origin))
                != Some((size, origin));
        self.voxel_current.clear();
        self.voxel_current.extend(indices.iter().copied());
        let (first, full) = {
            match self.voxel_state.get(entity) {
                None => (true, true),
                Some(prev) => {
                    self.voxel_added.clear();
                    self.voxel_removed.clear();
                    self.voxel_added
                        .extend(self.voxel_current.difference(prev).copied());
                    self.voxel_removed
                        .extend(prev.difference(&self.voxel_current).copied());
                    (
                        false,
                        force_full
                            || self.voxel_added.len() > VOXELS_MAX
                            || self.voxel_removed.len() > VOXELS_MAX,
                    )
                }
            }
        };
        if first || full {
            self.publish_voxels(kind::VOXELS, entity, indices, size, origin, t);
        } else {
            if !self.voxel_added.is_empty() {
                self.publish_voxels(kind::VOXELS_ADD, entity, &self.voxel_added, size, origin, t);
            }
            if !self.voxel_removed.is_empty() {
                self.publish_voxels(
                    kind::VOXELS_REMOVE,
                    entity,
                    &self.voxel_removed,
                    size,
                    origin,
                    t,
                );
            }
        }
        // 换入新集合：复用缓冲拿到上一 tick 的集合，下轮直接清空重填（不克隆）。
        match self.voxel_state.get_mut(entity) {
            Some(prev) => std::mem::swap(prev, &mut self.voxel_current),
            None => {
                self.voxel_state
                    .insert(entity.to_string(), std::mem::take(&mut self.voxel_current));
            }
        }
    }

    /// 连续发布完整更新的所有分块；空快照通过一个空块表达。
    fn publish_voxels(
        &self,
        kind: u32,
        entity: &str,
        indices: &[[i32; 3]],
        size: [f32; 3],
        origin: [f32; 3],
        t: f64,
    ) {
        let Some(publisher) = &self.viz_pub else {
            return;
        };
        let chunks = indices.len().div_ceil(VOXELS_MAX).max(1);
        for chunk in 0..chunks {
            let begin = chunk * VOXELS_MAX;
            let end = (begin + VOXELS_MAX).min(indices.len());
            let mut msg = VoxelMessage::base(kind, t, entity);
            msg.voxel_count = (end - begin) as u32;
            msg.voxels[..end - begin].copy_from_slice(&indices[begin..end]);
            msg.voxel_size = size;
            msg.voxel_origin = origin;
            msg.chunk_index = chunk as u32;
            msg.chunk_count = chunks as u32;
            let _ = publisher.publish(msg);
        }
    }

    /// `center` 周围 `local_update_range` 对应的窗口索引范围（闭区间，已裁剪
    /// 到栅格内）；越界返回 `None`。
    fn window_bounds(map: &GridMap, center: Vector3<f64>) -> Option<([usize; 3], [usize; 3])> {
        let range = Vector3::new(
            PERCEIVED_VIEW_RADIUS_M,
            PERCEIVED_VIEW_RADIUS_M,
            PERCEIVED_VIEW_RADIUS_M,
        );
        let origin = map.origin();
        let res = map.resolution();
        let dims = map.dims();
        let mut lo = [0usize; 3];
        let mut hi = [0usize; 3];
        for axis in 0..3 {
            let min = ((center[axis] - range[axis] - origin[axis]) / res).floor();
            let max = ((center[axis] + range[axis] - origin[axis]) / res).ceil();
            let min = min.max(0.0) as usize;
            let max = (max.max(0.0) as usize).min(dims[axis].saturating_sub(1));
            if min > max || dims[axis] == 0 {
                return None;
            }
            lo[axis] = min;
            hi[axis] = max;
        }
        Some((lo, hi))
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
/// 计时基准是管线**实际吃进一帧**（深度+位姿齐备并写入地图）的墙钟时刻，
/// 深度或位姿任一断流即饥饿，超时置位后锁存，直到管线再次吃进新帧才复位。
struct DepthFreshness {
    /// 最近一次实际建图消费的墙钟秒数（相对进程启动）；首帧之前为 `None`。
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
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!(
                "{e}\n用法：planner [--map <map.ffmap>] [--config configs/planner.toml] [--start x y z] [--goal x y z] [--mandatory-stop]\n\n--goal 可省略（悬停等待 `ffctl planner goal X Y Z` 动态目标）；--mandatory-stop 向 {MANDATORY_STOP_TOPIC} 发一条强制停止指令后退出"
            );
            std::process::exit(2);
        }
    };
    if args.mandatory_stop {
        init_observability();
        if let Err(e) = emit_mandatory_stop() {
            log::error!("强制停止指令发布失败：{e}");
            firefly_observability::flush();
            std::process::exit(1);
        }
        log::info!("已发布强制停止指令到 {MANDATORY_STOP_TOPIC}");
        firefly_observability::flush();
        return;
    }
    let toml_cfg = match firefly_config::startup::<config::PlannerToml>(
        &args.config,
        init_observability,
        |_| Ok(()),
    ) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("加载配置失败：{e}");
            std::process::exit(1);
        }
    };
    let map_file = args
        .map
        .as_ref()
        .map(MapFile::from_file)
        .transpose()
        .unwrap_or_else(|e| {
            log::error!("加载显式场地地图失败：{e}");
            std::process::exit(1);
        });
    // 可视化发布端在 App::new 内随进程共享节点创建（经 Firefly/Viz 话题，
    // firefly-viz 进程统一写 rerun）
    match App::new(
        map_file,
        toml_cfg.config,
        toml_cfg.mapping,
        toml_cfg.manager,
        args.start,
        args.goal,
    ) {
        Ok(mut app) => {
            // 静态先验一次性记录（体素索引收集在发布端完成）
            let grid = app.fsm.manager().map().clone();
            let mut indices = Vec::new();
            let (size, origin) = App::occupied_indices(&grid, None, &mut indices);
            app.publish_voxel_delta("plan/map", &indices, size, origin, 0.0, true);
            if let Err(e) = app.run() {
                log::error!("planner 失败：{e}");
                firefly_observability::pump_log_ipc(&app.log_ipc);
                firefly_observability::flush();
                drop(app);
                std::process::exit(1);
            }
            firefly_observability::pump_log_ipc(&app.log_ipc);
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
}
