mod config;
mod depth_mapping;
mod map_input;
mod output;
#[cfg(test)]
mod tests;
mod voxel_display;

use std::path::PathBuf;
use std::time::{Duration, Instant};

use fastrace::prelude::*;
use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{Plane, VirtualWall};
use firefly_observability::{init as init_observability, rate::Rate};
use firefly_odometry::{OdometryReceiver, Snapshot, StateInput};
use firefly_planner::worker::{MapVersion, PlanningWorker, WorkerStats};
use firefly_planner::{ManagerOptions, PlannerConfig, PlannerManager, ReplanFsm, State};
use firefly_pubsub::goal::{GOAL_TOPIC, GoalMessage, PASS};
use firefly_pubsub::node::create_node;
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::request::MissionRequest;
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::trajectory::{MANDATORY_STOP_TOPIC, MandatoryStopMessage};
use firefly_pubsub::viz::{
    ARROWS_MAX, POINTS_MAX, ScalarMessage, VIZ_TOPIC, VizMessage, VizPublisher, kind,
};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetRunResult;
use nalgebra::Vector3;

use map_input::MapStatus;

/// 主循环频率（官方 `exec_timer` 0.01s）。
const LOOP_PERIOD: Duration = Duration::from_millis(10);
/// 地图融合新鲜度阈值（秒），沿用官方 `grid_map/odom_depth_timeout`。
/// 默认值 1.0（`plan_env/src/grid_map.cpp`）。
const MAP_TIMEOUT: f64 = 1.0;
/// 缺省配置：按安装布局解析，与启动目录无关。
fn default_config() -> PathBuf {
    firefly_config::paths::config("planner.toml")
}

/// 里程计提供位置和速度；从观测起算时采用零加速度边界。
fn planning_state(sample: Snapshot) -> State {
    State {
        position: sample.position,
        velocity: sample.velocity,
        acceleration: Vector3::zeros(),
    }
}

/// 运行轨迹规划或发送强制停止指令
#[derive(usage_rs::Cli)]
#[usage(bin = "planner")]
struct Arguments {
    /// 应用配置文件
    #[usage(long)]
    config: Option<PathBuf>,
    /// 初始位置（米），默认位于原点
    #[usage(long, num_args = 3, value_names = ["X", "Y", "Z"], allow_negative_numbers)]
    start: Option<Vec<f64>>,
    /// 初始目标（米），省略时等待动态目标
    #[usage(long, num_args = 3, value_names = ["X", "Y", "Z"], allow_negative_numbers)]
    goal: Option<Vec<f64>>,
    /// 向 `Firefly/MandatoryStop` 发一条强制停止指令后退出
    #[usage(long, exclusive)]
    mandatory_stop: bool,
}

/// 打开可选订阅器：失败降级为 `None` 并记录（辅助输入流可缺失，状态门控仍生效）。
fn log_subscription<T: std::fmt::Debug + ZeroCopySend + 'static>(
    port: Result<Subscriber<T>>,
    topic: &str,
    ok_msg: &str,
    err_msg: &str,
) -> Option<Subscriber<T>> {
    match port {
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

enum Passage {
    Inactive,
    Approaching {
        point: Vector3<f64>,
        successor: Vector3<f64>,
    },
    Passed,
}
#[allow(clippy::struct_excessive_bools)]
struct App {
    fsm: ReplanFsm,
    solver: PlanningWorker,
    output: output::OutputPublisher,
    published_trajectory: u32,
    /// 重复目标只更新目标身份，轨迹未变；仍需重发当前轨迹以携带新的 `goal_timestamp`。
    republish_trajectory: bool,
    /// 管理器行为参数（来自配置，启动日志展示）。
    manager_options: ManagerOptions,
    view_half_angle: f64,
    /// 可视化发布端（经 `Firefly/Viz` 话题，`mimir` 进程统一写 rerun）。
    viz_pub: Option<VizPublisher>,
    /// 日志聚合句柄（主循环作用域持有，每 tick 传给 `pump_log_ipc`；
    /// 发布端只活在创建线程，无跨线程触碰，见 `firefly-observability`）。
    log_ipc: firefly_observability::LogIpc,
    /// 当前机体位置（odom 系，米）：感知地图只发它附近的局部窗口。
    perceived_center: Vector3<f64>,
    display: voxel_display::DisplayWorker,
    /// 状态时钟（秒），只由通过校验的局部里程计推进。
    t_sim: f64,
    odom: OdometryReceiver,
    state_input: StateInput,
    received_state: bool,
    /// 本拍是否拿到新鲜里程计：决定体素快照是否重扫（无状态时不扫）。
    state_fresh: bool,
    state_loss_reported: bool,
    initial_goal: Option<[f64; 3]>,
    depth: depth_mapping::DepthReceiver,
    map_input: map_input::MapInput,
    goal_sub: Option<Subscriber<GoalMessage>>,
    /// 强制停止订阅（对照官方 `mandatory_stop` topic；单进程仿真下由外部
    /// 工具经 `planner --mandatory-stop` 注入）。
    stop_sub: Option<Subscriber<MandatoryStopMessage>>,
    /// 收到的最近目标（本 tick 处理一次，处理完清空；快速连续发布取最新）。
    pending_goal: Option<GoalMessage>,
    /// Current pass point and its successor, both in the local odometry frame.
    passage: Passage,
    /// 强制停止待处理标记（poll 置位，step 消费后清零）。
    pending_mandatory_stop: bool,
    /// 最新 odom 携带的 trace 上下文 `(trace_id, span_id, sampled)`（续接用）。
    odom_trace: Option<(u128, u64, bool)>,
    loop_rate: Rate,
    fsm_rate: Rate,
    submitted_rate: Rate,
    replan_rate: Rate,
    previous_step: Option<Instant>,
    fsm_time: Option<Duration>,
}

impl App {
    /// 接线：地图 → planner → 管理器 → 订阅/发布端口。
    ///
    /// # Errors
    ///
    /// 地图体素化 / 全局引导构造 / IPC 端口创建失败。
    #[allow(clippy::too_many_lines)]
    fn new(
        config: PlannerConfig,
        mapping: &config::MappingOptions,
        manager_options: ManagerOptions,
        start: [f64; 3],
        goal: Option<[f64; 3]>,
    ) -> Result<Self> {
        let mut grid = mapping.build(config.obstacle_clearance)?;
        let [fx, _, cx, _] = mapping.depth.intrinsics;
        let view_half_angle =
            (cx.min(firefly_pubsub::camera::IMAGE_WIDTH as f64 - 1. - cx) / fx).atan();
        let depth_fusion =
            depth_mapping::DepthFusion::new(&grid, mapping.depth, config.obstacle_clearance)?;
        if config.virtual_ground.is_some() || config.virtual_ceiling.is_some() {
            grid.set_virtual_wall(VirtualWall {
                ground: config.virtual_ground.unwrap_or(f64::NEG_INFINITY),
                ceil: config.virtual_ceiling.unwrap_or(f64::INFINITY),
            });
        }
        let planner = firefly_planner::Planner::new(config, grid);
        let initial_goal = goal;
        let goal = start;
        let manager = PlannerManager::with_planner(
            planner,
            manager_options,
            Vector3::new(start[0], start[1], start[2]),
            Vector3::new(goal[0], goal[1], goal[2]),
        )?;
        let node = create_node()?;
        let log_ipc = firefly_observability::init_ipc(&node, "planner");
        let odom =
            OdometryReceiver::start().map_err(|e| e.with_operation("创建规划器局部里程计订阅"))?;
        let depth = depth_mapping::DepthReceiver::start(depth_fusion, odom.history())?;
        let goal_sub = log_subscription(
            GoalMessage::subscriber(&node),
            GOAL_TOPIC,
            "已订阅目标话题（`ffctl planner goal X Y Z` 发布）",
            "目标订阅不可用",
        );
        let stop_sub = log_subscription(
            Subscriber::<MandatoryStopMessage>::with_topic(&node, MANDATORY_STOP_TOPIC),
            MANDATORY_STOP_TOPIC,
            "已订阅强制停止话题（`planner --mandatory-stop` 发布）",
            "强制停止订阅不可用",
        );
        let output = output::OutputPublisher::new(&node)?;
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
            fsm: ReplanFsm::new(manager),
            solver: PlanningWorker::new()
                .map_err(|e| Error::caused_by(ErrorKind::Internal, "创建求解线程失败", e))?,
            output,
            published_trajectory: 0,
            republish_trajectory: false,
            manager_options,
            viz_pub,
            view_half_angle,
            log_ipc,
            perceived_center: Vector3::zeros(),
            display: voxel_display::DisplayWorker::start()?,
            t_sim: 0.0,
            state_input: StateInput::default(),
            received_state: false,
            state_fresh: false,
            state_loss_reported: false,
            initial_goal,
            odom,
            depth,
            map_input: map_input::MapInput::default(),
            goal_sub,
            stop_sub,
            pending_goal: None,
            passage: Passage::Inactive,
            pending_mandatory_stop: false,
            odom_trace: None,
            loop_rate: Rate::default(),
            fsm_rate: Rate::default(),
            submitted_rate: Rate::default(),
            replan_rate: Rate::default(),
            previous_step: None,
            fsm_time: None,
        })
    }

    fn poll_odometry(&mut self) -> Result<()> {
        let (state, ctx) = self.odom.snapshot()?;
        self.state_input.replace_snapshot(state);
        if let Some(odometry) = self.state_input.odometry() {
            self.t_sim = odometry.timestamp;
            self.odom_trace = ctx
                .is_traced()
                .then(|| (ctx.trace_id(), ctx.span_id, ctx.sampled()));
        }
        Ok(())
    }

    /// 排空 odom/地图订阅：续接 trace span、锚定 sim 时钟、记录最新快照。
    #[fastrace::trace]
    fn poll_sensors(&mut self) -> Result<()> {
        let started = Instant::now();
        self.poll_odometry()?;
        if let Some(snapshot) = self.depth.take_snapshot()? {
            self.map_input.observe_local(snapshot)?;
        }
        if let Some(sub) = &self.goal_sub {
            while let Some(sample) = sub.receive()? {
                if !sample.is_valid()
                    || sample.timestamp
                        <= self
                            .pending_goal
                            .map_or(self.output.goal_timestamp, |goal| goal.timestamp)
                {
                    continue;
                }
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
                self.pending_mandatory_stop = true;
            }
        }
        log_slow_stage("sensors", started);
        Ok(())
    }

    /// 执行回调：传感器 → 局部深度地图 → 状态机 → 完整轨迹交付与可视化。
    #[fastrace::trace]
    fn step(&mut self) -> Result<()> {
        let started = Instant::now();
        let interval = self
            .previous_step
            .replace(started)
            .map_or(Duration::ZERO, |previous| started.duration_since(previous));
        self.fsm_time = None;
        let before = self.solver.stats();
        self.exec_step()?;
        if self.state_fresh {
            self.output.status(
                self.t_sim,
                self.fsm.is_finished()
                    && !self.fsm.is_emergency()
                    && !self.fsm.is_mandatory_stopped(),
                matches!(self.passage, Passage::Passed),
            )?;
        }
        self.publish_performance(before, started, interval);
        Ok(())
    }

    fn publish_performance(&mut self, before: WorkerStats, started: Instant, interval: Duration) {
        if let Some(viz) = &self.viz_pub {
            let after = self.solver.stats();
            let now = Instant::now();
            let step_ms = now.duration_since(started).as_secs_f64() * 1000.;
            let publish = |entity: &str, values: &[f64]| {
                let mut message = ScalarMessage::base(self.t_sim, entity);
                message.scalar_count = values.len() as u32;
                message.scalars[..values.len()].copy_from_slice(values);
                let _ = viz.publish(message);
            };
            for (entity, rate) in [
                ("planner/loop_hz", self.loop_rate.observe(now, 1)),
                (
                    "planner/fsm_hz",
                    self.fsm_rate
                        .observe(now, u64::from(self.fsm_time.is_some())),
                ),
                (
                    "planner/replan_submitted_hz",
                    self.submitted_rate
                        .observe(now, after.submitted - before.submitted),
                ),
                (
                    "planner/replan_hz",
                    self.replan_rate
                        .observe(now, after.adopted - before.adopted),
                ),
            ] {
                if let Some(rate) = rate {
                    publish(entity, &[rate]);
                }
            }
            publish(
                "planner/fsm_timing_ms",
                &[
                    self.fsm_time
                        .map_or(0., |duration| duration.as_secs_f64() * 1000.),
                    step_ms,
                    interval.as_secs_f64() * 1000.,
                ],
            );
            if after.adopted + after.discarded > before.adopted + before.discarded {
                publish(
                    "planner/replan_ms",
                    &[
                        after.completion_time.as_secs_f64() * 1000.,
                        after.solve_time.as_secs_f64() * 1000.,
                        f64::from(after.adopted > before.adopted),
                    ],
                );
            }
        }
    }

    #[allow(clippy::too_many_lines)]
    fn exec_step(&mut self) -> Result<()> {
        self.state_fresh = false;
        self.output.heartbeat()?;
        self.poll_sensors()?;
        let now = self.t_sim;
        if self.pending_mandatory_stop {
            self.pending_mandatory_stop = false;
            self.solver.cancel();
            self.fsm.mandatory_stop();
        }
        firefly_observability::set_sim_time(now);
        firefly_observability::pump_log_ipc(&self.log_ipc);

        let Some(snapshot) = self.state_input.current(Instant::now()) else {
            self.solver.cancel();
            if self.state_input.stopped() && !self.state_loss_reported {
                log::error!("局部里程计失联或无效，停止发布规划参考；恢复定位后须重启 planner");
                self.state_loss_reported = true;
            }
            self.output.control(false)?;
            return Ok(());
        };
        self.state_fresh = true;
        let measured = Some(planning_state(snapshot));
        if !self.received_state {
            self.fsm
                .manager_mut()
                .set_goal(now, measured, snapshot.position.coords)?;
            if let Some(goal) = self.initial_goal {
                self.fsm.set_goal(now, measured, Vector3::from(goal))?;
            }
            self.received_state = true;
            log::info!("局部状态就绪，规划从实测位置开始");
        }
        let odometry = self.state_input.odometry().unwrap();
        if self.synchronize_map(&odometry) != MapStatus::Ready {
            self.solver.cancel();
            self.output.control(false)?;
            return Ok(());
        }
        self.perceived_center = snapshot.position.coords;
        self.fsm
            .manager_mut()
            .update_motion_context(firefly_planner::motion::MotionContext {
                measured_at: odometry.timestamp,
                evidence_max_age: Duration::from_secs_f64(MAP_TIMEOUT),
                measured_yaw: snapshot.orientation.euler_angles().2,
                view_half_angle: self.view_half_angle,
            });

        if let Some(goal) = self.pending_goal.take() {
            let target = Vector3::new(goal.position_x, goal.position_y, goal.position_z);
            let next = Vector3::from(goal.next_position);
            let repeated = (target - self.fsm.manager().goal().coords).norm() < 1e-6;
            match self.fsm.set_goal(now, measured, target) {
                Ok(()) => {
                    let heading_changed = self
                        .fsm
                        .set_goal_yaw((goal.has_yaw == 1).then_some(goal.yaw));
                    if !self.fsm.is_mandatory_stopped() {
                        self.output.goal_timestamp = goal.timestamp;
                        self.passage = if goal.arrival == PASS {
                            Passage::Approaching {
                                point: target,
                                successor: next,
                            }
                        } else {
                            Passage::Inactive
                        };
                        self.republish_trajectory = repeated && !heading_changed;
                    }
                    log::info!(
                        "目标更新 ({:.1},{:.1},{:.1})，重新规划中",
                        target.x,
                        target.y,
                        target.z
                    );
                }
                Err(e) => log::warn!("目标 ({target:?}) 不可达，忽略：{e}"),
            }
        }
        if let Passage::Approaching { point, successor } = self.passage
            && !self.fsm.is_mandatory_stopped()
            && !self.fsm.is_emergency()
            && (point - snapshot.position.coords).norm() <= 0.35
        {
            self.passage = Passage::Inactive;
            match self.fsm.set_goal(now, measured, successor) {
                Ok(()) => self.passage = Passage::Passed,
                Err(error) => log::warn!("途经点的后继目标不可达：{error}"),
            }
        }
        self.output.control(true)?;
        let fsm_started = Instant::now();
        let mut report = self.fsm.exec_tick_deferred(now, measured);
        self.fsm_time = Some(fsm_started.elapsed());
        let map_version = MapVersion {
            localization: 0,
            geometry: self.fsm.manager().map().revision(),
        };
        report.replanned |= self.solver.tick(&mut self.fsm, now, measured, map_version);
        self.handoff_trajectory()?;

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
            snapshot.position.coords.into(),
            [q.i, q.j, q.k, q.w],
            now,
        );
        Ok(())
    }

    fn handoff_trajectory(&mut self) -> Result<()> {
        self.poll_odometry()?;
        let ready = match self.state_input.odometry() {
            Some(odom) if !self.state_input.stopped() => {
                self.synchronize_map(&odom) == MapStatus::Ready
            }
            _ => false,
        };
        if !ready {
            self.solver.cancel();
            self.output.control(false)?;
            return Ok(());
        }
        if let Some(local) = self.fsm.manager().local()
            && (local.traj_id != self.published_trajectory || self.republish_trajectory)
        {
            self.output.trajectory(local)?;
            if self.published_trajectory == 0 {
                log::info!(
                    "planner trajectory committed: trajectory={} goal_timestamp={}",
                    local.traj_id,
                    self.output.goal_timestamp
                );
            }
            self.published_trajectory = local.traj_id;
            self.republish_trajectory = false;
        }

        Ok(())
    }

    /// Execution and safety callbacks must query the same accepted map snapshot.
    fn synchronize_map(&mut self, odometry: &firefly_pubsub::odom::OdomMessage) -> MapStatus {
        let started = Instant::now();
        if self.depth.terminated() {
            return MapStatus::Unavailable;
        }
        let status = self.map_input.synchronize(
            self.fsm.manager_mut().map_mut(),
            odometry,
            Instant::now(),
            Duration::from_secs_f64(MAP_TIMEOUT),
        );
        log_slow_stage("map", started);
        status
    }

    /// 独立安全定时回调，与执行状态机共用单线程事件队列。
    fn safety_step(&mut self) -> Result<()> {
        self.poll_sensors()?;
        if self.pending_mandatory_stop {
            self.pending_mandatory_stop = false;
            self.solver.cancel();
            self.fsm.mandatory_stop();
        }
        let Some(snapshot) = self.state_input.current(Instant::now()) else {
            self.solver.cancel();
            self.output.control(false)?;
            return Ok(());
        };
        let now = self.t_sim;
        let measured = Some(planning_state(snapshot));
        let odometry = self.state_input.odometry().unwrap();
        let status = self.synchronize_map(&odometry);
        if status != MapStatus::Ready {
            self.solver.cancel();
            if status == MapStatus::Unavailable
                && self.fsm.manager().local().is_some()
                && !self.fsm.is_finished()
            {
                if !self.fsm.is_emergency() {
                    log::warn!("深度输入过期或停止：锁存急停 t={now}");
                }
                self.fsm.trigger_emergency_stop_disable_failsafe();
            }
            self.output.control(false)?;
            return Ok(());
        }

        self.fsm
            .manager_mut()
            .update_motion_context(firefly_planner::motion::MotionContext {
                measured_at: odometry.timestamp,
                evidence_max_age: Duration::from_secs_f64(MAP_TIMEOUT),
                measured_yaw: snapshot.orientation.euler_angles().2,
                view_half_angle: self.view_half_angle,
            });
        let report = self.fsm.safety_tick_deferred(now);
        if report.emergency_stop || self.fsm.is_mandatory_stopped() {
            self.solver.cancel();
            let _ = self.fsm.exec_tick_deferred(now, measured);
        }
        self.handoff_trajectory()
    }

    /// `WaitSet` 分别驱动执行状态机 100Hz 与安全检查 20Hz。
    fn run(&mut self) -> Result<()> {
        self.log_line_strip(
            "plan/global_path",
            self.fsm.manager().global_path(),
            (90, 235, 120),
            self.t_sim,
        );
        log::info!(
            "主循环启动：状态机 100Hz / 安全检查 20Hz，重规划阈值 {:.1}s，规划视界 {:.0}m",
            self.manager_options.replan_thresh,
            self.manager_options.planning_horizon,
        );
        let mut frame = 0usize;
        let mut last_visualization = Instant::now();
        let waitset = WaitSetBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| Error::caused_by(ErrorKind::Internal, "创建 WaitSet 失败", e))?;
        let tick_guard = waitset
            .attach_interval(LOOP_PERIOD)
            .map_err(|e| Error::caused_by(ErrorKind::Internal, "挂载节拍定时器失败", e))?;
        let safety_guard = waitset
            .attach_interval(Duration::from_millis(50))
            .map_err(|e| Error::caused_by(ErrorKind::Internal, "挂载安全定时器失败", e))?;
        let on_tick = |attachment_id: WaitSetAttachmentId<ipc::Service>| {
            if attachment_id.has_event_from(&safety_guard) {
                let started = Instant::now();
                let result = self.safety_step();
                log_slow_stage("safety", started);
                if let Err(error) = result {
                    log::warn!("安全检查失败：{error}");
                    return self.stop_failed_callback();
                }
                return CallbackProgression::Continue;
            }
            if !attachment_id.has_event_from(&tick_guard) {
                return CallbackProgression::Continue;
            }
            // 接收线程可能先消费进程终止信号，主循环必须同步检查停止标志。
            if self.odom.terminated() || self.depth.terminated() {
                self.stop_failed_callback();
                return CallbackProgression::Stop;
            }
            let root = match self.odom_trace.filter(|_| !self.state_input.stopped()) {
                Some((tid, sid, sampled)) => Span::root(
                    "planner",
                    SpanContext::new(TraceId(tid), SpanId(sid)).sampled(sampled),
                ),
                None => Span::root("planner", SpanContext::random().sampled(false)),
            };
            let guard = root.set_local_parent();
            let started = Instant::now();
            let step = self.step();
            log_slow_stage("execution", started);
            drop(guard);
            drop(root);
            if let Err(error) = step {
                log::warn!("tick 失败：{error}");
                return self.stop_failed_callback();
            }
            if last_visualization.elapsed() < Duration::from_millis(100) {
                return CallbackProgression::Continue;
            }
            last_visualization = Instant::now();
            frame += 1;
            let started = Instant::now();
            if let Err(error) = self.publish_collision_map(frame) {
                log::error!("voxel display failed: {error}");
                self.stop_failed_callback();
                return CallbackProgression::Stop;
            }
            log_slow_stage("display-submit", started);
            CallbackProgression::Continue
        };

        match waitset.wait_and_process(on_tick) {
            Ok(WaitSetRunResult::Interrupt | WaitSetRunResult::TerminationRequest) => {
                log::info!("收到终止信号，优雅退出");
            }
            Ok(_) => {}
            Err(e) => {
                return Err(Error::caused_by(
                    ErrorKind::Internal,
                    "WaitSet 事件等待失败",
                    e,
                ));
            }
        }
        self.odom.snapshot()?;
        if let Some(snapshot) = self.depth.take_snapshot()? {
            self.map_input.observe_local(snapshot)?;
        }
        self.solver.cancel();
        self.output.control(false)?;
        log::info!(
            "进程退出：当前目标 {} 次重规划，仿真时长 {:.1}s，规划状态 {:?}，轨迹正常结束 {}",
            self.fsm.manager().replans(),
            self.t_sim,
            self.fsm.state(),
            self.fsm.is_finished()
        );
        Ok(())
    }

    /// Hand off an immutable view for display outside the FSM thread.
    fn publish_collision_map(&self, frame: usize) -> Result<()> {
        if self.state_fresh {
            self.display.submit(voxel_display::Request {
                map: self.fsm.manager().map().clone(),
                center: self.perceived_center,
                timestamp: self.t_sim,
                force_full: frame.is_multiple_of(voxel_display::VOXEL_RESYNC_TICKS),
            })?;
        }
        Ok(())
    }

    fn stop_failed_callback(&mut self) -> CallbackProgression {
        self.solver.cancel();
        if let Err(error) = self.output.control(false) {
            log::error!("停止规划输出失败：{error}");
            return CallbackProgression::Stop;
        }
        CallbackProgression::Continue
    }
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
        self.log_line_strip(entity, &pts, (255, 96, 96), t);
        let mut msg = VizMessage::base(kind::ARROWS, t, &format!("{entity}/velocity"));
        msg.color = [255, 96, 96];
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

fn log_slow_stage(stage: &str, started: Instant) {
    let elapsed = started.elapsed();
    if elapsed >= Duration::from_millis(10) {
        log::debug!(target: "planner-profile", "stage={stage} elapsed_ms={}", elapsed.as_secs_f64() * 1000.);
    }
}

fn main() -> std::process::ExitCode {
    let args = Arguments::parse();
    let (Ok(start), Ok(goal)) = (
        args.start.map(TryInto::try_into).transpose(),
        args.goal.map(TryInto::try_into).transpose(),
    ) else {
        eprintln!("--start 和 --goal 各接受一组 X Y Z 坐标");
        return std::process::ExitCode::from(2);
    };
    let result = run(args.config, args.mandatory_stop, start, goal);
    firefly_observability::flush();
    result.unwrap_or_else(|e| firefly_error::report("planner", Err(e)))
}

fn run(
    config: Option<PathBuf>,
    mandatory_stop: bool,
    start: Option<[f64; 3]>,
    goal: Option<[f64; 3]>,
) -> Result<std::process::ExitCode> {
    if mandatory_stop {
        init_observability();
        let result = emit_mandatory_stop()
            .inspect(|()| log::info!("已发布强制停止指令到 {MANDATORY_STOP_TOPIC}"));
        return Ok(firefly_error::report_with(result, |e| {
            log::error!("强制停止指令发布失败：{e}");
        }));
    }
    let cfg = firefly_config::startup::<config::PlannerToml>(
        config.unwrap_or_else(default_config),
        init_observability,
        |_| Ok(()),
    )?;
    let mut app = App::new(
        cfg.config,
        &cfg.mapping,
        cfg.manager,
        start.unwrap_or([0.0; 3]),
        goal,
    )?;
    let code = firefly_error::report_with(app.run(), |error| log::error!("planner 失败：{error}"));
    firefly_observability::pump_log_ipc(&app.log_ipc);
    Ok(code)
}
