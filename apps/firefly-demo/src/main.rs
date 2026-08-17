//! 10Hz 重规划主循环演示（仿官方 `ego_replan_fsm`）。
//!
//! 运行：`cargo run -p firefly-demo -- --map apps/firefly-demo/maps/gate.ffmap`
//!
//! 地图为 `FFMap` 标准格式（见 `docs/map-format.md`），可含动态障碍：
//! 主循环每 tick 按航点插值更新障碍占据后重规划。
//!
//! **VIO 接入**：订阅 `apps/vio` 发布的 odom/imu（iceoryx2 + trace 上下文）。
//! - odom 新鲜（`ODOM_FRESH_TIMEOUT` 内）时作为**状态源**（replan 起点 +
//!   无人机位置），VIO 世界系经 `frame_offset`（= `--start`，合成标定）对齐地图系；
//! - vio 进程未启动时回退轨迹推进模拟（独立运行不受影响）；
//! - 每条 odom/imu 消息 `continue_span` 续接 trace，跨进程 span 树可观测。
//!
//! 主循环语义（对应官方 `execFSMCallback`）：
//! - `EXEC_TRAJ`：轨迹按时间推进，`t_cur > replan_thresh` 触发重规划；
//! - `REPLAN_TRAJ`：起点取当前状态（odom 或轨迹），目标取全局路径上
//!   `planning_horizon` 处的点（官方 `getLocalTarget`），规划失败保持旧轨迹；
//! - 无人机与目标距离 `< ARRIVE_DIST` 即任务完成，退出循环。

use std::collections::HashSet;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use fastrace::prelude::*;
use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{GridMap, MapFile, VoxelState};
use firefly_observability::init as init_observability;
use firefly_planner::{PlanResult, Planner, PlannerConfig, State};
use firefly_pubsub::imu::{IMU_TOPIC, ImuSubscriber};
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::reference::{REFERENCE_TOPIC, ReferenceMessage};
use firefly_pubsub::subscriber::OdomSubscriber;
use firefly_search::Astar;
use firefly_trajectory::Trajectory;
use firefly_viewer::Viewer;
use nalgebra::{Point3, Vector3};

/// 官方 `fsm/thresh_replan_time`（`advanced_param.xml`）。
const REPLAN_THRESH: f64 = 1.0;
/// 官方 `planning_horizon`（`advanced_param.xml`）。
const PLANNING_HORIZON: f64 = 6.0;
/// 主循环频率（官方 `exec_timer` 0.1s）。
const LOOP_PERIOD: Duration = Duration::from_millis(100);
/// odom 新鲜度阈值（秒）：超过该时长未收到 odom 则回退轨迹模拟状态源。
const ODOM_FRESH_TIMEOUT: f64 = 1.0;
/// 到达判定距离（米）：无人机与目标距离小于该值即任务完成。
const ARRIVE_DIST: f64 = 0.5;

struct Args {
    map: PathBuf,
    save: Option<PathBuf>,
    start: [f64; 3],
    goal: [f64; 3],
    /// VIO 世界系 → 地图系平移（MuJoCo 闭环下 vio 已在地图系，默认 0）。
    frame_offset: [f64; 3],
}

fn parse_args() -> Result<Args> {
    let mut map = None;
    let mut save = None;
    let mut start = [1.0, 4.0, 1.0];
    let mut goal = [27.0, 4.0, 1.0];
    let mut frame_offset = [0.0, 0.0, 0.0];
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--map" => map = Some(PathBuf::from(it.next().unwrap_or_default())),
            "--save" => save = Some(PathBuf::from(it.next().unwrap_or_default())),
            "--start" => start = parse_vec3(&mut it, "start")?,
            "--goal" => goal = parse_vec3(&mut it, "goal")?,
            "--frame-offset" => frame_offset = parse_vec3(&mut it, "frame-offset")?,
            other => {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    format!("unknown argument: {other}"),
                ));
            }
        }
    }
    let map = map.ok_or_else(|| Error::new(ErrorKind::InvalidArgument, "missing --map"))?;
    Ok(Args {
        map,
        save,
        start,
        goal,
        frame_offset,
    })
}

fn parse_vec3(it: &mut impl Iterator<Item = String>, name: &str) -> Result<[f64; 3]> {
    let mut v = [0.0; 3];
    for c in &mut v {
        *c = it
            .next()
            .ok_or_else(|| Error::new(ErrorKind::InvalidArgument, format!("missing {name} value")))?
            .parse()
            .map_err(|e| {
                Error::new(ErrorKind::InvalidArgument, format!("invalid {name} value"))
                    .with_source(e)
            })?;
    }
    Ok(v)
}

/// 执行中的局部轨迹（官方 `LocalTrajData`：轨迹 + 起始时刻）。
struct LocalTraj {
    traj: Trajectory,
    start_time: f64,
}

/// 最新 odom 快照（VIO 世界系 → 地图系变换后的状态）。
struct OdomSnapshot {
    /// 地图系下的状态（已施加 `frame_offset`）。
    state: State,
}

struct Demo {
    planner: Planner,
    viewer: Viewer,
    map_file: MapFile,
    /// 静态占据体素（动态障碍不得清掉它们）。
    static_occupied: HashSet<[usize; 3]>,
    /// 上一帧动态障碍占据体素。
    prev_dyn: Vec<[usize; 3]>,
    /// 全局路径点（官方 `global_traj`，首次规划后缓存）。
    global_path: Vec<Vector3<f64>>,
    local: Option<LocalTraj>,
    goal: Point3<f64>,
    /// 仿真时钟（秒），替代官方 `ros::Time::now()`。
    t_sim: f64,
    /// 目标是终点（`touch_goal`）：轨迹执行完毕即任务完成。
    touch_goal: bool,
    /// 是否完成。
    finished: bool,
    replans: usize,
    /// VIO 世界系 → 地图系变换（合成标定：vio 原点 = 任务起点 `--start`）。
    frame_offset: Vector3<f64>,
    /// odom 订阅（vio 进程未启动时为 `None`，回退轨迹模拟状态源）。
    odom: Option<OdomSubscriber>,
    /// imu 订阅（同 odom，可观测原始 IMU）。
    imu: Option<ImuSubscriber>,
    /// 最新 odom 快照（地图系）。
    latest_odom: Option<OdomSnapshot>,
    /// 收到最新 odom 时的仿真时刻（秒）。
    last_odom_recv: f64,
    /// 最新 odom 携带的 trace 上下文 `(trace_id, span_id, sampled)`（续接用）。
    odom_trace: Option<(u128, u64, bool)>,
    /// 参考状态发布器（闭环控制：MuJoCo 物理环境订阅后 PD 跟踪）。
    ref_pub: Option<Publisher<ReferenceMessage>>,
}

impl Demo {
    fn new(
        map_file: MapFile,
        viewer: Viewer,
        config: PlannerConfig,
        start: [f64; 3],
        goal: [f64; 3],
        frame_offset: [f64; 3],
    ) -> Result<Self> {
        let map = map_file.to_grid_map()?;
        let static_occupied = map_file
            .occupied
            .iter()
            .filter_map(|p| map.index_of(Vector3::new(p[0], p[1], p[2])))
            .collect();
        let planner = Planner::new(config, map);
        let mut astar = Astar::default();
        let path = astar.search(
            planner.map_ref(),
            Vector3::new(start[0], start[1], start[2]),
            Vector3::new(goal[0], goal[1], goal[2]),
        )?;
        // 字符串拉直：删除可直线直达的中间点（与 planner 内部 search_guide 一致）
        let global_path = firefly_search::simplify_path(planner.map_ref(), path.points());
        log::info!(
            "全局路径 {} 点，长度 {:.1}m，动态障碍 {} 个",
            global_path.len(),
            path_length(&global_path),
            map_file.motions.len()
        );
        // 订阅 VIO 输出（vio 进程未启动/IPC 不可用时降级为 None，保持独立运行）
        let odom = match OdomSubscriber::new() {
            Ok(s) => {
                log::info!("已订阅 odom 话题（VIO 状态源）");
                Some(s)
            }
            Err(e) => {
                log::warn!("odom 订阅不可用，回退轨迹模拟状态源：{e}");
                None
            }
        };
        let imu = match ImuSubscriber::new() {
            Ok(s) => {
                log::info!("已订阅 imu 话题 {IMU_TOPIC}");
                Some(s)
            }
            Err(e) => {
                log::warn!("imu 订阅不可用：{e}");
                None
            }
        };
        // VIO 世界系 → 地图系变换（MuJoCo 闭环下 vio 已在地图系，默认 0）
        let frame_offset = Vector3::new(frame_offset[0], frame_offset[1], frame_offset[2]);
        // 参考状态发布器（闭环控制回传；失败则降级为纯观测）
        let ref_pub = match Publisher::<ReferenceMessage>::with_topic(REFERENCE_TOPIC) {
            Ok(p) => {
                log::info!("已打开参考状态话题 {REFERENCE_TOPIC}（闭环控制回传）");
                Some(p)
            }
            Err(e) => {
                log::warn!("参考状态发布不可用（开环运行）：{e}");
                None
            }
        };
        Ok(Self {
            planner,
            viewer,
            map_file,
            static_occupied,
            prev_dyn: Vec::new(),
            global_path,
            local: None,
            goal: Point3::new(goal[0], goal[1], goal[2]),
            t_sim: 0.0,
            touch_goal: false,
            finished: false,
            replans: 0,
            frame_offset,
            odom,
            imu,
            latest_odom: None,
            last_odom_recv: f64::NEG_INFINITY,
            odom_trace: None,
            ref_pub,
        })
    }

    /// 排空 odom/imu 订阅：续接 trace span、记录最新状态、写日志。
    fn poll_sensors(&mut self, now: f64) -> Result<()> {
        if let Some(sub) = &self.odom {
            while let Some(sample) = sub.receive()? {
                let ctx = *sample.user_header();
                // 跨进程 trace 续接：本 span 的父即 vio 发布端 span
                let _span = ctx.continue_span("recv-odom");
                if ctx.is_traced() {
                    self.odom_trace = Some((ctx.trace_id(), ctx.span_id, ctx.sampled()));
                }
                let m: OdomMessage = *sample;
                let p = Vector3::new(m.position_x, m.position_y, m.position_z) + self.frame_offset;
                let v = Vector3::new(m.velocity_x, m.velocity_y, m.velocity_z);
                self.latest_odom = Some(OdomSnapshot {
                    state: State {
                        position: Point3::from(p),
                        velocity: v,
                        acceleration: Vector3::zeros(),
                    },
                });
                self.last_odom_recv = now;
                log::info!(
                    "odom recv t={:.2} p=({:.2},{:.2},{:.2}) v=({:.3},{:.3},{:.3}) trace_id={:032x}",
                    m.timestamp,
                    p.x,
                    p.y,
                    p.z,
                    v.x,
                    v.y,
                    v.z,
                    ctx.trace_id()
                );
            }
        }
        if let Some(sub) = &self.imu {
            while let Some(sample) = sub.receive()? {
                let ctx = *sample.user_header();
                let _span = ctx.continue_span("recv-imu");
                let m = *sample;
                log::debug!(
                    "imu recv t={:.3} w=({:.3},{:.3},{:.3}) a=({:.2},{:.2},{:.2})",
                    m.timestamp,
                    m.angular_velocity_x,
                    m.angular_velocity_y,
                    m.angular_velocity_z,
                    m.linear_acceleration_x,
                    m.linear_acceleration_y,
                    m.linear_acceleration_z,
                );
            }
        }
        Ok(())
    }

    /// 当前无人机状态源：新鲜 odom（VIO）优先，否则轨迹推进（独立运行）。
    fn current_state(&self, now: f64) -> State {
        let fresh = self
            .latest_odom
            .as_ref()
            .filter(|_| now - self.last_odom_recv < ODOM_FRESH_TIMEOUT);
        if let Some(o) = fresh {
            return o.state;
        }
        match &self.local {
            Some(local) => {
                let t_cur = (now - local.start_time).clamp(0.0, local.traj.duration());
                let s = local.traj.eval(t_cur);
                State {
                    position: Point3::new(s.position.x, s.position.y, s.position.z),
                    velocity: s.velocity,
                    acceleration: s.acceleration,
                }
            }
            None => State {
                position: Point3::from(self.global_path[0]),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
        }
    }

    /// 当前无人机位置（与 [`Demo::current_state`] 同源）。
    fn current_position(&self, now: f64) -> Vector3<f64> {
        self.current_state(now).position.coords
    }

    /// 官方 `getLocalTarget`：从 start 沿全局路径累计弧长，取 horizon 处的点；
    /// 路径取尽仍不足 horizon 时目标为终点，`touch_goal = true`。
    fn local_target(&self, start: Vector3<f64>) -> (Vector3<f64>, bool) {
        // 定位 start 在全局路径上的最近段（官方沿 global_traj 投影）
        let mut seg = 0usize;
        let mut best = f64::INFINITY;
        for i in 0..self.global_path.len() - 1 {
            let a = self.global_path[i];
            let b = self.global_path[i + 1];
            let ab = b - a;
            let t = ((start - a).dot(&ab) / ab.norm_squared()).clamp(0.0, 1.0);
            let d = (start - (a + ab * t)).norm_squared();
            if d < best {
                best = d;
                seg = i;
            }
        }
        // 从 start 沿剩余路径累计弧长到 horizon
        let mut arc = 0.0;
        let mut prev = start;
        for point in &self.global_path[seg + 1..] {
            let segment = *point - prev;
            let len = segment.norm();
            if arc + len >= PLANNING_HORIZON {
                let t = (PLANNING_HORIZON - arc) / len;
                return (prev + segment * t, false);
            }
            arc += len;
            prev = *point;
        }
        (Vector3::new(self.goal.x, self.goal.y, self.goal.z), true)
    }

    /// 官方 `planFromLocalTraj`：从当前状态（VIO odom 或轨迹推进）重规划到局部目标。
    fn replan(&mut self, now: f64) -> Result<()> {
        let _ = self.local.as_ref().expect("replan requires local traj");
        // 状态源：新鲜 odom（VIO）优先，否则回退轨迹推进（独立运行）
        let start = self.current_state(now);
        let pos = start.position.coords;
        let (target, touch_goal) = self.local_target(pos);
        log::info!(
            "replan #{:03} t={now:.1}s 从 ({:.1},{:.1},{:.1}) 到 ({:.1},{:.1},{:.1}){}",
            self.replans,
            pos.x,
            pos.y,
            pos.z,
            target.x,
            target.y,
            target.z,
            if touch_goal { " [touch_goal]" } else { "" }
        );
        self.replans += 1;

        let result: PlanResult = match self
            .planner
            .plan(start, Point3::new(target.x, target.y, target.z))
        {
            Ok(r) => r,
            Err(e) => {
                log::warn!("重规划失败，保持旧轨迹：{e}");
                return Ok(());
            }
        };
        self.viewer.log_trajectory(
            "local_traj",
            &result.trajectory,
            (80, 160, 255),
            (255, 200, 80),
        )?;
        self.viewer.log_planes("planes", &result.planes)?;
        log::info!(
            "轨迹最小间隙 {:.3}m",
            min_clearance(&result.trajectory, self.planner.map_ref())
        );
        self.local = Some(LocalTraj {
            traj: result.trajectory,
            start_time: now,
        });
        if touch_goal {
            self.touch_goal = true;
        }
        Ok(())
    }

    /// 首次规划（官方 `GEN_NEW_TRAJ`：从起点到目标全段）。
    fn initial_plan(&mut self) -> Result<()> {
        let start = State {
            position: Point3::new(
                self.global_path[0].x,
                self.global_path[0].y,
                self.global_path[0].z,
            ),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        // 官方 FSM：初始规划失败不退出，保持状态下一帧重试
        let result = match self.planner.plan(start, self.goal) {
            Ok(r) => r,
            Err(e) => {
                log::warn!("初始规划失败：{e}，下一帧重试");
                return Ok(());
            }
        };
        self.viewer
            .log_path("global_path", &self.global_path, (80, 200, 120))?;
        self.viewer.log_trajectory(
            "local_traj",
            &result.trajectory,
            (80, 160, 255),
            (255, 200, 80),
        )?;
        self.viewer.log_planes("planes", &result.planes)?;
        self.local = Some(LocalTraj {
            traj: result.trajectory,
            start_time: self.t_sim,
        });
        Ok(())
    }

    /// 动态障碍按仿真时钟插值，增量更新占据地图。
    fn update_motion(&mut self) {
        if self.map_file.motions.is_empty() {
            return;
        }
        let dyn_voxels = self
            .map_file
            .motion_voxels(self.t_sim, self.planner.map_ref());
        let map = self.planner.map_mut();
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

    /// 官方 `execFSMCallback`：10Hz 主循环单步。
    fn tick(&mut self) -> Result<()> {
        let now = self.t_sim;
        // 先消费 VIO 输出（续接 trace span、更新最新状态）
        self.poll_sensors(now)?;
        self.update_motion();
        match &self.local {
            None => {
                self.initial_plan()?;
            }
            Some(local) => {
                let t_cur = now - local.start_time;
                if t_cur > local.traj.duration() {
                    // 轨迹执行完毕：目标是终点则任务完成，否则（异常）重规划
                    if self.touch_goal {
                        self.finished = true;
                        return Ok(());
                    }
                    log::warn!("轨迹执行完毕但未到达目标，强制重规划");
                    self.replan(now)?;
                } else if t_cur > REPLAN_THRESH {
                    self.replan(now)?;
                }
            }
        }
        // 当前无人机位置（VIO odom 优先，否则轨迹推进）
        let pos = self.current_position(now);
        // 发布参考状态（闭环控制：MuJoCo 物理环境订阅后 PD 跟踪执行中的轨迹）
        if let Some(local) = &self.local {
            let t_cur = (now - local.start_time).clamp(0.0, local.traj.duration());
            let s = local.traj.eval(t_cur);
            if let Some(pub_) = &self.ref_pub
                && let Err(e) = pub_.publish(ReferenceMessage {
                    timestamp: now,
                    position_x: s.position.x,
                    position_y: s.position.y,
                    position_z: s.position.z,
                    velocity_x: s.velocity.x,
                    velocity_y: s.velocity.y,
                    velocity_z: s.velocity.z,
                })
            {
                log::warn!("参考状态发布失败: {e}");
            }
        }
        // 到达判定：无人机与目标距离 < ARRIVE_DIST（odom 状态源下任务也能正常结束）
        if self.local.is_some() && (pos - self.goal.coords).norm() < ARRIVE_DIST {
            log::info!(
                "到达目标 ({:.1},{:.1},{:.1})，任务完成（重规划 {} 次）",
                pos.x,
                pos.y,
                pos.z,
                self.replans
            );
            self.finished = true;
            return Ok(());
        }
        self.viewer.log_position("drone", [pos.x, pos.y, pos.z], (255, 140, 40))?;
        // 动态障碍按真实尺寸渲染（motions 实体）
        if !self.map_file.motions.is_empty() {
            let mut indices = Vec::new();
            for m in &self.map_file.motions {
                let p = m.position_at(now);
                indices.extend(human_voxels(p[0], p[1]));
            }
            self.viewer.log_voxel_grid(
                "motions",
                &indices,
                [0.1, 0.1, 0.1],
                [0.0, 0.0, 0.0],
                (220, 60, 60),
            )?;
        }
        Ok(())
    }

    fn run(&mut self) -> Result<()> {
        log::info!("主循环启动：10Hz，重规划阈值 {REPLAN_THRESH}s");
        let mut frame = 0usize;
        while !self.finished {
            // 每帧 trace 上下文：续接最新 odom 的 trace（跨进程同周期一条 trace），
            // 无新鲜 odom 时自建新 root
            let fresh_odom = self
                .odom_trace
                .filter(|_| self.t_sim - self.last_odom_recv < ODOM_FRESH_TIMEOUT);
            let root = match fresh_odom {
                Some((tid, sid, sampled)) => Span::root(
                    "firefly-demo",
                    SpanContext::new(TraceId(tid), SpanId(sid)).sampled(sampled),
                ),
                None => Span::root("firefly-demo", SpanContext::random()),
            };
            let _guard = root.set_local_parent();
            let t0 = Instant::now();
            self.tick()?;
            if frame.is_multiple_of(10) {
                log::info!(
                    "t={:.1}s 位置 ({:.1},{:.1},{:.1})",
                    self.t_sim,
                    pos_of(self).x,
                    pos_of(self).y,
                    pos_of(self).z
                );
            }
            frame += 1;
            self.t_sim += LOOP_PERIOD.as_secs_f64();
            let elapsed = t0.elapsed();
            if elapsed < LOOP_PERIOD {
                std::thread::sleep(LOOP_PERIOD.saturating_sub(elapsed));
            }
        }
        log::info!(
            "任务完成：{} 次重规划，总耗时 {:.1}s",
            self.replans,
            self.t_sim
        );
        Ok(())
    }
}

fn pos_of(demo: &Demo) -> Vector3<f64> {
    demo.current_position(demo.t_sim)
}

/// 人形体素（0.1m 格）：双腿 + 躯干 + 头，脚底 z=0，中心对齐 (cx, cy)。
fn human_voxels(cx: f64, cy: f64) -> Vec<(i32, i32, i32)> {
    let ox = (cx / 0.1).round() as i32 - 1;
    let oy = (cy / 0.1).round() as i32 - 1;
    let mut out = Vec::with_capacity(40);
    // 双腿：1×1×8
    for z in 0..=7 {
        out.push((ox - 1, oy, z));
        out.push((ox, oy, z));
    }
    // 躯干：3×2×4
    for x in -1..=1 {
        for y in 0..=1 {
            for z in 8..=11 {
                out.push((ox + x, oy + y, z));
            }
        }
    }
    // 头：2×2×2
    for x in 0..=1 {
        for y in 0..=1 {
            for z in 14..=15 {
                out.push((ox + x, oy + y, z));
            }
        }
    }
    out
}

/// 轨迹中心到最近占据体素表面的最小距离（邻域 3 格扫描，减半格为表面距）。
fn min_clearance(traj: &Trajectory, map: &GridMap) -> f64 {
    const SAMPLES: usize = 40;
    let mut min = f64::INFINITY;
    let res = map.resolution();
    for (piece, piece_dur) in traj.durations().iter().enumerate() {
        for k in 0..SAMPLES {
            let tau = k as f64 / SAMPLES as f64;
            let mut time = 0.0;
            for dur in traj.durations().iter().take(piece) {
                time += dur;
            }
            time += tau * piece_dur;
            let point = traj.eval(time).position;
            let Some(idx) = map.index_of(point) else {
                continue;
            };
            let dims = map.dims();
            for dx in -3i32..=3 {
                for dy in -3i32..=3 {
                    for dz in -3i32..=3 {
                        let nb = [
                            i32::try_from(idx[0]).unwrap() + dx,
                            i32::try_from(idx[1]).unwrap() + dy,
                            i32::try_from(idx[2]).unwrap() + dz,
                        ];
                        if nb.iter().any(|&v| v < 0)
                            || nb[0] >= i32::try_from(dims[0]).unwrap()
                            || nb[1] >= i32::try_from(dims[1]).unwrap()
                            || nb[2] >= i32::try_from(dims[2]).unwrap()
                        {
                            continue;
                        }
                        if map.state([nb[0] as usize, nb[1] as usize, nb[2] as usize])
                            != VoxelState::Occupied
                        {
                            continue;
                        }
                        let center = Vector3::new(
                            map.origin().x + (f64::from(nb[0]) + 0.5) * res,
                            map.origin().y + (f64::from(nb[1]) + 0.5) * res,
                            map.origin().z + (f64::from(nb[2]) + 0.5) * res,
                        );
                        let dist = (point - center).norm() - 0.5 * res;
                        if dist < min {
                            min = dist;
                        }
                    }
                }
            }
        }
    }
    min
}

fn path_length(points: &[Vector3<f64>]) -> f64 {
    points.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
}

fn main() {
    init_observability();
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!(
                "{e}\n用法：firefly-demo --map <map.ffmap> [--save out.rrd] [--start x y z] [--goal x y z]"
            );
            std::process::exit(2);
        }
    };
    let map_file = match MapFile::from_file(&args.map) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("加载地图失败：{e}");
            std::process::exit(1);
        }
    };
    let viewer = match &args.save {
        Some(path) => Viewer::save("firefly-demo", path),
        None => Viewer::spawn("firefly-demo"),
    };
    let viewer = match viewer {
        Ok(v) => v,
        Err(e) => {
            eprintln!("启动 viewer 失败：{e}");
            std::process::exit(1);
        }
    };
    let grid = match map_file.to_grid_map() {
        Ok(g) => g,
        Err(e) => {
            eprintln!("地图体素化失败：{e}");
            std::process::exit(1);
        }
    };
    viewer.log_map("map", &grid).expect("log map");
    // 装饰层（草丛）：不参与规划，绿色体素
    if !map_file.decor.is_empty() {
        let indices: Vec<(i32, i32, i32)> = map_file
            .decor
            .iter()
            .filter_map(|p| grid.index_of(Vector3::new(p[0], p[1], p[2])))
            .map(|idx| (idx[0] as i32, idx[1] as i32, idx[2] as i32))
            .collect();
        viewer
            .log_voxel_grid(
                "decor",
                &indices,
                [0.1, 0.1, 0.1],
                [0.0, 0.0, 0.0],
                (90, 200, 90),
            )
            .expect("log decor");
    }
    match Demo::new(
        map_file,
        viewer,
        PlannerConfig::default(),
        args.start,
        args.goal,
        args.frame_offset,
    ) {
        Ok(mut demo) => {
            if let Err(e) = demo.run() {
                log::error!("demo 失败：{e}");
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
