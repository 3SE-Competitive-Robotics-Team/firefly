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
use firefly_map::{update_from_depth, DepthCamera, GridMap, MapFile, VoxelState};
use firefly_observability::init as init_observability;
use firefly_planner::{PlanResult, Planner, PlannerConfig, State};
use firefly_pubsub::camera::{DEPTH_TOPIC, DepthImageMessage};
use firefly_pubsub::imu::{IMU_TOPIC, ImuSubscriber};
use firefly_pubsub::odom::{GROUND_TRUTH_TOPIC, OdomMessage};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::reference::{REFERENCE_TOPIC, ReferenceMessage};
use firefly_pubsub::subscriber::{OdomSubscriber, Subscriber};
use firefly_search::Astar;
use firefly_trajectory::Trajectory;
use firefly_viewer::Viewer;
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};

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
    /// 静态地图文件（可选：MuJoCo 闭环下省略，深度建图填充）。
    map: Option<PathBuf>,
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
    let map = map; // Option<PathBuf>；MuJoCo 闭环下可省略
    Ok(Args {
        map,
        save,
        start,
        goal,
        frame_offset,
    })
}

/// `MuJoCo` 闭环模式的空地图（无静态先验，由深度感知填充）。
/// 范围覆盖 `firefly-mujoco` 场景：x∈[0,32]、y∈[-5,9]、z∈[0,5.2]。
#[must_use]
fn empty_map_file() -> MapFile {
    MapFile {
        resolution: 0.4,
        origin: [0.0, -5.0, 0.0],
        dims: [80, 35, 13],
        occupied: Vec::new(),
        decor: Vec::new(),
        motions: Vec::new(),
    }
}

/// `MuJoCo` 默认场景静态地图：与 `firefly_mujoco/scene.py` 的障碍布局
/// 同构（box 中心 + 半尺寸），体素化后作先验，保证**全局路径**在空地图上
/// 也会绕柱蛇形（纯深度感知在航线上才看到障碍，全局路径会是直线）。
///
/// 布局：中线上一串孤立高柱（约 0.8~1.2m 见方），逼小幅左右绕行；
/// x=12/19 为走廊外小障（装饰）。
#[must_use]
fn mujoco_map_file() -> MapFile {
    let mut map = empty_map_file();
    let boxes: [[f64; 6]; 5] = [
        [9.0, 4.0, 1.5, 0.4, 0.5, 1.5],
        [12.0, 6.5, 1.0, 0.4, 0.7, 1.0],
        [16.0, 4.0, 1.5, 0.4, 0.6, 1.5],
        [19.0, 1.8, 0.9, 0.4, 0.5, 0.9],
        [22.0, 3.6, 1.5, 0.4, 0.5, 1.5],
    ];
    let res = map.resolution;
    let o = map.origin;
    for [cx, cy, cz, hx, hy, hz] in boxes {
        for x in 0..map.dims[0] {
            for y in 0..map.dims[1] {
                for z in 0..map.dims[2] {
                    let p = [
                        o[0] + (x as f64 + 0.5) * res,
                        o[1] + (y as f64 + 0.5) * res,
                        o[2] + (z as f64 + 0.5) * res,
                    ];
                    if (p[0] - cx).abs() <= hx && (p[1] - cy).abs() <= hy && (p[2] - cz).abs() <= hz
                    {
                        map.occupied.push(p);
                    }
                }
            }
        }
    }
    map
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
    /// 仿真时钟（秒）。**唯一权威 = 收到的传感器时间戳**（GT/odom 均带
    /// `MuJoCo` sim 时钟），在本仿真内代替官方 `ros::Time::now()`；无传感器
    /// 才按 tick 本地递增回退。所有计算/viewer 时间轴都用它，与 vio/仿真对齐。
    t_sim: f64,
    /// 本 tick 是否收到带 sim 时间戳的消息（决定 `t_sim` 是否本地回退递增）。
    sensor_this_tick: bool,
    /// 目标是终点（`touch_goal`）：轨迹执行完毕且**物理到达**目标才任务完成。
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
    /// 重规划失败后的冷却截止时刻（秒）：失败不每 tick 重试，避免
    /// A* 空转（失败不改 `start_time`，`t_cur` 持续 > 阈值会触发逐 tick 重试）。
    replan_cooldown_until: f64,
    /// 连续重规划失败计数：≥3 且轨迹耗尽时沿全局路径直飞回退（脱困）。
    replan_fail_streak: usize,
    /// 参考状态发布器（闭环控制：MuJoCo 物理环境订阅后 PD 跟踪）。
    ref_pub: Option<Publisher<ReferenceMessage>>,
    /// 深度订阅（MuJoCo 物理环境发布；感知建图输入）。
    depth: Option<Subscriber<DepthImageMessage>>,
    /// 真值订阅（仿真阶段感知位姿源；VIO 修复后换 odom）。
    gt: Option<Subscriber<OdomMessage>>,
    /// 最新深度帧。
    latest_depth: Option<DepthImageMessage>,
    /// 最新真值（地图系）`(state, quat_xyzw)`；仿真阶段的状态源与感知位姿源。
    latest_gt: Option<(State, [f64; 4])>,
    /// 深度相机标定（MuJoCo 合成场景）。
    depth_cam: DepthCamera,
}

impl Demo {
    // 长构造器（含订阅/标定/诊断），clippy too_many_lines 允许
    #[allow(clippy::too_many_lines)]
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
        // 诊断：全局路径中段（绕柱检测，compact 输出）
        let path_str: String = global_path
            .iter()
            .map(|p| format!("{:.1},{:.1}", p.x, p.y))
            .collect::<Vec<_>>()
            .join(" ");
        log::info!("全局路径点: {path_str}");
        // 诊断：x=9 柱处地图占用
        for y in [3.2f64, 4.0, 4.8] {
            let o = planner.map_ref().is_occupied_inflated(Vector3::new(9.0, y, 1.0));
            let raw = planner.map_ref().is_occupied(Vector3::new(9.0, y, 1.0));
            log::info!("map@(9,{y},1) inflated={o} raw={raw}");
        }
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
        // 深度订阅（感知建图输入）
        let depth = match Subscriber::<DepthImageMessage>::with_topic(DEPTH_TOPIC) {
            Ok(s) => {
                log::info!("已订阅深度话题 {DEPTH_TOPIC}（感知建图）");
                Some(s)
            }
            Err(e) => {
                log::warn!("深度订阅不可用（无感知建图）：{e}");
                None
            }
        };
        // 真值订阅（仿真阶段感知位姿源；VIO 修复后换 odom）
        let gt = match Subscriber::<OdomMessage>::with_topic(GROUND_TRUTH_TOPIC) {
            Ok(s) => {
                log::info!("已订阅真值话题 {GROUND_TRUTH_TOPIC}（感知位姿源）");
                Some(s)
            }
            Err(e) => {
                log::warn!("真值订阅不可用（无感知建图）：{e}");
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
            sensor_this_tick: false,
            touch_goal: false,
            finished: false,
            replans: 0,
            frame_offset,
            odom,
            imu,
            latest_odom: None,
            last_odom_recv: f64::NEG_INFINITY,
            odom_trace: None,
            replan_cooldown_until: f64::NEG_INFINITY,
            replan_fail_streak: 0,
            ref_pub,
            depth,
            gt,
            latest_depth: None,
            latest_gt: None,
            depth_cam: DepthCamera::mujoco_default(),
        })
    }

    /// 排空 odom/imu/深度/真值订阅：续接 trace span、记录最新状态。
    ///
    /// 收到带 sim 时间戳的消息（odom/GT）时把权威时钟 `t_sim` 单调锚定到该
    /// 时间戳（MuJoCo sim 时钟），使 demo 与 vio/仿真同一时间轴、viewer
    /// 回放对齐；并置 `sensor_this_tick`。
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
                let v = Vector3::new(m.velocity_x, m.velocity_y, m.velocity_z);
                self.latest_odom = Some(OdomSnapshot {
                    state: State {
                        position: Point3::from(p),
                        velocity: v,
                        acceleration: Vector3::zeros(),
                    },
                });
                self.last_odom_recv = m.timestamp;
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
        // 深度 + 真值（感知建图输入；只取最新帧）
        if let Some(sub) = &self.depth {
            while let Some(sample) = sub.receive()? {
                let ctx = *sample.user_header();
                let _span = ctx.continue_span("recv-depth");
                self.latest_depth = Some(*sample);
                log::debug!("depth recv t={:.3}", sample.timestamp);
            }
        }
        if let Some(sub) = &self.gt {
            while let Some(sample) = sub.receive()? {
                let ctx = *sample.user_header();
                let _span = ctx.continue_span("recv-gt");
                let m = *sample;
                // 锚定 sim 时钟到真值时间戳（MuJoCo sim 时钟，与 vio 对齐）
                self.t_sim = self.t_sim.max(m.timestamp);
                self.sensor_this_tick = true;
                let p = Vector3::new(m.position_x, m.position_y, m.position_z) + self.frame_offset;
                let v = Vector3::new(m.velocity_x, m.velocity_y, m.velocity_z);
                self.latest_gt = Some((
                    State {
                        position: Point3::from(p),
                        velocity: v,
                        acceleration: Vector3::zeros(),
                    },
                    [m.quat_x, m.quat_y, m.quat_z, m.quat_w],
                ));
                log::debug!(
                    "gt recv t={:.3} p=({:.2},{:.2},{:.2}) q=({:.2},{:.2},{:.2},{:.2})",
                    m.timestamp,
                    p.x,
                    p.y,
                    p.z,
                    m.quat_x,
                    m.quat_y,
                    m.quat_z,
                    m.quat_w
                );
            }
        }
        Ok(())
    }

    /// 深度 → 占据体素（感知建图）：用最新真值位姿把深度帧射线写入 planner 地图。
    ///
    /// 仿真阶段用真值位姿（`vio` 估计发散未修复）；VIO 修复后改用 odom。
    fn update_map_from_depth(&mut self) {
        let (Some(depth), Some((state, quat))) = (&self.latest_depth, self.latest_gt) else {
            return;
        };
        let pos = state.position.coords;
        let q = UnitQuaternion::from_quaternion(Quaternion::new(
            quat[3], quat[0], quat[1], quat[2],
        ));
        let pose = Isometry3::from_parts(Translation3::new(pos.x, pos.y, pos.z), q);
        update_from_depth(self.planner.map_mut(), &self.depth_cam, &pose, &depth.data);
    }

    /// 当前无人机状态源：真值（仿真阶段）→ 新鲜 odom（VIO）→ 轨迹推进。
    fn current_state(&self, now: f64) -> State {
        // 仿真阶段：真值优先（vio 估计发散未修复，修复后切回 odom）
        if let Some((state, _)) = &self.latest_gt {
            return *state;
        }
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

    /// 官方 `getLocalTarget`：从 start 沿全局路径累计弧长，取 `arc` 处的点；
    /// 路径取尽仍不足 `arc` 时目标为终点，`touch_goal = true`。
    fn path_point_at_arc(&self, start: Vector3<f64>, arc: f64) -> (Vector3<f64>, bool) {
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
        // 从 start 沿剩余路径累计弧长到 arc
        let mut acc = 0.0;
        let mut prev = start;
        for point in &self.global_path[seg + 1..] {
            let segment = *point - prev;
            let len = segment.norm();
            if acc + len >= arc {
                let t = (arc - acc) / len;
                return (prev + segment * t, false);
            }
            acc += len;
            prev = *point;
        }
        (Vector3::new(self.goal.x, self.goal.y, self.goal.z), true)
    }

    /// 局部目标：6m horizon 点，落进障碍膨胀区或贴近障碍时沿路径逐步回退
    /// 到安全点。
    ///
    /// 全局路径是 A* 网格路径，其直线段可能切过膨胀体素，按弧长插值的
    /// horizon 点会落在墙内；A* 不接受 occupied goal（会持续重规划失败、
    /// 参考卡死），故回退找安全点（每步 0.4m，[`Self::target_clear`]）。
    fn local_target(&self, start: Vector3<f64>) -> (Vector3<f64>, bool) {
        let mut arc = PLANNING_HORIZON;
        loop {
            let (point, touch) = self.path_point_at_arc(start, arc);
            if touch || arc <= 0.0 {
                return (point, touch);
            }
            if self.target_clear(point) {
                return (point, false);
            }
            arc -= 0.4;
        }
    }

    /// 目标点安全判据：不在膨胀占据区，且 26 邻域（1 格 0.4m）无占据体素
    /// ——给 MINCO 留足绕弯余量，避免轨迹切墙角导致优化"stuck"。
    fn target_clear(&self, point: Vector3<f64>) -> bool {
        let map = self.planner.map_ref();
        if map.is_occupied_inflated(point) {
            return false;
        }
        let Some(idx) = map.index_of(point) else {
            return true;
        };
        let dims = map.dims();
        for dx in -1i32..=1 {
            for dy in -1i32..=1 {
                for dz in -1i32..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let nb = [
                        i32::try_from(idx[0]).unwrap_or(i32::MAX) + dx,
                        i32::try_from(idx[1]).unwrap_or(i32::MAX) + dy,
                        i32::try_from(idx[2]).unwrap_or(i32::MAX) + dz,
                    ];
                    if nb.iter().any(|&v| v < 0)
                        || nb[0] >= i32::try_from(dims[0]).unwrap_or(i32::MIN)
                        || nb[1] >= i32::try_from(dims[1]).unwrap_or(i32::MIN)
                        || nb[2] >= i32::try_from(dims[2]).unwrap_or(i32::MIN)
                    {
                        continue;
                    }
                    if map.state([nb[0] as usize, nb[1] as usize, nb[2] as usize])
                        == VoxelState::Occupied
                    {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// 官方 `planFromLocalTraj`：从当前状态（VIO odom 或轨迹推进）重规划到局部目标。
    fn replan(&mut self, now: f64) -> Result<()> {
        let _ = self.local.as_ref().expect("replan requires local traj");
        // 起点 = 上一轨迹在重规划时刻的**参考状态**（位置/速度/加速度取自
        // `traj.eval`），保证重规划前后参考在时间上连续——若改用无人机实际
        // 状态（GT 滞后 ~0.2m + PD 欠阻尼 overshoot），每次重规划都会把参考
        // 拉回无人机的滞后位置，形成"进-退"振荡（实测参考周期回退 0.2~0.5m）。
        // 无上一轨迹时（异常）回退当前状态。
        let start = match &self.local {
            Some(local) => {
                let t_cur = (now - local.start_time).clamp(0.0, local.traj.duration());
                let s = local.traj.eval(t_cur);
                State {
                    position: Point3::new(s.position.x, s.position.y, s.position.z),
                    velocity: s.velocity,
                    acceleration: s.acceleration,
                }
            }
            None => self.current_state(now),
        };
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
            Ok(r) => { self.replan_fail_streak = 0; r }
            Err(e) => {
                log::warn!("重规划失败，保持旧轨迹：{e}");
                self.replan_cooldown_until = now + 0.5;
                self.replan_fail_streak += 1;
                return Ok(());
            }
        };
        // 退化轨迹防护：时长短到下一 tick 就"过期"（切角/无信息时的异常解）
        // 会触发逐 tick 强制重规划空转，直接丢弃。
        if result.trajectory.duration() < 0.5 {
            log::warn!("重规划产出退化轨迹（时长 {:.2}s），保持旧轨迹", result.trajectory.duration());
            self.replan_cooldown_until = now + 0.5;
            return Ok(());
        }
        self.viewer.log_trajectory(
            "local_traj",
            &result.trajectory,
            (80, 160, 255),
            (255, 200, 80),
        )?;
        self.viewer.log_planes("planes", &result.planes)?;
        log::info!(
            "轨迹最小间隙 {:.3}m 时长 {:.2}s 起({:.2},{:.2}) 目标({:.2},{:.2})",
            min_clearance(&result.trajectory, self.planner.map_ref()),
            result.trajectory.duration(),
            start.position.x,
            start.position.y,
            target.x,
            target.y
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
        // 先消费传感器（更新权威仿真时钟 `t_sim`），再取当帧 sim 时刻；
        // 保证所有计算/viewer 时间轴落在真实仿真时间上。
        self.poll_sensors()?;
        let now = self.t_sim;
        self.sensor_this_tick = false;
        // 统一仿真时间轴（与 vio 的传感器/odom 同轴，跨进程对齐回放）
        self.viewer.set_time(now);
        // 深度 → 占据体素（感知建图）
        self.update_map_from_depth();
        self.update_motion();
        match &self.local {
            None => {
                self.initial_plan()?;
            }
            Some(local) => {
                let t_cur = now - local.start_time;
                if t_cur > local.traj.duration() {
                    // 轨迹执行完毕，目标为终点：仅当无人机**物理到达**目标才完成
                    //（避免靠近目标但轨迹耗尽即提前结束）；未到达则继续逼近。
                    let pos = self.current_position(now);
                    if self.touch_goal && (pos - self.goal.coords).norm() < ARRIVE_DIST {
                        self.finished = true;
                        return Ok(());
                    }
                    log::warn!("轨迹执行完毕但未到达目标，强制重规划");
                    if now >= self.replan_cooldown_until {
                        self.replan(now)?;
                    }
                } else if t_cur > REPLAN_THRESH && now >= self.replan_cooldown_until {
                    self.replan(now)?;
                }
            }
        }
        // 当前无人机位置（VIO odom 优先，否则轨迹推进）
        let pos = self.current_position(now);
        // 发布参考状态（闭环控制：MuJoCo 物理环境订阅后 PD 跟踪执行中的轨迹）
        if let Some(local) = &self.local {
            let t_cur = (now - local.start_time).clamp(0.0, local.traj.duration());
            // 脱困回退：轨迹已耗尽且重规划连续失败（贴墙死锁）→ 沿全局路径
            // 向下一自由点直飞（引导速度 1 m/s），物理移动解开几何死锁，
            // 也给 VIO 提供视差（对照 EGO-Planner-v2 FSM 的失败后换路径恢复）。
            let (px, py, pz, vx, vy, vz) = if t_cur >= local.traj.duration() - 1e-6
                && self.replan_fail_streak >= 3
            {
                // 短步进沿全局路径跟随绕行（1m 步，避免直线穿障）
                let (tp, _) = self.path_point_at_arc(pos, 1.0);
                let dir = tp - pos;
                let dir = if dir.norm_squared() < 1e-9 { Vector3::zeros() } else { dir.normalize() };
                log::info!(
                    "脱困回退: 当前位置({:.2},{:.2}) 目标({:.2},{:.2})",
                    pos.x, pos.y, tp.x, tp.y
                );
                (tp.x, tp.y, tp.z, 1.0 * dir.x, 1.0 * dir.y, 1.0 * dir.z)
            } else {
                let s = local.traj.eval(t_cur);
                (
                    s.position.x,
                    s.position.y,
                    s.position.z,
                    s.velocity.x,
                    s.velocity.y,
                    s.velocity.z,
                )
            };
            if let Some(pub_) = &self.ref_pub
                && let Err(e) = pub_.publish(ReferenceMessage {
                    timestamp: now,
                    position_x: px,
                    position_y: py,
                    position_z: pz,
                    velocity_x: vx,
                    velocity_y: vy,
                    velocity_z: vz,
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
            // 每 2.5s 更新 viewer 中的感知占据体素（深度建图可视化）
            if frame.is_multiple_of(25) {
                let map = self.planner.map_ref();
                let mut occupied = 0usize;
                let (mut lo, mut hi) = (
                    Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
                    Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
                );
                let res = map.resolution();
                let o = map.origin();
                for x in 0..map.dims()[0] {
                    for y in 0..map.dims()[1] {
                        for z in 0..map.dims()[2] {
                            if map.state([x, y, z]) == VoxelState::Occupied {
                                occupied += 1;
                                let p = o + Vector3::new(
                                    (x as f64 + 0.5) * res,
                                    (y as f64 + 0.5) * res,
                                    (z as f64 + 0.5) * res,
                                );
                                lo = lo.inf(&p);
                                hi = hi.sup(&p);
                            }
                        }
                    }
                }
                if occupied > 0 {
                    log::info!(
                        "感知地图：{occupied} 个占据体素，包围盒 [{:.1},{:.1},{:.1}]~[{:.1},{:.1},{:.1}]",
                        lo.x, lo.y, lo.z, hi.x, hi.y, hi.z
                    );
                } else {
                    log::info!("感知地图：0 个占据体素");
                }
                self.viewer.log_map("perceived", map)?;
            }
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
            // 时钟推进：本 tick 收到带 sim 时间戳消息则由传感器锚定（已在
            // 轮询时更新 `t_sim`）；否则本地回退递增（独立运行无传感器时）。
            if !self.sensor_this_tick {
                self.t_sim += LOOP_PERIOD.as_secs_f64();
            }
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
                "{e}\n用法：firefly-demo [--map <map.ffmap>] [--save out.rrd] [--start x y z] [--goal x y z] [--frame-offset x y z]"
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
