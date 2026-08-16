//! 10Hz 重规划主循环演示（仿官方 `ego_replan_fsm`）。
//!
//! 运行：`cargo run -p firefly-demo -- --map apps/firefly-demo/maps/gate.ffmap`
//!
//! 地图为 `FFMap` 标准格式（见 `docs/map-format.md`），可含动态障碍：
//! 主循环每 tick 按航点插值更新障碍占据后重规划。
//!
//! 主循环语义（对应官方 `execFSMCallback`）：
//! - `EXEC_TRAJ`：轨迹按时间推进，`t_cur > replan_thresh` 触发重规划；
//! - `REPLAN_TRAJ`：起点取当前轨迹状态，目标取全局路径上
//!   `planning_horizon` 处的点（官方 `getLocalTarget`），规划失败保持旧轨迹；
//! - 到达目标（`touch_goal`）后任务完成，退出循环。

use std::collections::HashSet;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use fastrace::prelude::*;
use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{MapFile, VoxelState};
use firefly_observability::init as init_observability;
use firefly_planner::{PlanResult, Planner, PlannerConfig, State};
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

struct Args {
    map: PathBuf,
    save: Option<PathBuf>,
    start: [f64; 3],
    goal: [f64; 3],
}

fn parse_args() -> Result<Args> {
    let mut map = None;
    let mut save = None;
    let mut start = [1.0, 4.0, 1.0];
    let mut goal = [27.0, 4.0, 1.0];
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--map" => map = Some(PathBuf::from(it.next().unwrap_or_default())),
            "--save" => save = Some(PathBuf::from(it.next().unwrap_or_default())),
            "--start" => start = parse_vec3(&mut it, "start")?,
            "--goal" => goal = parse_vec3(&mut it, "goal")?,
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
    /// 是否完成（官方 `touch_goal` 后到达）。
    finished: bool,
    replans: usize,
}

impl Demo {
    fn new(
        map_file: MapFile,
        viewer: Viewer,
        config: PlannerConfig,
        start: [f64; 3],
        goal: [f64; 3],
    ) -> Result<Self> {
        let map = map_file.to_grid_map()?;
        let static_occupied = map_file
            .occupied
            .iter()
            .filter_map(|p| map.index_of(Vector3::new(p[0], p[1], p[2])))
            .collect();
        let planner = Planner::new(config, map);
        let mut astar = Astar::default();
        let global_path = astar
            .search(
                planner.map_ref(),
                Vector3::new(start[0], start[1], start[2]),
                Vector3::new(goal[0], goal[1], goal[2]),
            )?
            .points()
            .to_vec();
        log::info!(
            "全局路径 {} 点，长度 {:.1}m，动态障碍 {} 个",
            global_path.len(),
            path_length(&global_path),
            map_file.motions.len()
        );
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
            finished: false,
            replans: 0,
        })
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

    /// 官方 `planFromLocalTraj`：从当前轨迹状态重规划到局部目标。
    fn replan(&mut self, now: f64) -> Result<()> {
        let local = self.local.as_ref().expect("replan requires local traj");
        let t_cur = (now - local.start_time).clamp(0.0, local.traj.duration());
        let s = local.traj.eval(t_cur);
        let start = State {
            position: Point3::new(s.position.x, s.position.y, s.position.z),
            velocity: s.velocity,
            acceleration: s.acceleration,
        };
        let (target, touch_goal) = self.local_target(s.position);
        log::info!(
            "replan #{:03} t={now:.1}s 从 ({:.1},{:.1},{:.1}) 到 ({:.1},{:.1},{:.1}){}",
            self.replans,
            s.position.x,
            s.position.y,
            s.position.z,
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
        self.local = Some(LocalTraj {
            traj: result.trajectory,
            start_time: now,
        });
        if touch_goal {
            self.finished = true;
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
        let result = self.planner.plan(start, self.goal)?;
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
        self.update_motion();
        match &self.local {
            None => {
                self.initial_plan()?;
            }
            Some(local) => {
                let t_cur = now - local.start_time;
                if t_cur > local.traj.duration() {
                    // 轨迹执行完毕：touch_goal 则任务完成，否则（异常）重规划
                    if self.finished {
                        return Ok(());
                    }
                    log::warn!("轨迹执行完毕但未到达目标，强制重规划");
                    self.replan(now)?;
                } else if t_cur > REPLAN_THRESH {
                    self.replan(now)?;
                }
            }
        }
        // 当前位置（按轨迹推进，模拟里程计）
        if let Some(local) = &self.local {
            let t_cur = (now - local.start_time).clamp(0.0, local.traj.duration());
            let s = local.traj.eval(t_cur);
            let pos = [s.position.x, s.position.y, s.position.z];
            self.viewer.log_position("drone", pos, (255, 140, 40))?;
        }
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
            // 每帧建立 trace 上下文（fastrace：`#[trace]` 仅在 root span 下收集）
            let root = Span::root("firefly-demo", SpanContext::random());
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
    match &demo.local {
        Some(local) => {
            let t_cur = (demo.t_sim - local.start_time).clamp(0.0, local.traj.duration());
            local.traj.eval(t_cur).position
        }
        None => demo.global_path[0],
    }
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
    match Demo::new(
        map_file,
        viewer,
        PlannerConfig::default(),
        args.start,
        args.goal,
    ) {
        Ok(mut demo) => {
            if let Err(e) = demo.run() {
                log::error!("demo 失败：{e}");
                std::process::exit(1);
            }
        }
        Err(e) => {
            eprintln!("初始化失败：{e}");
            std::process::exit(1);
        }
    }
    firefly_observability::flush();
}
