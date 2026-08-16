//! 10Hz 重规划主循环演示（仿官方 `ego_replan_fsm`）。
//!
//! 运行：`cargo run -p firefly-demo -- --map apps/firefly-demo/maps/wall.json`
//!
//! 主循环语义（对应官方 `execFSMCallback`）：
//! - `EXEC_TRAJ`：轨迹按时间推进，`t_cur > replan_thresh` 触发重规划；
//! - `REPLAN_TRAJ`：起点取当前轨迹状态，目标取全局路径上
//!   `planning_horizon` 处的点（官方 `getLocalTarget`），规划失败保持旧轨迹；
//! - 到达目标（`touch_goal`）后任务完成，退出循环。

use std::path::PathBuf;
use std::time::{Duration, Instant};

use firefly_error::{Error, ErrorKind, Result};
use firefly_map::Scene;
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
}

fn parse_args() -> Result<Args> {
    let mut map = None;
    let mut save = None;
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--map" => map = Some(PathBuf::from(it.next().unwrap_or_default())),
            "--save" => save = Some(PathBuf::from(it.next().unwrap_or_default())),
            other => {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    format!("unknown argument: {other}"),
                ));
            }
        }
    }
    let map = map.ok_or_else(|| Error::new(ErrorKind::InvalidArgument, "missing --map"))?;
    Ok(Args { map, save })
}

/// 执行中的局部轨迹（官方 `LocalTrajData`：轨迹 + 起始时刻）。
struct LocalTraj {
    traj: Trajectory,
    start_time: f64,
}

struct Demo {
    planner: Planner,
    viewer: Viewer,
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
    fn new(scene: &Scene, viewer: Viewer, config: PlannerConfig) -> Result<Self> {
        let map = scene.to_grid_map()?;
        let planner = Planner::new(config, map);
        let start = Vector3::new(scene.start[0], scene.start[1], scene.start[2]);
        let astar = Astar::new(planner.map_ref());
        let global_path = astar
            .search(
                start,
                Vector3::new(scene.goal[0], scene.goal[1], scene.goal[2]),
            )?
            .points()
            .to_vec();
        log::info!(
            "全局路径 {} 点，长度 {:.1}m",
            global_path.len(),
            path_length(&global_path)
        );
        Ok(Self {
            planner,
            viewer,
            global_path,
            local: None,
            goal: Point3::new(scene.goal[0], scene.goal[1], scene.goal[2]),
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
        self.viewer
            .log_trajectory("local_traj", &result.trajectory)?;
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
        self.viewer.log_path("global_path", &self.global_path)?;
        self.viewer
            .log_trajectory("local_traj", &result.trajectory)?;
        self.viewer.log_planes("planes", &result.planes)?;
        self.local = Some(LocalTraj {
            traj: result.trajectory,
            start_time: self.t_sim,
        });
        Ok(())
    }

    /// 官方 `execFSMCallback`：10Hz 主循环单步。
    fn tick(&mut self) -> Result<()> {
        let now = self.t_sim;
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
            self.viewer.log_position("drone", pos)?;
        }
        Ok(())
    }

    fn run(&mut self) -> Result<()> {
        log::info!("主循环启动：10Hz，重规划阈值 {REPLAN_THRESH}s");
        let mut frame = 0usize;
        while !self.finished {
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
            let s = local.traj.eval(t_cur);
            s.position
        }
        None => demo.global_path[0],
    }
}

fn path_length(points: &[Vector3<f64>]) -> f64 {
    points.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
}

fn main() {
    init_observability();
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}\n用法：firefly-demo --map <scene.json> [--save out.rrd]");
            std::process::exit(2);
        }
    };
    let scene = match Scene::from_json(&args.map) {
        Ok(s) => s,
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
    viewer
        .log_map("map", &scene.to_grid_map().expect("地图体素化"))
        .expect("log map");
    match Demo::new(&scene, viewer, PlannerConfig::default()) {
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
}
