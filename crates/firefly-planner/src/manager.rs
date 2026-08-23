//! 任务执行管理器：局部轨迹簿记、重规划触发判定、局部目标选取与参考指令
//! 生成（对照 EGO-Planner-v2 `planner_manager.cpp` 的 `LocalTrajData`/
//! `computeInitState`/`getLocalTarget` 与 `ego_replan_fsm.cpp` 的触发逻辑）。
//!
//! 分层边界：不持有传输（IPC）与可视化——应用层喂入状态观测、取走参考指令；
//! 地图更新（深度感知建图/动态障碍写入）经 [`PlannerManager::map_mut`] 由
//! 应用层驱动。时钟由应用层锚定后经 `tick(now, ..)` 注入。

use firefly_error::Result;
use firefly_map::GridMap;
use firefly_search::Astar;
use firefly_trajectory::Trajectory;
use nalgebra::{Point3, Vector3};

use crate::planner::{InitSource, PlanResult, Planner, State};

/// 重规划触发阈值（秒，官方 `fsm/thresh_replan_time`）。
pub const DEFAULT_REPLAN_THRESH: f64 = 1.0;
/// 规划视界（米，官方 `manager/planning_horizon`）。
pub const DEFAULT_PLANNING_HORIZON: f64 = 6.0;
/// 到达判定距离（米）。
pub const DEFAULT_ARRIVE_DIST: f64 = 0.5;
const REPLAN_COOLDOWN: f64 = 0.5;
/// 连续重规划失败达到该次数且轨迹耗尽时，沿全局路径脱困直飞。
const FAIL_STREAK_ESCAPE: usize = 3;
/// 脱困引导速度（m/s）。
const ESCAPE_SPEED: f64 = 1.0;

/// 管理器行为参数。
#[derive(Debug, Clone, Copy)]
pub struct ManagerOptions {
    /// 重规划触发阈值（秒）。
    pub replan_thresh: f64,
    /// 规划视界（米）。
    pub planning_horizon: f64,
    /// 到达判定距离（米）。
    pub arrive_dist: f64,
}

impl Default for ManagerOptions {
    fn default() -> Self {
        Self {
            replan_thresh: DEFAULT_REPLAN_THRESH,
            planning_horizon: DEFAULT_PLANNING_HORIZON,
            arrive_dist: DEFAULT_ARRIVE_DIST,
        }
    }
}

/// 执行中的局部轨迹（官方 `LocalTrajData`：轨迹 + 起始时刻）。
#[derive(Debug)]
pub struct LocalTraj {
    pub traj: Trajectory,
    pub start_time: f64,
}

/// 参考状态指令（闭环 PD 跟踪目标）。
#[derive(Debug, Clone, Copy)]
pub struct Reference {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
}

/// 一次 [`PlannerManager::tick`] 的产出。
#[derive(Debug, Default)]
pub struct TickReport {
    /// 参考状态（`None` = 尚无可执行轨迹）。
    pub reference: Option<Reference>,
    /// 本 tick 产生了新轨迹（应用层据此记录可视化产物）。
    pub replanned: bool,
    /// 任务完成（物理到达目标）。
    pub finished: bool,
}

/// 规划视界目标（全局路径上按弧长截取的一段）。
struct Horizon {
    target: Point3<f64>,
    touch_goal: bool,
    /// 全局路径从当前位置投影到目标点的延续段（暖启动延续走向用）。
    tail: Vec<Vector3<f64>>,
}

/// 任务执行管理器：持有 [`Planner`] 与执行态，驱动"初始规划 → 周期重规划 →
/// 参考 → 到达"全流程。
pub struct PlannerManager {
    planner: Planner,
    options: ManagerOptions,
    /// 全局路径点（官方 `global_traj`，首次构造时 A* 简化缓存）。
    global_path: Vec<Vector3<f64>>,
    goal: Point3<f64>,
    local: Option<LocalTraj>,
    last_result: Option<PlanResult>,
    touch_goal: bool,
    finished: bool,
    replans: usize,
    /// 重规划失败后的冷却截止时刻：失败不逐 tick 重试空转。
    replan_cooldown_until: f64,
    /// 连续重规划失败计数：达阈值且轨迹耗尽时沿全局路径脱困。
    replan_fail_streak: usize,
}

impl PlannerManager {
    /// 实际构造入口：给定已构建的 [`Planner`]（应用层负责地图来源）。
    ///
    /// # Errors
    ///
    /// 全局路径搜索失败。
    pub fn with_planner(
        planner: Planner,
        options: ManagerOptions,
        start: Vector3<f64>,
        goal: Vector3<f64>,
    ) -> Result<Self> {
        let mut astar = Astar::default();
        let path = astar.search(planner.map_ref(), start, goal)?;
        // 字符串拉直：删除可直线直达的中间点（避免直线擦边穿越膨胀层）
        let global_path = firefly_search::simplify_path(planner.map_ref(), path.points());
        log::info!(
            "全局路径 {} 点，长度 {:.1}m",
            global_path.len(),
            path_length(&global_path)
        );
        Ok(Self {
            planner,
            options,
            global_path,
            goal: Point3::from(goal),
            local: None,
            last_result: None,
            touch_goal: false,
            finished: false,
            replans: 0,
            replan_cooldown_until: 0.0,
            replan_fail_streak: 0,
        })
    }

    /// 只读地图访问。
    #[must_use]
    pub fn map(&self) -> &GridMap {
        self.planner.map_ref()
    }

    /// 可变地图访问（深度感知建图/动态障碍写入入口）。
    pub fn map_mut(&mut self) -> &mut GridMap {
        self.planner.map_mut()
    }

    #[must_use]
    pub fn global_path(&self) -> &[Vector3<f64>] {
        &self.global_path
    }

    #[must_use]
    pub fn goal(&self) -> Point3<f64> {
        self.goal
    }

    #[must_use]
    pub const fn replans(&self) -> usize {
        self.replans
    }

    #[must_use]
    pub const fn is_finished(&self) -> bool {
        self.finished
    }

    /// 当前执行的局部轨迹。
    #[must_use]
    pub fn local(&self) -> Option<&LocalTraj> {
        self.local.as_ref()
    }

    /// 最近一次成功规划的结果（可视化平面等衍生数据源）。
    #[must_use]
    pub fn last_result(&self) -> Option<&PlanResult> {
        self.last_result.as_ref()
    }

    /// 主循环单步（10Hz，官方 `execFSMCallback`）。
    ///
    /// `measured`：最新里程计观测（新鲜时由应用层传入；`None` 时管理器以
    /// 轨迹参考推进作为位置估计）。返回本 tick 的参考指令与状态标志。
    /// 规划失败不报错——冷却重试语义在管理器内（官方 FSM 语义）。
    #[must_use]
    pub fn tick(&mut self, now: f64, measured: Option<State>) -> TickReport {
        let mut report = TickReport::default();
        match self.local {
            None => self.initial_plan(now, measured),
            Some(ref local) => {
                let t_cur = now - local.start_time;
                if t_cur > local.traj.duration() {
                    // 轨迹执行完毕：仅当物理到达目标才完成任务（防提前结束）
                    let pos = self.estimated_position(now, measured);
                    if self.touch_goal && (pos - self.goal.coords).norm() < self.options.arrive_dist
                    {
                        self.finished = true;
                        report.finished = true;
                        return report;
                    }
                    log::warn!("轨迹执行完毕但未到达目标，强制重规划");
                    if now >= self.replan_cooldown_until {
                        self.replan(now);
                        report.replanned = true;
                    }
                } else if t_cur > self.options.replan_thresh && now >= self.replan_cooldown_until {
                    self.replan(now);
                    report.replanned = true;
                }
            }
        }
        // 参考指令：跟踪当前轨迹；耗尽且连续失败时沿全局路径脱困直飞
        report.reference = self.reference(now, measured);
        // 到达判定（任意轨迹阶段，物理位置为准）
        if self.local.is_some() {
            let pos = self.estimated_position(now, measured);
            if (pos - self.goal.coords).norm() < self.options.arrive_dist {
                log::info!(
                    "到达目标 ({:.1},{:.1},{:.1})，任务完成（重规划 {} 次）",
                    pos.x,
                    pos.y,
                    pos.z,
                    self.replans
                );
                self.finished = true;
                report.finished = true;
            }
        }
        report
    }

    /// 位置估计：实测优先（odom），否则沿当前轨迹参考推进。
    fn estimated_position(&self, now: f64, measured: Option<State>) -> Vector3<f64> {
        if let Some(s) = measured {
            return s.position.coords;
        }
        match &self.local {
            Some(local) => {
                let t_cur = (now - local.start_time).clamp(0.0, local.traj.duration());
                local.traj.eval(t_cur).position
            }
            None => self.global_path.first().copied().unwrap_or_default(),
        }
    }

    /// 首次规划（官方 `GEN_NEW_TRAJ`）：起点为实测位置（无观测时全局路径起点），
    /// 失败保持无轨迹下一帧重试（官方 FSM 语义，不退出）。
    fn initial_plan(&mut self, now: f64, measured: Option<State>) {
        let start = measured.unwrap_or(State {
            position: Point3::from(self.global_path.first().copied().unwrap_or_default()),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        });
        match self.planner.plan(start, self.goal) {
            Ok(result) => {
                self.last_result = Some(result.clone());
                self.local = Some(LocalTraj {
                    traj: result.trajectory,
                    start_time: now,
                });
            }
            Err(e) => log::warn!("初始规划失败：{e}，下一帧重试"),
        }
    }

    /// 重规划（官方 `planFromLocalTraj`）：起点取上一轨迹在重规划时刻的
    /// **参考状态**（保证前后参考时间连续——改用实测滞后位置会引发
    /// "进-退"振荡）；初始解走暖启动优先、冷启动兜底的降级链。
    fn replan(&mut self, now: f64) {
        let Some(local) = &self.local else {
            return;
        };
        let t_cur = (now - local.start_time).clamp(0.0, local.traj.duration());
        let s = local.traj.eval(t_cur);
        let start = State {
            position: Point3::new(s.position.x, s.position.y, s.position.z),
            velocity: s.velocity,
            acceleration: s.acceleration,
        };
        let horizon = self.horizon(start.position.coords);
        log::info!(
            "replan #{:03} t={now:.1}s 从 ({:.1},{:.1},{:.1}) 到 ({:.1},{:.1},{:.1}){}",
            self.replans,
            start.position.coords.x,
            start.position.coords.y,
            start.position.coords.z,
            horizon.target.coords.x,
            horizon.target.coords.y,
            horizon.target.coords.z,
            if horizon.touch_goal {
                " [touch_goal]"
            } else {
                ""
            }
        );
        self.replans += 1;

        // 官方降级链：case2 暖启动优先，失败降级 case1 冷启动
        //（克隆轨迹绕开 &mut self 与 &local.traj 的借用冲突，控制点量级小）
        let warm = match &self.local {
            Some(l) => Some((l.traj.clone(), now - l.start_time, horizon.tail.clone())),
            None => None,
        };
        let planned = match &warm {
            Some((prev, elapsed, tail)) => self.planner.plan_with_init(
                start,
                horizon.target,
                InitSource::WarmStart {
                    prev,
                    elapsed: *elapsed,
                    guide_tail: tail,
                },
            ),
            None => self.planner.plan(start, horizon.target),
        };
        let result = match planned {
            Ok(r) => {
                self.replan_fail_streak = 0;
                r
            }
            Err(warm_err) => {
                if warm.is_some() {
                    match self.planner.plan(start, horizon.target) {
                        Ok(r) => {
                            log::info!("暖启动失败（{warm_err}），冷启动成功");
                            r
                        }
                        Err(e) => {
                            log::warn!("重规划失败，保持旧轨迹：{e}");
                            self.replan_cooldown_until = now + REPLAN_COOLDOWN;
                            self.replan_fail_streak += 1;
                            return;
                        }
                    }
                } else {
                    log::warn!("重规划失败，保持旧轨迹：{warm_err}");
                    self.replan_cooldown_until = now + REPLAN_COOLDOWN;
                    self.replan_fail_streak += 1;
                    return;
                }
            }
        };
        // 退化轨迹防护：时长过短的异常解会触发逐 tick 强制重规划空转
        if result.trajectory.duration() < 0.5 {
            log::warn!(
                "重规划产出退化轨迹（时长 {:.2}s），保持旧轨迹",
                result.trajectory.duration()
            );
            self.replan_cooldown_until = now + REPLAN_COOLDOWN;
            return;
        }
        if horizon.touch_goal {
            self.touch_goal = true;
        }
        self.last_result = Some(result.clone());
        self.local = Some(LocalTraj {
            traj: result.trajectory,
            start_time: now,
        });
    }

    /// 局部目标选取（官方 `getLocalTarget`）：全局路径上距当前位置
    /// `planning_horizon` 弧长处；落进障碍膨胀区或贴障时沿路径逐步回退
    /// （每步 0.4m）到安全点。同时产出暖启动延续段。
    fn horizon(&self, start: Vector3<f64>) -> Horizon {
        let mut arc = self.options.planning_horizon;
        loop {
            let h = self.walk_global_path(start, arc);
            if h.touch_goal || arc <= 0.0 {
                return h;
            }
            if self.target_clear(h.target.coords) {
                return h;
            }
            arc -= 0.4;
        }
    }

    /// 从 `start` 在全局路径上的最近投影起累计弧长 `arc`：
    /// 返回目标点、是否触及终点、以及沿途经过的路径段（暖启动 tail）。
    fn walk_global_path(&self, start: Vector3<f64>, arc: f64) -> Horizon {
        let goal_point = Point3::from(self.goal.coords);
        if self.global_path.len() < 2 {
            return Horizon {
                target: goal_point,
                touch_goal: true,
                tail: Vec::new(),
            };
        }
        // 定位 start 的最近段（官方沿 global_traj 投影）
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
        // 从投影点起沿剩余路径累计弧长
        let mut tail: Vec<Vector3<f64>> = vec![start];
        let mut acc = 0.0;
        let mut prev = start;
        for point in &self.global_path[seg + 1..] {
            let segment = *point - prev;
            let len = segment.norm();
            if acc + len >= arc {
                let t = (arc - acc) / len;
                tail.push(prev + segment * t);
                return Horizon {
                    target: Point3::from(prev + segment * t),
                    touch_goal: false,
                    tail,
                };
            }
            acc += len;
            tail.push(*point);
            prev = *point;
        }
        Horizon {
            target: goal_point,
            touch_goal: true,
            tail,
        }
    }

    /// 目标点安全判据：不在膨胀占据区，且 26 邻域无占据体素——给 MINCO
    /// 留足绕弯余量，避免轨迹切墙角导致优化卡死。
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
                        == firefly_map::VoxelState::Occupied
                    {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// 参考指令生成：正常态取轨迹在 `now` 的参考状态（时间连续）；轨迹耗尽
    /// 且连续重规划失败（贴墙死锁）→ 沿全局路径向下一自由点直飞脱困
    /// （物理移动解开几何死锁，也给 VIO 提供视差）。
    fn reference(&mut self, now: f64, measured: Option<State>) -> Option<Reference> {
        let local = self.local.as_ref()?;
        let t_cur = (now - local.start_time).clamp(0.0, local.traj.duration());
        if t_cur >= local.traj.duration() - 1e-6 && self.replan_fail_streak >= FAIL_STREAK_ESCAPE {
            let pos = self.estimated_position(now, measured);
            let escape = self.walk_global_path(pos, 1.0);
            let dir = escape.target.coords - pos;
            let dir = if dir.norm_squared() < 1e-9 {
                Vector3::zeros()
            } else {
                dir.normalize()
            };
            log::info!(
                "脱困回退: 当前位置({:.2},{:.2}) 目标({:.2},{:.2})",
                pos.x,
                pos.y,
                escape.target.coords.x,
                escape.target.coords.y
            );
            return Some(Reference {
                position: escape.target.coords,
                velocity: ESCAPE_SPEED * dir,
            });
        }
        let s = local.traj.eval(t_cur);
        Some(Reference {
            position: s.position,
            velocity: s.velocity,
        })
    }
}

/// 路径累计长度。
fn path_length(points: &[Vector3<f64>]) -> f64 {
    points.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
}
