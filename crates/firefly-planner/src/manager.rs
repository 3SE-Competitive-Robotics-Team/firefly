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
        let global_path = search_global_path(planner.map_ref(), &mut astar, start, goal)?;
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

    /// 动态重目标（外部工具经 `Firefly/Goal` 发布新目标点）：从当前位置
    /// 重算全局路径（A* + 简化）、重置状态机，下一 tick 即重新规划飞往新目标。
    /// 无人机悬停等待目标时 `goal == start`（零长路径 → 原地悬停），
    /// 收到目标后自然切换。
    ///
    /// # Errors
    ///
    /// 新目标不可达（A* 失败 / 地图外）——保持原目标不变，由调用方记录。
    pub fn set_goal(
        &mut self,
        now: f64,
        measured: Option<State>,
        goal: Vector3<f64>,
    ) -> Result<()> {
        // 同目标幂等：CLI 为防一次性投递竞态会连发数条，重复目标不重置
        // 状态机（避免中途丢弃当前轨迹导致规划抖动）。
        if (goal - self.goal.coords).norm() < 1e-6 {
            return Ok(());
        }
        let start = self.estimated_position(now, measured);
        let mut astar = Astar::default();
        let global_path = search_global_path(self.planner.map_ref(), &mut astar, start, goal)?;
        log::info!(
            "目标更新为 ({:.1},{:.1},{:.1})：全局路径 {} 点，长度 {:.1}m",
            goal.x,
            goal.y,
            goal.z,
            global_path.len(),
            path_length(&global_path)
        );
        self.global_path = global_path;
        self.goal = Point3::from(goal);
        // 重置状态机：丢弃旧轨迹/旧终点标记，下一 tick 初始规划飞往新目标
        self.local = None;
        self.last_result = None;
        self.touch_goal = false;
        self.finished = false;
        self.replans = 0;
        self.replan_cooldown_until = 0.0;
        self.replan_fail_streak = 0;
        Ok(())
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
        // 官方 GEN_NEW_TRAJ:目标即全局终点 → touch_goal = true
        match self
            .planner
            .plan_with_init(start, self.goal, InitSource::ColdStart, true)
        {
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
                horizon.touch_goal,
            ),
            None => self.planner.plan_with_init(
                start,
                horizon.target,
                InitSource::ColdStart,
                horizon.touch_goal,
            ),
        };
        let result = match planned {
            Ok(r) => {
                self.replan_fail_streak = 0;
                r
            }
            Err(warm_err) => {
                if warm.is_some() {
                    match self.planner.plan_with_init(
                        start,
                        horizon.target,
                        InitSource::ColdStart,
                        horizon.touch_goal,
                    ) {
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

/// A* 搜索 + 字符串拉直：删除可直线直达的中间点（避免直线擦边穿越膨胀层）。
fn search_global_path(
    map: &GridMap,
    astar: &mut Astar,
    start: Vector3<f64>,
    goal: Vector3<f64>,
) -> Result<Vec<Vector3<f64>>> {
    let path = astar.search(map, start, goal)?;
    Ok(firefly_search::simplify_path(map, path.points()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PlannerConfig;
    use firefly_map::{GridMapBuilder, VoxelState};

    fn state_at(p: Vector3<f64>) -> State {
        State {
            position: Point3::from(p),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        }
    }

    /// 沿折线从 `from`(视为在路径上)累计弧长 `arc` 的点(测试预言机,
    /// 与实现独立——只做简单的逐段行走)。
    fn arc_point(path: &[Vector3<f64>], from: Vector3<f64>, arc: f64) -> Vector3<f64> {
        let mut remaining = arc;
        let mut prev = from;
        for p in path.iter().skip(1) {
            let seg = *p - prev;
            let len = seg.norm();
            if remaining <= len {
                return prev + seg * (remaining / len.max(1e-12));
            }
            remaining -= len;
            prev = *p;
        }
        *path.last().unwrap()
    }

    /// 空旷地图上的管理器:起点 (1,1,1) → 终点 (10,1,1)。
    fn open_manager() -> PlannerManager {
        let map = GridMapBuilder::new(0.5, [40, 24, 16]).build().unwrap();
        let planner = Planner::new(PlannerConfig::default(), map);
        PlannerManager::with_planner(
            planner,
            ManagerOptions::default(),
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(10.0, 1.0, 1.0),
        )
        .unwrap()
    }

    #[test]
    fn walk_global_path_arc_projection() {
        let m = open_manager();
        // 断言沿**实际**全局路径的弧长语义,不假设路径经过固定坐标
        let path = m.global_path().to_vec();
        let start = path[0];
        let h = m.walk_global_path(start, 6.0);
        assert!(!h.touch_goal, "6m 应未到终点");
        let expect = arc_point(&path, start, 6.0);
        assert!(
            (h.target.coords - expect).norm() < 1e-6,
            "弧长 6m 目标 = 沿路径走 6m:期望 {expect:?},实际 {:?}",
            h.target.coords
        );
        // tail:起点 + 沿途 + 目标点
        assert!(h.tail.len() >= 2);
        assert!((*h.tail.last().unwrap() - h.target.coords).norm() < 1e-6);
        // 弧长远超路径末端 → touch_goal,目标 = 全局终点
        let h2 = m.walk_global_path(start, 1e9);
        assert!(h2.touch_goal);
        assert!((h2.target.coords - m.goal().coords).norm() < 1e-6);
        // 起点在路径中途:从当前位置起算弧长
        let mid = arc_point(&path, start, 3.0);
        let h3 = m.walk_global_path(mid, 2.0);
        let expect3 = arc_point(&path, mid, 2.0);
        assert!(
            (h3.target.coords - expect3).norm() < 1e-6,
            "中途起点应按剩余路径走弧长:期望 {expect3:?},实际 {:?}",
            h3.target.coords
        );
    }

    #[test]
    fn target_clear_free_and_blocked() {
        let mut m = open_manager();
        // 空旷 → 自由
        assert!(m.target_clear(Vector3::new(5.0, 1.0, 1.0)));
        // 加一堵墙(x=5.0,res 0.5 → 体素 [10,2,2])并膨胀 0.2
        m.map_mut().set_state([10, 2, 2], VoxelState::Occupied);
        m.map_mut().inflate_obstacles(0.2);
        // 墙点(膨胀后 x∈[4.8,5.2])→ 不自由
        assert!(!m.target_clear(Vector3::new(5.0, 1.0, 1.0)));
        // 远处自由
        assert!(m.target_clear(Vector3::new(9.0, 1.0, 1.0)));
    }

    #[test]
    fn horizon_backs_off_blocked_target() {
        let mut m = open_manager();
        // 动态取弧长目标的落点,把墙放在那里(不假设坐标);
        // 目标被堵后 horizon 应沿路径回退到安全点。
        let path = m.global_path().to_vec();
        let start = path[0];
        let arc_target = m.walk_global_path(start, 6.0).target;
        let idx = m.map().index_of(arc_target.coords).expect("目标在地图内");
        m.map_mut()
            .set_state([idx[0], idx[1], idx[2]], VoxelState::Occupied);
        m.map_mut().inflate_obstacles(0.2);
        // 场景前提:弧长目标确实被堵
        assert!(
            !m.target_clear(arc_target.coords),
            "场景前提:弧长目标应被堵"
        );

        let h = m.horizon(start);
        assert!(!h.touch_goal, "未到终点不应 touch_goal");
        assert!(m.target_clear(h.target.coords), "回退后的目标必须安全");
        // 回退:目标严格比 arc_target 更接近起点(沿路径),且未退过头
        let d_arc = (arc_target.coords - start).norm();
        let d_h = (h.target.coords - start).norm();
        assert!(d_h < d_arc - 1e-6, "被堵目标应回退({d_h:.3} vs {d_arc:.3})");
        assert!(d_h > d_arc - 1.5, "回退不应退过头({d_h:.3} vs {d_arc:.3})");
    }

    #[test]
    fn tick_initial_plan_then_replan_at_threshold() {
        let mut m = open_manager();
        // 首帧:初始规划建立局部轨迹
        let r0 = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        assert!(m.local().is_some(), "首帧应建立局部轨迹");
        assert!(!r0.replanned);
        assert_eq!(m.replans(), 0, "初始规划不计入重规划次数");
        let ref0 = r0.reference.expect("首帧应有参考指令");
        assert!((ref0.position - Vector3::new(1.0, 1.0, 1.0)).norm() < 1e-6);
        assert!(!m.is_finished());
        // t=1.5 > replan_thresh(1.0) → 触发重规划(暖启动)
        let r1 = m.tick(1.5, None);
        assert!(r1.replanned, "超过阈值应重规划");
        assert_eq!(m.replans(), 1);
        assert!(r1.reference.is_some());
    }

    #[test]
    fn tick_arrival_finishes() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        // 实测位置到达目标 → 任务完成(任意轨迹阶段)
        let r = m.tick(2.0, Some(state_at(m.goal().coords)));
        assert!(r.finished, "到达目标应标记完成");
        assert!(m.is_finished());
    }

    #[test]
    fn warm_start_falls_back_to_cold_when_exhausted() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        // 重规划到 horizon(非终点)的局部轨迹,使"轨迹耗尽"不会误触到达判定
        let _ = m.tick(1.5, None);
        let dur = m.local().unwrap().traj.duration();
        assert!(m.local().unwrap().start_time < 1.5 + 1e-9);
        // 轨迹耗尽(now = start + duration + 0.1),未到达目标 → 强制重规划。
        // 暖启动的 elapsed 超过旧轨迹时长 → init_warm_start 必然失败
        // → 降级冷启动(官方 case2 → case1 策略链)仍须成功。
        let t_end = m.local().unwrap().start_time + dur + 0.1;
        let r = m.tick(t_end, None);
        assert!(r.replanned, "轨迹耗尽应强制重规划");
        assert_eq!(m.replans(), 2, "降级链应产出一次重规划");
        assert!(m.local().is_some(), "降级后应仍有新轨迹");
        assert!(!m.is_finished());
    }

    #[test]
    fn escape_fly_when_traj_exhausted_and_failing() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        // 先重规划出 horizon 局部轨迹(终点 = 规划视界内某点,≠ 全局终点)
        let _ = m.tick(1.5, None);
        let dur = m.local().unwrap().traj.duration();
        m.replan_cooldown_until = f64::INFINITY; // 阻止重规划,聚焦 reference 分支
        // 脱困方向 = 全局路径前进方向(不假设具体坐标轴)
        let path = m.global_path();
        let path_dir = (path[1] - path[0]).normalize();

        // 正常态:轨迹末端参考 = 轨迹终点状态(速度≈0)
        let r1 = m.tick(m.local().unwrap().start_time + dur + 0.1, None);
        let ref1 = r1.reference.expect("正常态应有参考");
        assert!(
            ref1.velocity.norm() < 0.1,
            "末端参考速度应≈0,实际 {}",
            ref1.velocity.norm()
        );
        assert!(!m.is_finished(), "末端未到达全局终点不应完成");

        // 脱困态:连续失败达阈值且轨迹耗尽 → 沿全局路径向下一自由点直飞
        m.replan_fail_streak = FAIL_STREAK_ESCAPE;
        let r2 = m.tick(m.local().unwrap().start_time + dur + 0.2, None);
        let ref2 = r2.reference.expect("脱困态应有参考");
        let expect_vel = ESCAPE_SPEED * path_dir;
        assert!(
            (ref2.velocity - expect_vel).norm() < 1e-6,
            "脱困应沿路径直飞(速度 {expect_vel:?}),实际 {:?}",
            ref2.velocity
        );
        assert!(
            (ref2.position - ref1.position).dot(&path_dir) > 0.5,
            "脱困目标应沿路径前移(末端 {:?},脱困 {:?})",
            ref1.position,
            ref2.position
        );
    }

    #[test]
    fn set_goal_rebuilds_path_and_dedups_same_target() {
        let mut m = open_manager();
        // 先正常规划出局部轨迹(飞行中状态)
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        let _ = m.tick(1.5, None);
        assert!(m.local().is_some(), "前置:应有局部轨迹");

        // 动态重目标:新目标重建全局路径、重置状态机(目标取地图内部点)
        let new_goal = Vector3::new(15.0, 1.0, 1.0);
        m.set_goal(2.0, None, new_goal).unwrap();
        assert!((m.goal().coords - new_goal).norm() < 1e-9, "目标应更新");
        assert_eq!(m.global_path().last().unwrap(), &new_goal);
        assert!(m.local().is_none(), "重目标应丢弃旧轨迹");
        assert!(!m.is_finished());
        // 重目标后下一 tick 应重新规划出新轨迹(初始规划不计 replans,
        // 语义见 tick_initial_plan_then_replan_at_threshold)
        let _ = m.tick(2.1, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        let local = m.local().expect("重目标后应重新规划出新轨迹");
        assert!(
            (local.start_time - 2.1).abs() < 1e-9,
            "新轨迹应在重目标后的 tick 生成(start_time={})",
            local.start_time
        );

        // 同目标幂等:CLI 连发防竞态,重复目标不得重置状态机/清空轨迹
        let replans_after_retarget = m.replans();
        let local_after_retarget = m.local().is_some();
        m.set_goal(2.2, None, new_goal).unwrap();
        assert_eq!(
            m.local().is_some(),
            local_after_retarget,
            "同目标重复 set_goal 不应丢弃当前轨迹"
        );
        assert_eq!(
            m.replans(),
            replans_after_retarget,
            "同目标重复 set_goal 不应触发重规划计数"
        );
    }
}
