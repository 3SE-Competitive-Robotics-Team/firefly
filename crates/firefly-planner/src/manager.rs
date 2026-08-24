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
use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder, Trajectory};
use nalgebra::{Point3, Vector3};

use crate::obstacles::{ObstacleScanner, PointsToCheck};
use crate::planner::{InitSource, PlanResult, Planner, State};

/// 重规划触发阈值（秒，官方 `fsm/thresh_replan_time`）。
pub const DEFAULT_REPLAN_THRESH: f64 = 1.0;
/// 急停时限（秒，官方 `fsm/emergency_time`）：碰撞监控命中后重规划失败、
/// 且剩余碰撞时间小于该值时进入急停。
pub const DEFAULT_EMERGENCY_TIME: f64 = 1.0;
/// 急停恢复速度阈值（m/s）：fail-safe 允许且测量速度低于该值才从急停
/// 恢复重规划（官方 `EMERGENCY_STOP` case 的 0.1）。
pub const DEFAULT_EMERGENCY_RECOVER_SPEED: f64 = 0.1;
/// 停车轨迹单段时长（秒，官方 `EmergencyStop` 的两段各 1.0）。
const STOP_PIECE_DURATION: f64 = 1.0;
/// 规划视界（米，官方 `manager/planning_horizon`）。
pub const DEFAULT_PLANNING_HORIZON: f64 = 6.0;
/// 到达判定距离（米）。
pub const DEFAULT_ARRIVE_DIST: f64 = 0.5;
const REPLAN_COOLDOWN: f64 = 0.5;
/// 连续重规划失败达到该次数且轨迹耗尽时，沿全局路径脱困直飞。
const FAIL_STREAK_ESCAPE: usize = 3;
/// 脱困引导速度（m/s）。
const ESCAPE_SPEED: f64 = 1.0;
/// 前视时间（秒，官方 `traj_server/time_forward`，swarm-playground 各 launch
/// 均取 1.0）：yaw 期望方向取参考位置再往前该时长的轨迹点。
const TIME_FORWARD: f64 = 1.0;
/// yaw 角速度上限（rad/s，官方 `traj_server` 的 `YAW_DOT_MAX_PER_SEC`）。
const YAW_DOT_MAX: f64 = 2.0 * std::f64::consts::PI;
/// yaw 角加速度上限（rad/s²，官方 `YAW_DOT_DOT_MAX_PER_SEC`）。
const YAW_DOT_DOT_MAX: f64 = 5.0 * std::f64::consts::PI;
/// 地面高度测量前视距离（米，官方 `measureGroundHeight` 的 `2.0/max_vel` 前方采样）。
const GROUND_LOOKAHEAD_DIST: f64 = 2.0;

/// 管理器行为参数。
#[derive(Debug, Clone, Copy, serde::Deserialize)]
#[serde(default)]
pub struct ManagerOptions {
    /// 重规划触发阈值（秒）。
    pub replan_thresh: f64,
    /// 规划视界（米）。
    pub planning_horizon: f64,
    /// 到达判定距离（米）。
    pub arrive_dist: f64,
    /// 急停时限（秒，见 [`DEFAULT_EMERGENCY_TIME`]）。
    pub emergency_time: f64,
    /// 急停恢复速度阈值（m/s，见 [`DEFAULT_EMERGENCY_RECOVER_SPEED`]）。
    pub emergency_recover_speed: f64,
}

impl Default for ManagerOptions {
    fn default() -> Self {
        Self {
            replan_thresh: DEFAULT_REPLAN_THRESH,
            planning_horizon: DEFAULT_PLANNING_HORIZON,
            arrive_dist: DEFAULT_ARRIVE_DIST,
            emergency_time: DEFAULT_EMERGENCY_TIME,
            emergency_recover_speed: DEFAULT_EMERGENCY_RECOVER_SPEED,
        }
    }
}

/// 执行中的局部轨迹（官方 `LocalTrajData`：轨迹 + 起始时刻 + 检查点）。
#[derive(Debug)]
pub struct LocalTraj {
    pub traj: Trajectory,
    /// 轨迹起始时刻（管理器时钟，秒）；检查点/监控的时间原点。
    pub start_time: f64,
    /// 带时间戳检查点（官方 `PtsChk_t`，规划成功后生成一次）：执行期
    /// 碰撞监控的扫描源。不变量：非空（采样退化的轨迹不入执行，
    /// 见 [`PlannerManager::adopt_plan`]）。
    pub points_to_check: PointsToCheck,
}

/// 参考状态指令（闭环 PD 跟踪目标）。
#[derive(Debug, Clone, Copy)]
pub struct Reference {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    /// 参考偏航角（rad，地图系 x 轴为零向，包装在 [-π,π]；对照官方
    /// `traj_server::calculate_yaw` 的限幅输出）。
    pub yaw: f64,
    /// 参考偏航角速度（rad/s，两级限幅后的实际变化率；非官方发布值——
    /// 官方 `position_cmd.yaw_dot` 实际携带的是未限幅期望航向，此处按字段
    /// 语义取限幅后的角速度）。
    pub yaw_dot: f64,
}

/// 一次 [`PlannerManager::tick`] 的产出。
// 各标志相互独立（参考/重规划/到达/安全监控），非状态机编码
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Default)]
pub struct TickReport {
    /// 参考状态（`None` = 尚无可执行轨迹）。
    pub reference: Option<Reference>,
    /// 本 `tick` 产生了新轨迹（应用层据此记录可视化产物）。
    pub replanned: bool,
    /// 任务完成（物理到达目标）。
    pub finished: bool,
    /// 执行期碰撞监控命中：当前轨迹前方存在障碍，本 tick 已绕过
    /// `replan_thresh` 与冷却期立即重规划。
    pub collision: bool,
    /// 当前处于急停态（官方 `EMERGENCY_STOP`）：参考输出为停车轨迹上的
    /// 原地定点；恢复条件满足前持续为真。管理器不做锁桨等底层处置——
    /// 那由应用层按本标志决定。
    pub emergency_stop: bool,
    /// 地面高度测量（对照官方 `measureGroundHeight`）：当前轨迹前方
    /// [`GROUND_LOOKAHEAD_DIST`] 米处采样点正下方的地面 z 坐标（地图系，
    /// 米）。不变量：返回的是首个占据体素的采样 z（按分辨率逐格下降的
    /// 离散值），非连续表面拟合。`None` = 本 tick 未测得（无可执行轨迹、
    /// 前方点超出当前轨迹时长、或扫描至图底仍无占据）。
    pub ground_height: Option<f64>,
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
/// fail-safe 开关与急停态是独立开关（急停中可关 fail-safe 禁自动恢复），
/// 非状态机枚举。
#[allow(clippy::struct_excessive_bools)]
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
    /// 上一帧参考偏航角（rad，[-π,π]；官方 `traj_server` 的
    /// `last_yaw_`，初值 0）。
    last_yaw: f64,
    /// 上一帧参考偏航角速度（rad/s；官方 `last_yawdot_`，初值 0）。
    last_yaw_dot: f64,
    /// 上次 yaw 更新时刻（管理器时钟，秒）：限幅剖面按帧间 `dt` 推进。
    last_yaw_time: Option<f64>,
    /// fail-safe 开关（官方 `enable_fail_safe_`，初值 true）：急停自动恢复的
    /// 前提；深度丢失触发入口将其关闭后不自动恢复。
    failsafe_enabled: bool,
    /// 急停态（官方 `EMERGENCY_STOP` 态）：进入时生成一次停车轨迹并入库为
    /// 可执行轨迹，参考输出变为原地定点悬停；恢复判据见 [`Self::tick`]。
    emergency: bool,
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
            last_yaw: 0.0,
            last_yaw_dot: 0.0,
            last_yaw_time: None,
            failsafe_enabled: true,
            emergency: false,
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
        self.emergency = false;
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

    /// 当前偏航状态 `(yaw, yaw_dot)`（rad / rad/s）：到达悬停等无轨迹
    /// 阶段由应用层构造 [`Reference`] 时沿用，保持最后朝向。
    #[must_use]
    pub const fn yaw_state(&self) -> (f64, f64) {
        (self.last_yaw, self.last_yaw_dot)
    }

    /// 主循环单步（10Hz，官方 `execFSMCallback`）。
    ///
    /// `measured`：最新里程计观测（新鲜时由应用层传入；`None` 时管理器以
    /// 轨迹参考推进作为位置估计）。返回本 tick 的参考指令与状态标志。
    /// 规划失败不报错——冷却重试语义在管理器内（官方 FSM 语义）。
    /// 含执行期碰撞监控（见 [`Self::next_collision`]）：命中即绕过
    /// `replan_thresh` 与冷却期立即重规划。
    ///
    /// 急停态（官方 case EMERGENCY_STOP）只做恢复判定：fail-safe 允许且
    /// 测量速度低于 [`ManagerOptions::emergency_recover_speed`] 时从当前
    /// 位姿沿全局轨迹完整重规划退出；无测量观测时不恢复（速度无法核实）。
    /// 常规重规划/碰撞监控仅在非急停态生效（对照官方仅 `EXEC_TRAJ` case
    /// 处理重规划的结构）。
    #[must_use]
    pub fn tick(&mut self, now: f64, measured: Option<State>) -> TickReport {
        let mut report = TickReport::default();
        if self.emergency {
            // 官方 case EMERGENCY_STOP：停车轨迹已在进入时生成一次，本态只判恢复
            if self.failsafe_enabled
                && let Some(start) =
                    measured.filter(|s| s.velocity.norm() < self.options.emergency_recover_speed)
                && self.recover_from_global_traj(now, start)
            {
                report.replanned = true;
            }
        } else {
            self.tick_normal(now, measured, &mut report);
        }
        // 地面高度测量（官方 checkCollisionCallback 入口的顺带测量：
        // 无可执行轨迹时内部自返回 None）
        report.ground_height = self.measure_ground_height(now);
        // 参考指令：跟踪当前轨迹；耗尽且连续失败时沿全局路径脱困直飞
        report.reference = self.reference(now, measured);
        // 到达判定（任意轨迹阶段，物理位置为准；急停悬停不算完成任务）
        if self.local.is_some() && !self.emergency {
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
        report.emergency_stop = self.emergency;
        report
    }

    /// 非急停态的常规执行分支：初始规划 / 碰撞监控与立即重规划 /
    /// 阈值周期重规划 / 轨迹耗尽强制重规划。
    fn tick_normal(&mut self, now: f64, measured: Option<State>, report: &mut TickReport) {
        match self.local {
            None => self.initial_plan(now, measured),
            Some(ref local) => {
                let t_cur = now - local.start_time;
                let traj_duration = local.traj.duration();
                // 执行期膨胀层刷新（官方 clearAndInflateLocalMap 的监控侧调用）：
                // 地图可能在本 tick 被深度感知/动态障碍更新，目标点检查与碰撞
                // 监控都必须基于最新膨胀层
                self.refresh_inflation();
                // 目标落入障碍修正（官方 EXEC_TRAJ case 1，居各分支之首；先于
                // 到达判定——防止飞抵障碍内目标附近被误判任务完成）
                if self.modify_in_collision_final_goal(now, measured, report) {
                    // pass（对照官方 case 1 结构）
                } else if t_cur > traj_duration {
                    // 轨迹执行完毕：仅当物理到达目标才完成任务（防提前结束）
                    let pos = self.estimated_position(now, measured);
                    if self.touch_goal && (pos - self.goal.coords).norm() < self.options.arrive_dist
                    {
                        self.finished = true;
                        report.finished = true;
                        return;
                    }
                    log::warn!("轨迹执行完毕但未到达目标，强制重规划");
                    if now >= self.replan_cooldown_until {
                        report.replanned = self.replan(now);
                    }
                } else {
                    // 执行期碰撞监控（官方 checkCollisionCallback，每 tick 一次
                    // 等价其 20Hz 定时器）：沿未来检查点扫描（膨胀层已在分支入口刷新）
                    if let Some(hit_t) = self.next_collision(now) {
                        // 官方发现碰撞路径：立即 planFromLocalTraj，不受阈值与
                        // 冷却期约束；失败且碰撞迫近急停时限 → 进入急停
                        log::warn!(
                            "碰撞监控：当前轨迹 {:.2}s 后进入障碍，立即重规划",
                            hit_t - t_cur
                        );
                        report.collision = true;
                        report.replanned = self.replan(now);
                        if !report.replanned && hit_t - t_cur < self.options.emergency_time {
                            log::warn!("重规划失败且碰撞迫近（{:.2}s），进入急停", hit_t - t_cur);
                            self.enter_emergency_stop(now, measured);
                        }
                    } else if t_cur > self.options.replan_thresh
                        && now >= self.replan_cooldown_until
                    {
                        report.replanned = self.replan(now);
                    }
                }
            }
        }
    }

    /// 目标落入障碍修正（对照官方 `ego_replan_fsm::mondifyInCollisionFinalGoal`，
    /// `EXEC_TRAJ` case 1、各分支之首）：终点位于膨胀占据区时，沿当前全局路径
    /// 从末端向起点以分辨率步长回扫（官方时间步 `t_step = resolution /
    /// max_vel` 在匀速上限假设下对应一个分辨率的弧长），找首个自由点；找到后
    /// 从当前位置对该点重生成全局路径并换目标（对照官方 `planNextWaypoint`——
    /// 不触碰 replans/冷却期/连败计数/急停态等执行态），随即本 tick 立即重规划
    /// （官方经 `REPLAN_TRAJ` 延迟一拍；firefly 无独立状态机节拍，取碰撞命中
    /// 路径的立即重规划作为等价执行节奏）。某点全局路径搜索失败视为该点
    /// 不可用，继续向前回扫（对照官方 planNextWaypoint 失败继续循环）；回扫
    /// 耗尽仅报错，目标与轨迹保持不变。返回是否完成了一次目标修正（前置
    /// "local 存在"由 [`Self::tick_normal`] 的调用位置保证）。
    fn modify_in_collision_final_goal(
        &mut self,
        now: f64,
        measured: Option<State>,
        report: &mut TickReport,
    ) -> bool {
        if !self
            .planner
            .map_ref()
            .is_occupied_inflated(self.goal.coords)
        {
            return false;
        }
        let orig_goal = self.goal.coords;
        let reso = self.planner.map_ref().resolution();
        // 小向量（A* 简化路径），克隆避开后续 &mut 借用冲突
        let path = self.global_path.clone();
        let cum = cumulative_arcs(&path);
        let mut s = cum.last().copied().unwrap_or(0.0);
        while s > 0.0 {
            let pt = point_at_arc(&path, &cum, s);
            if !self.planner.map_ref().is_occupied_inflated(pt) {
                let start = self.estimated_position(now, measured);
                let mut astar = Astar::default();
                match search_global_path(self.planner.map_ref(), &mut astar, start, pt) {
                    Ok(new_path) => {
                        log::info!(
                            "目标 ({:.2},{:.2},{:.2}) 落入障碍，修正为 ({:.2},{:.2},{:.2})",
                            orig_goal.x,
                            orig_goal.y,
                            orig_goal.z,
                            pt.x,
                            pt.y,
                            pt.z
                        );
                        self.global_path = new_path;
                        self.goal = Point3::from(pt);
                        report.replanned = self.replan(now);
                        return true;
                    }
                    Err(e) => log::debug!("候选修正点不可达（{e}），继续回扫"),
                }
            }
            if s <= reso {
                log::error!(
                    "全局路径上找不到任何无碰点，保持障碍内目标 ({:.2},{:.2},{:.2})",
                    orig_goal.x,
                    orig_goal.y,
                    orig_goal.z
                );
                break;
            }
            s -= reso;
        }
        false
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
                self.adopt_plan(result, now, true);
            }
            Err(e) => log::warn!("初始规划失败：{e}，下一帧重试"),
        }
    }

    /// 重规划（官方 `planFromLocalTraj`）：起点取上一轨迹在重规划时刻的
    /// **参考状态**（保证前后参考时间连续——改用实测滞后位置会引发
    /// "进-退"振荡）；初始解走暖启动优先、冷启动兜底的降级链。
    /// 返回是否入库了新轨迹；碰撞监控路径不受冷却期约束直接调用，
    /// 失败仍会设置冷却期（下一 tick 的阈值路径不空转）。
    fn replan(&mut self, now: f64) -> bool {
        let Some(local) = &self.local else {
            return false;
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
                            return false;
                        }
                    }
                } else {
                    log::warn!("重规划失败，保持旧轨迹：{warm_err}");
                    self.replan_cooldown_until = now + REPLAN_COOLDOWN;
                    self.replan_fail_streak += 1;
                    return false;
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
            return false;
        }
        if !self.adopt_plan(result, now, horizon.touch_goal) {
            // 无监控覆盖的轨迹不入执行，视同本次重规划失败（官方
            // setLocalTrajFromOpt 失败时同样不更新局部轨迹）
            self.replan_cooldown_until = now + REPLAN_COOLDOWN;
            self.replan_fail_streak += 1;
            return false;
        }
        true
    }

    /// 官方 `setLocalTrajFromOpt`：规划结果连同带时间戳检查点一并入库，
    /// 并登记该轨迹的 `touch_goal`（碰撞监控扫描上界依据）。检查点生成失败
    /// （稠密采样退化）视同规划失败——无监控覆盖的轨迹不得进入执行。
    /// 返回是否入库成功。
    fn adopt_plan(&mut self, result: PlanResult, now: f64, touch_goal: bool) -> bool {
        if !self.set_local_traj(result.trajectory.clone(), now, touch_goal) {
            // 无监控覆盖的轨迹不入执行，视同本次重规划失败（官方
            // setLocalTrajFromOpt 失败时同样不更新局部轨迹）
            return false;
        }
        self.last_result = Some(result);
        true
    }

    /// 官方 `setLocalTrajFromOpt`：轨迹连同带时间戳检查点一并入库。检查点
    /// 生成失败（稠密采样退化）视同入库失败——无监控覆盖的轨迹不得进入
    /// 执行。返回是否入库成功。
    fn set_local_traj(&mut self, traj: Trajectory, now: f64, touch_goal: bool) -> bool {
        let (samples, max_vel) = {
            let cfg = self.planner.config();
            (cfg.constraint_points_per_piece, cfg.max_velocity)
        };
        let scanner = ObstacleScanner::new(self.planner.map_ref())
            .with_samples(samples)
            .with_max_vel(max_vel);
        let Some(points_to_check) = scanner.compute_points_to_check(&traj, touch_goal) else {
            log::warn!("检查点生成失败（采样退化），丢弃本次轨迹");
            return false;
        };
        self.local = Some(LocalTraj {
            traj,
            start_time: now,
            points_to_check,
        });
        self.touch_goal = touch_goal;
        true
    }

    /// 刷新障碍膨胀层（官方 `clearAndInflateLocalMap` 的监控侧调用）：地图
    /// 可能在本 tick 被深度感知/动态障碍更新，碰撞判定必须基于最新膨胀层。
    fn refresh_inflation(&mut self) {
        let inflation = self.planner.config().obstacle_inflation;
        self.planner.map_mut().inflate_obstacles(inflation);
    }

    /// 执行期碰撞监控（官方 `checkCollisionCallback` 轨迹检查部分）：沿当前
    /// 轨迹检查点只扫 **未来** 采样，返回首个进入膨胀占据区的采样时刻
    /// （相对轨迹起点，秒）；无碰撞或无可执行轨迹返回 `None`。
    ///
    /// 定位对照官方两级 i/j：`i_start` = `t_cur` 所在段下标（桶序的保守
    /// 下界），再按时间戳推进到首个未来检查点——已飞过部分不回扫。
    /// 扫描上界对照官方：`touch_goal` 查全程，否则前 3/4 桶（生成端已按
    /// `two_thirds_id` 截断，此处再留余量）。集群间距不在本函数范围
    /// （swarm 约束由规划内层保证）。
    #[must_use]
    fn next_collision(&self, now: f64) -> Option<f64> {
        let local = self.local.as_ref()?;
        let pts = &local.points_to_check;
        if pts.is_empty() {
            return None;
        }
        let t_cur = now - local.start_time;
        let map = self.planner.map_ref();
        let mut i_start = piece_index_at(&local.traj, t_cur);
        if i_start >= pts.len() {
            return None;
        }
        let mut j_start = 0usize;
        let mut located = false;
        while i_start < pts.len() && !located {
            for (j, (t, _)) in pts[i_start].iter().enumerate() {
                if *t > t_cur {
                    j_start = j;
                    located = true;
                    break;
                }
            }
            if !located {
                i_start += 1;
            }
        }
        if !located {
            return None;
        }
        let scan_end = if self.touch_goal {
            pts.len()
        } else {
            pts.len() * 3 / 4
        };
        for (i, bucket) in pts.iter().enumerate().take(scan_end).skip(i_start) {
            let skip = if i == i_start { j_start } else { 0 };
            for (t, p) in bucket.iter().skip(skip) {
                if map.is_occupied_inflated(*p) {
                    return Some(*t);
                }
            }
        }
        None
    }

    /// 参考偏航计算（对照官方 `traj_server::calculate_yaw`）：期望航向取
    /// `pos → target` 方向（轨迹分支即前视 `TIME_FORWARD` 秒的方向；模长
    /// ≤ 0.1 m 保持上一帧 yaw），期望差做 ±π 包装后按角速度/角加速度
    /// 两级限幅的梯形剖面推进。状态存入 `last_yaw/last_yaw_dot`；首帧或
    /// `dt` 退化时保持当前状态不变。
    fn update_yaw(&mut self, pos: Vector3<f64>, target: Vector3<f64>, now: f64) -> (f64, f64) {
        let Some(t_last) = self.last_yaw_time else {
            self.last_yaw_time = Some(now);
            return (self.last_yaw, self.last_yaw_dot);
        };
        let dt = now - t_last;
        if dt <= 1e-9 {
            // 官方依赖 ros 定时器保证 dt > 0；此处显式防护除零
            return (self.last_yaw, self.last_yaw_dot);
        }
        self.last_yaw_time = Some(now);

        let dir = target - pos;
        let yaw_temp = if dir.norm() > 0.1 {
            dir.y.atan2(dir.x)
        } else {
            self.last_yaw
        };

        let mut d_yaw = yaw_temp - self.last_yaw;
        if d_yaw >= std::f64::consts::PI {
            d_yaw -= 2.0 * std::f64::consts::PI;
        }
        if d_yaw <= -std::f64::consts::PI {
            d_yaw += 2.0 * std::f64::consts::PI;
        }

        let rate_cap = if d_yaw >= 0.0 {
            YAW_DOT_MAX
        } else {
            -YAW_DOT_MAX
        };
        let accel_cap = if d_yaw >= 0.0 {
            YAW_DOT_DOT_MAX
        } else {
            -YAW_DOT_DOT_MAX
        };
        let d_yaw_max = if (self.last_yaw_dot + dt * accel_cap).abs() <= rate_cap.abs() {
            // 加速段可达：匀加速位移剖面
            self.last_yaw_dot * dt + 0.5 * accel_cap * dt * dt
        } else {
            // 本帧内先匀加速到限值再匀速：梯形面积
            let t1 = (rate_cap - self.last_yaw_dot) / accel_cap;
            ((dt - t1) + dt) * (rate_cap - self.last_yaw_dot) / 2.0
        };
        if d_yaw.abs() > d_yaw_max.abs() {
            d_yaw = d_yaw_max;
        }
        let yaw_dot = d_yaw / dt;
        let mut yaw = self.last_yaw + d_yaw;
        if yaw > std::f64::consts::PI {
            yaw -= 2.0 * std::f64::consts::PI;
        }
        if yaw < -std::f64::consts::PI {
            yaw += 2.0 * std::f64::consts::PI;
        }
        self.last_yaw = yaw;
        self.last_yaw_dot = yaw_dot;
        (yaw, yaw_dot)
    }

    /// 地面高度测量（对照官方 `ego_replan_fsm::measureGroundHeight`）：沿
    /// 当前轨迹取前方 [`GROUND_LOOKAHEAD_DIST`] 米处（`2.0/max_vel` 秒后）
    /// 的采样点，按地图分辨率逐格下降扫描，首个占据体素的 z 即地面高度；
    /// 扫描出图底（官方 `getOccupancy == -1`）仍无占据则失败。无可执行
    /// 轨迹或前方点超出轨迹时长不测量。
    #[must_use]
    fn measure_ground_height(&self, now: f64) -> Option<f64> {
        let local = self.local.as_ref()?;
        let max_vel = self.planner.config().max_velocity;
        let traj_t = (now - local.start_time) + GROUND_LOOKAHEAD_DIST / max_vel;
        if traj_t > local.traj.duration() {
            return None;
        }
        let mut p = local.traj.eval(traj_t).position;
        let map = self.planner.map_ref();
        let reso = map.resolution();
        loop {
            map.index_of(p)?; // 出界即图底：无地面（官方 `getOccupancy == -1`）
            if map.is_occupied(p) {
                return Some(p.z);
            }
            p.z -= reso;
        }
    }

    /// 深度丢失急停入口（官方 checkCollisionCallback 的 `getOdomDepthTimeout`
    /// 分支）：先关闭 fail-safe 再进入急停——此后不自动恢复，恢复须由
    /// 外部处置后重新走碰撞监控/重规划流程。
    pub fn trigger_emergency_stop_disable_failsafe(&mut self, now: f64, measured: Option<State>) {
        self.failsafe_enabled = false;
        self.enter_emergency_stop(now, measured);
    }

    /// 进入急停（官方 `changeFSMExecState(EMERGENCY_STOP)` + 首轮
    /// `callEmergencyStop(odom_pos_)`）：按当前位置生成停车轨迹并入库为
    /// 可执行轨迹，参考输出变为原地定点悬停。幂等：已在急停中不再重复
    /// 生成（对照 `flag_escape_emergency_` 的防重复调用语义）。
    /// 停车轨迹入库失败（采样退化）仍置急停态，保持旧轨迹悬停等待恢复。
    fn enter_emergency_stop(&mut self, now: f64, measured: Option<State>) {
        if self.emergency {
            return;
        }
        let stop_pos = self.estimated_position(now, measured);
        match emergency_stop_traj(stop_pos) {
            Ok(traj) => {
                // 官方 EmergencyStop 经 setLocalTrajFromOpt(stopMJO, false)：
                // 停车轨迹同样生成检查点入库，touch_goal 取 false
                if !self.set_local_traj(traj, now, false) {
                    log::warn!("急停轨迹检查点生成失败，保持原轨迹");
                }
            }
            Err(e) => log::warn!("急停轨迹构造失败：{e}"),
        }
        self.emergency = true;
        log::warn!(
            "进入急停 @ ({:.2},{:.2},{:.2})，fail-safe {}",
            stop_pos.x,
            stop_pos.y,
            stop_pos.z,
            if self.failsafe_enabled {
                "允许"
            } else {
                "禁止"
            }
        );
    }

    /// 急停恢复（对照官方 `EMERGENCY_STOP` → `GEN_NEW_TRAJ` / `planFromGlobalTraj`）:
    /// 从当前测量位姿沿全局轨迹完整重规划（冷启动），成功即退出急停；
    /// 失败保持急停，下一 tick 重试（官方同态重入语义）。返回是否退出。
    #[must_use]
    fn recover_from_global_traj(&mut self, now: f64, start: State) -> bool {
        let horizon = self.horizon(start.position.coords);
        match self.planner.plan_with_init(
            start,
            horizon.target,
            InitSource::ColdStart,
            horizon.touch_goal,
        ) {
            Ok(result) => {
                // 同 replan 的退化防护：异常短轨迹会触发逐 tick 空转
                if result.trajectory.duration() < 0.5 {
                    log::warn!(
                        "急停恢复产出退化轨迹（时长 {:.2}s），保持急停",
                        result.trajectory.duration()
                    );
                    return false;
                }
                if !self.adopt_plan(result, now, horizon.touch_goal) {
                    log::warn!("急停恢复轨迹入库失败，保持急停");
                    return false;
                }
                self.replans += 1;
                self.replan_fail_streak = 0;
                self.emergency = false;
                log::info!(
                    "急停恢复：从 ({:.2},{:.2},{:.2}) 重规划至 ({:.2},{:.2},{:.2}){}",
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
                true
            }
            Err(e) => {
                log::warn!("急停恢复重规划失败：{e}，保持急停");
                false
            }
        }
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
    /// （物理移动解开几何死锁，也给 VIO 提供视差）。急停态不走脱困分支：
    /// 停车轨迹耗尽后按末端定点输出悬停参考。
    fn reference(&mut self, now: f64, measured: Option<State>) -> Option<Reference> {
        let local = self.local.as_ref()?;
        let t_cur = (now - local.start_time).clamp(0.0, local.traj.duration());
        if !self.emergency
            && t_cur >= local.traj.duration() - 1e-6
            && self.replan_fail_streak >= FAIL_STREAK_ESCAPE
        {
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
            // 脱困直飞无轨迹可前视：yaw 对准脱困方向（同前视朝向运动方向的语义）
            let (yaw, yaw_dot) = self.update_yaw(pos, escape.target.coords, now);
            return Some(Reference {
                position: escape.target.coords,
                velocity: ESCAPE_SPEED * dir,
                yaw,
                yaw_dot,
            });
        }
        let s = local.traj.eval(t_cur);
        // 前视方向（官方 calculate_yaw：t_cur+TIME_FORWARD 未出轨迹则取该点，
        // 否则取轨迹末端）
        let t_look = t_cur + TIME_FORWARD;
        let look_pos = if t_look <= local.traj.duration() {
            local.traj.eval(t_look).position
        } else {
            local.traj.eval(local.traj.duration()).position
        };
        let (yaw, yaw_dot) = self.update_yaw(s.position, look_pos, now);
        Some(Reference {
            position: s.position,
            velocity: s.velocity,
            yaw,
            yaw_dot,
        })
    }
}

/// 官方 `EGOPlannerManager::EmergencyStop`：head/tail 边界状态全同
/// `[stop_pos, 0, 0]`、中间点即 `stop_pos` 本身、两段各 [`STOP_PIECE_DURATION`]——
/// 全部约束相等 ⇒ 多项式恒为常值，参考跟踪自然刹车悬停。
fn emergency_stop_traj(stop_pos: Vector3<f64>) -> Result<Trajectory> {
    let endpoint = Endpoint {
        position: stop_pos,
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let minco = MincoBuilder::new(SolverOrder::MinimumJerk, endpoint, endpoint).build(
        &[nalgebra::Point3::from(stop_pos)],
        &[STOP_PIECE_DURATION, STOP_PIECE_DURATION],
    )?;
    minco.solve()
}

/// 官方 `poly_traj::Trajectory::locatePieceIdx` 的段号部分：`t` 所在段
/// 下标（越界夹紧到首/末段；官方还会把 `t` 局部化到段内，此处只要段号）。
fn piece_index_at(traj: &Trajectory, t: f64) -> usize {
    let durations = traj.durations();
    let mut idx = 0;
    let mut remaining = t;
    while idx < durations.len() && remaining > durations[idx] {
        remaining -= durations[idx];
        idx += 1;
    }
    if idx == durations.len() {
        durations.len() - 1
    } else {
        idx
    }
}

/// 路径累计长度。
fn path_length(points: &[Vector3<f64>]) -> f64 {
    points.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
}

/// 折线各顶点距起点的累计弧长（与 `points` 等长）。
fn cumulative_arcs(points: &[Vector3<f64>]) -> Vec<f64> {
    let mut cum = Vec::with_capacity(points.len());
    let mut acc = 0.0;
    for w in points.windows(2) {
        cum.push(acc);
        acc += (w[1] - w[0]).norm();
    }
    cum.push(acc);
    cum
}

/// 折线上距起点弧长 `s` 的插值点（`s` 超出总长取末端；退化段取段首）。
fn point_at_arc(points: &[Vector3<f64>], cum: &[f64], s: f64) -> Vector3<f64> {
    for i in 1..points.len() {
        if cum[i] >= s {
            let len = cum[i] - cum[i - 1];
            let t = if len < 1e-12 {
                0.0
            } else {
                (s - cum[i - 1]) / len
            };
            return points[i - 1] + (points[i] - points[i - 1]) * t;
        }
    }
    points.last().copied().unwrap_or_default()
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

    #[test]
    fn collision_monitor_triggers_immediate_replan() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        assert!(m.local().is_some(), "前置:初始轨迹存在");
        // 飞行中途向地图注入新障碍(初始直线路径前方 x≈4,res 0.5 → 体素 [8,2,2],
        // 膨胀 1 格后 x∈[3.5,5.0),稠密采样间距 ≤0.63m 必命中)
        m.map_mut().set_state([8, 2, 2], VoxelState::Occupied);
        assert!(
            !m.map().is_occupied_inflated(Vector3::new(4.2, 1.0, 1.0)),
            "膨胀层刷新前新障碍不可见"
        );
        // t=0.5 < replan_thresh(1.0):只有碰撞监控能触发重规划
        let r = m.tick(0.5, None);
        assert!(r.collision, "监控应发现前方碰撞");
        assert!(r.replanned, "发现碰撞应绕过 replan_thresh/冷却期立即重规划");
        assert_eq!(m.replans(), 1);
        assert!(!r.emergency_stop, "重规划成功不应请求急停");
        let local = m.local().expect("碰撞重规划后应有新轨迹");
        assert!(
            (local.start_time - 0.5).abs() < 1e-9,
            "新轨迹从本 tick 起飞"
        );
        assert!(
            !local.points_to_check.is_empty(),
            "入库轨迹必带检查点(监控覆盖不变量)"
        );
    }

    #[test]
    fn collision_monitor_ignores_already_flown_part() {
        // 阈值拉高隔离阈值重规划,聚焦监控行为
        let map = GridMapBuilder::new(0.5, [40, 24, 16]).build().unwrap();
        let planner = Planner::new(PlannerConfig::default(), map);
        let options = ManagerOptions {
            replan_thresh: 100.0,
            ..ManagerOptions::default()
        };
        let mut m = PlannerManager::with_planner(
            planner,
            options,
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(10.0, 1.0, 1.0),
        )
        .unwrap();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));

        // 已飞过位置(t=0.5 时所在)注入障碍;t=2.0 时机体已远离
        let past = m.local().unwrap().traj.eval(0.5).position;
        let now_pos = m.local().unwrap().traj.eval(2.0).position;
        assert!(
            past.x < now_pos.x - 0.5,
            "场景前提:t=2.0 应已飞过 t=0.5 位置"
        );
        let idx = m.map().index_of(past).expect("身后点在地图内");
        m.map_mut().set_state(idx, VoxelState::Occupied);
        m.map_mut().inflate_obstacles(0.2);
        assert!(
            m.map().is_occupied_inflated(past),
            "场景前提:身后障碍真实存在(防空洞断言)"
        );

        let r = m.tick(2.0, None);
        assert!(!r.collision, "已飞过部分的障碍不得误报");
        assert!(!r.replanned, "无误报则不应重规划");
        assert_eq!(m.replans(), 0);
    }

    /// 沿 +y 方向直飞的管理器（起点 (1,1,1)，终点 (1,10,1)）：航向恒为 π/2。
    fn northbound_manager() -> PlannerManager {
        let map = GridMapBuilder::new(0.5, [24, 40, 16]).build().unwrap();
        let planner = Planner::new(PlannerConfig::default(), map);
        PlannerManager::with_planner(
            planner,
            ManagerOptions::default(),
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(1.0, 10.0, 1.0),
        )
        .unwrap()
    }

    #[test]
    fn yaw_tracks_straight_line_heading() {
        let mut m = northbound_manager();
        // 首帧仅初始化 yaw 状态（官方首帧行为：last_yaw_=0 起）
        let r0 = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        let ref0 = r0.reference.expect("首帧应有参考指令");
        assert!(ref0.yaw.abs() < 1e-12);
        assert!(ref0.yaw_dot.abs() < 1e-12);
        // 第二帧（低于重规划阈值，不扰动轨迹）：前视方向沿 +y → yaw = π/2
        let r1 = m.tick(0.5, None);
        let ref1 = r1.reference.expect("第二帧应有参考指令");
        assert!(
            (ref1.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-6,
            "直线轨迹 yaw 应为航向角 π/2，实际 {}",
            ref1.yaw
        );
        // 首帧 yaw 从初值 0 出发，本帧一步内完成对准（dt=0.5 的限幅剖面
        // 容许 π/2），角速度为实际变化率且不超限幅
        assert!(
            ref1.yaw_dot.abs() <= YAW_DOT_MAX + 1e-9,
            "角速度超限幅: {}",
            ref1.yaw_dot
        );
        // 第三帧已对准航向：yaw 保持 π/2，角速度归零
        let r2 = m.tick(1.0, None);
        let ref2 = r2.reference.expect("第三帧应有参考指令");
        assert!((ref2.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
        assert!(ref2.yaw_dot.abs() < 1e-9, "已对准航向后角速度应≈0");
        // 包装不变量：yaw 始终在 [-π,π]
        assert!(ref1.yaw.abs() <= std::f64::consts::PI + 1e-12);
    }

    #[test]
    fn yaw_wraps_across_pi_without_jumping() {
        let mut m = open_manager();
        // 上一帧 yaw=3.0，期望航向 -3.0：原始差 -6.0，包装后应沿 +0.283 方向
        // 小步推进（穿过 ±π 边界），而不是反向跳转到 -3.0
        m.last_yaw = 3.0;
        m.last_yaw_dot = 0.0;
        m.last_yaw_time = Some(0.0);
        let pos = Vector3::zeros();
        let target = Vector3::new((-3.0f64).cos() * 5.0, (-3.0f64).sin() * 5.0, 0.0);
        let (yaw, yaw_dot) = m.update_yaw(pos, target, 0.1);
        // dt=0.1、零初速：梯形剖面首步位移 = 0.5·(5π)·dt² = 0.0785398...
        let expect_step = 0.5 * YAW_DOT_DOT_MAX * 0.01;
        assert!(
            (yaw - (3.0 + expect_step)).abs() < 1e-9,
            "yaw 应从 3.0 沿正向小步推进到 {}，实际 {yaw}",
            3.0 + expect_step
        );
        assert!((yaw_dot - expect_step / 0.1).abs() < 1e-9);
        assert!((m.last_yaw - yaw).abs() < 1e-12, "状态应同步更新");
    }

    #[test]
    fn yaw_rate_capped_under_large_turn() {
        let mut m = open_manager();
        // 大转向：期望航向与当前 yaw 相差 π，逐步积分整条限幅剖面
        m.last_yaw = 0.0;
        m.last_yaw_dot = 0.0;
        m.last_yaw_time = Some(0.0);
        let pos = Vector3::zeros();
        let target = Vector3::new(-5.0, 0.0, 0.0); // 航向 π（与 0 反向）
        let mut now = 0.0;
        let mut converged = false;
        for _ in 0..400 {
            now += 0.02;
            let (yaw, yaw_dot) = m.update_yaw(pos, target, now);
            assert!(
                yaw_dot.abs() <= YAW_DOT_MAX + 1e-9,
                "|yaw_dot|={} 超出限幅 {}",
                yaw_dot.abs(),
                YAW_DOT_MAX
            );
            assert!(yaw.abs() <= std::f64::consts::PI + 1e-12, "包装越界: {yaw}");
            // 到达期望航向（±π 同义）即收敛
            if (yaw.abs() - std::f64::consts::PI).abs() < 1e-3 {
                converged = true;
                break;
            }
        }
        assert!(converged, "限幅剖面应在有限时间内转过 π");
    }

    #[test]
    fn yaw_holds_last_on_deadzone() {
        let mut m = open_manager();
        m.last_yaw = 0.7;
        m.last_yaw_dot = -0.3;
        m.last_yaw_time = Some(1.0);
        // 前视方向模长 ≤ 0.1（悬停/末端）：保持 last_yaw，角速度归零
        let pos = Vector3::new(2.0, 2.0, 1.0);
        let target = pos + Vector3::new(0.05, 0.0, 0.0);
        let (yaw, yaw_dot) = m.update_yaw(pos, target, 1.1);
        assert!((yaw - 0.7).abs() < 1e-12, "死区应保持 last_yaw，实际 {yaw}");
        assert!(yaw_dot.abs() < 1e-12);
        let state = m.yaw_state();
        assert!((state.0 - 0.7).abs() < 1e-12 && state.1.abs() < 1e-12);
        // 首帧初始化：不计算方向，保持初值 (0, 0)
        let mut m2 = open_manager();
        let (yaw2, dot2) = m2.update_yaw(pos, target, 5.0);
        assert!(yaw2.abs() < 1e-12 && dot2.abs() < 1e-12);
    }

    #[test]
    fn ground_height_measures_flat_floor() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        // 平地：整个底层体素层（z∈[0,0.5)）填满占据
        let dims = m.map().dims();
        for i in 0..dims[0] {
            for j in 0..dims[1] {
                m.map_mut().set_state([i, j, 0], VoxelState::Occupied);
            }
        }
        let h = m
            .measure_ground_height(m.local().unwrap().start_time)
            .expect("平地应测得地面高度");
        assert!(
            (0.0..0.5).contains(&h),
            "测得高度应落在地面体素层内，实际 {h}",
        );
        // 首个命中不变量：命中体素占据，其上一层自由
        let fwd = m
            .local()
            .unwrap()
            .traj
            .eval(GROUND_LOOKAHEAD_DIST / 1.5)
            .position;
        assert!(m.map().is_occupied(Vector3::new(fwd.x, fwd.y, h)));
        assert!(!m.map().is_occupied(Vector3::new(fwd.x, fwd.y, h + 0.5)));
    }

    #[test]
    fn ground_height_fails_at_map_bottom() {
        let mut m = open_manager();
        // 空旷地图（无地面）：逐格下降扫描至图底仍无占据 → 测量失败
        let r = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        assert!(m.local().is_some(), "前置：轨迹已存在");
        assert!(r.ground_height.is_none(), "图底之下无地面应返回失败");
        assert!(m.measure_ground_height(0.0).is_none());
    }

    #[test]
    fn ground_height_skipped_without_plan() {
        let m = open_manager();
        // 无可执行轨迹（官方 pts_chk < 3 的前置条件）不测量
        assert!(m.measure_ground_height(0.0).is_none());
    }

    #[test]
    fn emergency_stop_generates_stationary_reference() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        let stop_pos = Vector3::new(2.0, 1.0, 1.0);
        m.enter_emergency_stop(0.5, Some(state_at(stop_pos)));
        assert!(m.emergency, "进入后应处于急停态");
        // 停车轨迹：两段各 STOP_PIECE_DURATION，全程定点零速零加速度
        // （官方 EmergencyStop 的边界状态全同约束 ⇒ 恒为常值多项式）
        let local = m.local().expect("急停应入库停车轨迹");
        assert!((local.traj.duration() - 2.0 * STOP_PIECE_DURATION).abs() < 1e-9);
        for t in [0.0, 0.7, 1.3, 2.0] {
            let s = local.traj.eval(t);
            assert!((s.position - stop_pos).norm() < 1e-9, "t={t} 应停在原点");
            assert!(s.velocity.norm() < 1e-9, "t={t} 参考速度应为零");
            assert!(s.acceleration.norm() < 1e-9, "t={t} 参考加速度应为零");
        }
        // 急停中的 tick：报告急停态，参考为原地悬停
        let r = m.tick(1.0, None);
        assert!(r.emergency_stop, "急停中报告应急停标志");
        let reference = r.reference.expect("急停态应有悬停参考");
        assert!((reference.position - stop_pos).norm() < 1e-6);
        assert!(reference.velocity.norm() < 1e-9);
    }

    #[test]
    fn emergency_stop_survives_trajectory_exhaustion() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        let stop_pos = Vector3::new(2.0, 1.0, 1.0);
        m.enter_emergency_stop(0.5, Some(state_at(stop_pos)));
        let replans_before = m.replans();
        // 远超停车轨迹时长（2s）：不得触发常规强制重规划或脱困直飞
        // （对照官方：重规划仅在 EXEC_TRAJ case 生效）
        let r = m.tick(10.0, None);
        assert!(m.emergency, "轨迹耗尽不得退出急停");
        assert!(!r.replanned, "急停中轨迹耗尽不得常规重规划");
        assert_eq!(m.replans(), replans_before);
        let reference = r.reference.expect("耗尽后仍应有悬停参考");
        assert!((reference.position - stop_pos).norm() < 1e-6);
        assert!(reference.velocity.norm() < 1e-9);
    }

    #[test]
    fn emergency_recovers_when_failsafe_enabled_and_slow() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        let stop_pos = Vector3::new(2.0, 1.0, 1.0);
        m.enter_emergency_stop(0.5, Some(state_at(stop_pos)));
        // 超阈值速度：即使 fail-safe 允许也不恢复（官方 vel < 0.1 判据）
        let mut fast = state_at(stop_pos);
        fast.velocity = Vector3::new(1.0, 0.0, 0.0);
        let r_fast = m.tick(0.8, Some(fast));
        assert!(m.emergency, "超阈值速度不应恢复");
        assert!(!r_fast.replanned);
        // 低速观测 + fail-safe 允许（默认）→ 从当前位姿完整重规划并退出
        let r = m.tick(1.0, Some(state_at(stop_pos)));
        assert!(!m.emergency, "低速且 fail-safe 允许应退出急停");
        assert!(r.replanned, "恢复应产出新轨迹");
        assert!(!r.emergency_stop);
        let local = m.local().expect("恢复后应有新轨迹");
        assert!(
            (local.start_time - 1.0).abs() < 1e-9,
            "新轨迹应在恢复 tick 入库"
        );
    }

    #[test]
    fn in_collision_goal_modified_to_nearest_free_point() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        let old_path = m.global_path().to_vec();
        let old_goal = m.goal();
        // 目标附近整块占据（res 0.5，goal=(10,1,1) → 体素 [20,2,2] 及邻域），
        // 使回扫从末端起至少跨过一个步长才碰到自由点
        for i in 18..=21 {
            for j in 1..=3 {
                for k in 1..=3 {
                    m.map_mut().set_state([i, j, k], VoxelState::Occupied);
                }
            }
        }
        m.map_mut().inflate_obstacles(0.2);
        assert!(
            m.map().is_occupied_inflated(old_goal.coords),
            "场景前提：目标应落在膨胀占据内"
        );
        // t=0.5 < replan_thresh：本 tick 只有目标修正分支能动作
        let r = m.tick(0.5, None);
        assert!(r.replanned, "目标修正应本 tick 立即重规划");
        let new_goal = m.goal();
        assert!(
            (new_goal.coords - old_goal.coords).norm() > 1e-6,
            "目标应被修正，实际仍为 {new_goal:?}"
        );
        assert!(
            !m.map().is_occupied_inflated(new_goal.coords),
            "修正后的目标必须无碰"
        );
        // 修正点 = 沿旧全局路径末端回扫的首个自由点：用测试预言机独立复算期望值
        let reso = m.map().resolution();
        let mut expected = None;
        let mut s = path_length(&old_path);
        while s > 0.0 {
            let pt = arc_point(&old_path, old_path[0], s);
            if !m.map().is_occupied_inflated(pt) {
                expected = Some(pt);
                break;
            }
            s -= reso;
        }
        let expected = expected.expect("场景前提：回扫必能找到自由点");
        assert!(
            (new_goal.coords - expected).norm() < 1e-6,
            "修正点应为末端回扫首个自由点：期望 {expected:?}，实际 {new_goal:?}"
        );
        assert!(!r.finished);
        assert!(!r.emergency_stop);
    }

    #[test]
    fn free_goal_left_untouched() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        let goal = m.goal();
        let path = m.global_path().to_vec();
        // 目标自由且低于重规划阈值：目标与轨迹均不变
        let r = m.tick(0.5, None);
        assert!(!r.replanned);
        assert_eq!(m.goal(), goal);
        assert_eq!(m.global_path(), path.as_slice());
        assert_eq!(m.replans(), 0);
    }

    #[test]
    fn fully_blocked_global_path_keeps_goal_without_panic() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        let goal = m.goal();
        let path = m.global_path().to_vec();
        // 整条直线路径占据：路径 y=z=1，占据 y=z∈[1.0,1.5) 的体素条带即覆盖全程
        let dims = m.map().dims();
        for i in 0..dims[0] {
            m.map_mut().set_state([i, 2, 2], VoxelState::Occupied);
        }
        m.map_mut().inflate_obstacles(0.2);
        for p in &path {
            assert!(m.map().is_occupied_inflated(*p), "场景前提：{p:?} 应被占据");
        }
        // 回扫耗尽：不 panic，目标与轨迹保持不变（后续碰撞监控/急停接管）
        let r = m.tick(0.5, None);
        assert_eq!(m.goal(), goal, "回扫耗尽目标不得变动");
        assert_eq!(m.global_path(), path.as_slice());
        assert!(!r.finished);
    }

    #[test]
    fn emergency_without_failsafe_never_recovers() {
        let mut m = open_manager();
        let _ = m.tick(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
        let stop_pos = Vector3::new(2.0, 1.0, 1.0);
        m.trigger_emergency_stop_disable_failsafe(0.5, Some(state_at(stop_pos)));
        assert!(m.emergency, "深度丢失入口应进入急停");
        // fail-safe 已关闭：即使完全静止也不自动恢复（官方语义）
        let r = m.tick(1.0, Some(state_at(stop_pos)));
        assert!(m.emergency, "fail-safe 关闭不得自动恢复");
        assert!(r.emergency_stop);
        assert!(!r.replanned);
    }
}
