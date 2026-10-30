//! 规划管理：全局引导、局部目标、初始化、优化调用及轨迹入库。

use firefly_error::Result;
use firefly_map::GridMap;
use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder, Trajectory};
use nalgebra::{Point3, Vector3};
use rand::RngExt;

use crate::obstacles::{ObstacleScanner, PointsToCheck};
use crate::planner::{InitSource, PlanResult, Planner, State};

/// 重规划触发阈值（秒，官方 `fsm/thresh_replan_time`）。
pub const DEFAULT_REPLAN_THRESH: f64 = 1.0;
/// 急停时限（秒，官方 `fsm/emergency_time`）：同步 FSM 先尝试重规划；异步
/// 安全回调在剩余碰撞时间小于该值时直接急停，并以此限制求解任务的年龄。
pub const DEFAULT_EMERGENCY_TIME: f64 = 1.0;
/// 急停恢复速度阈值（m/s）：fail-safe 允许且测量速度低于该值才从急停
/// 恢复重规划（官方 `EMERGENCY_STOP` case 的 0.1）。
pub const DEFAULT_EMERGENCY_RECOVER_SPEED: f64 = 0.1;
/// 停车轨迹单段时长（秒，官方 `EmergencyStop` 的两段各 1.0）。
const STOP_PIECE_DURATION: f64 = 1.0;
/// 规划视界（米，官方 `manager/planning_horizon`）。
pub const DEFAULT_PLANNING_HORIZON: f64 = 6.0;
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
            emergency_time: DEFAULT_EMERGENCY_TIME,
            emergency_recover_speed: DEFAULT_EMERGENCY_RECOVER_SPEED,
        }
    }
}

/// 执行中的局部轨迹（官方 `LocalTrajData`：轨迹 + 起始时刻 + 检查点）。
#[derive(Debug, Clone)]
pub struct LocalTraj {
    pub traj: Trajectory,
    /// 轨迹起始时刻（管理器时钟，秒）；检查点/监控的时间原点。
    pub start_time: f64,
    /// 带时间戳检查点（官方 `PtsChk_t`，规划成功后生成一次）：执行期
    /// 碰撞监控的扫描源。不变量：非空（采样退化的轨迹不入执行，
    /// 见 [`PlannerManager::adopt_plan`]）。
    pub points_to_check: PointsToCheck,
    /// 轨迹序号（官方 `LocalTrajData::traj_id`，见 `plan_container.hpp:36`）：
    /// 由 [`PlannerManager::adopt_plan`] 入库时写入管理器计数器当前值；
    /// 广播时随轨迹携带，接收方据此丢弃过期包。
    pub traj_id: u32,
}

/// 已登记的其他机轨迹（官方 `swarm_traj` 槽位元素）。
#[derive(Debug, Clone)]
pub(crate) struct SwarmTraj {
    pub(crate) traj: Trajectory,
    /// 轨迹起始时刻（管理器时钟，秒）：预测对齐用（官方
    /// `t_X = t + (my_start_time - other_start_time)`）。
    pub(crate) start_time: f64,
    /// 该机期望净距（米，官方 `des_clearance`）：检查阈值 =
    /// `swarm_clearance + des_clearance`。
    pub(crate) des_clearance: f64,
}

/// 局部目标（官方 `getLocalTarget` 输出，`planner_manager.cpp:327`：位置 +
/// 目标速度 + 是否触及全局终点）。
pub(crate) struct Horizon {
    /// 局部目标位置（地图系，米）：全局轨迹上距当前位置 `planning_horizon`
    /// 的等时步进点，或全局终点。
    pub(crate) target: Vector3<f64>,
    /// 局部目标处期望速度（地图系，m/s）：距全局终点 < 刹车距离
    /// `max_vel²/(2·max_acc)` 时取零速，否则取全局轨迹在该时刻的速度。
    pub(crate) target_vel: Vector3<f64>,
    /// 局部目标是否即全局终点（官方 `touch_goal`，影响约束采样是否覆盖
    /// 全程/碰撞监控扫描上界）。
    pub(crate) touch_goal: bool,
}
impl Horizon {
    pub(crate) fn endpoint(&self) -> Endpoint {
        Endpoint {
            position: self.target,
            velocity: self.target_vel,
            acceleration: Vector3::zeros(),
        }
    }
}

/// 全局轨迹与局部目标时间簿记（官方 `GlobalTrajData`，见
/// `plan_container.hpp:20-31`）：MINCO 时间参数化轨迹 + 管理器时钟锚点 +
/// 当前/上一局部目标对应的全局轨迹时间。
#[derive(Clone)]
pub(crate) struct GlobalTraj {
    /// MINCO 时间参数化轨迹（官方 `planGlobalTrajWaypoints` 产出）：
    /// `eval(t)` 的 t ∈ [0, `duration`]，t=0 起点、t=duration 终点。
    pub(crate) traj: Trajectory,
    /// 轨迹锚点（管理器时钟，秒，官方 `global_start_time`）：
    /// `eval(t - start_time)` 定位到世界时刻 `t`。
    pub(crate) start_time: f64,
    /// 当前局部目标对应的全局轨迹时间（管理器时钟，秒，官方
    /// `glb_t_of_lc_tgt`）：局部目标选取的步进游标。
    pub(crate) glb_t_of_lc_tgt: f64,
    /// 上一局部目标对应的全局轨迹时间（管理器时钟，秒，官方
    /// `last_glb_t_of_lc_tgt`）：暖启动内点拼接的段起点。
    pub(crate) last_glb_t_of_lc_tgt: f64,
}

impl GlobalTraj {
    /// 官方 `setGlobalTraj`（`plan_container.hpp:62`）：锚定世界时并重置
    /// 局部目标时间簿记（`glb = world_time`、`last = -1.0`）。
    fn new(traj: Trajectory, world_time: f64) -> Self {
        Self {
            traj,
            start_time: world_time,
            glb_t_of_lc_tgt: world_time,
            last_glb_t_of_lc_tgt: -1.0,
        }
    }
}

pub struct PlannerManager {
    pub(crate) planner: Planner,
    pub(crate) options: ManagerOptions,
    pub(crate) global_path: Vec<Vector3<f64>>,
    pub(crate) global: GlobalTraj,
    pub(crate) goal: Point3<f64>,
    pub(crate) local: Option<LocalTraj>,
    pub(crate) traj_id: u32,
    pub(crate) last_result: Option<PlanResult>,
    pub(crate) touch_goal: bool,
    pub(crate) replans: usize,
    pub(crate) swarm: Vec<Option<SwarmTraj>>,
    pub(crate) have_recv_pre_agent: bool,
    pub(crate) consecutive_failures: u32,
}

impl PlannerManager {
    pub(crate) fn snapshot(&self) -> Self {
        Self {
            planner: self.planner.snapshot(),
            options: self.options,
            global_path: self.global_path.clone(),
            global: self.global.clone(),
            goal: self.goal,
            local: self.local.clone(),
            traj_id: self.traj_id,
            last_result: None,
            touch_goal: self.touch_goal,
            replans: self.replans,
            swarm: self.swarm.clone(),
            have_recv_pre_agent: self.have_recv_pre_agent,
            consecutive_failures: self.consecutive_failures,
        }
    }
    /// 实际构造入口：给定已构建的 [`Planner`]（应用层负责地图来源）。
    ///
    /// # Errors
    ///
    /// 任务航点不在地图内或全局轨迹构造失败。
    pub fn with_planner(
        planner: Planner,
        options: ManagerOptions,
        start: Vector3<f64>,
        goal: Vector3<f64>,
    ) -> Result<Self> {
        let global_path = global_waypoints(start, goal)?;
        let max_vel = planner.config().max_velocity;
        let global_traj = plan_global_traj_waypoints(start, &global_path, max_vel)?;
        // 初始全局轨迹锚点取管理器时钟原点（0.0）：glb_t 簿记是相对游标，
        // 与绝对时钟无关（官方 setGlobalTraj 以规划时刻锚定，此处等义）
        let global = GlobalTraj::new(global_traj, 0.0);
        // drone_id ≤ 0（单机/0 号机）无需顺序起飞等待（官方 SEQUENTIAL_START 短路）
        let have_recv_pre_agent = planner.config().drone_id <= 0;
        Ok(Self {
            planner,
            options,
            global_path,
            global,
            goal: Point3::from(goal),
            local: None,
            traj_id: 0,
            last_result: None,
            touch_goal: false,
            replans: 0,
            swarm: Vec::new(),
            // 官方 init：have_recv_pre_agent_ = false；drone_id ≤ 0 的检查恒
            // 短路为真，直接置 true 等价
            have_recv_pre_agent,
            consecutive_failures: 0,
        })
    }
    /// # Errors
    /// 目标非法或全局轨迹生成失败。
    pub fn set_goal(
        &mut self,
        now: f64,
        measured: Option<State>,
        goal: Vector3<f64>,
    ) -> Result<()> {
        // 重复目标视为幂等操作，保留当前执行状态。
        if (goal - self.goal.coords).norm() < 1e-6 {
            return Ok(());
        }
        let start = self.estimated_position(now, measured);
        let global_path = global_waypoints(start, goal)?;
        let max_vel = self.planner.config().max_velocity;
        let global_traj = plan_global_traj_waypoints(start, &global_path, max_vel)?;
        log::info!(
            "目标更新为 ({:.1},{:.1},{:.1})：全局引导 {} 点，长度 {:.1}m",
            goal.x,
            goal.y,
            goal.z,
            global_path.len(),
            path_length(&global_path)
        );
        self.global_path = global_path;
        // 官方 setGlobalTraj：全局轨迹重建即重置 glb_t 簿记（glb = now，last = -1）
        self.global = GlobalTraj::new(global_traj, now);
        self.goal = Point3::from(goal);
        // 清除当前轨迹与到达标记，使下一 tick 为新目标执行初始规划。
        self.touch_goal = false;
        self.replans = 0;
        Ok(())
    }

    /// 坐标修正使执行轨迹失效；地图系任务目标保持固定，锁存停止状态保留。
    /// # Errors
    /// 非有限位置或全局引导轨迹构造失败。
    pub fn invalidate_localization(&mut self, now: f64, measured: State) -> Result<()> {
        self.local = None;
        self.last_result = None;
        self.touch_goal = false;
        let start = measured.position.coords;
        let path = global_waypoints(start, self.goal.coords)?;
        let trajectory =
            plan_global_traj_waypoints(start, &path, self.planner.config().max_velocity)?;
        self.global_path = path;
        self.global = GlobalTraj::new(trajectory, now);
        Ok(())
    }

    /// 只读地图访问。
    /// 对照 FSM planFromGlobalTraj：同一状态调用内采用相同初始化模式。
    pub(crate) fn plan_from_global_traj(
        &mut self,
        now: f64,
        measured: State,
        trials: usize,
        random: bool,
    ) -> bool {
        let start = State {
            acceleration: Vector3::zeros(),
            ..measured
        };
        (0..trials).any(|_| self.call_rebound_replan(now, start, true, random))
    }

    /// 对照 planFromLocalTraj：暖启动、确定性多项式、随机多项式。
    pub(crate) fn plan_from_local_traj(
        &mut self,
        now: f64,
        trials: usize,
        new_target: bool,
    ) -> bool {
        let Some(local) = &self.local else {
            return false;
        };
        let sample = local.traj.eval(now - local.start_time);
        let start = State {
            position: Point3::from(sample.position),
            velocity: sample.velocity,
            acceleration: sample.acceleration,
        };
        self.replans += 1;
        if self.call_rebound_replan(now, start, new_target, false) {
            return true;
        }
        if self.call_rebound_replan(now, start, true, false) {
            return true;
        }
        (0..trials).any(|_| self.call_rebound_replan(now, start, true, true))
    }

    fn call_rebound_replan(
        &mut self,
        now: f64,
        start: State,
        poly_init: bool,
        random: bool,
    ) -> bool {
        if self.planner.cancelled() {
            return false;
        }
        let horizon = self.horizon(start.position.coords);
        let peers = self.peers_at(now);
        let previous = self
            .local
            .as_ref()
            .map(|local| (local.traj.clone(), now - local.start_time));
        let source = if random {
            InitSource::RandomStart {
                mid: self.random_mid(start.position.coords, horizon.target, &mut rand::rng()),
            }
        } else if !poly_init && let Some((previous, elapsed)) = &previous {
            InitSource::WarmStart {
                prev: previous,
                elapsed: *elapsed,
                global: &self.global.traj,
                global_start: self.global.last_glb_t_of_lc_tgt - self.global.start_time,
                global_end: self.global.glb_t_of_lc_tgt - self.global.start_time,
            }
        } else {
            InitSource::ColdStart
        };
        match self.planner.plan_in_swarm_with_init(
            start,
            horizon.endpoint(),
            &peers,
            source,
            horizon.touch_goal,
        ) {
            Ok(result) => {
                if !self.adopt_plan(result, now, horizon.touch_goal) {
                    return false;
                }
                self.consecutive_failures = 0;
                true
            }
            Err(error) => {
                self.consecutive_failures = self.consecutive_failures.saturating_add(1);
                log::debug!("局部规划失败：{error}");
                false
            }
        }
    }

    pub(crate) fn emergency_stop(&mut self, now: f64, stop_pos: Vector3<f64>) {
        match emergency_stop_traj(stop_pos) {
            Ok(traj) => {
                if self.set_local_traj(traj, now, false, self.traj_id + 1) {
                    self.traj_id += 1;
                }
            }
            Err(error) => log::error!("急停轨迹生成失败：{error}"),
        }
    }

    #[must_use]
    pub fn map(&self) -> &GridMap {
        self.planner.map_ref()
    }

    /// 可变地图访问（深度感知建图/动态障碍写入入口）。
    pub fn map_mut(&mut self) -> &mut GridMap {
        self.planner.map_mut()
    }

    /// 本机编号（官方 `pp_.drone_id`，来自 [`crate::PlannerConfig::drone_id`]）。
    #[must_use]
    pub fn drone_id(&self) -> i32 {
        self.planner.config().drone_id
    }

    /// 对外广播的期望净距（米，官方发包时 `MINCO_msg.des_clearance =
    /// getSwarmClearance()`，见 `ego_replan_fsm.cpp:820`）：即本机
    /// `optimization/swarm_clearance` 参数。对端收到后存入其
    /// `swarm_traj[本机].des_clearance`，作检查阈值
    /// `其对端 swarm_clearance + 本机 des_clearance` 的对端项（非对称：
    /// 各机参数可不同）。
    #[must_use]
    pub fn broadcast_des_clearance(&self) -> f64 {
        self.planner.config().swarm_clearance
    }

    /// 顺序起飞就绪（官方 `SEQUENTIAL_START` 的进入条件）：`drone_id ≤ 0`
    /// 恒就绪；否则须已收齐 `0..drone_id` 全部前序机轨迹。
    #[must_use]
    pub const fn sequential_start_ready(&self) -> bool {
        self.have_recv_pre_agent
    }

    /// 登记其他机轨迹（单进程仿真下由外部注入；对照官方
    /// `RecvBroadcastMINCOTrajCallback` 的存储段）：槽位按 `agent_id` 索引，
    /// 重复登记以最新为准。`des_clearance` 为对端广播的期望净距——调用方
    /// 应以对端管理器 `broadcast_des_clearance()` 的返回值传入（各机参数
    /// 可不同；检查阈值 = 本机 `swarm_clearance + 对端 des_clearance`，
    /// 见官方 `planner_manager.cpp:416`）。同时按官方条件推进顺序起飞就绪位——
    /// `swarm_traj.size() >= drone_id` 且 `0..drone_id` 各槽位全部有效。
    /// 自身槽位（`agent_id == drone_id`）拒收（官方 `recv_id == 自身`直接返回）。
    pub fn register_swarm_traj(
        &mut self,
        agent_id: usize,
        traj: Trajectory,
        start_time: f64,
        des_clearance: f64,
    ) {
        if agent_id as i32 == self.drone_id() {
            return;
        }
        if self.swarm.len() <= agent_id {
            self.swarm.resize_with(agent_id + 1, || None);
        }
        self.swarm[agent_id] = Some(SwarmTraj {
            traj,
            start_time,
            des_clearance,
        });
        // 官方 have_recv_pre_agent_ 推进：一次性锁存，置位后不回退
        let need = usize::try_from(self.drone_id()).unwrap_or(usize::MAX);
        if !self.have_recv_pre_agent
            && need >= 1
            && self.swarm.len() >= need
            && (0..need).all(|i| self.swarm[i].is_some())
        {
            self.have_recv_pre_agent = true;
            log::info!("已收齐前序 agent 0..{need} 轨迹，顺序起飞就绪");
        }
    }

    #[must_use]
    pub fn global_path(&self) -> &[Vector3<f64>] {
        &self.global_path
    }

    /// 可变规划器（队形/集群配置注入，测试与应用层装配用）。
    pub fn planner_mut(&mut self) -> &mut Planner {
        &mut self.planner
    }

    #[must_use]
    pub fn goal(&self) -> Point3<f64> {
        self.goal
    }

    #[must_use]
    pub const fn replans(&self) -> usize {
        self.replans
    }

    /// 当前执行的局部轨迹。
    #[must_use]
    pub fn local(&self) -> Option<&LocalTraj> {
        self.local.as_ref()
    }

    /// 已入库轨迹序号（官方 `TrajContainer::setLocalTraj` 的 `traj_id++` 结果）：
    /// 初始 `0` = 尚无可执行轨迹（官方 `traj_id <= 0` 哨兵，见
    /// `ego_replan_fsm.cpp:307`）；此后每次成功入库新轨迹递增。广播新轨迹时
    /// 随包携带，对端据此丢弃过期包。
    #[must_use]
    pub const fn traj_id(&self) -> u32 {
        self.traj_id
    }

    /// 最近一次成功规划的结果（可视化平面等衍生数据源）。
    #[must_use]
    pub fn last_result(&self) -> Option<&PlanResult> {
        self.last_result.as_ref()
    }
    pub(crate) fn modify_in_collision_final_goal(
        &mut self,
        now: f64,
        measured: Option<State>,
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
        // 任务航点连线，克隆避开后续 &mut 借用冲突
        let path = self.global_path.clone();
        let cum = cumulative_arcs(&path);
        let mut s = cum.last().copied().unwrap_or(0.0);
        while s > 0.0 {
            let pt = point_at_arc(&path, &cum, s);
            if !self.planner.map_ref().is_occupied_inflated(pt) {
                let start = self.estimated_position(now, measured);
                match global_waypoints(start, pt) {
                    Ok(new_path) => {
                        let max_vel = self.planner.config().max_velocity;
                        if let Ok(new_global_traj) =
                            plan_global_traj_waypoints(start, &new_path, max_vel)
                        {
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
                            // 官方 planNextWaypoint：全局轨迹重建即重置 glb_t 簿记
                            self.global = GlobalTraj::new(new_global_traj, now);
                            self.goal = Point3::from(pt);
                            return true;
                        }
                        log::debug!("修正点全局轨迹构造失败，继续回扫");
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
    pub(crate) fn estimated_position(&self, now: f64, measured: Option<State>) -> Vector3<f64> {
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

    pub(crate) fn peers_at(&self, now: f64) -> Vec<firefly_cost::Peer> {
        self.swarm
            .iter()
            .enumerate()
            .filter_map(|(id, entry)| {
                entry.as_ref().map(|peer| {
                    firefly_cost::Peer::new(
                        id,
                        peer.start_time - now,
                        peer.traj.clone(),
                        peer.des_clearance,
                    )
                })
            })
            .collect()
    }

    /// 上游随机中点：整数随机数归一化至 [0,1]，沿水平与垂直方向偏移。
    /// 归一化遵循 Eigen 的零向量语义，不对小位移设置额外退化阈值。
    pub(crate) fn random_mid(
        &self,
        start: Vector3<f64>,
        goal: Vector3<f64>,
        rng: &mut impl rand::Rng,
    ) -> Vector3<f64> {
        let d = start - goal;
        let h = d.cross(&Vector3::z());
        let h = h.try_normalize(0.).unwrap_or(h);
        let v = d.cross(&h);
        let v = v.try_normalize(0.).unwrap_or(v);
        let k = -0.978 / (f64::from(self.consecutive_failures) + 0.989) + 0.989;
        let horizontal = f64::from(rng.random_range(0..=i32::MAX)) / f64::from(i32::MAX);
        let vertical = f64::from(rng.random_range(0..=i32::MAX)) / f64::from(i32::MAX);
        (start + goal) / 2.
            + (horizontal - 0.5) * d.norm() * h * 0.8 * k
            + (vertical - 0.5) * d.norm() * v * 0.4 * k
    }

    /// 官方 `setLocalTrajFromOpt`：规划结果连同带时间戳检查点一并入库，
    /// 并登记该轨迹的 `touch_goal`（碰撞监控扫描上界依据）。检查点生成失败
    /// （稠密采样退化）视同规划失败——无监控覆盖的轨迹不得进入执行。
    /// 返回是否入库成功。序号递增（官方 `setLocalTraj` 的 `traj_id++`，
    /// `plan_container.hpp:78`）：仅成功入库才消耗——入库失败不占序号。
    pub(crate) fn adopt_plan(&mut self, result: PlanResult, now: f64, touch_goal: bool) -> bool {
        if !self.set_local_traj(result.trajectory.clone(), now, touch_goal, self.traj_id + 1) {
            // 无监控覆盖的轨迹不入执行，视同本次重规划失败（官方
            // setLocalTrajFromOpt 失败时同样不更新局部轨迹）
            return false;
        }
        self.traj_id += 1;
        self.last_result = Some(result);
        true
    }

    /// 官方 `setLocalTrajFromOpt`：轨迹连同带时间戳检查点一并入库。检查点
    /// 生成失败（稠密采样退化）视同入库失败——无监控覆盖的轨迹不得进入
    /// 执行。`traj_id` 为写入本条的轨迹序号：常规规划由 [`Self::adopt_plan`]
    /// 以递增计数传入；急停轨迹不经 `adopt_plan`、以管理器当前计数器传入
    /// （不消耗新序号）。返回是否入库成功。
    pub(crate) fn set_local_traj(
        &mut self,
        traj: Trajectory,
        now: f64,
        touch_goal: bool,
        traj_id: u32,
    ) -> bool {
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
            traj_id,
        });
        self.touch_goal = touch_goal;
        true
    }

    /// 刷新障碍膨胀层（官方 `clearAndInflateLocalMap` 的监控侧调用）：地图
    /// 可能在本 tick 被深度感知/动态障碍更新，碰撞判定必须基于最新膨胀层。
    pub(crate) fn refresh_inflation(&mut self) {
        self.planner.map_mut().inflate_obstacles();
    }

    /// 执行期碰撞监控（官方 `checkCollisionCallback` 轨迹检查部分）：沿当前
    /// 轨迹检查点只扫 **未来** 采样，返回首个进入膨胀占据区的采样时刻
    /// （相对轨迹起点，秒）；无碰撞或无可执行轨迹返回 `None`。入口以
    /// `traj_id == 0`（无可执行轨迹）短路（官方 `ego_replan_fsm.cpp:307`）。
    ///
    /// 定位与扫描上界见 [`future_scan_range`]。集群间距检查在同一定位上的
    /// 姊妹实现 [`Self::next_swarm_conflict`]。
    ///
    /// 官方 `EXEC_TRAJ` `close_to_current_traj_end`：检查点末端时刻距当前
    /// 不足急停时限（`pts_chk.back.back.t − t_cur < emergency_time`）。
    #[must_use]
    pub(crate) fn close_to_traj_end(&self, t_cur: f64) -> bool {
        let Some(local) = self.local.as_ref() else {
            return false;
        };
        let Some(last_bucket) = local.points_to_check.last() else {
            return false;
        };
        let Some(&(t_end, _)) = last_bucket.last() else {
            return false;
        };
        t_end - t_cur < self.options.emergency_time
    }

    pub(crate) fn next_collision(&self, now: f64) -> Option<f64> {
        // 官方 ego_replan_fsm.cpp:307：traj_id <= 0 等价无可执行轨迹，入口短路
        if self.traj_id == 0 {
            return None;
        }
        let local = self.local.as_ref()?;
        if local.points_to_check.is_empty() {
            return None;
        }
        let t_cur = now - local.start_time;
        let (i_start, j_start, scan_end) =
            future_scan_range(&local.points_to_check, &local.traj, t_cur, self.touch_goal)?;
        let map = self.planner.map_ref();
        for (i, bucket) in local
            .points_to_check
            .iter()
            .enumerate()
            .take(scan_end)
            .skip(i_start)
        {
            let skip = if i == i_start { j_start } else { 0 };
            for (t, p) in bucket.iter().skip(skip) {
                if map.is_occupied_inflated(*p) {
                    return Some(*t);
                }
            }
        }
        None
    }

    /// FSM 层 swarm 预测检查（官方 `ego_replan_fsm.cpp` checkCollisionCallback
    /// 的 swarm 段）：沿当前轨迹未来检查点，对每个已登记其他机轨迹取预测位置
    /// （`t_X = t + (my_start_time - other_start_time)`，仅 `0 < t_X <
    /// other.duration` 有效），距离 `< swarm_clearance + des_clearance`
    /// 即冲突。返回 `(agent_id, 冲突检查点时刻（相对轨迹起点，秒））`。
    /// 入口同 [`Self::next_collision`]：`traj_id == 0` 直接短路。
    #[must_use]
    pub(crate) fn next_swarm_conflict(&self, now: f64) -> Option<(usize, f64)> {
        // 官方 ego_replan_fsm.cpp:307：traj_id <= 0 等价无可执行轨迹，入口短路
        if self.traj_id == 0 {
            return None;
        }
        if self.swarm.iter().all(Option::is_none) {
            return None;
        }
        let local = self.local.as_ref()?;
        if local.points_to_check.is_empty() {
            return None;
        }
        let t_cur = now - local.start_time;
        let (i_start, j_start, scan_end) =
            future_scan_range(&local.points_to_check, &local.traj, t_cur, self.touch_goal)?;
        let clearance = self.planner.config().swarm_clearance;
        for (i, bucket) in local
            .points_to_check
            .iter()
            .enumerate()
            .take(scan_end)
            .skip(i_start)
        {
            let skip = if i == i_start { j_start } else { 0 };
            for (t, p) in bucket.iter().skip(skip) {
                for (id, slot) in self.swarm.iter().enumerate() {
                    let Some(peer) = slot else { continue };
                    let t_x = *t + (local.start_time - peer.start_time);
                    if t_x > 0.0
                        && t_x < peer.traj.duration()
                        && (*p - peer.traj.eval(t_x).position).norm()
                            < clearance + peer.des_clearance
                    {
                        return Some((id, *t));
                    }
                }
            }
        }
        None
    }

    /// 地面高度测量（对照官方 `ego_replan_fsm::measureGroundHeight`）：沿
    /// 当前轨迹取前方 [`GROUND_LOOKAHEAD_DIST`] 米处（`2.0/max_vel` 秒后）
    /// 的采样点，按地图分辨率逐格下降扫描，首个占据体素的 z 即地面高度；
    /// 扫描出图底（官方 `getOccupancy == -1`）仍无占据则失败。无可执行
    /// 轨迹或前方点超出轨迹时长不测量。
    #[must_use]
    pub(crate) fn measure_ground_height(&self, now: f64) -> Option<f64> {
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
            // 虚拟地板不是地面（官方 `ret == -1` 先于 `ret == 1`）
            if map.virtual_wall().is_some_and(|w| p.z <= w.ground) {
                return None;
            }
            if map.is_occupied(p) {
                return Some(p.z);
            }
            p.z -= reso;
        }
    }

    /// 局部目标选取（官方 `getLocalTarget`，`planner_manager.cpp:327`）。
    /// 从 `glb_t_of_lc_tgt` 起以 `t_step = planning_horizon/20/max_vel` 步进
    /// 全局时间，取首个距 `start` ≥ `planning_horizon` 的点为局部目标并推进
    /// 簿记；步进到全局终点仍未达到 → 局部目标 = 全局终点、`touch_goal =
    /// true`、`glb_t_of_lc_tgt = start_time + duration`。目标速度：距全局终点
    /// < 刹车距离 `max_vel²/(2·max_acc)` 取零速，否则取全局轨迹在该时刻的
    /// 速度（官方 `local_target_vel`）。入口先将 `last_glb_t_of_lc_tgt =
    /// glb_t_of_lc_tgt`（暖启动内点拼接的段起点）。
    pub(crate) fn horizon(&mut self, start: Vector3<f64>) -> Horizon {
        let max_vel = self.planner.config().max_velocity;
        let max_acc = self.planner.config().max_acceleration;
        let hor = self.options.planning_horizon;
        let g = &mut self.global;
        g.last_glb_t_of_lc_tgt = g.glb_t_of_lc_tgt;
        let t_step = hor / 20.0 / max_vel.max(1e-9);
        let t_end = g.start_time + g.traj.duration();
        let mut t = g.glb_t_of_lc_tgt;
        let mut target = self.goal.coords;
        while t < t_end {
            let pos = g
                .traj
                .eval((t - g.start_time).clamp(0.0, g.traj.duration()))
                .position;
            if (pos - start).norm() >= hor {
                target = pos;
                g.glb_t_of_lc_tgt = t;
                break;
            }
            t += t_step;
        }
        // 步进越过全局终点（含首步即越过）→ 局部目标 = 全局终点（官方
        // 末点判定 `t - start >= duration - 1e-5`：循环中途命中但落进末窗
        // 同样覆盖为目标）
        let touch_goal = t - g.start_time >= g.traj.duration() - 1e-5;
        if touch_goal {
            target = self.goal.coords;
            g.glb_t_of_lc_tgt = t_end;
        }
        // 刹车距离（max_vel²/2·max_acc）内目标速度取零（官方 `local_target_vel`）
        let end_dist = (self.goal.coords - target).norm();
        let target_vel = if end_dist < max_vel * max_vel / (2.0 * max_acc.max(1e-9)) {
            Vector3::zeros()
        } else {
            g.traj
                .eval((t - g.start_time).clamp(0.0, g.traj.duration()))
                .velocity
        };
        Horizon {
            target,
            target_vel,
            touch_goal,
        }
    }
}

/// 官方 `EGOPlannerManager::EmergencyStop`：head/tail 边界状态全同
/// `[stop_pos, 0, 0]`、中间点即 `stop_pos` 本身、两段各 [`STOP_PIECE_DURATION`]——
/// 全部约束相等 ⇒ 多项式恒为常值，参考跟踪自然刹车悬停。
pub(crate) fn emergency_stop_traj(stop_pos: Vector3<f64>) -> Result<Trajectory> {
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

/// 官方 `EGOPlannerManager::planGlobalTrajWaypoints`（`planner_manager.cpp:425`）：
/// 以任务航点生成时间参数化全局引导；不施加障碍约束。首末状态完整（起点/
/// 终点，零速零加速），内点为任务航点（`path[0]` 即起点、末端即终点），
/// 段数 = `path.len() - 1`；段时间 = 段长 `/des_vel`（`des_vel = max_vel/1.5`），
/// 迭代 2 次：首轮最大速度超标（> `max_vel`）时 `des_vel /= 1.5` 重生成，
/// 达标即提前退出。
///
/// # Errors
///
/// 折线退化（`len < 2`，`goal == start` 悬停）时产出单段常值轨迹不报错；
/// MINCO 系统奇异才报错。
pub(crate) fn plan_global_traj_waypoints(
    start: Vector3<f64>,
    path: &[Vector3<f64>],
    max_vel: f64,
) -> Result<Trajectory> {
    // 退化（goal == start 悬停）：单段常值轨迹，无时间推进
    if path.len() < 2 {
        let endpoint = Endpoint {
            position: start,
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let minco =
            MincoBuilder::new(SolverOrder::MinimumJerk, endpoint, endpoint).build(&[], &[1.0])?;
        return minco.solve();
    }
    // 内点 = path[1..len-1]（排除起点与终点）：段数 = path.len() - 1
    let inner: Vec<nalgebra::Point3<f64>> = path[1..path.len() - 1]
        .iter()
        .map(|p| nalgebra::Point3::from(*p))
        .collect();
    let start_end = Endpoint {
        position: start,
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let goal_end = Endpoint {
        position: *path.last().expect("len >= 2"),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let mut des_vel = max_vel / 1.5;
    let mut traj = None;
    for _ in 0..2 {
        let mut durations = Vec::with_capacity(path.len() - 1);
        let mut prev = start;
        for wp in &path[1..] {
            durations.push((*wp - prev).norm() / des_vel.max(1e-9));
            prev = *wp;
        }
        let minco = MincoBuilder::new(SolverOrder::MinimumJerk, start_end, goal_end)
            .build(&inner, &durations)?;
        let t = minco.solve()?;
        if crate::planner::Planner::trajectory_max_vel(&t) < max_vel {
            traj = Some(t);
            break;
        }
        des_vel /= 1.5;
        traj = Some(t);
    }
    Ok(traj.expect("迭代 2 次必有产出"))
}

/// 未来检查点扫描定位（官方 `checkCollisionCallback` 的 i/j 定位与扫描上界）：
/// 返回 `(起始桶, 起始桶内首个未来采样下标, 扫描终点桶排他上界)`；无未来
/// 可扫采样时返回 `None`。已飞过部分不回扫；上界：`touch_goal` 查全程，
/// 否则前 3/4 桶（生成端已按 `two_thirds_id` 截断，此处再留余量）。
pub(crate) fn future_scan_range(
    pts: &PointsToCheck,
    traj: &Trajectory,
    t_cur: f64,
    touch_goal: bool,
) -> Option<(usize, usize, usize)> {
    let mut i_start = piece_index_at(traj, t_cur);
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
    let scan_end = if touch_goal {
        pts.len()
    } else {
        pts.len() * 3 / 4
    };
    Some((i_start, j_start, scan_end))
}

/// 官方 `poly_traj::Trajectory::locatePieceIdx` 的段号部分：`t` 所在段
/// 下标（越界夹紧到首/末段；官方还会把 `t` 局部化到段内，此处只要段号）。
pub(crate) fn piece_index_at(traj: &Trajectory, t: f64) -> usize {
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
pub(crate) fn path_length(points: &[Vector3<f64>]) -> f64 {
    points.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
}

/// 折线各顶点距起点的累计弧长（与 `points` 等长）。
pub(crate) fn cumulative_arcs(points: &[Vector3<f64>]) -> Vec<f64> {
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
pub(crate) fn point_at_arc(points: &[Vector3<f64>], cum: &[f64], s: f64) -> Vector3<f64> {
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

/// 官方 `planNextWaypoint` 的单目标输入：全局引导不承担碰撞验收，
/// 局部 A* / Rebound 与最终连续轨迹检查负责生成可执行参考。
pub(crate) fn global_waypoints(
    start: Vector3<f64>,
    goal: Vector3<f64>,
) -> Result<Vec<Vector3<f64>>> {
    for (name, point) in [("start", start), ("goal", goal)] {
        if point.iter().any(|v| !v.is_finite()) {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::OutOfRange,
                "global waypoint is not finite",
            )
            .with_context(name, format!("{point:?}")));
        }
    }
    Ok(if start == goal {
        vec![start]
    } else {
        vec![start, goal]
    })
}
