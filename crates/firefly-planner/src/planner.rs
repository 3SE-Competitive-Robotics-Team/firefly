//! EGO 规划器（Rebound 主循环）。
//!
//! 论文流程（Sci. Robot. 2022, Trajectory planning procedure）：
//! 1. 局部目标选择（规划距离内）
//! 2. A* 无碰撞引导路径（firefly-search）
//! 3. 引导路径 → MINCO 初始 {q, T}（firefly-trajectory）
//! 4. Rebound 循环：扫描新障碍 → 生成 {s,v} 平面 → L-BFGS 优化
//! 5. 无新障碍时返回轨迹

use firefly_cost::{
    Cost, FeasibilityPenalty, FormationPenalty, ObstaclePenalty, SmoothnessPenalty, SwarmPenalty,
    TimePenalty, UniformPenalty,
};
use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{GridMap, Plane};
use firefly_optimize::{Lbfgs, LbfgsConfig};
use firefly_trajectory::{Endpoint, Minco, MincoBuilder, SolverOrder, Trajectory};
use nalgebra::{Point3, Vector3};

use crate::config::PlannerConfig;
use crate::init::{self, InitConfig};
use crate::objective::MincoObjective;
use crate::obstacles::{Hit, ObstacleScanner};

#[derive(Debug, Clone, Copy)]
pub struct State {
    pub position: Point3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}

/// 初始解来源（对照官方 `computeInitState` 的两个 case）。
#[derive(Debug, Clone, Copy)]
pub enum InitSource<'a> {
    /// case 1：A* 引导路径重建（首帧 / 暖启动失败降级）。
    ColdStart,
    /// case 2：从上一条最优轨迹暖启动。`elapsed` 为已执行时长；
    /// `guide_tail` 为全局路径上从当前投影到局部目标的延续段
    /// （旧轨迹耗尽后的走向）。
    WarmStart {
        prev: &'a Trajectory,
        elapsed: f64,
        guide_tail: &'a [Vector3<f64>],
    },
}

#[derive(Debug, Clone)]
pub struct PlanResult {
    pub trajectory: Trajectory,
    pub iterations: usize,
    pub planes: Vec<Plane>,
}

/// 队形规格（官方语义：队形沿直线移动，每机固定偏移）。
#[derive(Debug, Clone)]
pub struct FormationSpec {
    pub line_start: Vector3<f64>,
    pub line_end: Vector3<f64>,
    /// 每架机在队形坐标系中的偏移（x 沿队形线、y 垂直、z 高度）。
    pub offsets: Vec<Vector3<f64>>,
    pub self_id: usize,
    /// 其他机轨迹（含绝对开始时间）。
    pub peers: Vec<firefly_cost::Peer>,
}

pub struct Planner {
    config: PlannerConfig,
    map: GridMap,
    formation: Option<FormationSpec>,
    astar: firefly_search::Astar,
}

impl Planner {
    /// Rebound 全局迭代上限（rebound/restart/缩放重试共享预算）。
    const REBOUND_MAX_ITERATIONS: usize = 40;

    #[must_use]
    pub fn new(config: PlannerConfig, mut map: GridMap) -> Self {
        map.inflate_obstacles(config.obstacle_inflation);
        Self {
            config,
            map,
            formation: None,
            astar: firefly_search::Astar::default(),
        }
    }

    /// 设置队形规格（论文 Formation expectation + 官方 formationGradCostP 动态推断）。
    #[must_use]
    pub fn with_formation(mut self, spec: FormationSpec) -> Self {
        self.formation = Some(spec);
        self
    }

    pub fn set_formation(&mut self, spec: FormationSpec) {
        self.formation = Some(spec);
    }

    pub fn clear_formation(&mut self) {
        self.formation = None;
    }

    #[must_use]
    pub fn config(&self) -> &PlannerConfig {
        &self.config
    }

    /// 规划器持有的地图（可视化等用途）。
    #[must_use]
    pub fn map_ref(&self) -> &GridMap {
        &self.map
    }

    /// 可变地图（动态障碍更新）。
    pub fn map_mut(&mut self) -> &mut GridMap {
        &mut self.map
    }

    /// # Errors
    ///
    /// `NotFound`：起点/终点不可达；`Convergence`：Rebound 超出迭代上限。
    #[fastrace::trace]
    pub fn plan(&mut self, start: State, goal: Point3<f64>) -> Result<PlanResult> {
        self.plan_in_swarm(start, goal, &[])
    }

    /// 带初始解来源的规划（连续重规划用暖启动，官方 `planFromLocalTraj`
    /// 策略链：case2 暖启动 → 失败降级 case1 冷启动）。
    ///
    /// # Errors
    ///
    /// `NotFound`：起点/终点不可达；`Convergence`：Rebound 超出迭代上限。
    #[fastrace::trace]
    pub fn plan_with_init(
        &mut self,
        start: State,
        goal: Point3<f64>,
        source: InitSource<'_>,
    ) -> Result<PlanResult> {
        self.plan_in_swarm_init(start, goal, &[], source)
    }

    /// 集群规划：本机轨迹避让其他机轨迹（论文 decentralized framework：
    /// 接收其他机轨迹作为约束，只规划自己）。
    /// # Errors
    ///
    /// `NotFound`：起点/终点不可达；`Convergence`：Rebound 超出迭代上限。
    #[fastrace::trace]
    pub fn plan_in_swarm(
        &mut self,
        start: State,
        goal: Point3<f64>,
        peers: &[firefly_cost::Peer],
    ) -> Result<PlanResult> {
        self.plan_in_swarm_init(start, goal, peers, InitSource::ColdStart)
    }

    /// 集群规划（带初始解来源，见 [`InitSource`]）。
    fn plan_in_swarm_init(
        &mut self,
        start: State,
        goal: Point3<f64>,
        peers: &[firefly_cost::Peer],
        source: InitSource<'_>,
    ) -> Result<PlanResult> {
        // 地图可能已被动态障碍更新：重算膨胀层（官方 clearAndInflateLocalMap）
        self.map.inflate_obstacles(self.config.obstacle_inflation);
        let start_endpoint = Endpoint {
            position: start.position.coords,
            velocity: start.velocity,
            acceleration: start.acceleration,
        };
        let local_goal = self.pick_local_goal(start.position.coords, goal.coords);
        let guide = init::search_guide(
            &mut self.astar,
            &self.map,
            start.position.coords,
            local_goal,
        )?;

        let minco = match source {
            InitSource::ColdStart => {
                let pieces = init::pieces_for_guide(&guide, self.config.piece_length);
                let init_config = InitConfig {
                    piece_length: self.config.piece_length,
                    pieces,
                    max_velocity: self.config.max_velocity,
                };
                init::init_from_path(
                    &init_config,
                    start_endpoint,
                    Point3::from(local_goal),
                    &guide,
                )?
            }
            InitSource::WarmStart {
                prev,
                elapsed,
                guide_tail,
            } => {
                let init_config = InitConfig {
                    piece_length: self.config.piece_length,
                    pieces: 0, // 暖启动段数按官方 case2 由距离决定，不使用
                    max_velocity: self.config.max_velocity,
                };
                match init::init_warm_start(
                    &init_config,
                    start_endpoint,
                    Point3::from(local_goal),
                    prev,
                    elapsed,
                    guide_tail,
                ) {
                    Ok(m) => m,
                    Err(e) => {
                        log::debug!("暖启动不可用（{e}），降级冷启动");
                        let pieces = init::pieces_for_guide(&guide, self.config.piece_length);
                        let cfg = InitConfig {
                            piece_length: self.config.piece_length,
                            pieces,
                            max_velocity: self.config.max_velocity,
                        };
                        init::init_from_path(
                            &cfg,
                            start_endpoint,
                            Point3::from(local_goal),
                            &guide,
                        )?
                    }
                }
            }
        };

        self.rebound(
            minco,
            &guide,
            start_endpoint,
            Point3::from(local_goal),
            peers,
        )
    }

    /// Rebound 主循环（论文 v1 Alg.2）：
    /// Rebound 主循环（对齐官方 v2 `PolyTrajOptimizer::optimize`）：
    /// - L-BFGS 内部动态检测约束点（`ReboundDetector`），发现新穿入即提前终止
    ///   （官方 `STOP_FOR_REBOUND`，`rebound_times ≤ 20`）；
    /// - 优化后 fine check（官方 `finelyCheckAndSetConstraintPoints`），
    ///   碰撞则并入新平面后 restart（`restart_nums ≤ 3`）；
    /// - swarm 距离不满足时权重 ×2（官方 `wei_swarm_mod_ *= 2`）；
    /// - 全局迭代上限 [`Self::REBOUND_MAX_ITERATIONS`] 兜底所有分支。
    // 主循环编排（初始化/L-BFGS/fine check/restart 多阶段），clippy too_many_lines 允许
    #[allow(clippy::too_many_lines)]
    fn rebound(
        &self,
        mut minco: Minco,
        guide: &[Vector3<f64>],
        start_endpoint: Endpoint,
        local_goal: Point3<f64>,
        peers: &[firefly_cost::Peer],
    ) -> Result<PlanResult> {
        let scanner =
            ObstacleScanner::new(&self.map).with_samples(self.config.constraint_points_per_piece);
        let n_points = minco.pieces() * (self.config.constraint_points_per_piece + 1);
        let mut planes_by_point: Vec<Vec<Plane>> = vec![Vec::new(); n_points];
        let mut prev_formation_dev = f64::MAX;

        let mut rebound_times = 0usize;
        let mut restart_nums = 0usize;
        let mut iteration = 0usize;

        Self::initial_constraints(&scanner, &mut minco, guide, &mut planes_by_point)?;

        loop {
            iteration += 1;
            // 全局迭代上限：所有内部分支（rebound/restart/时间缩放失败重试）
            // 共享此预算。没有它，"解安全但时间缩放后不可行"的确定性失败
            // 会零计数空转（try_finish 返回 None 不推进任何计数器）。
            // TODO(官方对齐)：官方以 in/out 自由点搜索 + A* 绕障重建约束，
            // 不存在该空转路径；上限是过渡防护，根治需按官方重建约束生成。
            if iteration > Self::REBOUND_MAX_ITERATIONS {
                return Err(Error::temporary(
                    ErrorKind::Convergence,
                    format!("planner exceeded rebound iteration limit ({iteration})"),
                ));
            }
            let _span =
                fastrace::local::LocalSpan::enter_with_local_parent(format!("rebound-{iteration}"));

            // 队形是软约束（官方靠持续重规划收敛）：单次规划中
            // 偏差不再改善即接受当前解，避免迭代耗尽
            if self.formation.is_some() && iteration > 1 {
                let traj = minco.solve()?;
                let dev = self.formation_deviation(&traj);
                if (prev_formation_dev - dev).abs() < 0.05 {
                    return self.finish(&minco, &planes_by_point, iteration);
                }
                prev_formation_dev = dev;
            }

            // 一次全量 L-BFGS（官方 max_iterations=200）；objective 内部检测约束点
            let mut objective = self.build_objective(
                start_endpoint,
                local_goal,
                &planes_by_point,
                peers,
                minco.pieces(),
                self.config.weight_obstacle,
                self.config.weight_swarm,
            );
            let x0 = Self::pack(&minco);
            // 队形目标是稳定约束（非动态障碍），一次优化到位需要更多迭代
            let config = if self.formation.is_some() {
                LbfgsConfig {
                    max_iterations: 300,
                    delta: 1e-4,
                    ..LbfgsConfig::default()
                }
            } else {
                LbfgsConfig::default()
            };
            let report = Lbfgs::new(config).minimize(&mut objective, x0)?;
            log::debug!(
                "rebound {iteration}: lbfgs iter={} converged={} early_exit={} grad={:.2e}",
                report.iterations,
                report.converged,
                report.early_exit,
                report.gradient_norm
            );
            // 吸收优化中检测到的新穿入点
            let pending = objective.take_pending();
            Self::absorb_pending(&self.map, &pending, &mut planes_by_point);
            // 无论是否 early exit，都从当前解继续（官方 lbfgs 就地更新 x）
            minco = objective.rebuild(&report.final_x)?;
            if report.early_exit {
                // 官方 STOP_FOR_REBOUND：约束已变，重新优化
                rebound_times += 1;
                log::debug!(
                    "rebound {iteration}: 内循环检测触发（planes={}）",
                    planes_by_point.iter().map(Vec::len).sum::<usize>()
                );
                if rebound_times > 20 {
                    return Err(Error::temporary(
                        ErrorKind::Convergence,
                        "planner exceeded rebound limit",
                    ));
                }
                continue;
            }

            // fine check（官方 finelyCheckAndSetConstraintPoints）
            let traj = minco.solve()?;
            if let Some(result) = self.try_finish(
                &minco,
                &traj,
                &scanner,
                guide,
                peers,
                &planes_by_point,
                iteration,
            )? {
                return Ok(result);
            }
            let (hits, safe) = scanner.scan_all(&traj, guide, &planes_by_point);
            if !safe {
                let outcome = Self::handle_unsafe(
                    &self.map,
                    &hits,
                    &mut planes_by_point,
                    &mut restart_nums,
                    iteration,
                );
                match outcome {
                    Ok(Some(())) => continue,
                    Ok(None) => {
                        return Err(Error::temporary(
                            ErrorKind::Convergence,
                            "planner stuck: trajectory unsafe with no new obstacle information",
                        ));
                    }
                    Err(e) => return Err(e),
                }
            }
            // TODO(官方对齐)：safe 但 swarm 不满足时官方 wei_swarm_mod_ *= 2，
            // 当前实现会放大梯度到失控（越界点无平面保护），先固定权重
            log::debug!("rebound {iteration}: swarm 不满足，保持权重重试");
        }
    }

    /// 官方 `planner_manager`：optimize 前对初始轨迹 fine check 建约束，
    /// 避免 L-BFGS 内循环检测到初始碰撞点而频繁提前终止。
    fn initial_constraints(
        scanner: &ObstacleScanner,
        minco: &mut Minco,
        guide: &[Vector3<f64>],
        planes_by_point: &mut [Vec<Plane>],
    ) -> Result<()> {
        let traj0 = minco.solve()?;
        let (init_hits, _) = scanner.scan_all(&traj0, guide, planes_by_point);
        Self::add_hit_planes(scanner.map(), &init_hits, planes_by_point);
        Ok(())
    }

    /// 组装最终 PlanResult（轨迹 + 迭代数 + 平面）。
    fn finish(
        &self,
        minco: &Minco,
        planes_by_point: &[Vec<Plane>],
        iteration: usize,
    ) -> Result<PlanResult> {
        let trajectory = self.ensure_feasible(minco)?;
        Ok(PlanResult {
            trajectory,
            iterations: iteration,
            planes: planes_by_point.iter().flatten().cloned().collect(),
        })
    }

    /// fine check 失败处理：有新穿入点则并入平面后 restart（≤3 次），
    /// 无新信息（stuck）返回 `None`——官方不强制重建，直接失败让 FSM 保持旧轨迹。
    fn handle_unsafe(
        map: &GridMap,
        hits: &[Hit],
        planes_by_point: &mut [Vec<Plane>],
        restart_nums: &mut usize,
        iteration: usize,
    ) -> Result<Option<()>> {
        if hits.is_empty() {
            return Ok(None);
        }
        log::debug!(
            "rebound {iteration}: fine check 碰撞 new_hits={} planes={}",
            hits.len(),
            planes_by_point.iter().map(Vec::len).sum::<usize>()
        );
        Self::add_hit_planes(map, hits, planes_by_point);
        *restart_nums += 1;
        // 重启上限：引导路径修正（simplify 膨胀）后，贴墙翻越/窄缝场景可能
        // 需要更多次"合并平面→重启"才把轨迹顶出膨胀层，3 次过紧导致
        // plan 经常失败；放宽到 6 次（每次重启都会带上新平面，收敛方向确定）。
        if *restart_nums > 6 {
            return Err(Error::temporary(
                ErrorKind::Convergence,
                "planner exceeded restart limit",
            ));
        }
        Ok(Some(()))
    }

    /// 轨迹安全（障碍/集群/队形）时构造最终结果；否则返回 `None` 继续迭代。
    #[allow(clippy::too_many_arguments)]
    fn try_finish(
        &self,
        minco: &Minco,
        traj: &Trajectory,
        scanner: &ObstacleScanner,
        guide: &[Vector3<f64>],
        peers: &[firefly_cost::Peer],
        planes_by_point: &[Vec<Plane>],
        iteration: usize,
    ) -> Result<Option<PlanResult>> {
        if !self.swarm_safe(traj, peers) || !self.formation_safe(traj) {
            return Ok(None);
        }
        let (_, safe) = scanner.scan_all(traj, guide, planes_by_point);
        if !safe {
            return Ok(None);
        }
        let trajectory = self.ensure_feasible(minco)?;
        if !scanner.is_safe(&trajectory) {
            // 时间缩放按时间采样重建，窄通道处采样点位移会造成安全误判
            //（几何理论不变，数值上可能翻转）。不 panic：丢弃该解继续迭代，
            // 外层循环有重启上限兜底（否则 dev 构建 debug_assert 直接杀进程）。
            log::debug!("time-rescaled trajectory unsafe, keep iterating");
            return Ok(None);
        }
        Ok(Some(PlanResult {
            trajectory,
            iterations: iteration,
            planes: planes_by_point.iter().flatten().cloned().collect(),
        }))
    }

    /// 轨迹与队形目标的最大偏差（诊断用，动态推断）。
    fn formation_deviation(&self, traj: &Trajectory) -> f64 {
        let Some(f) = &self.formation else {
            return 0.0;
        };
        let penalty = FormationPenalty::new(
            f.line_start,
            f.line_end,
            f.offsets.clone(),
            f.self_id,
            f.peers.clone(),
        );
        let kappa = self.config.constraint_points_per_piece;
        let mut max_dev: f64 = 0.0;
        for (i, ti) in traj.durations().iter().enumerate() {
            for j in 0..=kappa {
                let tau = j as f64 / kappa as f64;
                let t_abs = segment_abs_time(traj, i, *ti, tau);
                let (o, a, l, _) = penalty.formation_state(t_abs);
                let tar = penalty.target(o, a, l);
                max_dev = max_dev.max((traj.eval(t_abs).position - tar).norm());
            }
        }
        max_dev
    }

    /// 队形达成：所有约束点与动态目标的最大偏差 < 阈值（软约束容差）。
    fn formation_safe(&self, traj: &Trajectory) -> bool {
        const THRESHOLD: f64 = 0.8;
        let Some(f) = &self.formation else {
            return true;
        };
        // 软约束容差：x 方向时间形状差异（轨迹与 peer 不同构）是正常的，
        // 队形保持的核心是相对位置；官方依赖持续重规划收敛。
        let penalty = FormationPenalty::new(
            f.line_start,
            f.line_end,
            f.offsets.clone(),
            f.self_id,
            f.peers.clone(),
        );
        let kappa = self.config.constraint_points_per_piece;
        for (i, ti) in traj.durations().iter().enumerate() {
            for j in 0..=kappa {
                let tau = j as f64 / kappa as f64;
                let t_abs = segment_abs_time(traj, i, *ti, tau);
                let (o, a, l, _) = penalty.formation_state(t_abs);
                let tar = penalty.target(o, a, l);
                if (traj.eval(t_abs).position - tar).norm() > THRESHOLD {
                    return false;
                }
            }
        }
        true
    }

    /// 集群安全：所有约束点对每架 peer 的椭球距离 ≥ Cw（同一绝对时刻）。
    fn swarm_safe(&self, traj: &Trajectory, peers: &[firefly_cost::Peer]) -> bool {
        const KAPPA: usize = 20;
        if peers.is_empty() {
            return true;
        }
        // 官方 swarm_too_close：min_dist² < ((Cw_self + des_clearance) × 1.25)²
        for (i, ti) in traj.durations().iter().enumerate() {
            for j in 0..=KAPPA {
                let tau = j as f64 / KAPPA as f64;
                let mut t_abs = 0.0;
                for l in 0..i {
                    t_abs += traj.durations()[l];
                }
                t_abs += tau * ti;
                let p = traj.eval(t_abs).position;
                for peer in peers {
                    let duration = peer.traj.duration();
                    let pp = if t_abs < duration {
                        peer.traj.eval(t_abs).position
                    } else {
                        let s = peer.traj.eval(duration);
                        s.position + s.velocity * (t_abs - duration)
                    };
                    let diff = p - pp;
                    let d2 = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z / 4.0;
                    let c = (self.config.swarm_clearance + peer.clearance) * 1.25;
                    if d2 < c * c {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// 可行性 post-check（论文：constraints are converted into objectives,
    /// feasibility is guaranteed by postchecking）。
    /// 时间等比缩放：re = max{|v/vm|, √|a/am|, ∛|j/jm|}，
    /// 导数 ∝ 1/T 幂次缩放，闭式满足限制且不改变轨迹形状。
    fn ensure_feasible(&self, minco: &Minco) -> firefly_error::Result<Trajectory> {
        let traj = minco.solve()?;
        let mut re = 1.0f64;
        for k in 0..400 {
            let t = traj.duration() * f64::from(k) / 400.0;
            let s = traj.eval(t);
            re = re
                .max(s.velocity.norm() / self.config.max_velocity)
                .max((s.acceleration.norm() / self.config.max_acceleration).sqrt())
                .max((s.jerk.norm() / self.config.max_jerk).cbrt());
        }
        if re <= 1.0 {
            return Ok(traj);
        }
        let scale = re * 1.05;
        log::info!("time rescale x{scale:.3} for dynamical feasibility");
        let durations: Vec<f64> = (0..minco.pieces())
            .map(|i| minco.piece_duration(i) * scale)
            .collect();
        let waypoints: Vec<nalgebra::Point3<f64>> = minco.waypoints().collect();
        MincoBuilder::new(SolverOrder::MinimumJerk, minco.start(), minco.end())
            .build(&waypoints, &durations)
            .map(|m| m.solve().expect("nonsingular"))
    }

    #[allow(clippy::too_many_arguments)]
    fn build_objective<'a>(
        &'a self,
        start: Endpoint,
        goal: Point3<f64>,
        planes_by_point: &[Vec<Plane>],
        peers: &[firefly_cost::Peer],
        pieces: usize,
        weight_obstacle: f64,
        weight_swarm: f64,
    ) -> MincoObjective<'a> {
        let end = Endpoint {
            position: goal.coords,
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let mut cost = Cost::new()
            .add(self.config.weight_smoothness, SmoothnessPenalty)
            .add(self.config.weight_time, TimePenalty)
            .add(
                self.config.weight_feasibility,
                FeasibilityPenalty::new(
                    self.config.max_velocity,
                    self.config.max_acceleration,
                    self.config.max_jerk,
                )
                .with_samples(20),
            )
            .add(
                weight_obstacle,
                ObstaclePenalty::new(
                    self.config.obstacle_clearance,
                    self.config.obstacle_clearance_soft,
                    self.config.weight_obstacle_soft,
                    self.config.constraint_points_per_piece,
                    planes_by_point.to_vec(),
                ),
            )
            .add(
                weight_swarm,
                SwarmPenalty::new(self.config.swarm_clearance, 2.0, 1.0, peers.to_vec())
                    // 高密度采样：防止优化器压缩时长让采样点跳过 peer 时刻
                    .with_samples(20),
            )
            // 约束点均匀分布：防段时长消失（MINCO 奇异点）与薄障碍漏检
            .add(
                self.config.weight_formation,
                UniformPenalty::new().with_samples(self.config.constraint_points_per_piece),
            );
        // 队形保持（可选）：目标点从其他机位置动态推断（官方语义）
        if let Some(f) = &self.formation {
            cost = cost.add(
                self.config.weight_formation,
                FormationPenalty::new(
                    f.line_start,
                    f.line_end,
                    f.offsets.clone(),
                    f.self_id,
                    f.peers.clone(),
                )
                .with_samples(self.config.constraint_points_per_piece),
            );
        }
        MincoObjective::new(start, end, pieces, cost).with_detector(
            &self.map,
            self.config.constraint_points_per_piece,
            planes_by_point.to_vec(),
        )
    }

    fn pack(minco: &Minco) -> nalgebra::DVector<f64> {
        let pieces = minco.pieces();
        let mut x = nalgebra::DVector::zeros(3 * (pieces - 1) + pieces);
        for (i, w) in minco.waypoints().enumerate() {
            x[i * 3] = w.x;
            x[i * 3 + 1] = w.y;
            x[i * 3 + 2] = w.z;
        }
        for i in 0..pieces {
            x[3 * (pieces - 1) + i] = minco.piece_duration(i).ln();
        }
        x
    }

    /// 吸收 L-BFGS 内循环检测到的新穿入点（跳过与已有平面几乎重合的重复项）。
    fn absorb_pending(map: &GridMap, pending: &[Hit], planes_by_point: &mut [Vec<Plane>]) {
        Self::add_hit_planes(map, pending, planes_by_point);
    }

    /// 为穿入点添加平面（跳过与已有平面几乎重合的重复项）。
    fn add_hit_planes(map: &GridMap, hits: &[Hit], planes_by_point: &mut [Vec<Plane>]) {
        for hit in hits {
            let plane = ObstacleScanner::build_plane(map, hit.sample.position, hit.guide_point);
            let point = &mut planes_by_point[hit.point_index];
            if point.iter().all(|p| {
                (p.point() - plane.point()).norm() >= 0.1 || p.normal().dot(&plane.normal()) <= 0.99
            }) {
                point.push(plane);
            }
        }
    }

    fn pick_local_goal(&self, start: Vector3<f64>, goal: Vector3<f64>) -> Vector3<f64> {
        let to_goal = goal - start;
        let dist = to_goal.norm();
        if dist <= self.config.planning_distance {
            // 目标在规划距离内：若被膨胀障碍占用（随机地图常见），沿方向回退到
            // 最近自由点，否则 A* 直接报 "goal is occupied"——plan() 对随机地图
            // 成功率骤降（demo 有 target_clear，planner 层 `plan` API 也要兜底）。
            if self.map.is_occupied_inflated(goal) {
                let dir = to_goal / dist.max(1e-9);
                for step in 1..=24 {
                    let candidate = goal - dir * (f64::from(step) * self.map.resolution());
                    if !self.map.is_occupied_inflated(candidate) {
                        return candidate;
                    }
                }
            }
            return goal;
        }
        let dir = to_goal / dist;
        let base = start + dir * self.config.planning_distance;
        // 局部目标被障碍占用时沿方向回退找最近自由点（A* 要求目标可达，判定用膨胀层）
        if self.map.is_occupied_inflated(base) {
            for step in 1..=24 {
                let candidate =
                    start + dir * (self.config.planning_distance - f64::from(step) * 0.25);
                if !self.map.is_occupied_inflated(candidate) {
                    return candidate;
                }
            }
        }
        base
    }
}

/// 采样点的绝对时间（段前缀和 + τ·Tᵢ）。
fn segment_abs_time(traj: &Trajectory, piece: usize, duration: f64, tau: f64) -> f64 {
    let mut t = 0.0;
    for l in 0..piece {
        t += traj.durations()[l];
    }
    t + tau * duration
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_map::{GridMapBuilder, VoxelState};

    fn wall_scenario() -> (Planner, State, Point3<f64>, Vec<Vector3<f64>>) {
        // 0.1m 分辨率（与 demo 一致）：墙 x=4.5，高 z<1.5
        let mut map = GridMapBuilder::new(0.1, [100, 100, 100]).build().unwrap();
        for y in 0..100 {
            for z in 0..15 {
                map.set_state([45, y, z], VoxelState::Occupied);
            }
        }
        let planner = Planner::new(PlannerConfig::default(), map);
        let start = State {
            position: Point3::new(0.5, 0.5, 0.5),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let goal = Point3::new(9.0, 0.5, 0.5);
        // 绕墙引导路径（从墙上方跨过）
        let guide = vec![
            Vector3::new(0.5, 0.5, 0.5),
            Vector3::new(4.0, 0.5, 2.0),
            Vector3::new(5.0, 0.5, 2.0),
            Vector3::new(8.0, 0.5, 0.5),
        ];
        (planner, start, goal, guide)
    }

    #[test]
    fn rebound_escapes_wall() {
        firefly_observability::init();
        let (planner, start, goal, guide) = wall_scenario();
        let start_endpoint = Endpoint {
            position: start.position.coords,
            velocity: start.velocity,
            acceleration: start.acceleration,
        };
        let local_goal = planner.pick_local_goal(start.position.coords, goal.coords);

        // 真实初始化流程：MINCO 拟合引导路径（官方 initMJO），
        // 拐角切角会浅穿入膨胀层——rebound 修正的就是这类浅穿入
        let init_config = crate::init::InitConfig {
            piece_length: planner.config.piece_length,
            pieces: crate::init::pieces_for_guide(&guide, planner.config.piece_length),
            max_velocity: planner.config.max_velocity,
        };
        let wall_hitting = crate::init::init_from_path(
            &init_config,
            start_endpoint,
            Point3::from(local_goal),
            &guide,
        )
        .unwrap();

        // 初始轨迹必须真的浅穿入（测试前提：拐角切角进入膨胀层）
        let scanner = ObstacleScanner::new(&planner.map)
            .with_samples(planner.config.constraint_points_per_piece);
        let traj0 = wall_hitting.solve().unwrap();
        assert!(!scanner.is_safe(&traj0), "测试前提：初始轨迹必须穿入膨胀层");

        let result = planner
            .rebound(
                wall_hitting,
                &guide,
                start_endpoint,
                Point3::from(local_goal),
                &[],
            )
            .expect("rebound 必须逃出障碍");
        assert!(scanner.is_safe(&result.trajectory), "最终轨迹必须物理安全");
        assert!(!result.planes.is_empty(), "逃逸过程必须生成平面");

        // 边界条件保持
        let s0 = result.trajectory.eval(0.0);
        assert!((s0.position - start.position.coords).norm() < 1e-6);
        let sf = result.trajectory.eval(result.trajectory.duration());
        assert!((sf.position - local_goal).norm() < 1e-6);
    }
}
