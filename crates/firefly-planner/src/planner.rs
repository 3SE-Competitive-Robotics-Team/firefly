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
use crate::obstacles::ObstacleScanner;

#[derive(Debug, Clone, Copy)]
pub struct State {
    pub position: Point3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}

#[derive(Debug)]
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
    #[must_use]
    pub fn new(config: PlannerConfig, map: GridMap) -> Self {
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

        let init_config = InitConfig {
            trajectory_pieces: self.config.trajectory_pieces,
            max_velocity: self.config.max_velocity,
        };
        let minco = init::init_from_path(
            &init_config,
            start_endpoint,
            Point3::from(local_goal),
            &guide,
        )?;

        self.rebound(
            minco,
            &guide,
            start_endpoint,
            Point3::from(local_goal),
            peers,
        )
    }

    /// Rebound 主循环（论文 v1 Alg.2）：
    /// 1. 轨迹安全？返回（post-check 可行性）
    /// 2. 扫描新障碍 → 生成 {s,v} → 少量迭代优化（warm start）
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
        let n_points =
            self.config.trajectory_pieces * (self.config.constraint_points_per_piece + 1);
        let mut planes_by_point: Vec<Vec<Plane>> = vec![Vec::new(); n_points];

        let mut prev_formation_dev = f64::MAX;
        for iteration in 0..16 {
            let _span =
                fastrace::local::LocalSpan::enter_with_local_parent(format!("rebound-{iteration}"));
            let traj = minco.solve()?;
            // 队形是软约束（官方靠持续重规划收敛）：单次规划中
            // 偏差不再改善即接受当前解，避免迭代耗尽
            if self.formation.is_some() && iteration > 0 {
                let dev = self.formation_deviation(&traj);
                if (prev_formation_dev - dev).abs() < 0.05 {
                    let trajectory = self.ensure_feasible(&minco)?;
                    return Ok(PlanResult {
                        trajectory,
                        iterations: iteration,
                        planes: planes_by_point.iter().flatten().cloned().collect(),
                    });
                }
            }
            let (hits, safe) = scanner.scan_all(&traj, guide, &planes_by_point);
            prev_formation_dev = self.formation_deviation(&traj);
            if safe && self.swarm_safe(&traj, peers) && self.formation_safe(&traj) {
                let trajectory = self.ensure_feasible(&minco)?;
                debug_assert!(
                    scanner.is_safe(&trajectory),
                    "rescale must not change geometry"
                );
                return Ok(PlanResult {
                    trajectory,
                    iterations: iteration,
                    planes: planes_by_point.iter().flatten().cloned().collect(),
                });
            }
            if hits.is_empty() && !safe {
                let (mut xmin, mut xmax, mut ymin, mut ymax, mut zmin, mut zmax) =
                    (f64::MAX, f64::MIN, f64::MAX, f64::MIN, f64::MAX, f64::MIN);
                for k in 0..100 {
                    let t = traj.duration() * f64::from(k) / 100.0;
                    let p = traj.eval(t).position;
                    xmin = xmin.min(p.x);
                    xmax = xmax.max(p.x);
                    ymin = ymin.min(p.y);
                    ymax = ymax.max(p.y);
                    zmin = zmin.min(p.z);
                    zmax = zmax.max(p.z);
                }
                log::warn!(
                    "stuck check: safe=false, traj x[{xmin:.1},{xmax:.1}] y[{ymin:.1},{ymax:.1}] z[{zmin:.1},{zmax:.1}]"
                );
                // 轨迹仍被占据但无新障碍信息（被已有平面覆盖）：继续迭代无意义。
                // 注意：地图已安全（safe）时是集群避碰在推进，不能提前退出。
                return Err(Error::temporary(
                    ErrorKind::Convergence,
                    "planner stuck: trajectory unsafe with no new obstacle information",
                )
                .with_context("iteration", iteration));
            }
            log::debug!(
                "rebound {iteration}: planes={} new_hits={} formation_dev={:.3}",
                planes_by_point.iter().map(Vec::len).sum::<usize>(),
                hits.len(),
                self.formation_deviation(&traj)
            );
            for hit in &hits {
                let plane =
                    ObstacleScanner::build_plane(&self.map, hit.sample.position, hit.guide_point);
                let point = &mut planes_by_point[hit.point_index];
                if point.iter().all(|p| {
                    (p.point() - plane.point()).norm() >= 0.1
                        || p.normal().dot(&plane.normal()) <= 0.99
                }) {
                    point.push(plane);
                }
            }

            let mut objective =
                self.build_objective(start_endpoint, local_goal, &planes_by_point, peers);
            let x0 = self.pack(&minco);
            // 论文 v1 Alg.2 OneStepOptimize 精神：目标每轮随新障碍变化，
            // 每轮只跑少量迭代就重新检查，避免在动态目标上过度优化。
            // 队形引导是稳定目标（每轮不变），一次优化到位。
            let iterations = if self.formation.is_some() { 300 } else { 40 };
            let config = LbfgsConfig {
                max_iterations: iterations,
                ..LbfgsConfig::default()
            };
            let report = Lbfgs::new(config).minimize(&mut objective, x0)?;
            if !report.converged {
                log::debug!(
                    "rebound {iteration}: lbfgs partial (grad={:.3e})",
                    report.gradient_norm
                );
            }
            minco = objective.rebuild(&report.final_x)?;
        }

        Err(Error::temporary(
            ErrorKind::Convergence,
            "planner exceeded rebound iterations",
        ))
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

    fn build_objective(
        &self,
        start: Endpoint,
        goal: Point3<f64>,
        planes_by_point: &[Vec<Plane>],
        peers: &[firefly_cost::Peer],
    ) -> MincoObjective {
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
                self.config.weight_obstacle,
                ObstaclePenalty::new(
                    self.config.obstacle_clearance,
                    self.config.constraint_points_per_piece,
                    planes_by_point.to_vec(),
                ),
            )
            .add(
                self.config.weight_swarm,
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
        MincoObjective::new(start, end, self.config.trajectory_pieces, cost)
    }

    fn pack(&self, minco: &Minco) -> nalgebra::DVector<f64> {
        let pieces = self.config.trajectory_pieces;
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

    fn pick_local_goal(&self, start: Vector3<f64>, goal: Vector3<f64>) -> Vector3<f64> {
        let to_goal = goal - start;
        let dist = to_goal.norm();
        if dist <= self.config.planning_distance {
            return goal;
        }
        let dir = to_goal / dist;
        let base = start + dir * self.config.planning_distance;
        // 局部目标被障碍占用时沿方向回退找最近自由点（A* 要求目标可达）
        if self.map.is_occupied(base) {
            for step in 1..=8 {
                let candidate =
                    start + dir * (self.config.planning_distance - f64::from(step) * 0.5);
                if !self.map.is_occupied(candidate) {
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
        let mut map = GridMapBuilder::new(0.5, [20, 20, 20]).build().unwrap();
        for y in 0..20 {
            for z in 0..3 {
                map.set_state([9, y, z], VoxelState::Occupied);
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
        let (planner, start, goal, guide) = wall_scenario();
        let start_endpoint = Endpoint {
            position: start.position.coords,
            velocity: start.velocity,
            acceleration: start.acceleration,
        };
        let local_goal = planner.pick_local_goal(start.position.coords, goal.coords);

        // 构造穿墙初始轨迹：q 沿 x 直线（无视墙），T 保守
        let wall_hitting = MincoBuilder::new(
            SolverOrder::MinimumJerk,
            start_endpoint,
            Endpoint {
                position: local_goal,
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
        )
        .build(
            &[
                Point3::new(2.0, 0.5, 0.5),
                Point3::new(4.0, 0.5, 0.5),
                Point3::new(6.0, 0.5, 0.5),
                Point3::new(7.5, 0.5, 0.5),
            ],
            &[2.0, 2.0, 2.0, 2.0, 2.0],
        )
        .unwrap();

        // 初始轨迹必须真的穿墙（测试前提）
        let scanner = ObstacleScanner::new(&planner.map)
            .with_samples(planner.config.constraint_points_per_piece);
        let traj0 = wall_hitting.solve().unwrap();
        assert!(!scanner.is_safe(&traj0), "测试前提：初始轨迹必须穿墙");

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
