//! 时空联合优化目标。
//!
//! 变量 x = [q 展平 (3(M−1)), vt(M)]，vt 为虚拟时间，对照官方
//! `poly_traj_optimizer.cpp:VirtualT2RealT` 的分段有理映射保证真实时间 T 恒正。
//! 代价：firefly-cost 各项(平滑/时间/可行/障碍/集群/队形/均匀)，
//! 梯度经 `Minco::propagate_gradient` 传播到 {q, T} 后经虚拟时间链式法则转回。
//!
//! L-BFGS 内循环碰撞检测对齐官方 `poly_traj_optimizer.cpp`:
//! 代价回调内 `iter_num_ >= 3 且采样约束点方向足够平顺` 时调用
//! `roughlyCheckConstraintPoints`——在约束点数组上检测未覆盖穿入点、
//! 沿数组 in/out 自由点搜索 + A\* 绕障 + 交点平面,命中即提前终止
//! (官方 `STOP_FOR_REBOUND`),由外层吸收新平面后重新优化。

use firefly_cost::{Accumulator, MincoCost};
use firefly_map::{GridMap, Plane};
use firefly_optimize::Objective;
use firefly_trajectory::{Endpoint, Minco, MincoBuilder, SolverOrder};
use nalgebra::{DMatrix, DVector, Point3, Vector3};

use crate::obstacles::{ObstacleScanner, constraint_sample_points};

/// 虚拟时间 → 真实时间，对照官方 `poly_traj_optimizer.cpp:1200` `VirtualT2RealT`。
#[inline]
pub(crate) fn virtual_to_real(vt: f64) -> f64 {
    if vt > 0.0 {
        (0.5 * vt + 1.0) * vt + 1.0
    } else {
        1.0 / ((0.5 * vt - 1.0) * vt + 1.0)
    }
}

/// 真实时间 → 虚拟时间，对照官方 `poly_traj_optimizer.cpp:1190` `RealT2VirtualT`。
#[inline]
pub(crate) fn real_to_virtual(rt: f64) -> f64 {
    if rt > 1.0 {
        (2.0 * rt - 1.0).sqrt() - 1.0
    } else {
        1.0 - (2.0 / rt - 1.0).sqrt()
    }
}

/// dT/dVT，对照官方 `poly_traj_optimizer.cpp:1209` `VirtualTGradCost`。
#[inline]
pub(crate) fn d_real_d_virtual(vt: f64) -> f64 {
    if vt > 0.0 {
        vt + 1.0
    } else {
        let den = (0.5 * vt - 1.0) * vt + 1.0;
        (1.0 - vt) / (den * den)
    }
}

/// 内循环碰撞检测(官方 `roughlyCheckConstraintPoints`):
/// 持有平面池,检测到新穿入点即就地追加 {s,v} 平面。
pub struct ReboundDetector<'a> {
    map: &'a GridMap,
    samples_per_piece: usize,
    max_vel: f64,
    touch_goal: bool,
    planes: Vec<Vec<Plane>>,
    astar: firefly_search::Astar,
    /// 官方 `use_multitopology_trajs`:开启时按 criterion 3 门控检测。
    multitopo: bool,
    /// 官方 `initial_obstacles_avoided`:初始障碍已避开的锁存位
    /// 同一次完整优化的内层重启共享；新一次完整优化开始时复位。
    gate_latched: bool,
}

impl<'a> ReboundDetector<'a> {
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        map: &'a GridMap,
        samples_per_piece: usize,
        max_vel: f64,
        touch_goal: bool,
        planes: Vec<Vec<Plane>>,
        multitopo: bool,
        gate_latched: bool,
    ) -> Self {
        Self {
            map,
            samples_per_piece,
            max_vel,
            touch_goal,
            planes,
            astar: firefly_search::Astar::default(),
            multitopo,
            gate_latched,
        }
    }

    /// 官方 allowRebound criterion 3 的锁存位(诊断/测试用)。
    #[must_use]
    pub fn gate_latched(&self) -> bool {
        self.gate_latched
    }

    /// 官方 `roughlyCheckConstraintPoints`:检测新穿入点并在平面池中追加
    /// 约束。返回是否发现新障碍(触发 Rebound)。
    #[must_use]
    pub fn check(&mut self, traj: &firefly_trajectory::Trajectory) -> bool {
        let points = constraint_sample_points(traj, self.samples_per_piece);
        self.check_points(&points)
    }

    /// 用调用方已算好的唯一约束点做检测。
    pub fn check_points(&mut self, points: &[Vector3<f64>]) -> bool {
        let scanner = ObstacleScanner::new(self.map)
            .with_samples(self.samples_per_piece)
            .with_max_vel(self.max_vel);
        if !constraint_points_straight_enough(points) {
            return false;
        }
        // 官方 allowRebound criterion 3:多拓扑下任一内部控制点仍在初始基点
        // 错误侧((p−base)·direction < 0,只看每点第 0 个平面)时停用内循环
        // 检测——让候选先在自己初始化的一侧收敛;判定已避开即锁存为 true。
        if self.multitopo && !self.gate_latched {
            let avoided = points[1..points.len() - 1]
                .iter()
                .enumerate()
                .all(|(k, p)| {
                    let i = k + 1;
                    self.planes[i].is_empty()
                        || (*p - self.planes[i][0].point()).dot(&self.planes[i][0].normal()) >= 0.0
                });
            self.gate_latched = avoided;
            if !avoided {
                return false;
            }
        }
        scanner.roughly_check(&mut self.astar, points, &mut self.planes, self.touch_goal)
    }

    /// 取走(含新追加平面的)完整平面池,供外层下一轮使用。
    #[must_use]
    pub fn take_planes(&mut self) -> Vec<Vec<Plane>> {
        std::mem::take(&mut self.planes)
    }
}

pub struct MincoObjective<'a> {
    start: Endpoint,
    end: Endpoint,
    pieces: usize,
    // 本轮优化的约束快照；Detector 追加平面不修改此处的代价或梯度输入。
    cost: MincoCost,
    // 复用 MINCO 和轨迹缓冲；联合回调每次重新生成系数并应用当前代价项。
    cache: Option<(DVector<f64>, Minco, firefly_trajectory::Trajectory)>,
    // 平面池归检测器独占，新增约束由 take_planes 交给下一轮目标。
    detector: Option<ReboundDetector<'a>>,
    eval_count: usize,
    gradient_count: usize,
    lu_count: usize,
    early_exit: bool,
    acc: Accumulator,
    points: Vec<Point3<f64>>,
    /// 时长缓冲：与 `points` 一样按需复用，避免每轮重建。
    scratch_t: Vec<f64>,
    /// 梯度传播的复用缓冲：转置解 `6M×3`、`d_f_d_q`、`d_f_d_t`。
    scratch_g: DMatrix<f64>,
    scratch_dq: DMatrix<f64>,
    scratch_dt: DVector<f64>,
    detector_points: Vec<Vector3<f64>>,
}

impl<'a> MincoObjective<'a> {
    #[must_use]
    pub fn new(start: Endpoint, end: Endpoint, pieces: usize, cost: MincoCost) -> Self {
        Self {
            start,
            end,
            pieces,
            cost,
            cache: None,
            detector: None,
            eval_count: 0,
            gradient_count: 0,
            lu_count: 0,
            early_exit: false,
            acc: Accumulator::new(pieces),
            points: Vec::with_capacity(pieces.saturating_sub(1)),
            scratch_t: Vec::with_capacity(pieces),
            scratch_g: DMatrix::zeros(6 * pieces, 3),
            scratch_dq: DMatrix::zeros(3, pieces.saturating_sub(1)),
            scratch_dt: DVector::zeros(pieces),
            detector_points: Vec::new(),
        }
    }

    /// (代价求值次数，完整梯度次数，成功 LU 分解次数)。
    #[must_use]
    pub fn evaluation_counts(&self) -> (usize, usize, usize) {
        (self.eval_count, self.gradient_count, self.lu_count)
    }

    /// 挂载内循环碰撞检测(官方 `allowRebound` 三判据；
    /// 多拓扑门控见 [`ReboundDetector::check`],`gate_latched` 由调用方传入
    /// 以跨外层迭代保持锁存)。
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn with_detector(
        mut self,
        map: &'a GridMap,
        samples_per_piece: usize,
        max_vel: f64,
        touch_goal: bool,
        planes: Vec<Vec<Plane>>,
        multitopo: bool,
        gate_latched: bool,
    ) -> Self {
        self.detector = Some(ReboundDetector::new(
            map,
            samples_per_piece,
            max_vel,
            touch_goal,
            planes,
            multitopo,
            gate_latched,
        ));
        self
    }

    /// 官方 criterion 3 锁存位当前值(供调用方在下一次构建时透传)。
    #[must_use]
    pub fn gate_latched(&self) -> bool {
        self.detector
            .as_ref()
            .is_some_and(ReboundDetector::gate_latched)
    }

    /// 取走本轮优化后的完整平面池（含检测到的新平面），交给下一轮目标。
    /// 本轮目标只供结束后的诊断读取，不得取走平面池后继续优化。
    pub fn take_planes(&mut self) -> Vec<Vec<Plane>> {
        match &mut self.detector {
            Some(d) => d.take_planes(),
            None => Vec::new(),
        }
    }

    /// MINCO generate：每次联合回调重新求系数，时长不变时复用 LU。
    fn generate(&mut self, x: &DVector<f64>) -> Option<()> {
        self.unpack_into(x);
        if let Some((last_x, minco, traj)) = &mut self.cache {
            let times_changed = self
                .scratch_t
                .iter()
                .enumerate()
                .any(|(i, duration)| *duration != minco.piece_duration(i));
            minco
                .update(self.start, self.end, &self.points, &self.scratch_t)
                .ok()?;
            self.lu_count += usize::from(times_changed);
            minco.solve_into(traj).ok()?;
            last_x.clone_from(x);
        } else {
            let minco = MincoBuilder::new(SolverOrder::MinimumJerk, self.start, self.end)
                .build(&self.points, &self.scratch_t)
                .ok()?;
            self.lu_count += 1;
            let traj = minco.solve().ok()?;
            self.cache = Some((x.clone(), minco, traj));
        }
        Some(())
    }

    /// 独立测试回调共用同一条轨迹，代价和梯度仍各自计算。
    fn solve_cached(&mut self, x: &DVector<f64>) -> Option<()> {
        if self
            .cache
            .as_ref()
            .is_some_and(|(last_x, _, _)| last_x == x)
        {
            Some(())
        } else {
            self.generate(x)
        }
    }

    /// 就地展开参数：复用 `points` 与 `scratch_t`，不产生中间 `Vec`。
    ///
    /// 参数布局与 [`Self::unpack`] 一致；热路径（每个 LBFGS 迭代）走这里。
    fn unpack_into(&mut self, x: &DVector<f64>) {
        let n_q = 3 * (self.pieces - 1);
        self.points.clear();
        for i in 0..self.pieces - 1 {
            self.points
                .push(Point3::new(x[i * 3], x[i * 3 + 1], x[i * 3 + 2]));
        }
        self.scratch_t.clear();
        for i in 0..self.pieces {
            self.scratch_t.push(virtual_to_real(x[n_q + i]));
        }
    }

    #[must_use]
    pub fn unpack(&self, x: &DVector<f64>) -> (Vec<Vector3<f64>>, Vec<f64>) {
        let n_q = 3 * (self.pieces - 1);
        let mut q = Vec::with_capacity(self.pieces - 1);
        for i in 0..self.pieces - 1 {
            q.push(Vector3::new(x[i * 3], x[i * 3 + 1], x[i * 3 + 2]));
        }
        // 对照官方 `VirtualT2RealT`：无 clamp，分段有理映射本身不会溢出。
        let t = (0..self.pieces)
            .map(|i| virtual_to_real(x[n_q + i]))
            .collect();
        (q, t)
    }

    /// # Errors
    ///
    /// `InvalidArgument`:x 中时长非正(虚拟时间映射保证恒正，不应发生)。
    pub fn rebuild(&self, x: &DVector<f64>) -> firefly_error::Result<Minco> {
        let (q, t) = self.unpack(x);
        let points: Vec<Point3<f64>> = q.iter().map(|v| Point3::from(*v)).collect();
        MincoBuilder::new(SolverOrder::MinimumJerk, self.start, self.end).build(&points, &t)
    }

    /// 打包独立测试接口的梯度，dt 已包含时间项。
    #[must_use]
    pub fn pack(&self, dq: &DMatrix<f64>, dt: &DVector<f64>, vt: &[f64]) -> DVector<f64> {
        let mut g = DVector::zeros(3 * (self.pieces - 1) + self.pieces);
        for i in 0..self.pieces - 1 {
            for d in 0..3 {
                g[i * 3 + d] = dq[(d, i)];
            }
        }
        for i in 0..self.pieces {
            // 对照官方 `VirtualTGradCost`：`gdVT = (gdRT + wei_time) * dT/dVT`。
            // firefly 的 `TimePenalty` 已把 `wei_time` 并入 `dt`（`accumulate` 中
            // `d_f_d_t += weight`），故此处 `dt` 已等价于 `gdRT + wei_time`，不得
            // 重复加；`dT/dVT` 按虚拟时间分段，对照官方 `gdVT2Rt`。
            g[3 * (self.pieces - 1) + i] = dt[i] * d_real_d_virtual(vt[i]);
        }
        g
    }
}

/// 官方 allowRebound criterion 2：采样约束点的内部相邻方向余弦 ≥ 0.87。
/// 首尾各三个点不参与判定；MINCO 路点不代表轨迹上的采样方向。
fn constraint_points_straight_enough(points: &[Vector3<f64>]) -> bool {
    if points.len() < 7 {
        return true;
    }
    (3..=points.len() - 4).all(|i| {
        let before = points[i] - points[i - 1];
        let after = points[i + 1] - points[i];
        let product = before.normalize().dot(&after.normalize());
        // 零长段无方向，官方 min_product 不因 NaN 更新。
        product.is_nan() || product >= 0.87
    })
}

impl Objective for MincoObjective<'_> {
    /// 对照上游 costFunctionCallback 的执行顺序。
    #[fastrace::trace]
    fn evaluate_gradient(&mut self, x: &DVector<f64>, gradient: &mut DVector<f64>) -> f64 {
        gradient.resize_vertically_mut(x.len(), 0.0);
        let Some(()) = self.generate(x) else {
            gradient.fill(0.0);
            return f64::INFINITY;
        };
        let (_, minco, traj) = self.cache.as_ref().expect("solved trajectory");
        let smoothness_cost = self
            .cost
            .init_and_get_smoothness_grad_cost(traj, &mut self.acc);
        let costs = self
            .cost
            .add_pvaj_grad_cost(traj, &mut self.acc, &mut self.detector_points);
        if self.eval_count >= 3
            && let Some(detector) = &mut self.detector
            && detector.check_points(&self.detector_points)
        {
            self.early_exit = true;
        }
        // 检测只更新下一轮的平面池；本次伴随传播使用惩罚累加时的同一快照。
        let propagated = minco.propagate_gradient_into(
            traj,
            &self.acc.d_f_d_c,
            &self.acc.d_f_d_t,
            &mut self.scratch_g,
            &mut self.scratch_dq,
            &mut self.scratch_dt,
        );
        if propagated.is_err() {
            gradient.fill(0.0);
            return f64::INFINITY;
        }
        let n_q = 3 * (self.pieces - 1);
        for i in 0..self.pieces - 1 {
            for dim in 0..3 {
                gradient[3 * i + dim] = self.scratch_dq[(dim, i)];
            }
        }
        for i in 0..self.pieces {
            gradient[n_q + i] =
                (self.scratch_dt[i] + self.cost.weight_time) * d_real_d_virtual(x[n_q + i]);
        }
        let time_cost = traj.duration() * self.cost.weight_time;
        self.gradient_count += 1;
        self.eval_count += 1;
        smoothness_cost + costs.iter().sum::<f64>() + time_cost
    }

    fn evaluate(&mut self, x: &DVector<f64>) -> f64 {
        self.eval_count += 1;
        let Some(()) = self.solve_cached(x) else {
            return f64::INFINITY;
        };
        let (_, _, traj) = self.cache.as_ref().expect("solved trajectory");
        if self.eval_count > 3 {
            self.detector_points = constraint_sample_points(traj, self.cost.samples_per_piece);
            if let Some(detector) = &mut self.detector
                && detector.check_points(&self.detector_points)
            {
                self.early_exit = true;
            }
        }
        self.cost.evaluate(traj)
    }

    fn early_exit(&self) -> bool {
        self.early_exit
    }

    fn gradient(&mut self, x: &DVector<f64>) -> DVector<f64> {
        let Some(()) = self.solve_cached(x) else {
            return DVector::zeros(3 * (self.pieces - 1) + self.pieces);
        };
        let (_, minco, traj) = self.cache.as_ref().expect("solved trajectory");
        self.cost.gradient_into(traj, &mut self.acc);
        let propagated = minco.propagate_gradient_into(
            traj,
            &self.acc.d_f_d_c,
            &self.acc.d_f_d_t,
            &mut self.scratch_g,
            &mut self.scratch_dq,
            &mut self.scratch_dt,
        );
        if propagated.is_err() {
            return DVector::zeros(3 * (self.pieces - 1) + self.pieces);
        }
        self.gradient_count += 1;
        let n_q = 3 * (self.pieces - 1);
        self.pack(&self.scratch_dq, &self.scratch_dt, &x.as_slice()[n_q..])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_map::{GridMapBuilder, VoxelState};

    #[test]
    #[allow(clippy::too_many_lines)]
    fn joint_callback_matches_pinned_cpp_and_constraint_points() {
        use firefly_cost::{FeasibilityPenalty, ObstaclePenalty, Peer, SwarmPenalty};
        let mut fixture = include_str!("../../firefly-cost/tests/reference/ego_callback.txt")
            .lines()
            .filter(|line| !line.starts_with('#'))
            .flat_map(str::split_whitespace)
            .map(|value| value.parse::<f64>().unwrap());
        let close = |actual: f64, expected: f64| {
            assert!(
                (actual - expected).abs() <= 2e-9 * expected.abs().max(1.0),
                "actual={actual:.17e}, upstream={expected:.17e}"
            );
        };
        let start = Endpoint {
            position: Vector3::zeros(),
            velocity: Vector3::new(0.5, -0.2, 0.1),
            acceleration: Vector3::new(0.1, 0.0, -0.1),
        };
        let end = Endpoint {
            position: Vector3::new(4.0, 2.0, 1.0),
            velocity: Vector3::new(0.2, 0.1, -0.1),
            acceleration: Vector3::new(0.0, -0.1, 0.0),
        };
        let peer_start = Endpoint {
            position: Vector3::new(1.8, 0.9, 0.4),
            velocity: Vector3::new(0.2, -0.1, 0.05),
            acceleration: Vector3::zeros(),
        };
        let peer_end = Endpoint {
            position: Vector3::new(2.1, 1.1, 0.5),
            velocity: Vector3::new(0.3, 0.1, -0.05),
            ..peer_start
        };
        let peer_trajectory = MincoBuilder::new(SolverOrder::MinimumJerk, peer_start, peer_end)
            .build(&[], &[1.4])
            .unwrap()
            .solve()
            .unwrap();
        let x = DVector::from_vec(vec![
            1.5,
            0.8,
            0.3,
            2.8,
            1.5,
            0.6,
            real_to_virtual(0.8),
            real_to_virtual(1.0),
            real_to_virtual(1.2),
        ]);
        for variant in 0..3 {
            let cutoff = if variant == 1 { 15 } else { 11 };
            let limits = if variant == 2 {
                [1e9; 3]
            } else {
                [1.2, 2.0, 4.0]
            };
            let planes = (0..16)
                .map(|index| {
                    if index % 4 == 0 {
                        Vec::new()
                    } else {
                        vec![
                            Plane::new(Vector3::new(1.0, 0.3, 0.2), Vector3::x()),
                            Plane::new(Vector3::new(0.0, 1.6, 0.0), -Vector3::y()),
                        ]
                    }
                })
                .collect();
            let cost = MincoCost {
                weight_smoothness: 1.2,
                weight_time: 0.4,
                weight_sqrvariance: 0.8,
                obstacle: Some((
                    1.0,
                    ObstaclePenalty::new(
                        if variant == 2 { -100.0 } else { 0.1 },
                        if variant == 2 { -100.0 } else { 0.5 },
                        2.3,
                        0.7,
                        5,
                        planes,
                    )
                    .with_two_thirds(cutoff),
                )),
                swarm: Some((
                    1.7,
                    SwarmPenalty::new(
                        0.5,
                        2.0,
                        1.0,
                        vec![Peer::new(1, 0.15, peer_trajectory.clone(), 0.45)],
                    )
                    .with_two_thirds(cutoff),
                )),
                feasibility: Some((
                    0.3,
                    FeasibilityPenalty::new(limits[0], limits[1], limits[2]),
                )),
                ..MincoCost::default()
            };
            let mut objective = MincoObjective::new(start, end, 3, cost);
            let mut gradient = DVector::zeros(0);
            close(
                objective.evaluate_gradient(&x, &mut gradient),
                fixture.next().unwrap(),
            );
            for dim in 0..3 {
                for waypoint in 0..2 {
                    close(gradient[waypoint * 3 + dim], fixture.next().unwrap());
                }
            }
            for i in 0..3 {
                close(gradient[6 + i], fixture.next().unwrap());
            }
            for dim in 0..3 {
                for point in &objective.detector_points {
                    close(point[dim], fixture.next().unwrap());
                }
            }
            assert_eq!(objective.evaluation_counts(), (1, 1, 1));
        }
        assert!(fixture.next().is_none());
    }

    #[test]
    fn rebound_keeps_current_snapshot_and_passes_planes_to_next_objective() {
        use firefly_cost::ObstaclePenalty;
        let map = wall_map();
        let start = Endpoint {
            position: Vector3::new(1.0, 0.5, 0.5),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(9.0, 0.5, 0.5),
            ..start
        };
        let make_cost = |planes| MincoCost {
            weight_smoothness: 1.0,
            weight_time: 0.3,
            obstacle: Some((1.0, ObstaclePenalty::new(0.1, 0.5, 2.0, 5.0, 5, planes))),
            ..MincoCost::default()
        };
        let mut objective = MincoObjective::new(start, end, 1, make_cost(vec![Vec::new(); 6]))
            .with_detector(&map, 5, 10.0, false, vec![Vec::new(); 6], false, false);
        let mut reference = MincoObjective::new(start, end, 1, make_cost(vec![Vec::new(); 6]));
        let x = DVector::from_vec(vec![real_to_virtual(4.0)]);
        let mut gradient = DVector::zeros(1);
        let mut expected_gradient = DVector::zeros(1);
        let expected_cost = reference.evaluate_gradient(&x, &mut expected_gradient);
        for call in 1..=4 {
            assert_eq!(
                objective.evaluate_gradient(&x, &mut gradient),
                expected_cost
            );
            assert_eq!(gradient, expected_gradient);
            assert_eq!(objective.early_exit(), call == 4);
            assert_eq!(objective.evaluation_counts(), (call, call, 1));
        }
        assert!((objective.evaluate(&x) - expected_cost).abs() < 1e-10);
        assert!((objective.gradient(&x) - &expected_gradient).amax() < 1e-10);
        let planes = objective.take_planes();
        assert!(planes.iter().any(|point| !point.is_empty()));
        let mut next = MincoObjective::new(start, end, 1, make_cost(planes));
        let updated_cost = next.evaluate_gradient(&x, &mut gradient);
        assert!(updated_cost > expected_cost);
        assert!((&gradient - expected_gradient).amax() > 1e-6);
    }

    #[test]
    fn joint_callback_preserves_cache_counts_and_current_weights() {
        use firefly_cost::FeasibilityPenalty;
        let start = Endpoint {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(3.0, 0.5, 1.0),
            ..start
        };
        let cost = MincoCost {
            weight_smoothness: 1.0,
            weight_time: 0.3,
            feasibility: Some((0.2, FeasibilityPenalty::new(2.0, 3.0, 5.0))),
            weight_sqrvariance: 0.8,
            ..MincoCost::default()
        };
        let mut objective = MincoObjective::new(start, end, 2, cost);
        let mut x = DVector::from_vec(vec![1.2, 0.8, -0.2, 0.2, -0.5]);
        let mut joint_gradient = DVector::zeros(x.len());
        for change in 0..3 {
            if change == 1 {
                x[1] += 0.15;
            }
            if change == 2 {
                x[3] += 0.1;
            }
            let cost = objective.evaluate_gradient(&x, &mut joint_gradient);
            assert_eq!(
                objective.evaluation_counts(),
                (
                    change * 2 + 1,
                    change * 2 + 1,
                    if change < 2 { 1 } else { 2 }
                )
            );
            let split_cost = objective.evaluate(&x);
            let split_gradient = objective.gradient(&x);
            assert!((cost - split_cost).abs() < 1e-12 * split_cost.abs().max(1.0));
            assert!(
                (&joint_gradient - split_gradient).amax() < 1e-10 * joint_gradient.amax().max(1.0)
            );
        }
        let coefficients = objective.cache.as_ref().unwrap().2.coefficients().as_ptr();
        let counts = objective.evaluation_counts();
        objective.cost = MincoCost {
            weight_time: 7.0,
            ..MincoCost::default()
        };
        let cost = objective.evaluate_gradient(&x, &mut joint_gradient);
        assert!((cost - 7.0 * (virtual_to_real(x[3]) + virtual_to_real(x[4]))).abs() < 1e-12);
        assert_eq!(
            coefficients,
            objective.cache.as_ref().unwrap().2.coefficients().as_ptr()
        );
        assert_eq!(
            objective.evaluation_counts(),
            (counts.0 + 1, counts.1 + 1, counts.2)
        );
        assert_eq!(joint_gradient.rows(0, 3).amax(), 0.0);
    }

    #[test]
    fn callbacks_share_trajectory_but_recompute_changed_cost() {
        let start = Endpoint {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(3.0, 0.5, 1.0),
            ..start
        };
        let mut objective = MincoObjective::new(
            start,
            end,
            2,
            MincoCost {
                weight_time: 1.0,
                ..MincoCost::default()
            },
        );
        let mut x = DVector::from_vec(vec![1.2, 0.8, -0.2, 0.2, -0.5]);
        let first_gradient = objective.gradient(&x);
        let coefficients = objective.cache.as_ref().unwrap().2.coefficients().as_ptr();
        let cost = objective.evaluate(&x);
        assert!((cost - (1.22 + 1.0 / 1.625)).abs() < 1e-12);
        assert_eq!(
            coefficients,
            objective.cache.as_ref().unwrap().2.coefficients().as_ptr()
        );
        assert_eq!(first_gradient, objective.gradient(&x));
        objective.cost = MincoCost {
            weight_time: 3.0,
            ..MincoCost::default()
        };
        assert!((objective.evaluate(&x) - 3.0 * cost).abs() < 1e-12);
        assert!((objective.gradient(&x) - first_gradient * 3.0).norm() < 1e-12);
        assert_eq!(
            coefficients,
            objective.cache.as_ref().unwrap().2.coefficients().as_ptr()
        );

        objective.cost = MincoCost {
            weight_smoothness: 1.0,
            weight_time: 0.3,
            ..MincoCost::default()
        };
        for change in 0..3 {
            if change == 1 {
                x[1] += 0.15;
            }
            if change == 2 {
                x[3] += 0.1;
            }
            objective.evaluate(&x);
            let gradient = objective.gradient(&x);
            for i in 0..x.len() {
                let mut plus = x.clone();
                let mut minus = x.clone();
                plus[i] += 1e-6;
                minus[i] -= 1e-6;
                let numeric = (objective.evaluate(&plus) - objective.evaluate(&minus)) / 2e-6;
                assert!(
                    (numeric - gradient[i]).abs() < 1e-4 * gradient[i].abs().max(1.0),
                    "component {i}: {numeric} vs {}",
                    gradient[i]
                );
            }
        }
    }

    #[test]
    fn rebound_gate_uses_trajectory_samples_and_excludes_end_regions() {
        let start = Endpoint {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let waypoint = Vector3::new(2., 0., 0.);
        let angle = 35f64.to_radians();
        let end = Endpoint {
            position: waypoint + Vector3::new(2. * angle.cos(), 2. * angle.sin(), 0.),
            ..start
        };
        let trajectory = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[Point3::from(waypoint)], &[2., 2.])
            .unwrap()
            .solve()
            .unwrap();
        let mut points = constraint_sample_points(&trajectory, 5);
        assert!(constraint_points_straight_enough(&points));
        points[0] = Vector3::new(-100., 100., 0.);
        assert!(constraint_points_straight_enough(&points));
        points[5] = Vector3::new(-100., 100., 0.);
        assert!(!constraint_points_straight_enough(&points));
        assert!(constraint_points_straight_enough(&vec![
            Vector3::zeros();
            11
        ]));
    }

    /// 墙地图:体素 x=14..=26 全高占据(留 y 通道供 A* 绕行),
    /// 膨胀 1 格 → 占据区世界 x∈[3.25,7.0)。分辨率 0.25。
    fn wall_map() -> GridMap {
        let mut map = GridMapBuilder::new(0.25, [40, 20, 20])
            .with_obstacles_inflation(0.25 * 0.99)
            .build()
            .unwrap();
        for x in 14..=26 {
            for y in 0..20 {
                if (8..12).contains(&y) {
                    continue;
                }
                for z in 0..20 {
                    map.set_state([x, y, z], VoxelState::Occupied);
                }
            }
        }
        map.inflate_obstacles();
        map
    }

    fn straight_traj() -> firefly_trajectory::Trajectory {
        let start = Endpoint {
            position: Vector3::new(1.0, 0.5, 0.5),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(9.0, 0.5, 0.5),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[], &[4.0])
            .unwrap()
            .solve()
            .unwrap()
    }

    #[test]
    fn gate_blocks_rebound_while_initial_obstacle_on_wrong_side() {
        let map = wall_map();
        let traj = straight_traj();
        // k=5 单段 → 6 个约束点,穿墙点无平面覆盖 → roughly_check 必触发
        let points = constraint_sample_points(&traj, 5);
        assert!(points.len() > 4);
        let mut planes = vec![Vec::new(); points.len()];
        // 内部点放一个"错误侧"平面:(p−base)·direction < 0
        planes[2].push(Plane::new(
            points[2] - Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
        ));
        assert!(
            (points[2] - planes[2][0].point()).dot(&planes[2][0].normal()) < 0.0,
            "测试前提:该点必须在平面错误侧"
        );

        let mut det = ReboundDetector::new(&map, 5, 10.0, false, planes.clone(), true, false);
        assert!(!det.gate_latched(), "完整优化入口的初始锁存位为 false");
        assert!(
            !det.check(&traj),
            "初始障碍未避开时内循环检测必须被门控停用"
        );
        assert!(!det.gate_latched(), "未避开时不得锁存");

        // 同样的几何、开关关(单拓扑):不门控,检测照常触发
        let mut det_single =
            ReboundDetector::new(&map, 5, 10.0, false, planes.clone(), false, false);
        assert!(det_single.check(&traj), "开关关时不允许被 criterion 3 拦截");
    }

    #[test]
    fn gate_latches_once_obstacle_avoided() {
        let map = wall_map();
        let traj = straight_traj();
        let points = constraint_sample_points(&traj, 5);
        let mut planes = vec![Vec::new(); points.len()];
        // 同一点放"正确侧"平面:(p−base)·direction ≥ 0 → 已避开
        planes[2].push(Plane::new(
            points[2] + Vector3::new(0.0, 1.5, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
        ));
        let mut det = ReboundDetector::new(&map, 5, 10.0, false, planes, true, false);
        // 首次 check:判定已避开 → 锁存,并放行 roughly_check(穿墙点未覆盖)
        assert!(det.check(&traj), "已避开时应放行内循环检测");
        assert!(det.gate_latched(), "判定已避开后必须锁存为 true");
    }
}
