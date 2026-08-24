//! 时空联合优化目标。
//!
//! 变量 x = [q 展平 (3(M−1)), τ(M)]，τ = ln T 保证时间恒正。
//! 代价：firefly-cost 各项(平滑/时间/可行/障碍/集群/队形/均匀)，
//! 梯度经 `Minco::propagate_gradient` 传播到 {q, T}。
//!
//! L-BFGS 内循环碰撞检测对齐官方 `poly_traj_optimizer.cpp`:
//! 代价回调内 `iter_num_ > 3 && smoo_cost/piece < 10` 时调用
//! `roughlyCheckConstraintPoints`——在约束点数组上检测未覆盖穿入点、
//! 沿数组 in/out 自由点搜索 + A\* 绕障 + 交点平面,命中即提前终止
//! (官方 `STOP_FOR_REBOUND`),由外层吸收新平面后重新优化。

use firefly_cost::{Cost, Penalty, SmoothnessPenalty};
use firefly_map::{GridMap, Plane};
use firefly_optimize::Objective;
use firefly_trajectory::{Endpoint, Minco, MincoBuilder, SolverOrder};
use nalgebra::{DMatrix, DVector, Point3, Vector3};

use crate::obstacles::{ObstacleScanner, constraint_sample_points};

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
    /// (一旦为 true 不再回头;每次内层优化开始复位为 false)。
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
        let scanner = ObstacleScanner::new(self.map)
            .with_samples(self.samples_per_piece)
            .with_max_vel(self.max_vel);
        let points = constraint_sample_points(traj, self.samples_per_piece);
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
        scanner.roughly_check(&mut self.astar, &points, &mut self.planes, self.touch_goal)
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
    cost: Cost,
    // L-BFGS 对同一 x 先 evaluate 再 gradient:缓存 solve 结果省一半计算
    cache: Option<(DVector<f64>, Minco, firefly_trajectory::Trajectory)>,
    detector: Option<ReboundDetector<'a>>,
    eval_count: usize,
    early_exit: bool,
}

impl<'a> MincoObjective<'a> {
    #[must_use]
    pub fn new(start: Endpoint, end: Endpoint, pieces: usize, cost: Cost) -> Self {
        Self {
            start,
            end,
            pieces,
            cost,
            cache: None,
            detector: None,
            eval_count: 0,
            early_exit: false,
        }
    }

    /// 挂载内循环碰撞检测(官方 allowRebound 条件:迭代 ≥4 且轨迹足够平滑;
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

    /// 取走本轮优化后的完整平面池(含检测到的新平面)。
    pub fn take_planes(&mut self) -> Vec<Vec<Plane>> {
        match &mut self.detector {
            Some(d) => d.take_planes(),
            None => Vec::new(),
        }
    }

    /// 重建 minco 并求解,命中缓存时复用。
    #[must_use]
    fn solve_cached(&mut self, x: &DVector<f64>) -> Option<firefly_trajectory::Trajectory> {
        if self
            .cache
            .as_ref()
            .is_some_and(|(last_x, _, _)| last_x == x)
        {
            return self.cache.as_ref().map(|(_, _, traj)| traj.clone());
        }
        let minco = self.rebuild(x).ok()?;
        let traj = minco.solve().ok()?;
        self.cache = Some((x.clone(), minco, traj.clone()));
        Some(traj)
    }

    #[must_use]
    pub fn unpack(&self, x: &DVector<f64>) -> (Vec<Vector3<f64>>, Vec<f64>) {
        let n_q = 3 * (self.pieces - 1);
        let mut q = Vec::with_capacity(self.pieces - 1);
        for i in 0..self.pieces - 1 {
            q.push(Vector3::new(x[i * 3], x[i * 3 + 1], x[i * 3 + 2]));
        }
        let t = (0..self.pieces)
            .map(|i| x[n_q + i].clamp(-8.0, 8.0).exp())
            .collect();
        (q, t)
    }

    /// # Errors
    ///
    /// `InvalidArgument`:x 中时长非正(对数参数化下不应发生)。
    pub fn rebuild(&self, x: &DVector<f64>) -> firefly_error::Result<Minco> {
        let (q, t) = self.unpack(x);
        let points: Vec<Point3<f64>> = q.iter().map(|v| Point3::from(*v)).collect();
        MincoBuilder::new(SolverOrder::MinimumJerk, self.start, self.end).build(&points, &t)
    }

    #[must_use]
    pub fn pack(&self, dq: &DMatrix<f64>, dt: &DVector<f64>, t: &[f64]) -> DVector<f64> {
        let mut g = DVector::zeros(3 * (self.pieces - 1) + self.pieces);
        for i in 0..self.pieces - 1 {
            for d in 0..3 {
                g[i * 3 + d] = dq[(d, i)];
            }
        }
        for i in 0..self.pieces {
            g[3 * (self.pieces - 1) + i] = dt[i] * t[i];
        }
        g
    }
}

impl Objective for MincoObjective<'_> {
    fn evaluate(&mut self, x: &DVector<f64>) -> f64 {
        self.eval_count += 1;
        let Some(traj) = self.solve_cached(x) else {
            return f64::INFINITY;
        };
        // 官方 allowRebound 条件:iter_num_ > 3 && smoo_cost/piece_num < 10.0
        if self.eval_count > 3
            && let Some(detector) = &mut self.detector
        {
            let smoo = SmoothnessPenalty.evaluate(&traj);
            if (smoo / traj.pieces() as f64) < 10.0 && detector.check(&traj) {
                self.early_exit = true;
            }
        }
        self.cost.evaluate(&traj)
    }

    fn early_exit(&self) -> bool {
        self.early_exit
    }

    fn gradient(&mut self, x: &DVector<f64>) -> DVector<f64> {
        let Some(traj) = self.solve_cached(x) else {
            return DVector::zeros(3 * (self.pieces - 1) + self.pieces);
        };
        let Some(minco) = self.rebuild(x).ok() else {
            return DVector::zeros(3 * (self.pieces - 1) + self.pieces);
        };
        let (d_f_d_c, d_f_d_t) = self.cost.gradient(&traj);
        let Ok((dq, dt)) = minco.propagate_gradient(&traj, &d_f_d_c, &d_f_d_t) else {
            return DVector::zeros(3 * (self.pieces - 1) + self.pieces);
        };
        let (_, t) = self.unpack(x);
        self.pack(&dq, &dt, &t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_map::{GridMapBuilder, VoxelState};

    /// 墙地图:体素 x=14..=26 全高占据(留 y 通道供 A* 绕行),
    /// 膨胀 1 格 → 占据区世界 x∈[3.25,7.0)。分辨率 0.25。
    fn wall_map() -> GridMap {
        let mut map = GridMapBuilder::new(0.25, [40, 20, 20]).build().unwrap();
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
        map.inflate_obstacles(0.25 * 0.99);
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
        assert!(!det.gate_latched(), "每次内层开始锁存位复位为 false");
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
