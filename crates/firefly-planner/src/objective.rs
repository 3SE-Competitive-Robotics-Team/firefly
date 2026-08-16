//! 时空联合优化目标。
//!
//! 变量 x = [q 展平 (3(M−1)), τ(M)]，τ = ln T 保证时间恒正。
//! 代价：firefly-cost 四项（平滑/时间/可行性/障碍），
//! 梯度经 `Minco::propagate_gradient` 传播到 {q, T}。

use firefly_cost::Cost;
use firefly_map::{GridMap, Plane};
use firefly_optimize::Objective;
use firefly_trajectory::{Endpoint, Minco, MincoBuilder, SolverOrder};
use nalgebra::{DMatrix, DVector, Point3, Vector3};

use crate::obstacles::{Hit, nearest_guide_point};

/// L-BFGS 内循环碰撞检测（官方 `roughlyCheckConstraintPoints`）：
/// 每轮求值检查前 2/3 约束点，发现未覆盖穿入点即请求提前终止，
/// 由外层吸收新平面后重新优化。
pub struct ReboundDetector<'a> {
    map: &'a GridMap,
    samples_per_piece: usize,
    planes: Vec<Vec<Plane>>,
    /// 段内局部 A*（官方每段重新搜绕行路径，约束方向指向它）。
    astar: firefly_search::Astar,
}

impl<'a> ReboundDetector<'a> {
    #[must_use]
    pub fn new(map: &'a GridMap, samples_per_piece: usize, planes: Vec<Vec<Plane>>) -> Self {
        Self {
            map,
            samples_per_piece,
            planes,
            astar: firefly_search::Astar::default(),
        }
    }

    /// 段内全点约束检测（官方 `roughlyCheckConstraintPoints` + `Assign parameters to
    /// each segment`）：
    /// 1. 遍历约束点（前 2/3），聚出未覆盖穿入的碰撞段；
    /// 2. 每段做**局部 A\***（官方 `AstarSearch(in, out)`）搜绕行路径；
    /// 3. 段内每个穿入点生成约束，方向指向局部绕行路径（官方
    ///    `direction = (A\*路径交点 − 轨迹点).normalized()`，改变拓扑而非简单推离）。
    ///
    /// 返回 `None` 表示官方 `allowRebound` criterion 2 不满足（约束点序列最小
    /// 转向角 < 30°）：大变形轨迹的局部碰撞检测无意义，等 L-BFGS 继续优化。
    #[must_use]
    pub fn check(&mut self, traj: &firefly_trajectory::Trajectory) -> Option<Vec<Hit>> {
        let two_thirds = (traj.durations().len() * 2 / 3).max(1);
        let res = self.map.resolution();
        let n_per_piece = self.samples_per_piece + 1;
        let total = two_thirds * n_per_piece;
        let sample = |idx: usize| -> firefly_trajectory::Sample {
            let i = idx / n_per_piece;
            let j = idx % n_per_piece;
            let tau = j as f64 / self.samples_per_piece as f64;
            let mut t = 0.0;
            for d in traj.durations().iter().take(i) {
                t += d;
            }
            t += tau * traj.durations()[i];
            traj.eval(t)
        };
        // 单次遍历同时算角度判据与穿入标记
        let mut min_product: f64 = 1.0;
        let mut prev: Option<Vector3<f64>> = None;
        let mut prev_dir: Option<Vector3<f64>> = None;
        let mut in_segment = false;
        let mut segments: Vec<(usize, usize)> = Vec::new(); // 段内索引范围（含未穿入边界）
        let mut seg_start = 0usize;
        let mut occupied_flags = vec![false; total];
        for (point_index, occupied) in occupied_flags.iter_mut().enumerate() {
            let p = sample(point_index).position;
            if let Some(q) = prev {
                let dir = (p - q).normalize();
                if let Some(d0) = prev_dir {
                    min_product = min_product.min(d0.dot(&dir));
                }
                prev_dir = Some(dir);
            }
            prev = Some(p);
            let covered = self.planes[point_index]
                .iter()
                .any(|pl| (p - pl.point()).dot(&pl.normal()) < res);
            *occupied = self.map.is_occupied_inflated(p) && !covered;
            if *occupied && !in_segment {
                in_segment = true;
                seg_start = point_index;
            } else if !*occupied && in_segment {
                in_segment = false;
                segments.push((seg_start, point_index));
            }
        }
        if in_segment {
            segments.push((seg_start, total - 1));
        }
        if min_product < 0.87 {
            return None;
        }
        if segments.is_empty() {
            return Some(Vec::new());
        }

        let mut hits = Vec::new();
        for (seg_in, seg_out) in segments {
            // 段边界：前一个自由点（in）与后一个自由点（out），官方 AstarSearch(in, out)
            let in_idx = (0..seg_in).rev().find(|&k| !occupied_flags[k]).unwrap_or(0);
            let out_idx = (seg_out + 1..total)
                .find(|&k| !occupied_flags[k])
                .unwrap_or(seg_out);
            let (start_pt, end_pt) = (sample(out_idx).position, sample(in_idx).position);
            let Ok(local_path) = self.astar.search(self.map, start_pt, end_pt) else {
                continue;
            };
            let path_pts = local_path.points();
            // 段内每个未覆盖穿入点：方向指向局部 A* 路径最近点（官方交点方向）
            for (idx, &occupied) in occupied_flags[seg_in..=seg_out].iter().enumerate() {
                if !occupied {
                    continue;
                }
                let point_index = seg_in + idx;
                let s = sample(point_index);
                if let Some(nearest) = nearest_guide_point(path_pts, s.position) {
                    hits.push(Hit {
                        point_index,
                        sample: s,
                        guide_point: nearest,
                    });
                }
            }
        }
        Some(hits)
    }
}

pub struct MincoObjective<'a> {
    start: Endpoint,
    end: Endpoint,
    pieces: usize,
    cost: Cost,
    // L-BFGS 对同一 x 先 evaluate 再 gradient：缓存 solve 结果省一半计算
    cache: Option<(DVector<f64>, Minco, firefly_trajectory::Trajectory)>,
    detector: Option<ReboundDetector<'a>>,
    eval_count: usize,
    early_exit: bool,
    pending: Vec<Hit>,
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
            pending: Vec::new(),
        }
    }

    /// 挂载内循环碰撞检测（官方 allowRebound：迭代 ≥3 后启用）。
    #[must_use]
    pub fn with_detector(
        mut self,
        map: &'a GridMap,
        samples_per_piece: usize,
        planes: Vec<Vec<Plane>>,
    ) -> Self {
        self.detector = Some(ReboundDetector::new(map, samples_per_piece, planes));
        self
    }

    /// 取走本次优化中检测到的新穿入点（外层并入平面池）。
    pub fn take_pending(&mut self) -> Vec<Hit> {
        std::mem::take(&mut self.pending)
    }

    /// 重建 minco 并求解，命中缓存时复用。
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
    /// `InvalidArgument`：x 中时长非正（对数参数化下不应发生）。
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
        // 官方 allowRebound criterion 1+2：前 3 次求值不检测；轨迹大转角不检测
        if self.eval_count >= 3
            && let Some(detector) = &mut self.detector
            && let Some(hits) = detector.check(&traj)
            && !hits.is_empty()
        {
            self.pending.extend(hits);
            self.early_exit = true;
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
