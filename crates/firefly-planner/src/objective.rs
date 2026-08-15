//! 时空联合优化目标。
//!
//! 变量 x = [q 展平 (3(M−1)), τ(M)]，τ = ln T 保证时间恒正。
//! 代价：firefly-cost 四项（平滑/时间/可行性/障碍），
//! 梯度经 `Minco::propagate_gradient` 传播到 {q, T}。

use firefly_cost::Cost;
use firefly_optimize::Objective;
use firefly_trajectory::{Endpoint, Minco, MincoBuilder, SolverOrder};
use nalgebra::{DMatrix, DVector, Point3, Vector3};

pub struct MincoObjective {
    start: Endpoint,
    end: Endpoint,
    pieces: usize,
    cost: Cost,
    // L-BFGS 对同一 x 先 evaluate 再 gradient：缓存 solve 结果省一半计算
    cache: Option<(DVector<f64>, Minco, firefly_trajectory::Trajectory)>,
}

impl MincoObjective {
    #[must_use]
    pub fn new(start: Endpoint, end: Endpoint, pieces: usize, cost: Cost) -> Self {
        Self {
            start,
            end,
            pieces,
            cost,
            cache: None,
        }
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

impl Objective for MincoObjective {
    fn evaluate(&mut self, x: &DVector<f64>) -> f64 {
        match self.solve_cached(x) {
            Some(traj) => self.cost.evaluate(&traj),
            None => f64::INFINITY,
        }
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
