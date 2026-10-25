//! MINCO 轨迹类。
//!
//! 对 m 维 s 阶积分链（本系统 s = 3，最小加加速度），
//! M 段轨迹，每段 2s−1 = 5 次多项式，整体 C^{s−1} 连续。
//!
//! 核心映射（MINCO 论文 Theorem 2，Eq. 48–55）：
//! `M(T) c = b(q)`，M 为块下双对角带形矩阵，c 由带形 LU 求解。
//! 参数 {q, T} 与系数 c 之间可线性复杂度地传播梯度。

use crate::banded::{BandedMatrix, BandedSolver};
use nalgebra::DMatrix;
use nalgebra::DVector;
use nalgebra::Vector3;
use std::sync::Arc;

pub type Point3 = nalgebra::Point3<f64>;

const POLY_DEGREE: usize = 5;
const COEFFS_PER_PIECE: usize = POLY_DEGREE + 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolverOrder {
    MinimumJerk = 3,
}

#[derive(Debug, Clone, Copy)]
pub struct Endpoint {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct Minco {
    order: SolverOrder,
    pieces: usize,
    start: Endpoint,
    end: Endpoint,
    q: DMatrix<f64>,
    t: DVector<f64>,
    /// 不可变 LU 只依赖段时长；正向与伴随求解共用。
    solver: Arc<BandedSolver>,
}

#[derive(Debug)]
pub struct MincoBuilder {
    order: SolverOrder,
    start: Endpoint,
    end: Endpoint,
}

impl MincoBuilder {
    #[must_use]
    pub fn new(order: SolverOrder, start: Endpoint, end: Endpoint) -> Self {
        Self { order, start, end }
    }

    /// # Errors
    ///
    /// `InvalidArgument`：段数为零、中间点数不匹配或时长非正/非有限。
    /// `Convergence`：MINCO 系统奇异。
    pub fn build(self, waypoints: &[Point3], durations: &[f64]) -> firefly_error::Result<Minco> {
        let pieces = durations.len();
        let interior = waypoints.len();
        if pieces == 0 {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "at least one piece is required",
            ));
        }
        if interior + 1 != pieces {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "waypoint count must be piece count minus one",
            )
            .with_context("pieces", pieces)
            .with_context("waypoints", interior));
        }
        if durations.iter().any(|t| !t.is_finite() || *t <= 0.0) {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "durations must be positive",
            ));
        }
        let q = DMatrix::from_fn(3, interior, |r, c| waypoints[c][r]);
        let t = DVector::from_fn(pieces, |r, _| durations[r]);
        let solver = Arc::new(BandedSolver::new_owned(Minco::build_banded(durations))?);
        Ok(Minco {
            order: self.order,
            pieces,
            start: self.start,
            end: self.end,
            q,
            t,
            solver,
        })
    }
}

impl Minco {
    /// 更新固定段数的参数；只有时长变化才重新组装、分解系统。
    /// 失败保留完整的原参数及 LU。
    /// # Errors
    /// `InvalidArgument`：点数/段数不匹配或时长非正/非有限。
    /// `Convergence`：MINCO 系统奇异。
    pub fn update(
        &mut self,
        start: Endpoint,
        end: Endpoint,
        waypoints: &[Point3],
        durations: &[f64],
    ) -> firefly_error::Result<()> {
        if durations.len() != self.pieces
            || waypoints.len() + 1 != self.pieces
            || durations.iter().any(|t| !t.is_finite() || *t <= 0.0)
        {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "MINCO update requires matching piece count and positive finite durations",
            ));
        }
        if durations != self.t.as_slice() {
            let solver = Arc::new(BandedSolver::new_owned(Self::build_banded(durations))?);
            self.t.as_mut_slice().copy_from_slice(durations);
            self.solver = solver;
        }
        self.start = start;
        self.end = end;
        for (i, point) in waypoints.iter().enumerate() {
            self.q.set_column(i, &point.coords);
        }
        Ok(())
    }

    #[must_use]
    pub fn order(&self) -> SolverOrder {
        self.order
    }

    #[must_use]
    pub fn start(&self) -> Endpoint {
        self.start
    }

    #[must_use]
    pub fn end(&self) -> Endpoint {
        self.end
    }

    #[must_use]
    pub fn pieces(&self) -> usize {
        self.pieces
    }

    #[must_use]
    pub fn duration(&self) -> f64 {
        self.t.sum()
    }

    #[must_use]
    pub fn piece_duration(&self, i: usize) -> f64 {
        self.t[i]
    }

    /// # Errors
    ///
    /// `OutOfRange`：索引超出中间点范围。
    pub fn waypoint(&self, i: usize) -> firefly_error::Result<Point3> {
        if i >= self.pieces - 1 {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::OutOfRange,
                "waypoint index out of range",
            )
            .with_context("index", i)
            .with_context("count", self.pieces - 1));
        }
        Ok(Point3::new(self.q[(0, i)], self.q[(1, i)], self.q[(2, i)]))
    }

    pub fn waypoints(&self) -> impl Iterator<Item = Point3> + '_ {
        (0..self.pieces - 1).map(|i| Point3::new(self.q[(0, i)], self.q[(1, i)], self.q[(2, i)]))
    }

    #[fastrace::trace]
    /// # Errors
    ///
    /// 系统矩阵的奇异性在构建或更新时检查，此处使用已验证的 LU。
    pub fn solve(&self) -> firefly_error::Result<Trajectory> {
        let mut out = Trajectory {
            coefficients: DMatrix::zeros(COEFFS_PER_PIECE * self.pieces, 3),
            durations: self.t.clone(),
        };
        self.solve_into(&mut out)?;
        Ok(out)
    }

    /// 就地求解到已有 `Trajectory`：复用系数与时长缓冲。
    ///
    /// 一次优化内段数不变，形状稳定；形状不符时按需重建。
    ///
    /// # Errors
    ///
    /// 系统矩阵的奇异性在构建或更新时检查。
    pub fn solve_into(&self, out: &mut Trajectory) -> firefly_error::Result<()> {
        let n = COEFFS_PER_PIECE * self.pieces;
        if out.coefficients.shape() == (n, 3) {
            out.coefficients.fill(0.0);
        } else {
            out.coefficients = DMatrix::zeros(n, 3);
        }
        if out.durations.len() == self.pieces {
            out.durations.copy_from(&self.t);
        } else {
            out.durations = self.t.clone();
        }
        self.build_rhs_into(&mut out.coefficients);
        self.solver.solve_in_place(&mut out.coefficients);
        Ok(())
    }

    /// 从目标对 (c, T) 的偏导传播到对 (q, T) 的偏导。
    /// `d_f_d_c`: 6M × 3，`d_f_d_t`: M。
    /// 返回 (`d_f_d_q`: 3 × (M−1), `d_f_d_t`: M)。
    #[fastrace::trace]
    /// # Errors
    ///
    /// 系统矩阵的奇异性在构建或更新时检查，此处使用已验证的 LU。
    pub fn propagate_gradient(
        &self,
        trajectory: &Trajectory,
        d_f_d_c: &DMatrix<f64>,
        d_f_d_t: &DVector<f64>,
    ) -> firefly_error::Result<(DMatrix<f64>, DVector<f64>)> {
        let g = self.solver.solve_transpose(d_f_d_c);

        let mut d_f_d_q = DMatrix::zeros(3, self.pieces - 1);
        for i in 0..self.pieces - 1 {
            // 官方布局：qᵢ 在 b 的第 6i+5 行（位置约束行）
            let row = 6 * i + 5;
            for dim in 0..3 {
                d_f_d_q[(dim, i)] = g[(row, dim)];
            }
        }

        let mut d_f_d_t_out = d_f_d_t.clone();
        for i in 0..self.pieces {
            let w = self.dm_dt_c(i, &trajectory.coefficients);
            for dim in 0..3 {
                let mut dot = 0.0;
                let first_row = if i + 1 < self.pieces {
                    6 * i + 3
                } else {
                    6 * self.pieces - 3
                };
                let rows = if i + 1 < self.pieces { 6 } else { 3 };
                for r in 0..rows {
                    dot += g[(first_row + r, dim)] * w[r][dim];
                }
                d_f_d_t_out[i] -= dot;
            }
        }
        Ok((d_f_d_q, d_f_d_t_out))
    }

    /// ∂M/∂Tᵢ · c 的非零块：中间段六行，末段三行。
    fn dm_dt_c(&self, i: usize, c: &DMatrix<f64>) -> [[f64; 3]; 6] {
        let mut w = [[0.0; 3]; 6];
        let ti = self.t[i];
        let base = 6 * i;
        let interior = i + 1 < self.pieces;
        let offset = if interior {
            for dim in 0..3 {
                w[0][dim] = 24.0 * c[(base + 4, dim)] + 120.0 * ti * c[(base + 5, dim)];
                w[1][dim] = 120.0 * c[(base + 5, dim)];
            }
            2
        } else {
            0
        };
        let b1 = beta_derivative(1, ti);
        let b2 = beta_derivative(2, ti);
        let b3 = beta_derivative(3, ti);
        let velocity_row = if interior { 4 } else { 1 };
        for dim in 0..3 {
            for k in 0..COEFFS_PER_PIECE {
                let v = c[(base + k, dim)];
                w[offset][dim] += b1[k] * v;
                if interior {
                    w[offset + 1][dim] += b1[k] * v;
                }
                w[velocity_row][dim] += b2[k] * v;
                w[velocity_row + 1][dim] += b3[k] * v;
            }
        }
        w
    }

    /// 构造带形系统矩阵（官方 EGO-Planner-v2 `MinJerkOpt` 布局）。
    ///
    /// 行序：起点 PVA / 中间块 {jerk 连续, snap 连续, 位置, 位置连续,
    /// 速度连续, 加速度连续} / 终点 PVA。带宽 (6, 6)，
    /// 对角全非零 → 无主元带形 LU 稳定（官方注释：NO PIVOT for efficiency）。
    fn build_banded(durations: &[f64]) -> BandedMatrix {
        let pieces = durations.len();
        let n = COEFFS_PER_PIECE * pieces;
        let mut m = BandedMatrix::new(n, 6, 6);

        // 起点：位置/速度/加速度
        m.set(0, 0, 1.0);
        m.set(1, 1, 1.0);
        m.set(2, 2, 2.0);

        for (i, &t) in durations.iter().take(pieces - 1).enumerate() {
            let row = 6 * i + 3;
            let t2 = t * t;
            let t3 = t2 * t;
            let t4 = t3 * t;
            let t5 = t4 * t;
            // jerk 连续：p'''_i(T) − p'''_{i+1}(0) = 0
            m.set(row, row, 6.0);
            m.set(row, row + 1, 24.0 * t);
            m.set(row, row + 2, 60.0 * t2);
            m.set(row, row + 6, -6.0);
            // snap 连续：p⁗_i(T) − p⁗_{i+1}(0) = 0
            m.set(row + 1, row + 1, 24.0);
            m.set(row + 1, row + 2, 120.0 * t);
            m.set(row + 1, row + 7, -24.0);
            // 位置：p_i(T) = q_i（b 侧填 q）
            m.set(row + 2, row - 3, 1.0);
            m.set(row + 2, row - 2, t);
            m.set(row + 2, row - 1, t2);
            m.set(row + 2, row, t3);
            m.set(row + 2, row + 1, t4);
            m.set(row + 2, row + 2, t5);
            // 位置连续：p_i(T) − p_{i+1}(0) = 0
            m.set(row + 3, row - 3, 1.0);
            m.set(row + 3, row - 2, t);
            m.set(row + 3, row - 1, t2);
            m.set(row + 3, row, t3);
            m.set(row + 3, row + 1, t4);
            m.set(row + 3, row + 2, t5);
            m.set(row + 3, row + 3, -1.0);
            // 速度连续：ṗ_i(T) − ṗ_{i+1}(0) = 0
            m.set(row + 4, row - 2, 1.0);
            m.set(row + 4, row - 1, 2.0 * t);
            m.set(row + 4, row, 3.0 * t2);
            m.set(row + 4, row + 1, 4.0 * t3);
            m.set(row + 4, row + 2, 5.0 * t4);
            m.set(row + 4, row + 4, -1.0);
            // 加速度连续：p̈_i(T) − p̈_{i+1}(0) = 0
            m.set(row + 5, row - 1, 2.0);
            m.set(row + 5, row, 6.0 * t);
            m.set(row + 5, row + 1, 12.0 * t2);
            m.set(row + 5, row + 2, 20.0 * t3);
            m.set(row + 5, row + 5, -2.0);
        }

        // 终点：位置/速度/加速度
        let t = durations[pieces - 1];
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;
        let base = 6 * (pieces - 1);
        m.set(n - 3, base, 1.0);
        m.set(n - 3, base + 1, t);
        m.set(n - 3, base + 2, t2);
        m.set(n - 3, base + 3, t3);
        m.set(n - 3, base + 4, t4);
        m.set(n - 3, base + 5, t5);
        m.set(n - 2, base + 1, 1.0);
        m.set(n - 2, base + 2, 2.0 * t);
        m.set(n - 2, base + 3, 3.0 * t2);
        m.set(n - 2, base + 4, 4.0 * t3);
        m.set(n - 2, base + 5, 5.0 * t4);
        m.set(n - 1, base + 2, 2.0);
        m.set(n - 1, base + 3, 6.0 * t);
        m.set(n - 1, base + 4, 12.0 * t2);
        m.set(n - 1, base + 5, 20.0 * t3);
        m
    }

    fn build_rhs_into(&self, b: &mut DMatrix<f64>) {
        let n = COEFFS_PER_PIECE * self.pieces;
        for (k, state) in [
            self.start.position,
            self.start.velocity,
            self.start.acceleration,
        ]
        .into_iter()
        .enumerate()
        {
            for dim in 0..3 {
                b[(k, dim)] = state[dim];
            }
        }
        for i in 1..self.pieces {
            let row = 6 * (i - 1) + 5;
            for dim in 0..3 {
                b[(row, dim)] = self.q[(dim, i - 1)];
            }
        }
        for (k, state) in [self.end.position, self.end.velocity, self.end.acceleration]
            .into_iter()
            .enumerate()
        {
            for dim in 0..3 {
                b[(n - 3 + k, dim)] = state[dim];
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct Trajectory {
    coefficients: DMatrix<f64>,
    durations: DVector<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct Sample {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
    pub jerk: Vector3<f64>,
    pub snap: Vector3<f64>,
}

impl Trajectory {
    #[must_use]
    pub fn pieces(&self) -> usize {
        self.durations.len()
    }

    #[must_use]
    pub fn coefficients(&self) -> &DMatrix<f64> {
        &self.coefficients
    }

    pub fn coefficients_mut(&mut self) -> &mut DMatrix<f64> {
        &mut self.coefficients
    }

    #[must_use]
    pub fn durations(&self) -> &DVector<f64> {
        &self.durations
    }

    pub fn durations_mut(&mut self) -> &mut DVector<f64> {
        &mut self.durations
    }

    #[must_use]
    pub fn duration(&self) -> f64 {
        self.durations.sum()
    }

    /// 全局时刻采样。注意：段边界时刻（tau ∈ {0, 1}）的归属有歧义，
    /// jerk 在 waypoint 处不连续时取到相邻段的单侧极限；
    /// 需要固定段归属的逐段采样（如梯度累加）必须用 [`Self::eval_piece`]。
    #[must_use]
    pub fn eval(&self, time: f64) -> Sample {
        let mut t = time;
        let mut piece = 0usize;
        for i in 0..self.durations.len() {
            if t < self.durations[i] || i == self.durations.len() - 1 {
                piece = i;
                break;
            }
            t -= self.durations[i];
        }
        self.eval_piece(piece, t)
    }

    /// 指定段的局部时刻采样（`t_local` ∈ [0, `piece_duration(piece)`]）。
    /// 逐段采样必须用此方法而非 [`Self::eval`]：边界时刻经全局时间搜索
    /// 定位分段时有归属歧义，而梯度累加按 (piece, `beta(t_local)`) 归因，
    /// 两端约定必须一致。
    #[must_use]
    pub fn eval_piece(&self, piece: usize, t_local: f64) -> Sample {
        let t = t_local;
        let mut sample = Sample {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            jerk: Vector3::zeros(),
            snap: Vector3::zeros(),
        };
        let beta = beta_derivatives(t);
        for (derivative, out) in [
            &mut sample.position,
            &mut sample.velocity,
            &mut sample.acceleration,
            &mut sample.jerk,
            &mut sample.snap,
        ]
        .into_iter()
        .enumerate()
        {
            for (k, value) in beta[derivative].into_iter().enumerate() {
                let row = piece * COEFFS_PER_PIECE + k;
                out.x += self.coefficients[(row, 0)] * value;
                out.y += self.coefficients[(row, 1)] * value;
                out.z += self.coefficients[(row, 2)] * value;
            }
        }
        sample
    }
}

fn beta_derivatives(t: f64) -> [[f64; COEFFS_PER_PIECE]; 5] {
    let mut powers = [1.0; COEFFS_PER_PIECE];
    for i in 1..COEFFS_PER_PIECE {
        powers[i] = powers[i - 1] * t;
    }
    let mut out = [[0.0; COEFFS_PER_PIECE]; 5];
    out[0].copy_from_slice(&powers);
    for j in 1..COEFFS_PER_PIECE {
        out[1][j] = j as f64 * powers[j - 1];
    }
    for j in 2..COEFFS_PER_PIECE {
        out[2][j] = (j * (j - 1)) as f64 * powers[j - 2];
    }
    for j in 3..COEFFS_PER_PIECE {
        out[3][j] = (j * (j - 1) * (j - 2)) as f64 * powers[j - 3];
    }
    for j in 4..COEFFS_PER_PIECE {
        out[4][j] = (j * (j - 1) * (j - 2) * (j - 3)) as f64 * powers[j - 4];
    }
    out
}

fn beta_derivative(order: usize, t: f64) -> [f64; COEFFS_PER_PIECE] {
    beta_derivatives(t)[order]
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    fn endpoint(p: [f64; 3], v: [f64; 3], a: [f64; 3]) -> Endpoint {
        Endpoint {
            position: Vector3::new(p[0], p[1], p[2]),
            velocity: Vector3::new(v[0], v[1], v[2]),
            acceleration: Vector3::new(a[0], a[1], a[2]),
        }
    }

    fn sample() -> Minco {
        let start = endpoint([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let end = endpoint([2.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[Point3::new(1.0, 0.0, 0.0)], &[1.0, 1.0])
            .unwrap()
    }

    #[test]
    fn builder_rejects_invalid_inputs() {
        let start = endpoint([0.0; 3], [0.0; 3], [0.0; 3]);
        let end = endpoint([1.0; 3], [0.0; 3], [0.0; 3]);
        let b = || MincoBuilder::new(SolverOrder::MinimumJerk, start, end);
        // 中间点数量与时长数量不匹配
        let r = b().build(
            &[Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
            &[1.0, 1.0],
        );
        assert_eq!(
            r.unwrap_err().kind(),
            firefly_error::ErrorKind::InvalidArgument
        );
        // 非正时长
        let r = b().build(&[Point3::origin()], &[0.0]);
        assert_eq!(
            r.unwrap_err().kind(),
            firefly_error::ErrorKind::InvalidArgument
        );
        // 空输入
        let r = b().build(&[], &[]);
        assert_eq!(
            r.unwrap_err().kind(),
            firefly_error::ErrorKind::InvalidArgument
        );
    }

    #[test]
    fn accessors() {
        let m = sample();
        assert_eq!(m.pieces(), 2);
        assert_eq!(m.duration(), 2.0);
        assert_eq!(m.waypoint(0).unwrap(), Point3::new(1.0, 0.0, 0.0));
        assert_eq!(m.waypoints().count(), 1);
        assert_eq!(
            m.waypoint(1).unwrap_err().kind(),
            firefly_error::ErrorKind::OutOfRange
        );
    }

    #[test]
    fn trajectory_satisfies_constraints() {
        let m = sample();
        let traj = m.solve().unwrap();
        // 起止边界
        let s0 = traj.eval(0.0);
        assert!((s0.position - m.start.position).norm() < 1e-9);
        assert!((s0.velocity - m.start.velocity).norm() < 1e-9);
        assert!((s0.acceleration - m.start.acceleration).norm() < 1e-9);
        let sf = traj.eval(traj.duration());
        assert!((sf.position - m.end.position).norm() < 1e-9);
        assert!((sf.velocity - m.end.velocity).norm() < 1e-9);
        assert!((sf.acceleration - m.end.acceleration).norm() < 1e-9);
        // 途经中间点
        let s = traj.eval(1.0);
        assert!((s.position - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-9);
        // 分段处 C2 连续
        let t = m.piece_duration(0);
        let left = traj.eval(t - 1e-8);
        let right = traj.eval(t + 1e-8);
        assert!((left.position - right.position).norm() < 1e-6);
        assert!((left.velocity - right.velocity).norm() < 1e-6);
        assert!((left.acceleration - right.acceleration).norm() < 1e-6);
    }

    #[test]
    fn lu_reuse_and_parameter_updates_preserve_constraints() {
        let mut minco = sample();
        let original_lu = Arc::clone(&minco.solver);
        let start = endpoint([0.2, -0.3, 0.4], [0.1, 0.2, -0.1], [0.03, 0.02, 0.01]);
        let end = endpoint([3.0, 0.5, 0.8], [-0.2, 0.1, 0.3], [0.01, -0.03, 0.02]);
        let waypoint = Point3::new(1.2, 0.6, -0.2);
        for durations in [[1.0, 1.0], [0.8, 1.4]] {
            minco.update(start, end, &[waypoint], &durations).unwrap();
            assert_eq!(
                Arc::ptr_eq(&original_lu, &minco.solver),
                durations == [1.0, 1.0]
            );
            let trajectory = minco.solve().unwrap();
            let lu = Arc::clone(&minco.solver);
            minco
                .propagate_gradient(&trajectory, &DMatrix::zeros(12, 3), &DVector::zeros(2))
                .unwrap();
            assert!(Arc::ptr_eq(&lu, &minco.solver));
            for (sample, expected) in [
                (trajectory.eval(0.0), start),
                (trajectory.eval(trajectory.duration()), end),
            ] {
                assert!((sample.position - expected.position).norm() < 1e-9);
                assert!((sample.velocity - expected.velocity).norm() < 1e-9);
                assert!((sample.acceleration - expected.acceleration).norm() < 1e-9);
            }
            assert!(
                (trajectory.eval_piece(0, durations[0]).position - waypoint.coords).norm() < 1e-9
            );
            let left = trajectory.eval_piece(0, durations[0]);
            let right = trajectory.eval_piece(1, 0.0);
            assert!((left.position - right.position).norm() < 1e-9);
            assert!((left.velocity - right.velocity).norm() < 1e-9);
            assert!((left.acceleration - right.acceleration).norm() < 1e-9);
            let before = trajectory.coefficients.clone();
            for invalid in [[0.0, 1.0], [f64::NAN, 1.0], [1.0, f64::INFINITY]] {
                assert!(
                    minco
                        .update(minco.start, minco.end, &[Point3::origin()], &invalid)
                        .is_err()
                );
                assert!(Arc::ptr_eq(&lu, &minco.solver));
                assert_eq!(minco.solve().unwrap().coefficients, before);
            }
        }
    }

    #[test]
    fn updated_multi_piece_gradient_matches_all_axis_differences() {
        let start = endpoint([0.2, -0.3, 0.4], [0.1, 0.2, -0.1], [0.03, 0.02, 0.01]);
        let end = endpoint([3.0, 0.5, 0.8], [-0.2, 0.1, 0.3], [0.01, -0.03, 0.02]);
        let mut q = [Point3::new(1.0, 0.4, 0.2), Point3::new(2.0, -0.2, 0.9)];
        let mut t = [0.8, 1.2, 1.4];
        let mut minco = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&q, &t)
            .unwrap();
        let dc = DMatrix::from_fn(18, 3, |r, c| ((r * 3 + c) as f64).sin() * 0.1);
        let explicit_dt = DVector::from_vec(vec![0.05, 0.1, -0.03]);
        let loss = |q: &[Point3], t: &[f64]| {
            let trajectory = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
                .build(q, t)
                .unwrap()
                .solve()
                .unwrap();
            dc.component_mul(trajectory.coefficients()).sum()
                + explicit_dt.dot(trajectory.durations())
        };
        for change in 0..3 {
            if change == 1 {
                q[1].y += 0.3;
            }
            if change == 2 {
                t[0] += 0.2;
            }
            minco.update(start, end, &q, &t).unwrap();
            let trajectory = minco.solve().unwrap();
            let (dq, dt) = minco
                .propagate_gradient(&trajectory, &dc, &explicit_dt)
                .unwrap();
            let h = 1e-6;
            for i in 0..2 {
                for dim in 0..3 {
                    let mut plus = q;
                    let mut minus = q;
                    plus[i][dim] += h;
                    minus[i][dim] -= h;
                    let numeric = (loss(&plus, &t) - loss(&minus, &t)) / (2.0 * h);
                    assert!(
                        (numeric - dq[(dim, i)]).abs() < 1e-5,
                        "q[{i},{dim}] {numeric} vs {}",
                        dq[(dim, i)]
                    );
                }
            }
            for i in 0..3 {
                let mut plus = t;
                let mut minus = t;
                plus[i] += h;
                minus[i] -= h;
                let numeric = (loss(&q, &plus) - loss(&q, &minus)) / (2.0 * h);
                assert!(
                    (numeric - dt[i]).abs() < 1e-5,
                    "t[{i}] {numeric} vs {}",
                    dt[i]
                );
            }
        }
    }

    #[test]
    fn gradient_matches_numerical() {
        let m = sample();
        let traj = m.solve().unwrap();

        let d_f_d_c = DMatrix::from_fn(12, 3, |r, c| {
            let v = (r * 3 + c) as f64;
            v.sin() * 0.1
        });
        let d_f_d_t = DVector::from_fn(2, |r, _| 0.05 * (r + 1) as f64);

        let (dq, dt) = m.propagate_gradient(&traj, &d_f_d_c, &d_f_d_t).unwrap();
        let f = |q: &[f64; 3], t: &[f64; 2]| -> f64 {
            let start = m.start;
            let end = m.end;
            let qp = [Point3::new(q[0], q[1], q[2])];
            let mm = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
                .build(&qp, t)
                .unwrap();
            let tr = mm.solve().unwrap();
            let mut total = 0.0;
            for r in 0..12 {
                for c in 0..3 {
                    total += d_f_d_c[(r, c)] * tr.coefficients[(r, c)];
                }
            }
            for r in 0..2 {
                total += d_f_d_t[r] * tr.durations[r];
            }
            total
        };

        let q0 = [1.0, 0.0, 0.0];
        let t0 = [1.0, 1.0];
        let h = 1e-6;
        for (i, dq_i) in dq.column_iter().enumerate() {
            let mut qp = q0;
            qp[i] += h;
            let fwd = f(&qp, &t0);
            qp[i] -= 2.0 * h;
            let bwd = f(&qp, &t0);
            let numeric = (fwd - bwd) / (2.0 * h);
            assert!(
                (numeric - dq_i[0]).abs() < 1e-4,
                "dq[{i}] analytic={} numeric={numeric}",
                dq_i[0]
            );
        }
        for (i, dt_i) in dt.iter().enumerate() {
            let mut tp = t0;
            tp[i] += h;
            let fwd = f(&q0, &tp);
            tp[i] -= 2.0 * h;
            let bwd = f(&q0, &tp);
            let numeric = (fwd - bwd) / (2.0 * h);
            assert!(
                (numeric - dt_i).abs() < 1e-4,
                "dt[{i}] analytic={dt_i} numeric={numeric}"
            );
        }
    }

    #[test]
    fn single_piece_matches_closed_form_minimum_jerk() {
        // 单段最小 jerk 闭式解（由边界方程组直接推导）：
        // p(t) = p0 + v0 t + a0 t²/2 + c3 t³ + c4 t⁴ + c5 t⁵
        // c3 = (20Δp − (8vf+12v0)T + (af−3a0)T²) / (2T³)
        // c4 = (−15Δp + (7vf+8v0)T + (−af+1.5a0)T²) / T⁴
        // c5 = (12Δp − (6vf+6v0)T + (af−a0)T²) / (2T⁵)，Δp = pf−p0
        let p0 = Vector3::new(0.0, 1.0, 2.0);
        let v0 = Vector3::new(1.0, -0.5, 0.2);
        let a0 = Vector3::new(0.1, 0.3, -0.2);
        let pf = Vector3::new(3.0, 2.0, 1.0);
        let vf = Vector3::new(0.0, 0.5, -0.3);
        let af = Vector3::new(-0.2, 0.1, 0.4);
        let t = 2.5;

        let start = Endpoint {
            position: p0,
            velocity: v0,
            acceleration: a0,
        };
        let end = Endpoint {
            position: pf,
            velocity: vf,
            acceleration: af,
        };
        let m = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[], &[t])
            .unwrap();
        let traj = m.solve().unwrap();

        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;
        for dim in 0..3 {
            let (d0, d1, d2) = (p0[dim], v0[dim], a0[dim]);
            let (df, df1, df2) = (pf[dim], vf[dim], af[dim]);
            let dp = df - d0;
            let c3 = (20.0 * dp - (8.0 * df1 + 12.0 * d1) * t + (df2 - 3.0 * d2) * t2) / (2.0 * t3);
            let c4 = (-15.0 * dp + (7.0 * df1 + 8.0 * d1) * t + (-df2 + 1.5 * d2) * t2) / t4;
            let c5 = (12.0 * dp - (6.0 * df1 + 6.0 * d1) * t + (df2 - d2) * t2) / (2.0 * t5);
            let got = traj.coefficients.column(dim);
            assert!((got[3] - c3).abs() < 1e-9, "c3[{dim}] {} vs {c3}", got[3]);
            assert!((got[4] - c4).abs() < 1e-9, "c4[{dim}] {} vs {c4}", got[4]);
            assert!((got[5] - c5).abs() < 1e-9, "c5[{dim}] {} vs {c5}", got[5]);
        }
    }
}
