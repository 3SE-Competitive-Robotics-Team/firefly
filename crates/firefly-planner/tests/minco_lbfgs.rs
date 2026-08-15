//! 端到端集成：MINCO 梯度传播 + L-BFGS 时空联合优化。
//!
//! 目标：J = Js(c,T) + λt·sum(T)，Js = Σ ∫||jerk||²dt（平滑能量，闭式）。
//! 优化变量 x = [q 展平(3(M−1)), T(M)]，验证整条链路收敛。

use firefly_optimize::{Lbfgs, LbfgsConfig, Objective};
use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
use nalgebra::{DMatrix, DVector, Vector3};

struct MincoObjective {
    start: Endpoint,
    end: Endpoint,
    lambda_t: f64,
    pieces: usize,
    dim: usize,
}

impl MincoObjective {
    fn new(start: Endpoint, end: Endpoint, lambda_t: f64, pieces: usize) -> Self {
        Self {
            start,
            end,
            lambda_t,
            pieces,
            dim: 3,
        }
    }

    fn unpack(&self, x: &DVector<f64>) -> (Vec<Vector3<f64>>, Vec<f64>) {
        let n_q = self.dim * (self.pieces - 1);
        let mut q = Vec::with_capacity(self.pieces - 1);
        for i in 0..self.pieces - 1 {
            q.push(Vector3::new(x[i * 3], x[i * 3 + 1], x[i * 3 + 2]));
        }
        // 时间用对数参数化：T = exp(τ)，保证恒正；clamp 防止线搜索越界溢出
        let t = (0..self.pieces)
            .map(|i| x[n_q + i].clamp(-8.0, 8.0).exp())
            .collect();
        (q, t)
    }

    fn pack(&self, dq: &DMatrix<f64>, dt: &DVector<f64>, t: &[f64]) -> DVector<f64> {
        let mut g = DVector::zeros(self.dim * (self.pieces - 1) + self.pieces);
        for i in 0..self.pieces - 1 {
            for d in 0..self.dim {
                g[i * 3 + d] = dq[(d, i)];
            }
        }
        for i in 0..self.pieces {
            // dJ/dτ = dJ/dT · dT/dτ = dJ/dT · T
            g[self.dim * (self.pieces - 1) + i] = dt[i] * t[i];
        }
        g
    }

    fn build(&self, q: &[Vector3<f64>], t: &[f64]) -> firefly_trajectory::Minco {
        let points: Vec<_> = q
            .iter()
            .map(|v| nalgebra::Point3::new(v.x, v.y, v.z))
            .collect();
        MincoBuilder::new(SolverOrder::MinimumJerk, self.start, self.end)
            .build(&points, t)
            .expect("valid minco")
    }

    /// ∫β'''(τ)β'''(τ)ᵀdτ 的 Gram 矩阵（6×6，仅 3..5 阶非零）。
    fn jerk_gram(t: f64) -> DMatrix<f64> {
        let mut g = DMatrix::zeros(6, 6);
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;
        g[(3, 3)] = 36.0 * t;
        g[(3, 4)] = 72.0 * t2;
        g[(4, 3)] = 72.0 * t2;
        g[(3, 5)] = 120.0 * t3;
        g[(5, 3)] = 120.0 * t3;
        g[(4, 4)] = 192.0 * t3;
        g[(4, 5)] = 360.0 * t4;
        g[(5, 4)] = 360.0 * t4;
        g[(5, 5)] = 720.0 * t5;
        g
    }

    fn jerk_gram_derivative(t: f64) -> DMatrix<f64> {
        let mut g = DMatrix::zeros(6, 6);
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        g[(3, 3)] = 36.0;
        g[(3, 4)] = 144.0 * t;
        g[(4, 3)] = 144.0 * t;
        g[(3, 5)] = 360.0 * t2;
        g[(5, 3)] = 360.0 * t2;
        g[(4, 4)] = 576.0 * t2;
        g[(4, 5)] = 1440.0 * t3;
        g[(5, 4)] = 1440.0 * t3;
        g[(5, 5)] = 3600.0 * t4;
        g
    }
}

impl Objective for MincoObjective {
    fn evaluate(&mut self, x: &DVector<f64>) -> f64 {
        let (q, t) = self.unpack(x);
        let minco = self.build(&q, &t);
        let traj = minco.solve().expect("nonsingular");
        let c = traj.coefficients();
        let mut cost = 0.0;
        for (i, ti) in t.iter().enumerate() {
            let gram = Self::jerk_gram(*ti);
            for dim in 0..self.dim {
                let row = i * 6;
                for a in 0..6 {
                    for b in 0..6 {
                        cost += c[(row + a, dim)] * gram[(a, b)] * c[(row + b, dim)];
                    }
                }
            }
        }
        cost + self.lambda_t * t.iter().sum::<f64>()
    }

    fn gradient(&mut self, x: &DVector<f64>) -> DVector<f64> {
        let (q, t) = self.unpack(x);
        let minco = self.build(&q, &t);
        let traj = minco.solve().expect("nonsingular");
        let c = traj.coefficients();

        let mut d_f_d_c = DMatrix::zeros(6 * self.pieces, self.dim);
        let mut d_f_d_t = DVector::zeros(self.pieces);
        for i in 0..self.pieces {
            let gram = Self::jerk_gram(t[i]);
            let gram_dt = Self::jerk_gram_derivative(t[i]);
            for dim in 0..self.dim {
                let row = i * 6;
                for a in 0..6 {
                    for b in 0..6 {
                        d_f_d_c[(row + a, dim)] += 2.0 * gram[(a, b)] * c[(row + b, dim)];
                        d_f_d_t[i] += c[(row + a, dim)] * gram_dt[(a, b)] * c[(row + b, dim)];
                    }
                }
            }
            d_f_d_t[i] += self.lambda_t;
        }

        let (dq, dt) = minco
            .propagate_gradient(&traj, &d_f_d_c, &d_f_d_t)
            .expect("gradient propagation");
        self.pack(&dq, &dt, &t)
    }
}

#[test]
fn spatial_temporal_optimization_converges() {
    let start = Endpoint {
        position: Vector3::new(0.0, 0.0, 0.0),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let end = Endpoint {
        position: Vector3::new(5.0, 2.0, 1.0),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let pieces = 3;
    let mut objective = MincoObjective::new(start, end, 1.0, pieces);

    // 初始：q 在直线上，T 均匀
    let mut x0 = DVector::zeros(3 * (pieces - 1) + pieces);
    for i in 0..pieces - 1 {
        let alpha = (i + 1) as f64 / pieces as f64;
        x0[i * 3] = 5.0 * alpha;
        x0[i * 3 + 1] = 2.0 * alpha;
        x0[i * 3 + 2] = 1.0 * alpha;
    }
    for i in 0..pieces {
        x0[3 * (pieces - 1) + i] = 0.0; // τ = ln(1) = 0
    }

    let j0 = objective.evaluate(&x0);
    let lbfgs = Lbfgs::new(LbfgsConfig::default());
    let report = lbfgs.minimize(&mut objective, x0).expect("converges");
    assert!(report.converged);
    assert!(
        report.final_cost < j0,
        "cost must decrease: {j0} -> {}",
        report.final_cost
    );

    // 最优解验证：
    // 1. 梯度收敛（官方判据：相对无穷范数，停止时梯度残余可接受）
    assert!(
        report.gradient_norm < 1.0,
        "gradient norm {}",
        report.gradient_norm
    );
    // 2. 最终轨迹满足边界条件（静止到静止，轨迹应为 S 形，q 不要求在线）
    let (q, t) = objective.unpack(&report.final_x);
    let points: Vec<_> = q
        .iter()
        .map(|v| nalgebra::Point3::new(v.x, v.y, v.z))
        .collect();
    let minco = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&points, &t)
        .expect("valid");
    let traj = minco.solve().expect("nonsingular");
    let s0 = traj.eval(0.0);
    let sf = traj.eval(traj.duration());
    assert!((s0.position - start.position).norm() < 1e-9);
    assert!((s0.velocity - start.velocity).norm() < 1e-9);
    assert!((sf.position - end.position).norm() < 1e-9);
    assert!((sf.velocity - end.velocity).norm() < 1e-9);
    assert!(t.iter().all(|ti| *ti > 0.0), "durations must stay positive");
}
