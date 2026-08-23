//! 约束点均匀分布惩罚(官方 `distanceSqrVarianceWithGradCost2p`)。
//!
//! 官方公式(注意**非中心化**,且 R 是相邻距离的**平方**、方差取其平方均值):
//! - `dps[i] = p[i+1] − p[i]`,`R[i] = |dps[i]|²`
//! - `Ju = wei_sqrvar·ΣR[i]²/N`(N = 约束点数 − 1)
//! - `∂Ju/∂p[i] = wei_sqrvar·4/N·(R[i−1]·dps[i−1] − R[i]·dps[i])`
//!
//! 约束点数组 = 轨迹采样点(N·K+1,段边界不重复);采样点索引 `i_dp = i·K+j`。
//! 防止段时长消失(Tᵢ→0 是 MINCO 奇异点)与薄障碍漏检。
//! 采样不乘积分权重,但端点折半(`omg`,官方 `addPVAGradCost2CT` 中 uniform 段)。

use firefly_trajectory::{Sample, Trajectory};
use nalgebra::Vector3;

use crate::{Accumulator, Penalty};

pub struct UniformPenalty {
    pub samples_per_piece: usize,
}

impl UniformPenalty {
    #[must_use]
    pub fn new() -> Self {
        Self {
            samples_per_piece: 5,
        }
    }

    #[must_use]
    pub fn with_samples(mut self, samples: usize) -> Self {
        self.samples_per_piece = samples;
        self
    }

    /// 全部采样点(N·(K+1) 个,段边界**重复**:官方 `i_dp` 在边界两侧
    /// 各累加一次,`omg=0.5`,梯度按 `i_dp = i·K+j` 对齐)。
    fn samples(&self, traj: &Trajectory) -> Vec<(usize, usize, f64, Sample)> {
        let k = self.samples_per_piece;
        let mut out = Vec::with_capacity(traj.pieces() * (k + 1));
        let mut prefix = 0.0;
        for (i, ti) in traj.durations().iter().enumerate() {
            for j in 0..=k {
                let tau = j as f64 / k as f64;
                out.push((i * k + j, i, tau, traj.eval(prefix + tau * ti)));
            }
            prefix += ti;
        }
        out
    }

    /// 唯一约束点(N·K+1:段边界只算一次)数量。
    fn n_points(&self, traj: &Trajectory) -> usize {
        traj.pieces() * self.samples_per_piece + 1
    }
}

impl Default for UniformPenalty {
    fn default() -> Self {
        Self::new()
    }
}

impl Penalty for UniformPenalty {
    fn evaluate(&self, traj: &Trajectory) -> f64 {
        // 唯一约束点序列(N·K+1)求相邻距离(官方 ps 列)。
        let k = self.samples_per_piece;
        let n_points = self.n_points(traj);
        let n = n_points - 1;
        if n == 0 {
            return 0.0;
        }
        let mut pos: Vec<Vector3<f64>> = Vec::with_capacity(n_points);
        let mut prefix = 0.0;
        for (i, ti) in traj.durations().iter().enumerate() {
            let j_max = if i + 1 == traj.pieces() { k } else { k - 1 };
            for j in 0..=j_max {
                let tau = j as f64 / k as f64;
                pos.push(traj.eval(prefix + tau * ti).position);
            }
            prefix += ti;
        }
        let mut dquar_sum = 0.0;
        for i in 0..n {
            let r = (pos[i + 1] - pos[i]).norm_squared();
            dquar_sum += r * r;
        }
        dquar_sum / n as f64
    }

    // 与官方公式逐行对应(k/n/i/j/tau/dps/r),单字符命名保可读性
    #[allow(clippy::many_single_char_names)]
    fn accumulate(&self, traj: &Trajectory, weight: f64, acc: &mut Accumulator) {
        let k = self.samples_per_piece;
        let pts = self.samples(traj);
        let n_points = self.n_points(traj);
        let n = n_points - 1;
        if n == 0 {
            return;
        }
        // 唯一约束点序列的相邻差与平方距离(官方 dps / dsqrs)
        let mut dps: Vec<Vector3<f64>> = Vec::with_capacity(n);
        let mut r: Vec<f64> = Vec::with_capacity(n);
        let mut prefix = 0.0;
        let mut last = traj.eval(0.0).position;
        for (i, ti) in traj.durations().iter().enumerate() {
            let j_max = if i + 1 == traj.pieces() { k } else { k - 1 };
            for j in 0..=j_max {
                let s = traj.eval(prefix + j as f64 / k as f64 * ti);
                if i > 0 || j > 0 {
                    let d = s.position - last;
                    dps.push(d);
                    r.push(d.norm_squared());
                }
                last = s.position;
            }
            prefix += ti;
        }
        // 官方梯度(不含 wei,wei 由 Cost 权重提供):4/N·(R[i−1]·dps[i−1] − R[i]·dps[i])
        let mut gdp = vec![Vector3::zeros(); n_points];
        for i in 0..=n {
            if i != 0 {
                gdp[i] += 4.0 / n as f64 * r[i - 1] * dps[i - 1];
            }
            if i != n {
                gdp[i] += -4.0 / n as f64 * r[i] * dps[i];
            }
        }
        // 全采样(含段边界重复):边界点两侧各累加一次,omg=0.5(官方一致)
        for &(i_dp, piece, tau, ref s) in &pts {
            let ti = traj.durations()[piece];
            // 官方 omg = (j==0 || j==K) ? 0.5 : 1.0,即 i_dp % K == 0
            let omg = if i_dp % k == 0 { 0.5 } else { 1.0 };
            let d_p = gdp[i_dp] * (weight * omg);
            acc.add(
                piece,
                tau,
                ti,
                s,
                d_p,
                Vector3::zeros(),
                Vector3::zeros(),
                Vector3::zeros(),
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_minco;
    use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};

    #[test]
    fn uniform_vs_nonuniform() {
        // 匀速直线:约束点均匀分布 → 代价接近零(端点折半权重下严格为 0 的
        // 情形是全部相邻距离相等;此处 4 段等长直线)
        let start = Endpoint {
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(1.0, 0.0, 0.0),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(4.0, 0.0, 0.0),
            velocity: Vector3::new(1.0, 0.0, 0.0),
            acceleration: Vector3::zeros(),
        };
        let minco = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(
                &[
                    nalgebra::Point3::new(1.0, 0.0, 0.0),
                    nalgebra::Point3::new(2.0, 0.0, 0.0),
                    nalgebra::Point3::new(3.0, 0.0, 0.0),
                ],
                &[1.0, 1.0, 1.0, 1.0],
            )
            .unwrap();
        let traj = minco.solve().unwrap();
        // 官方 distanceSqrVarianceWithGradCost2p 非中心化:等距直线 →
        // 相邻距离平方 R = 0.2² 全同,Ju = mean(R²) = (0.2²)² = 1.6e-3。
        // 端点梯度非 0(移动端点改变首/末 R),梯度正确性由
        // gradient_matches_numerical_in_parameter_space 数值验证。
        let actual = UniformPenalty::new().evaluate(&traj);
        assert!(
            (actual - 1.6e-3).abs() < 1e-9,
            "等距直线 Ju 应为 1.6e-3,实际 {actual}"
        );
        // 非均匀时长 → 代价不同(非 1.6e-3)
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        let nonuniform = UniformPenalty::new().evaluate(&traj);
        assert!((nonuniform - 1.6e-3).abs() > 1e-9, "非均匀轨迹代价应不同");
    }

    #[test]
    fn gradient_matches_numerical_in_parameter_space() {
        use crate::Cost;
        use nalgebra::Point3;

        let start = Endpoint {
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.5, 0.0, 0.0),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(4.0, 2.0, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let minco = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(
                &[Point3::new(1.5, 0.8, 0.3), Point3::new(2.8, 1.5, 0.6)],
                &[2.0, 2.0, 2.0],
            )
            .unwrap();
        let traj = minco.solve().unwrap();
        let p = UniformPenalty::new();
        let cost = Cost::new().add(1.0, p);

        let (d_f_d_c, d_f_d_t) = cost.gradient(&traj);
        let (dq, dt) = minco.propagate_gradient(&traj, &d_f_d_c, &d_f_d_t).unwrap();

        let q0: Vec<_> = minco.waypoints().collect();
        let t0: Vec<f64> = (0..minco.pieces())
            .map(|i| minco.piece_duration(i))
            .collect();
        let h = 1e-6;
        let eval = |q: &[Point3<f64>], t: &[f64]| {
            let m = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
                .build(q, t)
                .unwrap();
            Cost::new()
                .add(1.0, UniformPenalty::new())
                .evaluate(&m.solve().unwrap())
        };
        for (i, qi) in q0.iter().enumerate() {
            for dim in 0..3 {
                let mut qp = q0.clone();
                let mut qm = q0.clone();
                let mut pp = *qi;
                pp[dim] += h;
                qp[i] = pp;
                let mut pm = *qi;
                pm[dim] -= h;
                qm[i] = pm;
                let numeric = (eval(&qp, &t0) - eval(&qm, &t0)) / (2.0 * h);
                let analytic = dq[(dim, i)];
                assert!(
                    (numeric - analytic).abs() < 1e-4 * (1.0 + analytic.abs()),
                    "dq[{i}][{dim}] analytic={analytic} numeric={numeric}"
                );
            }
        }
        for (i, ti) in t0.iter().enumerate() {
            let mut tp = t0.clone();
            let mut tm = t0.clone();
            tp[i] = ti + h;
            tm[i] = ti - h;
            let numeric = (eval(&q0, &tp) - eval(&q0, &tm)) / (2.0 * h);
            let analytic = dt[i];
            assert!(
                (numeric - analytic).abs() < 1e-4 * (1.0 + analytic.abs()),
                "dt[{i}] analytic={analytic} numeric={numeric}"
            );
        }
    }
}
