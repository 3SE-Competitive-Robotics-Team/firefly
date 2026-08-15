//! 约束点均匀分布惩罚（论文 Eq. S34–S36）。
//!
//! 防止段时长消失（Tᵢ→0 是 MINCO 奇异点）和薄障碍漏检。
//! R 为段内相邻约束点距离平方，Ju = 方差：
//! Ju = (1/Nc)‖R‖²₂ − (1/Nc²)‖R‖²₁
//! 梯度（S36）：∂Ju/∂p̊ᵢ,ⱼ = (4/Nc)[(Rₖ₋₁−meanR)(p̊ⱼ−p̊ⱼ₋₁)
//!                             + (Rₖ−meanR)(p̊ⱼ₊₁−p̊ⱼ)]（端点仅单侧）

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

    /// 采样约束点（段内 κ+1 个，含端点），返回每点位置。
    fn samples(&self, traj: &Trajectory) -> Vec<(usize, f64, Sample)> {
        let mut out = Vec::new();
        for (i, ti) in traj.durations().iter().enumerate() {
            for j in 0..=self.samples_per_piece {
                let tau = j as f64 / self.samples_per_piece as f64;
                let t = segment_time(traj, i, *ti, tau);
                out.push((i, tau, traj.eval(t)));
            }
        }
        out
    }

    /// 相邻对距离平方 R，及每对所属段（段内相邻，不跨段）。
    fn pair_distances(
        &self,
        traj: &Trajectory,
        pts: &[(usize, f64, Sample)],
    ) -> (Vec<f64>, Vec<(usize, usize)>) {
        let mut r = Vec::new();
        let mut pairs = Vec::new();
        let per_piece = self.samples_per_piece + 1;
        for (i, _) in traj.durations().iter().enumerate() {
            for j in 0..self.samples_per_piece {
                let a = i * per_piece + j;
                let b = a + 1;
                r.push((pts[b].2.position - pts[a].2.position).norm_squared());
                pairs.push((a, b));
            }
        }
        (r, pairs)
    }
}

impl Default for UniformPenalty {
    fn default() -> Self {
        Self::new()
    }
}

impl Penalty for UniformPenalty {
    fn evaluate(&self, traj: &Trajectory) -> f64 {
        let pts = self.samples(traj);
        let (r, _) = self.pair_distances(traj, &pts);
        let n = r.len() as f64;
        let mean = r.iter().sum::<f64>() / n;
        let mean_sq = r.iter().map(|v| v * v).sum::<f64>() / n;
        mean_sq - mean * mean
    }

    fn accumulate(&self, traj: &Trajectory, weight: f64, acc: &mut Accumulator) {
        let pts = self.samples(traj);
        let (r, pairs) = self.pair_distances(traj, &pts);
        let n = r.len() as f64;
        let mean = r.iter().sum::<f64>() / n;

        // ∂Ju/∂p̊ = (4/Nc)[(Rₖ₋₁−meanR)·(p̊ⱼ−p̊ⱼ₋₁) + (Rₖ−meanR)·(p̊ⱼ₊₁−p̊ⱼ)]
        let mut d_p_by_point = vec![Vector3::zeros(); pts.len()];
        for (k, (a, b)) in pairs.iter().enumerate() {
            let (pa, pb) = (pts[*a].2.position, pts[*b].2.position);
            let factor = 4.0 / n * (r[k] - mean);
            d_p_by_point[*a] += factor * (pa - pb);
            d_p_by_point[*b] += factor * (pb - pa);
        }

        for (idx, (piece, tau, s)) in pts.iter().enumerate() {
            let ti = traj.durations()[*piece];
            // Ju 是方差（Eq. S34），非积分形式：梯度不乘采样权重
            let d_p = d_p_by_point[idx] * weight;
            acc.add(
                *piece,
                *tau,
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

fn segment_time(traj: &Trajectory, piece: usize, duration: f64, tau: f64) -> f64 {
    let mut t = 0.0;
    for k in 0..piece {
        t += traj.durations()[k];
    }
    t + tau * duration
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_minco;
    use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};

    #[test]
    fn uniform_vs_nonuniform() {
        // 匀速直线：约束点均匀分布 → 零代价
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
        assert!(UniformPenalty::new().evaluate(&traj) < 1e-10);
        // 非均匀时长 → 正代价
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        assert!(UniformPenalty::new().evaluate(&traj) > 0.0);
    }

    #[test]
    fn gradient_matches_numerical_in_parameter_space() {
        use crate::Cost;
        use firefly_trajectory::MincoBuilder;
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
                let mut p = *qi;
                p[dim] += h;
                qp[i] = p;
                p = *qi;
                p[dim] -= h;
                qm[i] = p;
                let numeric = (eval(&qp, &t0) - eval(&qm, &t0)) / (2.0 * h);
                let analytic = dq[(dim, i)];
                assert!(
                    (numeric - analytic).abs() < 1e-5 * (1.0 + analytic.abs()),
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
                (numeric - analytic).abs() < 1e-5 * (1.0 + analytic.abs()),
                "dt[{i}] analytic={analytic} numeric={numeric}"
            );
        }
    }
}
