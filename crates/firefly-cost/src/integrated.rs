//! 对照 EGO-Planner-v2 `9d85475e` 的 `main_ws/poly_traj_optimizer.cpp`。
//! 主积分循环现场求 P/V/A/J/snap；Uniform 在其后单独计算并映射。

use firefly_trajectory::{Sample, Trajectory};
use nalgebra::{SVector, Vector3};

use crate::{
    Accumulator, FeasibilityPenalty, FormationPenalty, ObstaclePenalty, Penalty, SmoothnessPenalty,
    SwarmPenalty,
};

/// MINCO 联合目标的具体参数；所有采样项使用同一个 K。
///
/// `formation` 是 formation 工作区的可选项，默认关闭；其余执行顺序对应 `main_ws`。
/// 时间项在伴随传播完成后由调用方按 `VirtualTGradCost` 加入。
pub struct MincoCost {
    pub samples_per_piece: usize,
    pub weight_smoothness: f64,
    pub weight_time: f64,
    pub obstacle: Option<(f64, ObstaclePenalty)>,
    pub swarm: Option<(f64, SwarmPenalty)>,
    pub feasibility: Option<(f64, FeasibilityPenalty)>,
    pub weight_sqrvariance: f64,
    pub formation: Option<(f64, FormationPenalty)>,
}

impl Default for MincoCost {
    fn default() -> Self {
        Self {
            samples_per_piece: 5,
            weight_smoothness: 0.0,
            weight_time: 0.0,
            obstacle: None,
            swarm: None,
            feasibility: None,
            weight_sqrvariance: 0.0,
            formation: None,
        }
    }
}

impl MincoCost {
    /// 对照 `initAndGetSmoothnessGradCost2PT` / `MinJerkOpt::initGradCost`。
    /// 重置偏导，依次计算 jerk 能量、显式时间梯度和系数梯度。
    #[allow(clippy::many_single_char_names)]
    pub fn init_and_get_smoothness_grad_cost(
        &self,
        traj: &Trajectory,
        acc: &mut Accumulator,
    ) -> f64 {
        acc.reset(traj.pieces());
        let b = traj.coefficients();
        let mut cost = 0.0;
        for (i, t) in traj.durations().iter().copied().enumerate() {
            let c3 = b.row(6 * i + 3);
            let c4 = b.row(6 * i + 4);
            let c5 = b.row(6 * i + 5);
            let t2 = t * t;
            let t3 = t2 * t;
            let t4 = t3 * t;
            let t5 = t4 * t;
            cost += 36.0 * c3.norm_squared() * t
                + 144.0 * c4.dot(&c3) * t2
                + 192.0 * c4.norm_squared() * t3
                + 240.0 * c5.dot(&c3) * t3
                + 720.0 * c5.dot(&c4) * t4
                + 720.0 * c5.norm_squared() * t5;
        }
        for (i, t) in traj.durations().iter().copied().enumerate() {
            let c3 = b.row(6 * i + 3);
            let c4 = b.row(6 * i + 4);
            let c5 = b.row(6 * i + 5);
            let t2 = t * t;
            let t3 = t2 * t;
            let t4 = t3 * t;
            acc.d_f_d_t[i] = self.weight_smoothness
                * (36.0 * c3.norm_squared()
                    + 288.0 * c4.dot(&c3) * t
                    + 576.0 * c4.norm_squared() * t2
                    + 720.0 * c5.dot(&c3) * t2
                    + 2880.0 * c5.dot(&c4) * t3
                    + 3600.0 * c5.norm_squared() * t4);
        }
        for (i, t) in traj.durations().iter().copied().enumerate() {
            let c3 = b.row(6 * i + 3);
            let c4 = b.row(6 * i + 4);
            let c5 = b.row(6 * i + 5);
            let t2 = t * t;
            let t3 = t2 * t;
            let t4 = t3 * t;
            let t5 = t4 * t;
            for dim in 0..3 {
                acc.d_f_d_c[(6 * i + 5, dim)] = self.weight_smoothness
                    * (240.0 * c3[dim] * t3 + 720.0 * c4[dim] * t4 + 1440.0 * c5[dim] * t5);
                acc.d_f_d_c[(6 * i + 4, dim)] = self.weight_smoothness
                    * (144.0 * c3[dim] * t2 + 384.0 * c4[dim] * t3 + 720.0 * c5[dim] * t4);
                acc.d_f_d_c[(6 * i + 3, dim)] = self.weight_smoothness
                    * (72.0 * c3[dim] * t + 144.0 * c4[dim] * t2 + 240.0 * c5[dim] * t3);
            }
        }
        self.weight_smoothness * cost
    }

    /// 对照 `addPVAJGradCost2CT`，输出障碍、集群、可行性、Uniform、可选队形代价。
    /// `points` 是 N·K+1 唯一约束点，段边界由后一段 j=0 覆盖。
    #[fastrace::trace]
    #[allow(
        clippy::many_single_char_names,
        clippy::similar_names,
        clippy::too_many_lines
    )]
    pub fn add_pvaj_grad_cost(
        &self,
        traj: &Trajectory,
        acc: &mut Accumulator,
        points: &mut Vec<Vector3<f64>>,
    ) -> [f64; 5] {
        let n = traj.pieces();
        let k = self.samples_per_piece;
        points.resize(n * k + 1, Vector3::zeros());
        let mut costs = [0.0; 5];
        let mut i_dp = 0;
        let mut time = 0.0;
        let formation_frame = self.formation.as_ref().map(|(_, penalty)| penalty.frame());
        for i in 0..n {
            let c = traj.coefficients().fixed_view::<6, 3>(i * 6, 0);
            let step = traj.durations()[i] / k as f64;
            let mut s1 = 0.0;
            for j in 0..=k {
                let s2 = s1 * s1;
                let s3 = s2 * s1;
                let s4 = s2 * s2;
                let s5 = s4 * s1;
                let beta0 = SVector::<f64, 6>::new(1.0, s1, s2, s3, s4, s5);
                let beta1 =
                    SVector::<f64, 6>::new(0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4);
                let beta2 = SVector::<f64, 6>::new(0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3);
                let beta3 = SVector::<f64, 6>::new(0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2);
                let beta4 = SVector::<f64, 6>::new(0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1);
                let alpha = 1.0 / k as f64 * j as f64;
                let s = Sample {
                    position: c.transpose() * beta0,
                    velocity: c.transpose() * beta1,
                    acceleration: c.transpose() * beta2,
                    jerk: c.transpose() * beta3,
                    snap: c.transpose() * beta4,
                };
                let omg = if j == 0 || j == k { 0.5 } else { 1.0 };
                points[i_dp] = s.position;

                if let Some((weight, penalty)) = &self.obstacle
                    && let Some((costp, gradp)) = penalty.grad_cost(i_dp, s.position)
                {
                    let costp = *weight * costp;
                    let gradp = *weight * gradp;
                    acc.add_coefficient_gradient(i, beta0.as_slice(), omg * step * gradp);
                    let grad_viola_pt = alpha * gradp.dot(&s.velocity);
                    acc.d_f_d_t[i] += omg * (costp / k as f64 + step * grad_viola_pt);
                    costs[0] += omg * step * costp;
                }
                if let Some((weight, penalty)) = &self.swarm
                    && let Some((costp, gradp, gradt, grad_prev_t)) =
                        penalty.grad_cost(i_dp, time + step * j as f64, &s, *weight)
                {
                    acc.add_coefficient_gradient(i, beta0.as_slice(), omg * step * gradp);
                    let grad_viola_pt = alpha * gradt;
                    acc.d_f_d_t[i] += omg * (costp / k as f64 + step * grad_viola_pt);
                    for previous in 0..i {
                        acc.d_f_d_t[previous] += omg * step * grad_prev_t;
                    }
                    costs[1] += omg * step * costp;
                }
                if let Some((weight, penalty)) = &self.feasibility {
                    if let Some((costv, gradv)) =
                        crate::feasibility::grad_cost(s.velocity, penalty.max_velocity, *weight)
                    {
                        acc.add_coefficient_gradient(i, beta1.as_slice(), omg * step * gradv);
                        let grad_viola_vt = alpha * gradv.dot(&s.acceleration);
                        acc.d_f_d_t[i] += omg * (costv / k as f64 + step * grad_viola_vt);
                        costs[2] += omg * step * costv;
                    }
                    if let Some((costa, grada)) = crate::feasibility::grad_cost(
                        s.acceleration,
                        penalty.max_acceleration,
                        *weight,
                    ) {
                        acc.add_coefficient_gradient(i, beta2.as_slice(), omg * step * grada);
                        let grad_viola_at = alpha * grada.dot(&s.jerk);
                        acc.d_f_d_t[i] += omg * (costa / k as f64 + step * grad_viola_at);
                        costs[2] += omg * step * costa;
                    }
                    if let Some((costj, gradj)) =
                        crate::feasibility::grad_cost(s.jerk, penalty.max_jerk, *weight)
                    {
                        acc.add_coefficient_gradient(i, beta3.as_slice(), omg * step * gradj);
                        let grad_viola_jt = alpha * gradj.dot(&s.snap);
                        acc.d_f_d_t[i] += omg * (costj / k as f64 + step * grad_viola_jt);
                        costs[2] += omg * step * costj;
                    }
                }
                // formation 工作区的额外约束，不改变 main_ws 各项的相对顺序。
                if let Some((weight, penalty)) = &self.formation
                    && let Some(frame) = &formation_frame
                    && let Some((costp, gradp, gradt, grad_prev_t)) =
                        penalty.grad_cost(i_dp, time + step * j as f64, &s, frame)
                {
                    acc.add_coefficient_gradient(i, beta0.as_slice(), omg * step * *weight * gradp);
                    acc.d_f_d_t[i] += omg * *weight * (costp / k as f64 + step * alpha * gradt);
                    for previous in 0..i {
                        acc.d_f_d_t[previous] += omg * step * *weight * grad_prev_t;
                    }
                    costs[4] += omg * step * *weight * costp;
                }
                s1 += step;
                if j != k || i == n - 1 {
                    i_dp += 1;
                }
            }
            time += traj.durations()[i];
        }

        let (gdp, variance) = self.distance_sqr_variance_with_grad_cost(points);
        i_dp = 0;
        for i in 0..n {
            let step = traj.durations()[i] / k as f64;
            let mut s1 = 0.0;
            let c = traj.coefficients().fixed_view::<6, 3>(i * 6, 0);
            for j in 0..=k {
                let s2 = s1 * s1;
                let s3 = s2 * s1;
                let s4 = s2 * s2;
                let s5 = s4 * s1;
                let beta0 = SVector::<f64, 6>::new(1.0, s1, s2, s3, s4, s5);
                let beta1 =
                    SVector::<f64, 6>::new(0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4);
                let alpha = 1.0 / k as f64 * j as f64;
                let vel = c.transpose() * beta1;
                let omg = if j == 0 || j == k { 0.5 } else { 1.0 };
                let grad_viola_pt = alpha * gdp[i_dp].dot(&vel);
                acc.add_coefficient_gradient(i, beta0.as_slice(), omg * gdp[i_dp]);
                acc.d_f_d_t[i] += omg * grad_viola_pt;
                s1 += step;
                if j != k || i == n - 1 {
                    i_dp += 1;
                }
            }
        }
        costs[3] += variance;
        costs
    }

    /// 对照 `distanceSqrVarianceWithGradCost2p` 的 dps / dsqrs / gdp 数组。
    #[allow(clippy::many_single_char_names)]
    fn distance_sqr_variance_with_grad_cost(
        &self,
        points: &[Vector3<f64>],
    ) -> (Vec<Vector3<f64>>, f64) {
        let n = points.len() - 1;
        let dps: Vec<_> = points.windows(2).map(|pair| pair[1] - pair[0]).collect();
        let dsqrs: Vec<_> = dps.iter().map(Vector3::norm_squared).collect();
        let dquarsum: f64 = dsqrs.iter().map(|value| value * value).sum();
        let variance = self.weight_sqrvariance * (dquarsum / n as f64);
        let mut gdp = vec![Vector3::zeros(); n + 1];
        for i in 0..=n {
            if i != 0 {
                gdp[i] += self.weight_sqrvariance * (4.0 * dsqrs[i - 1] / n as f64 * dps[i - 1]);
            }
            if i != n {
                gdp[i] += self.weight_sqrvariance * (-4.0 * dsqrs[i] / n as f64 * dps[i]);
            }
        }
        (gdp, variance)
    }

    /// 独立代价实现，用于与联合回调比较。
    #[must_use]
    pub fn evaluate(&self, traj: &Trajectory) -> f64 {
        let mut cost = self.weight_smoothness * SmoothnessPenalty.evaluate(traj)
            + self.weight_time * traj.duration();
        if let Some((weight, penalty)) = &self.obstacle {
            cost += weight * penalty.evaluate(traj);
        }
        if let Some((weight, penalty)) = &self.swarm {
            cost += weight * penalty.evaluate(traj);
        }
        if let Some((weight, penalty)) = &self.feasibility {
            cost += weight * penalty.evaluate(traj);
        }
        cost += self.weight_sqrvariance
            * crate::UniformPenalty::new()
                .with_samples(self.samples_per_piece)
                .evaluate(traj);
        if let Some((weight, penalty)) = &self.formation {
            cost += weight * penalty.evaluate(traj);
        }
        cost
    }

    /// 独立偏导实现，时间项在此按显式时间偏导加入。
    pub fn gradient_into(&self, traj: &Trajectory, acc: &mut Accumulator) {
        acc.reset(traj.pieces());
        SmoothnessPenalty.accumulate(traj, self.weight_smoothness, acc);
        crate::TimePenalty.accumulate(traj, self.weight_time, acc);
        if let Some((weight, penalty)) = &self.obstacle {
            penalty.accumulate(traj, *weight, acc);
        }
        if let Some((weight, penalty)) = &self.swarm {
            penalty.accumulate(traj, *weight, acc);
        }
        if let Some((weight, penalty)) = &self.feasibility {
            penalty.accumulate(traj, *weight, acc);
        }
        crate::UniformPenalty::new()
            .with_samples(self.samples_per_piece)
            .accumulate(traj, self.weight_sqrvariance, acc);
        if let Some((weight, penalty)) = &self.formation {
            penalty.accumulate(traj, *weight, acc);
        }
    }
}
