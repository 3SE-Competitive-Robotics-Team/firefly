//! 动力学可行性惩罚(官方 `feasibilityGradCostV/A`)。
//!
//! 官方 v2 目标中**只有速度与加速度**惩罚:
//! `wei_feas·max{(v²−vm²),0}³ + wei_feas·max{(a²−am²),0}³`
//! (无 jerk 项;`max_jer` 参数官方只读取不使用)。
//! 采样对齐 `addPVAGradCost2CT`:梯形权重 `omg·T/K`,全段施力(不截断)。

use firefly_trajectory::Trajectory;
use nalgebra::Vector3;

use crate::sampling::trapezoid_weight;
use crate::{Accumulator, Penalty};

pub struct FeasibilityPenalty {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub samples_per_piece: usize,
}

impl FeasibilityPenalty {
    #[must_use]
    pub fn new(max_velocity: f64, max_acceleration: f64) -> Self {
        Self {
            max_velocity,
            max_acceleration,
            samples_per_piece: 5,
        }
    }

    #[must_use]
    pub fn with_samples(mut self, samples: usize) -> Self {
        self.samples_per_piece = samples;
        self
    }
}

impl Penalty for FeasibilityPenalty {
    fn evaluate(&self, traj: &Trajectory) -> f64 {
        let k = self.samples_per_piece;
        let mut cost = 0.0;
        for (i, ti) in traj.durations().iter().enumerate() {
            let step = ti / k as f64;
            for j in 0..=k {
                let tau = j as f64 / k as f64;
                let s = traj.eval(segment_time(traj, i, *ti, tau));
                let omg = trapezoid_weight(j, k);
                cost += omg
                    * step
                    * (penalty(s.velocity.norm_squared(), self.max_velocity)
                        + penalty(s.acceleration.norm_squared(), self.max_acceleration));
            }
        }
        cost
    }

    fn accumulate(&self, traj: &Trajectory, weight: f64, acc: &mut Accumulator) {
        let k = self.samples_per_piece;
        for (i, ti) in traj.durations().iter().enumerate() {
            let step = ti / k as f64;
            for j in 0..=k {
                let tau = j as f64 / k as f64;
                let s = traj.eval(segment_time(traj, i, *ti, tau));
                let omg = trapezoid_weight(j, k);
                let d_v = derivative(s.velocity, self.max_velocity) * (weight * omg * step);
                let d_a = derivative(s.acceleration, self.max_acceleration) * (weight * omg * step);
                // 采样权重 (omg·T/K) 对 T 的导数:omg·f/K
                let point_cost = penalty(s.velocity.norm_squared(), self.max_velocity)
                    + penalty(s.acceleration.norm_squared(), self.max_acceleration);
                acc.d_f_d_t[i] += weight * omg * point_cost / k as f64;
                acc.add(
                    i,
                    tau,
                    *ti,
                    &s,
                    Vector3::zeros(),
                    d_v,
                    d_a,
                    Vector3::zeros(),
                );
            }
        }
    }
}

/// f(c²) = max{(c²−cm²),0}³ 对 c 的梯度:6·max{(c²−cm²),0}²·c
fn derivative(c: Vector3<f64>, limit: f64) -> Vector3<f64> {
    let excess = c.norm_squared() - limit * limit;
    if excess <= 0.0 {
        Vector3::zeros()
    } else {
        6.0 * excess * excess * c
    }
}

fn penalty(squared_norm: f64, limit: f64) -> f64 {
    let excess = squared_norm - limit * limit;
    if excess <= 0.0 {
        0.0
    } else {
        excess * excess * excess
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
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::test_minco;

    #[test]
    fn feasibility_cost_zero_and_positive() {
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        // 无限限制 → 零代价
        assert_eq!(FeasibilityPenalty::new(1e9, 1e9).evaluate(&traj), 0.0);
        // 紧限制 → 正代价
        assert!(FeasibilityPenalty::new(0.1, 0.1).evaluate(&traj) > 0.0);
    }
    #[test]
    fn tight_limits_give_positive_cost() {
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        let p = FeasibilityPenalty::new(0.1, 0.1);
        assert!(p.evaluate(&traj) > 0.0);
    }
}
