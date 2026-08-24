//! 障碍避碰惩罚(官方 `obstacleGradCostP`)。
//!
//! 平面障碍模型 `(x−s)ᵀv = 0`(`d_o` = `(p−s)ᵀv`),
//! 硬层 `wei_obs·max{(Co−d),0}³` + 软层平滑尾 `wei_obs_soft·r²(√(1+err²/r²)−1)`
//! (r=0.05)。
//!
//! 采样对齐官方 `addPVAGradCost2CT`:每段 K 个约束点 + 尾端点(N·K+1)、
//! 梯形权重 `omg·T/K`、**仅前 2/3 约束点施力**(`two_thirds_id`)。
//! 平面按约束点索引组织(`i_dp = i·K + j`)。

use firefly_map::Plane;
use firefly_trajectory::Trajectory;
use nalgebra::Vector3;

use crate::sampling::{sample_index, trapezoid_weight};
use crate::{Accumulator, Penalty};

/// 障碍距离惩罚(官方 v2 双层 clearance)。
pub struct ObstaclePenalty {
    pub clearance_hard: f64,
    pub clearance_soft: f64,
    pub weight_soft: f64,
    pub samples_per_piece: usize,
    planes_by_point: Vec<Vec<Plane>>,
    /// 前 2/3 截断(`two_thirds_id`);`None` = 不限(独立使用)。
    two_thirds: Option<usize>,
}

impl ObstaclePenalty {
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        clearance_hard: f64,
        clearance_soft: f64,
        weight_soft: f64,
        samples_per_piece: usize,
        planes_by_point: Vec<Vec<Plane>>,
    ) -> Self {
        Self {
            clearance_hard,
            clearance_soft,
            weight_soft,
            samples_per_piece,
            planes_by_point,
            two_thirds: None,
        }
    }

    /// 施加官方前 2/3 截断(规划器在 Rebound 时传入)。
    #[must_use]
    pub fn with_two_thirds(mut self, id: usize) -> Self {
        self.two_thirds = Some(id);
        self
    }
}

impl Penalty for ObstaclePenalty {
    fn evaluate(&self, traj: &Trajectory) -> f64 {
        let k = self.samples_per_piece;
        let mut cost = 0.0;
        for (i, ti) in traj.durations().iter().enumerate() {
            let step = ti / k as f64;
            for j in 0..=k {
                let tau = j as f64 / k as f64;
                let s = traj.eval_piece(i, tau * ti);
                let idx = sample_index(i, j, k);
                if let Some(t) = self.two_thirds
                    && (idx == 0 || idx > t)
                {
                    continue;
                }
                let omg = trapezoid_weight(j, k);
                cost += omg * step * self.point_cost(s.position, &self.planes_by_point[idx]);
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
                let s = traj.eval_piece(i, tau * ti);
                let idx = sample_index(i, j, k);
                if let Some(t) = self.two_thirds
                    && (idx == 0 || idx > t)
                {
                    continue;
                }
                let omg = trapezoid_weight(j, k);
                // 采样权重 (omg·T/K) 对 T 的导数:omg·f/K
                let point_cost = self.point_cost(s.position, &self.planes_by_point[idx]);
                acc.d_f_d_t[i] += weight * omg * point_cost / k as f64;
                let d_p = self.point_gradient(s.position, &self.planes_by_point[idx])
                    * (weight * omg * step);
                acc.add(
                    i,
                    tau,
                    *ti,
                    &s,
                    d_p,
                    Vector3::zeros(),
                    Vector3::zeros(),
                    Vector3::zeros(),
                );
            }
        }
    }
}

impl ObstaclePenalty {
    fn point_cost(&self, p: Vector3<f64>, planes: &[Plane]) -> f64 {
        planes
            .iter()
            .map(|plane| {
                let d = plane.distance(p).value();
                // 硬层:dist < clearance_hard → err³
                let err_hard = self.clearance_hard - d;
                let mut cost = 0.0;
                if err_hard > 0.0 {
                    cost += err_hard * err_hard * err_hard;
                }
                // 软层:dist < clearance_soft → 平滑尾 r²(√(1+err²/r²) − 1)
                let err_soft = self.clearance_soft - d;
                if err_soft > 0.0 {
                    let r = 0.05;
                    let rsqr = r * r;
                    let term = (1.0 + err_soft * err_soft / rsqr).sqrt();
                    cost += self.weight_soft * rsqr * (term - 1.0);
                }
                cost
            })
            .sum()
    }

    fn point_gradient(&self, p: Vector3<f64>, planes: &[Plane]) -> Vector3<f64> {
        planes
            .iter()
            .map(|plane| {
                let d = plane.distance(p).value();
                let normal = plane.normal();
                // 硬层梯度:−3·err²·n
                let err_hard = self.clearance_hard - d;
                let mut grad = Vector3::zeros();
                if err_hard > 0.0 {
                    grad += -3.0 * err_hard * err_hard * normal;
                }
                // 软层梯度:−soft·err/term·n
                let err_soft = self.clearance_soft - d;
                if err_soft > 0.0 {
                    let r = 0.05;
                    let rsqr = r * r;
                    let term = (1.0 + err_soft * err_soft / rsqr).sqrt();
                    grad += -self.weight_soft * err_soft / term * normal;
                }
                grad
            })
            .sum()
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::test_minco;

    fn penalty(planes: Vec<Vec<Plane>>) -> ObstaclePenalty {
        ObstaclePenalty::new(0.1, 0.5, 5000.0, 5, planes)
    }

    #[test]
    fn plane_cost_zero_and_positive() {
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        // 远处平面无代价
        let plane = Plane::new(Vector3::new(-10.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let n_points = traj.pieces() * 5 + 1;
        assert_eq!(penalty(vec![vec![plane]; n_points]).evaluate(&traj), 0.0);
        // 相交平面有正代价
        let plane = Plane::new(Vector3::new(5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let n_points = traj.pieces() * 5 + 1;
        assert!(penalty(vec![vec![plane]; n_points]).evaluate(&traj) > 0.0);
    }

    #[test]
    fn two_thirds_truncation_skips_tail() {
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        let plane = Plane::new(Vector3::new(5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let n_points = traj.pieces() * 5 + 1;
        let full = penalty(vec![vec![plane.clone()]; n_points]).evaluate(&traj);
        // 截断到 2/3 后代价应减小或不变(官方只对前 2/3 施力)
        let truncated =
            penalty(vec![vec![plane]; n_points]).with_two_thirds(n_points - 1 - (n_points - 2) / 3);
        assert!(truncated.evaluate(&traj) <= full + 1e-12);
    }

    #[test]
    fn gradient_matches_numerical() {
        let plane = Plane::new(Vector3::new(0.2, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let p = penalty(vec![vec![plane]]);
        let planes = &p.planes_by_point[0];

        let h = 1e-6;
        let point = Vector3::new(1.5, 0.8, 0.3);
        for dim in 0..3 {
            let f = |x: f64| {
                p.point_cost(
                    Vector3::new(
                        if dim == 0 { x } else { point[0] },
                        if dim == 1 { x } else { point[1] },
                        if dim == 2 { x } else { point[2] },
                    ),
                    planes,
                )
            };
            let numeric = (f(point[dim] + h) - f(point[dim] - h)) / (2.0 * h);
            let analytic = p.point_gradient(point, planes)[dim];
            assert!(
                (numeric - analytic).abs() < 1e-6,
                "dim={dim} numeric={numeric} analytic={analytic}"
            );
        }
    }
}
