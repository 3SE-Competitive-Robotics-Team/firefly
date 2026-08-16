//! 障碍避碰惩罚。
//!
//! 平面障碍模型 (x−s)ᵀv = 0（论文 From single to swarm 节）：
//! Jo = Σ max{(Co−do),0}³，do = (p−s)ᵀv。
//!
//! 补充材料 S6：每个约束点 pkey 私有拥有自己的 {s,v} 对，
//! 不计算与其他 pkey 的 {s,v} 的距离。因此平面按采样点索引组织。
//! 采样点索引：idx = piece·(κ+1) + j（piece 段内，j = 0..=κ）。

use firefly_map::Plane;
use firefly_trajectory::Trajectory;
use nalgebra::Vector3;

use crate::{Accumulator, Penalty};

/// 障碍距离惩罚（官方 v2 双层 clearance）：
/// - 硬层 `clearance_hard`：三次方惩罚（`weight_hard × err³`）；
/// - 软层 `clearance_soft`：平滑尾 `r²(√(1+err²/r²) − 1)`（`r = 0.05`）。
pub struct ObstaclePenalty {
    pub clearance_hard: f64,
    pub clearance_soft: f64,
    pub weight_soft: f64,
    pub samples_per_piece: usize,
    planes_by_point: Vec<Vec<Plane>>,
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
        }
    }
}

impl Penalty for ObstaclePenalty {
    fn evaluate(&self, traj: &Trajectory) -> f64 {
        let mut cost = 0.0;
        for (i, ti) in traj.durations().iter().enumerate() {
            let weight = ti / self.samples_per_piece as f64;
            for j in 0..=self.samples_per_piece {
                let tau = j as f64 / self.samples_per_piece as f64;
                let s = traj.eval(segment_time(traj, i, *ti, tau));
                let planes = &self.planes_by_point[self.point_index(i, j)];
                cost += weight * self.point_cost(s.position, planes);
            }
        }
        cost
    }

    fn accumulate(&self, traj: &Trajectory, weight: f64, acc: &mut Accumulator) {
        for (i, ti) in traj.durations().iter().enumerate() {
            let sample_weight = weight * ti / self.samples_per_piece as f64;
            for j in 0..=self.samples_per_piece {
                let tau = j as f64 / self.samples_per_piece as f64;
                let s = traj.eval(segment_time(traj, i, *ti, tau));
                let planes = &self.planes_by_point[self.point_index(i, j)];
                let d_p = self.point_gradient(s.position, planes) * sample_weight;
                // 采样权重 (T/κ) 对 T 的导数：f/κ
                let point_cost = self.point_cost(s.position, planes);
                acc.d_f_d_t[i] += weight * point_cost / self.samples_per_piece as f64;
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
    fn point_index(&self, piece: usize, j: usize) -> usize {
        piece * (self.samples_per_piece + 1) + j
    }

    fn point_cost(&self, p: Vector3<f64>, planes: &[Plane]) -> f64 {
        planes
            .iter()
            .map(|plane| {
                let d = plane.distance(p).value();
                // 硬层：dist < clearance_hard → err³
                let err_hard = self.clearance_hard - d;
                let mut cost = 0.0;
                if err_hard > 0.0 {
                    cost += err_hard * err_hard * err_hard;
                }
                // 软层：dist < clearance_soft → 平滑尾 r²(√(1+err²/r²) − 1)
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
                // 硬层梯度：−3·err²·n
                let err_hard = self.clearance_hard - d;
                let mut grad = Vector3::zeros();
                if err_hard > 0.0 {
                    grad += -3.0 * err_hard * err_hard * normal;
                }
                // 软层梯度：−soft·err/term·n
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

    fn penalty(planes: Vec<Vec<Plane>>) -> ObstaclePenalty {
        ObstaclePenalty::new(0.1, 0.5, 5000.0, 5, planes)
    }

    #[test]
    fn plane_cost_zero_and_positive() {
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        // 远处平面无代价
        let plane = Plane::new(Vector3::new(-10.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let n_points = traj.pieces() * 6;
        assert_eq!(penalty(vec![vec![plane]; n_points]).evaluate(&traj), 0.0);
        // 相交平面有正代价
        let plane = Plane::new(Vector3::new(5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let n_points = traj.pieces() * 6;
        assert!(penalty(vec![vec![plane]; n_points]).evaluate(&traj) > 0.0);
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
