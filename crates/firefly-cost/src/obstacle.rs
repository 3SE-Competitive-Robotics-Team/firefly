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

pub struct ObstaclePenalty {
    pub clearance: f64,
    pub samples_per_piece: usize,
    planes_by_point: Vec<Vec<Plane>>,
}

impl ObstaclePenalty {
    #[must_use]
    pub fn new(clearance: f64, samples_per_piece: usize, planes_by_point: Vec<Vec<Plane>>) -> Self {
        Self {
            clearance,
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
                let excess = self.clearance - d;
                // v1 论文 Eq.5：深处用二次尾部，避免梯度爆炸式过推
                if excess <= 0.0 {
                    0.0
                } else if excess <= self.clearance {
                    excess * excess * excess
                } else {
                    3.0 * self.clearance * excess * excess
                        - 3.0 * self.clearance * self.clearance * excess
                        + self.clearance * self.clearance * self.clearance
                }
            })
            .sum()
    }

    fn point_gradient(&self, p: Vector3<f64>, planes: &[Plane]) -> Vector3<f64> {
        planes
            .iter()
            .map(|plane| {
                let d = plane.distance(p).value();
                let excess = self.clearance - d;
                let slope = if excess <= 0.0 {
                    0.0
                } else if excess <= self.clearance {
                    3.0 * excess * excess
                } else {
                    6.0 * self.clearance * excess - 3.0 * self.clearance * self.clearance
                };
                -slope * plane.normal()
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
        ObstaclePenalty::new(0.3, 5, planes)
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
