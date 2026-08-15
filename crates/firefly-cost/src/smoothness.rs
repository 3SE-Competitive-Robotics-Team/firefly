//! 平滑惩罚：Js = ∫||p‴(t)||²dt（最小加加速度能量，闭式）。

use firefly_trajectory::Trajectory;
use nalgebra::DMatrix;

use crate::{Accumulator, Penalty};

pub struct SmoothnessPenalty;

impl Penalty for SmoothnessPenalty {
    fn evaluate(&self, traj: &Trajectory) -> f64 {
        let c = traj.coefficients();
        let mut cost = 0.0;
        for (i, ti) in traj.durations().iter().enumerate() {
            let gram = jerk_gram(*ti);
            for dim in 0..3 {
                let row = i * 6;
                for a in 0..6 {
                    for b in 0..6 {
                        cost += c[(row + a, dim)] * gram[(a, b)] * c[(row + b, dim)];
                    }
                }
            }
        }
        cost
    }

    fn accumulate(&self, traj: &Trajectory, weight: f64, acc: &mut Accumulator) {
        let c = traj.coefficients();
        for (i, ti) in traj.durations().iter().enumerate() {
            let gram = jerk_gram(*ti);
            let gram_dt = jerk_gram_derivative(*ti);
            for dim in 0..3 {
                let row = i * 6;
                for a in 0..6 {
                    for b in 0..6 {
                        acc.d_f_d_c[(row + a, dim)] +=
                            weight * 2.0 * gram[(a, b)] * c[(row + b, dim)];
                        acc.d_f_d_t[i] +=
                            weight * c[(row + a, dim)] * gram_dt[(a, b)] * c[(row + b, dim)];
                    }
                }
            }
        }
    }
}

/// ∫β‴(τ)β‴(τ)ᵀdτ 的 Gram 矩阵（6×6，仅 3..5 阶非零）。
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_minco;
    use nalgebra::Vector3;

    #[test]
    fn smoothness_energy() {
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        // 非负且有限
        let j = SmoothnessPenalty.evaluate(&traj);
        assert!(j.is_finite() && j >= 0.0);
        // 静止到静止单段最小 jerk：jerk = 60t−30 → ∫jerk² = 720
        let minco2 = firefly_trajectory::MincoBuilder::new(
            firefly_trajectory::SolverOrder::MinimumJerk,
            firefly_trajectory::Endpoint {
                position: Vector3::zeros(),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
            firefly_trajectory::Endpoint {
                position: Vector3::new(1.0, 0.0, 0.0),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
        )
        .build(&[], &[1.0])
        .unwrap()
        .solve()
        .unwrap();
        let cost = SmoothnessPenalty.evaluate(&minco2);
        assert!((cost - 720.0).abs() < 1e-6, "cost={cost}");
    }
    #[test]
    fn zero_velocity_trajectory_has_positive_energy() {
        // 静止到静止的单段轨迹，jerk 能量应为正
        use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
        use nalgebra::Vector3;
        let start = Endpoint {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(1.0, 0.0, 0.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let minco = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[], &[1.0])
            .unwrap();
        let traj = minco.solve().unwrap();
        // 单段静止到静止最小 jerk：jerk = 60t−30 → ∫jerk² = 720
        let cost = SmoothnessPenalty.evaluate(&traj);
        assert!((cost - 720.0).abs() < 1e-6, "cost={cost}");
    }
}
