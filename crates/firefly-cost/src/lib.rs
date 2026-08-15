//! 轨迹代价函数。
//!
//! 论文（Sci. Robot. 2022）约束转惩罚 + 均匀采样离散化：
//! J = Σ λx·Jx，x ∈ {s 平滑, t 总时间, d 可行性, o 障碍}。
//! 各惩罚项输出 (J, ∂F/∂c, ∂F/∂T 显式部分)，经 `Minco::propagate_gradient`
//! 传播到 {q, T} 空间。

mod accumulator;
mod feasibility;
mod formation;
mod obstacle;
mod peer;
mod smoothness;
mod swarm;
mod time;
mod uniform;

pub use accumulator::Accumulator;
pub use feasibility::FeasibilityPenalty;
pub use formation::FormationPenalty;
pub use obstacle::ObstaclePenalty;
pub use peer::Peer;
pub use smoothness::SmoothnessPenalty;
pub use swarm::SwarmPenalty;
pub use time::TimePenalty;
pub use uniform::UniformPenalty;

use firefly_trajectory::Trajectory;
use nalgebra::{DMatrix, DVector};

pub trait Penalty {
    fn evaluate(&self, traj: &Trajectory) -> f64;
    fn accumulate(&self, traj: &Trajectory, weight: f64, acc: &mut Accumulator);
}

pub struct Cost {
    penalties: Vec<(f64, Box<dyn Penalty>)>,
}

impl Cost {
    #[must_use]
    pub fn new() -> Self {
        Self {
            penalties: Vec::new(),
        }
    }

    #[must_use]
    pub fn add(mut self, weight: f64, penalty: impl Penalty + 'static) -> Self {
        self.penalties.push((weight, Box::new(penalty)));
        self
    }

    #[must_use]
    pub fn evaluate(&self, traj: &Trajectory) -> f64 {
        self.penalties
            .iter()
            .map(|(w, p)| w * p.evaluate(traj))
            .sum()
    }

    /// 总偏导：(dF/dc, dF/dT 显式部分)。
    #[must_use]
    pub fn gradient(&self, traj: &Trajectory) -> (DMatrix<f64>, DVector<f64>) {
        let mut acc = Accumulator::new(traj.pieces());
        for (w, p) in &self.penalties {
            p.accumulate(traj, *w, &mut acc);
        }
        (acc.d_f_d_c, acc.d_f_d_t)
    }
}

impl Default for Cost {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
pub(crate) fn test_minco() -> firefly_trajectory::Minco {
    use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
    use nalgebra::Vector3;

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
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(
            &[
                nalgebra::Point3::new(1.5, 0.8, 0.3),
                nalgebra::Point3::new(2.8, 1.5, 0.6),
            ],
            &[0.8, 1.0, 1.2],
        )
        .unwrap()
}
