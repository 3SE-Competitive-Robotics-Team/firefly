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
mod sampling;
mod smoothness;
mod swarm;
mod time;
mod uniform;

pub use accumulator::Accumulator;
pub use feasibility::FeasibilityPenalty;
pub use formation::FormationPenalty;
pub use obstacle::ObstaclePenalty;
pub use peer::Peer;
pub use sampling::Samples;
pub use smoothness::SmoothnessPenalty;
pub use swarm::SwarmPenalty;
pub use time::TimePenalty;
pub use uniform::UniformPenalty;

use firefly_trajectory::Trajectory;
use nalgebra::{DMatrix, DVector};

pub trait Penalty {
    fn evaluate(&self, traj: &Trajectory) -> f64;
    fn accumulate(&self, traj: &Trajectory, weight: f64, acc: &mut Accumulator);

    /// 使用调用方预先算好的约束点采样；默认退化为独立采样。
    fn evaluate_with(&self, traj: &Trajectory, _samples: &Samples) -> f64 {
        self.evaluate(traj)
    }

    /// 使用调用方预先算好的约束点采样；默认退化为独立采样。
    fn accumulate_with(
        &self,
        traj: &Trajectory,
        _samples: &Samples,
        weight: f64,
        acc: &mut Accumulator,
    ) {
        self.accumulate(traj, weight, acc);
    }
}

pub struct Cost {
    penalties: Vec<(f64, Box<dyn Penalty>)>,
    samples_per_piece: usize,
}

impl Cost {
    #[must_use]
    pub fn new() -> Self {
        Self {
            penalties: Vec::new(),
            samples_per_piece: 5,
        }
    }

    /// 共用采样网格的每段点数（默认 5）。
    #[must_use]
    pub fn with_samples(mut self, samples_per_piece: usize) -> Self {
        self.samples_per_piece = samples_per_piece;
        self
    }

    #[must_use]
    pub fn add(mut self, weight: f64, penalty: impl Penalty + 'static) -> Self {
        self.penalties.push((weight, Box::new(penalty)));
        self
    }

    #[must_use]
    pub fn evaluate(&self, traj: &Trajectory) -> f64 {
        let samples = Samples::new(traj, self.samples_per_piece);
        self.evaluate_with(traj, &samples)
    }

    /// 使用调用方缓存的约束点采样求值。
    #[must_use]
    pub fn evaluate_with(&self, traj: &Trajectory, samples: &Samples) -> f64 {
        self.penalties
            .iter()
            .map(|(w, p)| w * p.evaluate_with(traj, samples))
            .sum()
    }

    #[must_use]
    pub fn samples_per_piece(&self) -> usize {
        self.samples_per_piece
    }

    /// 总偏导：(dF/dc, dF/dT 显式部分)。
    #[must_use]
    pub fn gradient(&self, traj: &Trajectory) -> (DMatrix<f64>, DVector<f64>) {
        let mut acc = Accumulator::new(traj.pieces());
        self.gradient_into(traj, &mut acc);
        (acc.d_f_d_c, acc.d_f_d_t)
    }

    /// 复用调用方 `Accumulator`，避免每次梯度回调重新分配。
    pub fn gradient_into(&self, traj: &Trajectory, acc: &mut Accumulator) {
        let samples = Samples::new(traj, self.samples_per_piece);
        self.gradient_into_with(traj, &samples, acc);
    }

    /// 使用调用方缓存的约束点采样累加梯度。
    pub fn gradient_into_with(&self, traj: &Trajectory, samples: &Samples, acc: &mut Accumulator) {
        acc.reset(traj.pieces());
        for (w, p) in &self.penalties {
            p.accumulate_with(traj, samples, *w, acc);
        }
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
