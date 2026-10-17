//! 官方 `addPVAGradCost2CT` 采样约定:
//! - 每段 K 个约束点 + 尾端点(N·K+1 总点数,段边界不重复);
//! - 采样点索引 `i_dp = i·K + j`(段 i,采样 j,边界点与下一段 j=0 同索引);
//! - 梯形积分权重 `omg = (j==0 ‖ j==K) ? 0.5 : 1.0`,权重 `omg·T/K`;
//! - 障碍/集群/队形仅对前 2/3 约束点(`two_thirds_id`)施力。

/// 采样点索引(官方 `getInitConstraintPoints` 布局)。
#[must_use]
pub fn sample_index(piece: usize, j: usize, samples_per_piece: usize) -> usize {
    piece * samples_per_piece + j
}

/// 梯形积分权重(官方 `omg`)。
#[must_use]
pub fn trapezoid_weight(j: usize, samples_per_piece: usize) -> f64 {
    if j == 0 || j == samples_per_piece {
        0.5
    } else {
        1.0
    }
}

use firefly_trajectory::{Sample, Trajectory};
use nalgebra::Vector3;

/// 一条轨迹在 `i·K+j` 约束点网格上的采样缓存。
///
/// 多个惩罚项共用同一批采样点，避免重复 `eval_piece`。
pub struct Samples {
    pub samples_per_piece: usize,
    grid: Vec<Sample>,
}

impl Samples {
    #[must_use]
    pub fn new(traj: &Trajectory, samples_per_piece: usize) -> Self {
        let k = samples_per_piece;
        let mut grid = Vec::with_capacity(traj.pieces() * (k + 1));
        for (i, ti) in traj.durations().iter().enumerate() {
            for j in 0..=k {
                grid.push(traj.eval_piece(i, j as f64 / k as f64 * ti));
            }
        }
        Self {
            samples_per_piece: k,
            grid,
        }
    }

    #[must_use]
    pub fn get(&self, piece: usize, j: usize) -> &Sample {
        &self.grid[piece * (self.samples_per_piece + 1) + j]
    }

    /// 写入唯一约束点序列（段边界只保留一次），顺序与
    /// `constraint_sample_points` 一致。
    pub fn unique_positions(&self, out: &mut Vec<Vector3<f64>>) {
        out.clear();
        let pieces = self.grid.len() / (self.samples_per_piece + 1);
        for i in 0..pieces {
            let j_max = if i + 1 == pieces {
                self.samples_per_piece
            } else {
                self.samples_per_piece - 1
            };
            for j in 0..=j_max {
                out.push(self.get(i, j).position);
            }
        }
    }
}
