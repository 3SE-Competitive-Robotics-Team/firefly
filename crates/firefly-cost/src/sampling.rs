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
