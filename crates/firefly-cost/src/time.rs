//! 总时间惩罚：Jt = sum(T)。
//!
//! 对照官方 `poly_traj_optimizer.cpp:1209` `VirtualTGradCost`：
//! `costT = sum(T)·wei_time`，梯度 `gdVT = (gdRT + wei_time)·dT/dVT`。
//! firefly 将 `wei_time` 并入 `d_f_d_t`（此处 `+= weight`），由
//! `firefly-planner::objective::pack` 的链式 `dt·dT/dVT` 完成，不在
//! 映射层重复加。

use firefly_trajectory::Trajectory;

use crate::{Accumulator, Penalty};

pub struct TimePenalty;

impl Penalty for TimePenalty {
    fn evaluate(&self, traj: &Trajectory) -> f64 {
        traj.duration()
    }

    fn accumulate(&self, traj: &Trajectory, weight: f64, acc: &mut Accumulator) {
        for i in 0..traj.pieces() {
            acc.d_f_d_t[i] += weight;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_minco;

    #[test]
    fn time_penalty_is_total_duration() {
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        assert!((TimePenalty.evaluate(&traj) - 3.0).abs() < 1e-12);
    }
}
