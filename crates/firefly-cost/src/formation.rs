//! 队形保持惩罚（官方 EGO-Planner-v2 `formationGradCostP` 语义）。
//!
//! 目标点不是固定轨迹，而是从其他机位置动态推断：
//! - 队形沿直线 `line_start` → `line_end` 移动（方向 a）
//! - 每架机在队形坐标系中的偏移 `offsets[id]`（x 沿队形线、y 垂直、z 高度）
//! - 进度 l = 其他机位置在队形线上的平均投影（扣除队形 x 偏移）
//! - 目标点 `tar_p = O + 旋转(a)·(偏移 + l·a)`
//!
//! 梯度（官方）：
//! - `grad_p = 2·wei·(p − tar_p)`
//! - `grad_t = wei·dJ·(v − a·dl_dt)`（相对队形运动）
//! - `grad_prev_t = wei·dJ·(−a·dl_dt)`（前面段：仅目标随队形移动）
//!
//! 官方只对前 2/3 约束点施力（接近目标时解散队形）。

use firefly_trajectory::Trajectory;
use nalgebra::Vector3;

use crate::sampling::{sample_index, trapezoid_weight};
use crate::{Accumulator, Peer, Penalty};

pub struct FormationPenalty {
    pub line_start: Vector3<f64>,
    pub line_end: Vector3<f64>,
    pub offsets: Vec<Vector3<f64>>,
    pub self_id: usize,
    pub peers: Vec<Peer>,
    pub samples_per_piece: usize,
    /// 前 2/3 截断(`two_thirds_id`);`None` = 不限。
    two_thirds: Option<usize>,
}

impl FormationPenalty {
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        line_start: Vector3<f64>,
        line_end: Vector3<f64>,
        offsets: Vec<Vector3<f64>>,
        self_id: usize,
        peers: Vec<Peer>,
    ) -> Self {
        Self {
            line_start,
            line_end,
            offsets,
            self_id,
            peers,
            samples_per_piece: 5,
            two_thirds: None,
        }
    }

    #[must_use]
    pub fn with_samples(mut self, samples: usize) -> Self {
        self.samples_per_piece = samples;
        self
    }

    /// 施加官方前 2/3 截断。
    #[must_use]
    pub fn with_two_thirds(mut self, id: usize) -> Self {
        self.two_thirds = Some(id);
        self
    }

    /// 队形轴与进度（官方：`a` 归一化，`l`/`dl_dt` 从其他机推断）。
    #[must_use]
    #[allow(clippy::many_single_char_names)]
    pub fn formation_state(&self, t_abs: f64) -> (Vector3<f64>, Vector3<f64>, f64, f64) {
        let o = self.line_start;
        let a = (self.line_end - self.line_start).normalize();
        let mut l = 0.0;
        let mut dl_dt = 0.0;
        let mut n = 0usize;
        for peer in &self.peers {
            if peer.drone_id == self.self_id || peer.drone_id >= self.offsets.len() {
                continue;
            }
            let local_t = t_abs - peer.start_time;
            let (p, v) = if local_t < peer.traj.duration() {
                let s = peer.traj.eval(local_t);
                (s.position, s.velocity)
            } else {
                // 超出轨迹：外推匀速（官方处理）
                let s = peer.traj.eval(peer.traj.duration());
                (
                    s.position + s.velocity * (local_t - peer.traj.duration()),
                    s.velocity,
                )
            };
            l += (p - o).dot(&a) - self.offsets[peer.drone_id].x;
            dl_dt += a.dot(&v);
            n += 1;
        }
        if n > 0 {
            l /= n as f64;
            dl_dt /= n as f64;
        }
        (o, a, l, dl_dt)
    }

    /// 目标点（官方 `tar_p` 计算：xy 用 `a` 的 2D 旋转，z 独立）。
    #[must_use]
    pub fn target(&self, o: Vector3<f64>, a: Vector3<f64>, l: f64) -> Vector3<f64> {
        let f = self.offsets[self.self_id];
        Vector3::new(
            a.x * (f.x + l) - a.y * f.y + o.x,
            a.y * (f.x + l) + a.x * f.y + o.y,
            a.z * l + f.z + o.z,
        )
    }
}

impl Penalty for FormationPenalty {
    // 与官方公式逐行对应(k/i/j/o/a/l/s),单字符命名保可读性
    #[allow(clippy::many_single_char_names)]
    fn evaluate(&self, traj: &Trajectory) -> f64 {
        if self.peers.is_empty() {
            return 0.0;
        }
        let k = self.samples_per_piece;
        let mut cost = 0.0;
        for (i, ti) in traj.durations().iter().enumerate() {
            let step = ti / k as f64;
            for j in 0..=k {
                let tau = j as f64 / k as f64;
                if let Some(t) = self.two_thirds
                    && {
                        let idx = sample_index(i, j, k);
                        idx == 0 || idx > t
                    }
                {
                    continue;
                }
                let t_abs = absolute_time(traj, i, *ti, tau);
                let (o, a, l, _) = self.formation_state(t_abs);
                let s = traj.eval(t_abs);
                let omg = trapezoid_weight(j, k);
                cost += omg * step * (s.position - self.target(o, a, l)).norm_squared();
            }
        }
        cost
    }

    // 与官方公式逐行对应(k/i/j/o/a/l/s),单字符命名保可读性
    #[allow(clippy::many_single_char_names)]
    fn accumulate(&self, traj: &Trajectory, weight: f64, acc: &mut Accumulator) {
        if self.peers.is_empty() {
            return;
        }
        let k = self.samples_per_piece;
        for (i, ti) in traj.durations().iter().enumerate() {
            let step = ti / k as f64;
            for j in 0..=k {
                let tau = j as f64 / k as f64;
                if let Some(t) = self.two_thirds
                    && {
                        let idx = sample_index(i, j, k);
                        idx == 0 || idx > t
                    }
                {
                    continue;
                }
                let omg = trapezoid_weight(j, k);
                let t_abs = absolute_time(traj, i, *ti, tau);
                let (o, a, l, dl_dt) = self.formation_state(t_abs);
                let s = traj.eval(t_abs);
                let tar = self.target(o, a, l);
                let d_p = 2.0 * (s.position - tar) * (weight * omg * step);
                let point_cost = (s.position - tar).norm_squared();
                // 采样权重 (omg·T/K) 对 T 的导数:omg·f/K
                acc.d_f_d_t[i] += weight * omg * point_cost / k as f64;
                // 官方梯度：本段用 v − a·dl_dt（相对队形），前面段仅 −a·dl_dt
                acc.add_absolute(
                    i,
                    tau,
                    *ti,
                    &s,
                    a * dl_dt,
                    d_p,
                    Vector3::zeros(),
                    Vector3::zeros(),
                    Vector3::zeros(),
                );
            }
        }
    }
}

fn absolute_time(traj: &Trajectory, piece: usize, duration: f64, tau: f64) -> f64 {
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
    use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};

    fn peers_on_line(y: f64) -> Vec<Peer> {
        // 1 架 peer 沿 y 直线飞行（队形线 x 方向）
        let traj = MincoBuilder::new(
            SolverOrder::MinimumJerk,
            Endpoint {
                position: Vector3::new(0.0, y, 1.0),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
            Endpoint {
                position: Vector3::new(8.0, y, 1.0),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
        )
        .build(&[], &[10.0])
        .unwrap()
        .solve()
        .unwrap();
        vec![Peer::new(0, 0.0, traj, 1.0)]
    }

    #[test]
    fn formation_cost_zero_and_positive() {
        // 对齐（自己与 peer 同队形偏移）→ 零代价
        let traj = MincoBuilder::new(
            SolverOrder::MinimumJerk,
            Endpoint {
                position: Vector3::new(0.0, 1.0, 1.0),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
            Endpoint {
                position: Vector3::new(8.0, 1.0, 1.0),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
        )
        .build(&[], &[10.0])
        .unwrap()
        .solve()
        .unwrap();
        let p = FormationPenalty::new(
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(8.0, 0.0, 1.0),
            vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)],
            1,
            peers_on_line(1.0),
        );
        assert!(p.evaluate(&traj) < 1e-6, "对齐应有零代价");
        // 偏移（peer 在 y=0，自己在 y=2）→ 正代价
        let traj = MincoBuilder::new(
            SolverOrder::MinimumJerk,
            Endpoint {
                position: Vector3::new(0.0, 2.0, 1.0),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
            Endpoint {
                position: Vector3::new(8.0, 2.0, 1.0),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
        )
        .build(&[], &[10.0])
        .unwrap()
        .solve()
        .unwrap();
        let p = FormationPenalty::new(
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(8.0, 0.0, 1.0),
            vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)],
            1,
            peers_on_line(0.0),
        );
        assert!(p.evaluate(&traj) > 0.0, "偏移应有正代价");
    }
}
