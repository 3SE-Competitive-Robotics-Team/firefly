//! 集群避碰惩罚。
//!
//! 论文 Eq. S24–S28：本机轨迹 `u_p(t)` 与每架其他机轨迹 `k_p(τ)` 在**同一绝对时刻**
//! 保持椭球距离 ≥ Cw。E = diag(1,1,1/c)，c>1 缩短 z 轴缓解下洗。
//! 采样点用绝对时间（对齐其他机），前面段时长变化会移动采样点
//! （`Accumulator::add_absolute，论文` Eq. S28）。

use firefly_trajectory::{Sample, Trajectory};
use nalgebra::Vector3;

use crate::{Accumulator, Peer, Penalty};

pub struct SwarmPenalty {
    /// 本机集群安全距离 `Cw`（与对方 `des_clearance` 之和构成避让距离）。
    pub self_clearance: f64,
    /// 椭球 z 轴系数（官方 a = 2.0：z 距离贡献 1/a²，防下洗更严）。
    pub ellipsoid_a: f64,
    /// 椭球 xy 轴系数（官方 b = 1.0）。
    pub ellipsoid_b: f64,
    pub peers: Vec<Peer>,
    pub samples_per_piece: usize,
}

impl SwarmPenalty {
    #[must_use]
    pub fn new(self_clearance: f64, ellipsoid_a: f64, ellipsoid_b: f64, peers: Vec<Peer>) -> Self {
        Self {
            self_clearance,
            ellipsoid_a,
            ellipsoid_b,
            peers,
            samples_per_piece: 5,
        }
    }

    #[must_use]
    pub fn with_samples(mut self, samples: usize) -> Self {
        self.samples_per_piece = samples;
        self
    }
}

impl Penalty for SwarmPenalty {
    fn evaluate(&self, traj: &Trajectory) -> f64 {
        let mut cost = 0.0;
        for (i, ti) in traj.durations().iter().enumerate() {
            let weight = ti / self.samples_per_piece as f64;
            for j in 0..=self.samples_per_piece {
                let tau = j as f64 / self.samples_per_piece as f64;
                let t_abs = absolute_time(traj, i, *ti, tau);
                let s = traj.eval(t_abs);
                cost += weight * self.point_cost(&s, t_abs);
            }
        }
        cost
    }

    fn accumulate(&self, traj: &Trajectory, weight: f64, acc: &mut Accumulator) {
        for (i, ti) in traj.durations().iter().enumerate() {
            let sample_weight = weight * ti / self.samples_per_piece as f64;
            for j in 0..=self.samples_per_piece {
                let tau = j as f64 / self.samples_per_piece as f64;
                let t_abs = absolute_time(traj, i, *ti, tau);
                let s = traj.eval(t_abs);
                // 逐 peer 累加：每个 peer 在同一绝对时刻求值，
                // 时间移动用相对速度（本机 − peer）
                for peer in &self.peers {
                    let ps = eval_at(peer, t_abs);
                    let diff = s.position - ps.position;
                    let c = self.clearance(peer);
                    let excess = c * c - self.d2(diff);
                    if excess <= 0.0 {
                        continue;
                    }
                    let point_cost = excess * excess * excess;
                    let d_p = -6.0 * excess * excess * self.e_mul(diff) * sample_weight;
                    acc.d_f_d_t[i] += weight * point_cost / self.samples_per_piece as f64;
                    acc.add_absolute(
                        i,
                        tau,
                        *ti,
                        &s,
                        ps.velocity,
                        d_p,
                        Vector3::zeros(),
                        Vector3::zeros(),
                        Vector3::zeros(),
                    );
                }
            }
        }
    }
}

impl SwarmPenalty {
    /// 椭球距离平方：`d² = dz²/a² + (dx²+dy²)/b²`（官方 `ellip_dist2`）。
    fn d2(&self, diff: Vector3<f64>) -> f64 {
        let ia2 = 1.0 / (self.ellipsoid_a * self.ellipsoid_a);
        let ib2 = 1.0 / (self.ellipsoid_b * self.ellipsoid_b);
        diff.z * diff.z * ia2 + (diff.x * diff.x + diff.y * diff.y) * ib2
    }

    /// E·diff（梯度方向因子）
    fn e_mul(&self, diff: Vector3<f64>) -> Vector3<f64> {
        let ia2 = 1.0 / (self.ellipsoid_a * self.ellipsoid_a);
        let ib2 = 1.0 / (self.ellipsoid_b * self.ellipsoid_b);
        Vector3::new(diff.x * ib2, diff.y * ib2, diff.z * ia2)
    }

    /// 避让距离：`CLEARANCE = (Cw_self + des_clearance) × 1.5`（官方补偿轻微约束违反）。
    fn clearance(&self, peer: &Peer) -> f64 {
        (self.self_clearance + peer.clearance) * 1.5
    }

    fn point_cost(&self, s: &Sample, t_abs: f64) -> f64 {
        self.peers
            .iter()
            .map(|peer| {
                let ps = eval_at(peer, t_abs);
                let c2 = self.clearance(peer);
                let c2 = c2 * c2;
                let excess = c2 - self.d2(s.position - ps.position);
                if excess <= 0.0 {
                    0.0
                } else {
                    excess * excess * excess
                }
            })
            .sum()
    }
}

/// 在绝对时刻求值 peer 轨迹；超出轨迹时长时外推匀速（官方行为）。
fn eval_at(peer: &Peer, t_abs: f64) -> firefly_trajectory::Sample {
    let duration = peer.traj.duration();
    if t_abs < duration {
        peer.traj.eval(t_abs)
    } else {
        let s = peer.traj.eval(duration);
        let exceed = t_abs - duration;
        firefly_trajectory::Sample {
            position: s.position + s.velocity * exceed,
            velocity: s.velocity,
            acceleration: s.acceleration,
            jerk: s.jerk,
            snap: s.snap,
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
    use crate::test_minco;

    fn single_peer() -> (firefly_trajectory::Minco, firefly_trajectory::Trajectory) {
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        // 幽灵机：静止在轨迹中点附近（会触发避碰）
        let peer = firefly_trajectory::MincoBuilder::new(
            firefly_trajectory::SolverOrder::MinimumJerk,
            firefly_trajectory::Endpoint {
                position: Vector3::new(2.0, 0.8, 0.3),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
            firefly_trajectory::Endpoint {
                position: Vector3::new(2.0, 0.8, 0.3),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
        )
        .build(&[], &[traj.duration()])
        .unwrap()
        .solve()
        .unwrap();
        (minco, peer)
    }

    #[test]
    fn peer_cost_zero_and_positive() {
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        // 远处 peer → 零代价
        let far = firefly_trajectory::MincoBuilder::new(
            firefly_trajectory::SolverOrder::MinimumJerk,
            firefly_trajectory::Endpoint {
                position: Vector3::new(50.0, 50.0, 50.0),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
            firefly_trajectory::Endpoint {
                position: Vector3::new(50.0, 50.0, 50.0),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
        )
        .build(&[], &[traj.duration()])
        .unwrap()
        .solve()
        .unwrap();
        let p = SwarmPenalty::new(0.5, 2.0, 1.0, vec![Peer::new(0, 0.0, far, 0.5)]);
        assert_eq!(p.evaluate(&traj), 0.0);
        // 近处 peer（同一轨迹，距离恒 0）→ 正代价
        let p = SwarmPenalty::new(0.5, 2.0, 1.0, vec![Peer::new(0, 0.0, traj.clone(), 0.5)]);
        assert!(p.evaluate(&traj) > 0.0);
    }

    #[test]
    #[allow(clippy::many_single_char_names)]
    fn gradient_matches_numerical() {
        let (minco, peer) = single_peer();
        let traj = minco.solve().unwrap();
        let p = SwarmPenalty::new(0.5, 2.0, 1.0, vec![Peer::new(0, 0.0, peer.clone(), 0.5)]);

        // 点梯度验证（手写 ∂excess³/∂p = −6·excess²·E(p−p_k)）
        let h = 1e-6;
        let s = traj.eval(1.0);
        let (ia2, ib2) = (
            1.0 / (p.ellipsoid_a * p.ellipsoid_a),
            1.0 / (p.ellipsoid_b * p.ellipsoid_b),
        );
        let cw = (0.5 + 0.5) * 1.5; // CLEARANCE = (Cw_self + des_clearance) × 1.5
        for dim in 0..3 {
            let f = |x: f64| {
                let mut s2 = s;
                s2.position[dim] = x;
                p.point_cost(&s2, 1.0)
            };
            let numeric = (f(s.position[dim] + h) - f(s.position[dim] - h)) / (2.0 * h);
            let ps = peer.clone().eval(1.0);
            let diff = s.position - ps.position;
            let d2 = diff.z * diff.z * ia2 + (diff.x * diff.x + diff.y * diff.y) * ib2;
            let excess = cw * cw - d2;
            let emul = Vector3::new(diff.x * ib2, diff.y * ib2, diff.z * ia2);
            let analytic = if excess > 0.0 {
                -6.0 * excess * excess * emul[dim]
            } else {
                0.0
            };
            assert!(
                (numeric - analytic).abs() < 1e-6,
                "dim={dim} numeric={numeric} analytic={analytic}"
            );
        }
    }
}
