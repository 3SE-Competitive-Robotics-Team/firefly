//! 集群避碰惩罚(官方 `swarmGradCostP`)。
//!
//! 椭球距离 `ellip_dist2 = dz²/a² + (dx²+dy²)/b²`(a=2, b=1,缓解下洗),
//! 避让距离 `CLEARANCE = (Cw·1.5)`(官方对轻微约束违反的补偿),
//! 惩罚 `wei_swarm·max{(CLEARANCE²−ellip_dist2),0}³`。
//! 仅对前 2/3 约束点施力;采样梯形权重 `omg·T/K`。

use firefly_trajectory::{Sample, Trajectory};
use nalgebra::Vector3;

use crate::sampling::{sample_index, trapezoid_weight};
use crate::{Accumulator, Peer, Penalty};

pub struct SwarmPenalty {
    /// 本机集群安全距离 `Cw`(官方 `swarm_clearance`)。
    pub self_clearance: f64,
    /// 椭球 z 轴系数(官方 a = 2.0:z 距离贡献 1/a²,防下洗更严)。
    pub ellipsoid_a: f64,
    /// 椭球 xy 轴系数(官方 b = 1.0)。
    pub ellipsoid_b: f64,
    pub peers: Vec<Peer>,
    pub samples_per_piece: usize,
    /// 前 2/3 截断(`two_thirds_id`);`None` = 不限。
    two_thirds: Option<usize>,
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
}

impl Penalty for SwarmPenalty {
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
                let s = traj.eval(t_abs);
                let omg = trapezoid_weight(j, k);
                cost += omg * step * self.point_cost(&s, t_abs);
            }
        }
        cost
    }

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
                let s = traj.eval(t_abs);
                // 逐 peer 累加:每个 peer 在同一绝对时刻求值,
                // 时间移动用相对速度(本机 − peer)
                for peer in &self.peers {
                    let ps = eval_at(peer, t_abs);
                    let diff = s.position - ps.position;
                    let c = self.clearance();
                    let excess = c * c - self.d2(diff);
                    if excess <= 0.0 {
                        continue;
                    }
                    let point_cost = excess * excess * excess;
                    let d_p = -6.0 * excess * excess * self.e_mul(diff) * (weight * omg * step);
                    acc.d_f_d_t[i] += weight * omg * point_cost / k as f64;
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
    /// 椭球距离平方:`d² = dz²/a² + (dx²+dy²)/b²`(官方 `ellip_dist2`)。
    fn d2(&self, diff: Vector3<f64>) -> f64 {
        let ia2 = 1.0 / (self.ellipsoid_a * self.ellipsoid_a);
        let ib2 = 1.0 / (self.ellipsoid_b * self.ellipsoid_b);
        diff.z * diff.z * ia2 + (diff.x * diff.x + diff.y * diff.y) * ib2
    }

    /// E·diff(梯度方向因子)
    fn e_mul(&self, diff: Vector3<f64>) -> Vector3<f64> {
        let ia2 = 1.0 / (self.ellipsoid_a * self.ellipsoid_a);
        let ib2 = 1.0 / (self.ellipsoid_b * self.ellipsoid_b);
        Vector3::new(diff.x * ib2, diff.y * ib2, diff.z * ia2)
    }

    /// 避让距离(官方:`CLEARANCE = swarm_clearance × 1.5`)。
    fn clearance(&self) -> f64 {
        self.self_clearance * 1.5
    }

    fn point_cost(&self, s: &Sample, t_abs: f64) -> f64 {
        self.peers
            .iter()
            .map(|peer| {
                let ps = eval_at(peer, t_abs);
                let c = self.clearance();
                let c2 = c * c;
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

/// 在绝对时刻求值 peer 轨迹;超出轨迹时长时外推匀速(官方行为)。
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
        // 幽灵机:静止在轨迹中点附近(会触发避碰)
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
        // 近处 peer(同一轨迹,距离恒 0)→ 正代价
        let p = SwarmPenalty::new(0.5, 2.0, 1.0, vec![Peer::new(0, 0.0, traj.clone(), 0.5)]);
        assert!(p.evaluate(&traj) > 0.0);
    }

    #[test]
    #[allow(clippy::many_single_char_names)]
    fn gradient_matches_numerical() {
        let (_, peer) = single_peer();
        let minco = test_minco();
        let traj = minco.solve().unwrap();
        let p = SwarmPenalty::new(0.5, 2.0, 1.0, vec![Peer::new(0, 0.0, peer.clone(), 0.5)]);

        // 点梯度验证(手写 ∂excess³/∂p = −6·excess²·E(p−p_k))
        let h = 1e-6;
        let s = traj.eval(1.0);
        let (ia2, ib2) = (
            1.0 / (p.ellipsoid_a * p.ellipsoid_a),
            1.0 / (p.ellipsoid_b * p.ellipsoid_b),
        );
        let cw = 0.5 * 1.5; // CLEARANCE = Cw × 1.5
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
