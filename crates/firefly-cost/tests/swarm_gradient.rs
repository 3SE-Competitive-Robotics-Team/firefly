//! `SwarmPenalty` 端到端梯度验证：绝对时间采样模式的 ∂F/∂q、∂F/∂T
//! vs 对 {q, T} 直接中心差分（论文 Eq. S28 的正确性验证）。

use firefly_cost::{Cost, SwarmPenalty};
use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
use nalgebra::{Point3, Vector3};

fn build_scene() -> (firefly_trajectory::Minco, firefly_trajectory::Trajectory) {
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
    let minco = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(
            &[Point3::new(1.5, 0.8, 0.3), Point3::new(2.8, 1.5, 0.6)],
            &[2.0, 2.0, 2.0],
        )
        .unwrap();
    let traj = minco.solve().unwrap();

    // 幽灵机：与轨迹相交的对向飞行（触发避碰 + 绝对时间梯度）
    let peer = MincoBuilder::new(
        SolverOrder::MinimumJerk,
        Endpoint {
            position: Vector3::new(2.5, 0.8, 0.3),
            velocity: Vector3::new(-0.4, 0.0, 0.0),
            acceleration: Vector3::zeros(),
        },
        Endpoint {
            position: Vector3::new(1.0, 1.2, 0.5),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        },
    )
    .build(
        &[Point3::new(2.0, 1.0, 0.4)],
        &[traj.duration() / 2.0, traj.duration() / 2.0],
    )
    .unwrap()
    .solve()
    .unwrap();

    (minco, peer)
}

#[test]
fn swarm_gradient_matches_numerical() {
    let (minco, peer) = build_scene();
    let traj = minco.solve().unwrap();
    let swarm = SwarmPenalty::new(
        0.5,
        2.0,
        1.0,
        vec![firefly_cost::Peer::new(0, 0.0, peer.clone(), 0.7)],
    )
    .with_samples(5);
    let cost = Cost::new().add(1.0, swarm);
    let j0 = cost.evaluate(&traj);
    assert!(j0 > 0.0, "对飞场景必须触发避碰惩罚 (j0={j0})");

    let (d_f_d_c, d_f_d_t) = cost.gradient(&traj);
    let (dq, dt) = minco.propagate_gradient(&traj, &d_f_d_c, &d_f_d_t).unwrap();

    let q0: Vec<_> = minco.waypoints().collect();
    let t0: Vec<f64> = (0..minco.pieces())
        .map(|i| minco.piece_duration(i))
        .collect();
    let h = 1e-6;

    let eval = |q: &[Point3<f64>], t: &[f64]| {
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
        let m = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(q, t)
            .unwrap();
        let swarm = SwarmPenalty::new(
            0.5,
            2.0,
            1.0,
            vec![firefly_cost::Peer::new(0, 0.0, peer.clone(), 0.7)],
        )
        .with_samples(5);
        Cost::new().add(1.0, swarm).evaluate(&m.solve().unwrap())
    };

    for (i, qi) in q0.iter().enumerate() {
        for dim in 0..3 {
            let mut qp = q0.clone();
            let mut qm = q0.clone();
            let mut p = *qi;
            p[dim] += h;
            qp[i] = p;
            p = *qi;
            p[dim] -= h;
            qm[i] = p;
            let numeric = (eval(&qp, &t0) - eval(&qm, &t0)) / (2.0 * h);
            let analytic = dq[(dim, i)];
            assert!(
                (numeric - analytic).abs() < 1e-5 * (1.0 + analytic.abs()),
                "dq[{i}][{dim}] analytic={analytic} numeric={numeric}"
            );
        }
    }

    for (i, ti) in t0.iter().enumerate() {
        let mut tp = t0.clone();
        let mut tm = t0.clone();
        tp[i] = ti + h;
        tm[i] = ti - h;
        let numeric = (eval(&q0, &tp) - eval(&q0, &tm)) / (2.0 * h);
        let analytic = dt[i];
        assert!(
            (numeric - analytic).abs() < 1e-5 * (1.0 + analytic.abs()),
            "dt[{i}] analytic={analytic} numeric={numeric}"
        );
    }
}
