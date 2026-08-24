//! 全链路梯度验证：所有 cost 项经 MINCO 传播后的解析梯度
//! vs 对 {q, T} 直接中心差分。

use firefly_cost::{Cost, FeasibilityPenalty, ObstaclePenalty, SmoothnessPenalty, TimePenalty};
use firefly_map::Plane;
use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
use nalgebra::{Point3, Vector3};

fn build_minco() -> firefly_trajectory::Minco {
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
    // 总时长 6s：峰值速度约 1.2 m/s < vm=1.5，加速度/加加速度远低于限制
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(
            &[Point3::new(1.5, 0.8, 0.3), Point3::new(2.8, 1.5, 0.6)],
            &[2.0, 2.0, 2.0],
        )
        .unwrap()
}

fn build_cost() -> Cost {
    let planes = vec![
        // x=0.2：轨迹从 x=0 出发，浅度交叉，惩罚温和
        Plane::new(Vector3::new(0.2, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
        Plane::new(Vector3::new(3.8, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0)),
    ];
    // 论文语义：每个约束点私有拥有平面
    let per_point: Vec<Vec<Plane>> = (0..3 * 6).map(|_| planes.clone()).collect();
    Cost::new()
        .add(1.0, SmoothnessPenalty)
        .add(10.0, TimePenalty)
        .add(10_000.0, FeasibilityPenalty::new(1.5, 6.0, 20.0))
        .add(
            10_000.0,
            ObstaclePenalty::new(0.1, 0.5, 5000.0, 5, per_point),
        )
}

fn rebuild(q: &[Point3<f64>], t: &[f64]) -> firefly_trajectory::Minco {
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
        .build(q, t)
        .unwrap()
}

#[test]
fn analytic_gradient_matches_numerical_end_to_end() {
    let minco = build_minco();
    let cost = build_cost();
    let traj = minco.solve().unwrap();

    let j0 = cost.evaluate(&traj);
    let (d_f_d_c, d_f_d_t) = cost.gradient(&traj);
    let (dq, dt) = minco.propagate_gradient(&traj, &d_f_d_c, &d_f_d_t).unwrap();

    let q0: Vec<_> = minco.waypoints().collect();
    let t0: Vec<f64> = (0..minco.pieces())
        .map(|i| minco.piece_duration(i))
        .collect();
    let h = 1e-6;

    let eval = |q: &[Point3<f64>], t: &[f64]| {
        let m = rebuild(q, t);
        cost.evaluate(&m.solve().unwrap())
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
                (numeric - analytic).abs() < 5e-4 * (1.0 + analytic.abs()),
                "dq[{i}][{dim}] analytic={analytic} numeric={numeric} (j0={j0})"
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
            (numeric - analytic).abs() < 5e-4 * (1.0 + analytic.abs()),
            "dt[{i}] analytic={analytic} numeric={numeric} (j0={j0})"
        );
    }

    // 顺带验证 DMatrix 形状（防 API 回归）
    assert_eq!(dq.nrows(), 3);
    assert_eq!(dq.ncols(), minco.pieces() - 1);
    assert_eq!(dt.len(), minco.pieces());
}
