//! 固定上游 C++ 的分项代价、系数/显式时间梯度和传播梯度对照。
//! 夹具由 reference/generate.py 编译固定版本的实际 MINCO 与代价函数生成。

#![allow(clippy::similar_names)]

use firefly_cost::{
    Accumulator, FeasibilityPenalty, FormationPenalty, MincoCost, ObstaclePenalty, Peer,
    SwarmPenalty,
};
use firefly_map::Plane;
use firefly_trajectory::{Endpoint, Minco, MincoBuilder, SolverOrder};
use nalgebra::{Point3, Vector3};

fn endpoints() -> (Endpoint, Endpoint) {
    (
        Endpoint {
            position: Vector3::zeros(),
            velocity: Vector3::new(0.5, -0.2, 0.1),
            acceleration: Vector3::new(0.1, 0.0, -0.1),
        },
        Endpoint {
            position: Vector3::new(4.0, 2.0, 1.0),
            velocity: Vector3::new(0.2, 0.1, -0.1),
            acceleration: Vector3::new(0.0, -0.1, 0.0),
        },
    )
}

fn minco() -> Minco {
    let (start, end) = endpoints();
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(
            &[Point3::new(1.5, 0.8, 0.3), Point3::new(2.8, 1.5, 0.6)],
            &[0.8, 1.0, 1.2],
        )
        .unwrap()
}

fn peer() -> Peer {
    let start = Endpoint {
        position: Vector3::new(1.8, 0.9, 0.4),
        velocity: Vector3::new(0.2, -0.1, 0.05),
        acceleration: Vector3::zeros(),
    };
    let end = Endpoint {
        position: Vector3::new(2.1, 1.1, 0.5),
        velocity: Vector3::new(0.3, 0.1, -0.05),
        ..start
    };
    let trajectory = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&[], &[1.4])
        .unwrap()
        .solve()
        .unwrap();
    Peer::new(1, 0.15, trajectory, 0.45)
}

fn costs(variant: usize) -> Vec<MincoCost> {
    let cutoff = if variant == 1 { 15 } else { 11 };
    let clearance = if variant == 2 { -100.0 } else { 0.1 };
    let soft_clearance = if variant == 2 { -100.0 } else { 0.5 };
    let limits = if variant == 2 {
        [1e9; 3]
    } else {
        [1.2, 2.0, 4.0]
    };
    let planes = (0..16)
        .map(|index| {
            if index % 4 == 0 {
                Vec::new()
            } else {
                vec![
                    Plane::new(Vector3::new(1.0, 0.3, 0.2), Vector3::x()),
                    Plane::new(Vector3::new(0.0, 1.6, 0.0), -Vector3::y()),
                ]
            }
        })
        .collect();
    vec![
        MincoCost {
            weight_smoothness: 1.2,
            ..MincoCost::default()
        },
        MincoCost {
            obstacle: Some((
                1.0,
                ObstaclePenalty::new(clearance, soft_clearance, 2.3, 0.7, 5, planes)
                    .with_two_thirds(cutoff),
            )),
            ..MincoCost::default()
        },
        MincoCost {
            swarm: Some((
                1.7,
                SwarmPenalty::new(0.5, 2.0, 1.0, vec![peer()]).with_two_thirds(cutoff),
            )),
            ..MincoCost::default()
        },
        MincoCost {
            feasibility: Some((
                0.3,
                FeasibilityPenalty::new(limits[0], limits[1], limits[2]),
            )),
            ..MincoCost::default()
        },
        MincoCost {
            weight_sqrvariance: 0.8,
            ..MincoCost::default()
        },
    ]
}

fn joint(
    cost: &MincoCost,
    trajectory: &firefly_trajectory::Trajectory,
    acc: &mut Accumulator,
) -> f64 {
    let smoothness = cost.init_and_get_smoothness_grad_cost(trajectory, acc);
    let terms = cost.add_pvaj_grad_cost(trajectory, acc, &mut Vec::new());
    // 独立偏导比较包括时间项；生产回调在伴随传播后加入该项。
    acc.d_f_d_t.add_scalar_mut(cost.weight_time);
    smoothness + terms.iter().sum::<f64>() + cost.weight_time * trajectory.duration()
}

fn close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 2e-9 * expected.abs().max(1.0),
        "actual={actual:.17e}, upstream={expected:.17e}"
    );
}

#[test]
fn joint_matches_pinned_cpp_per_term_and_all_gradient_spaces() {
    let mut fixture = include_str!("reference/ego_cost.txt")
        .lines()
        .filter(|line| !line.starts_with('#'))
        .flat_map(str::split_whitespace)
        .map(|value| value.parse::<f64>().unwrap());
    for variant in 0..3 {
        let minco = minco();
        let trajectory = minco.solve().unwrap();
        for row in 0..18 {
            for dim in 0..3 {
                close(
                    trajectory.coefficients()[(row, dim)],
                    fixture.next().unwrap(),
                );
            }
        }
        for cost in costs(variant) {
            let mut acc = Accumulator::new(3);
            let value = joint(&cost, &trajectory, &mut acc);
            close(value, fixture.next().unwrap());
            close(value, cost.evaluate(&trajectory));
            for row in 0..18 {
                for dim in 0..3 {
                    close(acc.d_f_d_c[(row, dim)], fixture.next().unwrap());
                }
            }
            for value in acc.d_f_d_t.iter() {
                close(*value, fixture.next().unwrap());
            }
            let (dq, dt) = minco
                .propagate_gradient(&trajectory, &acc.d_f_d_c, &acc.d_f_d_t)
                .unwrap();
            for dim in 0..3 {
                for waypoint in 0..2 {
                    close(dq[(dim, waypoint)], fixture.next().unwrap());
                }
            }
            for value in dt.iter() {
                close(*value, fixture.next().unwrap());
            }
        }
    }
    assert!(fixture.next().is_none());
}

#[test]
fn joint_preserves_formation_and_accumulator_reset() {
    let minco = minco();
    let trajectory = minco.solve().unwrap();
    let cost = MincoCost {
        samples_per_piece: 7,
        weight_smoothness: 1.2,
        weight_time: 0.4,
        feasibility: Some((0.3, FeasibilityPenalty::new(1.2, 2.0, 4.0).with_samples(7))),
        swarm: Some((
            0.5,
            SwarmPenalty::new(0.5, 2.0, 1.0, vec![peer()]).with_samples(7),
        )),
        weight_sqrvariance: 0.8,
        formation: Some((
            0.6,
            FormationPenalty::new(
                Vector3::zeros(),
                Vector3::new(4.0, 2.0, 0.0),
                vec![Vector3::zeros(), Vector3::new(0.3, 0.2, 0.1)],
                0,
                vec![peer()],
            )
            .with_samples(7),
        )),
        ..MincoCost::default()
    };
    let mut acc = Accumulator::new(1);
    let value = joint(&cost, &trajectory, &mut acc);
    close(value, cost.evaluate(&trajectory));
    let mut split = Accumulator::new(3);
    cost.gradient_into(&trajectory, &mut split);
    let (split_dc, split_dt) = (split.d_f_d_c, split.d_f_d_t);
    assert!((&split_dc - &acc.d_f_d_c).amax() < 1e-12 * split_dc.amax().max(1.0));
    assert!((&split_dt - &acc.d_f_d_t).amax() < 1e-12 * split_dt.amax().max(1.0));
    let (dq, dt) = minco
        .propagate_gradient(&trajectory, &acc.d_f_d_c, &acc.d_f_d_t)
        .unwrap();
    let (start, end) = endpoints();
    let waypoints: Vec<_> = minco.waypoints().collect();
    let times = [0.8, 1.0, 1.2];
    let evaluate = |points: &[Point3<f64>], times: &[f64]| {
        let trajectory = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(points, times)
            .unwrap()
            .solve()
            .unwrap();
        joint(&cost, &trajectory, &mut Accumulator::new(3))
    };
    for variable in 0..9 {
        let mut plus_points = waypoints.clone();
        let mut minus_points = waypoints.clone();
        let mut plus_times = times;
        let mut minus_times = times;
        let expected = if variable < 6 {
            plus_points[variable / 3][variable % 3] += 1e-6;
            minus_points[variable / 3][variable % 3] -= 1e-6;
            dq[(variable % 3, variable / 3)]
        } else {
            plus_times[variable - 6] += 1e-6;
            minus_times[variable - 6] -= 1e-6;
            dt[variable - 6]
        };
        let numeric =
            (evaluate(&plus_points, &plus_times) - evaluate(&minus_points, &minus_times)) / 2e-6;
        assert!((numeric - expected).abs() < 1e-4 * expected.abs().max(1.0));
    }
    close(joint(&cost, &trajectory, &mut acc), value);
    assert!((&split_dc - &acc.d_f_d_c).amax() < 1e-12 * split_dc.amax().max(1.0));
    let inactive = MincoCost {
        feasibility: Some((0.3, FeasibilityPenalty::new(1e9, 1e9, 1e9))),
        ..MincoCost::default()
    };
    assert_eq!(joint(&inactive, &trajectory, &mut acc), 0.0);
    assert_eq!(acc.d_f_d_c.amax(), 0.0);
    assert_eq!(acc.d_f_d_t.amax(), 0.0);
}
