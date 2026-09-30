//! 官方公式数值对照与最终输出契约；期望值独立于被测实现。
use super::*;
use firefly_optimize::Objective;

fn endpoint(position: Vector3<f64>) -> Endpoint {
    Endpoint {
        position,
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    }
}

fn planner(config: PlannerConfig) -> Planner {
    Planner::new(
        config,
        firefly_map::GridMapBuilder::new(0.5, [40, 40, 20])
            .build()
            .unwrap(),
    )
}

#[test]
fn assembled_obstacle_cost_matches_independent_upstream_formula() {
    for (hard, soft) in [(10_000., 5000.), (0., 5000.), (10_000., 0.)] {
        let config = PlannerConfig {
            weight_smoothness: 0.,
            weight_time: 0.,
            weight_feasibility: 0.,
            weight_sqrvariance: 0.,
            weight_obstacle: hard,
            weight_obstacle_soft: soft,
            ..PlannerConfig::default()
        };
        let planner = planner(config);
        for distance in [0.05f64, 0.3, 0.8] {
            let state = endpoint(Vector3::new(distance, 0., 0.));
            let m = MincoBuilder::new(SolverOrder::MinimumJerk, state, state)
                .build(&[], &[1.])
                .unwrap();
            let planes = vec![vec![Plane::new(Vector3::zeros(), Vector3::x())]; 6];
            let mut objective =
                planner.build_objective(state, state, &planes, &[], 1, 5, true, false, false);
            let actual = objective.evaluate(&Planner::pack(&m));
            let hard_error = (0.1 - distance).max(0.);
            let soft_error = (0.5 - distance).max(0.);
            // 官方跳过第 0 个约束点：单段 K=5 的积分权重和为 0.9 秒。
            let expected = 0.9
                * (hard * hard_error.powi(3)
                    + soft * 0.0025 * ((1. + soft_error.powi(2) / 0.0025).sqrt() - 1.));
            assert!(
                (actual - expected).abs() < 1e-8 * (1. + expected.abs()),
                "hard={hard} soft={soft} d={distance}: {actual} != {expected}"
            );
        }
    }
}

#[test]
fn reallocation_checks_actual_derivatives_and_preserves_boundary_state() {
    let planner = planner(PlannerConfig::default());
    let start = Endpoint {
        velocity: Vector3::new(0.5, 0., 0.),
        acceleration: Vector3::new(-5., 0., 0.),
        ..endpoint(Vector3::new(2., 2., 2.))
    };
    let end = endpoint(Vector3::new(2.1, 2., 2.));
    let m = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&[], &[0.2])
        .unwrap();
    let t = planner.ensure_feasible(&m).unwrap();
    for (time, expected) in [(0., start), (t.duration(), end)] {
        let state = t.eval(time);
        assert!((state.position - expected.position).norm() < 1e-8);
        assert!((state.velocity - expected.velocity).norm() < 1e-8);
        assert!((state.acceleration - expected.acceleration).norm() < 1e-8);
    }
    // 稠密求值独立于生产代码的极值求根实现。
    for k in 0..=10000 {
        let state = t.eval(t.duration() * f64::from(k) / 10000.);
        assert!(state.velocity.norm() <= 1.5 + 1e-8);
        assert!(state.acceleration.norm() <= 6. + 1e-8);
        assert!(state.jerk.norm() <= 10. + 1e-8);
    }
    let bad = Endpoint {
        velocity: Vector3::new(2., 0., 0.),
        ..start
    };
    let m = MincoBuilder::new(SolverOrder::MinimumJerk, bad, end)
        .build(&[], &[1.])
        .unwrap();
    assert!(planner.ensure_feasible(&m).is_err());
}

#[test]
fn swarm_acceptance_uses_clock_offset_and_both_clearances() {
    let planner = planner(PlannerConfig::default());
    let own = endpoint(Vector3::new(5., 5., 2.));
    let t = MincoBuilder::new(SolverOrder::MinimumJerk, own, own)
        .build(&[], &[0.2])
        .unwrap()
        .solve()
        .unwrap();
    let start = Endpoint {
        velocity: Vector3::x(),
        ..endpoint(Vector3::new(4., 5., 2.))
    };
    let end = Endpoint {
        position: Vector3::new(7., 5., 2.),
        ..start
    };
    let other = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&[], &[3.])
        .unwrap()
        .solve()
        .unwrap();
    for (offset, safe) in [(-1., false), (2., true)] {
        let peer = firefly_cost::Peer::new(1, offset, other.clone(), 0.5);
        assert_eq!(planner.swarm_safe(&t, &[peer]), safe, "offset={offset}");
    }
    let stationary = endpoint(Vector3::new(4., 5., 2.));
    let other = MincoBuilder::new(SolverOrder::MinimumJerk, stationary, stationary)
        .build(&[], &[1.])
        .unwrap()
        .solve()
        .unwrap();
    for clearance in [0.1, 0.5] {
        let peer = firefly_cost::Peer::new(1, 0., other.clone(), clearance);
        assert_eq!(
            planner.swarm_safe(&t, std::slice::from_ref(&peer)),
            clearance < 0.5
        );
        let penalty = Cost::new().add(1., SwarmPenalty::new(0.5, 2., 1., vec![peer]));
        let expected = 0.2 * (((0.5 + clearance) * 1.5).powi(2) - 1.).max(0.).powi(3);
        assert!((penalty.evaluate(&t) - expected).abs() < 1e-9);
    }
}

#[test]
fn cold_and_warm_plans_preserve_nonzero_terminal_velocity() {
    let mut planner = planner(PlannerConfig::default());
    let start = State {
        position: Point3::new(2., 2., 2.),
        velocity: Vector3::new(0.4, 0., 0.),
        acceleration: Vector3::zeros(),
    };
    let end = Endpoint {
        velocity: Vector3::new(0.6, 0., 0.),
        ..endpoint(Vector3::new(5., 2., 2.))
    };
    let cold = planner
        .plan_with_init(start, end, InitSource::ColdStart, true)
        .unwrap();
    let warm = planner
        .plan_with_init(
            start,
            end,
            InitSource::WarmStart {
                prev: &cold.trajectory,
                elapsed: 0.,
                glb_seg: 0.,
                guide_tail: &[],
            },
            true,
        )
        .unwrap();
    for result in [cold, warm] {
        let state = result.trajectory.eval(result.trajectory.duration());
        assert!((state.position - end.position).norm() < 1e-7);
        assert!((state.velocity - end.velocity).norm() < 1e-7);
        assert!((state.acceleration - end.acceleration).norm() < 1e-7);
    }
}

#[test]
fn swarm_acceptance_checks_reallocated_timeline() {
    let mut planner = planner(PlannerConfig::default());
    let start = endpoint(Vector3::new(2., 2., 2.));
    let end = endpoint(Vector3::new(3., 2., 2.));
    let m = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&[], &[0.2])
        .unwrap();
    let original = m.solve().unwrap();
    let reallocated = planner.ensure_feasible(&m).unwrap();
    let crossing_time = reallocated.duration() / 2.;
    let peer_start = Endpoint {
        position: Vector3::new(2.5, 2. + crossing_time, 2.),
        velocity: -Vector3::y(),
        acceleration: Vector3::zeros(),
    };
    let peer_end = Endpoint {
        position: peer_start.position + peer_start.velocity * 4.,
        ..peer_start
    };
    let peer_traj = MincoBuilder::new(SolverOrder::MinimumJerk, peer_start, peer_end)
        .build(&[], &[4.])
        .unwrap()
        .solve()
        .unwrap();
    let peers = [firefly_cost::Peer::new(1, 0., peer_traj, 0.)];
    assert!(planner.swarm_safe(&original, &peers));
    assert!(!planner.swarm_safe(&reallocated, &peers));
    let mut planes = vec![Vec::new(); 6];
    assert!(matches!(
        planner
            .try_finish(&m, &original, &peers, &mut planes, true)
            .unwrap(),
        FinishCheck::SwarmTooClose
    ));
}

#[test]
fn unavoidable_peer_at_fixed_boundary_is_rejected_with_bounded_restarts() {
    let mut planner = planner(PlannerConfig::default());
    let own = endpoint(Vector3::new(2., 2., 2.));
    let hover = MincoBuilder::new(SolverOrder::MinimumJerk, own, own)
        .build(&[], &[30.])
        .unwrap()
        .solve()
        .unwrap();
    let peers = [firefly_cost::Peer::new(1, 0., hover, 0.5)];
    let start = State {
        position: Point3::from(own.position),
        velocity: own.velocity,
        acceleration: own.acceleration,
    };
    let result = planner.plan_in_swarm(start, Point3::new(5., 2., 2.), &peers);
    assert!(result.is_err());
    assert!(planner.last_swarm_weight_mod() > 1.);
}
