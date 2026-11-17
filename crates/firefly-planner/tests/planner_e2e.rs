use firefly_map::{GridMapBuilder, VoxelState};
use firefly_planner::{Planner, PlannerConfig, State};
use nalgebra::{Point3, Vector3};

#[test]
fn plan_avoids_wall() {
    let mut map = GridMapBuilder::new(0.5, [20, 20, 20]).build().unwrap();
    for y in 0..20 {
        for z in 0..1 {
            map.set_state([9, y, z], VoxelState::Occupied);
        }
    }

    let config = PlannerConfig::default();
    let mut planner = Planner::new(config, map);
    let start = State {
        position: Point3::new(0.5, 0.5, 0.5),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let goal = Point3::new(9.0, 0.5, 0.5);

    let result = planner.plan(start, goal).expect("plan succeeds");

    let traj = &result.trajectory;
    assert!(traj.pieces() >= 5 && traj.pieces() <= 24, "pieces 自适应");

    let s0 = traj.eval(0.0);
    assert!((s0.position - start.position.coords).norm() < 1e-6);
    assert!(s0.velocity.norm() < 1e-6);
    let sf = traj.eval(traj.duration());
    let expected_goal = start.position.coords
        + (goal.coords - start.position.coords).normalize() * planner.config().planning_distance;
    assert!((sf.position - expected_goal).norm() < 1e-6);

    let mut crossed_high = false;
    for k in 0..200 {
        let t = traj.duration() * f64::from(k) / 200.0;
        let s = traj.eval(t);
        if s.position.x > 4.5 && s.position.z > 1.0 + 0.1 {
            crossed_high = true;
            break;
        }
    }
    assert!(crossed_high, "轨迹必须从墙上绕过（z > 1.0）");
    for k in 0..=400 {
        let s = traj.eval(traj.duration() * f64::from(k) / 400.0);
        assert!(!planner.map_ref().is_occupied_inflated(s.position));
    }
}

#[test]
fn plan_open_field_is_direct() {
    let map = GridMapBuilder::new(0.5, [20, 20, 20]).build().unwrap();
    let mut planner = Planner::new(PlannerConfig::default(), map);
    let start = State {
        position: Point3::new(0.5, 0.5, 0.5),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let goal = Point3::new(8.0, 0.5, 0.5);

    let result = planner.plan(start, goal).expect("plan succeeds");
    assert!(result.planes.is_empty(), "无障碍场景不应生成平面");

    let traj = &result.trajectory;
    for k in 0..200 {
        let t = traj.duration() * f64::from(k) / 200.0;
        let s = traj.eval(t);
        assert!(
            (s.position.y - 0.5).abs() < 5e-3,
            "y={} at t={t}",
            s.position.y
        );
        assert!(
            (s.position.z - 0.5).abs() < 5e-3,
            "z={} at t={t}",
            s.position.z
        );
    }
}

#[test]
fn plan_avoids_isolated_column() {
    let mut map = firefly_map::GridMapBuilder::new(0.5, [40, 40, 10])
        .with_obstacles_inflation(0.3)
        .build()
        .unwrap();
    for z in 0..4 {
        for y in 19..21 {
            map.set_state([8, y, z], firefly_map::VoxelState::Occupied);
        }
    }
    map.inflate_obstacles();

    let config = firefly_planner::PlannerConfig::default();
    let mut planner = firefly_planner::Planner::new(config, map);
    let start = firefly_planner::State {
        position: nalgebra::Point3::new(0.5, 10.0, 1.0),
        velocity: nalgebra::Vector3::zeros(),
        acceleration: nalgebra::Vector3::zeros(),
    };
    let goal = nalgebra::Point3::new(19.0, 10.0, 1.0);
    let result = planner.plan(start, goal).expect("plan succeeds");
    let traj = &result.trajectory;
    assert!((traj.eval(0.0).position - start.position.coords).norm() < 1e-6);
    let sf = traj.eval(traj.duration());
    let exp = start.position.coords
        + (goal.coords - start.position.coords).normalize() * planner.config().planning_distance;
    assert!((sf.position - exp).norm() < 1e-6);

    for k in 0..400 {
        let s = traj.eval(traj.duration() * f64::from(k) / 400.0);
        assert!(
            !planner.map_ref().is_occupied_inflated(s.position),
            "轨迹点穿入障碍膨胀区 at t={:.2} p={:?}",
            traj.duration() * f64::from(k) / 400.0,
            s.position
        );
    }
    let mut max_dev = 0.0f64;
    for k in 0..200 {
        let s = traj.eval(traj.duration() * f64::from(k) / 200.0);
        max_dev = max_dev.max((s.position.y - 10.0).abs());
    }
    assert!(max_dev > 0.15, "应横向绕开柱（偏移 {max_dev:.3}）");
}
