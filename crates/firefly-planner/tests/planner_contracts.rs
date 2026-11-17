use firefly_map::{GridMapBuilder, VoxelState};
use firefly_planner::{Planner, PlannerConfig, State};
use firefly_trajectory::Endpoint;
use nalgebra::{Point3, Vector3};
mod support;

#[test]
fn wall_crossing_and_unreachable_goal() {
    let mut map = GridMapBuilder::new(0.5, [24, 24, 20]).build().unwrap();
    for y in 0..24 {
        map.set_state([12, y, 0], VoxelState::Occupied);
    }
    let config = PlannerConfig::default();
    let start = State {
        position: Point3::new(2., 6., 0.5),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let end = Endpoint {
        position: Vector3::new(9., 6., 0.5),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let mut planner = Planner::new(config.clone(), map.clone());
    let result = planner
        .plan_with_init(start, end, firefly_planner::InitSource::ColdStart, true)
        .expect("mandatory wall crossing");
    support::assert_trajectory_contract(&result.trajectory, start, end);
    for y in 0..24 {
        for z in 0..20 {
            map.set_state([12, y, z], VoxelState::Occupied);
        }
    }
    let mut planner = Planner::new(config, map);
    let error = planner
        .plan_with_init(start, end, firefly_planner::InitSource::ColdStart, true)
        .expect_err("solid separating wall is unreachable");
    assert!(matches!(
        error.kind(),
        firefly_error::ErrorKind::NotFound | firefly_error::ErrorKind::Convergence
    ));
}

#[test]
fn moving_boundaries_in_open_space() {
    let config = PlannerConfig::default();
    let map = GridMapBuilder::new(0.5, [30, 30, 20]).build().unwrap();
    let mut planner = Planner::new(config, map);
    let start = State {
        position: Point3::new(3., 4., 2.),
        velocity: Vector3::new(0.4, 0.1, 0.),
        acceleration: Vector3::new(0.1, 0., 0.),
    };
    let end = Endpoint {
        position: Vector3::new(7., 6., 2.),
        velocity: Vector3::new(0.6, 0.2, 0.),
        acceleration: Vector3::zeros(),
    };
    let result = planner
        .plan_with_init(start, end, firefly_planner::InitSource::ColdStart, true)
        .expect("mandatory moving boundaries");
    support::assert_trajectory_contract(&result.trajectory, start, end);
}
