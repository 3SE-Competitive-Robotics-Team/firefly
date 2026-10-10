//! 真实 CAD 导出地图的消费端验收；从停机坪上空向前爬升避开台阶膨胀层。
mod support;

use firefly_map::MapFile;
use firefly_planner::{InitSource, Planner, PlannerConfig, State};
use firefly_trajectory::Endpoint;
use nalgebra::{Point3, Vector3};

#[test]
#[ignore = "requires generated RMUC2026 assets"]
fn exported_rmuc_map_supports_pad_departure() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../apps/planner/maps/rmuc2026.ffmap");
    let map = MapFile::from_file(path).unwrap().to_grid_map(0.0).unwrap();
    let config = PlannerConfig::default();
    let mut planner = Planner::new(config.clone(), map);
    let start = State {
        position: Point3::new(-13., 0., 1.4),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let end = Endpoint {
        position: Vector3::new(-11., 0., 2.0),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    assert!(
        !planner
            .map_ref()
            .is_occupied_inflated(start.position.coords)
    );
    assert!(!planner.map_ref().is_occupied_inflated(end.position));
    let result = planner
        .plan_with_init(start, end, InitSource::ColdStart, true)
        .expect("RMUC pad departure must be feasible");
    support::assert_trajectory_contract(&result.trajectory, planner.map_ref(), &config, start, end);
}
