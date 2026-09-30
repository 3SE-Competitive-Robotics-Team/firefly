//! 固定种子安全属性测试：返回轨迹必须满足完整契约，拒绝必须是显式有界不收敛。
//! 求解成功率不在本测试中定义；确定性任务完成由 mandatory_* 用例验证。

use firefly_map::{GridMapBuilder, VoxelState};
use firefly_planner::{Planner, PlannerConfig, State};
use firefly_trajectory::Endpoint;
use nalgebra::{Point3, Vector3};
mod support;

/// 简单确定性 LCG（避免引入 rand 依赖，种子可复现）。
struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }

    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, min: f64, max: f64) -> f64 {
        min + (max - min) * self.next()
    }
}

struct RandomMap {
    map: firefly_map::GridMap,
    start: Point3<f64>,
    goal: Point3<f64>,
}

/// 圆柱限制在地图中央；端点与上下绕行空间由几何构造保证空闲。
fn random_map(rng: &mut Lcg) -> RandomMap {
    let mut map = GridMapBuilder::new(0.5, [40, 40, 16]).build().unwrap();
    for _ in 0..6 {
        let cx = rng.range(7., 13.);
        let cy = rng.range(7., 13.);
        let radius = rng.range(0.5, 1.5);
        let height = rng.range(1., 3.);
        for ix in 0..40 {
            for iy in 0..40 {
                let x = f64::midpoint(ix as f64, 0.5);
                let y = f64::midpoint(iy as f64, 0.5);
                if (x - cx).powi(2) + (y - cy).powi(2) <= radius.powi(2) {
                    for iz in 0..(height / 0.5).ceil() as usize {
                        map.set_state([ix, iy, iz], VoxelState::Occupied);
                    }
                }
            }
        }
    }
    let start = Point3::new(2., 10., 1.);
    let goal = Point3::new(18., 10., 1.);
    assert!(!map.is_occupied_inflated(start.coords) && !map.is_occupied_inflated(goal.coords));
    RandomMap { map, start, goal }
}

fn check_case(index: usize) {
    let mut rng = Lcg::new(20_260_815);
    let mut fixture = random_map(&mut rng);
    for _ in 0..index {
        fixture = random_map(&mut rng);
    }
    let config = PlannerConfig {
        use_multitopology_trajs: true,
        ..PlannerConfig::default()
    };
    let mut planner = Planner::new(config.clone(), fixture.map.clone());
    let start = State {
        position: fixture.start,
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let end = Endpoint {
        position: fixture.goal.coords,
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    match planner.plan_with_init(start, end, firefly_planner::InitSource::ColdStart, true) {
        Ok(result) => support::assert_trajectory_contract(
            &result.trajectory,
            planner.map_ref(),
            &config,
            start,
            end,
        ),
        Err(error) => {
            assert_eq!(
                error.kind(),
                firefly_error::ErrorKind::Convergence,
                "seed=20260815 case={index}: {error}"
            );
            assert_eq!(error.status(), firefly_error::ErrorStatus::Temporary);
        }
    }
}
macro_rules! cases {
    ($($name:ident: $index:expr),* $(,)?) => { $(#[test] fn $name() { check_case($index); })* };
}
cases! {
    safety_seed_00: 0,
    safety_seed_01: 1,
    safety_seed_02: 2,
    safety_seed_03: 3,
    safety_seed_04: 4,
    safety_seed_05: 5,
    safety_seed_06: 6,
    safety_seed_07: 7,
    safety_seed_08: 8,
    safety_seed_09: 9,
    safety_seed_10: 10,
    safety_seed_11: 11,
    safety_seed_12: 12,
    safety_seed_13: 13,
    safety_seed_14: 14,
    safety_seed_15: 15,
    safety_seed_16: 16,
    safety_seed_17: 17,
    safety_seed_18: 18,
    safety_seed_19: 19,
    safety_seed_20: 20,
    safety_seed_21: 21,
    safety_seed_22: 22,
    safety_seed_23: 23,
    safety_seed_24: 24,
    safety_seed_25: 25,
    safety_seed_26: 26,
    safety_seed_27: 27,
    safety_seed_28: 28,
    safety_seed_29: 29
}

#[test]
fn mandatory_wall_crossing_and_unreachable_goal() {
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
    support::assert_trajectory_contract(&result.trajectory, planner.map_ref(), &config, start, end);
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
fn mandatory_moving_boundaries_in_open_space() {
    let config = PlannerConfig::default();
    let map = GridMapBuilder::new(0.5, [30, 30, 20]).build().unwrap();
    let mut planner = Planner::new(config.clone(), map);
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
    support::assert_trajectory_contract(&result.trajectory, planner.map_ref(), &config, start, end);
}
