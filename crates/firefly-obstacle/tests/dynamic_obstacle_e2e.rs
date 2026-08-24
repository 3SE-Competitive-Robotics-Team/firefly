//! 端到端：动态障碍预测轨迹作为 Peer，planner 应提前避让。
//!
//! 论文 Dynamic obstacle avoidance：预测轨迹与无人机轨迹同通道处理，
//! 仅 Cw 按障碍体积不同（`Peer::clearance`）。

use firefly_map::GridMapBuilder;
use firefly_obstacle::{MovingObstacle, ObstaclePredictor, PredictorConfig};
use firefly_planner::{Planner, PlannerConfig, State};
use nalgebra::{Point3, Vector2, Vector3};

#[test]
#[allow(clippy::similar_names)]
fn planner_avoids_dynamic_obstacle() {
    firefly_observability::init();
    // 空旷地图，本机沿 x 轴飞行
    // y 方向留足避让空间（地图 12×8×6m）
    let map = GridMapBuilder::new(0.5, [24, 16, 12]).build().unwrap();
    let config = PlannerConfig::default();
    let mut planner = Planner::new(config, map);
    let start = State {
        position: Point3::new(0.5, 3.0, 1.0),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let goal = Point3::new(9.0, 3.0, 1.0);

    // 动态障碍：横向穿过本机路径（y=3.4 向下渐停,接近本机 y=3.0 直线），
    // 需小幅 y 避让。官方语义：CLEARANCE = swarm_clearance × 1.5 = 0.75
    //（对照 v2 `swarmGradCostP`，不叠加 peer 体积）——障碍须进入 0.75 范围
    // 才触发避让,故让障碍起始点贴近路径。
    let obstacle = MovingObstacle::new(
        Vector2::new(4.5, 3.4),
        Vector2::new(0.0, -0.2),
        -core::f64::consts::FRAC_PI_2,
        1.0,
    );
    let predictor = ObstaclePredictor::new(PredictorConfig::default());
    // 无油门：阻尼下滑行（官方 "gradually stop like a real obstacle"）
    let predicted = predictor
        .predict_trajectory(&obstacle, 0.0, 0.0, 1.0)
        .expect("预测轨迹生成");

    // 预测轨迹作为 peer，障碍体积 → clearance 0.8
    let peer = firefly_cost::Peer::new(99, 0.0, predicted.clone(), 0.5); // des_clearance（官方默认）
    // 无障碍 peer 时自由轨迹与预测轨迹的最小距离应很小（初始不安全）
    let free0 = planner.plan(start, goal).unwrap();
    let mut free0_min = f64::MAX;
    for k in 0..200 {
        let t = free0.trajectory.duration() * f64::from(k) / 200.0;
        if t > predicted.duration() {
            continue;
        }
        let p = free0.trajectory.eval(t).position;
        let op = predicted.eval(t).position;
        free0_min = free0_min.min((p - op).norm());
    }
    assert!(
        free0_min < 0.75,
        "场景前提：自由轨迹应进入官方避让范围 CLEARANCE=0.75（min={free0_min:.3}）"
    );
    let result = planner
        .plan_in_swarm(start, goal, &[peer])
        .expect("避让动态障碍");

    let traj = &result.trajectory;
    // 本机轨迹与预测轨迹的最小距离应 ≥ 0.8·CLEARANCE（软约束容差）
    let mut min_d = f64::MAX;
    for k in 0..200 {
        let t = traj.duration() * f64::from(k) / 200.0;
        if t > predicted.duration() {
            continue;
        }
        let p = traj.eval(t).position;
        let op = predicted.eval(t).position;
        min_d = min_d.min((p - op).norm());
    }
    assert!(min_d > 0.6, "轨迹应避开动态障碍预测路径：min_d={min_d:.3}");

    // 对照：无 peer 时轨迹沿直线（会穿过障碍路径）
    let free = planner.plan(start, goal).unwrap();
    let mut free_min = f64::MAX;
    for k in 0..200 {
        let t = free.trajectory.duration() * f64::from(k) / 200.0;
        if t > predicted.duration() {
            continue;
        }
        let p = free.trajectory.eval(t).position;
        let op = predicted.eval(t).position;
        free_min = free_min.min((p - op).norm());
    }
    assert!(
        free_min * 1.2 < min_d,
        "避让后距离（{min_d:.3}）应显著大于自由轨迹（{free_min:.3}）"
    );
}
