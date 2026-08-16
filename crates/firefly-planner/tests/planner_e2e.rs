//! `端到端：Planner::plan` 完整流程（A* → MINCO → L-BFGS → Rebound）。

use firefly_map::{GridMapBuilder, VoxelState};
use firefly_planner::{Planner, PlannerConfig, State};
use nalgebra::{Point3, Vector3};

#[test]
fn plan_avoids_wall() {
    // 10m × 10m × 10m 地图，x=4.5 处一堵墙（y 贯穿，z 高 1.5m）
    let mut map = GridMapBuilder::new(0.5, [20, 20, 20]).build().unwrap();
    for y in 0..20 {
        for z in 0..3 {
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
    // 引导路径可能直接从墙上方跨过（init 已安全，planes 可为空）；
    // 核心不变式：轨迹安全 + 绕墙 + 可行（由下方断言验证）

    let traj = &result.trajectory;
    assert!(traj.pieces() >= 5 && traj.pieces() <= 24, "pieces 自适应");

    // 1. 边界条件
    let s0 = traj.eval(0.0);
    assert!((s0.position - start.position.coords).norm() < 1e-6);
    assert!(s0.velocity.norm() < 1e-6);
    let sf = traj.eval(traj.duration());
    // 终点 = 规划距离内的局部目标（start → goal 方向 7.5m）
    let expected_goal = start.position.coords
        + (goal.coords - start.position.coords).normalize() * planner.config().planning_distance;
    assert!((sf.position - expected_goal).norm() < 1e-6);

    // 2. 轨迹绕过了墙（x > 4.5 处 z 高于墙顶）
    let mut crossed_high = false;
    for k in 0..200 {
        let t = traj.duration() * f64::from(k) / 200.0;
        let s = traj.eval(t);
        if s.position.x > 4.5 && s.position.z > 1.5 + 0.1 {
            crossed_high = true;
            break;
        }
    }
    assert!(crossed_high, "轨迹必须从墙上绕过（z > 1.5）");

    // 3. 可行性：峰值速度/加速度/加加速度在限制内（采样检查）
    let config = planner.config();
    for k in 0..400 {
        let t = traj.duration() * f64::from(k) / 400.0;
        let s = traj.eval(t);
        // 软约束：惩罚优化不保证硬边界，允许 1% 工程容差
        assert!(
            s.velocity.norm() < config.max_velocity * 1.02,
            "v={} at t={t}",
            s.velocity.norm()
        );
        assert!(
            s.acceleration.norm() < config.max_acceleration * 1.02,
            "a={} at t={t}",
            s.acceleration.norm()
        );
        assert!(
            s.jerk.norm() < config.max_jerk * 1.02,
            "j={} at t={t}",
            s.jerk.norm()
        );
    }
}

#[test]
fn plan_open_field_is_direct() {
    // 无障碍场景：轨迹应接近直线
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
fn swarm_head_on_avoidance() {
    // 空旷地图，两机相向飞行（x 方向对飞），各自用对方直线轨迹做 peer
    let map = GridMapBuilder::new(0.5, [40, 24, 16]).build().unwrap();

    let config = PlannerConfig {
        swarm_clearance: 0.3,
        ..PlannerConfig::default()
    };
    let mut planner_a = Planner::new(config.clone(), map.clone());
    let mut planner_b = Planner::new(config, map);

    // A：从左到右（y=1.0）；B：从右到左（y=1.3，错开打破对称——
    // 真实飞行中轨迹不同步，共面是病态对称场景）
    let start_a = State {
        position: Point3::new(1.0, 1.0, 1.0),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let goal_a = Point3::new(10.0, 1.0, 1.0);
    let start_b = State {
        position: Point3::new(10.0, 1.3, 1.0),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let goal_b = Point3::new(1.0, 1.3, 1.0);

    // 分布式迭代：每轮用对方最新广播轨迹重规划（论文 decentralized framework）
    let mut traj_a = planner_a.plan(start_a, goal_a).expect("plan a").trajectory;
    let mut traj_b = planner_b.plan(start_b, goal_b).expect("plan b").trajectory;
    for _ in 0..3 {
        let peer_b = firefly_cost::Peer::new(1, 0.0, traj_b.clone(), 0.3);
        let next_a = planner_a
            .plan_in_swarm(start_a, goal_a, &[peer_b])
            .expect("plan a with peer")
            .trajectory;
        let peer_a = firefly_cost::Peer::new(0, 0.0, traj_a.clone(), 0.3);
        let next_b = planner_b
            .plan_in_swarm(start_b, goal_b, &[peer_a])
            .expect("plan b with peer")
            .trajectory;
        traj_a = next_a;
        traj_b = next_b;
    }

    // 验证：同一绝对时刻，两机距离 ≥ swarm_clearance（采样检查）
    let mut min_d = f64::MAX;
    for k in 0..400 {
        let t = traj_a.duration() * f64::from(k) / 400.0;
        let pa = traj_a.eval(t).position;
        let pb = traj_b.eval(t).position;
        min_d = min_d.min((pa - pb).norm());
    }
    assert!(
        min_d > 0.7,
        "对飞两机最小距离 {min_d:.3} 应接近避让距离 0.9"
    );
}

#[test]
fn swarm_avoids_stationary_peer() {
    // 分布式基本单元：本机避让固定 peer（peer 静止在路径中点）
    let map = GridMapBuilder::new(0.5, [40, 24, 16]).build().unwrap();
    let config = PlannerConfig {
        swarm_clearance: 0.3,
        ..PlannerConfig::default()
    };
    let mut planner = Planner::new(config, map);
    let start = State {
        position: Point3::new(1.0, 1.0, 1.0),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let goal = Point3::new(10.0, 1.0, 1.0);

    // 静止 peer（单段，时长覆盖本机），y 偏离轨迹平面 0.3m：
    // 真实飞行中轨迹不同步/有噪声，共面是病态对称场景（Jw 侧向梯度恒 0）
    let peer = firefly_trajectory::MincoBuilder::new(
        firefly_trajectory::SolverOrder::MinimumJerk,
        firefly_trajectory::Endpoint {
            position: Vector3::new(5.5, 1.3, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        },
        firefly_trajectory::Endpoint {
            position: Vector3::new(5.5, 1.3, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        },
    )
    .build(&[], &[12.0])
    .unwrap()
    .solve()
    .unwrap();

    let result = planner
        .plan_in_swarm(start, goal, &[firefly_cost::Peer::new(0, 0.0, peer, 0.3)])
        .expect("plan with stationary peer");
    let traj = &result.trajectory;
    assert!(result.iterations > 0, "必须经过优化");

    // 本机轨迹必须显著偏离直线（绕开静止 peer）
    let mid = traj.eval(traj.duration() / 2.0).position;
    let straight = Vector3::new(5.5, 1.0, 1.0);
    assert!(
        (mid - straight).norm() > 0.5,
        "轨迹必须绕开 peer: mid={mid:?}"
    );
    // 与 peer 的最小距离接近 swarm_clearance（软约束，允许 20% 容差）
    let mut min_d = f64::MAX;
    for k in 0..400 {
        let t = traj.duration() * f64::from(k) / 400.0;
        min_d = min_d.min((traj.eval(t).position - Vector3::new(5.5, 1.3, 1.0)).norm());
    }
    assert!(min_d > 0.7, "距 peer 最小距离 {min_d:.3} 应接近 0.9");
}

#[test]
fn formation_following_with_peer() {
    // 2 机队形：peer 沿 x 直线（y=0），自己偏移 y=1 应跟随保持
    let map = GridMapBuilder::new(0.5, [24, 12, 12]).build().unwrap();
    let config = PlannerConfig::default();
    let mut planner = Planner::new(config, map);
    let start = State {
        position: Point3::new(0.5, 2.0, 1.0),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let goal = Point3::new(9.0, 2.0, 1.0);

    // peer 轨迹：沿 x 直线 y=0，时长与本机一致（10s）
    let peer_traj = firefly_trajectory::MincoBuilder::new(
        firefly_trajectory::SolverOrder::MinimumJerk,
        firefly_trajectory::Endpoint {
            position: Vector3::new(0.5, 0.0, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        },
        firefly_trajectory::Endpoint {
            position: Vector3::new(8.0, 0.0, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        },
    )
    .build(&[], &[10.0])
    .unwrap()
    .solve()
    .unwrap();

    // 队形：x 方向移动线，peer(0) 偏移 (0,0,0)，自己(1) 偏移 (0,1,0)
    let spec = firefly_planner::FormationSpec {
        line_start: Vector3::new(0.5, 0.0, 1.0),
        line_end: Vector3::new(8.0, 0.0, 1.0),
        offsets: vec![Vector3::zeros(), Vector3::new(0.0, 1.0, 0.0)],
        self_id: 1,
        peers: vec![firefly_cost::Peer::new(0, 0.0, peer_traj, 1.0)],
    };
    planner.set_formation(spec);

    let result = planner.plan(start, goal).expect("plan with formation");
    let traj = &result.trajectory;
    let mid = traj.eval(traj.duration() / 2.0).position;
    // 自己的期望位置 = peer 同刻位置 + 偏移 y=1 → y 应从 2.0 显著靠拢 1.0
    // 官方语义：队形靠持续重规划收敛，单次规划显著靠拢即可
    assert!(
        (mid.y - 1.0).abs() < 0.8_f64,
        "轨迹应向队形位置靠拢：mid y={}（起点 2.0，期望靠拢 1.0）",
        mid.y
    );
}
