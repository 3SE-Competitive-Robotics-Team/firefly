use crate::InitSource;
use crate::config::PlannerConfig;
use crate::manager::*;
const YAW_DOT_MAX: f64 = 2.0 * std::f64::consts::PI;
const YAW_DOT_DOT_MAX: f64 = 5.0 * std::f64::consts::PI;
const GROUND_LOOKAHEAD_DIST: f64 = 2.0;
const STOP_PIECE_DURATION: f64 = 1.0;
use crate::{
    ExecState, Planner, Reference, ReplanFsm, State, TickReport, TrajectoryCommand,
    TrajectoryServer,
};
use firefly_error::Result;
use firefly_map::{GridMapBuilder, VoxelState};
use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder, Trajectory};
use nalgebra::{Point3, Vector3};

/// 组装真实 FSM、规划管理器和轨迹消费者；cycle 推进一次完整规划周期。
struct PlannerManager {
    fsm: ReplanFsm,
    server: TrajectoryServer,
    accepted: Option<(u32, f64)>,
}
struct Report {
    event: TickReport,
    reference: Option<Reference>,
}
impl std::ops::Deref for Report {
    type Target = TickReport;
    fn deref(&self) -> &TickReport {
        &self.event
    }
}
impl std::ops::Deref for PlannerManager {
    type Target = crate::PlannerManager;
    fn deref(&self) -> &Self::Target {
        self.fsm.manager()
    }
}
impl std::ops::DerefMut for PlannerManager {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.fsm.manager_mut()
    }
}
impl PlannerManager {
    fn with_planner(
        planner: Planner,
        options: ManagerOptions,
        start: Vector3<f64>,
        goal: Vector3<f64>,
    ) -> Result<Self> {
        let server = TrajectoryServer::new(
            planner.config().yaw_hold,
            planner.config().heartbeat_timeout,
        );
        let mut fsm = ReplanFsm::new(crate::PlannerManager::with_planner(
            planner, options, start, goal,
        )?);
        fsm.set_goal(0.0, Some(state_at(start)), goal)?;
        let _ = fsm.exec_tick(0.0, Some(state_at(start)));
        let _ = fsm.exec_tick(0.0, Some(state_at(start)));
        Ok(Self {
            fsm,
            server,
            accepted: None,
        })
    }
    fn synchronize(&mut self) {
        if let Some(local) = self.fsm.manager().local() {
            let identity = (local.traj_id, local.start_time);
            if self.accepted != Some(identity) {
                self.server.receive(TrajectoryCommand {
                    trajectory: local.traj.clone(),
                    start_time: local.start_time,
                    traj_id: local.traj_id,
                });
                self.accepted = Some(identity);
            }
        } else {
            self.server.invalidate();
            self.accepted = None;
        }
    }
    fn cycle(&mut self, now: f64, measured: Option<State>) -> Report {
        let mut event = self.fsm.safety_tick(now, measured);
        let exec = self.fsm.exec_tick(now, measured);
        event.replanned |= exec.replanned;
        event.finished |= exec.finished;
        if matches!(
            self.fsm.state(),
            ExecState::ReplanTraj | ExecState::GenNewTraj
        ) {
            let exec = self.fsm.exec_tick(now, measured);
            event.replanned |= exec.replanned;
            event.finished |= exec.finished;
        }
        event.emergency_stop = self.fsm.is_emergency();
        self.synchronize();
        self.server.heartbeat(now);
        let reference = self.server.sample(now, now);
        Report { event, reference }
    }
    fn is_finished(&self) -> bool {
        self.fsm.is_finished()
    }
    fn is_mandatory_stopped(&self) -> bool {
        self.fsm.is_mandatory_stopped()
    }
    fn yaw_state(&self) -> (f64, f64) {
        self.server.yaw_state()
    }
    fn update_yaw(&mut self, pos: Vector3<f64>, target: Vector3<f64>, now: f64) -> (f64, f64) {
        self.server.update_yaw(pos, target, now)
    }
    fn set_goal(&mut self, now: f64, measured: Option<State>, goal: Vector3<f64>) -> Result<()> {
        self.fsm.set_goal(now, measured, goal)
    }
    fn invalidate_localization(&mut self, now: f64, measured: State) -> Result<()> {
        self.fsm.invalidate_localization(now, measured)
    }
    fn mandatory_stop(&mut self, now: f64, measured: Option<State>) {
        self.fsm.mandatory_stop(now, measured);
        let _ = self.fsm.exec_tick(now, measured);
    }
    fn enter_emergency_stop(&mut self, now: f64, measured: Option<State>) {
        self.fsm.request_emergency();
        let _ = self.fsm.exec_tick(now, measured);
    }
    fn trigger_emergency_stop_disable_failsafe(&mut self, now: f64, measured: Option<State>) {
        self.fsm
            .trigger_emergency_stop_disable_failsafe(now, measured);
        let _ = self.fsm.exec_tick(now, measured);
    }
    fn initial_plan(&mut self, now: f64, measured: Option<State>) {
        if let Some(measured) = measured {
            self.fsm
                .manager_mut()
                .plan_from_global_traj(now, measured, 10, false);
        }
    }
}

fn state_at(p: Vector3<f64>) -> State {
    State {
        position: Point3::from(p),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    }
}

/// 沿折线从 `from`(视为在路径上)累计弧长 `arc` 的点(测试预言机,
/// 与实现独立——只做简单的逐段行走)。
fn arc_point(path: &[Vector3<f64>], from: Vector3<f64>, arc: f64) -> Vector3<f64> {
    let mut remaining = arc;
    let mut prev = from;
    for p in path.iter().skip(1) {
        let seg = *p - prev;
        let len = seg.norm();
        if remaining <= len {
            return prev + seg * (remaining / len.max(1e-12));
        }
        remaining -= len;
        prev = *p;
    }
    *path.last().unwrap()
}

/// 空旷地图上的管理器:起点 (1,1,1) → 终点 (10,1,1)。可选注入虚拟墙
/// `(ground, ceil)`（米，世界坐标 z）。
fn open_manager_with_wall(virtual_wall: Option<(f64, f64)>) -> PlannerManager {
    let mut builder = GridMapBuilder::new(0.5, [40, 24, 16]);
    if let Some((ground, ceil)) = virtual_wall {
        builder = builder.with_virtual_wall(ground, ceil);
    }
    let planner = Planner::new(PlannerConfig::default(), builder.build().unwrap());
    PlannerManager::with_planner(
        planner,
        ManagerOptions::default(),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(10.0, 1.0, 1.0),
    )
    .unwrap()
}

fn open_manager() -> PlannerManager {
    open_manager_with_wall(None)
}

#[test]
fn single_waypoint_matches_analytic_quintic_and_valid_22m_horizon() {
    let start = Vector3::new(
        -12.747_030_258_178_711,
        -0.012_813_190_929_591_656,
        1.441_254_615_783_691_4,
    );
    let goal = Vector3::new(9., 0., 2.);
    let map = GridMapBuilder::new(0.15, [213, 121, 48])
        .with_origin(Vector3::new(-15.975, -9.075, -1.125))
        .with_virtual_wall(-0.1, 3.)
        .build()
        .unwrap();
    let mut manager = PlannerManager::with_planner(
        Planner::new(PlannerConfig::default(), map),
        ManagerOptions::default(),
        start,
        goal,
    )
    .unwrap();
    assert_eq!(manager.global_path(), &[start, goal]);
    let traj = &manager.global.traj;
    let duration = traj.duration();
    // 零端速/加速度的单段解析解 s(u)=10u³−15u⁴+6u⁵。
    for i in 0..=100 {
        let u = f64::from(i) / 100.;
        let s = 10. * u.powi(3) - 15. * u.powi(4) + 6. * u.powi(5);
        let ds = (30. * u.powi(2) - 60. * u.powi(3) + 30. * u.powi(4)) / duration;
        let sample = traj.eval(u * duration);
        assert!((sample.position - (start + (goal - start) * s)).norm() < 1e-9);
        assert!((sample.velocity - (goal - start) * ds).norm() < 1e-9);
        assert!(manager.map().index_of(sample.position).is_some());
        assert!(sample.position.z > -0.1 && sample.position.z < 3.);
        if i > 0 && i < 100 {
            let dt = 1e-3;
            let finite_difference = (traj.eval(u * duration + dt).position
                - traj.eval(u * duration - dt).position)
                / (2. * dt);
            assert!((sample.velocity - finite_difference).norm() < 1e-7);
        }
    }
    let h = manager.horizon(start);
    assert!(manager.map().index_of(h.target).is_some());
    assert!((h.target - start).norm() >= manager.options.planning_horizon);
    assert!((start.z..=goal.z).contains(&h.target.z));
    assert!(!h.touch_goal);
}

#[test]
fn distant_goal_is_a_guide_outside_the_rolling_window() {
    let start = Vector3::new(1., 1., 1.);
    let goal = Vector3::new(30., 1., 1.);
    assert_eq!(global_waypoints(start, goal).unwrap(), vec![start, goal]);
    assert!(global_waypoints(start, Vector3::repeat(f64::NAN)).is_err());
}

#[test]
fn localization_correction_invalidates_trajectory_preserving_goal_and_stop() {
    let mut manager = open_manager();
    let _ = manager.cycle(0., Some(state_at(Vector3::new(1., 1., 1.))));
    let goal = manager.goal();
    manager
        .invalidate_localization(1., state_at(Vector3::new(2., 1., 1.)))
        .unwrap();
    assert!(manager.local().is_none());
    assert_eq!(manager.goal(), goal);
    manager.mandatory_stop(1., None);
    manager
        .invalidate_localization(2., state_at(Vector3::new(2., 1., 1.)))
        .unwrap();
    assert!(manager.fsm.mandatory_stop);
}

#[test]
#[ignore = "requires generated RMUC2026 assets"]
fn rmuc_22m_initial_trajectory_is_executable() {
    let file = firefly_map::MapFile::from_file(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../apps/planner/maps/rmuc2026-2.0.0.ffmap"),
    )
    .unwrap();
    let mut map = file.to_grid_map(0.4).unwrap();
    map.set_virtual_wall(firefly_map::VirtualWall {
        ground: -0.1,
        ceil: 2.6,
    });
    let start = Vector3::new(
        -12.747_030_258_178_711,
        -0.012_813_190_929_591_656,
        1.441_254_615_783_691_4,
    );
    let goal = Vector3::new(9., 0., 2.);
    let config = PlannerConfig {
        obstacle_clearance: 0.4,
        ..Default::default()
    };
    let mut manager = PlannerManager::with_planner(
        Planner::new(config, map),
        ManagerOptions::default(),
        start,
        goal,
    )
    .unwrap();
    let h = manager.horizon(start);
    assert!(manager.map().index_of(h.target).is_some());
    assert!((start.z..=goal.z).contains(&h.target.z));
    let result = manager
        .planner
        .plan_with_init(
            State {
                position: Point3::from(start),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
            h.endpoint(),
            InitSource::ColdStart,
            h.touch_goal,
        )
        .expect("the specified 22m mission must produce an executable initial trajectory");
    assert!(crate::collision::trajectory_is_free(
        manager.map(),
        &result.trajectory
    ));
}

#[test]
fn initial_local_goal_preserves_global_velocity() {
    let mut manager = open_manager();
    let start = state_at(Vector3::new(1., 1., 1.));
    let target = manager.horizon(start.position.coords);
    assert!(!target.touch_goal && target.target_vel.norm() > 0.1);
    manager.initial_plan(100., Some(start));
    let local = manager.local().expect("initial moving local goal");
    let end = local.traj.eval(local.traj.duration());
    assert!((end.position - target.target).norm() < 1e-6);
    assert!((end.velocity - target.target_vel).norm() < 1e-6);
    assert!(!manager.touch_goal);
}

#[test]
fn registered_peers_are_rebased_to_planning_epoch() {
    let mut manager = open_manager();
    manager.register_swarm_traj(1, hover_traj(Vector3::new(4., 3., 2.), 10.), 98., 0.37);
    let peers = manager.peers_at(100.);
    assert_eq!(peers.len(), 1);
    assert!((peers[0].start_time + 2.).abs() < 1e-12);
    assert!((peers[0].clearance - 0.37).abs() < 1e-12);
    assert!((peers[0].sample_at(0.5).position - Vector3::new(4., 3., 2.)).norm() < 1e-12);
}

#[test]
fn global_traj_time_parameterization() {
    // 全局轨迹多项式化（官方 planGlobalTrajWaypoints）：时间参数化的
    // MINCO 轨迹，首末状态完整（零速零加速）、内点为任务航点。
    let m = open_manager();
    let start = m.global_path()[0];
    let goal = m.goal().coords;
    let g = &m.global;
    assert!(
        (g.traj.eval(0.0).position - start).norm() < 1e-6,
        "getPos(0) = 起点"
    );
    assert!(
        (g.traj.eval(g.traj.duration()).position - goal).norm() < 1e-6,
        "getPos(duration) = 终点"
    );
    assert!(g.traj.duration() > 0.0, "全局轨迹有非零时长");
    assert!(g.traj.eval(0.0).velocity.norm() < 1e-6, "起点零速");
    assert!(
        g.traj.eval(g.traj.duration()).velocity.norm() < 1e-6,
        "终点零速"
    );
    // des_vel 迭代保证最大速度不超 max_vel（官方达标即断/再缩放一次）
    let max_vel = PlannerConfig::default().max_velocity;
    assert!(
        crate::planner::Planner::trajectory_max_vel(&g.traj) <= max_vel + 1e-6,
        "全局轨迹最大速度 {} 超限 {max_vel}",
        crate::planner::Planner::trajectory_max_vel(&g.traj)
    );
    // 内点即任务航点：逐段时长 = 段长/des_vel，总时长 = 路径长
    // /des_vel（首轮达标则 des_vel = max_vel/1.5；超标再缩放一半）
    let dur = g.traj.duration();
    let path_len = path_length(m.global_path());
    assert!(
        (dur - path_len / 1.0).abs() < 1e-6 || (dur - path_len / (1.0 / 1.5)).abs() < 1e-6,
        "总时长应为路径长/des_vel(0.667 或 1.0):路径 {path_len},实际 {dur}"
    );
}

#[test]
fn glb_t_of_lc_tgt_advances_and_resets_on_retarget() {
    let mut m = open_manager();
    // 构造/重目标后按官方 setGlobalTraj 初始化：glb = 锚点、last = -1
    assert!(
        (m.global.glb_t_of_lc_tgt - m.global.start_time).abs() < 1e-9,
        "初始 glb = 锚点"
    );
    assert!(
        (m.global.last_glb_t_of_lc_tgt + 1.0).abs() < 1e-9,
        "初始 last = -1"
    );
    let path = m.global_path().to_vec();
    let start = path[0];
    // 首次 horizon：入口 last = 旧 glb（0），glb 前推
    let _ = m.horizon(start);
    assert!(
        (m.global.last_glb_t_of_lc_tgt).abs() < 1e-9,
        "入口 last = 旧 glb"
    );
    let glb1 = m.global.glb_t_of_lc_tgt;
    assert!(glb1 > 0.0, "glb 应前推");
    // 同 start 再选：命中同一目标 → last = 前次 glb、glb 不变
    let _ = m.horizon(start);
    assert!(
        (m.global.last_glb_t_of_lc_tgt - glb1).abs() < 1e-9,
        "last = 前次 glb"
    );
    assert!(
        (m.global.glb_t_of_lc_tgt - glb1).abs() < 1e-9,
        "同 start 命中同一目标，glb 不变"
    );
    // 起点前移：目标更晚 → glb 前推
    let mid = arc_point(&path, start, 3.0);
    let _ = m.horizon(mid);
    assert!(m.global.glb_t_of_lc_tgt > glb1, "起点前移后 glb 应前推");
    // 手动推进到全局终点：touch_goal → glb = 锚点 + duration
    m.global.glb_t_of_lc_tgt = m.global.start_time + m.global.traj.duration();
    let h = m.horizon(mid);
    assert!(h.touch_goal, "步进越过终点应 touch_goal");
    assert!(
        (m.global.glb_t_of_lc_tgt - (m.global.start_time + m.global.traj.duration())).abs() < 1e-9,
        "touch 后 glb = 锚点 + duration"
    );
    assert!(
        (h.target - m.goal().coords).norm() < 1e-9,
        "touch 目标 = 全局终点"
    );
    // 重设目标：官方 setGlobalTraj 等效（glb = now、last = -1）
    m.set_goal(10.0, None, Vector3::new(15.0, 1.0, 1.0))
        .unwrap();
    assert!(
        (m.global.glb_t_of_lc_tgt - 10.0).abs() < 1e-9,
        "重目标重置 glb = now"
    );
    assert!(
        (m.global.last_glb_t_of_lc_tgt + 1.0).abs() < 1e-9,
        "重目标重置 last = -1"
    );
}

#[test]
fn horizon_time_stepping_local_target() {
    // 官方 getLocalTarget：时间步进 + 距离判定——首个距 start ≥ 视界的
    // 全局轨迹点为局部目标（连同 glb 时刻的轨迹速度）。
    let mut m = open_manager();
    let start = m.global_path()[0];
    let h = m.horizon(start);
    assert!(!h.touch_goal, "9m 路径、6m 视界:不应触及终点");
    let glb = m.global.glb_t_of_lc_tgt;
    let on_traj = m.global.traj.eval(glb - m.global.start_time).position;
    assert!(
        (h.target - on_traj).norm() < 1e-9,
        "目标 = 全局轨迹在 glb 时刻的位置"
    );
    let d = (h.target - start).norm();
    assert!(
        d >= self::DEFAULT_PLANNING_HORIZON - 1e-6,
        "目标距起点 ≥ 视界:实际 {d:.3}"
    );
    assert!(
        d < 6.5,
        "目标应为首次越过视界的步进点(不应超远):实际 {d:.3}"
    );
    // 远离终点（> 刹车距离）→ 目标速度 = 全局轨迹在该时刻的速度（非零）
    let tv = m.global.traj.eval(glb - m.global.start_time).velocity;
    assert!(
        (h.target_vel - tv).norm() < 1e-9,
        "目标速度 = 全局轨迹时刻速度"
    );
    assert!(h.target_vel.norm() > 0.1, "远离终点目标速度非零");
}

#[test]
fn horizon_touch_goal_zeroes_velocity() {
    // 短路径(< 视界)：步进耗尽 → touch_goal、目标 = 全局终点、
    // 目标速度为零（触及时终点距离为 0 < 刹车距离）。
    let map = GridMapBuilder::new(0.5, [40, 24, 16]).build().unwrap();
    let planner = Planner::new(PlannerConfig::default(), map);
    let mut m = PlannerManager::with_planner(
        planner,
        ManagerOptions::default(),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(3.0, 1.0, 1.0),
    )
    .unwrap();
    let start = m.global_path()[0];
    let h = m.horizon(start);
    assert!(h.touch_goal, "路径短于视界应触及终点");
    assert!(
        (h.target - m.goal().coords).norm() < 1e-9,
        "touch 目标 = 全局终点"
    );
    assert!(
        (m.global.glb_t_of_lc_tgt - (m.global.start_time + m.global.traj.duration())).abs() < 1e-9,
        "touch 后 glb = 锚点 + duration"
    );
    assert!(h.target_vel.norm() < 1e-9, "触及终点目标速度应为零");
}

#[test]
fn horizon_zero_vel_within_braking_distance() {
    // 低限加速度拉大刹车距离 max_vel²/(2·max_acc)：非 touch_goal 目标
    // 落入刹车距离内时目标速度仍取零（官方 local_target_vel 判定）。
    let map = GridMapBuilder::new(0.5, [40, 24, 16]).build().unwrap();
    let planner = Planner::new(
        PlannerConfig {
            max_acceleration: 0.05,
            ..PlannerConfig::default()
        },
        map,
    );
    let mut m = PlannerManager::with_planner(
        planner,
        ManagerOptions::default(),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(10.0, 1.0, 1.0),
    )
    .unwrap();
    let start = m.global_path()[0];
    let h = m.horizon(start);
    let braking = 1.5 * 1.5 / (2.0 * 0.05); // 22.5m
    assert!(!h.touch_goal, "9m 路径、6m 视界:非 touch");
    assert!(
        (m.goal().coords - h.target).norm() < braking,
        "场景前提:目标在刹车距离内"
    );
    assert!(
        h.target_vel.norm() < 1e-9,
        "刹车距离内目标速度应为零,实际 {}",
        h.target_vel.norm()
    );
}

#[test]
fn tick_initial_plan_then_replan_at_threshold() {
    let mut m = open_manager();
    // 首帧:初始规划建立局部轨迹
    let r0 = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    assert!(m.local().is_some(), "首帧应建立局部轨迹");
    assert!(r0.replanned, "首条轨迹也作为新轨迹交给输出消费者");
    assert_eq!(m.replans(), 0, "初始规划不计入重规划次数");
    let ref0 = r0.reference.expect("首帧应有参考指令");
    assert!((ref0.position - Vector3::new(1.0, 1.0, 1.0)).norm() < 1e-6);
    assert!(!m.is_finished());
    // t=1.5 > replan_thresh(1.0) → 触发重规划(暖启动)
    let r1 = m.cycle(1.5, None);
    assert!(r1.replanned, "超过阈值应重规划");
    assert_eq!(m.replans(), 1);
    assert!(r1.reference.is_some());
}

/// 采用固定测试随机源，独立检查上游轴向、整数归一化与连败幅度公式。
#[test]
fn random_mid_matches_upstream_directions_and_failure_scaling() {
    use rand::{RngExt, SeedableRng, rngs::StdRng};
    let mut manager = open_manager();
    let (start, goal) = (Vector3::new(0., 0., 1.), Vector3::new(6., 0., 1.));
    for failures in [0, 1, 10] {
        manager.consecutive_failures = failures;
        let mut rng = StdRng::seed_from_u64(71);
        let mut expected_rng = StdRng::seed_from_u64(71);
        let h = f64::from(expected_rng.random_range(0..=i32::MAX)) / f64::from(i32::MAX);
        let v = f64::from(expected_rng.random_range(0..=i32::MAX)) / f64::from(i32::MAX);
        let scale = -0.978 / (f64::from(failures) + 0.989) + 0.989;
        let expected = Vector3::new(
            3.,
            (h - 0.5) * 6. * 0.8 * scale,
            1. - (v - 0.5) * 6. * 0.4 * scale,
        );
        let actual = manager.random_mid(start, goal, &mut rng);
        assert!((actual - expected).norm() < 1e-12);
        assert_eq!(rng.random::<u64>(), expected_rng.random::<u64>());
    }
    // Eigen 对零向量归一化保留零，同时仍消费两次随机数。
    for goal in [start, Vector3::new(0., 0., 7.)] {
        let mut rng = StdRng::seed_from_u64(71);
        let mut expected_rng = StdRng::seed_from_u64(71);
        let _ = expected_rng.random_range(0..=i32::MAX);
        let _ = expected_rng.random_range(0..=i32::MAX);
        assert!((manager.random_mid(start, goal, &mut rng) - (start + goal) / 2.).norm() < 1e-12);
        assert_eq!(rng.random::<u64>(), expected_rng.random::<u64>());
    }
}

/// 成功重规划清零连败（不变量，未失败时恒为 0）。
#[test]
fn successful_replan_keeps_failures_at_zero() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let _ = m.cycle(1.5, None);
    assert_eq!(m.consecutive_failures, 0);
}

#[test]
fn task_finishes_at_final_trajectory_end() {
    let mut m = PlannerManager::with_planner(
        Planner::new(
            PlannerConfig::default(),
            GridMapBuilder::new(0.5, [40, 24, 16]).build().unwrap(),
        ),
        ManagerOptions {
            planning_horizon: 20.0,
            ..ManagerOptions::default()
        },
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(10.0, 1.0, 1.0),
    )
    .unwrap();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let end = m.local().unwrap().start_time + m.local().unwrap().traj.duration();
    assert!(m.touch_goal);
    let early = m.fsm.exec_tick(0.1, Some(state_at(m.goal().coords)));
    assert!(!early.finished, "实测位置不能跳过 EXEC_TRAJ 的轨迹结束条件");
    let report = m.fsm.exec_tick(end, Some(state_at(m.goal().coords)));
    assert!(report.finished);
    assert!(m.is_finished());
    assert_eq!(m.fsm.state(), ExecState::WaitTarget);
}

#[test]
fn warm_start_falls_back_to_cold_when_exhausted() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    // 重规划到 horizon(非终点)的局部轨迹,使"轨迹耗尽"不会误触到达判定
    let _ = m.cycle(1.5, None);
    let dur = m.local().unwrap().traj.duration();
    assert!(m.local().unwrap().start_time < 1.5 + 1e-9);
    // 轨迹耗尽(now = start + duration + 0.1),未到达目标 → 强制重规划。
    // 暖启动的 elapsed 超过旧轨迹时长 → init_warm_start 必然失败
    // → 降级冷启动(官方 case2 → case1 策略链)仍须成功。
    let t_end = m.local().unwrap().start_time + dur + 0.1;
    let r = m.cycle(t_end, None);
    assert!(r.replanned, "轨迹耗尽应强制重规划");
    assert_eq!(m.replans(), 2, "降级链应产出一次重规划");
    assert!(m.local().is_some(), "降级后应仍有新轨迹");
    assert!(!m.is_finished());
}

#[test]
fn set_goal_rebuilds_path_and_dedups_same_target() {
    let mut m = open_manager();
    // 先正常规划出局部轨迹(飞行中状态)
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let _ = m.cycle(1.5, None);
    assert!(m.local().is_some(), "前置:应有局部轨迹");

    // 动态重目标:新目标重建全局路径、通知 FSM 重规划(目标取地图内部点)
    let new_goal = Vector3::new(15.0, 1.0, 1.0);
    m.set_goal(2.0, None, new_goal).unwrap();
    assert!((m.goal().coords - new_goal).norm() < 1e-9, "目标应更新");
    assert_eq!(m.global_path().last().unwrap(), &new_goal);
    assert!(m.local().is_some(), "重目标期间保持旧轨迹，等待新轨迹入库");
    assert!(!m.is_finished());
    // 重目标后下一 tick 应重新规划出新轨迹(初始规划不计 replans,
    // 语义见 tick_initial_plan_then_replan_at_threshold)
    let _ = m.cycle(2.1, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let local = m.local().expect("重目标后应重新规划出新轨迹");
    assert!(
        (local.start_time - 2.1).abs() < 1e-9,
        "新轨迹应在重目标后的 tick 生成(start_time={})",
        local.start_time
    );

    // 同目标幂等:CLI 连发防竞态,重复目标不得重置状态机/清空轨迹
    let replans_after_retarget = m.replans();
    let local_after_retarget = m.local().is_some();
    m.set_goal(2.2, None, new_goal).unwrap();
    assert_eq!(
        m.local().is_some(),
        local_after_retarget,
        "同目标重复 set_goal 不应丢弃当前轨迹"
    );
    assert_eq!(
        m.replans(),
        replans_after_retarget,
        "同目标重复 set_goal 不应触发重规划计数"
    );
}

#[test]
fn collision_monitor_triggers_immediate_replan() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    assert!(m.local().is_some(), "前置:初始轨迹存在");
    let local = m.local().unwrap();
    let obstacle = local.traj.eval(local.traj.duration() * 0.45).position;
    let voxel = m.map().index_of(obstacle).unwrap();
    m.map_mut().set_state(voxel, VoxelState::Occupied);
    assert!(
        !m.map().is_occupied_inflated(obstacle),
        "膨胀层刷新前新障碍不可见"
    );
    // t=0.5 < replan_thresh(1.0):只有碰撞监控能触发重规划
    let r = m.cycle(0.5, None);
    assert!(r.collision, "监控应发现前方碰撞");
    assert!(r.replanned, "发现碰撞应绕过 周期重规划阈值立即重规划");
    assert_eq!(m.replans(), 1);
    assert!(!r.emergency_stop, "重规划成功不应请求急停");
    let local = m.local().expect("碰撞重规划后应有新轨迹");
    assert!(
        (local.start_time - 0.5).abs() < 1e-9,
        "新轨迹从本 tick 起飞"
    );
    assert!(
        !local.points_to_check.is_empty(),
        "入库轨迹必带检查点(监控覆盖不变量)"
    );
}

#[test]
fn collision_monitor_ignores_already_flown_part() {
    // 阈值拉高隔离阈值重规划,聚焦监控行为
    let map = GridMapBuilder::new(0.5, [40, 24, 16]).build().unwrap();
    let planner = Planner::new(PlannerConfig::default(), map);
    let options = ManagerOptions {
        replan_thresh: 100.0,
        ..ManagerOptions::default()
    };
    let mut m = PlannerManager::with_planner(
        planner,
        options,
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(10.0, 1.0, 1.0),
    )
    .unwrap();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));

    // 已飞过位置(t=0.5 时所在)注入障碍;t=2.0 时机体已远离
    let past = m.local().unwrap().traj.eval(0.5).position;
    let now_pos = m.local().unwrap().traj.eval(2.0).position;
    assert!(
        past.x < now_pos.x - 0.5,
        "场景前提:t=2.0 应已飞过 t=0.5 位置"
    );
    let idx = m.map().index_of(past).expect("身后点在地图内");
    m.map_mut().set_state(idx, VoxelState::Occupied);
    m.map_mut().inflate_obstacles();
    assert!(
        m.map().is_occupied_inflated(past),
        "场景前提:身后障碍真实存在(防空洞断言)"
    );

    let r = m.cycle(2.0, None);
    assert!(!r.collision, "已飞过部分的障碍不得误报");
    assert!(!r.replanned, "无误报则不应重规划");
    assert_eq!(m.replans(), 0);
}

/// 沿 +y 方向直飞的管理器（起点 (1,1,1)，终点 (1,10,1)）：航向恒为 π/2。
fn northbound_manager() -> PlannerManager {
    let map = GridMapBuilder::new(0.5, [24, 40, 16]).build().unwrap();
    let planner = Planner::new(PlannerConfig::default(), map);
    PlannerManager::with_planner(
        planner,
        ManagerOptions::default(),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(1.0, 10.0, 1.0),
    )
    .unwrap()
}

// yaw_hold（本项目扩展）：锁定起飞朝向横飞，不跟速度方向，角速度恒零。
#[test]
fn yaw_hold_keeps_takeoff_heading() {
    let map = GridMapBuilder::new(0.5, [24, 40, 16]).build().unwrap();
    let cfg = PlannerConfig {
        yaw_hold: true,
        ..Default::default()
    };
    let planner = Planner::new(cfg, map);
    let mut m = PlannerManager::with_planner(
        planner,
        ManagerOptions::default(),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(1.0, 10.0, 1.0),
    )
    .unwrap();
    for t in [0.0, 0.5, 1.0, 2.0] {
        let r = m.cycle(t, Some(state_at(Vector3::new(1.0, 1.0 + t, 1.0))));
        let reference = r.reference.expect("应有参考指令");
        assert!(
            reference.yaw.abs() < 1e-12,
            "hold 下偏航应锁定 0，实际 {}",
            reference.yaw
        );
        assert!(reference.yaw_dot.abs() < 1e-12);
    }
}

#[test]
fn yaw_tracks_straight_line_heading() {
    let mut m = northbound_manager();
    // 首帧仅初始化 yaw 状态（官方首帧行为：last_yaw_=0 起）
    let r0 = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let ref0 = r0.reference.expect("首帧应有参考指令");
    assert!(ref0.yaw.abs() < 1e-12);
    assert!(ref0.yaw_dot.abs() < 1e-12);
    // 第二帧（低于重规划阈值，不扰动轨迹）：前视方向沿 +y → yaw = π/2
    let r1 = m.cycle(0.5, None);
    let ref1 = r1.reference.expect("第二帧应有参考指令");
    assert!(
        (ref1.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-6,
        "直线轨迹 yaw 应为航向角 π/2，实际 {}",
        ref1.yaw
    );
    // 首帧 yaw 从初值 0 出发，本帧一步内完成对准（dt=0.5 的限幅剖面
    // 容许 π/2），角速度为实际变化率且不超限幅
    assert!(
        ref1.yaw_dot.abs() <= YAW_DOT_MAX + 1e-9,
        "角速度超限幅: {}",
        ref1.yaw_dot
    );
    // 第三帧已对准航向：yaw 保持 π/2，角速度归零
    let r2 = m.cycle(1.0, None);
    let ref2 = r2.reference.expect("第三帧应有参考指令");
    assert!((ref2.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
    assert!(ref2.yaw_dot.abs() < 1e-9, "已对准航向后角速度应≈0");
    // 包装不变量：yaw 始终在 [-π,π]
    assert!(ref1.yaw.abs() <= std::f64::consts::PI + 1e-12);
}

#[test]
fn yaw_wraps_across_pi_without_jumping() {
    let mut m = open_manager();
    // 上一帧 yaw=3.0，期望航向 -3.0：原始差 -6.0，包装后应沿 +0.283 方向
    // 小步推进（穿过 ±π 边界），而不是反向跳转到 -3.0
    m.server.last_yaw = 3.0;
    m.server.last_yaw_dot = 0.0;
    m.server.last_yaw_time = Some(0.0);
    let pos = Vector3::zeros();
    let target = Vector3::new((-3.0f64).cos() * 5.0, (-3.0f64).sin() * 5.0, 0.0);
    let (yaw, yaw_dot) = m.update_yaw(pos, target, 0.1);
    // dt=0.1、零初速：梯形剖面首步位移 = 0.5·(5π)·dt² = 0.0785398...
    let expect_step = 0.5 * YAW_DOT_DOT_MAX * 0.01;
    assert!(
        (yaw - (3.0 + expect_step)).abs() < 1e-9,
        "yaw 应从 3.0 沿正向小步推进到 {}，实际 {yaw}",
        3.0 + expect_step
    );
    assert!((yaw_dot - expect_step / 0.1).abs() < 1e-9);
    assert!((m.server.last_yaw - yaw).abs() < 1e-12, "状态应同步更新");
}

#[test]
fn yaw_rate_capped_under_large_turn() {
    let mut m = open_manager();
    // 大转向：期望航向与当前 yaw 相差 π，逐步积分整条限幅剖面
    m.server.last_yaw = 0.0;
    m.server.last_yaw_dot = 0.0;
    m.server.last_yaw_time = Some(0.0);
    let pos = Vector3::zeros();
    let target = Vector3::new(-5.0, 0.0, 0.0); // 航向 π（与 0 反向）
    let mut now = 0.0;
    let mut converged = false;
    for _ in 0..400 {
        now += 0.02;
        let (yaw, yaw_dot) = m.update_yaw(pos, target, now);
        assert!(
            yaw_dot.abs() <= YAW_DOT_MAX + 1e-9,
            "|yaw_dot|={} 超出限幅 {}",
            yaw_dot.abs(),
            YAW_DOT_MAX
        );
        assert!(yaw.abs() <= std::f64::consts::PI + 1e-12, "包装越界: {yaw}");
        // 到达期望航向（±π 同义）即收敛
        if (yaw.abs() - std::f64::consts::PI).abs() < 1e-3 {
            converged = true;
            break;
        }
    }
    assert!(converged, "限幅剖面应在有限时间内转过 π");
}

#[test]
fn yaw_holds_last_on_deadzone() {
    let mut m = open_manager();
    m.server.last_yaw = 0.7;
    m.server.last_yaw_dot = -0.3;
    m.server.last_yaw_time = Some(1.0);
    // 前视方向模长 ≤ 0.1（悬停/末端）：保持 last_yaw，角速度归零
    let pos = Vector3::new(2.0, 2.0, 1.0);
    let target = pos + Vector3::new(0.05, 0.0, 0.0);
    let (yaw, yaw_dot) = m.update_yaw(pos, target, 1.1);
    assert!((yaw - 0.7).abs() < 1e-12, "死区应保持 last_yaw，实际 {yaw}");
    assert!(yaw_dot.abs() < 1e-12);
    let state = m.yaw_state();
    assert!((state.0 - 0.7).abs() < 1e-12 && state.1.abs() < 1e-12);
    // 首帧初始化：不计算方向，保持初值 (0, 0)
    let mut m2 = open_manager();
    let (yaw2, dot2) = m2.update_yaw(pos, target, 5.0);
    assert!(yaw2.abs() < 1e-12 && dot2.abs() < 1e-12);
}

#[test]
fn ground_height_measures_flat_floor() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    // 平地：整个底层体素层（z∈[0,0.5)）填满占据
    let dims = m.map().dims();
    for i in 0..dims[0] {
        for j in 0..dims[1] {
            m.map_mut().set_state([i, j, 0], VoxelState::Occupied);
        }
    }
    let h = m
        .measure_ground_height(m.local().unwrap().start_time)
        .expect("平地应测得地面高度");
    assert!(
        (0.0..0.5).contains(&h),
        "测得高度应落在地面体素层内，实际 {h}",
    );
    // 首个命中不变量：命中体素占据，其上一层自由
    let fwd = m
        .local()
        .unwrap()
        .traj
        .eval(GROUND_LOOKAHEAD_DIST / 1.5)
        .position;
    assert!(m.map().is_occupied(Vector3::new(fwd.x, fwd.y, h)));
    assert!(!m.map().is_occupied(Vector3::new(fwd.x, fwd.y, h + 0.5)));
}

#[test]
fn ground_height_fails_at_map_bottom() {
    let mut m = open_manager();
    // 空旷地图（无地面）：逐格下降扫描至图底仍无占据 → 测量失败
    let r = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    assert!(m.local().is_some(), "前置：轨迹已存在");
    assert!(r.ground_height.is_none(), "图底之下无地面应返回失败");
    assert!(m.measure_ground_height(0.0).is_none());
}

#[test]
fn ground_height_skipped_without_plan() {
    let m = open_manager();
    // 无可执行轨迹（官方 pts_chk < 3 的前置条件）不测量
    assert!(m.measure_ground_height(0.0).is_none());
}

#[test]
fn ground_height_ignores_virtual_floor_below_ground() {
    // 虚拟地板低于真实地面：扫描命中真实占据层，仍报地面高度
    let mut m = open_manager_with_wall(Some((-0.1, f64::INFINITY)));
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let dims = m.map().dims();
    for i in 0..dims[0] {
        for j in 0..dims[1] {
            m.map_mut().set_state([i, j, 0], VoxelState::Occupied);
        }
    }
    let h = m
        .measure_ground_height(m.local().unwrap().start_time)
        .expect("虚拟地板低于真实地面时应测得地面高度");
    assert!((0.0..0.5).contains(&h), "实际 {h}");
}

#[test]
fn ground_height_fails_at_virtual_floor() {
    // 虚拟地板设在扫描起点与真实地面之间：先于占据命中触墙 → 测量失败
    let mut m = open_manager_with_wall(Some((0.7, f64::INFINITY)));
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let dims = m.map().dims();
    for i in 0..dims[0] {
        for j in 0..dims[1] {
            m.map_mut().set_state([i, j, 0], VoxelState::Occupied);
        }
    }
    assert!(m.local().is_some(), "前置：轨迹已存在");
    assert!(m.measure_ground_height(0.0).is_none());
}

#[test]
fn emergency_stop_generates_stationary_reference() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let stop_pos = Vector3::new(2.0, 1.0, 1.0);
    m.enter_emergency_stop(0.5, Some(state_at(stop_pos)));
    assert!(m.fsm.is_emergency(), "进入后应处于急停态");
    // 停车轨迹：两段各 STOP_PIECE_DURATION，全程定点零速零加速度
    // （官方 EmergencyStop 的边界状态全同约束 ⇒ 恒为常值多项式）
    let local = m.local().expect("急停应入库停车轨迹");
    assert!((local.traj.duration() - 2.0 * STOP_PIECE_DURATION).abs() < 1e-9);
    for t in [0.0, 0.7, 1.3, 2.0] {
        let s = local.traj.eval(t);
        assert!((s.position - stop_pos).norm() < 1e-9, "t={t} 应停在原点");
        assert!(s.velocity.norm() < 1e-9, "t={t} 参考速度应为零");
        assert!(s.acceleration.norm() < 1e-9, "t={t} 参考加速度应为零");
    }
    // 急停中的 tick：报告急停态，参考为原地悬停
    let r = m.cycle(1.0, None);
    assert!(r.emergency_stop, "急停中报告应急停标志");
    let reference = r.reference.expect("急停态应有悬停参考");
    assert!((reference.position - stop_pos).norm() < 1e-6);
    assert!(reference.velocity.norm() < 1e-9);
}

#[test]
fn emergency_stop_survives_trajectory_exhaustion() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let stop_pos = Vector3::new(2.0, 1.0, 1.0);
    m.enter_emergency_stop(0.5, Some(state_at(stop_pos)));
    let replans_before = m.replans();
    // 远超停车轨迹时长（2s）：不得触发常规强制重规划或脱困直飞
    // （对照官方：重规划仅在 EXEC_TRAJ case 生效）
    let r = m.cycle(10.0, None);
    assert!(m.fsm.is_emergency(), "轨迹耗尽不得退出急停");
    assert!(!r.replanned, "急停中轨迹耗尽不得常规重规划");
    assert_eq!(m.replans(), replans_before);
    let reference = r.reference.expect("耗尽后仍应有悬停参考");
    assert!((reference.position - stop_pos).norm() < 1e-6);
    assert!(reference.velocity.norm() < 1e-9);
}

#[test]
fn emergency_recovers_when_failsafe_enabled_and_slow() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let stop_pos = Vector3::new(2.0, 1.0, 1.0);
    m.enter_emergency_stop(0.5, Some(state_at(stop_pos)));
    // 超阈值速度：即使 fail-safe 允许也不恢复（官方 vel < 0.1 判据）
    let mut fast = state_at(stop_pos);
    fast.velocity = Vector3::new(1.0, 0.0, 0.0);
    let r_fast = m.cycle(0.8, Some(fast));
    assert!(m.fsm.is_emergency(), "超阈值速度不应恢复");
    assert!(!r_fast.replanned);
    // 低速观测 + fail-safe 允许（默认）→ 从当前位姿完整重规划并退出
    let r = m.cycle(1.0, Some(state_at(stop_pos)));
    assert!(!m.fsm.is_emergency(), "低速且 fail-safe 允许应退出急停");
    assert!(r.replanned, "恢复应产出新轨迹");
    assert!(!r.emergency_stop);
    let local = m.local().expect("恢复后应有新轨迹");
    assert!(
        (local.start_time - 1.0).abs() < 1e-9,
        "新轨迹应在恢复 tick 入库"
    );
}

#[test]
fn in_collision_goal_modified_to_nearest_free_point() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let old_path = m.global_path().to_vec();
    let old_goal = m.goal();
    // 目标附近整块占据（res 0.5，goal=(10,1,1) → 体素 [20,2,2] 及邻域），
    // 使回扫从末端起至少跨过一个步长才碰到自由点
    for i in 18..=21 {
        for j in 1..=3 {
            for k in 1..=3 {
                m.map_mut().set_state([i, j, k], VoxelState::Occupied);
            }
        }
    }
    m.map_mut().inflate_obstacles();
    assert!(
        m.map().is_occupied_inflated(old_goal.coords),
        "场景前提：目标应落在膨胀占据内"
    );
    // t=0.5 < replan_thresh：本 tick 只有目标修正分支能动作
    let r = m.cycle(0.5, None);
    assert!(r.replanned, "目标修正应本 tick 立即重规划");
    let new_goal = m.goal();
    assert!(
        (new_goal.coords - old_goal.coords).norm() > 1e-6,
        "目标应被修正，实际仍为 {new_goal:?}"
    );
    assert!(
        !m.map().is_occupied_inflated(new_goal.coords),
        "修正后的目标必须无碰"
    );
    // 修正点 = 沿旧全局路径末端回扫的首个自由点：用测试预言机独立复算期望值
    let reso = m.map().resolution();
    let mut expected = None;
    let mut s = path_length(&old_path);
    while s > 0.0 {
        let pt = arc_point(&old_path, old_path[0], s);
        if !m.map().is_occupied_inflated(pt) {
            expected = Some(pt);
            break;
        }
        s -= reso;
    }
    let expected = expected.expect("场景前提：回扫必能找到自由点");
    assert!(
        (new_goal.coords - expected).norm() < 1e-6,
        "修正点应为末端回扫首个自由点：期望 {expected:?}，实际 {new_goal:?}"
    );
    assert!(!r.finished);
    assert!(!r.emergency_stop);
}

#[test]
fn free_goal_left_untouched() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let goal = m.goal();
    let path = m.global_path().to_vec();
    // 目标自由且低于重规划阈值：目标与轨迹均不变
    let r = m.cycle(0.5, None);
    assert!(!r.replanned);
    assert_eq!(m.goal(), goal);
    assert_eq!(m.global_path(), path.as_slice());
    assert_eq!(m.replans(), 0);
}

#[test]
fn fully_blocked_global_path_keeps_goal_without_panic() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let goal = m.goal();
    let path = m.global_path().to_vec();
    // 整条直线路径占据：路径 y=z=1，占据 y=z∈[1.0,1.5) 的体素条带即覆盖全程
    let dims = m.map().dims();
    for i in 0..dims[0] {
        m.map_mut().set_state([i, 2, 2], VoxelState::Occupied);
    }
    m.map_mut().inflate_obstacles();
    for p in &path {
        assert!(m.map().is_occupied_inflated(*p), "场景前提：{p:?} 应被占据");
    }
    // 回扫耗尽：不 panic，目标与轨迹保持不变（后续碰撞监控/急停接管）
    let r = m.cycle(0.5, None);
    assert_eq!(m.goal(), goal, "回扫耗尽目标不得变动");
    assert_eq!(m.global_path(), path.as_slice());
    assert!(!r.finished);
}

#[test]
fn emergency_without_failsafe_never_recovers() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let stop_pos = Vector3::new(2.0, 1.0, 1.0);
    m.trigger_emergency_stop_disable_failsafe(0.5, Some(state_at(stop_pos)));
    assert!(m.fsm.is_emergency(), "深度丢失入口应进入急停");
    // fail-safe 已关闭：即使完全静止也不自动恢复（官方语义）
    let r = m.cycle(1.0, Some(state_at(stop_pos)));
    assert!(m.fsm.is_emergency(), "fail-safe 关闭不得自动恢复");
    assert!(r.emergency_stop);
    assert!(!r.replanned);
}

/// 定点悬停轨迹（单段）：swarm 对端注入用。
fn hover_traj(pos: Vector3<f64>, duration: f64) -> Trajectory {
    let endpoint = Endpoint {
        position: pos,
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    MincoBuilder::new(SolverOrder::MinimumJerk, endpoint, endpoint)
        .build(&[], &[duration])
        .unwrap()
        .solve()
        .unwrap()
}

/// 指定 `drone_id` 的空旷管理器（其余同 `open_manager`）。
fn manager_with_drone_id(drone_id: i32) -> PlannerManager {
    let map = GridMapBuilder::new(0.5, [40, 24, 16]).build().unwrap();
    let planner = Planner::new(
        PlannerConfig {
            drone_id,
            ..PlannerConfig::default()
        },
        map,
    );
    PlannerManager::with_planner(
        planner,
        ManagerOptions::default(),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(10.0, 1.0, 1.0),
    )
    .unwrap()
}

#[test]
fn sequential_start_waits_for_predecessors() {
    let mut m = manager_with_drone_id(2);
    // 未收齐前序：不规划、无参考输出（官方 SEQUENTIAL_START 停留语义）
    let r = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    assert!(!m.sequential_start_ready());
    assert!(m.local().is_none(), "未收齐前序 agent 轨迹不得初始规划");
    assert!(r.reference.is_none());
    // 只收 agent 0（缺 agent 1）：仍等待
    m.register_swarm_traj(0, hover_traj(Vector3::new(15.0, 10.0, 1.0), 30.0), 0.0, 0.5);
    let _ = m.cycle(0.1, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    assert!(!m.sequential_start_ready());
    assert!(m.local().is_none());
    // 收齐 agent 1：本 tick 即初始规划
    m.register_swarm_traj(1, hover_traj(Vector3::new(18.0, 10.0, 1.0), 30.0), 0.0, 0.5);
    let _ = m.cycle(0.2, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    assert!(m.sequential_start_ready());
    assert!(m.local().is_some(), "收齐前序后应立即规划");
}

#[test]
fn single_drone_needs_no_predecessors() {
    let mut m = manager_with_drone_id(0);
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    assert!(m.sequential_start_ready());
    assert!(m.local().is_some(), "drone_id=0 无需等待前序即可规划");
}

#[test]
fn mandatory_stop_disables_failsafe_recovery() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    let stop_pos = Vector3::new(2.0, 1.0, 1.0);
    m.mandatory_stop(0.5, Some(state_at(stop_pos)));
    assert!(m.is_mandatory_stopped(), "强制停止标志应锁存");
    assert!(m.fsm.is_emergency(), "强制停止应进入急停");
    // fail-safe 已关闭：完全静止也不自动恢复（官方 enable_fail_safe_=false）
    let r = m.cycle(1.0, Some(state_at(stop_pos)));
    assert!(m.fsm.is_emergency(), "强制停止后不得自动恢复");
    assert!(r.emergency_stop);
    assert!(!r.replanned);
}

#[test]
fn mandatory_stop_latch_rejects_retarget() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    m.mandatory_stop(0.5, Some(state_at(Vector3::new(2.0, 1.0, 1.0))));
    // 锁存中收到新目标：目标与急停态均不变（急停后不自动恢复）
    let goal_before = m.goal();
    m.set_goal(0.6, None, Vector3::new(15.0, 10.0, 1.0))
        .unwrap();
    assert_eq!(m.goal(), goal_before, "锁存中不得换目标");
    assert!(m.fsm.is_emergency(), "新目标不得解除急停");
}

#[test]
fn swarm_conflict_triggers_immediate_replan() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    assert!(m.local().is_some());
    let local = m.local().unwrap();
    // 非零侧偏打破静态对称，避免在避碰代价驻点上测试求解器收敛。
    let position =
        local.traj.eval(local.traj.duration() * 0.45).position + Vector3::new(0., 0.2, 0.);
    m.register_swarm_traj(1, hover_traj(position, 30.), 0., 0.);
    assert!(
        m.next_swarm_conflict(0.5).is_some(),
        "测试对端必须位于监控覆盖范围内"
    );
    // t=0.5 < replan_thresh(1.0)：仅 swarm 预测检查能触发重规划
    let r = m.cycle(0.5, None);
    assert!(r.replanned, "预测距离过近应立即重规划");
    assert_eq!(m.replans(), 1);
    assert!(r.collision, "安全回调统一处理障碍与对端轨迹危险");
}

#[test]
fn far_swarm_peer_does_not_disturb() {
    let mut m = open_manager();
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    // 对端横向偏离航迹 4m：远超净距，无冲突无重规划
    m.register_swarm_traj(1, hover_traj(Vector3::new(4.0, 5.0, 1.0), 30.0), 0.0, 0.0);
    let r = m.cycle(0.5, None);
    assert!(!r.replanned);
    assert_eq!(m.replans(), 0);
    assert!(!r.collision);
}

#[test]
fn own_swarm_slot_is_rejected() {
    let mut m = open_manager(); // drone_id = 0
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    // 自身槽位拒收（官方 recv_id == 自身直接返回）：不得触发冲突重规划
    m.register_swarm_traj(0, hover_traj(Vector3::new(4.0, 1.0, 1.0), 30.0), 0.0, 0.0);
    let r = m.cycle(0.5, None);
    assert!(!r.replanned, "自身槽位不得参与 swarm 检查");
    assert_eq!(m.replans(), 0);
}

#[test]
fn traj_id_increments_on_each_adopted_plan() {
    let mut m = open_manager();
    assert_eq!(
        m.traj_id(),
        0,
        "构造后无轨迹：0 = 无轨迹哨兵（官方 traj_id <= 0）"
    );
    // 初始规划（官方 GEN_NEW_TRAJ 首次 setLocalTraj）→ 序号 1
    let _ = m.cycle(0.0, Some(state_at(Vector3::new(1.0, 1.0, 1.0))));
    assert_eq!(m.traj_id(), 1, "初始规划应拿到 1 号");
    assert_eq!(m.local().unwrap().traj_id, 1, "入库轨迹应携带当前序号");
    // 超阈值重规划 → 序号 2（每条新轨迹递增，供对端区分新旧）
    let _ = m.cycle(1.5, None);
    assert_eq!(m.traj_id(), 2, "首次重规划应拿到 2 号");
    assert_eq!(m.local().unwrap().traj_id, 2);
    // 再重规划 → 序号 3（新型轨迹一定会被对端视为更新）
    let _ = m.cycle(3.0, None);
    assert_eq!(m.traj_id(), 3, "每次成功入库持续递增");
    assert_eq!(m.local().unwrap().traj_id, 3);
}

#[test]
fn monitors_short_circuit_without_issued_traj() {
    // 构造后未规划（traj_id == 0）：官方 ego_replan_fsm.cpp:307 入口短路——
    // 即便地图有障碍、对端有轨迹，碰撞/swarm 监控都不扫描
    let mut m = open_manager();
    assert_eq!(m.traj_id(), 0);
    m.map_mut().set_state([8, 2, 2], VoxelState::Occupied);
    m.map_mut().inflate_obstacles();
    assert!(m.next_collision(0.5).is_none(), "无轨迹时碰撞监控必须短路");
    m.register_swarm_traj(1, hover_traj(Vector3::new(4.0, 1.0, 1.0), 30.0), 0.0, 0.0);
    assert!(
        m.next_swarm_conflict(0.5).is_none(),
        "无轨迹时 swarm 检查必须短路"
    );
}

#[test]
fn broadcast_des_clearance_returns_config_swarm_clearance() {
    // 广播期望净距 = 本机 optimization/swarm_clearance 参数（官方发包时
    // MINCO_msg.des_clearance = getSwarmClearance()，ego_replan_fsm.cpp:820）；
    // 不依赖是否已规划
    let map = GridMapBuilder::new(0.5, [40, 24, 16]).build().unwrap();
    let planner = Planner::new(
        PlannerConfig {
            swarm_clearance: 1.2,
            ..PlannerConfig::default()
        },
        map,
    );
    let m = PlannerManager::with_planner(
        planner,
        ManagerOptions::default(),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(10.0, 1.0, 1.0),
    )
    .unwrap();
    assert!(
        (m.broadcast_des_clearance() - 1.2).abs() < 1e-9,
        "广播净距应等于配置值"
    );
    // 默认配置回落默认值
    let default = PlannerConfig::default().swarm_clearance;
    assert!(
        (open_manager().broadcast_des_clearance() - default).abs() < 1e-9,
        "默认配置下广播净距应等于默认 swarm_clearance"
    );
}
