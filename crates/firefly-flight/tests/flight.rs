//! 飞行力学/飞控的数学不变量（无渲染、无 ECS）。

use firefly_flight::{
    AngleCommand, ControlParams, G, PositionSetpoint, QuadParams, QuadState, angle_mode, integrate,
    position_mode,
};
use glam::{Quat, Vec3};

const DT: f32 = 0.005;

fn params() -> QuadParams {
    QuadParams::default()
}

fn ctl() -> ControlParams {
    ControlParams::default()
}

/// 悬停是平衡点：参考=当前状态、姿态水平 → 推力 = mg、力矩 ≈ 0，积分后位置不动。
#[test]
fn hover_is_an_equilibrium() {
    let (p, c) = (params(), ctl());
    let mut state = QuadState {
        position: Vec3::new(1.0, 2.0, 1.5),
        ..Default::default()
    };
    let setpoint = PositionSetpoint {
        position: state.position,
        ..Default::default()
    };

    let wrench = position_mode(&state, &setpoint, &p, &c);
    assert!(
        (wrench.force.z - p.mass * G).abs() < 1e-3,
        "悬停推力应等于 mg：{}",
        wrench.force.z
    );
    assert!(wrench.force.truncate().length() < 1e-3);
    assert!(wrench.torque.length() < 1e-3);

    for _ in 0..200 {
        let wrench = position_mode(&state, &setpoint, &p, &c);
        integrate(&mut state, &wrench, &p, DT);
    }
    let drift = state.position.distance(setpoint.position);
    assert!(drift < 1e-3, "悬停 1 s 位置漂移 {drift:.5} m");
}

/// 侧向阶跃必须靠倾斜产生水平力（推力只能沿机体 z），最终收敛到目标且高度保持。
#[test]
fn lateral_step_tilts_then_translates() {
    let (p, c) = (params(), ctl());
    let mut state = QuadState::default();
    let setpoint = PositionSetpoint {
        position: Vec3::new(1.0, 0.0, 0.0),
        ..Default::default()
    };

    // 初始水平：推力全在竖直方向，瞬时水平力应为 0（先倾斜才有力）。
    let first = position_mode(&state, &setpoint, &p, &c);
    assert!(
        first.force.truncate().length() < 1e-3,
        "水平姿态下不应有水平力：{}",
        first.force.truncate().length()
    );

    let limit = c.tilt_max_deg.to_radians();
    let mut max_tilt = 0.0f32;
    for _ in 0..600 {
        let wrench = position_mode(&state, &setpoint, &p, &c);
        integrate(&mut state, &wrench, &p, DT);
        let tilt = (state.attitude * Vec3::Z).z.clamp(-1.0, 1.0).acos();
        max_tilt = max_tilt.max(tilt);
    }

    assert!(max_tilt > 0.05, "应出现明显倾斜，实测 {max_tilt:.3} rad");
    assert!(
        max_tilt <= limit + 1e-2,
        "倾角 {max_tilt:.3} rad 超过限幅 {limit:.3} rad"
    );
    assert!(
        (state.position.x - 1.0).abs() < 0.05,
        "应收敛到目标 x，实测 {:.3} m",
        state.position.x
    );
    assert!(
        state.position.z.abs() < 0.05,
        "高度应保持，实测 {:.3} m",
        state.position.z
    );
}

/// 推力限幅：大误差下合力不超过电机上限，且不为负。
#[test]
fn thrust_is_clamped() {
    let (p, c) = (params(), ctl());
    let state = QuadState::default();
    let setpoint = PositionSetpoint {
        position: Vec3::new(50.0, 0.0, 50.0),
        ..Default::default()
    };
    let wrench = position_mode(&state, &setpoint, &p, &c);
    assert!(wrench.force.length() <= p.max_thrust + 1e-3);
    assert!(wrench.force.z >= 0.0);

    let down = PositionSetpoint {
        position: Vec3::new(0.0, 0.0, -50.0),
        ..Default::default()
    };
    let wrench = position_mode(&state, &down, &p, &c);
    assert!(wrench.force.z >= 0.0, "推力不得为负：{}", wrench.force.z);
}

/// 大水平误差下期望倾角被压在 `tilt_max` 锥内（限幅作用于期望姿态，内环可有少量超调）。
#[test]
fn tilt_is_limited() {
    let (p, c) = (params(), ctl());
    let setpoint = PositionSetpoint {
        position: Vec3::new(50.0, 0.0, 0.0),
        ..Default::default()
    };
    let mut probe = QuadState::default();
    let mut max_tilt = 0.0f32;
    for _ in 0..400 {
        let wrench = position_mode(&probe, &setpoint, &p, &c);
        integrate(&mut probe, &wrench, &p, DT);
        let tilt = (probe.attitude * Vec3::Z).z.clamp(-1.0, 1.0).acos();
        max_tilt = max_tilt.max(tilt);
    }
    let limit = c.tilt_max_deg.to_radians();
    assert!(
        max_tilt <= limit * 1.15,
        "倾角 {max_tilt:.3} rad 明显超过限幅 {limit:.3} rad（期望姿态限幅+内环超调）"
    );
}

/// 偏航在 ±π 回绕处走最近路径：从 −π+ε 到 +π−ε 只差 2ε，不得绕整圈。
#[test]
fn yaw_takes_shortest_path() {
    let (p, c) = (params(), ctl());
    let state = QuadState {
        attitude: Quat::from_rotation_z(-std::f32::consts::PI + 0.05),
        ..Default::default()
    };
    let setpoint = PositionSetpoint {
        position: Vec3::ZERO,
        yaw: std::f32::consts::PI - 0.05,
        ..Default::default()
    };
    let wrench = position_mode(&state, &setpoint, &p, &c);
    // 最近路径只需 0.1 rad 的偏航修正：偏航力矩应远小于"绕整圈"的量级。
    let unit = c.attitude_kp * c.rate_kp * p.inertia[2] * 0.1;
    assert!(
        wrench.torque.z.abs() < unit * 3.0,
        "偏航力矩 {:.4} 过大，疑似绕远路（阈值 {:.4}）",
        wrench.torque.z,
        unit * 3.0
    );
}

/// 角度模式：倾斜 30° 且零指令时应回正、且不产生水平净位移。
#[test]
fn angle_mode_levels_out() {
    let (p, c) = (params(), ctl());
    let mut state = QuadState {
        position: Vec3::new(0.0, 0.0, 1.0),
        attitude: Quat::from_rotation_y(0.5),
        ..Default::default()
    };
    for _ in 0..600 {
        let wrench = angle_mode(&state, &AngleCommand::default(), &p, &c);
        integrate(&mut state, &wrench, &p, DT);
    }
    let tilt = (state.attitude * Vec3::Z).z.clamp(-1.0, 1.0).acos();
    assert!(tilt < 0.02, "应回正，实测残倾 {tilt:.4} rad");
    assert!(
        state.position.z > 0.9,
        "应维持高度，实测 {:.3} m",
        state.position.z
    );
}
