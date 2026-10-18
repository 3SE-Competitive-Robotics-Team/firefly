//! PX4 控制链的闭环不变量（与 `apps/fc` 相同的接线：控制器 → 电机 → 刚体）。

use firefly_flight::{
    Airframe, Command, Controller, FlightFsm, FlightState, FsmParams, G, Health, PositionSetpoint,
    Px4Params, QuadParams, QuadState, Wrench, integrate,
};
use nalgebra::{UnitQuaternion, Vector3};

const DT: f32 = 0.001;

/// 去掉气动阻尼：与 `MuJoCo` 被控对象一致，只留 `PX4` 控制律本身。
fn plant() -> QuadParams {
    QuadParams {
        linear_drag: 0.0,
        angular_drag: 0.0,
        ..QuadParams::default()
    }
}

fn controller() -> Controller {
    Controller::new(Px4Params::default())
}

fn tick(
    state: &mut QuadState,
    setpoint: &PositionSetpoint,
    ctrl: &mut Controller,
    quad: &QuadParams,
    air: &Airframe,
    wind: Vector3<f32>,
) -> [f32; 4] {
    let motors = ctrl
        .update(state, setpoint, quad, air, DT, false)
        .expect("finite setpoint");
    let realized = air.realize(state.attitude, &motors);
    integrate(
        state,
        &Wrench {
            force: realized.force + wind,
            torque: realized.torque,
        },
        quad,
        DT,
    );
    motors
}

#[test]
fn hover_is_an_equilibrium() {
    let (quad, air) = (plant(), Airframe::default());
    let mut ctrl = controller();
    let mut state = QuadState {
        position: Vector3::new(1.0, 2.0, 1.5),
        ..Default::default()
    };
    let setpoint = PositionSetpoint {
        position: state.position,
        ..Default::default()
    };
    let mut motors = [0.0; 4];
    for _ in 0..2000 {
        motors = tick(
            &mut state,
            &setpoint,
            &mut ctrl,
            &quad,
            &air,
            Vector3::zeros(),
        );
    }
    assert!((state.position - setpoint.position).norm() < 2e-3);
    assert!(state.velocity.norm() < 2e-3);
    let total: f32 = motors.iter().sum();
    assert!((total - quad.mass * G).abs() < 0.02, "悬停总推力 {total}");
    for motor in motors {
        assert!((motor - total / 4.0).abs() < 0.02, "电机不均衡 {motor}");
    }
}

#[test]
fn lateral_step_tilts_then_translates() {
    let (quad, air) = (plant(), Airframe::default());
    let mut ctrl = controller();
    let mut state = QuadState {
        position: Vector3::new(0.0, 0.0, 1.5),
        ..Default::default()
    };
    let setpoint = PositionSetpoint {
        position: Vector3::new(1.0, 0.0, 1.5),
        ..Default::default()
    };
    let mut max_tilt = 0.0_f32;
    for _ in 0..6000 {
        tick(
            &mut state,
            &setpoint,
            &mut ctrl,
            &quad,
            &air,
            Vector3::zeros(),
        );
        let tilt = (state.attitude * Vector3::z())
            .z
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        max_tilt = max_tilt.max(tilt);
    }
    assert!(max_tilt > 3.0, "应倾斜，实测 {max_tilt:.2}°");
    assert!(max_tilt <= 40.0, "倾角超限 {max_tilt:.2}°");
    assert!(
        (state.position.x - 1.0).abs() < 0.05,
        "x={:.3}",
        state.position.x
    );
    assert!(
        (state.position.z - 1.5).abs() < 0.1,
        "高度漂移 {:.3}",
        state.position.z - 1.5
    );
}

/// 大水平误差不得靠集合推力顶满把姿态权限挤掉（旧实现 15m 会锁死爬升）。
#[test]
fn large_lateral_error_tilts_instead_of_climbing() {
    let (quad, air) = (plant(), Airframe::default());
    let mut ctrl = controller();
    let mut state = QuadState {
        position: Vector3::new(0.0, 0.0, 1.5),
        ..Default::default()
    };
    let setpoint = PositionSetpoint {
        position: Vector3::new(20.0, 0.0, 1.5),
        ..Default::default()
    };
    let mut max_tilt = 0.0_f32;
    let mut max_height = 0.0_f32;
    for _ in 0..1000 {
        let motors = tick(
            &mut state,
            &setpoint,
            &mut ctrl,
            &quad,
            &air,
            Vector3::zeros(),
        );
        assert!(motors.iter().all(|m| m.is_finite() && *m >= 0.0));
        max_tilt = max_tilt.max(
            (state.attitude * Vector3::z())
                .z
                .clamp(-1.0, 1.0)
                .acos()
                .to_degrees(),
        );
        max_height = max_height.max((state.position.z - 1.5).abs());
    }
    assert!(max_tilt > 15.0, "应明显倾斜，实测 {max_tilt:.1}°");
    assert!(max_height < 0.5, "1s 内高度偏差 {max_height:.2} m");
    assert!(
        state.position.x > 0.5,
        "应朝目标移动，实测 {:.2} m",
        state.position.x
    );
}

/// 电机推力永不超过机体上限，且不为负。
#[test]
fn motors_stay_within_limits() {
    let (quad, air) = (plant(), Airframe::default());
    let mut ctrl = controller();
    let mut state = QuadState {
        position: Vector3::new(0.0, 0.0, 1.5),
        ..Default::default()
    };
    let setpoint = PositionSetpoint {
        position: Vector3::new(50.0, 0.0, 50.0),
        ..Default::default()
    };
    for _ in 0..2000 {
        let motors = tick(
            &mut state,
            &setpoint,
            &mut ctrl,
            &quad,
            &air,
            Vector3::zeros(),
        );
        for motor in motors {
            assert!(motor >= 0.0 && motor <= air.max_thrust_per_motor + 1e-4);
        }
    }
}

/// 常值侧风下速度积分项消除稳态位置偏差。
#[test]
fn wind_is_rejected_by_velocity_integrator() {
    let (quad, air) = (plant(), Airframe::default());
    let mut ctrl = controller();
    let mut state = QuadState {
        position: Vector3::new(0.0, 0.0, 1.5),
        ..Default::default()
    };
    let setpoint = PositionSetpoint {
        position: Vector3::new(0.0, 0.0, 1.5),
        ..Default::default()
    };
    let wind = Vector3::new(0.5, 0.0, 0.0);
    for _ in 0..20_000 {
        tick(&mut state, &setpoint, &mut ctrl, &quad, &air, wind);
    }
    assert!(
        state.position.x.abs() < 0.1,
        "0.5N 侧风稳态偏差 {:.3} m",
        state.position.x
    );
    assert!(
        (state.position.z - 1.5).abs() < 0.1,
        "高度偏差 {:.3}",
        state.position.z - 1.5
    );
}

/// 匀速参考 + 真实阻力：速度积分项把滞后压到可接受范围。
#[test]
fn velocity_tracking_with_drag_has_small_lag() {
    let quad = QuadParams::default();
    let air = Airframe::default();
    let mut ctrl = controller();
    let mut state = QuadState {
        position: Vector3::new(0.0, 0.0, 1.5),
        ..Default::default()
    };
    let mut lag = 0.0;
    for k in 0..20_000 {
        let t = k as f32 * DT;
        let setpoint = PositionSetpoint {
            position: Vector3::new(t, 0.0, 1.5),
            velocity: Vector3::new(1.0, 0.0, 0.0),
            ..Default::default()
        };
        tick(
            &mut state,
            &setpoint,
            &mut ctrl,
            &quad,
            &air,
            Vector3::zeros(),
        );
        lag = t - state.position.x;
    }
    assert!(lag.abs() < 0.1, "1m/s 稳态滞后 {lag:.3} m");
}

/// 产品系航向设定值：+90° 偏航后机体 +X 指向产品世界 +Y。
#[test]
fn yaw_setpoint_turns_to_requested_heading() {
    let (quad, air) = (plant(), Airframe::default());
    let mut ctrl = controller();
    let mut state = QuadState {
        position: Vector3::new(0.0, 0.0, 1.5),
        ..Default::default()
    };
    let setpoint = PositionSetpoint {
        position: Vector3::new(0.0, 0.0, 1.5),
        yaw: std::f32::consts::FRAC_PI_2,
        ..Default::default()
    };
    for _ in 0..6000 {
        tick(
            &mut state,
            &setpoint,
            &mut ctrl,
            &quad,
            &air,
            Vector3::zeros(),
        );
    }
    let yaw = firefly_flight::yaw_of(state.attitude);
    assert!(
        (yaw - std::f32::consts::FRAC_PI_2).abs() < 0.05,
        "航向 {yaw:.3} rad"
    );
    assert!(
        (state.position.z - 1.5).abs() < 0.1,
        "高度漂移 {:.3}",
        state.position.z - 1.5
    );
}

/// 姿态支持只稳定姿态与悬停推力，不产生水平加速度。
#[test]
fn fallback_support_is_level_and_hovering() {
    let (quad, air) = (plant(), Airframe::default());
    let mut ctrl = controller();
    let mut state = QuadState {
        attitude: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.3),
        position: Vector3::new(0.0, 0.0, 1.5),
        ..Default::default()
    };
    for _ in 0..3000 {
        let motors = ctrl
            .support(&state, 0.0, &quad, &air, DT)
            .expect("valid support");
        let realized = air.realize(state.attitude, &motors);
        integrate(&mut state, &realized, &quad, DT);
    }
    let tilt = (state.attitude * Vector3::z())
        .z
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees();
    assert!(tilt < 3.0, "应回正，实测 {tilt:.2}°");
    assert!(
        state.position.z > 1.0,
        "应维持高度，实测 {:.2}",
        state.position.z
    );
}

/// 产品级闭环：起飞 → 保持 → 降落，状态机与 PX4 控制链一起跑。
#[test]
fn arm_takeoff_hold_land_closes_the_loop() {
    let (quad, air) = (plant(), Airframe::default());
    let mut ctrl = controller();
    let mut fsm = FlightFsm::new(FsmParams::default());
    let mut state = QuadState {
        position: Vector3::new(0.0, 0.0, 0.02),
        ..Default::default()
    };
    let health = Health {
        estimator_ready: true,
        airframe_ready: true,
        odometry_alive: true,
        imu_alive: true,
        attitude_session_ok: true,
    };
    fsm.update(DT, &state, None, health);
    fsm.command(Command::Arm, health, &state).unwrap();
    fsm.command(Command::Takeoff { altitude: 1.0 }, health, &state)
        .unwrap();

    let fly = |state: &mut QuadState, fsm: &mut FlightFsm, ctrl: &mut Controller, steps: usize| {
        for _ in 0..steps {
            let out = fsm.update(DT, state, None, health);
            let motors = if out.motors_enabled {
                ctrl.update(state, &out.setpoint, &quad, &air, DT, fsm.on_ground(state))
                    .expect("valid setpoint")
            } else {
                [0.0; 4]
            };
            let realized = air.realize(state.attitude, &motors);
            integrate(state, &realized, &quad, DT);
        }
    };

    fly(&mut state, &mut fsm, &mut ctrl, 12_000);
    assert_eq!(fsm.state(), FlightState::Hold);
    let altitude = fsm.altitude_above_origin(&state);
    assert!((altitude - 1.0).abs() < 0.12, "起飞高度 {altitude:.3} m");

    fsm.command(Command::Land, health, &state).unwrap();
    fly(&mut state, &mut fsm, &mut ctrl, 12_000);
    assert_eq!(fsm.state(), FlightState::Disarmed);
    assert!(
        fsm.altitude_above_origin(&state) < 0.1,
        "落地高度 {:.3}",
        fsm.altitude_above_origin(&state)
    );
}

/// 瞬时无效位置设定值保留上一次有效推力，不能把电机切零。
#[test]
fn invalid_setpoint_keeps_last_valid_thrust() {
    let (quad, air) = (plant(), Airframe::default());
    let mut ctrl = controller();
    let mut state = QuadState {
        position: Vector3::new(0.0, 0.0, 1.5),
        ..Default::default()
    };
    let hover = PositionSetpoint {
        position: state.position,
        ..Default::default()
    };
    for _ in 0..100 {
        tick(&mut state, &hover, &mut ctrl, &quad, &air, Vector3::zeros());
    }
    let bad = PositionSetpoint {
        position: Vector3::repeat(f32::NAN),
        velocity: Vector3::repeat(f32::NAN),
        acceleration: Vector3::repeat(f32::NAN),
        ..Default::default()
    };
    let motors = ctrl
        .update(&state, &bad, &quad, &air, DT, false)
        .expect("last valid attitude setpoint is kept");
    assert!(ctrl.input_invalid);
    assert!(motors.iter().all(|m| m.is_finite() && *m > 0.0));
}
