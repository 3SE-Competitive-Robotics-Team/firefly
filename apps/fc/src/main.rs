//! 飞控进程：1kHz 控制环（`firefly-flight`）+ iceoryx2。
//!
//! 输入：
//! - `Firefly/PlantState`（真值，每物理步）——仅供倾斜误差评测。
//! - `Firefly/Airframe`（质量/惯量/旋翼几何/电机上限，1Hz 电平）——飞控参数的唯一来源；
//! - `Firefly/Odometry`——连续 odom 系位置/速度/航向反馈；
//! - `Firefly/CorrectedOdometry`——同时间戳配对建立 map←odom，转换地图参考；
//! - `Firefly/Imu`（陀螺 + 加计，1kHz）——独立线程的备用姿态 ESKF；
//! - `Firefly/VioAttitude`——VIO 后验的高频姿态与角速度预测；
//! - `Firefly/Reference`（规划参考：位置/速度/偏航/偏航角速度）；
//! - `Firefly/Command`（地面站指令：解锁/上锁/起飞/保持/跟踪/降落）。
//!
//! 输出：`Firefly/Control`（4 电机推力，1kHz；被控对象每步取最新）——**每个** tick 都发，
//! 上锁时发零推力：被控对象按指令新鲜度决定物理是否推进，飞控停发等于世界停转。
//!
//! [`FlightFsm`] 根据估计状态和健康条件生成参考与电机使能信号。
//! [`firefly_flight::Controller`] 是 PX4 控制核心的 1:1 移植：
//! 位置 → 姿态 → 角速度 → 分配；本进程只做坐标/单位边界与主备姿态选择。
//! 内环正常读取 VIO 预测，降级读取独立 ESKF 的姿态与去偏角速度，位置/速度来自局部里程计。
//! 真值只用于评测。姿态主备选择约定见 `docs/architecture.md#imu`。
//!
//! 控制节拍使用墙钟，诊断时间取自 IMU。未就绪时持续发零推力，使被控对象能产生初始化测量。
//!
//! 本进程不建逐 tick span（1kHz）：时延证据用 tick 统计进 rrd，
//! 跨进程时延用 Control 消息头里的发送时间戳（sim 侧测）。
//!
//! 运行：`cargo run --release -p fc`（配合 `uv run firefly-sim`）。

mod attitude;
mod config;
mod frames;
mod imu;

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use fastrace::prelude::*;
use firefly_flight::{
    Airframe, ArmingCheck, Command, Controller as Px4Controller, Event, FailsafeAction,
    FailsafeFlags, FlightFsm, FsmParams, Health, PositionSetpoint, Px4Params, QuadParams,
    QuadState, Reject,
};
use firefly_pubsub::attitude::{ATTITUDE_TOPIC, AttitudeMessage};
use firefly_pubsub::command::{COMMAND_TOPIC, CommandMessage, kind as command_kind};
use firefly_pubsub::control::{CONTROL_BUFFER_SIZE, CONTROL_TOPIC, ControlMessage};
use firefly_pubsub::node::{IpcNode, create_node};
use firefly_pubsub::odom::{CORRECTED_ODOM_TOPIC, ODOM_TOPIC, OdomMessage};
use firefly_pubsub::plant::{
    AIRFRAME_BUFFER_SIZE, AIRFRAME_TOPIC, AirframeMessage, PLANT_STATE_BUFFER_SIZE,
    PLANT_STATE_TOPIC, PlantStateMessage,
};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::reference::{REFERENCE_TOPIC, ReferenceMessage};
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::viz::{SCALARS_MAX, ScalarMessage, VizPublisher};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalGuard, SignalHandler};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

/// `configs/fc.toml` 缺省路径（相对运行目录，通常为仓库根）。
/// 缺省配置：按安装布局解析，与启动目录无关。
fn default_config() -> PathBuf {
    firefly_config::paths::config("fc.toml")
}
/// 每拍末尾用于对齐截止时间的自旋窗口。
const SPIN_MARGIN: Duration = Duration::from_micros(150);
/// 里程计陈旧阈值（墙钟秒）：覆盖 10Hz 视觉更新间隔，触发姿态降级。
const ODOM_STALE_LIMIT: Duration = Duration::from_millis(500);
/// IMU 陈旧阈值（墙钟）；陈旧则不得解锁。
const IMU_STALE_LIMIT: Duration = Duration::from_millis(50);
/// 参考流新鲜窗口（墙钟）：planner 以 10Hz 发布，5 帧未到即视为流断（断开后由
/// 状态机的 `reference_timeout` 计时回落位置保持）。
const REFERENCE_STALE_LIMIT: Duration = Duration::from_millis(500);
/// 落后节拍上限：连续落后超过这么多个周期就重新对齐（避免越追越紧）。
const MAX_LAG_TICKS: u32 = 10;
/// 可视化发布周期（秒）：10Hz。
const VIZ_PERIOD: f64 = 0.1;
/// 诊断（日志 + 可视化）周期（秒）：1Hz。
const STATS_PERIOD: f64 = 1.0;

/// 退出标志（信号处理器置位，主循环检查）。
static RUNNING: AtomicBool = AtomicBool::new(true);

/// 解析 `--config <path>`（缺省 [`default_config`]）。
fn parse_config_path() -> Result<PathBuf, String> {
    let mut it = std::env::args().skip(1);
    let mut path = default_config();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--config" => {
                path = PathBuf::from(
                    it.next()
                        .ok_or_else(|| "missing --config value".to_owned())?,
                );
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(path)
}

/// 连续 odom 系位置/速度反馈快照，仅来自 `Firefly/Odometry`。
#[derive(Clone, Copy)]
struct Estimate {
    /// 状态的测量时间（秒）。
    timestamp: f64,
    /// 估计位置（m）。
    position: Vector3<f32>,
    /// 估计速度（m/s）。
    velocity: Vector3<f32>,
    /// 最近有效估计到达的墙钟时刻。
    received_at: Instant,
    /// 估计器已初始化（解锁前置条件之一）。
    initialized: bool,
}

/// 运行期输入快照（每 tick 排空到最新）。
#[derive(Default)]
struct Inputs {
    /// 真值评测样本；控制与健康判据不得读取。
    plant: Option<(PlantStateMessage, Instant)>,
    /// 机体/执行器（装配成 `firefly-flight` 的参数）。
    vehicle: Option<(QuadParams, Airframe)>,
    /// odom 系位置/速度反馈。
    estimate: Option<Estimate>,
    frames: frames::ControlFrames,
    /// 最新 IMU 传感器时间（秒），作为控制与诊断时间轴。
    imu_time: f64,
    /// 独立 IMU 工作线程发布的完整快照。
    attitude: Option<imu::Snapshot>,
    primary: attitude::Input,
    /// 参考（位置/速度/偏航/偏航角速度）与到达墙钟时刻（判新鲜）。
    reference: Option<(ReferenceMessage, Instant)>,
    /// 最新地面站指令（序号最大的那条）。
    command: Option<CommandMessage>,
}

/// 控制输入、指令输出与评测端口。
struct Ports {
    plant: Subscriber<PlantStateMessage>,
    airframe: Subscriber<AirframeMessage>,
    odom: Subscriber<OdomMessage>,
    corrected: Subscriber<OdomMessage>,
    attitude: Subscriber<AttitudeMessage>,
    reference: Subscriber<ReferenceMessage>,
    command: Subscriber<CommandMessage>,
    control: Publisher<ControlMessage>,
    viz: VizPublisher,
}

impl Ports {
    fn open(node: &IpcNode) -> Result<Self, firefly_error::Error> {
        Ok(Self {
            plant: Subscriber::with_topic_and_buffer(
                node,
                PLANT_STATE_TOPIC,
                PLANT_STATE_BUFFER_SIZE,
            )?,
            airframe: Subscriber::with_topic_and_buffer(
                node,
                AIRFRAME_TOPIC,
                AIRFRAME_BUFFER_SIZE,
            )?,
            odom: Subscriber::with_topic(node, ODOM_TOPIC)?,
            corrected: Subscriber::with_topic(node, CORRECTED_ODOM_TOPIC)?,
            attitude: Subscriber::with_topic(node, ATTITUDE_TOPIC)?,
            reference: Subscriber::with_topic(node, REFERENCE_TOPIC)?,
            command: Subscriber::with_topic(node, COMMAND_TOPIC)?,
            control: Publisher::with_topic_and_buffer(
                node,
                CONTROL_TOPIC,
                Some(CONTROL_BUFFER_SIZE),
            )?,
            viz: VizPublisher::new(node)?,
        })
    }

    /// 可视化发布器（只读借用，供 [`Controller::publish_viz`]）。
    fn viz_ref(&self) -> &VizPublisher {
        &self.viz
    }

    /// 非阻塞排空控制输入，保留每个话题的最新有效消息。
    #[allow(clippy::too_many_lines)]
    fn drain(&self, inputs: &mut Inputs) -> Result<(), firefly_error::Error> {
        while let Some(sample) = self.attitude.receive()? {
            inputs.primary.observe(*sample, Instant::now());
        }
        while let Some(sample) = self.plant.receive()? {
            inputs.plant = Some((*sample, Instant::now()));
        }
        while let Some(sample) = self.airframe.receive()? {
            inputs.vehicle = Some(airframe_to_params(&sample));
        }
        while let Some(sample) = self.odom.receive()? {
            let m = *sample;
            let received_at = Instant::now();
            if !m.is_initialized
                && m.timestamp.is_finite()
                && let Some(estimate) = inputs
                    .estimate
                    .as_mut()
                    .filter(|e| m.timestamp > e.timestamp)
            {
                estimate.initialized = false;
            }
            if !inputs.frames.observe_odom(&m, received_at) {
                continue;
            }
            let position = Vector3::new(
                m.position_x as f32,
                m.position_y as f32,
                m.position_z as f32,
            );
            let velocity = Vector3::new(
                m.velocity_x as f32,
                m.velocity_y as f32,
                m.velocity_z as f32,
            );
            if !position
                .iter()
                .chain(velocity.iter())
                .all(|v| v.is_finite())
            {
                continue;
            }
            inputs.estimate = Some(Estimate {
                timestamp: m.timestamp,
                position,
                velocity,
                initialized: m.is_initialized,
                received_at,
            });
        }
        while let Some(sample) = self.corrected.receive()? {
            inputs.frames.observe_corrected(*sample, Instant::now());
        }
        while let Some(sample) = self.reference.receive()? {
            let m = *sample;
            if valid_reference(&m)
                && m.timestamp >= 0.0
                && inputs
                    .reference
                    .is_none_or(|(old, _)| m.timestamp > old.timestamp)
            {
                inputs.reference = Some((m, Instant::now()));
            }
        }
        // 只留序号最大的指令：一次性 CLI 为打通信道会重复投递同一条，积压也折叠成最新
        while let Some(sample) = self.command.receive()? {
            if inputs
                .command
                .is_none_or(|cur| sample.sequence > cur.sequence)
            {
                inputs.command = Some(*sample);
            }
        }
        Ok(())
    }
}

fn valid_reference(m: &ReferenceMessage) -> bool {
    [
        m.timestamp,
        m.position_x,
        m.position_y,
        m.position_z,
        m.velocity_x,
        m.velocity_y,
        m.velocity_z,
        m.yaw,
        m.yaw_dot,
    ]
    .iter()
    .all(|v| v.is_finite())
}

/// `AirframeMessage` → `firefly-flight` 参数（被控对象是唯一来源）。
fn airframe_to_params(msg: &AirframeMessage) -> (QuadParams, Airframe) {
    let quad = QuadParams {
        mass: msg.mass as f32,
        inertia: [
            msg.inertia_x as f32,
            msg.inertia_y as f32,
            msg.inertia_z as f32,
        ],
        linear_drag: msg.linear_drag as f32,
        angular_drag: msg.angular_drag as f32,
    };
    let airframe = Airframe {
        rotors: std::array::from_fn(|i| firefly_flight::Rotor {
            position: msg.rotor_positions[i].map(|v| v as f32),
            spin: msg.rotor_spins[i] as f32,
        }),
        max_thrust_per_motor: msg.max_thrust_per_motor as f32,
        torque_coefficient: msg.torque_coefficient as f32,
    };
    (quad, airframe)
}

/// 健康电平：解锁前置条件与失效保护的输入（飞控自己算，不由被控对象喂）。
fn health_of(inputs: &Inputs) -> Health {
    Health {
        estimator_ready: inputs.estimate.is_some_and(|e| e.initialized)
            && inputs.primary.fresh(inputs.imu_time),
        airframe_ready: inputs.vehicle.is_some(),
        attitude_session_ok: inputs.primary.session_ok(),
        odometry_alive: inputs
            .estimate
            .is_some_and(|e| e.received_at.elapsed() <= ODOM_STALE_LIMIT),
        imu_alive: inputs
            .attitude
            .is_some_and(|s| s.ready && s.received.elapsed() <= IMU_STALE_LIMIT)
            || inputs.primary.fresh(inputs.imu_time),
    }
}

/// 参考外推上限（仿真秒）：超过就认为参考流本身有问题，不做外推。
const REFERENCE_EXTRAPOLATE_LIMIT_S: f64 = 0.3;
/// 参考流新鲜时转成位置模式参考；陈旧样本按不存在处理——断开时长由状态机计时
/// （状态机只在 `Track` 下消它，其余模式自己生参考）。
fn fresh_reference(inputs: &Inputs) -> Option<PositionSetpoint> {
    let (msg, at) = inputs.reference?;
    if at.elapsed() > REFERENCE_STALE_LIMIT {
        return None;
    }
    // 用 IMU 时刻（仿真钟）算外推量：参考消息带的是它被算出的仿真时刻。
    let now_sim = if inputs.attitude.is_some() {
        inputs.imu_time
    } else {
        msg.timestamp
    };
    let dt = (now_sim - msg.timestamp).clamp(0.0, REFERENCE_EXTRAPOLATE_LIMIT_S);
    inputs
        .frames
        .reference(&msg, Instant::now(), ODOM_STALE_LIMIT, dt)
}

/// 线上指令 → 状态机指令（`None` = 未知编码，忽略）。
fn command_of(msg: &CommandMessage) -> Option<Command> {
    match msg.kind {
        command_kind::ARM => Some(Command::Arm),
        command_kind::DISARM => Some(Command::Disarm),
        command_kind::TAKEOFF => Some(Command::Takeoff {
            altitude: msg.altitude as f32,
        }),
        command_kind::HOLD => Some(Command::Hold),
        command_kind::TRACK => Some(Command::Track),
        command_kind::LAND => Some(Command::Land),
        _ => None,
    }
}

/// 状态转移与失效保护进日志（双 sink：stderr + `Firefly/Log` → rrd `logs/fc`）。
fn report_event(event: Event) {
    match event {
        Event::None => {}
        Event::Transition { from, to } => log::info!("模式 {} → {}", from.name(), to.name()),
        Event::FailsafeEstimatorLost => {
            log::error!("失效保护：状态估计丢失：空中仅姿态稳定，地面上锁；无法保证保高或定点");
        }
        Event::FailsafeReferenceLost => {
            log::warn!("失效保护：参考流失联 → 位置保持");
        }
    }
}

/// 控制环的跨 tick 状态。
struct Controller {
    /// 最近一次控制使用的姿态；所属航向系由 source 决定。
    attitude: UnitQuaternion<f32>,
    attitude_ready: bool,
    source: attitude::Source,
    standby_yaw: f32,
    position_error: Vector3<f32>,
    position_target: Vector3<f32>,
    /// 飞行状态机（模式与安全层）。
    fsm: FlightFsm,
    /// 已执行的最大指令序号（重复投递去重）。
    last_command_seq: u64,
    /// 相对起飞点高度（m；最近一 tick 的估计高度，进日志与 rrd）。
    altitude: f32,
    /// 上次可视化时刻（仿真时间）。
    last_viz: f64,
    /// 最近一次控制使用的机体系角速度（FLU，rad/s），供诊断。
    ang_vel: Vector3<f32>,
    /// PX4 控制链（位置 → 姿态 → 角速度 → 分配）。
    px4: Px4Controller,
    /// 解锁前首个失败项的位掩码（未解锁时有效）。
    arming_failed: u32,
    /// 最近一 tick 的失效条件位掩码。
    failsafe_flags: FailsafeFlags,
    /// 最近一 tick 的失效动作。
    failsafe_action: FailsafeAction,
}

impl Controller {
    fn new(fsm: FsmParams, params: Px4Params) -> Self {
        Self {
            attitude: UnitQuaternion::identity(),
            attitude_ready: false,
            source: attitude::Source::Unavailable,
            standby_yaw: 0.,
            position_error: Vector3::zeros(),
            position_target: Vector3::zeros(),
            fsm: FlightFsm::new(fsm),
            last_command_seq: 0,
            altitude: 0.0,
            last_viz: f64::NEG_INFINITY,
            ang_vel: Vector3::zeros(),
            px4: Px4Controller::new(params),
            arming_failed: 0,
            failsafe_flags: FailsafeFlags::default(),
            failsafe_action: FailsafeAction::None,
        }
    }

    /// 空中失联只用 IMU 维持姿态支持；禁止陈旧位置进入控制律。
    fn motor_output(&mut self, inputs: &Inputs, dt: f32) -> ([f32; 4], bool) {
        let health = health_of(inputs);
        let recovering = self.fsm.state() == firefly_flight::FlightState::AttitudeFallback
            && health.estimator_ready
            && health.odometry_alive
            && inputs.command.is_some_and(|m| {
                m.sequence > self.last_command_seq
                    && matches!(m.kind, command_kind::HOLD | command_kind::LAND)
            });
        let standby = !health.estimator_ready
            || !health.odometry_alive
            || (self.fsm.state() == firefly_flight::FlightState::AttitudeFallback && !recovering);
        let (attitude, ang_vel) = self.attitude_and_rates(inputs, standby);
        let (position, velocity) = inputs
            .estimate
            .map_or((Vector3::zeros(), Vector3::zeros()), |est| {
                (est.position, est.velocity)
            });
        let state = QuadState {
            position,
            velocity,
            attitude,
            ang_vel,
        };
        self.ang_vel = state.ang_vel;
        // 解锁还要求独立 ESKF 热备可用；诊断必须与 `apply_command` 的实际门控一致，
        // 否则地面站会看到“检查全过、Arm 仍被拒”。
        let mut arm_health = health;
        arm_health.estimator_ready &= inputs
            .attitude
            .is_some_and(|s| s.ready && s.received.elapsed() <= IMU_STALE_LIMIT);
        self.arming_failed = if self.fsm.state().motors_enabled() {
            0
        } else {
            self.fsm
                .arming_check(arm_health, &state)
                .err()
                .map_or(0, ArmingCheck::bit)
        };
        let mut command_health = health;
        if inputs.command.is_some_and(|m| m.kind == command_kind::ARM) {
            command_health.estimator_ready &= inputs
                .attitude
                .is_some_and(|s| s.ready && s.received.elapsed() <= IMU_STALE_LIMIT);
        }
        self.apply_command(inputs.command, command_health, &state);
        let reference = fresh_reference(inputs);
        let out = self.fsm.update(dt, &state, reference.as_ref(), health);
        self.failsafe_flags = out.flags;
        self.failsafe_action = out.failsafe;
        self.position_target = out.setpoint.position;
        self.position_error = out.setpoint.position - position;
        report_event(out.event);
        self.altitude = self.fsm.altitude_above_origin(&state);
        if !out.motors_enabled || !health.imu_alive || !self.attitude_ready {
            return ([0.0; 4], false);
        }
        let Some((quad, airframe)) = inputs.vehicle else {
            return ([0.0; 4], false);
        };
        let (motors, saturated) =
            if self.fsm.state() == firefly_flight::FlightState::AttitudeFallback {
                self.px4
                    .support(&state, self.standby_yaw, &quad, &airframe, dt)
                    .map_or(([0.0; 4], true), |m| (m, self.px4.saturated))
            } else {
                if !health.estimator_ready || !health.odometry_alive {
                    return ([0.0; 4], false);
                }
                self.px4
                    .update(
                        &state,
                        &out.setpoint,
                        &quad,
                        &airframe,
                        dt,
                        self.fsm.on_ground(&state),
                    )
                    .map_or(([0.0; 4], true), |m| (m, self.px4.saturated))
            };
        if !self.fsm.state().motors_enabled() {
            self.px4.reset();
        }
        (motors, saturated)
    }

    /// 每 tick 发布控制；启动等待期的零推力允许仿真推进并产生初始化测量。
    fn step(
        &mut self,
        ports: &Ports,
        inputs: &Inputs,
        dt: f32,
        tick: u64,
        stats: &mut Stats,
    ) -> Result<(), firefly_error::Error> {
        let (motors, saturated) = self.motor_output(inputs, dt);
        stats.saturated += u64::from(saturated);
        ports.control.publish(ControlMessage {
            state_time: inputs.imu_time,
            thrust: motors.map(f64::from),
            tick,
        })?;
        stats.published += 1;
        let truth = inputs.plant.map(|(plant, _)| {
            UnitQuaternion::new_normalize(Quaternion::new(
                plant.quat_w as f32,
                plant.quat_x as f32,
                plant.quat_y as f32,
                plant.quat_z as f32,
            ))
        });
        self.publish_viz(ports, inputs.imu_time, self.attitude, truth, &motors);
        Ok(())
    }

    /// 执行最新指令一次：按序号去重（一次性 CLI 为打通信道会重复投递），
    /// 未知编码忽略，拒绝原因逐项记日志（进 rrd `logs/fc`）。
    fn apply_command(&mut self, msg: Option<CommandMessage>, health: Health, state: &QuadState) {
        let Some(msg) = msg else { return };
        if msg.sequence <= self.last_command_seq {
            return;
        }
        self.last_command_seq = msg.sequence;
        let Some(cmd) = command_of(&msg) else {
            log::warn!("未知指令编码 {}（已忽略）", msg.kind);
            return;
        };
        match self.fsm.command(cmd, health, state) {
            Ok(event) => {
                log::info!("指令 {cmd:?} → {}", self.fsm.state().name());
                report_event(event);
            }
            Err(Reject::Arming(check)) => {
                log::warn!("指令 {cmd:?} 被拒：{}（{}）", check.name(), check.reason());
            }
            Err(reject) => log::warn!("指令 {cmd:?} 被拒：{}", reject.reason()),
        }
    }

    /// 主备只选择、不融合；降级锁存的航向目标属于备用 ESKF 的本地重力系。
    fn attitude_and_rates(
        &mut self,
        inputs: &Inputs,
        standby: bool,
    ) -> (UnitQuaternion<f32>, Vector3<f32>) {
        let selected = if standby {
            inputs
                .attitude
                .filter(|s| s.ready && s.received.elapsed() <= IMU_STALE_LIMIT)
                .map(|s| (s.rotation, s.rate, attitude::Source::Standby))
        } else {
            inputs
                .primary
                .latest
                .filter(|_| inputs.primary.fresh(inputs.imu_time))
                .map(|s| (s.rotation, s.rate, attitude::Source::Vio))
        };
        self.attitude_ready = selected.is_some();
        let source = selected.map_or(attitude::Source::Unavailable, |(_, _, source)| source);
        if source != self.source {
            log::info!("姿态源 {} → {}", self.source.name(), source.name());
            if let Some((rotation, _, attitude::Source::Standby)) = selected {
                let forward = rotation * Vector3::x();
                self.standby_yaw = forward.y.atan2(forward.x);
            }
            self.source = source;
        }
        if let Some((rotation, rate, _)) = selected {
            self.attitude = rotation;
            (rotation, rate)
        } else {
            (self.attitude, Vector3::zeros())
        }
    }

    /// 控制量 + 模式 + 姿态校验进 rrd（10Hz）。
    #[allow(clippy::too_many_lines)] // 诊断字段按实体平铺，拆函数只会增加跳转
    fn publish_viz(
        &mut self,
        ports: &Ports,
        sim_time: f64,
        attitude: UnitQuaternion<f32>,
        plant_attitude: Option<UnitQuaternion<f32>>,
        motors: &[f32; 4],
    ) {
        if sim_time - self.last_viz < VIZ_PERIOD {
            return;
        }
        self.last_viz = sim_time;
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/imu/source",
            &[self.source.code()],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/imu/quaternion_xyzw",
            attitude.coords.cast::<f64>().as_slice(),
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/position_target_odom",
            self.position_target.cast::<f64>().as_slice(),
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/position_error_odom",
            self.position_error.cast::<f64>().as_slice(),
        );
        let total: f32 = motors.iter().sum();
        let tilt = (attitude * Vector3::z())
            .z
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/thrust_total",
            &[f64::from(total)],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/tilt_deg",
            &[f64::from(tilt)],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/motors",
            &motors.map(f64::from),
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/rate_sp_frd",
            &[
                f64::from(self.px4.rates_setpoint.roll),
                f64::from(self.px4.rates_setpoint.pitch),
                f64::from(self.px4.rates_setpoint.yaw),
            ],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/rate_flu",
            &[
                f64::from(self.ang_vel.x),
                f64::from(self.ang_vel.y),
                f64::from(self.ang_vel.z),
            ],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/torque_sp_frd",
            &[
                f64::from(self.px4.torque_setpoint.xyz.x),
                f64::from(self.px4.torque_setpoint.xyz.y),
                f64::from(self.px4.torque_setpoint.xyz.z),
            ],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/allocated_control",
            &[
                f64::from(self.px4.allocated_control[0]),
                f64::from(self.px4.allocated_control[1]),
                f64::from(self.px4.allocated_control[2]),
                f64::from(self.px4.allocated_control[3]),
            ],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/saturated",
            &[f64::from(self.px4.saturated)],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/hover_thrust",
            &[f64::from(self.px4.hover_thrust)],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/arming/failed",
            &[f64::from(self.arming_failed)],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/failsafe/flags",
            &[f64::from(self.failsafe_flags.bits())],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/failsafe/action",
            &[f64::from(self.failsafe_action.code())],
        );
        if let Some(truth) = plant_attitude {
            // 重力在机体系中的方向不依赖局部航向，可直接与真值比较。
            let error = (attitude.inverse() * Vector3::z())
                .angle(&(truth.inverse() * Vector3::z()))
                .to_degrees();
            publish_scalars(
                ports.viz_ref(),
                sim_time,
                "fc/debug/tilt_err_deg",
                &[f64::from(error)],
            );
        }
        // 模式编码见 `firefly_flight::FlightState::code`
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/state",
            &[f64::from(self.fsm.state().code())],
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/altitude",
            &[f64::from(self.altitude)],
        );
    }
}

/// 每 tick 的统计（1Hz 汇总进日志 + rrd）。
#[derive(Default)]
struct Stats {
    ticks: u64,
    published: u64,
    saturated: u64,
    late: u64,
    imu: u64,
    max_late: Duration,
}

/// 节拍对齐：睡到 `next - SPIN_MARGIN`，余下自旋；落后则计数，落后过多重新对齐。
fn pace(next: &mut Instant, period: Duration, stats: &mut Stats) {
    let now = Instant::now();
    if *next > now {
        let remaining = *next - now;
        if remaining > SPIN_MARGIN {
            thread::sleep(remaining.saturating_sub(SPIN_MARGIN));
        }
        while Instant::now() < *next {
            std::hint::spin_loop();
        }
    } else {
        let late = now - *next;
        stats.late += 1;
        stats.max_late = stats.max_late.max(late);
        if late > period * MAX_LAG_TICKS {
            *next = now + period;
            return;
        }
    }
    *next += period;
}

/// 发布一条标量可视化（10Hz；`values[..n]` 单实体多值）。
fn publish_scalars(viz: &VizPublisher, t: f64, entity: &str, values: &[f64]) {
    let mut msg = ScalarMessage::base(t, entity);
    let count = values.len().min(SCALARS_MAX);
    msg.scalars[..count].copy_from_slice(&values[..count]);
    msg.scalar_count = count as u32;
    let _ = viz.publish(msg);
}

#[allow(clippy::too_many_lines)] // 进程生命周期与控制节拍编排
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config_path = parse_config_path().map_err(|e| {
        eprintln!("{e}\n用法：fc [--config configs/fc.toml]");
        std::process::exit(2);
    })?;
    let cfg = firefly_config::startup::<config::FcConfig>(
        &config_path,
        firefly_observability::init,
        config::FcConfig::validate,
    )?;
    let period = Duration::from_secs_f64(1.0 / cfg.rate_hz);
    let dt = (1.0 / cfg.rate_hz) as f32;

    // 优雅退出：SIGINT/SIGTERM 置位退出标志（guard 存活到进程退出）。
    // 回调只写标志：信号上下文里不做日志/分配（异步信号安全）。
    let _signal_guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_signal| {
            RUNNING.store(false, Ordering::Relaxed);
        },
    )?;

    let node = create_node()?;
    let ports = Ports::open(&node)?;
    let log_ipc = firefly_observability::init_ipc(&node, "fc");
    log::info!(
        "飞控进程启动：{} Hz 控制环，配置 {}；初始 DISARMED，等 `Firefly/Command`",
        config_path.display(),
        cfg.rate_hz
    );

    let imu_worker = imu::Worker::spawn(cfg.imu)?;
    let mut imu_count = 0u64;
    let mut inputs = Inputs::default();
    let mut controller = Controller::new(cfg.fsm, cfg.control);
    let mut stats = Stats::default();
    let mut window_start = Instant::now();
    let mut sim_time = 0.0f64;
    let mut tick = 0u64;
    let mut next = Instant::now() + period;

    while RUNNING.load(Ordering::Relaxed) {
        ports.drain(&mut inputs)?;
        if let Some(snapshot) = imu_worker.snapshot() {
            stats.imu += snapshot.count.saturating_sub(imu_count);
            imu_count = snapshot.count;
            inputs.imu_time = snapshot.timestamp;
            inputs.attitude = Some(snapshot);
        }
        sim_time = inputs.imu_time;
        controller.step(&ports, &inputs, dt, tick, &mut stats)?;
        imu_worker.grounded.store(
            controller.fsm.state() == firefly_flight::FlightState::Disarmed,
            Ordering::Relaxed,
        );
        firefly_observability::pump_log_ipc(&log_ipc);
        stats.ticks += 1;
        tick += 1;
        pace(&mut next, period, &mut stats);

        // ---- 诊断（1Hz）：tick 率/晚到/饱和/发布数（唯一时延证据来源） ----
        let window = window_start.elapsed();
        if window >= Duration::from_secs_f64(STATS_PERIOD) {
            let root = Span::root("fc-stats", SpanContext::random().sampled(false));
            let _guard = root.set_local_parent();
            let rate = stats.ticks as f64 / window.as_secs_f64();
            log::info!(
                "t={:.2} 状态 {}（高度 {:.2} m），指令 {} 条（{:.0} Hz），tick {}，晚到 {:.1}%（最大 {:.2} ms），饱和 {:.1}%，IMU {:.0} Hz，姿态源 {}",
                sim_time,
                controller.fsm.state().name(),
                controller.altitude,
                stats.published,
                stats.published as f64 / window.as_secs_f64(),
                stats.ticks,
                stats.late as f64 / stats.ticks.max(1) as f64 * 100.0,
                stats.max_late.as_secs_f64() * 1e3,
                stats.saturated as f64 / stats.published.max(1) as f64 * 100.0,
                stats.imu as f64 / window.as_secs_f64(),
                controller.source.name(),
            );
            publish_scalars(
                &ports.viz,
                sim_time,
                "fc/imu/health",
                &[
                    f64::from(controller.attitude_ready),
                    f64::from(inputs.primary.fresh(sim_time)),
                    inputs
                        .primary
                        .latest
                        .map_or(-1., |s| sim_time - s.message.posterior_time),
                ],
            );
            if let Some(s) = inputs.attitude {
                publish_scalars(
                    &ports.viz,
                    sim_time,
                    "fc/standby/health",
                    &[f64::from(s.ready), s.received.elapsed().as_secs_f64()],
                );
                publish_scalars(
                    &ports.viz,
                    sim_time,
                    "fc/standby/accel_gate",
                    &[s.nis, f64::from(s.accel_accepted)],
                );
                publish_scalars(&ports.viz, sim_time, "fc/standby/gyro_bias", &s.bias);
                publish_scalars(
                    &ports.viz,
                    sim_time,
                    "fc/standby/quaternion_xyzw",
                    s.rotation.coords.map(f64::from).as_slice(),
                );
                publish_scalars(
                    &ports.viz,
                    sim_time,
                    "fc/standby/attitude_variance",
                    &s.covariance_diagonal[..3],
                );
                publish_scalars(
                    &ports.viz,
                    sim_time,
                    "fc/standby/bias_variance",
                    &s.covariance_diagonal[3..],
                );
            }
            if let Some(s) = inputs.primary.latest {
                publish_scalars(
                    &ports.viz,
                    sim_time,
                    "fc/vio/gyro_bias",
                    &s.message.gyro_bias,
                );
            }
            publish_scalars(&ports.viz, sim_time, "fc/debug/tick_rate", &[rate]);
            publish_scalars(
                &ports.viz,
                sim_time,
                "fc/debug/late_max_ms",
                &[stats.max_late.as_secs_f64() * 1e3],
            );
            publish_scalars(
                &ports.viz,
                sim_time,
                "fc/debug/saturated",
                &[stats.saturated as f64],
            );
            window_start = Instant::now();
            stats = Stats::default();
        }
    }

    firefly_observability::pump_log_ipc(&log_ipc);
    log::info!(
        "飞控进程退出（t={sim_time:.2}，状态 {}）",
        controller.fsm.state().name()
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stale_grounded_estimator_cannot_start_hover_thrust() {
        let mut controller = Controller::new(FsmParams::default(), Px4Params::default());
        let mut inputs = sensor_inputs();
        inputs.command = Some(CommandMessage {
            kind: command_kind::ARM,
            sequence: 1,
            ..Default::default()
        });
        controller.motor_output(&inputs, 0.001);
        inputs.estimate.as_mut().unwrap().initialized = false;
        assert_eq!(
            controller.motor_output(&inputs, 0.001).0.map(f32::to_bits),
            [0; 4]
        );
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::Disarmed
        );
    }

    fn sensor_inputs() -> Inputs {
        let mut primary = attitude::Input::default();
        primary.observe(
            AttitudeMessage {
                timestamp: 0.,
                posterior_time: 0.,
                session: 7,
                quat_xyzw: [0., 0., 0., 1.],
                angular_velocity: [0.; 3],
                gyro_bias: [0.; 3],
            },
            Instant::now(),
        );
        Inputs {
            primary,
            attitude: Some(imu::Snapshot {
                timestamp: 0.,
                received: Instant::now(),
                count: 1,
                rotation: UnitQuaternion::identity(),
                rate: Vector3::zeros(),
                ready: true,
                nis: 0.,
                accel_accepted: false,
                bias: [0.; 3],
                covariance_diagonal: [0.; 6],
            }),
            vehicle: Some((QuadParams::default(), Airframe::default())),
            estimate: Some(Estimate {
                timestamp: 0.,
                position: Vector3::zeros(),
                velocity: Vector3::zeros(),
                initialized: true,
                received_at: Instant::now(),
            }),
            ..Inputs::default()
        }
    }

    #[test]
    fn truth_cannot_initialize_or_arm_controller() {
        let inputs = Inputs {
            plant: Some((PlantStateMessage::default(), Instant::now())),
            vehicle: Some((QuadParams::default(), Airframe::default())),
            command: Some(CommandMessage {
                kind: command_kind::ARM,
                sequence: 1,
                ..CommandMessage::default()
            }),
            ..Inputs::default()
        };
        let mut controller = Controller::new(FsmParams::default(), Px4Params::default());
        let (motors, _) = controller.motor_output(&inputs, 0.001);
        assert_eq!(motors.map(f32::to_bits), [0; 4]);
        assert!(!controller.attitude_ready);
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::Disarmed
        );
    }

    #[test]
    fn changing_truth_cannot_change_motor_output() {
        let mut inputs = sensor_inputs();
        let mut a = Controller::new(FsmParams::default(), Px4Params::default());
        let mut b = Controller::new(FsmParams::default(), Px4Params::default());
        for _ in 0..1000 {
            a.motor_output(&inputs, 0.001);
            b.motor_output(&inputs, 0.001);
        }
        for (kind, sequence) in [(command_kind::ARM, 1), (command_kind::TAKEOFF, 2)] {
            inputs.command = Some(CommandMessage {
                kind,
                sequence,
                altitude: 1.0,
                ..CommandMessage::default()
            });
            inputs.plant = None;
            let (left, _) = a.motor_output(&inputs, 0.001);
            inputs.plant = Some((
                PlantStateMessage {
                    position_x: 1000.0,
                    velocity_z: -100.0,
                    quat_w: f64::NAN,
                    ..PlantStateMessage::default()
                },
                Instant::now(),
            ));
            let (right, _) = b.motor_output(&inputs, 0.001);
            assert_eq!(left.map(f32::to_bits), right.map(f32::to_bits));
        }
        assert_eq!(a.fsm.state(), firefly_flight::FlightState::Takeoff);
        assert!(a.motor_output(&inputs, 0.001).0.iter().any(|m| *m > 0.0));
        inputs.estimate.as_mut().unwrap().received_at =
            Instant::now().checked_sub(Duration::from_secs(2)).unwrap();
        let (fallback, _) = a.motor_output(&inputs, 0.001);
        assert!(fallback.iter().all(|m| m.is_finite() && *m > 0.));
        assert_eq!(a.fsm.state(), firefly_flight::FlightState::AttitudeFallback);
        let estimate = inputs.estimate.as_mut().unwrap();
        estimate.position = Vector3::repeat(10000.);
        estimate.velocity = Vector3::repeat(-1000.);
        let (same, _) = a.motor_output(&inputs, 0.001);
        assert_eq!(fallback.map(f32::to_bits), same.map(f32::to_bits));
    }
    fn airborne(inputs: &mut Inputs) -> Controller {
        let mut controller = Controller::new(FsmParams::default(), Px4Params::default());
        for (kind, sequence) in [(command_kind::ARM, 1), (command_kind::TAKEOFF, 2)] {
            inputs.command = Some(CommandMessage {
                kind,
                sequence,
                altitude: 1.,
                ..Default::default()
            });
            controller.motor_output(inputs, 0.001);
        }
        inputs.estimate.as_mut().unwrap().position.z = 1.;
        controller
    }

    #[test]
    fn primary_is_selected_and_loss_latches_independent_standby_until_command() {
        let mut inputs = sensor_inputs();
        let mut controller = airborne(&mut inputs);
        inputs.attitude.as_mut().unwrap().rotation = UnitQuaternion::from_euler_angles(0., 0., 1.8);
        controller.motor_output(&inputs, 0.001);
        assert_eq!(controller.source, attitude::Source::Vio);
        assert!(controller.attitude.angle() < 1e-6);
        // 高频主预测断流，而低频里程计仍新鲜，也必须接管。
        inputs.primary = attitude::Input::default();
        let motors = controller.motor_output(&inputs, 0.001).0;
        assert_eq!(controller.source, attitude::Source::Standby);
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::AttitudeFallback
        );
        assert!((controller.attitude.angle() - 1.8).abs() < 1e-6);
        assert!(motors.iter().all(|m| *m > 0.));
        assert!(motors.iter().all(|m| (*m - motors[0]).abs() < 1e-5));
        inputs.primary = sensor_inputs().primary;
        controller.motor_output(&inputs, 0.001);
        assert_eq!(controller.source, attitude::Source::Standby);
        inputs.command = Some(CommandMessage {
            kind: command_kind::HOLD,
            sequence: 3,
            ..Default::default()
        });
        controller.motor_output(&inputs, 0.001);
        assert_eq!(controller.source, attitude::Source::Vio);
        assert_eq!(controller.fsm.state(), firefly_flight::FlightState::Hold);
        assert!((inputs.attitude.unwrap().rotation.angle() - 1.8).abs() < 1e-6);
    }

    #[test]
    fn unavailable_standby_cuts_thrust_and_new_session_cannot_recover() {
        let mut inputs = sensor_inputs();
        let mut controller = airborne(&mut inputs);
        let mut restarted = inputs.primary.latest.unwrap().message;
        restarted.session += 1;
        restarted.timestamp += 0.01;
        inputs.primary.observe(restarted, Instant::now());
        inputs.command = Some(CommandMessage {
            kind: command_kind::HOLD,
            sequence: 3,
            ..Default::default()
        });
        controller.motor_output(&inputs, 0.001);
        assert_eq!(controller.source, attitude::Source::Standby);
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::AttitudeFallback
        );
        inputs.attitude.as_mut().unwrap().received =
            Instant::now().checked_sub(Duration::from_secs(2)).unwrap();
        assert_eq!(
            controller.motor_output(&inputs, 0.001).0.map(f32::to_bits),
            [0; 4]
        );
        assert_eq!(controller.source, attitude::Source::Unavailable);
    }

    #[test]
    fn arm_requires_ready_standby_even_with_valid_vio() {
        let mut inputs = sensor_inputs();
        inputs.attitude.as_mut().unwrap().ready = false;
        inputs.command = Some(CommandMessage {
            kind: command_kind::ARM,
            sequence: 1,
            ..Default::default()
        });
        let mut controller = Controller::new(FsmParams::default(), Px4Params::default());
        assert_eq!(
            controller.motor_output(&inputs, 0.001).0.map(f32::to_bits),
            [0; 4]
        );
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::Disarmed
        );
    }
}
