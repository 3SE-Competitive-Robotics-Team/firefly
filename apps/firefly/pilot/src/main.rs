mod attitude;
mod config;
mod frames;
mod imu;
mod matrix;
mod usb;

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use fastrace::prelude::*;
use firefly_flight::ipc::airframe_to_params;
use firefly_flight::sticks::AngleDemand;
use firefly_flight::{
    Airframe, ArmingCheck, Command, Controller as Px4Controller, Event, FailsafeAction,
    FailsafeFlags, FlightFsm, FsmParams, Health, PositionSetpoint, Px4Params, QuadParams,
    QuadState, Reject,
};
use firefly_pubsub::attitude::{ATTITUDE_TOPIC, AttitudeMessage};
use firefly_pubsub::command::{CommandMessage, kind as command_kind};
use firefly_pubsub::fc_status::{FC_STATUS_BUFFER_SIZE, FC_STATUS_TOPIC, FcStatusMessage};
use firefly_pubsub::landing::{LANDING_TARGET_TOPIC, LandingTargetMessage};
use firefly_pubsub::node::{IpcNode, create_node};
use firefly_pubsub::odom::{CORRECTED_ODOM_TOPIC, ODOM_TOPIC, OdomMessage};
use firefly_pubsub::plant::{
    AIRFRAME_BUFFER_SIZE, AIRFRAME_TOPIC, AirframeMessage, PLANT_STATE_BUFFER_SIZE,
    PLANT_STATE_TOPIC, PlantStateMessage,
};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::reference::{REFERENCE_TOPIC, ReferenceMessage};
use firefly_pubsub::request::MissionRequest;
use firefly_pubsub::sticks::{STICKS_TOPIC, SticksMessage};
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::viz::{SCALARS_MAX, ScalarMessage, VizPublisher};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalGuard, SignalHandler};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

fn default_config() -> PathBuf {
    firefly_config::paths::config("pilot.toml")
}
/// 每拍末尾用于对齐截止时间的自旋窗口。
const SPIN_MARGIN: Duration = Duration::from_micros(150);
/// 里程计到达新鲜窗口（墙钟）；超时触发姿态降级。
const ODOM_STALE_LIMIT: Duration = Duration::from_millis(500);
/// IMU 陈旧阈值（墙钟）；陈旧则不得解锁。
const IMU_STALE_LIMIT: Duration = Duration::from_millis(50);
/// 参考流到达新鲜窗口（墙钟）；断流后由状态机的 `reference_timeout` 计时回落位置保持。
const REFERENCE_STALE_LIMIT: Duration = Duration::from_millis(500);
/// 落后节拍上限：连续落后超过这么多个周期就重新对齐（避免越追越紧）。
const MAX_LAG_TICKS: u32 = 10;
const STATUS_PERIOD: f64 = 0.1;
const STATS_PERIOD: f64 = 1.0;

static RUNNING: AtomicBool = AtomicBool::new(true);

/// 运行飞行控制循环
#[derive(usage_rs::Cli)]
#[usage(bin = "pilot")]
struct Arguments {
    /// 仿真使用 IPC 杆量接口
    #[usage(long)]
    simulation: bool,
    /// 应用配置文件
    #[usage(long)]
    config: Option<PathBuf>,
}

#[derive(Clone, Copy)]
struct Estimate {
    /// 状态的测量时间（秒）。
    timestamp: f64,
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    /// 最近有效估计到达的墙钟时刻。
    received_at: Instant,
    initialized: bool,
}

#[derive(Default)]
struct Inputs {
    landing: Option<(LandingTargetMessage, Instant)>,
    /// 真值评测样本；控制与健康判据不得读取。
    plant: Option<(PlantStateMessage, Instant)>,
    vehicle: Option<(QuadParams, Airframe)>,
    estimate: Option<Estimate>,
    frames: frames::ControlFrames,
    /// 最新 IMU 传感器时间（秒），作为控制与诊断时间轴。
    imu_time: f64,
    attitude: Option<imu::Snapshot>,
    primary: attitude::Input,
    /// 参考（位置/速度/偏航/偏航角速度）与到达墙钟时刻（判新鲜）。
    reference: Option<(ReferenceMessage, Instant)>,
    command: Option<ReceivedCommand>,
}

impl Inputs {
    fn receive_command(&mut self, message: &CommandMessage) {
        if self
            .command
            .is_none_or(|cur| message.sequence > cur.sequence)
        {
            self.command = Some(ReceivedCommand::from(message));
        }
    }
}

struct Ports {
    landing: Subscriber<LandingTargetMessage>,
    plant: Subscriber<PlantStateMessage>,
    airframe: Subscriber<AirframeMessage>,
    odom: Subscriber<OdomMessage>,
    corrected: Subscriber<OdomMessage>,
    attitude: Subscriber<AttitudeMessage>,
    reference: Subscriber<ReferenceMessage>,
    command: Subscriber<CommandMessage>,
    sticks: Publisher<SticksMessage>,
    status: Publisher<FcStatusMessage>,
    viz: VizPublisher,
}

impl Ports {
    fn open(node: &IpcNode) -> Result<Self, firefly_error::Error> {
        Ok(Self {
            landing: Subscriber::with_topic(node, LANDING_TARGET_TOPIC)?,
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
            command: CommandMessage::subscriber(node)?,
            sticks: Publisher::with_topic_and_buffer(
                node,
                STICKS_TOPIC,
                Some(firefly_pubsub::sticks::STICKS_BUFFER_SIZE),
            )?,
            status: Publisher::with_topic_buffer_publishers(
                node,
                FC_STATUS_TOPIC,
                Some(FC_STATUS_BUFFER_SIZE),
                Some(1),
            )?,
            viz: VizPublisher::new(node)?,
        })
    }

    fn viz_ref(&self) -> &VizPublisher {
        &self.viz
    }

    /// 非阻塞排空控制输入，保留每个话题的最新有效消息。
    #[allow(clippy::too_many_lines)]
    fn drain(&self, inputs: &mut Inputs) -> Result<(), firefly_error::Error> {
        while let Some(sample) = self.landing.receive()? {
            if sample.timestamp.is_finite()
                && inputs
                    .landing
                    .is_none_or(|(old, _)| sample.timestamp > old.timestamp)
            {
                inputs.landing = Some((*sample, Instant::now()));
            }
        }
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
        while let Some(sample) = self.command.receive()? {
            inputs.receive_command(&sample);
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

fn health_of(inputs: &Inputs) -> Health {
    Health {
        estimator_ready: inputs.estimate.is_some_and(|e| e.initialized)
            && inputs.primary.fresh(inputs.imu_time),
        airframe_ready: inputs.vehicle.is_some(),
        attitude_session_ok: inputs.primary.session_ok(),
        landing_thrust_low: false,
        landing_motion_low: inputs.attitude.is_some_and(|s| {
            s.ready
                && s.received.elapsed() <= IMU_STALE_LIMIT
                && (s.specific_force - f64::from(firefly_flight::G)).abs() < 1.
                && s.rate.norm() < 0.15
        }),
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

#[derive(Clone, Copy)]
struct ReceivedCommand {
    sequence: u64,
    /// 未知编码也参与序号去重，消费时报告一次。
    command: Result<Command, u8>,
}

impl From<&CommandMessage> for ReceivedCommand {
    fn from(msg: &CommandMessage) -> Self {
        let command = match msg.kind {
            command_kind::ARM => Ok(Command::Arm),
            command_kind::DISARM => Ok(Command::Disarm),
            command_kind::TAKEOFF => Ok(Command::Takeoff {
                altitude: msg.altitude as f32,
            }),
            command_kind::HOLD => Ok(Command::Hold),
            command_kind::TRACK => Ok(Command::Track),
            command_kind::LAND => Ok(Command::Land),
            command_kind::PRECISION_LAND => Ok(Command::PrecisionLand),
            unknown => Err(unknown),
        };
        Self {
            sequence: msg.sequence,
            command,
        }
    }
}

fn report_event(event: Event) {
    match event {
        Event::None => {}
        Event::Transition { from, to } => log::info!("模式 {} → {}", from.name(), to.name()),
        Event::FailsafeEstimatorLost => {
            log::error!("失效保护：状态估计丢失：空中仅姿态稳定，地面上锁；无法保证保高或定点");
        }
        Event::FailsafeReferenceLost => {
            log::warn!("失效保护：参考流或降落目标失效 → 位置保持");
        }
    }
}

/// 已消费命令的单调序号；拒绝执行的命令同样只消费一次。
#[derive(Default)]
struct CommandSequence(u64);

impl CommandSequence {
    fn is_new(&self, sequence: u64) -> bool {
        sequence > self.0
    }

    fn consume(&mut self, sequence: u64) -> bool {
        if !self.is_new(sequence) {
            return false;
        }
        self.0 = sequence;
        true
    }
}

struct Controller {
    team: u32,
    landing_identity: Option<(u64, f64)>,
    /// 最近一次控制使用的姿态；所属航向系由 source 决定。
    attitude: UnitQuaternion<f32>,
    attitude_ready: bool,
    source: attitude::Source,
    standby_yaw: f32,
    position_error: Vector3<f32>,
    position_target: Vector3<f32>,
    fsm: FlightFsm,
    /// 已执行的最大指令序号（重复投递去重）。
    commands: CommandSequence,
    /// 相对起飞点高度（m；最近一 tick 的估计高度，进日志与 rrd）。
    altitude: f32,
    last_status: f64,
    status_sequence: u64,
    last_thrust: Option<f32>,
    /// 最近一次控制使用的机体系角速度（FLU，rad/s），供诊断。
    ang_vel: Vector3<f32>,
    px4: Px4Controller,
    /// 解锁前首个失败项的位掩码（未解锁时有效）。
    arming_failed: u32,
    failsafe_flags: FailsafeFlags,
    failsafe_action: FailsafeAction,
}

impl Controller {
    fn new(fsm: FsmParams, params: Px4Params) -> Self {
        Self {
            team: firefly_config::game::Team::Red.code(),
            landing_identity: None,
            attitude: UnitQuaternion::identity(),
            attitude_ready: false,
            source: attitude::Source::Unavailable,
            standby_yaw: 0.,
            position_error: Vector3::zeros(),
            position_target: Vector3::zeros(),
            fsm: FlightFsm::new(fsm),
            commands: CommandSequence::default(),
            altitude: 0.0,
            last_status: f64::NEG_INFINITY,
            status_sequence: 0,
            last_thrust: None,
            ang_vel: Vector3::zeros(),
            px4: Px4Controller::new(params),
            arming_failed: 0,
            failsafe_flags: FailsafeFlags::default(),
            failsafe_action: FailsafeAction::None,
        }
    }

    /// 空中失联只用 IMU 维持姿态支持；禁止陈旧位置进入控制律。
    fn demand(&mut self, inputs: &Inputs, dt: f32) -> Option<AngleDemand> {
        let mut health = health_of(inputs);
        health.landing_thrust_low = self
            .last_thrust
            .is_some_and(|value| self.px4.low_collective(value));
        let recovering = self.fsm.state() == firefly_flight::FlightState::AttitudeFallback
            && health.estimator_ready
            && health.odometry_alive
            && inputs.command.is_some_and(|m| {
                self.commands.is_new(m.sequence)
                    && matches!(m.command, Ok(Command::Hold | Command::Land))
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
        let command_health = if inputs
            .command
            .is_some_and(|m| matches!(m.command, Ok(Command::Arm)))
        {
            arm_health
        } else {
            health
        };
        self.observe_landing(inputs);
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
            self.px4.reset();
            self.last_thrust = None;
            return None;
        }
        let (quad, airframe) = inputs.vehicle?;
        self.px4.set_airframe(&quad, &airframe);
        let (roll, pitch, yaw, thrust, feedforward) =
            if self.fsm.state() == firefly_flight::FlightState::AttitudeFallback {
                (0., 0., self.standby_yaw, self.px4.hover_thrust, 0.)
            } else {
                if !health.estimator_ready || !health.odometry_alive {
                    return None;
                }
                let target = self.px4.position_demand(
                    &state,
                    &out.setpoint,
                    dt,
                    self.fsm.ground_contact_candidate(&state),
                );
                let (roll, pitch, yaw) =
                    firefly_flight::px4::frames::attitude_from_frd_ned(target.q_d).euler_angles();
                (
                    roll,
                    pitch,
                    yaw,
                    -target.thrust_body.z,
                    -target.yaw_sp_move_rate,
                )
            };
        let error = firefly_flight::px4::frames::wrap_pi(yaw - state.yaw());
        self.last_thrust = Some(thrust);
        Some(AngleDemand {
            roll,
            pitch,
            yaw_rate: feedforward + 2. * error,
            thrust,
        })
    }

    fn observe_landing(&mut self, inputs: &Inputs) {
        let landing = inputs.landing.and_then(|(message, received)| {
            let age = inputs.imu_time - message.timestamp;
            (message.valid == 1
                && message.team == self.team
                && received.elapsed() <= Duration::from_millis(250)
                && (0.0..=0.25).contains(&age)
                && message.reprojection_px.is_finite()
                && (0.0..=2.).contains(&message.reprojection_px)
                && message.confidence.is_finite()
                && (0.5..=1.0).contains(&message.confidence)
                && inputs
                    .primary
                    .latest
                    .is_some_and(|primary| primary.message.session == message.session))
            .then_some(Vector3::from(message.position).cast::<f32>())
        });
        let identity = inputs
            .landing
            .map(|(message, _)| (message.session, message.timestamp));
        if landing.is_none() || identity != self.landing_identity {
            self.fsm.observe_landing_target(landing);
            self.landing_identity = identity;
        }
    }

    /// 每 tick 发布控制；启动等待期的零推力允许仿真推进并产生初始化测量。
    fn step(
        &mut self,
        ports: &Ports,
        inputs: &Inputs,
        dt: f32,
        stats: &mut Stats,
    ) -> Result<(), firefly_error::Error> {
        let demand = self.demand(inputs, dt).and_then(AngleDemand::channels);
        let sticks = demand.map(|channels| SticksMessage {
            state_time: inputs.imu_time,
            channels,
        });
        ports.sticks.publish(sticks.unwrap_or(SticksMessage {
            state_time: inputs.imu_time,
            channels: [1500, 1500, 1500, 1000],
        }))?;
        stats.published += 1;
        if inputs.imu_time - self.last_status < STATUS_PERIOD {
            return Ok(());
        }
        self.last_status = inputs.imu_time;
        ports.status.publish(FcStatusMessage {
            timestamp: inputs.imu_time,
            sequence: self.status_sequence,
            mode: self.fsm.state().code(),
            can_arm: u8::from(
                self.fsm.state() == firefly_flight::FlightState::Disarmed
                    && self.arming_failed == 0,
            ),
        })?;
        self.status_sequence = self.status_sequence.wrapping_add(1);
        let truth = inputs.plant.map(|(plant, _)| {
            UnitQuaternion::new_normalize(Quaternion::new(
                plant.quat_w as f32,
                plant.quat_x as f32,
                plant.quat_y as f32,
                plant.quat_z as f32,
            ))
        });
        self.publish_viz(ports, inputs.imu_time, self.attitude, truth);
        Ok(())
    }

    /// 执行最新指令一次：按序号去重（一次性 CLI 为打通信道会重复投递），
    /// 未知编码忽略，拒绝原因逐项记日志（进 rrd `logs/fc`）。
    fn apply_command(&mut self, msg: Option<ReceivedCommand>, health: Health, state: &QuadState) {
        let Some(msg) = msg else { return };
        if !self.commands.consume(msg.sequence) {
            return;
        }
        let cmd = match msg.command {
            Ok(cmd) => cmd,
            Err(kind) => {
                log::warn!("未知指令编码 {kind}（已忽略）");
                return;
            }
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

    #[allow(clippy::too_many_lines)] // 诊断字段按实体平铺，拆函数只会增加跳转
    fn publish_viz(
        &self,
        ports: &Ports,
        sim_time: f64,
        attitude: UnitQuaternion<f32>,
        plant_attitude: Option<UnitQuaternion<f32>>,
    ) {
        let precision = match self.fsm.precision_stage() {
            None => 0.,
            Some(firefly_flight::precision::PrecisionStage::Align) => 1.,
            Some(firefly_flight::precision::PrecisionStage::Descend) => 2.,
            Some(firefly_flight::precision::PrecisionStage::Final) => 3.,
            Some(firefly_flight::precision::PrecisionStage::Reacquire) => 4.,
        };
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/precision",
            &[precision],
        );
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

        let tilt = (attitude * Vector3::z())
            .z
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/tilt_deg",
            &[f64::from(tilt)],
        );
        let vertical = self.px4.debug_vertical();
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/vertical_ned",
            &[
                f64::from(vertical[0]),
                f64::from(vertical[1]),
                f64::from(vertical[2]),
                f64::from(vertical[3]),
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
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/altitude",
            &[f64::from(self.altitude)],
        );
    }
}

#[derive(Default)]
struct Stats {
    ticks: u64,
    published: u64,
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

fn publish_scalars(viz: &VizPublisher, t: f64, entity: &str, values: &[f64]) {
    let mut msg = ScalarMessage::base(t, entity);
    let count = values.len().min(SCALARS_MAX);
    msg.scalars[..count].copy_from_slice(&values[..count]);
    msg.scalar_count = count as u32;
    let _ = viz.publish(msg);
}

#[allow(clippy::too_many_lines)] // 进程生命周期与控制节拍编排
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Arguments::parse();
    let config_path = args.config.unwrap_or_else(default_config);
    let cfg = firefly_config::startup::<config::PilotConfig>(
        &config_path,
        firefly_observability::init,
        config::PilotConfig::validate,
    )?;
    let period = Duration::from_secs_f64(1.0 / cfg.rate_hz);
    let dt = (1.0 / cfg.rate_hz) as f32;

    // 回调只写标志：信号上下文里不做日志/分配（异步信号安全）。
    let _signal_guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_signal| {
            RUNNING.store(false, Ordering::Relaxed);
        },
    )?;

    if !args.simulation {
        return usb::run(&cfg.device);
    }
    let node = create_node()?;
    let ports = Ports::open(&node)?;
    let log_ipc = firefly_observability::init_ipc(&node, "pilot");
    log::info!(
        "飞控进程启动：{} Hz 控制环，配置 {}；初始 DISARMED，等 `Firefly/Command`",
        cfg.rate_hz,
        config_path.display()
    );

    let imu_worker = imu::Worker::spawn(cfg.imu)?;
    let mut imu_count = 0u64;
    let mut inputs = Inputs::default();
    let mut controller = Controller::new(cfg.fsm, cfg.control);
    controller.team = firefly_config::game::load()?.team.code();
    let mut stats = Stats::default();
    let mut window_start = Instant::now();
    let mut sim_time = 0.0f64;
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
        controller.step(&ports, &inputs, dt, &mut stats)?;
        imu_worker.grounded.store(
            controller.fsm.state() == firefly_flight::FlightState::Disarmed,
            Ordering::Relaxed,
        );
        firefly_observability::pump_log_ipc(&log_ipc);
        stats.ticks += 1;
        pace(&mut next, period, &mut stats);

        let window = window_start.elapsed();
        if window >= Duration::from_secs_f64(STATS_PERIOD) {
            let root = Span::root("pilot-stats", SpanContext::random().sampled(false));
            let _guard = root.set_local_parent();
            log::info!(
                "t={:.2} 状态 {}（高度 {:.2} m），指令 {} 条（{:.0} Hz），tick {}，晚到 {:.1}%（最大 {:.2} ms），IMU {:.0} Hz，姿态源 {}",
                sim_time,
                controller.fsm.state().name(),
                controller.altitude,
                stats.published,
                stats.published as f64 / window.as_secs_f64(),
                stats.ticks,
                stats.late as f64 / stats.ticks.max(1) as f64 * 100.0,
                stats.max_late.as_secs_f64() * 1e3,
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
                    "fc/standby_eskf/health",
                    &[f64::from(s.ready)],
                );
                publish_scalars(
                    &ports.viz,
                    sim_time,
                    "fc/standby_eskf/imu_age_s",
                    &[s.received.elapsed().as_secs_f64()],
                );
                publish_scalars(
                    &ports.viz,
                    sim_time,
                    "fc/standby_eskf/accel_gate",
                    &[f64::from(s.accel_accepted)],
                );
                publish_scalars(&ports.viz, sim_time, "fc/standby_eskf/accel_nis", &[s.nis]);
                publish_scalars(&ports.viz, sim_time, "fc/standby_eskf/gyro_bias", &s.bias);
                publish_scalars(
                    &ports.viz,
                    sim_time,
                    "fc/standby_eskf/quaternion_xyzw",
                    s.rotation.coords.map(f64::from).as_slice(),
                );
                publish_scalars(
                    &ports.viz,
                    sim_time,
                    "fc/standby_eskf/attitude_variance",
                    &s.covariance_diagonal[..3],
                );
                publish_scalars(
                    &ports.viz,
                    sim_time,
                    "fc/standby_eskf/bias_variance",
                    &s.covariance_diagonal[3..],
                );
            }
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
    fn received_commands_decode_their_payloads() {
        for (kind, expected) in [
            (command_kind::ARM, Ok(Command::Arm)),
            (command_kind::DISARM, Ok(Command::Disarm)),
            (
                command_kind::TAKEOFF,
                Ok(Command::Takeoff { altitude: 1.75 }),
            ),
            (command_kind::HOLD, Ok(Command::Hold)),
            (command_kind::TRACK, Ok(Command::Track)),
            (command_kind::LAND, Ok(Command::Land)),
            (command_kind::PRECISION_LAND, Ok(Command::PrecisionLand)),
            (255, Err(255)),
        ] {
            let received = ReceivedCommand::from(&CommandMessage {
                kind,
                sequence: 42,
                altitude: 1.75,
                ..Default::default()
            });
            assert_eq!(received.sequence, 42);
            assert_eq!(received.command, expected);
        }
    }

    #[test]
    fn newest_unknown_command_supersedes_pending_and_replayed_commands() {
        let mut controller = Controller::new(FsmParams::default(), Px4Params::default());
        let mut inputs = sensor_inputs();
        for (sequence, kind) in [
            (1, command_kind::ARM),
            (2, 255),
            (1, command_kind::ARM),
            (2, command_kind::ARM),
        ] {
            inputs.receive_command(&CommandMessage {
                sequence,
                kind,
                ..Default::default()
            });
        }
        controller.demand(&inputs, 0.001);
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::Disarmed
        );
        assert!(!controller.commands.is_new(2));
        inputs.receive_command(&CommandMessage {
            sequence: 3,
            kind: command_kind::ARM,
            ..Default::default()
        });
        controller.demand(&inputs, 0.001);
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::ArmedGrounded
        );
    }

    #[test]
    fn command_sequences_are_consumed_once_even_at_the_limit() {
        let mut sequence = super::CommandSequence::default();
        assert!(!sequence.consume(0));
        assert!(sequence.consume(3));
        assert!(!sequence.consume(3));
        assert!(!sequence.consume(2));
        assert!(sequence.consume(u64::MAX));
        assert!(!sequence.consume(u64::MAX));
        assert!(!sequence.consume(1));
    }

    #[test]
    fn stale_grounded_estimator_cannot_start_hover_thrust() {
        let mut controller = Controller::new(FsmParams::default(), Px4Params::default());
        let mut inputs = sensor_inputs();
        inputs.command = Some(ReceivedCommand {
            command: Ok(Command::Arm),
            sequence: 1,
        });
        controller.demand(&inputs, 0.001);
        inputs.estimate.as_mut().unwrap().initialized = false;
        assert_eq!(
            controller
                .demand(&inputs, 0.001)
                .and_then(AngleDemand::channels),
            None
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
                specific_force: f64::from(firefly_flight::G),
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
    fn contact_support_requires_fresh_gravity_force_and_low_rotation() {
        let mut inputs = sensor_inputs();
        assert!(health_of(&inputs).landing_motion_low);
        inputs.attitude.as_mut().unwrap().specific_force = 0.;
        assert!(!health_of(&inputs).landing_motion_low);
        inputs.attitude.as_mut().unwrap().specific_force = f64::from(firefly_flight::G);
        inputs.attitude.as_mut().unwrap().rate.x = 0.3;
        assert!(!health_of(&inputs).landing_motion_low);
        inputs.attitude.as_mut().unwrap().rate.x = 0.;
        inputs.attitude.as_mut().unwrap().received -= IMU_STALE_LIMIT * 2;
        assert!(!health_of(&inputs).landing_motion_low);
    }

    #[test]
    fn truth_cannot_initialize_or_arm_controller() {
        let inputs = Inputs {
            plant: Some((PlantStateMessage::default(), Instant::now())),
            vehicle: Some((QuadParams::default(), Airframe::default())),
            command: Some(ReceivedCommand {
                command: Ok(Command::Arm),
                sequence: 1,
            }),
            ..Inputs::default()
        };
        let mut controller = Controller::new(FsmParams::default(), Px4Params::default());
        assert!(controller.demand(&inputs, 0.001).is_none());
        assert!(!controller.attitude_ready);
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::Disarmed
        );
    }

    #[test]
    fn changing_truth_cannot_change_demand() {
        let mut inputs = sensor_inputs();
        let mut a = Controller::new(FsmParams::default(), Px4Params::default());
        let mut b = Controller::new(FsmParams::default(), Px4Params::default());
        for _ in 0..1000 {
            a.demand(&inputs, 0.001);
            b.demand(&inputs, 0.001);
        }
        for (sequence, command) in [(1, Command::Arm), (2, Command::Takeoff { altitude: 1.0 })] {
            inputs.command = Some(ReceivedCommand {
                sequence,
                command: Ok(command),
            });
            inputs.plant = None;
            let left = a.demand(&inputs, 0.001).and_then(AngleDemand::channels);
            inputs.plant = Some((
                PlantStateMessage {
                    position_x: 1000.0,
                    velocity_z: -100.0,
                    quat_w: f64::NAN,
                    ..PlantStateMessage::default()
                },
                Instant::now(),
            ));
            let right = b.demand(&inputs, 0.001).and_then(AngleDemand::channels);
            assert_eq!(left, right);
        }
        assert_eq!(a.fsm.state(), firefly_flight::FlightState::Takeoff);
        assert!(a.demand(&inputs, 0.001).unwrap().thrust > 0.);
        inputs.estimate.as_mut().unwrap().received_at =
            Instant::now().checked_sub(Duration::from_secs(2)).unwrap();
        let fallback = a.demand(&inputs, 0.001).unwrap();
        assert!(fallback.thrust > 0. && fallback.channels().is_some());
        assert_eq!(a.fsm.state(), firefly_flight::FlightState::AttitudeFallback);
        let estimate = inputs.estimate.as_mut().unwrap();
        estimate.position = Vector3::repeat(10000.);
        estimate.velocity = Vector3::repeat(-1000.);
        let same = a.demand(&inputs, 0.001).unwrap();
        assert_eq!(fallback.channels(), same.channels());
    }
    fn airborne(inputs: &mut Inputs) -> Controller {
        let mut controller = Controller::new(FsmParams::default(), Px4Params::default());
        for (sequence, command) in [(1, Command::Arm), (2, Command::Takeoff { altitude: 1.0 })] {
            inputs.command = Some(ReceivedCommand {
                sequence,
                command: Ok(command),
            });
            controller.demand(inputs, 0.001);
        }
        inputs.estimate.as_mut().unwrap().position.z = 1.;
        controller
    }

    #[test]
    fn primary_is_selected_and_loss_latches_independent_standby_until_command() {
        let mut inputs = sensor_inputs();
        let mut controller = airborne(&mut inputs);
        inputs.attitude.as_mut().unwrap().rotation = UnitQuaternion::from_euler_angles(0., 0., 1.8);
        controller.demand(&inputs, 0.001);
        assert_eq!(controller.source, attitude::Source::Vio);
        assert!(controller.attitude.angle() < 1e-6);
        inputs.primary = attitude::Input::default();
        let demand = controller.demand(&inputs, 0.001).unwrap();
        assert_eq!(controller.source, attitude::Source::Standby);
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::AttitudeFallback
        );
        assert!((controller.attitude.angle() - 1.8).abs() < 1e-6);
        assert!(demand.thrust > 0.);
        assert_eq!((demand.roll, demand.pitch), (0., 0.));
        inputs.primary = sensor_inputs().primary;
        controller.demand(&inputs, 0.001);
        assert_eq!(controller.source, attitude::Source::Standby);
        inputs.command = Some(ReceivedCommand {
            command: Ok(Command::Hold),
            sequence: 3,
        });
        controller.demand(&inputs, 0.001);
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
        inputs.command = Some(ReceivedCommand {
            command: Ok(Command::Hold),
            sequence: 3,
        });
        controller.demand(&inputs, 0.001);
        assert_eq!(controller.source, attitude::Source::Standby);
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::AttitudeFallback
        );
        inputs.attitude.as_mut().unwrap().received =
            Instant::now().checked_sub(Duration::from_secs(2)).unwrap();
        assert_eq!(
            controller
                .demand(&inputs, 0.001)
                .and_then(AngleDemand::channels),
            None
        );
        assert_eq!(controller.source, attitude::Source::Unavailable);
    }

    #[test]
    fn arm_requires_ready_standby_even_with_valid_vio() {
        let mut inputs = sensor_inputs();
        inputs.attitude.as_mut().unwrap().ready = false;
        inputs.command = Some(ReceivedCommand {
            command: Ok(Command::Arm),
            sequence: 1,
        });
        let mut controller = Controller::new(FsmParams::default(), Px4Params::default());
        assert_eq!(
            controller
                .demand(&inputs, 0.001)
                .and_then(AngleDemand::channels),
            None
        );
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::Disarmed
        );
    }
}
