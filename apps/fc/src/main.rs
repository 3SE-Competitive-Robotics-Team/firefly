//! 飞控进程：1kHz 控制环（`firefly-flight`）+ iceoryx2。
//!
//! 输入：
//! - `Firefly/PlantState`（真值，200Hz）——仅供倾斜误差评测。
//! - `Firefly/Airframe`（质量/惯量/旋翼几何/电机上限，1Hz 电平）——飞控参数的唯一来源；
//! - `Firefly/Odometry`——连续 odom 系位置/速度/航向反馈；
//! - `Firefly/CorrectedOdometry`——同时间戳配对建立 map←odom，转换地图参考；
//! - `Firefly/Imu`（陀螺 + 加计，100Hz）——姿态估计（内环）；
//! - `Firefly/Reference`（规划参考：位置/速度/偏航/偏航角速度）；
//! - `Firefly/Command`（地面站指令：解锁/上锁/起飞/保持/跟踪/降落）。
//!
//! 输出：`Firefly/Control`（4 电机推力，1kHz；被控对象每步取最新）——**每个** tick 都发，
//! 上锁时发零推力：被控对象按指令新鲜度决定物理是否推进，飞控停发等于世界停转。
//!
//! 模式与安全层是 [`FlightFsm`]：它输出"本 tick 该飞的参考 + 电机是否使能"，
//! 本进程只负责把估计状态与健康电平喂进去、把输出接给位置模式。
//!
//! 反馈来源分工：**内环姿态由飞控自估**（陀螺积分 + 加速度计水平修正，航向取 VIO，
//! 上电用加计定滚转/俯仰）；位置/速度来自里程计。缺少有效 IMU、初始化未完成
//! 或地面里程计陈旧时发送零推力；空中里程计失联进入仅姿态稳定。真值不参与控制与健康判据。
//!
//! 控制节拍使用墙钟，诊断时间取自 IMU。未就绪时持续发零推力，使被控对象能产生初始化测量。
//!
//! 本进程不建逐 tick span（1kHz）：时延证据用 tick 统计进 rrd，
//! 跨进程时延用 Control 消息头里的发送时间戳（sim 侧测）。
//!
//! 运行：`cargo run --release -p fc`（配合 `uv run firefly-sim`）。

mod config;
mod frames;
mod vio;

use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use fastrace::prelude::*;
use firefly_flight::{
    Airframe, AttitudeEstimator, Command, ControlParams, Event, FlightFsm, FsmParams, Health,
    PositionSetpoint, QuadParams, QuadState, attitude_support, position_mode, yaw_of,
};
use firefly_pubsub::command::{COMMAND_TOPIC, CommandMessage, kind as command_kind};
use firefly_pubsub::control::{ControlMessage, ControlPublisher};
use firefly_pubsub::imu::ImuSubscriber;
use firefly_pubsub::node::{IpcNode, create_node};
use firefly_pubsub::plant::{
    AirframeMessage, AirframeSubscriber, PlantStateMessage, PlantStateSubscriber,
};
use firefly_pubsub::reference::{REFERENCE_TOPIC, ReferenceMessage};
use firefly_pubsub::subscriber::{CorrectedOdomSubscriber, OdomSubscriber, Subscriber};
use firefly_pubsub::viz::{SCALARS_MAX, VizMessage, VizPublisher, kind};
use glam::{Quat, Vec3};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalGuard, SignalHandler};

/// `configs/fc.toml` 缺省路径（相对运行目录，通常为仓库根）。
const DEFAULT_CONFIG: &str = "configs/fc.toml";
/// 节拍余量：睡眠到此为止、余下自旋（1kHz 下约 15% 单核，换 µs 级 tick 精度；
/// 纯 sleep 在 macOS 上会落到 ~1.2ms 周期）。
const SPIN_MARGIN: Duration = Duration::from_micros(150);
/// 里程计陈旧阈值（墙钟秒）：覆盖 10Hz 视觉更新间隔，触发姿态降级。
const ODOM_STALE_LIMIT: Duration = Duration::from_millis(500);
/// IMU 陈旧阈值（墙钟）：100Hz 下 5 个周期；陈旧则不得解锁。
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

/// 解析 `--config <path>`（缺省 [`DEFAULT_CONFIG`]）。
fn parse_config_path() -> Result<String, String> {
    let mut it = std::env::args().skip(1);
    let mut path = DEFAULT_CONFIG.to_owned();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--config" => {
                path = it
                    .next()
                    .ok_or_else(|| "missing --config value".to_owned())?;
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(path)
}

/// 连续 odom 系位置/速度/航向反馈快照，仅来自 `Firefly/Odometry`。
#[derive(Clone, Copy)]
struct Estimate {
    /// 状态的测量时间（秒）。
    timestamp: f64,
    /// 估计位置（m）。
    position: Vec3,
    /// 估计速度（m/s）。
    velocity: Vec3,
    /// JPL 世界→机体四元数 `[x, y, z, w]`（航向修正用）。
    quat: [f64; 4],
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
    /// 位置/速度/航向反馈。
    estimate: Option<Estimate>,
    frames: frames::ControlFrames,
    /// 最新 IMU 传感器时间（秒），作为控制与诊断时间轴。
    imu_time: f64,
    /// 最新 IMU（陀螺、加计，机体系）与到达墙钟时刻（判陈旧）。
    imu: Option<(Vec3, Vec3, Instant)>,
    /// 本 tick 排空的有效 IMU 样本，按测量时刻消费一次。
    imu_samples: Vec<(f64, Vec3, Vec3)>,
    /// 参考（位置/速度/偏航/偏航角速度）与到达墙钟时刻（判新鲜）。
    reference: Option<(ReferenceMessage, Instant)>,
    /// 最新地面站指令（序号最大的那条）。
    command: Option<CommandMessage>,
}

/// 端口集合（避免主循环参数爆炸）。
struct Ports {
    plant: PlantStateSubscriber,
    airframe: AirframeSubscriber,
    imu: ImuSubscriber,
    odom: OdomSubscriber,
    corrected: CorrectedOdomSubscriber,
    reference: Subscriber<ReferenceMessage>,
    command: Subscriber<CommandMessage>,
    control: ControlPublisher,
    viz: VizPublisher,
}

impl Ports {
    fn open(node: &IpcNode) -> Result<Self, firefly_error::Error> {
        Ok(Self {
            plant: PlantStateSubscriber::new(node)?,
            airframe: AirframeSubscriber::new(node)?,
            imu: ImuSubscriber::new(node)?,
            odom: OdomSubscriber::new(node)?,
            corrected: CorrectedOdomSubscriber::new(node)?,
            reference: Subscriber::with_topic(node, REFERENCE_TOPIC)?,
            command: Subscriber::with_topic(node, COMMAND_TOPIC)?,
            control: ControlPublisher::new(node)?,
            viz: VizPublisher::new(node)?,
        })
    }

    /// 可视化发布器（只读借用，供 [`Controller::publish_viz`]）。
    fn viz_ref(&self) -> &VizPublisher {
        &self.viz
    }

    /// 排空所有输入（非阻塞），保留最新；返回本次排空到的 IMU 样本数。
    #[allow(clippy::too_many_lines)]
    fn drain(&self, inputs: &mut Inputs) -> Result<u32, firefly_error::Error> {
        let mut imu_count = 0u32;
        inputs.imu_samples.clear();
        while let Some(sample) = self.plant.receive()? {
            inputs.plant = Some((*sample, Instant::now()));
        }
        while let Some(sample) = self.airframe.receive()? {
            inputs.vehicle = Some(airframe_to_params(&sample));
        }
        while let Some(sample) = self.imu.receive()? {
            let m = *sample;
            if ![
                m.timestamp,
                m.angular_velocity_x,
                m.angular_velocity_y,
                m.angular_velocity_z,
                m.linear_acceleration_x,
                m.linear_acceleration_y,
                m.linear_acceleration_z,
            ]
            .iter()
            .all(|x| x.is_finite())
            {
                continue;
            }
            if m.timestamp < 0. || (inputs.imu.is_some() && m.timestamp <= inputs.imu_time) {
                continue;
            }
            inputs.imu_time = m.timestamp;
            inputs.imu = Some((
                Vec3::new(
                    m.angular_velocity_x as f32,
                    m.angular_velocity_y as f32,
                    m.angular_velocity_z as f32,
                ),
                Vec3::new(
                    m.linear_acceleration_x as f32,
                    m.linear_acceleration_y as f32,
                    m.linear_acceleration_z as f32,
                ),
                Instant::now(),
            ));
            let (gyro, accel, _) = inputs.imu.unwrap();
            inputs.imu_samples.push((m.timestamp, gyro, accel));
            imu_count += 1;
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
            let position = Vec3::new(
                m.position_x as f32,
                m.position_y as f32,
                m.position_z as f32,
            );
            let velocity = Vec3::new(
                m.velocity_x as f32,
                m.velocity_y as f32,
                m.velocity_z as f32,
            );
            if !position.is_finite() || !velocity.is_finite() {
                continue;
            }
            inputs.estimate = Some(Estimate {
                timestamp: m.timestamp,
                position,
                velocity,
                quat: [m.quat_x, m.quat_y, m.quat_z, m.quat_w],
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
        Ok(imu_count)
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
        estimator_ready: inputs.estimate.is_some_and(|e| e.initialized),
        airframe_ready: inputs.vehicle.is_some(),
        odometry_alive: inputs
            .estimate
            .is_some_and(|e| e.received_at.elapsed() <= ODOM_STALE_LIMIT),
        imu_alive: inputs
            .imu
            .is_some_and(|(_, _, at)| at.elapsed() <= IMU_STALE_LIMIT),
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
    let now_sim = if inputs.imu.is_some() {
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
    /// 姿态估计（内环）。上电用加计定滚转/俯仰，航向由 VIO 慢修。
    estimator: AttitudeEstimator,
    /// 姿态估计是否已启动。
    attitude_ready: bool,
    /// 上次航向修正时刻（估算修正 dt）。
    last_yaw_fix: Option<f64>,
    last_imu_time: Option<f64>,
    /// 飞行状态机（模式与安全层）。
    fsm: FlightFsm,
    /// 已执行的最大指令序号（重复投递去重）。
    last_command_seq: u64,
    /// 相对起飞点高度（m；最近一 tick 的估计高度，进日志与 rrd）。
    altitude: f32,
    /// 上次可视化时刻（仿真时间）。
    last_viz: f64,
}

impl Controller {
    fn new(fsm: FsmParams) -> Self {
        Self {
            estimator: AttitudeEstimator::default(),
            attitude_ready: false,
            last_yaw_fix: None,
            last_imu_time: None,
            fsm: FlightFsm::new(fsm),
            last_command_seq: 0,
            altitude: 0.0,
            last_viz: f64::NEG_INFINITY,
        }
    }

    /// 空中失联只用 IMU 维持姿态支持；禁止陈旧位置进入控制律。
    fn motor_output(&mut self, inputs: &Inputs, dt: f32, ctl: &ControlParams) -> ([f32; 4], bool) {
        let (attitude, ang_vel) = self.attitude_and_rates(inputs, dt);
        let (position, velocity) = inputs
            .estimate
            .map_or((Vec3::ZERO, Vec3::ZERO), |est| (est.position, est.velocity));
        let state = QuadState {
            position,
            velocity,
            attitude,
            ang_vel,
        };
        let health = health_of(inputs);
        self.apply_command(inputs.command, health, &state);
        let reference = fresh_reference(inputs);
        let out = self.fsm.update(dt, &state, reference.as_ref(), health);
        report_event(out.event);
        self.altitude = self.fsm.altitude_above_origin(&state);
        if !out.motors_enabled || !health.imu_alive || !self.attitude_ready {
            return ([0.0; 4], false);
        }
        let Some((quad, airframe)) = inputs.vehicle else {
            return ([0.0; 4], false);
        };
        let desired = if self.fsm.state() == firefly_flight::FlightState::AttitudeFallback {
            attitude_support(attitude, ang_vel, out.setpoint.yaw, &quad, ctl)
        } else {
            if !health.estimator_ready || !health.odometry_alive {
                return ([0.; 4], false);
            }
            position_mode(&state, &out.setpoint, &quad, ctl)
        };
        let allocation = airframe.allocate(attitude, &desired);
        (allocation.motors, allocation.saturated)
    }

    /// 每 tick 发布控制；启动等待期的零推力允许仿真推进并产生初始化测量。
    fn step(
        &mut self,
        ports: &Ports,
        inputs: &Inputs,
        dt: f32,
        ctl: &ControlParams,
        tick: u64,
        stats: &mut Stats,
    ) -> Result<(), firefly_error::Error> {
        let (motors, saturated) = self.motor_output(inputs, dt, ctl);
        stats.saturated += u64::from(saturated);
        ports.control.publish(ControlMessage {
            state_time: inputs.imu_time,
            thrust: motors.map(f64::from),
            tick,
        })?;
        stats.published += 1;
        let truth = inputs.plant.map(|(plant, _)| {
            Quat::from_xyzw(
                plant.quat_x as f32,
                plant.quat_y as f32,
                plant.quat_z as f32,
                plant.quat_w as f32,
            )
            .normalize()
        });
        self.publish_viz(
            ports,
            inputs.imu_time,
            self.estimator.attitude(),
            truth,
            &motors,
        );
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
            Err(reject) => log::warn!("指令 {cmd:?} 被拒：{}", reject.reason()),
        }
    }

    /// 姿态与角速度仅由 IMU 和里程计估计；无 IMU 时保持未就绪状态。
    fn attitude_and_rates(&mut self, inputs: &Inputs, _dt: f32) -> (Quat, Vec3) {
        let Some((gyro, accel, at)) = inputs.imu else {
            return (self.estimator.attitude(), Vec3::ZERO);
        };
        if at.elapsed() > IMU_STALE_LIMIT {
            return (self.estimator.attitude(), Vec3::ZERO);
        }
        if !self.attitude_ready {
            self.estimator = AttitudeEstimator::from_accel(accel);
            self.attitude_ready = true;
            log::info!(
                "姿态估计启动：加计定滚转/俯仰（{:.1}°），航向等 VIO 修正",
                (self.estimator.attitude() * Vec3::Z)
                    .z
                    .clamp(-1.0, 1.0)
                    .acos()
                    .to_degrees()
            );
        }
        if inputs.imu_samples.is_empty() {
            self.consume_imu(inputs.imu_time, gyro, accel);
        } else {
            for &(time, gyro, accel) in &inputs.imu_samples {
                self.consume_imu(time, gyro, accel);
            }
        }
        if let Some(est) = inputs
            .estimate
            .filter(|e| e.initialized && e.received_at.elapsed() <= ODOM_STALE_LIMIT)
        {
            if self.last_yaw_fix.is_none() {
                self.estimator.reset(vio::body_to_world_from_odom(est.quat));
                self.last_yaw_fix = Some(est.timestamp);
            } else if let Some(previous) = self.last_yaw_fix.filter(|t| est.timestamp > *t) {
                let yaw_src = yaw_of(vio::body_to_world_from_odom(est.quat));
                self.estimator
                    .correct_yaw(yaw_src, (est.timestamp - previous).min(0.5) as f32);
                self.last_yaw_fix = Some(est.timestamp);
            }
        }
        (self.estimator.attitude(), gyro)
    }

    /// 测量时钟闭合：重复/乱序数据不积分，暂停传感器时间不推进姿态。
    fn consume_imu(&mut self, timestamp: f64, gyro: Vec3, accel: Vec3) {
        if !timestamp.is_finite() {
            return;
        }
        if let Some(previous) = self.last_imu_time {
            if timestamp <= previous {
                return;
            }
            self.estimator
                .update(gyro, accel, (timestamp - previous) as f32);
        }
        self.last_imu_time = Some(timestamp);
    }

    /// 控制量 + 模式 + 姿态校验进 rrd（10Hz）。
    fn publish_viz(
        &mut self,
        ports: &Ports,
        sim_time: f64,
        attitude: Quat,
        plant_attitude: Option<Quat>,
        motors: &[f32; 4],
    ) {
        if sim_time - self.last_viz < VIZ_PERIOD {
            return;
        }
        self.last_viz = sim_time;
        let total: f32 = motors.iter().sum();
        let tilt = (attitude * Vec3::Z).z.clamp(-1.0, 1.0).acos().to_degrees();
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
        if let Some(truth) = plant_attitude {
            // 重力在机体系中的方向不依赖局部航向，可直接与真值比较。
            let error = (attitude.inverse() * Vec3::Z)
                .angle_between(truth.inverse() * Vec3::Z)
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
    let mut msg = VizMessage::base(kind::SCALARS, t, entity);
    let count = values.len().min(SCALARS_MAX);
    msg.scalars[..count].copy_from_slice(&values[..count]);
    msg.scalar_count = count as u32;
    let _ = viz.publish(msg);
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    let config_path = parse_config_path().map_err(|e| {
        eprintln!("{e}\n用法：fc [--config configs/fc.toml]");
        std::process::exit(2);
    })?;
    let cfg = config::load(&config_path);
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
        "飞控进程启动：{} Hz 控制环，配置 {config_path}；初始 DISARMED，等 `Firefly/Command`",
        cfg.rate_hz
    );

    let mut inputs = Inputs::default();
    let mut controller = Controller::new(cfg.fsm);
    let mut stats = Stats::default();
    let mut window_start = Instant::now();
    let mut sim_time = 0.0f64;
    let mut tick = 0u64;
    let mut next = Instant::now() + period;

    while RUNNING.load(Ordering::Relaxed) {
        stats.imu += u64::from(ports.drain(&mut inputs)?);
        sim_time = inputs.imu_time;
        controller.step(&ports, &inputs, dt, &cfg.control, tick, &mut stats)?;
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
                if controller.attitude_ready {
                    "自估"
                } else {
                    "等待 IMU"
                },
            );
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
    fn imu_integrates_measurement_time_once_even_when_control_ticks_repeat() {
        let mut controller = Controller::new(FsmParams::default());
        for i in 0..=100 {
            let time = f64::from(i) * 0.01;
            controller.consume_imu(time, Vec3::Z, Vec3::Z * firefly_flight::G);
            for _ in 0..20 {
                controller.consume_imu(time, Vec3::Z, Vec3::Z * firefly_flight::G);
            }
        }
        assert!((controller.estimator.yaw() - 1.).abs() < 1e-5);
        controller.consume_imu(0.5, Vec3::Z * 100., Vec3::ZERO);
        assert!((controller.estimator.yaw() - 1.).abs() < 1e-5);
    }

    #[test]
    fn stale_grounded_estimator_cannot_start_hover_thrust() {
        let mut controller = Controller::new(FsmParams::default());
        let mut inputs = sensor_inputs();
        inputs.command = Some(CommandMessage {
            kind: command_kind::ARM,
            sequence: 1,
            ..Default::default()
        });
        controller.motor_output(&inputs, 0.001, &ControlParams::default());
        inputs.estimate.as_mut().unwrap().initialized = false;
        assert_eq!(
            controller
                .motor_output(&inputs, 0.001, &ControlParams::default())
                .0
                .map(f32::to_bits),
            [0; 4]
        );
        assert_eq!(
            controller.fsm.state(),
            firefly_flight::FlightState::Disarmed
        );
    }

    fn sensor_inputs() -> Inputs {
        Inputs {
            vehicle: Some((QuadParams::default(), Airframe::default())),
            estimate: Some(Estimate {
                timestamp: 0.,
                position: Vec3::ZERO,
                velocity: Vec3::ZERO,
                quat: [0.0, 0.0, 0.0, 1.0],
                initialized: true,
                received_at: Instant::now(),
            }),
            imu: Some((Vec3::ZERO, Vec3::Z * firefly_flight::G, Instant::now())),
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
        let mut controller = Controller::new(FsmParams::default());
        let (motors, _) = controller.motor_output(&inputs, 0.001, &ControlParams::default());
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
        let mut a = Controller::new(FsmParams::default());
        let mut b = Controller::new(FsmParams::default());
        let ctl = ControlParams::default();
        for _ in 0..1000 {
            a.motor_output(&inputs, 0.001, &ctl);
            b.motor_output(&inputs, 0.001, &ctl);
        }
        for (kind, sequence) in [(command_kind::ARM, 1), (command_kind::TAKEOFF, 2)] {
            inputs.command = Some(CommandMessage {
                kind,
                sequence,
                altitude: 1.0,
                ..CommandMessage::default()
            });
            inputs.plant = None;
            let (left, _) = a.motor_output(&inputs, 0.001, &ctl);
            inputs.plant = Some((
                PlantStateMessage {
                    position_x: 1000.0,
                    velocity_z: -100.0,
                    quat_w: f64::NAN,
                    ..PlantStateMessage::default()
                },
                Instant::now(),
            ));
            let (right, _) = b.motor_output(&inputs, 0.001, &ctl);
            assert_eq!(left.map(f32::to_bits), right.map(f32::to_bits));
        }
        assert_eq!(a.fsm.state(), firefly_flight::FlightState::Takeoff);
        assert!(
            a.motor_output(&inputs, 0.001, &ctl)
                .0
                .iter()
                .any(|m| *m > 0.0)
        );
        inputs.estimate.as_mut().unwrap().received_at =
            Instant::now().checked_sub(Duration::from_secs(2)).unwrap();
        let (fallback, _) = a.motor_output(&inputs, 0.001, &ctl);
        assert!(fallback.iter().all(|m| m.is_finite() && *m > 0.));
        assert_eq!(a.fsm.state(), firefly_flight::FlightState::AttitudeFallback);
        let estimate = inputs.estimate.as_mut().unwrap();
        estimate.position = Vec3::splat(10000.);
        estimate.velocity = Vec3::splat(-1000.);
        let (same, _) = a.motor_output(&inputs, 0.001, &ctl);
        assert_eq!(fallback.map(f32::to_bits), same.map(f32::to_bits));
    }
}
