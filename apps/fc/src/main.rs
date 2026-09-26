//! 飞控进程：1kHz 控制环（`firefly-flight`）+ iceoryx2。
//!
//! 输入：
//! - `Firefly/PlantState`（被控对象真值状态，200Hz）——位置/速度回落源 + 姿态校验；
//! - `Firefly/Airframe`（质量/惯量/旋翼几何/电机上限，1Hz 电平）——飞控参数的唯一来源；
//! - `Firefly/Odometry` / `Firefly/CorrectedOdometry`——位置/速度/航向反馈（取消息新的）；
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
//! 上电用加计定滚转/俯仰）；位置/速度取 VIO 估计（飞控只看得到估计），估计未到达时
//! 回落被控对象真值（解锁前正常）。被控对象状态另作姿态误差校验（进 rrd）。
//!
//! 节拍：控制律无积分项，dt 不进控制，因而 tick 只需节拍正确、不需要硬实时时钟；
//! 落后节拍计数并进日志（唯一证据来源）。被控对象状态陈旧（>50ms 未更新）时
//! **不发指令**——被控对象侧同样不推进物理，绝不用陈旧状态算控制。
//!
//! 本进程不建逐 tick span（1kHz）：时延证据用 tick 统计进 rrd，
//! 跨进程时延用 Control 消息头里的发送时间戳（sim 侧测）。
//!
//! 运行：`cargo run --release -p fc`（配合 `uv run firefly-sim`）。

mod config;
mod vio;

use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use fastrace::prelude::*;
use firefly_flight::{
    Airframe, AttitudeEstimator, Command, ControlParams, Event, FlightFsm, FsmParams, Health,
    PositionSetpoint, QuadParams, QuadState, position_mode, yaw_of,
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
/// 被控对象状态陈旧阈值（墙钟）：超过即停发指令（被控对象侧同样不推进物理）。
const STALE_LIMIT: Duration = Duration::from_millis(50);
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

/// 位置/速度/航向反馈快照（`Firefly/Odometry` / `Firefly/CorrectedOdometry` 取消息新的）。
#[derive(Clone, Copy)]
struct Estimate {
    /// 估计器时间戳（仿真秒；仅用于取最新样本）。
    time: f64,
    /// 估计位置（m）。
    position: Vec3,
    /// 估计速度（m/s）。
    velocity: Vec3,
    /// JPL 机体→世界四元数 `[x, y, z, w]`（航向修正用）。
    quat: [f64; 4],
    /// 估计器已初始化（解锁前置条件之一）。
    initialized: bool,
}

/// 运行期输入快照（每 tick 排空到最新）。
#[derive(Default)]
struct Inputs {
    /// 被控对象状态与到达墙钟时刻（判陈旧）。
    plant: Option<(PlantStateMessage, Instant)>,
    /// 机体/执行器（装配成 `firefly-flight` 的参数）。
    vehicle: Option<(QuadParams, Airframe)>,
    /// 位置/速度/航向反馈。
    estimate: Option<Estimate>,
    /// 最新 IMU（陀螺、加计，机体系）与到达墙钟时刻（判陈旧）。
    imu: Option<(Vec3, Vec3, Instant)>,
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
    fn drain(&self, inputs: &mut Inputs) -> Result<u32, firefly_error::Error> {
        let mut imu_count = 0u32;
        while let Some(sample) = self.plant.receive()? {
            inputs.plant = Some((*sample, Instant::now()));
        }
        while let Some(sample) = self.airframe.receive()? {
            inputs.vehicle = Some(airframe_to_params(&sample));
        }
        while let Some(sample) = self.imu.receive()? {
            let m = *sample;
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
            imu_count += 1;
        }
        // 位置/速度/姿态反馈取原始与校正估计中消息更新的那个（与 planner 同规则）
        let mut newest: Option<Estimate> = None;
        for sample in [self.odom.receive()?, self.corrected.receive()?]
            .into_iter()
            .flatten()
        {
            let m = *sample;
            let candidate = Estimate {
                time: m.timestamp,
                position: Vec3::new(
                    m.position_x as f32,
                    m.position_y as f32,
                    m.position_z as f32,
                ),
                velocity: Vec3::new(
                    m.velocity_x as f32,
                    m.velocity_y as f32,
                    m.velocity_z as f32,
                ),
                quat: [m.quat_x, m.quat_y, m.quat_z, m.quat_w],
                initialized: m.is_initialized,
            };
            if newest.is_none_or(|cur| candidate.time > cur.time) {
                newest = Some(candidate);
            }
        }
        if let Some(candidate) = newest
            && inputs.estimate.is_none_or(|cur| candidate.time > cur.time)
        {
            inputs.estimate = Some(candidate);
        }
        while let Some(sample) = self.reference.receive()? {
            inputs.reference = Some((*sample, Instant::now()));
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
fn health_of(inputs: &Inputs, plant_at: Instant) -> Health {
    Health {
        estimator_ready: inputs.estimate.is_some_and(|e| e.initialized),
        airframe_ready: inputs.vehicle.is_some(),
        plant_alive: plant_at.elapsed() <= STALE_LIMIT,
        imu_alive: inputs
            .imu
            .is_some_and(|(_, _, at)| at.elapsed() <= IMU_STALE_LIMIT),
    }
}

/// 参考流新鲜时转成位置模式参考；陈旧样本按不存在处理——断开时长由状态机计时
/// （状态机只在 `Track` 下消它，其余模式自己生参考）。
fn fresh_reference(inputs: &Inputs) -> Option<PositionSetpoint> {
    let (msg, at) = inputs.reference?;
    if at.elapsed() > REFERENCE_STALE_LIMIT {
        return None;
    }
    Some(PositionSetpoint {
        position: Vec3::new(
            msg.position_x as f32,
            msg.position_y as f32,
            msg.position_z as f32,
        ),
        velocity: Vec3::new(
            msg.velocity_x as f32,
            msg.velocity_y as f32,
            msg.velocity_z as f32,
        ),
        yaw: msg.yaw as f32,
        yaw_rate: msg.yaw_dot as f32,
    })
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
            log::error!("失效保护：状态估计丢失 → 自动降落");
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
    last_yaw_fix: Option<Instant>,
    /// 飞行状态机（模式与安全层）。
    fsm: FlightFsm,
    /// 已执行的最大指令序号（重复投递去重）。
    last_command_seq: u64,
    /// 相对起飞点高度（m；最近一 tick 的估计高度，进日志与 rrd）。
    altitude: f32,
    /// 一次性告警标记。
    warned: Warned,
    /// 上次可视化时刻（仿真时间）。
    last_viz: f64,
}

/// 一次性告警标记（每类只发一条，条件恢复时复位）。
#[derive(Default)]
struct Warned {
    /// 位置反馈回落真值（估计未到达）。
    truth_fallback: bool,
    /// 被控对象状态陈旧。
    plant_stale: bool,
    /// 机体描述缺席。
    no_airframe: bool,
}

impl Controller {
    fn new(fsm: FsmParams) -> Self {
        Self {
            estimator: AttitudeEstimator::default(),
            attitude_ready: false,
            last_yaw_fix: None,
            fsm: FlightFsm::new(fsm),
            last_command_seq: 0,
            altitude: 0.0,
            warned: Warned::default(),
            last_viz: f64::NEG_INFINITY,
        }
    }

    /// 一步：姿态估计 → 位置/速度反馈 → 指令 → 状态机 → 位置模式 → 分配 → 发布指令。
    ///
    /// **每个** tick 都发指令（缺参数/上锁时发零推力）：被控对象按指令新鲜度决定
    /// 物理是否推进，飞控停发即世界停转——飞控绝不能是要被控对象等的乙方。
    /// 无被控对象状态（未启动或陈旧）时不发：状态算不出控制，被控对象侧也本就停着。
    fn step(
        &mut self,
        ports: &Ports,
        inputs: &Inputs,
        dt: f32,
        ctl: &ControlParams,
        tick: u64,
        stats: &mut Stats,
    ) -> Result<(), firefly_error::Error> {
        let Some((plant, received_at)) = inputs.plant else {
            return Ok(());
        };
        if received_at.elapsed() > STALE_LIMIT {
            if !self.warned.plant_stale {
                self.warned.plant_stale = true;
                log::error!("被控对象状态陈旧：停发指令（物理不推进），等被控对象恢复");
            }
            return Ok(());
        }
        self.warned.plant_stale = false;

        // 机体描述（质量/惯量/旋翼几何/电机上限）是参数的唯一来源，缺它就不能算推力
        if inputs.vehicle.is_none() {
            if !self.warned.no_airframe {
                self.warned.no_airframe = true;
                log::warn!("尚未收到机体描述（Firefly/Airframe），暂发零推力");
            }
        } else {
            self.warned.no_airframe = false;
        }

        let plant_attitude = Quat::from_xyzw(
            plant.quat_x as f32,
            plant.quat_y as f32,
            plant.quat_z as f32,
            plant.quat_w as f32,
        )
        .normalize();
        let (attitude, ang_vel) = self.attitude_and_rates(inputs, &plant, plant_attitude, dt);
        let (position, velocity) = self.feedback(inputs, &plant);
        let quad_state = QuadState {
            position,
            velocity,
            attitude,
            ang_vel,
        };

        // ---- 模式与安全层：指令 → 健康电平 → 本 tick 该飞的参考 ----
        let health = health_of(inputs, received_at);
        self.apply_command(inputs.command, health, &quad_state);
        let reference = fresh_reference(inputs);
        let out = self.fsm.update(dt, &quad_state, reference.as_ref(), health);
        report_event(out.event);
        self.altitude = self.fsm.altitude_above_origin(&quad_state);

        // ---- 控制：位置模式 → 期望 wrench → 4 电机分配（唯一饱和点） ----
        // 上锁发零推力，但**每条 tick 都发**：被控对象按指令新鲜度决定物理是否推进。
        let motors = match (inputs.vehicle, out.motors_enabled) {
            (Some((quad, airframe)), true) => {
                let desired = position_mode(&quad_state, &out.setpoint, &quad, ctl);
                let allocation = airframe.allocate(attitude, &desired);
                if allocation.saturated {
                    stats.saturated += 1;
                }
                allocation.motors
            }
            _ => [0.0; 4],
        };
        ports.control.publish(ControlMessage {
            state_time: plant.timestamp,
            thrust: motors.map(f64::from),
            tick,
        })?;
        stats.published += 1;
        self.publish_viz(ports, plant.timestamp, attitude, plant_attitude, &motors);
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

    /// 姿态（内环）与机体系角速度：飞控自估优先（陀螺积分 + 加计修正 + VIO 航向），
    /// 无 IMU 时回落被控对象真值（无传感器的最小可用路径）。
    fn attitude_and_rates(
        &mut self,
        inputs: &Inputs,
        plant: &PlantStateMessage,
        plant_attitude: Quat,
        dt: f32,
    ) -> (Quat, Vec3) {
        let Some((gyro, accel, _)) = inputs.imu else {
            return (
                plant_attitude,
                Vec3::new(
                    plant.angular_velocity_x as f32,
                    plant.angular_velocity_y as f32,
                    plant.angular_velocity_z as f32,
                ),
            );
        };
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
        self.estimator.update(gyro, accel, dt);
        if let Some(est) = inputs.estimate {
            let yaw_src = yaw_of(vio::body_to_world_from_odom(est.quat));
            let dt_fix = self
                .last_yaw_fix
                .map_or(dt, |t| t.elapsed().as_secs_f32().clamp(dt, 0.5));
            self.estimator.correct_yaw(yaw_src, dt_fix);
            self.last_yaw_fix = Some(Instant::now());
        }
        (self.estimator.attitude(), gyro)
    }

    /// 位置/速度反馈：VIO 估计优先，未到达回落被控对象真值（解锁前正常，告警一次）。
    fn feedback(&mut self, inputs: &Inputs, plant: &PlantStateMessage) -> (Vec3, Vec3) {
        if let Some(est) = inputs.estimate {
            return (est.position, est.velocity);
        }
        if !self.warned.truth_fallback {
            self.warned.truth_fallback = true;
            log::warn!("状态估计尚未到达，位置反馈暂回落被控对象真值（起飞前正常）");
        }
        (
            Vec3::new(
                plant.position_x as f32,
                plant.position_y as f32,
                plant.position_z as f32,
            ),
            Vec3::new(
                plant.velocity_x as f32,
                plant.velocity_y as f32,
                plant.velocity_z as f32,
            ),
        )
    }

    /// 控制量 + 模式 + 姿态校验进 rrd（10Hz）。
    fn publish_viz(
        &mut self,
        ports: &Ports,
        sim_time: f64,
        attitude: Quat,
        plant_attitude: Quat,
        motors: &[f32; 4],
    ) {
        if sim_time - self.last_viz < VIZ_PERIOD {
            return;
        }
        self.last_viz = sim_time;
        let total: f32 = motors.iter().sum();
        let tilt = (attitude * Vec3::Z).z.clamp(-1.0, 1.0).acos().to_degrees();
        let att_err = (attitude.inverse() * plant_attitude)
            .to_scaled_axis()
            .length()
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
            "fc/debug/attitude_err_deg",
            &[f64::from(att_err)],
        );
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
        sim_time = inputs.plant.map_or(sim_time, |(p, _)| p.timestamp);
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
                    "真值回落"
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
