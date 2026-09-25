//! 飞控进程：1kHz 控制环（`firefly-flight`）+ iceoryx2。
//!
//! 输入：
//! - `Firefly/PlantState`（被控对象真值状态，200Hz）——位置/速度回落源 + 姿态校验；
//! - `Firefly/Airframe`（质量/惯量/旋翼几何/电机上限，1Hz 电平）——飞控参数的唯一来源；
//! - `Firefly/Odometry` / `Firefly/CorrectedOdometry`——位置/速度/航向反馈（取消息新的）；
//! - `Firefly/Imu`（陀螺 + 加计，100Hz）——姿态估计（内环）；
//! - `Firefly/Reference`（规划参考：位置/速度/偏航/偏航角速度）。
//!
//! 输出：`Firefly/Control`（4 电机推力，1kHz；被控对象每步取最新）。
//!
//! 反馈来源分工：**内环姿态由飞控自估**（陀螺积分 + 加速度计水平修正，航向取 VIO，
//! 上电用加计定滚转/俯仰）；位置/速度取 VIO 估计（飞控只看得到估计），估计未到达时
//! 回落被控对象真值（起飞前正常）。被控对象状态另作姿态误差校验（进 rrd）。
//!
//! 节拍：控制律无积分项，dt 不进控制，因而 tick 只需节拍正确、不需要硬实时时钟；
//! 落后节拍计数并进日志（唯一证据来源），被控对象状态陈旧（>50ms 未更新）时
//! **不发指令**——由被控对象自己悬停兜底，绝不用陈旧状态算控制。
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
    Airframe, AttitudeEstimator, ControlParams, PositionSetpoint, QuadParams, QuadState,
    position_mode, yaw_of,
};
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
/// 被控对象状态陈旧阈值（墙钟）：超过即停发指令（被控对象悬停兜底）。
const STALE_LIMIT: Duration = Duration::from_millis(50);
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

/// 运行期输入快照（每 tick 排空到最新）。
#[derive(Default)]
struct Inputs {
    /// 被控对象状态与到达墙钟时刻（判陈旧）。
    plant: Option<(PlantStateMessage, Instant)>,
    /// 机体/执行器（装配成 `firefly-flight` 的参数）。
    vehicle: Option<(QuadParams, Airframe)>,
    /// 位置/速度反馈：`(时间戳, 位置, 速度, JPL 姿态 [x,y,z,w])`——姿态用于航向修正。
    estimate: Option<(f64, Vec3, Vec3, [f64; 4])>,
    /// 最新 IMU：`(陀螺, 加计)`，机体系。
    imu: Option<(Vec3, Vec3)>,
    /// 参考（位置/速度/偏航/偏航角速度）。
    reference: Option<ReferenceMessage>,
}

/// 端口集合（避免主循环参数爆炸）。
struct Ports {
    plant: PlantStateSubscriber,
    airframe: AirframeSubscriber,
    imu: ImuSubscriber,
    odom: OdomSubscriber,
    corrected: CorrectedOdomSubscriber,
    reference: Subscriber<ReferenceMessage>,
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
            ));
            imu_count += 1;
        }
        // 位置/速度/姿态反馈取原始与校正估计中消息更新的那个（与 planner 同规则）
        let mut newest: Option<(f64, Vec3, Vec3, [f64; 4])> = None;
        for sample in [self.odom.receive()?, self.corrected.receive()?]
            .into_iter()
            .flatten()
        {
            let m = *sample;
            let candidate = (
                m.timestamp,
                Vec3::new(
                    m.position_x as f32,
                    m.position_y as f32,
                    m.position_z as f32,
                ),
                Vec3::new(
                    m.velocity_x as f32,
                    m.velocity_y as f32,
                    m.velocity_z as f32,
                ),
                [m.quat_x, m.quat_y, m.quat_z, m.quat_w],
            );
            if newest.is_none_or(|cur| candidate.0 > cur.0) {
                newest = Some(candidate);
            }
        }
        if let Some(candidate) = newest
            && inputs.estimate.is_none_or(|cur| candidate.0 > cur.0)
        {
            inputs.estimate = Some(candidate);
        }
        while let Some(sample) = self.reference.receive()? {
            inputs.reference = Some(*sample);
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

/// 控制环的跨 tick 状态。
struct Controller {
    /// 姿态估计（内环）。上电用加计定滚转/俯仰，航向由 VIO 慢修。
    estimator: AttitudeEstimator,
    /// 姿态估计是否已启动。
    attitude_ready: bool,
    /// 上次航向修正时刻（估算修正 dt）。
    last_yaw_fix: Option<Instant>,
    /// 参考状态（规划未到时保持当前位置）。
    setpoint: PositionSetpoint,
    have_setpoint: bool,
    /// 真值回落告警是否已发（只发一次）。
    warned_truth_fallback: bool,
    /// 上次可视化时刻（仿真时间）。
    last_viz: f64,
}

impl Controller {
    fn new() -> Self {
        Self {
            estimator: AttitudeEstimator::default(),
            attitude_ready: false,
            last_yaw_fix: None,
            setpoint: PositionSetpoint::default(),
            have_setpoint: false,
            warned_truth_fallback: false,
            last_viz: f64::NEG_INFINITY,
        }
    }

    /// 一步：姿态估计 → 位置/速度反馈 → 参考 → 位置模式 → 分配 → 发布指令。
    ///
    /// 无输入或状态陈旧时直接返回（不发指令）：被控对象自有悬停兜底，
    /// 飞控绝不用陈旧状态算控制。
    fn step(
        &mut self,
        ports: &Ports,
        inputs: &Inputs,
        dt: f32,
        ctl: &ControlParams,
        tick: u64,
        stats: &mut Stats,
    ) -> Result<(), firefly_error::Error> {
        let (Some((plant, received_at)), Some((quad, airframe))) = (inputs.plant, inputs.vehicle)
        else {
            return Ok(());
        };
        if received_at.elapsed() > STALE_LIMIT {
            return Ok(());
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
        self.update_setpoint(inputs, position, attitude);

        // ---- 控制：位置模式 → 期望 wrench → 4 电机分配（唯一饱和点） ----
        let quad_state = QuadState {
            position,
            velocity,
            attitude,
            ang_vel,
        };
        let desired = position_mode(&quad_state, &self.setpoint, &quad, ctl);
        let allocation = airframe.allocate(attitude, &desired);
        ports.control.publish(ControlMessage {
            state_time: plant.timestamp,
            thrust: allocation.motors.map(f64::from),
            tick,
        })?;
        stats.published += 1;
        if allocation.saturated {
            stats.saturated += 1;
        }
        self.publish_viz(
            ports,
            plant.timestamp,
            attitude,
            plant_attitude,
            &allocation,
        );
        Ok(())
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
        let Some((gyro, accel)) = inputs.imu else {
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
        if let Some((_, _, _, odom_quat)) = inputs.estimate {
            let yaw_src = yaw_of(vio::body_to_world_from_odom(odom_quat));
            let dt_fix = self
                .last_yaw_fix
                .map_or(dt, |t| t.elapsed().as_secs_f32().clamp(dt, 0.5));
            self.estimator.correct_yaw(yaw_src, dt_fix);
            self.last_yaw_fix = Some(Instant::now());
        }
        (self.estimator.attitude(), gyro)
    }

    /// 位置/速度反馈：VIO 估计优先，未到达回落被控对象真值（起飞前正常，告警一次）。
    fn feedback(&mut self, inputs: &Inputs, plant: &PlantStateMessage) -> (Vec3, Vec3) {
        if let Some((_, pos, vel, _)) = inputs.estimate {
            return (pos, vel);
        }
        if !self.warned_truth_fallback {
            self.warned_truth_fallback = true;
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

    /// 更新参考：规划未到时保持当前位置（初始悬停点）。
    fn update_setpoint(&mut self, inputs: &Inputs, position: Vec3, attitude: Quat) {
        if let Some(reference) = inputs.reference {
            self.setpoint = PositionSetpoint {
                position: Vec3::new(
                    reference.position_x as f32,
                    reference.position_y as f32,
                    reference.position_z as f32,
                ),
                velocity: Vec3::new(
                    reference.velocity_x as f32,
                    reference.velocity_y as f32,
                    reference.velocity_z as f32,
                ),
                yaw: reference.yaw as f32,
                yaw_rate: reference.yaw_dot as f32,
            };
            self.have_setpoint = true;
        } else if !self.have_setpoint {
            self.setpoint = PositionSetpoint {
                position,
                yaw: yaw_of(attitude),
                ..PositionSetpoint::default()
            };
            self.have_setpoint = true;
        }
    }

    /// 控制量 + 姿态校验进 rrd（10Hz）。
    fn publish_viz(
        &mut self,
        ports: &Ports,
        sim_time: f64,
        attitude: Quat,
        plant_attitude: Quat,
        allocation: &firefly_flight::Allocation,
    ) {
        if sim_time - self.last_viz < VIZ_PERIOD {
            return;
        }
        self.last_viz = sim_time;
        let total: f32 = allocation.motors.iter().sum();
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
            &allocation.motors.map(f64::from),
        );
        publish_scalars(
            ports.viz_ref(),
            sim_time,
            "fc/debug/attitude_err_deg",
            &[f64::from(att_err)],
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
        "飞控进程启动：{} Hz 控制环，配置 {config_path}",
        cfg.rate_hz
    );

    let mut inputs = Inputs::default();
    let mut controller = Controller::new();
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
                "t={:.2} 指令 {} 条（{:.0} Hz），tick {}，晚到 {:.1}%（最大 {:.2} ms），饱和 {:.1}%，IMU {:.0} Hz，姿态源 {}",
                sim_time,
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
    log::info!("飞控进程退出（t={sim_time:.2}）");
    Ok(())
}
