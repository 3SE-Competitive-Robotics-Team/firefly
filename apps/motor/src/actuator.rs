//! `sysfs` PWM 输出、`Control` 订阅与 `Airframe` 发布。

use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use firefly_hal::esc::ThrustToDuty;
use firefly_pubsub::control::{CONTROL_BUFFER_SIZE, CONTROL_TOPIC, ControlMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::plant::{AIRFRAME_TOPIC, AirframeMessage};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::Subscriber;
use iceoryx2_bb_posix::clock::{ClockType, Time};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalGuard, SignalHandler};
use serde::Deserialize;

/// `configs/motor.toml` 缺省路径（相对运行目录，板上是 `/opt/firefly`）。
/// 缺省配置：按安装布局解析，与启动目录无关。
fn default_config() -> PathBuf {
    firefly_config::paths::config("motor.toml")
}
/// `sysfs` PWM 类目录。
const PWM_CLASS: &str = "/sys/class/pwm";
/// 电机数：与 `ControlMessage::thrust` 的定长数组一致。
const MOTORS: usize = 4;
/// 通道目录出现的最长等待；导出是异步的，刚写完 `export` 时目录还不存在。
const EXPORT_TIMEOUT: Duration = Duration::from_millis(200);
/// 导出轮询间隔。
const EXPORT_POLL: Duration = Duration::from_millis(10);
/// 日志推进周期。
const PUMP_PERIOD: Duration = Duration::from_millis(100);

/// 退出标志（信号处理器置位，主循环每拍检查）。
static RUNNING: AtomicBool = AtomicBool::new(true);

/// 执行器配置。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// 控制输出速率（Hz）。
    #[serde(default = "default_rate_hz")]
    rate_hz: f64,
    /// `Control` 断流判定（ms）；超过即回到零推力脉宽。
    #[serde(default = "default_stale_ms")]
    stale_ms: u64,
    /// `Airframe` 重发周期（ms）。
    #[serde(default = "default_airframe_ms")]
    airframe_ms: u64,
    /// PWM 时序。
    pwm: Pwm,
    /// 四路电调通道，顺序与 `ControlMessage::thrust` 一致。
    channel: [ChannelSpec; MOTORS],
    /// 机体与执行器描述。
    airframe: AirframeConfig,
}

/// PWM 时序参数。单位写进字段名，调用点不必回查文档。
#[allow(clippy::struct_field_names)] // 后缀是单位，不是重复的语义
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Pwm {
    /// 周期（ns）。
    period_ns: u64,
    /// 零推力脉宽（ns）。
    min_duty_ns: u64,
    /// 满推力脉宽（ns）。
    max_duty_ns: u64,
}

/// 一路 `sysfs` PWM 通道。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ChannelSpec {
    /// `pwmchip` 编号。
    chip: u32,
    /// 芯片内的通道号。
    pwm: u32,
}

/// 机体与执行器描述（`Firefly/Airframe` 的内容来源）。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AirframeConfig {
    /// 质量（kg）。
    mass: f64,
    /// 机体系转动惯量对角元（kg·m²）。
    inertia: [f64; 3],
    /// 平移阻尼（N·s/m）。
    #[serde(default)]
    linear_drag: f64,
    /// 角速度阻尼率（1/s）。
    #[serde(default)]
    angular_drag: f64,
    /// 4 旋翼的机体系安装位置（m）。
    rotor_positions: [[f64; 3]; MOTORS],
    /// 4 旋翼旋向（`+1` = 反扭矩指向机体 `+Z`）。
    rotor_spins: [f64; MOTORS],
    /// 单电机推力上限（N），同时是推力到脉宽映射的满量程。
    max_thrust_per_motor: f64,
    /// 反扭矩系数（m）。
    torque_coefficient: f64,
}

const fn default_rate_hz() -> f64 {
    200.0
}

const fn default_stale_ms() -> u64 {
    100
}

const fn default_airframe_ms() -> u64 {
    1_000
}

/// 解析 `--config <path>`（缺省 [`default_config`]）。
fn parse_config_path() -> Result<String, String> {
    let mut it = std::env::args().skip(1);
    let mut path = default_config().display().to_string();
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

/// 校验速率、PWM 时序与机体参数。
fn validate(config: &Config) -> firefly_error::Result<()> {
    let invalid = |message: String| {
        Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            message,
        ))
    };
    if !config.rate_hz.is_finite() || config.rate_hz <= 0.0 || config.rate_hz > 1_000.0 {
        return invalid(format!(
            "控制速率必须在 (0, 1000] Hz 内：{}",
            config.rate_hz
        ));
    }
    if config.stale_ms == 0 || config.airframe_ms == 0 {
        return invalid("stale_ms 与 airframe_ms 必须为正".to_owned());
    }
    let mut seen = std::collections::HashSet::new();
    for spec in &config.channel {
        if !seen.insert((spec.chip, spec.pwm)) {
            return invalid(format!(
                "通道 pwmchip{} / pwm{} 重复配置",
                spec.chip, spec.pwm
            ));
        }
    }
    if let Err(error) = ThrustToDuty::new(
        config.pwm.period_ns,
        config.pwm.min_duty_ns,
        config.pwm.max_duty_ns,
        config.airframe.max_thrust_per_motor,
    ) {
        return invalid(error);
    }
    let airframe = &config.airframe;
    if !airframe.mass.is_finite() || airframe.mass <= 0.0 {
        return invalid(format!("质量必须为正：{}", airframe.mass));
    }
    if airframe.inertia.iter().any(|value| *value <= 0.0) {
        return invalid(format!("惯量对角元必须为正：{:?}", airframe.inertia));
    }
    if airframe
        .rotor_spins
        .iter()
        .any(|spin| (*spin - 1.0).abs() > 1e-9 && (*spin + 1.0).abs() > 1e-9)
    {
        return invalid(format!("旋向只能是 +1 或 -1：{:?}", airframe.rotor_spins));
    }
    if airframe.torque_coefficient < 0.0 {
        return invalid(format!(
            "反扭矩系数不能为负：{}",
            airframe.torque_coefficient
        ));
    }
    Ok(())
}

/// 控制主循环：订阅 `Control`，把推力映射成脉宽写到 `sysfs`，并重发 `Airframe`。
pub fn run() -> Result<(), Box<dyn Error>> {
    let config_path =
        parse_config_path().map_err(|error| format!("{error}；用法：motor [--config <path>]"))?;
    let config: Config =
        firefly_config::startup(&config_path, firefly_observability::init, validate)?;
    let mapping = ThrustToDuty::new(
        config.pwm.period_ns,
        config.pwm.min_duty_ns,
        config.pwm.max_duty_ns,
        config.airframe.max_thrust_per_motor,
    )?;

    let _signal_guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_signal| {
            RUNNING.store(false, Ordering::Relaxed);
        },
    )?;

    let mut channels = Vec::with_capacity(MOTORS);
    for spec in &config.channel {
        channels.push(Channel::open(spec, &mapping)?);
    }

    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "motor");
    let control = Subscriber::<ControlMessage>::with_topic_and_buffer(
        &node,
        CONTROL_TOPIC,
        CONTROL_BUFFER_SIZE,
    )?;
    let airframe_pub = Publisher::<AirframeMessage>::with_topic(&node, AIRFRAME_TOPIC)?;
    log::info!(
        "电调：{} 路，{} Hz，脉宽 {}..{} ns / 周期 {} ns；推力上限 {} N",
        MOTORS,
        config.rate_hz,
        config.pwm.min_duty_ns,
        config.pwm.max_duty_ns,
        config.pwm.period_ns,
        config.airframe.max_thrust_per_motor
    );

    let result = drive(
        &mut channels,
        &control,
        &airframe_pub,
        &config,
        &mapping,
        &log_ipc,
    );
    for channel in &mut channels {
        if let Err(error) = channel.set_duty(mapping.min_duty_ns()) {
            log::error!("退出复位失败：{error}");
        }
    }
    firefly_observability::pump_log_ipc(&log_ipc);
    result
}

/// 主循环；返回后由 [`run`] 统一把通道复位到零推力脉宽。
fn drive(
    channels: &mut [Channel],
    control: &Subscriber<ControlMessage>,
    airframe_pub: &Publisher<AirframeMessage>,
    config: &Config,
    mapping: &ThrustToDuty,
    log_ipc: &firefly_observability::LogIpc,
) -> Result<(), Box<dyn Error>> {
    let period = Duration::from_secs_f64(1.0 / config.rate_hz);
    let stale = Duration::from_millis(config.stale_ms);
    let airframe_period = Duration::from_millis(config.airframe_ms);
    let mut deadline = Instant::now();
    let mut last_control: Option<Instant> = None;
    let mut last_airframe: Option<Instant> = None;
    let mut thrust = [0.0f64; MOTORS];
    let mut stale_reported = false;
    let mut last_pump = Instant::now();

    while RUNNING.load(Ordering::Relaxed) {
        while let Some(sample) = control.receive()? {
            thrust = sample.thrust;
            last_control = Some(Instant::now());
        }
        let fresh = last_control.is_some_and(|at| at.elapsed() <= stale);
        if fresh {
            stale_reported = false;
        } else if !stale_reported {
            log::warn!(
                "Control 中断超过 {} ms，四路回到零推力脉宽",
                config.stale_ms
            );
            stale_reported = true;
        }

        for (index, channel) in channels.iter_mut().enumerate() {
            let wanted = if fresh { thrust[index] } else { 0.0 };
            channel
                .set_duty(mapping.duty_ns(wanted))
                .map_err(|error| format!("电机 {index}: {error}"))?;
        }

        if last_airframe.is_none_or(|at| at.elapsed() >= airframe_period) {
            let stamp = monotonic_seconds()?;
            airframe_pub.publish_with(|message| {
                message.timestamp = stamp;
                message.mass = config.airframe.mass;
                message.inertia_x = config.airframe.inertia[0];
                message.inertia_y = config.airframe.inertia[1];
                message.inertia_z = config.airframe.inertia[2];
                message.linear_drag = config.airframe.linear_drag;
                message.angular_drag = config.airframe.angular_drag;
                message.rotor_positions = config.airframe.rotor_positions;
                message.rotor_spins = config.airframe.rotor_spins;
                message.max_thrust_per_motor = config.airframe.max_thrust_per_motor;
                message.torque_coefficient = config.airframe.torque_coefficient;
            })?;
            last_airframe = Some(Instant::now());
        }

        if last_pump.elapsed() >= PUMP_PERIOD {
            firefly_observability::pump_log_ipc(log_ipc);
            last_pump = Instant::now();
        }
        deadline += period;
        let now = Instant::now();
        if deadline > now {
            thread::sleep(deadline - now);
        } else {
            deadline = now;
        }
    }
    log::info!("收到终止信号，电调输出复位");
    Ok(())
}

/// 一路 `sysfs` PWM 通道。
struct Channel {
    /// `duty_cycle` 属性路径。
    duty_path: PathBuf,
    /// 当前写入的脉宽（ns），用于跳过重复写。
    current: u64,
}

impl Channel {
    /// 导出通道、设定周期与零推力脉宽、使能输出。
    fn open(spec: &ChannelSpec, mapping: &ThrustToDuty) -> Result<Self, String> {
        let chip_dir = Path::new(PWM_CLASS).join(format!("pwmchip{}", spec.chip));
        let channel_dir = chip_dir.join(format!("pwm{}", spec.pwm));
        if !channel_dir.exists() {
            fs::write(chip_dir.join("export"), spec.pwm.to_string())
                .map_err(|error| format!("导出 {}: {error}", channel_dir.display()))?;
            wait_for(&channel_dir)?;
        }
        let enable = channel_dir.join("enable");
        let duty_path = channel_dir.join("duty_cycle");
        // 先失能再改时序：部分驱动不允许在使能状态下改周期。
        let _ = fs::write(&enable, "0");
        // 周期未设时内核拒绝任何占空比（周期 0 非法），这次清零失败是预期的；
        // 它只在通道被上次运行留在高占空比时才起作用。
        let _ = write_attribute(&duty_path, 0);
        write_attribute(&channel_dir.join("period"), mapping.period_ns())?;
        write_attribute(&duty_path, mapping.min_duty_ns())?;
        write_attribute(&enable, 1).map_err(|error| {
            format!(
                "{error}；`sysfs` 通道使能需要设备树描述，`pwm-sunxi` 会拒绝未描述的通道\
                 （dmesg 里是 `can't parse pwm device`）"
            )
        })?;
        Ok(Self {
            duty_path,
            current: mapping.min_duty_ns(),
        })
    }

    /// 写入脉宽；与上次相同时跳过，避免高频 `sysfs` 写。
    fn set_duty(&mut self, duty_ns: u64) -> Result<(), String> {
        if self.current == duty_ns {
            return Ok(());
        }
        write_attribute(&self.duty_path, duty_ns)?;
        self.current = duty_ns;
        Ok(())
    }
}

/// 等待异步出现的通道目录。
fn wait_for(path: &Path) -> Result<(), String> {
    let started = Instant::now();
    while started.elapsed() < EXPORT_TIMEOUT {
        if path.exists() {
            return Ok(());
        }
        thread::sleep(EXPORT_POLL);
    }
    Err(format!("等待 {} 出现超时", path.display()))
}

/// 写一个 `sysfs` 属性。
fn write_attribute(path: &Path, value: impl std::fmt::Display) -> Result<(), String> {
    fs::write(path, value.to_string()).map_err(|error| format!("写 {}: {error}", path.display()))
}

/// `CLOCK_MONOTONIC` 秒：板载各进程共用的时间基准。
fn monotonic_seconds() -> Result<f64, Box<dyn Error>> {
    let time = Time::now_with_clock(ClockType::Monotonic)
        .map_err(|error| format!("读取单调时钟失败: {error:?}"))?;
    Ok(time.as_duration().as_secs_f64())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 仓库里的配置必须能被本进程解析并通过校验：板端启动才发现漂移太晚。
    #[test]
    fn shipped_config_parses() {
        let raw = include_str!("../../../configs/motor.toml");
        let config: Config = toml::from_str(raw).expect("configs/motor.toml");
        validate(&config).expect("valid motor config");
        ThrustToDuty::new(
            config.pwm.period_ns,
            config.pwm.min_duty_ns,
            config.pwm.max_duty_ns,
            config.airframe.max_thrust_per_motor,
        )
        .expect("valid pwm mapping");
    }

    #[test]
    fn duplicate_channels_are_rejected() {
        let raw = include_str!("../../../configs/motor.toml");
        let mut config: Config = toml::from_str(raw).expect("configs/motor.toml");
        config.channel[1] = ChannelSpec { chip: 0, pwm: 0 };
        assert!(validate(&config).is_err());
    }
}
