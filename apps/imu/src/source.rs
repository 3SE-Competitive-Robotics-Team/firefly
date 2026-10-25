//! `BMI088` 的 SPI 访问与采样主循环。

use std::error::Error;
use std::io;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use firefly_hal::bmi088;
use firefly_pubsub::imu::{IMU_TOPIC, ImuMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::publish::Publisher;
use iceoryx2_bb_posix::clock::{ClockType, Time};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalGuard, SignalHandler};
use serde::Deserialize;
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

/// `configs/imu.toml` 缺省路径（相对运行目录，板上是 `/opt/firefly`）。
/// 缺省配置：按安装布局解析，与启动目录无关。
fn default_config() -> PathBuf {
    firefly_config::paths::config("imu.toml")
}
/// 采样率上限（Hz）：再高只会重复读到同一输出数据率的结果。
const MAX_RATE_HZ: f64 = 1_000.0;
/// 日志与可视化的推进周期；高频采样下不必每拍都排空日志队列。
const PUMP_PERIOD: Duration = Duration::from_millis(100);
/// 加速度计软复位后的稳定时间。
const ACCEL_RESET_SETTLE: Duration = Duration::from_millis(50);
/// 陀螺软复位后的稳定时间。
const GYRO_RESET_SETTLE: Duration = Duration::from_millis(30);
/// 配置写入到可读数之间的稳定时间。
const POWER_UP_SETTLE: Duration = Duration::from_millis(5);

/// 退出标志（信号处理器置位，主循环每拍检查）。
static RUNNING: AtomicBool = AtomicBool::new(true);

/// IMU 配置。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// 采样率（Hz）。
    #[serde(default = "default_rate_hz")]
    rate_hz: f64,
    /// SPI 时钟模式（0..=3）。
    #[serde(default = "default_mode")]
    mode: u8,
    /// 加速度计从机。
    accel: AccelDevice,
    /// 陀螺从机。
    gyro: GyroDevice,
}

/// 加速度计从机与量程。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AccelDevice {
    /// `spidev` 设备节点。
    device: String,
    /// SPI 时钟（Hz）。
    #[serde(default = "default_speed_hz")]
    speed_hz: u32,
    /// 量程（g）。
    #[serde(default = "default_accel_range_g")]
    range_g: u32,
}

/// 陀螺从机与量程。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct GyroDevice {
    /// `spidev` 设备节点。
    device: String,
    /// SPI 时钟（Hz）。
    #[serde(default = "default_speed_hz")]
    speed_hz: u32,
    /// 量程（°/s）。
    #[serde(default = "default_gyro_range_dps")]
    range_dps: u32,
}

const fn default_rate_hz() -> f64 {
    400.0
}

const fn default_mode() -> u8 {
    3
}

const fn default_speed_hz() -> u32 {
    10_000_000
}

const fn default_accel_range_g() -> u32 {
    6
}

const fn default_gyro_range_dps() -> u32 {
    2_000
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

/// 校验采样率、SPI 模式与接线参数。
fn validate(config: &Config) -> firefly_error::Result<()> {
    let invalid = |message: String| {
        Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            message,
        ))
    };
    if !config.rate_hz.is_finite() || config.rate_hz <= 0.0 || config.rate_hz > MAX_RATE_HZ {
        return invalid(format!(
            "采样率必须在 (0, {MAX_RATE_HZ}] Hz 内：{}",
            config.rate_hz
        ));
    }
    if config.mode > 3 {
        return invalid(format!("SPI 模式只支持 0..=3：{}", config.mode));
    }
    for (name, speed_hz) in [
        ("accel", config.accel.speed_hz),
        ("gyro", config.gyro.speed_hz),
    ] {
        if speed_hz == 0 {
            return invalid(format!("{name} 的 speed_hz 必须为正"));
        }
    }
    Ok(())
}

/// 采样主循环：按固定速率读两片从机并发布一条 `Firefly/Imu`。
pub fn run() -> Result<(), Box<dyn Error>> {
    let config_path =
        parse_config_path().map_err(|error| format!("{error}；用法：imu [--config <path>]"))?;
    let config: Config =
        firefly_config::startup(&config_path, firefly_observability::init, validate)?;
    let accel_range = bmi088::AccelRange::from_g(config.accel.range_g)?;
    let gyro_range = bmi088::GyroRange::from_dps(config.gyro.range_dps)?;
    let odr = bmi088::AccelOdr::at_least(config.rate_hz)?;
    let mode = mode_flags(config.mode)?;

    let _signal_guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_signal| {
            RUNNING.store(false, Ordering::Relaxed);
        },
    )?;

    let accel = Spi::open(&config.accel.device, config.accel.speed_hz, mode)?;
    let gyro = Spi::open(&config.gyro.device, config.gyro.speed_hz, mode)?;
    init_accel(&accel, accel_range, odr)?;
    init_gyro(&gyro, gyro_range)?;

    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "imu");
    let publisher = Publisher::<ImuMessage>::with_topic(&node, IMU_TOPIC)?;
    log::info!(
        "IMU：加速度计 {}（±{} g）/ 陀螺 {}（±{} °/s），{} Hz",
        config.accel.device,
        config.accel.range_g,
        config.gyro.device,
        config.gyro.range_dps,
        config.rate_hz
    );

    let period = Duration::from_secs_f64(1.0 / config.rate_hz);
    let mut deadline = Instant::now();
    let mut last_pump = Instant::now();
    while RUNNING.load(Ordering::Relaxed) {
        let accel_bytes =
            accel.read::<6>(&bmi088::accel_read_command(bmi088::ACCEL_DATA_REGISTER))?;
        let gyro_bytes = gyro.read::<6>(&bmi088::gyro_read_command(bmi088::GYRO_DATA_REGISTER))?;
        let stamp = monotonic_seconds()?;
        let angular = bmi088::gyro_radians_per_second(bmi088::decode_axes(&gyro_bytes), gyro_range);
        let linear =
            bmi088::accel_metres_per_second_squared(bmi088::decode_axes(&accel_bytes), accel_range);
        publisher.publish(ImuMessage {
            timestamp: stamp,
            angular_velocity_x: angular[0],
            angular_velocity_y: angular[1],
            angular_velocity_z: angular[2],
            linear_acceleration_x: linear[0],
            linear_acceleration_y: linear[1],
            linear_acceleration_z: linear[2],
        })?;

        if last_pump.elapsed() >= PUMP_PERIOD {
            firefly_observability::pump_log_ipc(&log_ipc);
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
    log::info!("收到终止信号，IMU 退出");
    firefly_observability::pump_log_ipc(&log_ipc);
    Ok(())
}

/// 单片 `spidev` 从机。
struct Spi {
    device: Spidev,
    path: String,
}

impl Spi {
    /// 打开并配置一条 `spidev` 通道。
    fn open(path: &str, speed_hz: u32, mode: SpiModeFlags) -> io::Result<Self> {
        let mut device = Spidev::open(path)?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(speed_hz)
            .mode(mode)
            .build();
        device.configure(&options)?;
        Ok(Self {
            device,
            path: path.to_owned(),
        })
    }

    /// 先发 `header`，再读 `N` 字节。
    fn read<const N: usize>(&self, header: &[u8]) -> io::Result<[u8; N]> {
        let mut tx = vec![0u8; header.len() + N];
        tx[..header.len()].copy_from_slice(header);
        let mut rx = vec![0u8; tx.len()];
        {
            let mut transfer = SpidevTransfer::read_write(&tx, &mut rx);
            self.device.transfer(&mut transfer)?;
        }
        let mut out = [0u8; N];
        out.copy_from_slice(&rx[header.len()..]);
        Ok(out)
    }

    /// 读一个寄存器。
    fn read_byte(&self, header: &[u8]) -> io::Result<u8> {
        Ok(self.read::<1>(header)?[0])
    }

    /// 写一个寄存器。
    fn write(&self, register: u8, value: u8) -> io::Result<()> {
        let tx = [register & !0x80, value];
        let mut rx = [0u8; 2];
        let mut transfer = SpidevTransfer::read_write(&tx, &mut rx);
        self.device.transfer(&mut transfer)
    }
}

/// `BMI088` 加速度计初始化：确认型号、软复位、上电、设带宽与量程。
fn init_accel(
    spi: &Spi,
    range: bmi088::AccelRange,
    odr: bmi088::AccelOdr,
) -> Result<(), Box<dyn Error>> {
    let id = spi.read_byte(&bmi088::accel_read_command(bmi088::ACCEL_CHIP_ID_REGISTER))?;
    if id != bmi088::ACCEL_CHIP_ID {
        return Err(format!(
            "{} 的加速度计 WHO_AM_I = 0x{id:02X}，期望 0x{:02X}",
            spi.path,
            bmi088::ACCEL_CHIP_ID
        )
        .into());
    }
    spi.write(bmi088::ACCEL_SOFTRESET_REGISTER, bmi088::SOFTRESET_COMMAND)?;
    thread::sleep(ACCEL_RESET_SETTLE);
    spi.write(bmi088::ACCEL_PWR_CTRL_REGISTER, bmi088::ACCEL_PWR_CTRL_ON)?;
    thread::sleep(POWER_UP_SETTLE);
    spi.write(
        bmi088::ACCEL_PWR_CONF_REGISTER,
        bmi088::ACCEL_PWR_CONF_ACTIVE,
    )?;
    spi.write(bmi088::ACCEL_CONF_REGISTER, bmi088::accel_conf(odr))?;
    spi.write(bmi088::ACCEL_RANGE_REGISTER, range.bits())?;
    thread::sleep(POWER_UP_SETTLE);
    log::info!("{} 加速度计就绪（WHO_AM_I=0x{id:02X}）", spi.path);
    Ok(())
}

/// `BMI088` 陀螺初始化：确认型号、软复位、退出省电、设带宽与量程。
fn init_gyro(spi: &Spi, range: bmi088::GyroRange) -> Result<(), Box<dyn Error>> {
    let id = spi.read_byte(&bmi088::gyro_read_command(bmi088::GYRO_CHIP_ID_REGISTER))?;
    if id != bmi088::GYRO_CHIP_ID {
        return Err(format!(
            "{} 的陀螺 WHO_AM_I = 0x{id:02X}，期望 0x{:02X}",
            spi.path,
            bmi088::GYRO_CHIP_ID
        )
        .into());
    }
    spi.write(bmi088::GYRO_SOFTRESET_REGISTER, bmi088::SOFTRESET_COMMAND)?;
    thread::sleep(GYRO_RESET_SETTLE);
    spi.write(bmi088::GYRO_LPM1_REGISTER, bmi088::GYRO_LPM1_NORMAL)?;
    spi.write(bmi088::GYRO_RANGE_REGISTER, range.bits())?;
    spi.write(
        bmi088::GYRO_BANDWIDTH_REGISTER,
        bmi088::GYRO_BANDWIDTH_2000_HZ,
    )?;
    thread::sleep(POWER_UP_SETTLE);
    log::info!("{} 陀螺就绪（WHO_AM_I=0x{id:02X}）", spi.path);
    Ok(())
}

/// SPI 模式号 → 标志位。
fn mode_flags(mode: u8) -> Result<SpiModeFlags, String> {
    match mode {
        0 => Ok(SpiModeFlags::SPI_MODE_0),
        1 => Ok(SpiModeFlags::SPI_MODE_1),
        2 => Ok(SpiModeFlags::SPI_MODE_2),
        3 => Ok(SpiModeFlags::SPI_MODE_3),
        other => Err(format!("SPI 模式只支持 0..=3：{other}")),
    }
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
        let raw = include_str!("../../../configs/imu.toml");
        let config: Config = toml::from_str(raw).expect("configs/imu.toml");
        validate(&config).expect("valid imu config");
        bmi088::AccelRange::from_g(config.accel.range_g).expect("accel range");
        bmi088::GyroRange::from_dps(config.gyro.range_dps).expect("gyro range");
        bmi088::AccelOdr::at_least(config.rate_hz).expect("accel odr");
        mode_flags(config.mode).expect("spi mode");
    }
}
