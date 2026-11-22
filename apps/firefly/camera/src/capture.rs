use firefly_error::{Error, ErrorKind, Result};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};

use firefly_hal::pixel::{FrameFormat, gray_from_yuyv};
use firefly_pubsub::camera::FrameCadence;
use firefly_pubsub::camera::{
    CAMERA_DOWN_TOPIC, CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, DOWN_HEIGHT, DOWN_SENSOR_ID,
    DOWN_SIZE, DOWN_WIDTH, DownImageMessage, GrayImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH,
};
use firefly_pubsub::node::create_node;
use firefly_pubsub::publish::Publisher;
use iceoryx2_bb_posix::clock::{ClockType, Time};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalGuard, SignalHandler};
use serde::Deserialize;
use v4l::buffer::Type;
use v4l::device::Device;
use v4l::io::traits::CaptureStream;
use v4l::prelude::MmapStream;
use v4l::video::Capture;
use v4l::{Format, FourCC};

/// 默认采集缓冲数；太少容易在调度抖动时丢帧。
const DEFAULT_BUFFERS: u32 = 4;

static RUNNING: AtomicBool = AtomicBool::new(true);

struct DownWorker(Option<std::thread::JoinHandle<Result<()>>>);
impl DownWorker {
    fn finish(mut self) -> Result<()> {
        if let Some(worker) = self.0.take() {
            worker
                .join()
                .map_err(|_| Error::new(ErrorKind::Internal, "down camera worker panicked"))?
        } else {
            Ok(())
        }
    }
}
impl Drop for DownWorker {
    fn drop(&mut self) {
        RUNNING.store(false, Ordering::Relaxed);
        if let Some(worker) = self.0.take() {
            let _ = worker.join();
        }
    }
}

fn start_down(source: &DownSource) -> Result<DownWorker> {
    let source = source.clone();
    let (ready, started) = std::sync::mpsc::sync_channel(1);
    let worker = std::thread::spawn(move || {
        let result = (|| -> Result<()> {
            let node = create_node()?;
            let publisher = Publisher::<DownImageMessage>::with_topic(&node, CAMERA_DOWN_TOPIC)?;
            let _ = ready.send(());
            capture_down(&source, &publisher)?;
            Ok(())
        })();
        match result {
            Ok(()) => Ok(()),
            Err(error) => {
                log::error!("down camera: {error}");
                RUNNING.store(false, Ordering::Relaxed);
                Err(error)
            }
        }
    });
    let guard = DownWorker(Some(worker));
    if started.recv().is_err() {
        guard.finish()?;
        return Err(Error::new(
            ErrorKind::Internal,
            "down camera exited before startup",
        ));
    }
    Ok(guard)
}

/// 下视采集流随进程持续运行，退出作用域时 Drop 执行 STREAMOFF。
fn capture_down(source: &DownSource, publisher: &Publisher<DownImageMessage>) -> Result<()> {
    let rate = source.rate_hz;
    let device = Device::with_path(&source.device).map_err(|e| {
        Error::from(e)
            .with_operation("open camera")
            .with_context("device", &source.device)
    })?;
    configure(&device, source.format, DOWN_WIDTH, DOWN_HEIGHT)?;
    let format = device.format()?;
    let row_bytes = DOWN_WIDTH * source.format.bytes_per_pixel();
    let stride = (format.stride as usize).max(row_bytes);
    let params = v4l::video::capture::Parameters::with_fps(rate);
    match device.set_params(&params) {
        Ok(actual) => log::info!(
            "down camera requested {rate} FPS, driver interval {}",
            actual.interval
        ),
        Err(error) => log::warn!(
            "down camera frame interval unavailable: {error}; capture-time rate gate remains active"
        ),
    }
    let mut stream = MmapStream::with_buffers(&device, Type::VideoCapture, source.buffers)?;
    stream.set_timeout(std::time::Duration::from_millis(100));
    let mut cadence = FrameCadence::default();
    let mut gray = vec![0; DOWN_SIZE];
    let mut last_stamp = f64::NEG_INFINITY;
    while RUNNING.load(Ordering::Relaxed) {
        let (bytes, metadata) = match stream.next() {
            Ok(frame) => frame,
            Err(error)
                if matches!(
                    error.kind(),
                    std::io::ErrorKind::TimedOut
                        | std::io::ErrorKind::WouldBlock
                        | std::io::ErrorKind::Interrupted
                ) =>
            {
                continue;
            }
            Err(error) => return Err(error.into()),
        };
        if metadata.flags.contains(v4l::buffer::Flags::ERROR) {
            continue;
        }
        if !metadata
            .flags
            .contains(v4l::buffer::Flags::TIMESTAMP_MONOTONIC)
        {
            return Err(Error::new(
                ErrorKind::Unsupported,
                "down camera requires V4L2 CLOCK_MONOTONIC capture timestamps",
            ));
        }
        let stamp = metadata.timestamp.sec as f64 + metadata.timestamp.usec as f64 * 1e-6;
        let Some(bytes) = bytes.get(..metadata.bytesused as usize) else {
            continue;
        };
        if !stamp.is_finite()
            || stamp <= last_stamp
            || bytes.len() < (DOWN_HEIGHT - 1) * stride + row_bytes
        {
            continue;
        }
        if !cadence.admit(stamp, rate) {
            continue;
        }
        for y in 0..DOWN_HEIGHT {
            let row = &bytes[y * stride..y * stride + row_bytes];
            let destination = &mut gray[y * DOWN_WIDTH..(y + 1) * DOWN_WIDTH];
            if source.format == FrameFormat::Grey {
                destination.copy_from_slice(row);
            } else {
                let written = gray_from_yuyv(row, destination);
                if written != DOWN_WIDTH {
                    return Err(Error::new(
                        ErrorKind::InvalidData,
                        "incomplete down YUYV row",
                    ));
                }
            }
        }
        last_stamp = stamp;
        publisher.publish_with(|message| {
            message.timestamp = stamp;
            message.sensor_id = DOWN_SENSOR_ID;
            message.width = DOWN_WIDTH as u32;
            message.height = DOWN_HEIGHT as u32;
            message.data.copy_from_slice(&gray);
        })?;
    }
    Ok(())
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// 图像宽度（像素）；必须与消息布局一致。
    #[serde(default = "default_width")]
    width: u32,
    /// 图像高度（像素）；必须与消息布局一致。
    #[serde(default = "default_height")]
    height: u32,
    left: Source,
    right: Source,
    #[serde(default)]
    down: Option<DownSource>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Source {
    device: String,
    #[serde(default = "default_format")]
    format: FrameFormat,
    #[serde(default = "default_buffers")]
    buffers: u32,
}

/// OV2640 下视采集与请求帧率上界。
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DownSource {
    device: String,
    #[serde(default = "default_format")]
    format: FrameFormat,
    #[serde(default = "default_buffers")]
    buffers: u32,
    #[serde(default = "default_down_rate")]
    rate_hz: u32,
}

const fn default_down_rate() -> u32 {
    15
}

const fn default_width() -> u32 {
    IMAGE_WIDTH as u32
}

const fn default_height() -> u32 {
    IMAGE_HEIGHT as u32
}

const fn default_buffers() -> u32 {
    DEFAULT_BUFFERS
}

const fn default_format() -> FrameFormat {
    FrameFormat::Grey
}

fn validate(config: &Config) -> firefly_error::Result<()> {
    let invalid = |message: String| {
        Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            message,
        ))
    };
    if config.width as usize != IMAGE_WIDTH || config.height as usize != IMAGE_HEIGHT {
        return invalid(format!(
            "相机分辨率必须是 {IMAGE_WIDTH}×{IMAGE_HEIGHT}（IPC 消息定长），配置为 {}×{}",
            config.width, config.height
        ));
    }
    for (name, source) in [("left", &config.left), ("right", &config.right)] {
        if source.format == FrameFormat::Z16 {
            return invalid(format!("{name} 是灰度相机，不能配置 z16"));
        }
        if source.buffers == 0 {
            return invalid(format!("{name} 的 buffers 必须为正"));
        }
    }
    if let Some(source) = &config.down
        && (source.format == FrameFormat::Z16
            || source.buffers == 0
            || source.device.is_empty()
            || !matches!(source.rate_hz, 15 | 30))
    {
        return invalid("invalid OV2640 down source".into());
    }
    Ok(())
}

/// 采集主循环：左、右目各取一帧发布（下视由独立线程采集），直到收到 `SIGINT`/`SIGTERM`。
pub fn run(config_path: &Path) -> Result<()> {
    let config: Config =
        firefly_config::startup(config_path, firefly_observability::init, validate)?;
    let width = config.width as usize;
    let height = config.height as usize;

    let _signal_guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_signal| {
            RUNNING.store(false, Ordering::Relaxed);
        },
    )
    .map_err(|e| Error::caused_by(ErrorKind::Internal, "register shutdown signals", e))?;

    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "camera");
    let down = config.down.as_ref().map(start_down).transpose()?;
    let left_pub = Publisher::<GrayImageMessage>::with_topic(&node, CAMERA_LEFT_TOPIC)?;
    let right_pub = Publisher::<GrayImageMessage>::with_topic(&node, CAMERA_RIGHT_TOPIC)?;
    log::info!(
        "相机源：左目 {} / 右目 {}（{width}×{height}）",
        config.left.device,
        config.right.device
    );

    let left_dev = Device::with_path(&config.left.device).map_err(|e| {
        Error::from(e)
            .with_operation("open camera")
            .with_context("device", &config.left.device)
    })?;
    configure(&left_dev, config.left.format, width, height)?;
    let mut left_stream =
        MmapStream::with_buffers(&left_dev, Type::VideoCapture, config.left.buffers)?;

    let right_dev = Device::with_path(&config.right.device).map_err(|e| {
        Error::from(e)
            .with_operation("open camera")
            .with_context("device", &config.right.device)
    })?;
    configure(&right_dev, config.right.format, width, height)?;
    let mut right_stream =
        MmapStream::with_buffers(&right_dev, Type::VideoCapture, config.right.buffers)?;

    left_stream.set_timeout(std::time::Duration::from_millis(100));
    right_stream.set_timeout(std::time::Duration::from_millis(100));

    while RUNNING.load(Ordering::Relaxed) {
        let stamp = monotonic_seconds()?;
        pump_gray(
            &mut left_stream,
            &left_pub,
            &config.left,
            width,
            height,
            0,
            stamp,
        )?;
        pump_gray(
            &mut right_stream,
            &right_pub,
            &config.right,
            width,
            height,
            1,
            stamp,
        )?;
        firefly_observability::pump_log_ipc(&log_ipc);
    }
    if let Some(down) = down {
        let result = down.finish();
        firefly_observability::pump_log_ipc(&log_ipc);
        result?;
    }
    log::info!("收到终止信号，相机源退出");
    firefly_observability::pump_log_ipc(&log_ipc);
    Ok(())
}

/// 按目标格式协商设备像素格式；驱动必须原样接受，否则说明内核不支持该组合。
fn configure(device: &Device, format: FrameFormat, width: usize, height: usize) -> Result<()> {
    let requested = Format::new(width as u32, height as u32, fourcc(format));
    let actual = device.set_format(&requested)?;
    if actual.width != requested.width
        || actual.height != requested.height
        || actual.fourcc != requested.fourcc
    {
        return Err(Error::new(
            ErrorKind::Unsupported,
            format!(
                "设备不接受 {width}×{height} {}：驱动改为 {}×{} {}",
                requested.fourcc, actual.width, actual.height, actual.fourcc
            ),
        ));
    }
    Ok(())
}

fn fourcc(format: FrameFormat) -> FourCC {
    match format {
        FrameFormat::Grey => FourCC::new(b"GREY"),
        FrameFormat::Yuyv => FourCC::new(b"YUYV"),
        FrameFormat::Z16 => FourCC::new(b"Z16 "),
    }
}

/// 取一帧灰度并发布；帧长不足协商格式时丢帧并告警。
fn pump_gray(
    stream: &mut MmapStream<'_>,
    publisher: &Publisher<GrayImageMessage>,
    source: &Source,
    width: usize,
    height: usize,
    sensor_id: i32,
    stamp: f64,
) -> Result<()> {
    let (bytes, _meta) = match stream.next() {
        Ok(frame) => frame,
        Err(error)
            if matches!(
                error.kind(),
                std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
            ) =>
        {
            return Ok(());
        }
        Err(error) => return Err(error.into()),
    };
    let expected = width * height * source.format.bytes_per_pixel();
    if bytes.len() < expected {
        log::warn!(
            "{} 帧长 {} < 期望 {expected}（{}×{} {:?}），丢帧",
            source.device,
            bytes.len(),
            width,
            height,
            source.format
        );
        return Ok(());
    }
    let pixels = width * height;
    let mut written = pixels;
    publisher.publish_with(|message| {
        message.timestamp = stamp;
        message.sensor_id = sensor_id;
        message.width = width as u32;
        message.height = height as u32;
        match source.format {
            FrameFormat::Grey => {
                message.data[..pixels].copy_from_slice(&bytes[..pixels]);
            }
            FrameFormat::Yuyv => {
                written = gray_from_yuyv(&bytes[..expected], &mut message.data[..pixels]);
            }
            FrameFormat::Z16 => {}
        }
    })?;
    if written != pixels {
        log::warn!(
            "{} 只解出 {written}/{pixels} 个像素（{:?}），图像不完整",
            source.device,
            source.format
        );
    }
    Ok(())
}

/// `CLOCK_MONOTONIC` 秒：板载各进程共用的时间基准。
fn monotonic_seconds() -> Result<f64> {
    let time = Time::now_with_clock(ClockType::Monotonic)
        .map_err(|error| Error::caused_by(ErrorKind::Internal, "read monotonic clock", error))?;
    Ok(time.as_duration().as_secs_f64())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shipped_config_parses() {
        let raw = include_str!("../../../../configs/camera.toml");
        let config: Config = toml::from_str(raw).expect("configs/camera.toml");
        validate(&config).expect("valid camera config");
    }

    #[test]
    fn resolution_must_match_message_layout() {
        let raw = include_str!("../../../../configs/camera.toml");
        let mut config: Config = toml::from_str(raw).expect("configs/camera.toml");
        config.width = 640;
        assert!(validate(&config).is_err());
    }
}
