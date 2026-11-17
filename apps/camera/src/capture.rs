use std::error::Error;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};

use firefly_hal::pixel::{FrameFormat, gray_from_yuyv, meters_from_z16};
use firefly_pubsub::camera::FrameCadence;
use firefly_pubsub::camera::{
    CAMERA_DOWN_TOPIC, CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, DEPTH_TOPIC, DOWN_HEIGHT,
    DOWN_SENSOR_ID, DOWN_SIZE, DOWN_WIDTH, DepthImageMessage, DownImageMessage, GrayImageMessage,
    IMAGE_HEIGHT, IMAGE_WIDTH,
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

fn default_config() -> PathBuf {
    firefly_config::paths::config("camera.toml")
}
/// 默认采集缓冲数；太少容易在调度抖动时丢帧。
const DEFAULT_BUFFERS: u32 = 4;
/// `Z16` 深度默认按毫米缩放。
const DEFAULT_DEPTH_SCALE: f64 = 0.001;

static RUNNING: AtomicBool = AtomicBool::new(true);

struct DownWorker(Option<std::thread::JoinHandle<Result<(), String>>>);
impl DownWorker {
    fn finish(mut self) -> Result<(), Box<dyn Error>> {
        if let Some(worker) = self.0.take() {
            worker
                .join()
                .map_err(|_| "down camera worker panicked")?
                .map_err(Into::into)
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

fn start_down(source: &DownSource) -> Result<DownWorker, Box<dyn Error>> {
    let source = source.clone();
    let (ready, started) = std::sync::mpsc::sync_channel(1);
    let worker = std::thread::spawn(move || {
        let result = (|| -> Result<(), Box<dyn Error>> {
            let node = create_node()?;
            let publisher = Publisher::<DownImageMessage>::with_topic(&node, CAMERA_DOWN_TOPIC)?;
            let _ = ready.send(Ok(()));
            capture_down(&source, &publisher)?;
            Ok(())
        })();
        match result {
            Ok(()) => Ok(()),
            Err(error) => {
                log::error!("down camera: {error}");
                let _ = ready.send(Err(error.to_string()));
                RUNNING.store(false, Ordering::Relaxed);
                Err(error.to_string())
            }
        }
    });
    let guard = DownWorker(Some(worker));
    started
        .recv()
        .map_err(|e| e.to_string())?
        .map_err(|e| -> Box<dyn Error> { e.into() })?;
    Ok(guard)
}

/// 下视采集流随进程持续运行，退出作用域时 Drop 执行 STREAMOFF。
fn capture_down(
    source: &DownSource,
    publisher: &Publisher<DownImageMessage>,
) -> Result<(), Box<dyn Error>> {
    let rate = source.rate_hz;
    let device = Device::with_path(&source.device)?;
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
            return Err("down camera requires V4L2 CLOCK_MONOTONIC capture timestamps".into());
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
                    return Err("incomplete down YUYV row".into());
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
    /// 深度相机；省略则不发布 `Firefly/Depth`。
    #[serde(default)]
    depth: Option<Depth>,
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

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Depth {
    device: String,
    /// `Z16` 原始单位 → 米。
    #[serde(default = "default_depth_scale")]
    scale: f64,
    #[serde(default = "default_buffers")]
    buffers: u32,
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

const fn default_depth_scale() -> f64 {
    DEFAULT_DEPTH_SCALE
}

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
    if let Some(depth) = &config.depth {
        if !depth.scale.is_finite() || depth.scale <= 0.0 {
            return invalid(format!("depth 的 scale 必须为正：{}", depth.scale));
        }
        if depth.buffers == 0 {
            return invalid("depth 的 buffers 必须为正".to_owned());
        }
    }
    Ok(())
}

/// 采集主循环：三路流各取一帧发布，直到收到 `SIGINT`/`SIGTERM`。
pub fn run() -> Result<(), Box<dyn Error>> {
    let config_path =
        parse_config_path().map_err(|error| format!("{error}；用法：camera [--config <path>]"))?;
    let config: Config =
        firefly_config::startup(&config_path, firefly_observability::init, validate)?;
    let width = config.width as usize;
    let height = config.height as usize;

    let _signal_guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_signal| {
            RUNNING.store(false, Ordering::Relaxed);
        },
    )?;

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

    let left_dev = Device::with_path(&config.left.device)?;
    configure(&left_dev, config.left.format, width, height)?;
    let mut left_stream =
        MmapStream::with_buffers(&left_dev, Type::VideoCapture, config.left.buffers)?;

    let right_dev = Device::with_path(&config.right.device)?;
    configure(&right_dev, config.right.format, width, height)?;
    let mut right_stream =
        MmapStream::with_buffers(&right_dev, Type::VideoCapture, config.right.buffers)?;

    let depth_dev = match &config.depth {
        Some(depth) => {
            let device = Device::with_path(&depth.device)?;
            configure(&device, FrameFormat::Z16, width, height)?;
            log::info!("深度源：{}", depth.device);
            Some(device)
        }
        None => None,
    };
    let mut depth_stream = match (&depth_dev, &config.depth) {
        (Some(device), Some(depth)) => Some(MmapStream::with_buffers(
            device,
            Type::VideoCapture,
            depth.buffers,
        )?),
        _ => None,
    };
    let depth_pub = match &config.depth {
        Some(_) => Some(Publisher::<DepthImageMessage>::with_topic(
            &node,
            DEPTH_TOPIC,
        )?),
        None => None,
    };
    left_stream.set_timeout(std::time::Duration::from_millis(100));
    right_stream.set_timeout(std::time::Duration::from_millis(100));
    if let Some(stream) = &mut depth_stream {
        stream.set_timeout(std::time::Duration::from_millis(100));
    }
    let depth_scale = config.depth.as_ref().map(|depth| depth.scale);

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
        if let (Some(stream), Some(publisher), Some(scale)) =
            (depth_stream.as_mut(), depth_pub.as_ref(), depth_scale)
        {
            pump_depth(stream, publisher, scale, width, height, stamp)?;
        }
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
fn configure(
    device: &Device,
    format: FrameFormat,
    width: usize,
    height: usize,
) -> Result<(), Box<dyn Error>> {
    let requested = Format::new(width as u32, height as u32, fourcc(format));
    let actual = device.set_format(&requested)?;
    if actual.width != requested.width
        || actual.height != requested.height
        || actual.fourcc != requested.fourcc
    {
        return Err(format!(
            "设备不接受 {width}×{height} {}：驱动改为 {}×{} {}",
            requested.fourcc, actual.width, actual.height, actual.fourcc
        )
        .into());
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
) -> Result<(), Box<dyn Error>> {
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

/// 取一帧深度并发布（米制）；帧长不足时丢帧并告警。
fn pump_depth(
    stream: &mut MmapStream<'_>,
    publisher: &Publisher<DepthImageMessage>,
    scale: f64,
    width: usize,
    height: usize,
    stamp: f64,
) -> Result<(), Box<dyn Error>> {
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
    let pixels = width * height;
    if bytes.len() < pixels * 2 {
        log::warn!(
            "深度帧长 {} < 期望 {}（{width}×{height} z16），丢帧",
            bytes.len(),
            pixels * 2
        );
        return Ok(());
    }
    let scale = scale as f32;
    let mut written = pixels;
    publisher.publish_with(|message| {
        message.timestamp = stamp;
        message.sensor_id = 0;
        message.width = width as u32;
        message.height = height as u32;
        written = meters_from_z16(&bytes[..pixels * 2], &mut message.data[..pixels], scale);
    })?;
    if written != pixels {
        log::warn!("深度只解出 {written}/{pixels} 个像素，图像不完整");
    }
    Ok(())
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

    #[test]
    fn shipped_config_parses() {
        let raw = include_str!("../../../configs/camera.toml");
        let config: Config = toml::from_str(raw).expect("configs/camera.toml");
        validate(&config).expect("valid camera config");
    }

    #[test]
    fn resolution_must_match_message_layout() {
        let raw = include_str!("../../../configs/camera.toml");
        let mut config: Config = toml::from_str(raw).expect("configs/camera.toml");
        config.width = 640;
        assert!(validate(&config).is_err());
    }
}
