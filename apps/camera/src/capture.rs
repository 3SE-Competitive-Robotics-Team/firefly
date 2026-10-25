//! `V4L2` 采集主循环与配置。

use std::error::Error;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};

use firefly_hal::pixel::{FrameFormat, gray_from_yuyv, meters_from_z16};
use firefly_pubsub::camera::{
    CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, DEPTH_TOPIC, DepthImageMessage, GrayImageMessage,
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

/// `configs/camera.toml` 缺省路径（相对运行目录，板上是 `/opt/firefly`）。
/// 缺省配置：按安装布局解析，与启动目录无关。
fn default_config() -> PathBuf {
    firefly_config::paths::config("camera.toml")
}
/// 默认采集缓冲数；太少容易在调度抖动时丢帧。
const DEFAULT_BUFFERS: u32 = 4;
/// `Z16` 深度默认按毫米缩放。
const DEFAULT_DEPTH_SCALE: f64 = 0.001;

/// 退出标志（信号处理器置位，主循环每帧检查）。
static RUNNING: AtomicBool = AtomicBool::new(true);

/// 相机配置。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    /// 图像宽度（像素）；必须与消息布局一致。
    #[serde(default = "default_width")]
    width: u32,
    /// 图像高度（像素）；必须与消息布局一致。
    #[serde(default = "default_height")]
    height: u32,
    /// 左目相机。
    left: Source,
    /// 右目相机。
    right: Source,
    /// 深度相机；省略则不发布 `Firefly/Depth`。
    #[serde(default)]
    depth: Option<Depth>,
}

/// 灰度相机设备。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Source {
    /// `V4L2` 设备节点。
    device: String,
    /// 采集像素格式。
    #[serde(default = "default_format")]
    format: FrameFormat,
    /// 采集缓冲数。
    #[serde(default = "default_buffers")]
    buffers: u32,
}

/// 深度相机设备。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Depth {
    /// `V4L2` 设备节点。
    device: String,
    /// `Z16` 原始单位 → 米。
    #[serde(default = "default_depth_scale")]
    scale: f64,
    /// 采集缓冲数。
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

/// 校验配置与消息布局、采集格式的相容性。
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
    let depth_scale = config.depth.as_ref().map(|depth| depth.scale);

    while RUNNING.load(Ordering::Relaxed) {
        // 三路共用一次采样时刻：与仿真「一次渲染出三类图」的语义一致，
        // 双目配对的 dt 因此为零。
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

/// 像素格式对应的 `V4L2` 四字符码。
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
    let (bytes, _meta) = stream.next()?;
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
    let (bytes, _meta) = stream.next()?;
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

    /// 仓库里的配置必须能被本进程解析并通过校验：板端启动才发现漂移太晚。
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
