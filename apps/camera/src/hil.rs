//! HIL 传感器接入：从 USB `FunctionFS` 端点读取仿真传感器并发布到本机话题。
//!
//! 时间戳沿用主机下发的仿真时间轴；本进程只做转接，不重排测量时刻。
//! 图像按 `ImageChunk` 分块到达，块间的 IMU 立即发布；收齐后直接填进
//! iceoryx2 共享内存（`publish_with`），不再 decode 成中间消息再拷一次。

use std::fs::OpenOptions;
use std::io::ErrorKind;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use firefly_hil::frame::FrameReader;
use firefly_hil::gadget::{endpoint_file_at, touch_host_seen};
use firefly_hil::protocol::{IMAGE_CHUNK_HEADER_LEN, ImageChunkHeader, Kind, Message};
use firefly_pubsub::camera::{
    CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, DEPTH_TOPIC, DepthImageMessage, GrayImageMessage,
    IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH,
};
use firefly_pubsub::command::{COMMAND_TOPIC, CommandMessage};
use firefly_pubsub::goal::{GOAL_TOPIC, GoalMessage};
use firefly_pubsub::imu::{IMU_TOPIC, ImuMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::plant::{AIRFRAME_TOPIC, AirframeMessage};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::viz::{SCALAR_BUFFER_SIZE, SCALAR_TOPIC, ScalarMessage, VIZ_MAX_PUBLISHERS};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalGuard, SignalHandler};
use nix::fcntl::{FcntlArg, OFlag, fcntl};

/// HIL 功能实例名。
const HIL_INSTANCE: &str = "hil";
/// 下行端点在功能内的序号（第一个数据端点）。
const DOWNLINK_ENDPOINT: usize = 0;
/// 图像订阅端容量上限；与消费者声明的缓冲一致。
const IMAGE_SUBSCRIPTION: usize = 8;
/// IMU 订阅端容量上限（与 `IMU_SUBSCRIPTION` 一致）。
const IMU_SUBSCRIPTION_MAX: usize = 128;
/// 无数据轮询间隔；比 1ms 小，减少 IMU 到达的额外驻留。
const POLL: Duration = Duration::from_micros(200);
/// 主机心跳刷新间隔。
const HOST_HEARTBEAT: Duration = Duration::from_millis(500);

static RUNNING: AtomicBool = AtomicBool::new(true);

struct Ports {
    imu: Publisher<ImuMessage>,
    left: Publisher<GrayImageMessage>,
    right: Publisher<GrayImageMessage>,
    depth: Publisher<DepthImageMessage>,
    airframe: Publisher<AirframeMessage>,
    command: Publisher<CommandMessage>,
    goal: Publisher<GoalMessage>,
    /// 图像校验和回传：主机据此确认 USB 传输没有损坏像素。
    scalars: Publisher<ScalarMessage>,
}

pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    let _guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_signal| RUNNING.store(false, Ordering::Relaxed),
    )?;
    let node = create_node()?;
    let _log_ipc = firefly_observability::init_ipc(&node, "camera");
    let ports = Ports {
        imu: Publisher::with_topic_and_buffer(&node, IMU_TOPIC, Some(IMU_SUBSCRIPTION_MAX))?,
        left: Publisher::with_topic_and_buffer(&node, CAMERA_LEFT_TOPIC, Some(IMAGE_SUBSCRIPTION))?,
        right: Publisher::with_topic_and_buffer(
            &node,
            CAMERA_RIGHT_TOPIC,
            Some(IMAGE_SUBSCRIPTION),
        )?,
        depth: Publisher::with_topic_and_buffer(&node, DEPTH_TOPIC, Some(IMAGE_SUBSCRIPTION))?,
        airframe: Publisher::with_topic_and_buffer(&node, AIRFRAME_TOPIC, Some(4))?,
        command: Publisher::with_topic_and_buffer(&node, COMMAND_TOPIC, Some(4))?,
        goal: Publisher::with_topic_and_buffer(&node, GOAL_TOPIC, Some(4))?,
        scalars: Publisher::with_topic_buffer_publishers(
            &node,
            SCALAR_TOPIC,
            Some(SCALAR_BUFFER_SIZE),
            Some(VIZ_MAX_PUBLISHERS),
        )?,
    };
    let path = endpoint_file_at(HIL_INSTANCE, DOWNLINK_ENDPOINT)?;
    let file = OpenOptions::new().read(true).open(&path)?;
    let flags = OFlag::from_bits_truncate(fcntl(&file, FcntlArg::F_GETFL)?);
    fcntl(&file, FcntlArg::F_SETFL(flags | OFlag::O_NONBLOCK))?;
    log::info!("HIL 传感器端点 {}", path.display());

    let mut reader = FrameReader::new(file);
    let mut assembly: Option<ChunkAssembly> = None;
    let mut last_heartbeat = Instant::now();
    let mut imu_published = 0_u64;
    let mut last_imu_log = Instant::now();
    while RUNNING.load(Ordering::Relaxed) {
        match reader.read_raw() {
            Ok((header, body)) => {
                if last_imu_log.elapsed() >= Duration::from_secs(1) {
                    log::debug!("camera --hil 已发布 IMU {imu_published} 条");
                    last_imu_log = Instant::now();
                }
                if last_heartbeat.elapsed() >= HOST_HEARTBEAT {
                    touch_host_seen();
                    last_heartbeat = Instant::now();
                }
                if header.kind == Kind::ImageChunk {
                    let chunk_header = ImageChunkHeader::decode(&body)?;
                    handle_chunk(&mut assembly, chunk_header, body, &ports)?;
                } else {
                    let message = Message::decode(header.kind, &body)?;
                    imu_published += dispatch(&message, &ports)? as u64;
                }
            }
            Err(error) if error.kind() == ErrorKind::WouldBlock => std::thread::sleep(POLL),
            Err(error) if error.kind() == ErrorKind::UnexpectedEof => {
                log::error!("HIL 传感器端点关闭");
                break;
            }
            Err(error) => return Err(error.into()),
        }
    }
    log::info!("camera --hil 退出");
    Ok(())
}

/// 一张图的块重组状态。
struct ChunkAssembly {
    image_id: u32,
    sensor_id: u8,
    timestamp: f64,
    chunks: Vec<Option<Vec<u8>>>,
    received: usize,
}

fn handle_chunk(
    assembly: &mut Option<ChunkAssembly>,
    header: ImageChunkHeader,
    body: Vec<u8>,
    ports: &Ports,
) -> Result<(), Box<dyn std::error::Error>> {
    let restart = match assembly.as_ref() {
        Some(current) => {
            current.image_id != header.image_id || current.sensor_id != header.sensor_id
        }
        None => true,
    };
    if restart {
        *assembly = Some(ChunkAssembly {
            image_id: header.image_id,
            sensor_id: header.sensor_id,
            timestamp: header.timestamp,
            chunks: (0..header.chunk_count).map(|_| None).collect(),
            received: 0,
        });
    }
    let Some(current) = assembly.as_mut() else {
        return Ok(());
    };
    if usize::from(header.chunk_count) != current.chunks.len() {
        return Ok(());
    }
    let index = usize::from(header.chunk_index);
    if current.chunks[index].is_none() {
        current.chunks[index] = Some(body);
        current.received += 1;
    }
    if current.received == current.chunks.len() {
        let completed = assembly.take().expect("收齐后取出");
        publish_assembled(ports, completed)?;
    }
    Ok(())
}

fn publish_assembled(
    ports: &Ports,
    image: ChunkAssembly,
) -> Result<(), Box<dyn std::error::Error>> {
    let ChunkAssembly {
        sensor_id,
        timestamp,
        chunks,
        ..
    } = image;
    let payload_len = |chunk: &Option<Vec<u8>>| {
        chunk
            .as_ref()
            .map_or(0, |body| body.len().saturating_sub(IMAGE_CHUNK_HEADER_LEN))
    };
    let total: usize = chunks.iter().map(payload_len).sum();
    let width = u32::try_from(IMAGE_WIDTH).unwrap_or(0);
    let height = u32::try_from(IMAGE_HEIGHT).unwrap_or(0);
    match sensor_id {
        0 | 1 => {
            if total != IMAGE_SIZE {
                log::warn!("图像块总长 {total} != {IMAGE_SIZE}，丢弃");
                return Ok(());
            }
            let mut sum = 0_u64;
            let publisher = if sensor_id == 0 {
                &ports.left
            } else {
                &ports.right
            };
            publisher.publish_with(|message| {
                message.timestamp = timestamp;
                message.sensor_id = i32::from(sensor_id);
                message.width = width;
                message.height = height;
                let mut offset = 0;
                for chunk in &chunks {
                    let Some(body) = chunk.as_ref() else { continue };
                    let payload = &body[IMAGE_CHUNK_HEADER_LEN..];
                    let end = offset + payload.len();
                    message.data[offset..end].copy_from_slice(payload);
                    offset = end;
                    for &byte in payload {
                        sum += u64::from(byte);
                    }
                }
            })?;
            let side = if sensor_id == 0 { "left" } else { "right" };
            let mut scalar = ScalarMessage::base(timestamp, &format!("hil/camera/{side}"));
            scalar.scalars = [timestamp, sum as f64, f64::from(width), f64::from(height)];
            scalar.scalar_count = 4;
            ports.scalars.publish(scalar)?;
        }
        2 => {
            let expected = IMAGE_SIZE * 4;
            if total != expected {
                log::warn!("深度块总长 {total} != {expected}，丢弃");
                return Ok(());
            }
            let mut offset = 0_usize;
            ports.depth.publish_with(|message| {
                message.timestamp = timestamp;
                message.sensor_id = 0;
                message.width = width;
                message.height = height;
                for chunk in &chunks {
                    let Some(body) = chunk.as_ref() else { continue };
                    for value in body[IMAGE_CHUNK_HEADER_LEN..].as_chunks::<4>().0 {
                        message.data[offset] = f32::from_le_bytes(*value);
                        offset += 1;
                    }
                }
            })?;
        }
        other => log::warn!("未知图像传感器 {other}"),
    }
    Ok(())
}

/// 发布图像校验和：`[时间戳, 像素和, 宽, 高]`，主机按同一算法比对。
fn publish_image_checksum(
    ports: &Ports,
    image: &firefly_pubsub::camera::GrayImageMessage,
) -> Result<(), Box<dyn std::error::Error>> {
    let sum: u64 = image.data.iter().map(|&pixel| u64::from(pixel)).sum();
    let side = if image.sensor_id == 0 {
        "left"
    } else {
        "right"
    };
    let mut scalar = ScalarMessage::base(image.timestamp, &format!("hil/camera/{side}"));
    scalar.scalars = [
        image.timestamp,
        sum as f64,
        f64::from(image.width),
        f64::from(image.height),
    ];
    scalar.scalar_count = 4;
    ports.scalars.publish(scalar)?;
    Ok(())
}

fn dispatch(message: &Message, ports: &Ports) -> Result<usize, Box<dyn std::error::Error>> {
    let mut imu = 0;
    match message {
        Message::Imu(samples) => {
            for sample in samples {
                ports.imu.publish(*sample)?;
                imu += 1;
            }
        }
        Message::Gray(image) => {
            if image.sensor_id == 0 {
                ports.left.publish(**image)?;
            } else {
                ports.right.publish(**image)?;
            }
            publish_image_checksum(ports, image)?;
        }
        Message::Depth(image) => {
            ports.depth.publish(**image)?;
        }
        Message::Airframe(airframe) => {
            ports.airframe.publish(*airframe)?;
        }
        Message::Command(command) => {
            log::info!(
                "camera --hil 收到指令 kind={} seq={}",
                command.kind,
                command.sequence
            );
            ports.command.publish(*command)?;
        }
        Message::Goal(goal) => {
            ports.goal.publish(*goal)?;
        }
        // 会话管理帧不承载算法数据；心跳只用于刷新主机存活。
        Message::Hello(_) | Message::Ping(_) | Message::Bye(_) => {}
        other => log::warn!("camera --hil 忽略 {}", other.kind().name()),
    }
    Ok(imu)
}
