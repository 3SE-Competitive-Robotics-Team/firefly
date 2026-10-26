//! 主机侧链路运行时：USB 端点与本地 IPC 话题双向转接。
//!
//! iceoryx2 端口是单线程对象，因此主线程独占全部发布/订阅端口，USB 读与写
//! 各由一个线程持有；三方通过有界 `mpsc` 通道交接消息。图像走 `try_send`，
//! 满队列即丢当前帧；IMU、控制与指令走阻塞 `send`，不静默丢采样。

use std::collections::VecDeque;
use std::io::ErrorKind;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use firefly_hil::frame::{FrameReader, FrameWriter};
use firefly_hil::protocol::{Hello, Message, ROLE_HOST, VERSION};
use firefly_pubsub::attitude::{ATTITUDE_TOPIC, AttitudeMessage};
use firefly_pubsub::camera::{
    CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, DEPTH_TOPIC, DepthImageMessage, GrayImageMessage,
};
use firefly_pubsub::command::{COMMAND_TOPIC, CommandMessage};
use firefly_pubsub::control::{CONTROL_BUFFER_SIZE, CONTROL_TOPIC, ControlMessage};
use firefly_pubsub::goal::{GOAL_TOPIC, GoalMessage};
use firefly_pubsub::imu::{IMU_TOPIC, ImuMessage};
use firefly_pubsub::node::{IpcNode, create_node};
use firefly_pubsub::odom::{CORRECTED_ODOM_TOPIC, ODOM_TOPIC, OdomMessage};
use firefly_pubsub::plant::{AIRFRAME_TOPIC, AirframeMessage};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::{Subscriber, SubscriberOptions};
use firefly_pubsub::viz::{SCALAR_BUFFER_SIZE, SCALAR_TOPIC, ScalarMessage};
use nusb::io::{EndpointRead, EndpointWrite};
use nusb::transfer::Bulk;

use crate::config::Config;
use crate::host::HostLink;

/// 主机下行订阅容量（IMU 覆盖 100Hz 下约 1s 排空停顿）。
const IMU_SUBSCRIPTION: usize = 100;
/// 图像订阅容量：只关心最新帧。
const IMAGE_SUBSCRIPTION: usize = 2;
/// 下行发送队列深度；满时图像丢帧、IMU/指令阻塞。
const DOWNLINK_QUEUE: usize = 64;
/// 上行接收队列深度。
const UPLINK_QUEUE: usize = 1024;
/// IPC 轮询间隔。
const POLL: Duration = Duration::from_millis(1);
/// 统计标量发布周期。
const STATS_PERIOD: Duration = Duration::from_secs(1);
/// 双目配对的采集时间容差（秒）：与 okvis `stereo_tolerance` 一致。
const STEREO_TOLERANCE_S: f64 = 0.001;

struct Downlink {
    imu: Subscriber<ImuMessage>,
    left: Subscriber<GrayImageMessage>,
    right: Subscriber<GrayImageMessage>,
    depth: Subscriber<DepthImageMessage>,
    airframe: Subscriber<AirframeMessage>,
    command: Subscriber<CommandMessage>,
    goal: Subscriber<GoalMessage>,
}

struct Uplink {
    control: Publisher<ControlMessage>,
    odom: Publisher<OdomMessage>,
    corrected: Publisher<OdomMessage>,
    attitude: Publisher<AttitudeMessage>,
    scalar: Publisher<ScalarMessage>,
}

/// 运行双向桥，直到收到退出信号或 USB 断开。
///
/// # Errors
/// 设备缺席、端点不可用、IPC 端口创建失败或链路读写出错。
pub fn run(config: &Config) -> Result<(), String> {
    let link = HostLink::open(config)?;
    let (writer, reader, device, interface) =
        link.split(config.link.write_buffer, config.link.read_buffer)?;
    // 设备与接口句柄必须活到两个端点线程结束，否则 macOS 可能在端点仍在使用时释放设备。
    let _keep = (device, interface);
    let node = create_node().map_err(|error| error.to_string())?;
    let (down, up) = open_ports(&node)?;
    let session = session_id();
    let stop = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        signal_hook::flag::register(signal, Arc::clone(&stop))
            .map_err(|error| format!("register signal: {error}"))?;
    }
    let (down_tx, down_rx) = sync_channel(DOWNLINK_QUEUE);
    let (up_tx, up_rx) = sync_channel(UPLINK_QUEUE);
    let writer_stop = Arc::clone(&stop);
    let writer = std::thread::spawn(move || usb_writer(writer, down_rx, session, writer_stop));
    let reader_stop = Arc::clone(&stop);
    let reader = std::thread::spawn(move || usb_reader(reader, up_tx, reader_stop));
    log::info!("HIL 链路已打开，session={session:#x}");
    let result = ipc_loop(down, up, &down_tx, &up_rx, &stop, &config.link);
    drop(down_tx);
    let writer = writer.join().map_err(|_| "USB 写线程 panic".to_owned())?;
    let reader = reader.join().map_err(|_| "USB 读线程 panic".to_owned())?;
    writer?;
    reader?;
    result?;
    log::info!("HIL 链路已退出");
    Ok(())
}

fn open_ports(node: &IpcNode) -> Result<(Downlink, Uplink), String> {
    let down = Downlink {
        imu: Subscriber::with_options(
            node,
            IMU_TOPIC,
            SubscriberOptions {
                buffer_size: IMU_SUBSCRIPTION,
                service_max_buffer_size: Some(128),
                // sim 是唯一发布者；显式声明避免先建服务时把上限压到默认值。
                max_publishers: Some(2),
            },
        )
        .map_err(|error| error.to_string())?,
        left: Subscriber::with_topic_and_buffer(node, CAMERA_LEFT_TOPIC, IMAGE_SUBSCRIPTION)
            .map_err(|error| error.to_string())?,
        right: Subscriber::with_topic_and_buffer(node, CAMERA_RIGHT_TOPIC, IMAGE_SUBSCRIPTION)
            .map_err(|error| error.to_string())?,
        depth: Subscriber::with_topic_and_buffer(node, DEPTH_TOPIC, IMAGE_SUBSCRIPTION)
            .map_err(|error| error.to_string())?,
        airframe: Subscriber::with_options(
            node,
            AIRFRAME_TOPIC,
            SubscriberOptions {
                buffer_size: 4,
                service_max_buffer_size: Some(4),
                max_publishers: Some(2),
            },
        )
        .map_err(|error| error.to_string())?,
        command: Subscriber::with_options(
            node,
            COMMAND_TOPIC,
            SubscriberOptions {
                buffer_size: 8,
                service_max_buffer_size: Some(8),
                max_publishers: Some(2),
            },
        )
        .map_err(|error| error.to_string())?,
        goal: Subscriber::with_options(
            node,
            GOAL_TOPIC,
            SubscriberOptions {
                buffer_size: 4,
                service_max_buffer_size: Some(4),
                max_publishers: Some(2),
            },
        )
        .map_err(|error| error.to_string())?,
    };
    let up = Uplink {
        control: Publisher::with_topic_and_buffer(node, CONTROL_TOPIC, Some(CONTROL_BUFFER_SIZE))
            .map_err(|error| error.to_string())?,
        odom: Publisher::with_topic(node, ODOM_TOPIC).map_err(|error| error.to_string())?,
        corrected: Publisher::with_topic(node, CORRECTED_ODOM_TOPIC)
            .map_err(|error| error.to_string())?,
        attitude: Publisher::with_topic(node, ATTITUDE_TOPIC).map_err(|error| error.to_string())?,
        scalar: Publisher::with_topic_buffer_publishers(
            node,
            SCALAR_TOPIC,
            Some(SCALAR_BUFFER_SIZE),
            Some(8),
        )
        .map_err(|error| error.to_string())?,
    };
    Ok((down, up))
}

// 线程入口按值取得端口与停止标志；所有权随 `move` 闭包转移。
#[allow(clippy::needless_pass_by_value)]
fn usb_writer(
    writer: EndpointWrite<Bulk>,
    queue: Receiver<Message>,
    session: u64,
    stop: Arc<AtomicBool>,
) -> Result<(), String> {
    let mut writer = FrameWriter::new(writer);
    writer.set_session(session);
    writer
        .write_message(&Message::Hello(Hello {
            version: VERSION,
            role: ROLE_HOST,
            session,
        }))
        .map_err(|error| format!("写 hello: {error}"))?;
    while let Ok(message) = queue.recv() {
        if stop.load(Ordering::Relaxed) {
            break;
        }
        writer
            .write_message(&message)
            .map_err(|error| format!("写 {}: {error}", message.kind().name()))?;
    }
    Ok(())
}

#[allow(clippy::needless_pass_by_value)]
fn usb_reader(
    reader: EndpointRead<Bulk>,
    queue: SyncSender<Message>,
    stop: Arc<AtomicBool>,
) -> Result<(), String> {
    let mut reader = FrameReader::new(reader);
    while !stop.load(Ordering::Relaxed) {
        match reader.read_frame() {
            Ok(frame) => {
                if queue.send(frame.message).is_err() {
                    break;
                }
            }
            Err(error)
                if matches!(
                    error.kind(),
                    ErrorKind::TimedOut | ErrorKind::WouldBlock | ErrorKind::Interrupted
                ) => {}
            Err(error) => return Err(format!("读执行器端点: {error}")),
        }
    }
    Ok(())
}

#[allow(clippy::too_many_lines)] // 单一 IPC 轮询循环，分支与消息类型一一对应
#[allow(clippy::needless_pass_by_value)] // 端口所有权随本轮循环结束释放
fn ipc_loop(
    down: Downlink,
    up: Uplink,
    down_tx: &SyncSender<Message>,
    up_rx: &Receiver<Message>,
    stop: &AtomicBool,
    options: &crate::config::Link,
) -> Result<(), String> {
    let image_period = Duration::from_secs_f64(options.image_period_s);
    let mut last_image = Instant::now();
    let mut pending_left: Option<GrayImageMessage> = None;
    let mut pending_right: Option<GrayImageMessage> = None;
    let mut pending_depth: Option<DepthImageMessage> = None;
    let mut last_imu_stamp = f64::NAN;
    let mut last_image_stamp = f64::NAN;
    let mut last_timestamp_log = Instant::now();
    let mut stats = StatsCounters::default();
    let mut last_stats = Instant::now();
    let mut last_heartbeat = Instant::now();
    let mut session = 0_u64;
    let mut imu_batch: Vec<ImuMessage> = Vec::with_capacity(options.imu_batch);
    while !stop.load(Ordering::Relaxed) {
        let mut activity = false;
        while let Ok(Some(sample)) = down.command.receive() {
            let command = *sample.payload();
            log::info!(
                "hil 转发指令 kind={} seq={}",
                command.kind,
                command.sequence
            );
            send_blocking(down_tx, Message::Command(command))?;
            activity = true;
        }
        while let Ok(Some(sample)) = down.goal.receive() {
            send_blocking(down_tx, Message::Goal(*sample.payload()))?;
            activity = true;
        }
        while let Ok(Some(sample)) = down.airframe.receive() {
            send_blocking(down_tx, Message::Airframe(*sample.payload()))?;
            activity = true;
        }
        while imu_batch.len() < options.imu_batch {
            match down.imu.receive() {
                Ok(Some(sample)) => imu_batch.push(*sample.payload()),
                _ => break,
            }
        }
        if !imu_batch.is_empty() {
            last_imu_stamp = imu_batch
                .last()
                .map_or(last_imu_stamp, |sample| sample.timestamp);
            send_blocking(down_tx, Message::Imu(std::mem::take(&mut imu_batch)))?;
            activity = true;
        }
        // 图像按周期节流：板端 VIO 处理不过来的帧在主机侧丢弃，不排队进 USB。
        // 双目必须成对发送，缺右目会让 VIO 无法初始化。
        while let Ok(Some(sample)) = down.left.receive() {
            pending_left = Some(*sample.payload());
            activity = true;
        }
        while let Ok(Some(sample)) = down.right.receive() {
            pending_right = Some(*sample.payload());
            activity = true;
        }
        while let Ok(Some(sample)) = down.depth.receive() {
            if options.depth {
                pending_depth = Some(*sample.payload());
            }
            activity = true;
        }
        if let (Some(left), Some(right)) = (&pending_left, &pending_right) {
            // 双目必须同帧：时间戳不一致的旧帧直接丢弃，不能凑成一对。
            if (left.timestamp - right.timestamp).abs() > crate::bridge::STEREO_TOLERANCE_S {
                if left.timestamp < right.timestamp {
                    pending_left = None;
                } else {
                    pending_right = None;
                }
                stats.dropped_images += 1;
            } else if last_image.elapsed() >= image_period {
                let left = pending_left.take().expect("已检查非空");
                let right = pending_right.take().expect("已检查非空");
                record_image_checksum(&mut stats, &left);
                record_image_checksum(&mut stats, &right);
                try_image(down_tx, Message::Gray(Box::new(left)), &mut stats);
                try_image(down_tx, Message::Gray(Box::new(right)), &mut stats);
                if let Some(depth) = pending_depth.take() {
                    try_image(down_tx, Message::Depth(Box::new(depth)), &mut stats);
                }
                last_image_stamp = left.timestamp;
                last_image = Instant::now();
            }
        }
        while let Ok(message) = up_rx.try_recv() {
            dispatch_uplink(&up, &mut stats, &mut session, message, last_imu_stamp)?;
            activity = true;
        }
        if last_heartbeat.elapsed() >= Duration::from_millis(500) {
            // 主机在线但暂无 IMU 时也要让板端心跳新鲜，避免看门狗误恢复 adb。
            send_blocking(
                down_tx,
                Message::Ping(now_seconds().to_bits().wrapping_add(stats.heartbeat)),
            )?;
            stats.heartbeat = stats.heartbeat.wrapping_add(1);
            last_heartbeat = Instant::now();
        }
        if last_timestamp_log.elapsed() >= STATS_PERIOD {
            log::debug!("hil 时间戳：imu={last_imu_stamp:.3}s image={last_image_stamp:.3}s");
            last_timestamp_log = Instant::now();
        }
        if last_stats.elapsed() >= STATS_PERIOD {
            let gap = stats.max_imu_gap;
            stats.max_imu_gap = 0.0;
            if gap > 0.05 {
                log::warn!("控制时间戳间隙 {gap:.3}s");
            }
            let age_mean_ms = if stats.control_age_count == 0 {
                0.0
            } else {
                stats.control_age_sum / stats.control_age_count as f64 * 1000.0
            };
            log::info!(
                "hil 计数：control={} odom={} telemetry={} 图像丢帧={} 校验ok={} 校验不一致={} 校验未匹配={} 控制老化 均值={age_mean_ms:.1}ms 最大={:.1}ms",
                stats.control,
                stats.odometry,
                stats.telemetry,
                stats.dropped_images,
                stats.image_checks_ok,
                stats.image_mismatch,
                stats.image_unmatched,
                stats.control_age_max * 1000.0
            );
            publish_stats(&up.scalar, &mut stats);
            last_stats = Instant::now();
        }
        if !activity {
            std::thread::sleep(POLL);
        }
    }
    Ok(())
}

fn send_blocking(tx: &SyncSender<Message>, message: Message) -> Result<(), String> {
    tx.send(message).map_err(|_| "USB 写线程已退出".to_owned())
}

fn record_image_checksum(stats: &mut StatsCounters, image: &GrayImageMessage) {
    let sum: u64 = image.data.iter().map(|&pixel| u64::from(pixel)).sum();
    if stats.sent_images.len() >= 64 {
        stats.sent_images.pop_front();
    }
    stats.sent_images.push_back((
        u8::from(image.sensor_id != 0),
        image.timestamp.to_bits(),
        sum,
    ));
}

/// 板端回传的像素和必须与主机下发的完全一致；不一致说明 USB 传输损坏。
fn check_image_checksum(stats: &mut StatsCounters, scalar: &ScalarMessage) {
    let len = (scalar.entity_len as usize).min(scalar.entity.len());
    let Ok(entity) = std::str::from_utf8(&scalar.entity[..len]) else {
        return;
    };
    let Some(side) = entity
        .strip_prefix("hil/camera/")
        .and_then(|side| match side {
            "left" => Some(0_u8),
            "right" => Some(1_u8),
            _ => None,
        })
    else {
        return;
    };
    if scalar.scalar_count < 4 {
        return;
    }
    let bits = scalar.scalars[0].to_bits();
    let sum = scalar.scalars[1] as u64;
    let Some(index) = stats
        .sent_images
        .iter()
        .position(|&(entry_side, entry_bits, _)| entry_side == side && entry_bits == bits)
    else {
        stats.image_unmatched += 1;
        return;
    };
    let (_, _, expected) = stats.sent_images.remove(index).expect("索引来自 iter");
    if expected == sum {
        stats.image_checks_ok += 1;
    } else {
        stats.image_mismatch += 1;
        log::warn!(
            "图像校验不一致 side={side} ts={:.3} 期望和={expected} 板端和={sum}",
            scalar.scalars[0]
        );
    }
}

fn try_image(tx: &SyncSender<Message>, message: Message, stats: &mut StatsCounters) {
    match tx.try_send(message) {
        Ok(()) | Err(TrySendError::Disconnected(_)) => {}
        Err(TrySendError::Full(_)) => stats.dropped_images += 1,
    }
}

#[derive(Default)]
struct StatsCounters {
    control: u64,
    odometry: u64,
    telemetry: u64,
    dropped_images: u64,
    board_dropped_images: u64,
    board_imu_gap_seconds: f64,
    max_imu_gap: f64,
    last_control_time: f64,
    /// 控制所用 IMU 时刻相对主机最新下行 IMU 的滞后（秒）。
    control_age_sum: f64,
    control_age_max: f64,
    control_age_count: u64,
    scalar_sequence: u64,
    heartbeat: u64,
    /// 已下发图像的 `(side, 时间戳位, 像素和)`，用于和板端回传校验和比对。
    sent_images: VecDeque<(u8, u64, u64)>,
    image_checks_ok: u64,
    image_mismatch: u64,
    image_unmatched: u64,
}

fn dispatch_uplink(
    ports: &Uplink,
    stats: &mut StatsCounters,
    session: &mut u64,
    message: Message,
    latest_imu_stamp: f64,
) -> Result<(), String> {
    if *session == 0 {
        *session = 1;
    }
    match message {
        Message::Control(control) => {
            if stats.last_control_time > 0.0 {
                stats.max_imu_gap = stats
                    .max_imu_gap
                    .max(control.state_time - stats.last_control_time);
            }
            // 控制基于 `state_time` 的 IMU；与主机刚下发的最新 IMU 比较，
            // 得到「传感器下行 + FC 处理 + 控制上行」的整圈滞后。
            if latest_imu_stamp.is_finite() {
                let age = latest_imu_stamp - control.state_time;
                if age.is_finite() && age > -0.005 {
                    stats.control_age_sum += age.max(0.0);
                    stats.control_age_max = stats.control_age_max.max(age);
                    stats.control_age_count += 1;
                }
            }
            stats.last_control_time = control.state_time;
            ports
                .control
                .publish(control)
                .map_err(|error| format!("发布 control: {error}"))?;
            stats.control += 1;
        }
        Message::Odometry(odometry) => {
            ports
                .odom
                .publish(odometry)
                .map_err(|error| format!("发布 odometry: {error}"))?;
            stats.odometry += 1;
        }
        Message::CorrectedOdometry(odometry) => {
            ports
                .corrected
                .publish(odometry)
                .map_err(|error| format!("发布 corrected odometry: {error}"))?;
            stats.odometry += 1;
        }
        Message::VioAttitude(attitude) => {
            ports
                .attitude
                .publish(attitude)
                .map_err(|error| format!("发布 vio attitude: {error}"))?;
            stats.telemetry += 1;
        }
        Message::Scalar(mut scalar) => {
            check_image_checksum(stats, &scalar);
            // 板端各进程的序号在主机侧无法区分发布者，桥统一重编号。
            scalar.sequence = stats.scalar_sequence;
            stats.scalar_sequence = stats.scalar_sequence.wrapping_add(1);
            ports
                .scalar
                .publish(scalar)
                .map_err(|error| format!("发布 scalar: {error}"))?;
            stats.telemetry += 1;
        }
        Message::Stats(board) => {
            stats.board_dropped_images = board.dropped_images;
            stats.board_imu_gap_seconds = board.imu_gap_seconds;
        }
        Message::HelloAck(ack) => {
            log::info!(
                "板端握手：version={} status={} board_session={:#x} uptime={:.1}s",
                ack.version,
                ack.status,
                ack.board_session,
                ack.uptime_s
            );
        }
        other => log::warn!("hil 主机忽略 {}", other.kind().name()),
    }
    Ok(())
}

fn publish_stats(ports: &Publisher<ScalarMessage>, stats: &mut StatsCounters) {
    let mut link = ScalarMessage::base(now_seconds(), "hil/link");
    link.sequence = stats.scalar_sequence;
    stats.scalar_sequence = stats.scalar_sequence.wrapping_add(1);
    link.scalars = [
        stats.control as f64,
        stats.odometry as f64,
        stats.telemetry as f64,
        stats.dropped_images as f64,
    ];
    link.scalar_count = 4;
    if let Err(error) = ports.publish(link) {
        log::warn!("发布 hil/link 失败：{error}");
    }
    let mut board = ScalarMessage::base(now_seconds(), "hil/board");
    board.sequence = stats.scalar_sequence;
    stats.scalar_sequence = stats.scalar_sequence.wrapping_add(1);
    board.scalars = [stats.board_imu_gap_seconds, 0.0, 0.0, 0.0];
    board.scalar_count = 1;
    if let Err(error) = ports.publish(board) {
        log::warn!("发布 hil/board 失败：{error}");
    }
    stats.control = 0;
    stats.odometry = 0;
    stats.telemetry = 0;
    stats.dropped_images = 0;
    stats.image_checks_ok = 0;
    stats.image_mismatch = 0;
    stats.image_unmatched = 0;
    stats.control_age_sum = 0.0;
    stats.control_age_max = 0.0;
    stats.control_age_count = 0;
}

fn session_id() -> u64 {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_nanos());
    (nanos as u64) ^ (u64::from(std::process::id()) << 32)
}

fn now_seconds() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |elapsed| elapsed.as_secs_f64())
}
