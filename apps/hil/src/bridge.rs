use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, SyncSender, TrySendError, sync_channel};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use firefly_hil::frame::{FrameReader, FrameWriter};
use firefly_hil::protocol::{
    Hello, IMAGE_CHUNK_DATA_MAX, ImageChunkHeader, Kind, Message, ROLE_HOST, VERSION,
};
use firefly_hil::topics::{SensorSubscribers as Downlink, TelemetryPublishers as Uplink};
use firefly_pubsub::camera::{DepthImageMessage, DownImageMessage, GrayImageMessage};
use firefly_pubsub::imu::ImuMessage;
use firefly_pubsub::node::{IpcNode, create_node};
use firefly_pubsub::viz::ScalarMessage;

use crate::config::Config;
use crate::host::HostLink;

/// 下行发送队列深度；满时图像丢帧、IMU/指令阻塞。
const DOWNLINK_QUEUE: usize = 64;
const UPLINK_QUEUE: usize = 1024;
/// IPC 轮询间隔。
const POLL: Duration = Duration::from_millis(1);
const STATS_PERIOD: Duration = Duration::from_secs(1);
/// 双目配对的采集时间容差（秒）。
const STEREO_TOLERANCE_S: f64 = 0.001;

/// A queue entry admits both stereo views together; depth is independently optional.
enum ImageBatch {
    Stereo(Box<GrayImageMessage>, Box<GrayImageMessage>),
    Depth(Box<DepthImageMessage>),
    Down(Box<DownImageMessage>),
}

/// A worker finishing or unwinding ends the shared bridge session.
struct StopOnDrop<'a>(&'a AtomicBool);

impl Drop for StopOnDrop<'_> {
    fn drop(&mut self) {
        self.0.store(true, Ordering::Relaxed);
    }
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
    let (priority_tx, priority_rx) = sync_channel(DOWNLINK_QUEUE);
    // Each entry owns at most two images, preserving the configured image budget.
    let (image_tx, image_rx) = sync_channel(config.link.image_queue / 2);
    let (up_tx, up_rx) = sync_channel(UPLINK_QUEUE);
    let writer_stop = Arc::clone(&stop);
    let writer =
        std::thread::spawn(move || usb_writer(writer, priority_rx, image_rx, session, writer_stop));
    let reader_stop = Arc::clone(&stop);
    let reader = std::thread::spawn(move || usb_reader(reader, up_tx, reader_stop));
    log::info!("HIL 链路已打开，session={session:#x}");
    let result = ipc_loop(
        down,
        up,
        &priority_tx,
        &image_tx,
        &up_rx,
        &stop,
        &config.link,
    );
    stop.store(true, Ordering::Relaxed);
    drop(priority_tx);
    drop(image_tx);
    drop(up_rx);
    let writer = writer.join().map_err(|_| "USB 写线程 panic".to_owned());
    let reader = reader.join().map_err(|_| "USB 读线程 panic".to_owned());
    writer??;
    reader??;
    result?;
    log::info!("HIL 链路已退出");
    Ok(())
}

fn open_ports(node: &IpcNode) -> Result<(Downlink, Uplink), String> {
    Ok((
        Downlink::open(node, "Firefly").map_err(|e| e.to_string())?,
        Uplink::open(node, "Firefly").map_err(|e| e.to_string())?,
    ))
}

#[allow(clippy::needless_pass_by_value)]
fn usb_writer(
    writer: impl Write,
    priority: Receiver<Message>,
    images: Receiver<ImageBatch>,
    session: u64,
    stop: Arc<AtomicBool>,
) -> Result<(), String> {
    let _stop_on_exit = StopOnDrop(&stop);
    let mut writer = FrameWriter::new(writer);
    writer.set_session(session);
    writer
        .write_message(&Message::Hello(Hello {
            version: VERSION,
            role: ROLE_HOST,
            session,
        }))
        .map_err(|error| format!("写 hello: {error}"))?;
    let mut next_image_id = 0_u32;
    while !stop.load(Ordering::Relaxed) {
        let mut wrote = false;
        // 高优先队列优先：IMU/指令最多等一个图像块的传输时间。
        while !stop.load(Ordering::Relaxed)
            && let Ok(message) = priority.try_recv()
        {
            writer
                .write_message(&message)
                .map_err(|error| format!("写 {}: {error}", message.kind().name()))?;
            wrote = true;
        }
        if let Ok(message) = images.try_recv() {
            write_image(&mut writer, &message, &priority, &mut next_image_id, &stop)?;
            continue;
        }
        if wrote {
            continue;
        }
        match priority.recv_timeout(Duration::from_millis(1)) {
            Ok(message) => writer
                .write_message(&message)
                .map_err(|error| format!("写 {}: {error}", message.kind().name()))?,
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }
    }
    Ok(())
}

/// 把一张完整图像按块写出；每块之间排空高优先队列。
fn write_image(
    writer: &mut FrameWriter<impl Write>,
    message: &ImageBatch,
    priority: &Receiver<Message>,
    next_image_id: &mut u32,
    stop: &AtomicBool,
) -> Result<(), String> {
    match message {
        ImageBatch::Stereo(left, right) => {
            for (sensor, image) in [(0, left), (1, right)] {
                write_chunks(
                    writer,
                    priority,
                    image.timestamp,
                    sensor,
                    &image.data,
                    next_image_id,
                    stop,
                )?;
            }
            Ok(())
        }
        ImageBatch::Down(image) => write_chunks(
            writer,
            priority,
            image.timestamp,
            3,
            &image.data,
            next_image_id,
            stop,
        ),
        ImageBatch::Depth(image) => {
            let mut raw = Vec::with_capacity(image.data.len() * 4);
            for value in image.data {
                raw.extend_from_slice(&value.to_le_bytes());
            }
            write_chunks(
                writer,
                priority,
                image.timestamp,
                2,
                &raw,
                next_image_id,
                stop,
            )
        }
    }
}

fn write_chunks(
    writer: &mut FrameWriter<impl Write>,
    priority: &Receiver<Message>,
    timestamp: f64,
    sensor_id: u8,
    data: &[u8],
    next_image_id: &mut u32,
    stop: &AtomicBool,
) -> Result<(), String> {
    let count = u16::try_from(data.len().div_ceil(IMAGE_CHUNK_DATA_MAX).max(1))
        .map_err(|_| "图像块数过多".to_owned())?;
    let image_id = *next_image_id;
    *next_image_id = next_image_id.wrapping_add(1);
    for (index, chunk) in data.chunks(IMAGE_CHUNK_DATA_MAX).enumerate() {
        if stop.load(Ordering::Relaxed) {
            break;
        }
        let header = ImageChunkHeader {
            timestamp,
            sensor_id,
            image_id,
            chunk_index: u16::try_from(index).unwrap_or(0),
            chunk_count: count,
        };
        let encoded = header.encode();
        writer
            .write_parts(Kind::ImageChunk, timestamp, &[&encoded, chunk])
            .map_err(|error| format!("写图像块: {error}"))?;
        // 一块最多挡住 IMU 约 0.4ms；这里把积压的 IMU/指令立刻插出去。
        while !stop.load(Ordering::Relaxed)
            && let Ok(message) = priority.try_recv()
        {
            writer
                .write_message(&message)
                .map_err(|error| format!("写 {}: {error}", message.kind().name()))?;
        }
    }
    Ok(())
}

#[allow(clippy::needless_pass_by_value)]
fn usb_reader(
    reader: impl Read,
    queue: SyncSender<Message>,
    stop: Arc<AtomicBool>,
) -> Result<(), String> {
    let _stop_on_exit = StopOnDrop(&stop);
    let mut reader = FrameReader::new(reader);
    while !stop.load(Ordering::Relaxed) {
        match reader.read_frame() {
            Ok(frame) => {
                if queue.send(frame.message).is_err() {
                    break;
                }
            }
            Err(error) if firefly_hil::frame::is_retryable(&error) => {}
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
    priority_tx: &SyncSender<Message>,
    image_tx: &SyncSender<ImageBatch>,
    up_rx: &Receiver<Message>,
    stop: &AtomicBool,
    options: &crate::config::Link,
) -> Result<(), String> {
    let image_period = Duration::from_secs_f64(options.image_period_s);
    let mut image_deadline = Instant::now() + image_period;
    let mut pending_left: Option<GrayImageMessage> = None;
    let mut pending_right: Option<GrayImageMessage> = None;
    let mut pending_depth: Option<DepthImageMessage> = None;
    let mut pending_down = None;
    let mut down_deadline = Instant::now();
    let mut last_imu_stamp = f64::NAN;
    let mut last_image_stamp = f64::NAN;
    let mut last_timestamp_log = Instant::now();
    let mut stats = StatsCounters::default();
    let mut last_stats = Instant::now();
    let mut last_heartbeat = Instant::now();
    let mut session = 0_u64;
    let mut imu_batch: Vec<ImuMessage> = Vec::with_capacity(options.imu_batch);
    let (responses, rx) = sync_channel(4);
    crate::mission::serve(priority_tx.clone(), rx).map_err(|e| e.to_string())?;
    while !stop.load(Ordering::Relaxed) {
        let mut activity = false;
        // 先排空上行控制：图像/IMU 下行不能把 1 kHz 控制的回传压在后面。
        while let Ok(message) = up_rx.try_recv() {
            if let Message::MissionReply(id, body) = message {
                let _ = responses.try_send((id, body));
            } else {
                dispatch_uplink(&up, &mut stats, &mut session, message, last_imu_stamp)?;
            }
            activity = true;
        }
        activity |= down
            .requests
            .drain(None, |message| {
                send_blocking(priority_tx, message).map_err(std::io::Error::other)
            })
            .map_err(|e| e.to_string())?;
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
            send_blocking(priority_tx, Message::Imu(std::mem::take(&mut imu_batch)))?;
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
        while let Ok(Some(sample)) = down.down.receive() {
            pending_down = Some(Box::new(*sample));
            activity = true;
        }
        if Instant::now() >= down_deadline {
            if let Some(image) = pending_down.take() {
                let stamp = image.timestamp;
                let sum = image.data.iter().map(|&pixel| u64::from(pixel)).sum();
                if try_image(image_tx, ImageBatch::Down(image), &mut stats)? {
                    record_checksum(&mut stats, 3, stamp, sum);
                }
            }
            down_deadline = next_image_deadline(
                down_deadline,
                Instant::now(),
                Duration::from_secs_f64(1. / f64::from(options.down_rate_hz)),
            );
        }
        while let Ok(Some(sample)) = down.depth.receive() {
            pending_depth = Some(*sample.payload());
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
            } else if Instant::now() >= image_deadline {
                let left = pending_left.take().expect("已检查非空");
                let right = pending_right.take().expect("已检查非空");
                enqueue_stereo(image_tx, &left, &right, &mut stats)?;
                if let Some(depth) = pending_depth.take() {
                    try_image(image_tx, ImageBatch::Depth(Box::new(depth)), &mut stats)?;
                }
                last_image_stamp = left.timestamp;
                image_deadline = next_image_deadline(image_deadline, Instant::now(), image_period);
            }
        }
        if last_heartbeat.elapsed() >= Duration::from_millis(500) {
            // 主机在线但暂无 IMU 时也要让板端心跳新鲜，避免看门狗误恢复 adb。
            send_blocking(priority_tx, Message::Ping(stats.heartbeat))?;
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
            publish_stats(&up, &mut stats, last_imu_stamp.max(0.0));
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
    record_checksum(stats, u8::from(image.sensor_id != 0), image.timestamp, sum);
}

fn record_checksum(stats: &mut StatsCounters, side: u8, stamp: f64, sum: u64) {
    if stats.sent_images.len() >= 64 {
        stats.sent_images.pop_front();
    }
    stats.sent_images.push_back((side, stamp.to_bits(), sum));
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
            "down" => Some(3_u8),
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

fn enqueue_stereo(
    tx: &SyncSender<ImageBatch>,
    left: &GrayImageMessage,
    right: &GrayImageMessage,
    stats: &mut StatsCounters,
) -> Result<(), String> {
    if try_image(
        tx,
        ImageBatch::Stereo(Box::new(*left), Box::new(*right)),
        stats,
    )? {
        record_image_checksum(stats, left);
        record_image_checksum(stats, right);
    }
    Ok(())
}

fn try_image(
    tx: &SyncSender<ImageBatch>,
    message: ImageBatch,
    stats: &mut StatsCounters,
) -> Result<bool, String> {
    let image_count = match message {
        ImageBatch::Stereo(..) => 2,
        ImageBatch::Depth(_) | ImageBatch::Down(_) => 1,
    };
    match tx.try_send(message) {
        Ok(()) => Ok(true),
        Err(TrySendError::Disconnected(_)) => Err("USB 写线程已退出".to_owned()),
        Err(TrySendError::Full(_)) => {
            stats.dropped_images += image_count;
            Ok(false)
        }
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
    match &message {
        Message::Control(control) => {
            if stats.last_control_time > 0. {
                stats.max_imu_gap = stats
                    .max_imu_gap
                    .max(control.state_time - stats.last_control_time);
            }
            if latest_imu_stamp.is_finite() {
                let age = latest_imu_stamp - control.state_time;
                if age.is_finite() && age > -0.005 {
                    stats.control_age_sum += age.max(0.);
                    stats.control_age_max = stats.control_age_max.max(age);
                    stats.control_age_count += 1;
                }
            }
            stats.last_control_time = control.state_time;
            stats.control += 1;
        }
        Message::Odometry(_) | Message::CorrectedOdometry(_) => stats.odometry += 1,
        Message::Scalar(scalar) => {
            check_image_checksum(stats, scalar);
            stats.telemetry += 1;
        }
        Message::Log(_) => {}
        Message::Stats(board) => {
            stats.board_dropped_images = board.dropped_images;
            stats.board_imu_gap_seconds = board.imu_gap_seconds;
            return Ok(());
        }
        Message::HelloAck(ack) => {
            log::info!("board handshake: {ack:?}");
            return Ok(());
        }
        Message::Pong(_) => return Ok(()),
        _ => stats.telemetry += 1,
    }
    ports.publish(message).map_err(|error| error.to_string())
}

fn publish_stats(ports: &Uplink, stats: &mut StatsCounters, timestamp: f64) {
    let mut link = ScalarMessage::base(timestamp, "hil/link");
    link.scalars = [
        stats.control as f64,
        stats.odometry as f64,
        stats.telemetry as f64,
        stats.dropped_images as f64,
    ];
    link.scalar_count = 4;
    if let Err(error) = ports.publish(Message::Scalar(link)) {
        log::warn!("发布 hil/link 失败：{error}");
    }
    let mut board = ScalarMessage::base(timestamp, "hil/board");
    board.scalars = [stats.board_imu_gap_seconds, 0.0, 0.0, 0.0];
    board.scalar_count = 1;
    if let Err(error) = ports.publish(Message::Scalar(board)) {
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

/// Preserve the periodic schedule through ordinary polling jitter. After a full
/// missed period, resume from now so a stalled link never emits catch-up bursts.
fn next_image_deadline(deadline: Instant, now: Instant, period: Duration) -> Instant {
    let next = deadline + period;
    if next > now { next } else { now + period }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::ErrorKind;

    #[test]
    fn uplink_read_failure_stops_the_other_workers() {
        let (tx, rx) = sync_channel(1);
        let stop = Arc::new(AtomicBool::new(false));
        let result = usb_reader(
            std::io::Cursor::new(vec![0; firefly_hil::protocol::HEADER_LEN]),
            tx,
            Arc::clone(&stop),
        );
        assert!(result.is_err());
        assert!(stop.load(Ordering::Relaxed));
        assert!(rx.try_recv().is_err());
    }

    #[test]
    fn writer_failure_stops_the_other_workers() {
        struct FailingWriter;
        impl Write for FailingWriter {
            fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
                Err(std::io::Error::new(
                    ErrorKind::BrokenPipe,
                    "injected endpoint failure",
                ))
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        let (_priority, priority_rx) = sync_channel(1);
        let (_images, image_rx) = sync_channel(1);
        let stop = Arc::new(AtomicBool::new(false));
        assert!(usb_writer(FailingWriter, priority_rx, image_rx, 1, Arc::clone(&stop)).is_err());
        assert!(stop.load(Ordering::Relaxed));
    }

    #[test]
    fn worker_panic_stops_the_other_workers() {
        struct PanickingReader;
        impl Read for PanickingReader {
            fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
                panic!("injected endpoint panic")
            }
        }
        let (tx, _rx) = sync_channel(1);
        let stop = Arc::new(AtomicBool::new(false));
        let task_stop = Arc::clone(&stop);
        let worker = std::thread::spawn(move || usb_reader(PanickingReader, tx, task_stop));
        assert!(worker.join().is_err());
        assert!(stop.load(Ordering::Relaxed));
    }

    #[test]
    fn stereo_admission_never_leaves_half_a_pair() {
        let (tx, rx) = sync_channel(1);
        let mut stats = StatsCounters::default();
        let left = GrayImageMessage {
            timestamp: 1.,
            sensor_id: 0,
            ..Default::default()
        };
        let right = GrayImageMessage {
            timestamp: 1.,
            sensor_id: 1,
            ..Default::default()
        };
        enqueue_stereo(&tx, &left, &right, &mut stats).unwrap();
        enqueue_stereo(&tx, &left, &right, &mut stats).unwrap();
        assert_eq!(stats.dropped_images, 2);
        assert_eq!(
            stats.sent_images.len(),
            2,
            "only admitted images await checksums"
        );
        let ImageBatch::Stereo(left, right) = rx.try_recv().unwrap() else {
            panic!("expected stereo")
        };
        assert_eq!((left.sensor_id, right.sensor_id), (0, 1));
        assert_eq!(left.timestamp, right.timestamp);
        assert!(rx.try_recv().is_err());
        enqueue_stereo(&tx, &left, &right, &mut stats).unwrap();
        assert!(matches!(rx.try_recv(), Ok(ImageBatch::Stereo(..))));
        drop(rx);
        assert!(enqueue_stereo(&tx, &left, &right, &mut stats).is_err());
    }

    #[test]
    fn image_cadence_preserves_phase_and_skips_stalled_periods() {
        let start = Instant::now();
        let period = Duration::from_secs_f64(1. / 30.);
        let mut deadline = start + period;
        for frame in 1..=600 {
            let now = start + period * frame + Duration::from_millis(1);
            deadline = next_image_deadline(deadline, now, period);
            assert_eq!(deadline, start + period * (frame + 1));
        }
        let resumed = deadline + period * 10;
        assert_eq!(
            next_image_deadline(deadline, resumed, period),
            resumed + period
        );
        assert_eq!(
            next_image_deadline(deadline, deadline + period, period),
            deadline + period * 2
        );
    }
}
