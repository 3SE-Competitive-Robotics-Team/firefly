//! HIL 执行器回传：把本机控制与遥测写进 USB `FunctionFS` 端点。
//!
//! 时间戳与消息语义保持与真机相同；写端点阻塞即背压，不静默丢控制。
//! `Stats` 每秒回报一次收发计数，主机把它记进 RRD。

use std::collections::BTreeMap;
use std::fs::OpenOptions;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use firefly_hil::frame::FrameWriter;
use firefly_hil::gadget::endpoint_file_at;
use firefly_hil::protocol::{Message, Stats};
use firefly_pubsub::attitude::{ATTITUDE_TOPIC, AttitudeMessage};
use firefly_pubsub::control::{CONTROL_BUFFER_SIZE, CONTROL_TOPIC, ControlMessage};
use firefly_pubsub::log::{LOG_BUFFER_SIZE, LOG_MAX_PUBLISHERS, LOG_TOPIC, LogMessage};
use firefly_pubsub::node::{IpcNode, create_node};
use firefly_pubsub::odom::{CORRECTED_ODOM_TOPIC, ODOM_TOPIC, OdomMessage};
use firefly_pubsub::subscriber::{Subscriber, SubscriberOptions};
use firefly_pubsub::viz::{
    SCALAR_BUFFER_SIZE, SCALAR_TOPIC, SCALARS_MAX, ScalarMessage, VIZ_MAX_PUBLISHERS,
};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalGuard, SignalHandler};

/// HIL 功能实例名。
const HIL_INSTANCE: &str = "hil";
/// 上行端点在功能内的序号（第二个数据端点）。
const UPLINK_ENDPOINT: usize = 1;
/// 无数据轮询间隔。
const POLL: Duration = Duration::from_millis(1);
/// 统计回报周期。
const STATS_PERIOD: Duration = Duration::from_secs(1);

static RUNNING: AtomicBool = AtomicBool::new(true);

struct Ports {
    control: Subscriber<ControlMessage>,
    odom: Subscriber<OdomMessage>,
    corrected: Subscriber<OdomMessage>,
    attitude: Subscriber<AttitudeMessage>,
    scalar: Subscriber<ScalarMessage>,
    log: Subscriber<LogMessage>,
}

pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    let _guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_signal| RUNNING.store(false, Ordering::Relaxed),
    )?;
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "motor");
    let ports = open_ports(&node)?;
    forward(&ports, &log_ipc)
}

fn open_ports(node: &IpcNode) -> Result<Ports, Box<dyn std::error::Error>> {
    Ok(Ports {
        control: Subscriber::with_topic_and_buffer(node, CONTROL_TOPIC, CONTROL_BUFFER_SIZE)?,
        odom: Subscriber::with_topic(node, ODOM_TOPIC)?,
        corrected: Subscriber::with_topic(node, CORRECTED_ODOM_TOPIC)?,
        attitude: Subscriber::with_topic(node, ATTITUDE_TOPIC)?,
        log: Subscriber::with_options(
            node,
            LOG_TOPIC,
            SubscriberOptions {
                buffer_size: LOG_BUFFER_SIZE,
                service_max_buffer_size: Some(LOG_BUFFER_SIZE),
                max_publishers: Some(LOG_MAX_PUBLISHERS),
            },
        )?,
        // 使用 VizPublisher 的完整发布端和队列预算。
        scalar: Subscriber::with_options(
            node,
            SCALAR_TOPIC,
            SubscriberOptions {
                buffer_size: SCALAR_BUFFER_SIZE,
                service_max_buffer_size: Some(SCALAR_BUFFER_SIZE),
                max_publishers: Some(VIZ_MAX_PUBLISHERS),
            },
        )?,
    })
}

fn forward(
    ports: &Ports,
    log_ipc: &firefly_observability::LogIpc,
) -> Result<(), Box<dyn std::error::Error>> {
    let path = endpoint_file_at(HIL_INSTANCE, UPLINK_ENDPOINT)?;
    let file = OpenOptions::new().write(true).open(&path)?;
    log::info!("HIL 执行器端点 {}", path.display());

    let mut writer = FrameWriter::new(file);
    let mut stats = Stats::default();
    let mut last_stats = Instant::now();
    let mut scalar_sequences = BTreeMap::new();
    let mut log_sequences = BTreeMap::new();
    while RUNNING.load(Ordering::Relaxed) {
        firefly_observability::pump_log_ipc(log_ipc);
        let mut wrote = drain_control(&mut writer, &ports.control, &mut stats)?;
        while let Ok(Some(sample)) = ports.odom.receive() {
            if write_telemetry(
                &mut writer,
                &ports.control,
                &Message::Odometry(*sample.payload()),
                &mut stats,
            )? {
                wrote = true;
            }
        }
        while let Ok(Some(sample)) = ports.corrected.receive() {
            if write_telemetry(
                &mut writer,
                &ports.control,
                &Message::CorrectedOdometry(*sample.payload()),
                &mut stats,
            )? {
                wrote = true;
            }
        }
        while let Ok(Some(sample)) = ports.attitude.receive() {
            if write_telemetry(
                &mut writer,
                &ports.control,
                &Message::VioAttitude(*sample.payload()),
                &mut stats,
            )? {
                wrote = true;
            }
        }
        while let Ok(Some(sample)) = ports.scalar.receive() {
            observe_sequence(
                &mut scalar_sequences,
                sample.header().publisher_id().value(),
                sample.sequence,
                VIZ_MAX_PUBLISHERS,
            )?;
            let message = *sample.payload();
            if message.scalar_count as usize > SCALARS_MAX {
                continue;
            }
            if write_telemetry(
                &mut writer,
                &ports.control,
                &Message::Scalar(message),
                &mut stats,
            )? {
                wrote = true;
            }
        }
        while let Ok(Some(sample)) = ports.log.receive() {
            observe_sequence(
                &mut log_sequences,
                sample.header().publisher_id().value(),
                sample.sequence,
                LOG_MAX_PUBLISHERS,
            )?;
            write_with_control_priority(
                &mut writer,
                &ports.control,
                &Message::Log(*sample.payload()),
                &mut stats,
            )?;
            wrote = true;
        }
        if last_stats.elapsed() >= STATS_PERIOD {
            wrote |= drain_control(&mut writer, &ports.control, &mut stats)?;
            stats.uplink_backlog = 0;
            writer.write_message(&Message::Stats(stats))?;
            last_stats = Instant::now();
        }
        if !wrote {
            std::thread::sleep(POLL);
        }
    }
    log::info!("motor --hil 退出");
    Ok(())
}

fn observe_sequence(
    previous: &mut BTreeMap<u128, u64>,
    publisher: u128,
    sequence: u64,
    maximum: usize,
) -> std::io::Result<()> {
    if let Some(last) = previous.get(&publisher) {
        if sequence != last.wrapping_add(1) {
            return Err(std::io::Error::other(format!(
                "HIL diagnostic sequence gap: {last} -> {sequence}"
            )));
        }
    } else if previous.len() >= maximum {
        return Err(std::io::Error::other(
            "HIL diagnostic publisher budget exceeded",
        ));
    }
    previous.insert(publisher, sequence);
    Ok(())
}

fn write_telemetry<W: Write>(
    writer: &mut FrameWriter<W>,
    control: &Subscriber<ControlMessage>,
    message: &Message,
    stats: &mut Stats,
) -> Result<bool, Box<dyn std::error::Error>> {
    write_with_control_priority(writer, control, message, stats)?;
    stats.telemetry_messages += 1;
    Ok(true)
}

fn write_with_control_priority<W: Write>(
    writer: &mut FrameWriter<W>,
    control: &Subscriber<ControlMessage>,
    message: &Message,
    stats: &mut Stats,
) -> std::io::Result<()> {
    // Check between individual messages, including messages from the same queue.
    drain_control(writer, control, stats)?;
    writer.write_message(message)
}

fn drain_control<W: Write>(
    writer: &mut FrameWriter<W>,
    control: &Subscriber<ControlMessage>,
    stats: &mut Stats,
) -> std::io::Result<bool> {
    let mut wrote = false;
    while let Ok(Some(sample)) = control.receive() {
        writer.write_message(&Message::Control(*sample.payload()))?;
        stats.control_messages += 1;
        wrote = true;
    }
    Ok(wrote)
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_hil::frame::FrameReader;
    use firefly_hil::protocol::{HEADER_LEN, Kind, decode_header};
    use firefly_pubsub::publish::Publisher;
    use std::collections::BTreeMap;

    struct ControlDuringWrite {
        bytes: Vec<u8>,
        frame_start: usize,
        control: Publisher<ControlMessage>,
        tick: u64,
    }

    impl Write for ControlDuringWrite {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            self.bytes.extend_from_slice(bytes);
            Ok(bytes.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            let header = decode_header(
                self.bytes[self.frame_start..self.frame_start + HEADER_LEN]
                    .try_into()
                    .unwrap(),
            )?;
            self.frame_start = self.bytes.len();
            if header.kind != Kind::Control {
                self.tick += 1;
                assert_eq!(
                    self.control
                        .publish_counted(ControlMessage {
                            tick: self.tick,
                            ..Default::default()
                        })
                        .unwrap()
                        .0,
                    1
                );
            }
            Ok(())
        }
    }

    #[test]
    fn arriving_control_precedes_each_pending_scalar_and_log() {
        let node = create_node().unwrap();
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let topic = format!(
            "Firefly/Test/HilControlPriority/{}/{unique}",
            std::process::id()
        );
        let control =
            Subscriber::with_topic_and_buffer(&node, &topic, CONTROL_BUFFER_SIZE).unwrap();
        let publisher =
            Publisher::with_topic_and_buffer(&node, &topic, Some(CONTROL_BUFFER_SIZE)).unwrap();
        let mut writer = FrameWriter::new(ControlDuringWrite {
            bytes: Vec::new(),
            frame_start: 0,
            control: publisher,
            tick: 0,
        });
        let mut stats = Stats::default();
        let mut expected = Vec::new();
        for sequence in 0..2 {
            let mut scalar = ScalarMessage::base(1., "hil/board");
            scalar.sequence = sequence;
            let message = Message::Scalar(scalar);
            write_telemetry(&mut writer, &control, &message, &mut stats).unwrap();
            if sequence > 0 {
                expected.push(Message::Control(ControlMessage {
                    tick: sequence,
                    ..Default::default()
                }));
            }
            expected.push(message);
        }
        for sequence in 0..2 {
            let message = Message::Log(LogMessage {
                sequence,
                ..Default::default()
            });
            write_with_control_priority(&mut writer, &control, &message, &mut stats).unwrap();
            expected.push(Message::Control(ControlMessage {
                tick: sequence + 2,
                ..Default::default()
            }));
            expected.push(message);
        }
        let output = writer.into_inner();
        let mut reader = FrameReader::new(output.bytes.as_slice());
        let mut actual = Vec::new();
        while let Ok(frame) = reader.read_frame() {
            actual.push(frame.message);
        }
        assert_eq!(actual, expected);
        assert_eq!(stats.control_messages, 3);
        assert_eq!(stats.telemetry_messages, 2);
        // The control produced by the final write remains queued for the next turn.
        assert_eq!(control.receive().unwrap().unwrap().tick, 4);
    }

    #[test]
    fn sequences_are_checked_per_publisher_including_wraparound() {
        let mut previous = BTreeMap::new();
        observe_sequence(&mut previous, 1, u64::MAX, 2).unwrap();
        observe_sequence(&mut previous, 2, 10, 2).unwrap();
        observe_sequence(&mut previous, 1, 0, 2).unwrap();
        assert!(observe_sequence(&mut previous, 2, 12, 2).is_err());
        observe_sequence(&mut previous, 2, 11, 2).unwrap();
        assert!(observe_sequence(&mut previous, 2, 11, 2).is_err());
        assert!(observe_sequence(&mut previous, 3, 0, 2).is_err());
    }
}
