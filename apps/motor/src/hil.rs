//! HIL 执行器回传：把本机控制与遥测写进 USB `FunctionFS` 端点。
//!
//! 时间戳与消息语义保持与真机相同；写端点阻塞即背压，不静默丢控制。
//! `Stats` 每秒回报一次收发计数，主机把它记进 RRD。

use std::fs::OpenOptions;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use firefly_hil::frame::FrameWriter;
use firefly_hil::gadget::endpoint_file_at;
use firefly_hil::protocol::Kind;
use firefly_hil::protocol::{Message, Stats};
use firefly_hil::topics::TelemetrySubscribers as Ports;
use firefly_pubsub::node::create_node;
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

pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    let _guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_signal| RUNNING.store(false, Ordering::Relaxed),
    )?;
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "motor");
    let ports = Ports::open(&node, "Firefly")?;
    forward(&ports, &log_ipc)
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
    while RUNNING.load(Ordering::Relaxed) {
        firefly_observability::pump_log_ipc(log_ipc);
        let mut wrote = drain_control(&mut writer, ports, &mut stats)?;
        wrote |= ports.drain(None, |message| {
            if matches!(message, Message::Control(_)) {
                writer.write_message(&message)?;
                stats.control_messages += 1;
            } else if matches!(message, Message::Log(_)) {
                write_with_control_priority(&mut writer, ports, &message, &mut stats)?;
            } else {
                write_telemetry(&mut writer, ports, &message, &mut stats)?;
            }
            Ok(())
        })?;
        if last_stats.elapsed() >= STATS_PERIOD {
            wrote |= drain_control(&mut writer, ports, &mut stats)?;
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

fn write_telemetry<W: Write>(
    writer: &mut FrameWriter<W>,
    ports: &Ports,
    message: &Message,
    stats: &mut Stats,
) -> std::io::Result<()> {
    write_with_control_priority(writer, ports, message, stats)?;
    stats.telemetry_messages += 1;
    Ok(())
}

fn write_with_control_priority<W: Write>(
    writer: &mut FrameWriter<W>,
    ports: &Ports,
    message: &Message,
    stats: &mut Stats,
) -> std::io::Result<()> {
    // Check between individual messages, including messages from the same queue.
    drain_control(writer, ports, stats)?;
    writer.write_message(message)
}

fn drain_control<W: Write>(
    writer: &mut FrameWriter<W>,
    ports: &Ports,
    stats: &mut Stats,
) -> std::io::Result<bool> {
    ports.drain(Some(Kind::Control), |message| {
        writer.write_message(&message)?;
        stats.control_messages += 1;
        Ok(())
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_hil::frame::FrameReader;
    use firefly_hil::protocol::{HEADER_LEN, Kind, decode_header};
    use firefly_pubsub::{control::CONTROL_BUFFER_SIZE, log::LogMessage, viz::ScalarMessage};
    use firefly_pubsub::{control::ControlMessage, publish::Publisher};

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
        let prefix = format!("Test/HilControlPriority/{}/{unique}", std::process::id());
        let topic = format!("{prefix}/Control");
        let control = Ports::open(&node, &prefix).unwrap();
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
        let mut pending = Vec::new();
        control
            .drain(Some(Kind::Control), |message| {
                pending.push(message);
                Ok(())
            })
            .unwrap();
        assert_eq!(
            pending,
            [Message::Control(ControlMessage {
                tick: 4,
                ..Default::default()
            })]
        );
    }
}
