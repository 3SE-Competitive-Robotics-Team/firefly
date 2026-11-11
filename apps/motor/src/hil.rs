//! HIL 执行器回传：把本机控制与遥测写进 USB `FunctionFS` 端点。
//!
//! 时间戳与消息语义保持与真机相同；写端点阻塞即背压，不静默丢控制。
//! `Stats` 每秒回报一次收发计数，主机把它记进 RRD。

use std::collections::BTreeMap;
use std::fs::OpenOptions;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use firefly_hil::frame::FrameWriter;
use firefly_hil::gadget::endpoint_file_at;
use firefly_hil::protocol::{Message, Stats};
use firefly_pubsub::attitude::{ATTITUDE_TOPIC, AttitudeMessage};
use firefly_pubsub::control::{CONTROL_BUFFER_SIZE, CONTROL_TOPIC, ControlMessage};
use firefly_pubsub::log::{LOG_BUFFER_SIZE, LOG_MAX_PUBLISHERS, LOG_TOPIC, LogMessage};
use firefly_pubsub::node::create_node;
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
    let ports = Ports {
        control: Subscriber::with_topic_and_buffer(&node, CONTROL_TOPIC, CONTROL_BUFFER_SIZE)?,
        odom: Subscriber::with_topic(&node, ODOM_TOPIC)?,
        corrected: Subscriber::with_topic(&node, CORRECTED_ODOM_TOPIC)?,
        attitude: Subscriber::with_topic(&node, ATTITUDE_TOPIC)?,
        log: Subscriber::with_options(
            &node,
            LOG_TOPIC,
            SubscriberOptions {
                buffer_size: LOG_BUFFER_SIZE,
                service_max_buffer_size: Some(LOG_BUFFER_SIZE),
                max_publishers: Some(LOG_MAX_PUBLISHERS),
            },
        )?,
        // 使用 VizPublisher 的完整发布端和队列预算。
        scalar: Subscriber::with_options(
            &node,
            SCALAR_TOPIC,
            SubscriberOptions {
                buffer_size: SCALAR_BUFFER_SIZE,
                service_max_buffer_size: Some(SCALAR_BUFFER_SIZE),
                max_publishers: Some(VIZ_MAX_PUBLISHERS),
            },
        )?,
    };
    let path = endpoint_file_at(HIL_INSTANCE, UPLINK_ENDPOINT)?;
    let file = OpenOptions::new().write(true).open(&path)?;
    log::info!("HIL 执行器端点 {}", path.display());

    let mut writer = FrameWriter::new(file);
    let mut stats = Stats::default();
    let mut last_stats = Instant::now();
    let mut scalar_sequences = BTreeMap::new();
    let mut log_sequences = BTreeMap::new();
    while RUNNING.load(Ordering::Relaxed) {
        firefly_observability::pump_log_ipc(&log_ipc);
        let mut wrote = false;
        // 控制优先：上行链路被遥测堵住时不能拖慢控制回传。
        while let Ok(Some(sample)) = ports.control.receive() {
            writer.write_message(&Message::Control(*sample.payload()))?;
            stats.control_messages += 1;
            wrote = true;
        }
        while let Ok(Some(sample)) = ports.odom.receive() {
            if write_telemetry(
                &mut writer,
                &Message::Odometry(*sample.payload()),
                &mut stats,
            )? {
                wrote = true;
            }
        }
        while let Ok(Some(sample)) = ports.corrected.receive() {
            if write_telemetry(
                &mut writer,
                &Message::CorrectedOdometry(*sample.payload()),
                &mut stats,
            )? {
                wrote = true;
            }
        }
        while let Ok(Some(sample)) = ports.attitude.receive() {
            if write_telemetry(
                &mut writer,
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
            if write_telemetry(&mut writer, &Message::Scalar(message), &mut stats)? {
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
            writer.write_message(&Message::Log(*sample.payload()))?;
            wrote = true;
        }
        if last_stats.elapsed() >= STATS_PERIOD {
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

fn write_telemetry(
    writer: &mut FrameWriter<std::fs::File>,
    message: &Message,
    stats: &mut Stats,
) -> Result<bool, Box<dyn std::error::Error>> {
    writer.write_message(message)?;
    stats.telemetry_messages += 1;
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::observe_sequence;
    use std::collections::BTreeMap;

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
