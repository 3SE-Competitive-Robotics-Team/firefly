pub mod rate;

use std::cell::Cell;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};

use fastrace::prelude::*;
use firefly_pubsub::log::{LOG_BUFFER_SIZE, LOG_MAX_PUBLISHERS, LOG_TOPIC, LogMessage, level};
use firefly_pubsub::node::IpcNode;
use firefly_pubsub::publish::Publisher;
use logforth::filter::rustlog::RustLogFilterBuilder;

pub fn init() {
    logforth::starter_log::builder()
        .dispatch(|d| {
            d.filter(RustLogFilterBuilder::from_default_env().build())
                .diagnostic(logforth::diagnostic::FastraceDiagnostic::default())
                .append(logforth::append::Stderr::default())
        })
        .dispatch(|d| {
            d.filter(RustLogFilterBuilder::from_default_env().build())
                .append(logforth::append::FastraceEvent::default())
        })
        .dispatch(|d| {
            d.filter(RustLogFilterBuilder::from_default_env().build())
                .append(IpcAppend)
        })
        .apply();

    fastrace::set_reporter(CompactReporter, fastrace::collector::Config::default());
}

struct CompactReporter;

impl fastrace::collector::Reporter for CompactReporter {
    fn report(&mut self, spans: Vec<fastrace::collector::SpanRecord>) {
        for span in spans {
            eprintln!(
                "{:>9.3}ms  {:<44} trace={:032x}",
                span.duration_ns as f64 / 1e6,
                span.name,
                span.trace_id.0
            );
        }
    }
}

/// 当前 sim 时钟（秒）的位表示；负值表示尚无 sim 时钟（聚合端回落墙钟轴）。
static SIM_TIME_BITS: AtomicU64 = AtomicU64::new((-1.0_f64).to_bits());

/// 更新本进程的 sim 时钟（主循环每 tick 调用一次；`mimir` 聚合端除外）。
pub fn set_sim_time(t: f64) {
    SIM_TIME_BITS.store(t.to_bits(), Ordering::Relaxed);
}

fn sim_time_bits() -> u64 {
    SIM_TIME_BITS.load(Ordering::Relaxed)
}

static TAG: Mutex<String> = Mutex::new(String::new());

/// 日志聚合句柄（调用方持有：主循环作用域，与 `node` 同生命周期）。
/// 发布端只活在创建线程（主循环线程）——调用方每 tick 把句柄传给
/// [`pump_log_ipc`]，从构造上不可能跨线程触碰，无 `unsafe`。
/// `None` = 纯 stderr 模式（`init_ipc` 建端失败时）。
pub struct LogIpc {
    publisher: Option<Publisher<LogMessage>>,
    sequence: Cell<u64>,
    reported_loss: Cell<u64>,
}

static IPC_ARMED: Mutex<bool> = Mutex::new(false);

struct QueuedLog {
    log_level: u8,
    tag: String,
    text: String,
    sim_time_bits: u64,
    wall_secs: i64,
    wall_nanos: u32,
    trace_id_hi: u64,
    trace_id_lo: u64,
}

/// 有界 channel 容量；超载必须成为录制完整性故障。
const QUEUE_CAP: usize = 128;
static DROPPED_RECORDS: AtomicU64 = AtomicU64::new(0);

static LOG_QUEUE: OnceLock<Queue> = OnceLock::new();

struct Queue {
    tx: Mutex<std::sync::mpsc::SyncSender<QueuedLog>>,
    rx: Mutex<std::sync::mpsc::Receiver<QueuedLog>>,
}

fn log_queue() -> &'static Queue {
    LOG_QUEUE.get_or_init(|| {
        let (tx, rx) = std::sync::mpsc::sync_channel(QUEUE_CAP);
        Queue {
            tx: Mutex::new(tx),
            rx: Mutex::new(rx),
        }
    })
}

/// 挂载 IPC 聚合链路：记下进程标签，同线程建端并返回 [`LogIpc`] 句柄
/// （调用方在主循环作用域持有，每 tick 传给 [`pump_log_ipc`]）。
/// 建端失败时返回 `LogIpc { publisher: None }` = 纯 stderr 模式（记一条 warn）。
///
/// # Panics
/// 内部标签锁或武装锁中毒时。
#[must_use]
pub fn init_ipc(node: &IpcNode, tag: &str) -> LogIpc {
    tag.clone_into(&mut TAG.lock().expect("log tag mutex"));
    let _ = log_queue();
    match Publisher::with_topic_buffer_publishers(
        node,
        LOG_TOPIC,
        Some(LOG_BUFFER_SIZE),
        Some(LOG_MAX_PUBLISHERS),
    ) {
        Ok(publisher) => {
            *IPC_ARMED.lock().expect("ipc armed mutex") = true;
            log::info!("日志聚合已挂载（tag={tag}，Firefly/Log → mimir）");
            LogIpc {
                publisher: Some(publisher),
                sequence: Cell::new(0),
                reported_loss: Cell::new(0),
            }
        }
        Err(e) => {
            log::warn!("日志聚合挂载失败（降级纯 stderr）：{e}");
            LogIpc {
                publisher: None,
                sequence: Cell::new(0),
                reported_loss: Cell::new(0),
            }
        }
    }
}

/// 主循环驱动：排空日志队列并零拷贝发布（每 tick 调用一次，见模块文档）。
///
/// # Panics
/// 内部队列锁中毒时。
pub fn pump_log_ipc(log_ipc: &LogIpc) {
    if log_ipc.publisher.is_none() {
        return;
    }
    let queue = log_queue();
    let rx = queue.rx.lock().expect("log queue rx mutex");
    while let Ok(queued) = rx.try_recv() {
        let mut msg = LogMessage {
            log_level: queued.log_level,
            sim_time: f64::from_bits(queued.sim_time_bits),
            wall_secs: queued.wall_secs,
            wall_nanos: queued.wall_nanos,
            trace_id_hi: queued.trace_id_hi,
            trace_id_lo: queued.trace_id_lo,
            ..LogMessage::default()
        };
        msg.set_tag(&queued.tag);
        msg.set_text(&queued.text);
        log_ipc.publish(msg);
    }
    // Report even a lost final record; do not depend on a later application log.
    if DROPPED_RECORDS.load(Ordering::Relaxed) > log_ipc.reported_loss.get() {
        let now = jiff::Timestamp::now();
        let mut msg = LogMessage {
            log_level: level::ERROR,
            sim_time: f64::from_bits(sim_time_bits()),
            wall_secs: now.as_second(),
            wall_nanos: now.subsec_nanosecond() as u32,
            ..LogMessage::default()
        };
        msg.set_tag(&TAG.lock().expect("log tag mutex"));
        msg.set_text("log delivery failed; recording is incomplete");
        log_ipc.publish(msg);
    }
}

impl LogIpc {
    /// Record an application lifecycle event on its measurement clock.
    pub fn event(&self, tag: &str, stamp: f64, text: &str) {
        let wall = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default();
        let mut msg = LogMessage {
            sim_time: stamp,
            wall_secs: wall.as_secs() as i64,
            wall_nanos: wall.subsec_nanos(),
            ..Default::default()
        };
        msg.set_tag(tag);
        msg.set_text(text);
        self.publish(msg);
    }

    fn publish(&self, mut msg: LogMessage) {
        let Some(publisher) = &self.publisher else {
            return;
        };
        msg.sequence = self.sequence.get();
        self.sequence.set(msg.sequence.wrapping_add(1));
        msg.dropped_records = DROPPED_RECORDS.load(Ordering::Relaxed);
        if publisher.publish(msg).is_ok() {
            self.reported_loss.set(msg.dropped_records);
        } else {
            DROPPED_RECORDS.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// logforth → `Firefly/Log` 的聚合 appender（双 sink 中的 IPC 半边）。
///
/// 每条 `Record` 先在调用线程内格式化为 [`QueuedLog`]（含 trace 链与
/// sim 时钟快照——调用线程的 fastrace 上下文，事后不可追补），再
/// `try_send` 进有界队列；队列满时累计完整性故障，永不阻塞计算线程。
/// `target` 丢弃：实体检索走进程 `tag`，不走模块 `target`。
#[derive(Debug, Default)]
pub struct IpcAppend;

impl logforth::append::Append for IpcAppend {
    fn append(
        &self,
        record: &logforth::record::Record,
        _diags: &[Box<dyn logforth::Diagnostic>],
    ) -> Result<(), logforth::Error> {
        if !*IPC_ARMED.lock().expect("ipc armed mutex") {
            return Ok(());
        }
        let (trace_id_hi, trace_id_lo) = match SpanContext::current_local_parent() {
            Some(sc) => ((sc.trace_id.0 >> 64) as u64, sc.trace_id.0 as u64),
            None => (0, 0),
        };
        let now = jiff::Timestamp::now();
        let queued = QueuedLog {
            // logforth 16 级 → 5 级（2/3/4 后缀折叠到基线；对照
            // `RustLogFilterBuilder` 的 RUST_LOG 映射方向）
            log_level: match record.level() {
                logforth::record::Level::Error | logforth::record::Level::Error2 => level::ERROR,
                logforth::record::Level::Warn | logforth::record::Level::Warn2 => level::WARN,
                logforth::record::Level::Info | logforth::record::Level::Info2 => level::INFO,
                logforth::record::Level::Debug | logforth::record::Level::Debug2 => level::DEBUG,
                _ => level::TRACE,
            },
            tag: TAG.lock().expect("log tag mutex").clone(),
            text: format!("{}", record.payload()),
            sim_time_bits: sim_time_bits(),
            wall_secs: now.as_second(),
            wall_nanos: now.subsec_nanosecond() as u32,
            trace_id_hi,
            trace_id_lo,
        };
        if log_queue()
            .tx
            .lock()
            .expect("log queue tx mutex")
            .try_send(queued)
            .is_err()
        {
            DROPPED_RECORDS.fetch_add(1, Ordering::Relaxed);
        }
        Ok(())
    }

    fn flush(&self) -> Result<(), logforth::Error> {
        Ok(())
    }
}

pub fn flush() {
    // 日志队列由主循环排空；此处刷新 trace。stderr sink 无缓冲。
    fastrace::flush();
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_pubsub::subscriber::{Subscriber, SubscriberOptions};
    use logforth::append::Append;

    #[test]
    fn queue_overload_and_lost_tail_are_visible_over_ipc() {
        let node = firefly_pubsub::node::create_node().unwrap();
        let topic = format!(
            "Firefly/Test/Log/{}",
            jiff::Timestamp::now().as_nanosecond()
        );
        let subscriber = Subscriber::<LogMessage>::with_options(
            &node,
            &topic,
            SubscriberOptions {
                buffer_size: LOG_BUFFER_SIZE,
                service_max_buffer_size: Some(LOG_BUFFER_SIZE),
                max_publishers: Some(LOG_MAX_PUBLISHERS),
            },
        )
        .unwrap();
        let log_ipc = LogIpc {
            publisher: Some(
                Publisher::with_topic_buffer_publishers(
                    &node,
                    &topic,
                    Some(LOG_BUFFER_SIZE),
                    Some(LOG_MAX_PUBLISHERS),
                )
                .unwrap(),
            ),
            sequence: Cell::new(0),
            reported_loss: Cell::new(0),
        };
        *IPC_ARMED.lock().unwrap() = true;
        let record = logforth::record::RecordBuilder::default()
            .payload(format_args!("bounded queue probe"))
            .build();
        for _ in 0..=QUEUE_CAP {
            IpcAppend.append(&record, &[]).unwrap();
        }
        assert_eq!(DROPPED_RECORDS.load(Ordering::Relaxed), 1);
        pump_log_ipc(&log_ipc);
        for sequence in 0..QUEUE_CAP as u64 {
            let msg = subscriber.receive().unwrap().expect("queued log");
            assert_eq!(msg.sequence, sequence);
            assert_eq!(msg.dropped_records, 1);
        }
        assert!(subscriber.receive().unwrap().is_none());
        DROPPED_RECORDS.fetch_add(1, Ordering::Relaxed);
        pump_log_ipc(&log_ipc);
        let tail = subscriber
            .receive()
            .unwrap()
            .expect("loss without a later application log");
        assert_eq!(tail.sequence, QUEUE_CAP as u64);
        assert_eq!(tail.dropped_records, 2);
        assert_eq!(tail.log_level, level::ERROR);
    }

    #[test]
    fn startup_log_clock_satisfies_wire_timestamp_contract() {
        let time = f64::from_bits(super::sim_time_bits());
        assert!(time.is_finite() && time < 0.);
        super::set_sim_time(0.001);
        assert_eq!(super::sim_time_bits(), 0.001_f64.to_bits());
    }
}
