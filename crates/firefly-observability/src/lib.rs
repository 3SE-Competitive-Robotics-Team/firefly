//! 日志与追踪基础设施。
//!
//! 基于 logforth + fastrace 的最佳实践（fastrace/examples/log-integration）：
//! - 日志记录携带当前 span 信息（`FastraceDiagnostic`）
//! - 日志同时作为 span 事件（`FastraceEvent`），在 trace 中可见
//! - 日志级别由 `RUST_LOG` 控制，未设置时仅输出 error
//!
//! 聚合链路（[`IpcAppend`]）：各进程经 `log!` 宏的调用点**零改动**——
//! `init()` 在 stderr 链路旁常驻一个 dormant 的 [`IpcAppend`]，`init_ipc`
//! 在进程共享节点就绪后同线程建端并返回 [`LogIpc`] 句柄（调用方在主循环
//! 作用域持有）；此后每条日志同时写 stderr（原链路保留）与 IPC（聚合端
//! `firefly-viz` 统一写 rerun `TextLog`，持久可检索）。IPC 不可用/未初始化
//! 时静默降级为纯 stderr，不阻断进程。
//!
//! 线程模型（对照 logforth 官方 `appenders/async`：`Async` + `DropIncoming`）：
//! logforth 的 `append` 在**调用线程同步执行**——`IpcAppend::append` 只做
//! 纯内存格式化（`Record` → [`QueuedLog`] 入队，无锁化快照 sim 时钟/trace
//! 上下文）；真正的 IPC 发布由**主循环驱动**（每 tick [`pump_log_ipc`] 排空
//! channel、零拷贝发送）。iceoryx2 发布端不出 [`LogIpc`] 句柄的作用域——
//! 调用方持有句柄、主循环线程 pump，从构造上不可能跨线程触碰，无 `unsafe`。
//!
//! 背压语义：channel 有界（128），满即丢——日志永不阻塞计算线程。
//!
//! 用法：
//! ```
//! firefly_observability::init();
//! // ...
//! firefly_observability::flush();
//! ```
//!
//! 聚合版（节点就绪后调用一次，主循环每 tick 加三行）：
//! ```ignore
//! let node = firefly_pubsub::node::create_node()?;
//! let log_ipc = firefly_observability::init_ipc(&node, "vio");
//! // 主循环内：
//! // firefly_observability::set_sim_time(t);
//! // firefly_observability::pump_log_ipc(&log_ipc);
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};

use fastrace::prelude::*;
use firefly_pubsub::log::{LogMessage, LogPublisher, level};
use firefly_pubsub::node::IpcNode;
use logforth::filter::rustlog::RustLogFilterBuilder;

pub fn init() {
    logforth::starter_log::builder()
        .dispatch(|d| {
            d.filter(RustLogFilterBuilder::from_default_env().build())
                .diagnostic(logforth::diagnostic::FastraceDiagnostic::default())
                .append(logforth::append::Stderr::default())
        })
        .dispatch(|d| d.append(logforth::append::FastraceEvent::default()))
        // 聚合半边：`init_ipc` 接通前 dormant（无发布端即 no-op，见
        // [`IpcAppend::append`]），单次安装、此后不再 `try_apply`。
        .dispatch(|d| {
            d.filter(RustLogFilterBuilder::from_default_env().build())
                .append(IpcAppend)
        })
        .apply();

    fastrace::set_reporter(
        fastrace::collector::ConsoleReporter,
        fastrace::collector::Config::default(),
    );
}

/// 当前 sim 时钟（秒）的位表示；`NaN` = 尚无 sim 时钟（聚合端回落墙钟轴）。
static SIM_TIME_BITS: AtomicU64 = AtomicU64::new(f64::NAN.to_bits());

/// 更新本进程的 sim 时钟（主循环每 tick 调用一次；`firefly-viz` 聚合端除外）。
pub fn set_sim_time(t: f64) {
    SIM_TIME_BITS.store(t.to_bits(), Ordering::Relaxed);
}

fn sim_time_bits() -> u64 {
    SIM_TIME_BITS.load(Ordering::Relaxed)
}

/// 进程标签（`init_ipc` 写入；`IpcAppend` 每次 `append` 时读取）。
static TAG: Mutex<String> = Mutex::new(String::new());

/// 日志聚合句柄（调用方持有：主循环作用域，与 `node` 同生命周期）。
/// 发布端只活在创建线程（主循环线程）——调用方每 tick 把句柄传给
/// [`pump_log_ipc`]，从构造上不可能跨线程触碰，无 `unsafe`。
/// `None` = 纯 stderr 模式（`init_ipc` 建端失败时）。
pub struct LogIpc {
    publisher: Option<LogPublisher>,
}

/// 聚合链路是否已武装（`init_ipc` 置位；`IpcAppend` 的 dormant 判定；
/// `Mutex<bool>` 即 `Sync`，判定位只读、写一次，无线程模型问题）。
static IPC_ARMED: Mutex<bool> = Mutex::new(false);

/// 待发送日志（`append` 调用线程内格式化完毕，主循环只做 IPC 发布）。
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

/// 有界 channel 容量：日志低频，128 足够突发；满即丢（见 [`IpcAppend`]）。
const QUEUE_CAP: usize = 128;

/// 跨线程队列（`append` 入队 → 主循环 `pump_log_ipc` 出队；标准库 mpsc 足够）。
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
/// 内部标签/武装锁中毒时（调用方 bug，不应发生）。
#[must_use]
pub fn init_ipc(node: &IpcNode, tag: &str) -> LogIpc {
    tag.clone_into(&mut TAG.lock().expect("log tag mutex"));
    let _ = log_queue();
    *IPC_ARMED.lock().expect("ipc armed mutex") = true;
    match LogPublisher::new(node) {
        Ok(publisher) => {
            log::info!("日志聚合已挂载（tag={tag}，Firefly/Log → firefly-viz）");
            LogIpc {
                publisher: Some(publisher),
            }
        }
        Err(e) => {
            log::warn!("日志聚合挂载失败（降级纯 stderr）：{e}");
            LogIpc { publisher: None }
        }
    }
}

/// 主循环驱动：排空日志队列并零拷贝发布（每 tick 调用一次，见模块文档）。
///
/// # Panics
/// 内部队列锁中毒时（调用方 bug，不应发生）。
pub fn pump_log_ipc(log_ipc: &LogIpc) {
    let Some(publisher) = log_ipc.publisher.as_ref() else {
        return;
    };
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
        // 聚合失败静默丢弃：可视化不阻断计算链路
        let _ = publisher.publish(msg);
    }
}

/// logforth → `Firefly/Log` 的聚合 appender（双 sink 中的 IPC 半边）。
///
/// 每条 `Record` 先在调用线程内格式化为 [`QueuedLog`]（含 trace 链与
/// sim 时钟快照——调用线程的 fastrace 上下文，事后不可追补），再
/// `try_send` 进有界队列；队列满时静默丢弃，永不阻塞计算线程。
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
        // 满即丢：日志路径无 temporary 重试语义
        let _ = log_queue()
            .tx
            .lock()
            .expect("log queue tx mutex")
            .try_send(queued);
        Ok(())
    }

    fn flush(&self) -> Result<(), logforth::Error> {
        Ok(())
    }
}

pub fn flush() {
    // 日志队列由主循环 `pump_log_ipc(&log_ipc)` 排空（退出路径调一次，
    // 残余落盘）；此处只刷 trace（logforth stderr 无缓冲）。
    // 注：`flush()` 签名保持无参——调用方退出路径手补一次 pump 即可。
    fastrace::flush();
}
