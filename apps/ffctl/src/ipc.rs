//! IPC 投递：一次性 CLI 与订阅端的连接是异步建立的，投递策略集中在这里。
//!
//! `iceoryx2` 的 `send` 在无订阅端时不报错（样本直接丢弃），而**首次**投递往往只建立
//! 连接、样本进不了订阅端缓冲区——所以每条消息在 [`DELIVER_TIMEOUT`] 窗口内反复投递，
//! 以最后一次的接收端数为准；窗口内始终没人订阅即报错（退出码 1），不静默丢弃。
//!
//! 重复投递无害：指令按 `sequence` 去重（同一条只生效一次），目标点取最新。

use std::fmt::Debug;
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use firefly_error::{Error, ErrorKind};
use firefly_pubsub::command::{COMMAND_TOPIC, CommandMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::publish::Publisher;
use iceoryx2::prelude::ZeroCopySend;

/// 投递期限（一次性 CLI 与订阅端的连接是异步建立的，需重试到连接建立）。
const DELIVER_TIMEOUT: Duration = Duration::from_millis(300);
/// 投递重试间隔。
const DELIVER_RETRY: Duration = Duration::from_millis(30);

/// 打开某话题的发布端（节点只在本进程存活期间存在）。
///
/// # Errors
/// 节点或端口创建失败（IPC 资源不可用等）。
pub(crate) fn new_publisher<T: Debug + ZeroCopySend + 'static>(
    topic: &str,
) -> Result<Publisher<T>, Error> {
    let node = create_node()?;
    Publisher::<T>::with_topic(&node, topic)
}

/// 投递一条地面站指令（`Firefly/Command`）。
///
/// # Errors
/// 端口创建失败，或投递期限内无订阅端（飞控未启动）。
pub(crate) fn send_command(kind: u8, altitude: f64, label: &str) -> Result<(), Error> {
    let publisher = new_publisher::<CommandMessage>(COMMAND_TOPIC)?;
    let nanos = wall_nanos();
    let msg = CommandMessage {
        timestamp: nanos as f64 / 1e9,
        kind,
        altitude,
        // 指令序号：跨进程调用必须严格递增，飞控按它去重（重复投递只生效一次）
        sequence: nanos,
    };
    let receivers = deliver(&publisher, msg, COMMAND_TOPIC)?;
    println!("已投递指令 {label} → {COMMAND_TOPIC}（{receivers} 个订阅端）");
    Ok(())
}

/// 投递到至少一个订阅端（或超时报错），返回最后一次投递的接收端数。
///
/// 不看中间计数：首次投递可能只建立连接而样本丢失，以最后一次为准才可靠。
///
/// # Errors
/// 发送失败，或期限内始终无订阅端（目标进程没起）。
pub(crate) fn deliver<T: Copy + Debug + ZeroCopySend + 'static>(
    publisher: &Publisher<T>,
    msg: T,
    topic: &str,
) -> Result<usize, Error> {
    let deadline = Instant::now() + DELIVER_TIMEOUT;
    let mut receivers;
    loop {
        receivers = publisher.publish_counted(msg)?.0;
        if Instant::now() >= deadline {
            break;
        }
        thread::sleep(DELIVER_RETRY);
    }
    if receivers == 0 {
        return Err(Error::new(
            ErrorKind::Timeout,
            format!("`{topic}` 无订阅端（目标进程未启动？），消息未投递"),
        ));
    }
    Ok(receivers)
}

/// 墙钟纳秒（自 Unix 纪元；跨调用严格递增，用作指令序号）。
pub(crate) fn wall_nanos() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_nanos() as u64)
}

/// 墙钟秒（消息里的诊断时间戳；飞控/planner 只当参考，不参与时序判断）。
pub(crate) fn wall_seconds() -> f64 {
    wall_nanos() as f64 / 1e9
}

/// 命令结果的统一出口：成功 `0`，失败把**消息本体**写 stderr 并退 `1`。
///
/// 只打消息本体：`firefly_error` 的 `Display` 带 kind 与源码位置（给日志的），
/// 操作员要的是一行原因。
pub(crate) fn report(result: Result<(), Error>) -> ExitCode {
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("ffctl: {}", err.message());
            ExitCode::FAILURE
        }
    }
}
