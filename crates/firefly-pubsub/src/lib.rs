//! iceoryx2 zero-copy 发布订阅层（对照 `docs/architecture.md` 的
//! `firefly-pubsub`）。
//!
//! - [`trace`]：Trace 上下文中间件——每条消息的 User Header 自动携带
//!   发布端 fastrace 活动 span 的 `(trace_id, span_id, sampled)` 与双发送
//!   时间戳（jiff 墙钟 + `CLOCK_MONOTONIC`），订阅端可续接跨进程 span 树、
//!   计算端到端延迟（W3C Trace Context 对齐）；
//! - [`odom`]：`OdomMessage`——`#[repr(C)]` 定长零拷贝消息（`ZeroCopySend`）；
//! - [`publish`]：发布端封装（Node/Service/Publisher 生命周期管理 +
//!   自动注入 trace 上下文）；
//! - [`subscriber`]：订阅端封装（读取 User Header 中的 trace 上下文并续接）。
//!
//! 消息设计约束（iceoryx2 `ZeroCopySend` 要求）：自包含、无堆指针、
//! 统一内存布局、`'static`、不实现 `Drop`。

pub mod imu;
pub mod odom;
pub mod publish;
pub mod subscriber;
pub mod trace;
