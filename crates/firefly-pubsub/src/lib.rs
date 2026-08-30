//! iceoryx2 zero-copy 发布订阅层（对照 `docs/architecture.md` 的
//! `firefly-pubsub`）。
//!
//! - [`trace`]：Trace 上下文中间件——每条消息的 User Header 自动携带
//!   发布端 fastrace 活动 span 的 `(trace_id, span_id, sampled)` 与双发送
//!   时间戳（jiff 墙钟 + `CLOCK_MONOTONIC`），订阅端可续接跨进程 span 树、
//!   计算端到端延迟（W3C Trace Context 对齐）；
//! - [`odom`]：`OdomMessage`——`#[repr(C)]` 定长零拷贝消息（`ZeroCopySend`）；
//! - [`imu`]：`ImuMessage`——原始 IMU（角速度 + 比力）；
//! - [`camera`]：`GrayImageMessage`/`DepthImageMessage`——双目灰度 + 深度图；
//! - [`reference`]：`ReferenceMessage`——规划轨迹的参考状态（闭环控制回传）；
//! - [`goal`]：`GoalMessage`——外部工具发布的飞行目标（动态重目标入口）；
//! - [`viz`]：`VizMessage`——统一可视化消息（Rust 计算线程零 IO，经
//!   `Firefly/Viz` 话题由 `firefly-viz` Python 进程统一写 rerun）；
//! - [`publish`]/[`subscriber`]：泛型发布/订阅端（自动注入/续接 trace 上下文）；
//! - [`event`]：事件唤醒层——每话题配对同名 event service，发布后 notify、
//!   订阅端 `Listener` 挂 `WaitSet` 即到即醒（对照 iceoryx2 官方 event 示例）。
//!
//! 消息设计约束（iceoryx2 `ZeroCopySend` 要求）：自包含、无堆指针、
//! 统一内存布局、`'static`、不实现 `Drop`。

pub mod camera;
pub mod event;
pub mod goal;
pub mod imu;
pub mod node;
pub mod odom;
pub mod publish;
pub mod reference;
pub mod subscriber;
pub mod trace;
pub mod viz;
