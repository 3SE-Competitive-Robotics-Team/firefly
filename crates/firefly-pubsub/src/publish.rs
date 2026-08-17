//! 发布端封装：泛型零拷贝发布器 + 按话题的命名封装。
//!
//! 对照 iceoryx2 最佳实践（`examples/rust/publish_subscribe_with_user_header`）：
//! `NodeBuilder` → `service_builder(...).publish_subscribe::<T>()` →
//! `user_header::<TraceContext>()` → `publisher_builder().create()` →
//! `loan_uninit`/`write_payload`/`send`。trace 上下文与时间戳放在
//! User Header（零拷贝），payload 保持精简。
//!
//! [`Publisher`] 为泛型核心；[`OdomPublisher`] 是 odom 话题的命名封装。

use std::fmt::Debug;

use fastrace::prelude::*;
use iceoryx2::port::publisher::Publisher as Iox2Publisher;
use iceoryx2::prelude::*;

use crate::odom::OdomMessage;
use crate::trace::TraceContext;

/// odom 话题名（对照 `docs/architecture.md` 的 `topic: odom`）。
pub const ODOM_TOPIC: &str = "Firefly/Odometry";

/// 泛型零拷贝发布端（iceoryx2 ipc 服务，User Header 携带 trace 上下文）。
///
/// 约束对齐 iceoryx2 0.9.3 `publish_subscribe`（`Debug + ZeroCopySend`；
/// 0.9.999 起新增 `IceoryxSend`，升级时补上）。
pub struct Publisher<T: Debug + ZeroCopySend + 'static> {
    /// 服务节点（持有服务生命周期）。
    _node: Node<ipc::Service>,
    /// 发布器句柄。
    publisher: Iox2Publisher<ipc::Service, T, TraceContext>,
}

impl<T: Debug + ZeroCopySend + 'static> Publisher<T> {
    /// 以自定义话题名创建发布器。
    ///
    /// # Errors
    /// Node/Service/Publisher 创建失败（IPC 资源不可用等）。
    pub fn with_topic(topic: &str) -> Result<Self, firefly_error::Error> {
        let node = NodeBuilder::new().create::<ipc::Service>().map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::Internal,
                format!("创建 iceoryx2 node 失败: {e:?}"),
            )
        })?;
        let service = node
            .service_builder(&topic.try_into().map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::InvalidArgument,
                    format!("非法话题名 `{topic}`: {e:?}"),
                )
            })?)
            .publish_subscribe::<T>()
            .user_header::<TraceContext>()
            .open_or_create()
            .map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("打开/创建话题 `{topic}` 失败: {e:?}"),
                )
            })?;
        let publisher = service.publisher_builder().create().map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::Internal,
                format!("创建发布器失败: {e:?}"),
            )
        })?;
        Ok(Self {
            _node: node,
            publisher,
        })
    }

    /// 发布一条消息：**自动注入当前 fastrace 活动 span 的 trace 上下文**
    /// （`trace_id` + `span_id` + `sampled`）与双发送时间戳到 User Header，
    /// 零拷贝发送。返回实际写入的上下文（供调用方记录/关联）。
    ///
    /// 不在任何 fastrace span 内时，上下文仅携带时间戳
    /// （[`TraceContext::is_traced`] 为 `false`）。
    ///
    /// # Errors
    /// 借出样本失败或发送失败（如发布端超时）。
    pub fn publish(&self, msg: T) -> Result<TraceContext, firefly_error::Error> {
        // Trace 上下文中间件：注入当前活动 span（无 span 时仅带时间戳）
        let ctx = match SpanContext::current_local_parent() {
            Some(sc) => TraceContext::from_span_context(sc),
            None => TraceContext::timestamps_only(),
        };
        let mut sample = self.publisher.loan_uninit().map_err(|e| {
            firefly_error::Error::temporary(
                firefly_error::ErrorKind::ResourceExhausted,
                format!("借出零拷贝样本失败: {e:?}"),
            )
        })?;
        *sample.user_header_mut() = ctx;
        let sample = sample.write_payload(msg);
        sample.send().map(|_| ctx).map_err(|e| {
            firefly_error::Error::temporary(
                firefly_error::ErrorKind::ResourceExhausted,
                format!("发送样本失败: {e:?}"),
            )
        })
    }
}

/// 里程计发布器（话题 `Firefly/Odometry`，泛型核心的命名封装）。
pub struct OdomPublisher(Publisher<OdomMessage>);

impl OdomPublisher {
    /// 打开 odom 话题的发布器。
    ///
    /// # Errors
    /// 见 [`Publisher::with_topic`]。
    pub fn new() -> Result<Self, firefly_error::Error> {
        Self::with_topic(ODOM_TOPIC)
    }

    /// 以自定义话题名打开 odom 发布器。
    ///
    /// # Errors
    /// 见 [`Publisher::with_topic`]。
    pub fn with_topic(topic: &str) -> Result<Self, firefly_error::Error> {
        Ok(Self(Publisher::with_topic(topic)?))
    }

    /// 发布一条 odom 消息（trace 上下文自动注入，见 [`Publisher::publish`]）。
    ///
    /// # Errors
    /// 见 [`Publisher::publish`]。
    pub fn publish(&self, msg: OdomMessage) -> Result<TraceContext, firefly_error::Error> {
        self.0.publish(msg)
    }
}
