//! 订阅端封装：打开带 Trace 上下文 User Header 的 iceoryx2 服务。
//!
//! 与 [`crate::publish`] 对称：`NodeBuilder` → `service_builder(...)`
//! `.publish_subscribe::<T>()` → `.user_header::<TraceContext>()` →
//! `subscriber_builder().create()`。订阅端必须声明与发布端相同的
//! User Header 类型（iceoryx2 连接时校验签名兼容）。
//!
//! 收到的样本携带发布端 fastrace 活动 span 的上下文
//! （[`TraceContext::continue_span`] 可续接为跨进程 span 树）与双发送
//! 时间戳（可计算端到端延迟）。

use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;

use crate::odom::OdomMessage;
use crate::trace::TraceContext;

/// 收到的 odom 样本（零拷贝）：`*sample` 解引用取 payload，
/// `sample.user_header()` 取 trace 上下文。
pub type ReceivedOdom = iceoryx2::sample::Sample<ipc::Service, OdomMessage, TraceContext>;

/// 里程计订阅器（iceoryx2 ipc 服务，User Header 携带 trace 上下文）。
pub struct OdomSubscriber {
    /// 服务节点（持有服务生命周期）。
    _node: Node<ipc::Service>,
    /// 订阅器句柄。
    subscriber: Subscriber<ipc::Service, OdomMessage, TraceContext>,
}

impl OdomSubscriber {
    /// 打开 odom 话题的订阅器（与发布端话题名一致）。
    ///
    /// # Errors
    /// Node/Service/Subscriber 创建失败（IPC 资源不可用等）。
    pub fn new() -> Result<Self, firefly_error::Error> {
        Self::with_topic(crate::publish::ODOM_TOPIC)
    }

    /// 以自定义话题名打开订阅器。
    ///
    /// # Errors
    /// Node/Service/Subscriber 创建失败（IPC 资源不可用等）。
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
            .publish_subscribe::<OdomMessage>()
            .user_header::<TraceContext>()
            .open_or_create()
            .map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("打开/创建话题 `{topic}` 失败: {e:?}"),
                )
            })?;
        let subscriber = service.subscriber_builder().create().map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::Internal,
                format!("创建订阅器失败: {e:?}"),
            )
        })?;
        Ok(Self {
            _node: node,
            subscriber,
        })
    }

    /// 接收一条 odom 消息（非阻塞）：返回 `None` 表示当前无新样本。
    ///
    /// 样本的 User Header 携带发布端注入的 trace 上下文与发送时间戳
    /// （见 [`TraceContext`]），可用 [`TraceContext::continue_span`] 续接。
    ///
    /// # Errors
    /// 接收失败（连接中断等）。
    pub fn receive(&self) -> Result<Option<ReceivedOdom>, firefly_error::Error> {
        self.subscriber.receive().map_err(|e| {
            firefly_error::Error::temporary(
                firefly_error::ErrorKind::Internal,
                format!("接收样本失败: {e:?}"),
            )
        })
    }
}
