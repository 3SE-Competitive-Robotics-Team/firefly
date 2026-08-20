//! 订阅端封装：泛型零拷贝订阅器 + 按话题的命名封装。
//!
//! 与 [`crate::publish`] 对称：`NodeBuilder` → `service_builder(...)`
//! `.publish_subscribe::<T>()` → `.user_header::<TraceContext>()` →
//! `subscriber_builder().create()`。订阅端必须声明与发布端相同的
//! User Header 类型（iceoryx2 连接时校验签名兼容）。
//!
//! 收到的样本携带发布端 fastrace 活动 span 的上下文
//! （[`TraceContext::continue_span`] 可续接为跨进程 span 树）与双发送
//! 时间戳（可计算端到端延迟）。
//!
//! [`Subscriber`] 为泛型核心；[`OdomSubscriber`] 是 odom 话题的命名封装。

use std::fmt::Debug;

use iceoryx2::port::subscriber::Subscriber as Iox2Subscriber;
use iceoryx2::prelude::*;

use crate::odom::OdomMessage;
use crate::trace::TraceContext;

/// 收到的零拷贝样本（`*sample` 解引用取 payload，`sample.user_header()` 取上下文）。
pub type Received<T> = iceoryx2::sample::Sample<ipc::Service, T, TraceContext>;

/// 收到的 odom 样本。
pub type ReceivedOdom = Received<OdomMessage>;

/// 泛型零拷贝订阅器（iceoryx2 ipc 服务，User Header 携带 trace 上下文）。
///
/// 约束对齐 iceoryx2 0.9.3 `publish_subscribe`（`Debug + ZeroCopySend`；
/// 0.9.999 起新增 `IceoryxSend`，升级时补上）。
pub struct Subscriber<T: Debug + ZeroCopySend + 'static> {
    /// 服务节点（持有服务生命周期）。
    _node: Node<ipc::Service>,
    /// 订阅器句柄。
    subscriber: Iox2Subscriber<ipc::Service, T, TraceContext>,
}

impl<T: Debug + ZeroCopySend + 'static> Subscriber<T> {
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
            .publish_subscribe::<T>()
            .user_header::<TraceContext>()
            .open_or_create()
            .map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("打开/创建话题 `{topic}` 失败: {e:?}"),
                )
            })?;
        // depth=1：只保留最新帧，避免vio处理慢时left/right交叉导致配对失败
        let subscriber = service
            .subscriber_builder()
            .buffer_size(1)
            .create()
            .map_err(|e| {
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

    /// 接收一条消息（非阻塞）：返回 `None` 表示当前无新样本。
    ///
    /// 样本的 User Header 携带发布端注入的 trace 上下文与发送时间戳
    /// （见 [`TraceContext`]），可用 [`TraceContext::continue_span`] 续接。
    ///
    /// # Errors
    /// 接收失败（连接中断等）。
    pub fn receive(&self) -> Result<Option<Received<T>>, firefly_error::Error> {
        self.subscriber.receive().map_err(|e| {
            firefly_error::Error::temporary(
                firefly_error::ErrorKind::Internal,
                format!("接收样本失败: {e:?}"),
            )
        })
    }
}

/// 里程计订阅器（话题 `Firefly/Odometry`，泛型核心的命名封装）。
pub struct OdomSubscriber(Subscriber<OdomMessage>);

impl OdomSubscriber {
    /// 打开 odom 话题的订阅器（与发布端话题名一致）。
    ///
    /// # Errors
    /// 见 [`Subscriber::with_topic`]。
    pub fn new() -> Result<Self, firefly_error::Error> {
        Self::with_topic(crate::publish::ODOM_TOPIC)
    }

    /// 以自定义话题名打开 odom 订阅器。
    ///
    /// # Errors
    /// 见 [`Subscriber::with_topic`]。
    pub fn with_topic(topic: &str) -> Result<Self, firefly_error::Error> {
        Ok(Self(Subscriber::with_topic(topic)?))
    }

    /// 接收一条 odom 消息（见 [`Subscriber::receive`]）。
    ///
    /// # Errors
    /// 见 [`Subscriber::receive`]。
    pub fn receive(&self) -> Result<Option<ReceivedOdom>, firefly_error::Error> {
        self.0.receive()
    }
}
