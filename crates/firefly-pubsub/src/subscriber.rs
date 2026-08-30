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

use crate::node::IpcNode;
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
///
/// 节点由调用方持有（进程共享单节点，见 [`crate::node`]），端口只借用其
/// 创建服务；Drop 顺序由调用方作用域保证（节点最后释放）。
pub struct Subscriber<T: Debug + ZeroCopySend + 'static> {
    /// 订阅器句柄。
    subscriber: Iox2Subscriber<ipc::Service, T, TraceContext>,
}

impl<T: Debug + ZeroCopySend + 'static> Subscriber<T> {
    /// 以进程共享节点 + 自定义话题名打开订阅器。
    ///
    /// `buffer_size` 控制 iceoryx2 内部环形缓冲区深度：
    /// - IMU（100 Hz）建议 ≥20，避免 VIO 处理期间丢帧；
    /// - 相机（10 Hz）建议 1，只保留最新帧避免左右目交叉。
    ///
    /// # Errors
    /// Service/Subscriber 创建失败（IPC 资源不可用等）。
    pub fn with_topic_and_buffer(
        node: &IpcNode,
        topic: &str,
        buffer_size: usize,
    ) -> Result<Self, firefly_error::Error> {
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
        let subscriber = service
            .subscriber_builder()
            .buffer_size(buffer_size)
            .create()
            .map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("创建订阅器失败: {e:?}"),
                )
            })?;
        Ok(Self { subscriber })
    }

    /// 以自定义话题名打开订阅器（`buffer_size`=1，适用于相机等低频话题）。
    ///
    /// # Errors
    /// Service/Subscriber 创建失败（IPC 资源不可用等）。
    pub fn with_topic(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        Self::with_topic_and_buffer(node, topic, 1)
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

/// 校正后里程计订阅器（话题 `Firefly/CorrectedOdometry`）。
pub struct CorrectedOdomSubscriber(Subscriber<OdomMessage>);

impl OdomSubscriber {
    /// 打开 odom 话题的订阅器（与发布端话题名一致）。
    ///
    /// # Errors
    /// 见 [`Subscriber::with_topic`]。
    pub fn new(node: &IpcNode) -> Result<Self, firefly_error::Error> {
        Self::with_topic(node, crate::publish::ODOM_TOPIC)
    }

    /// 以自定义话题名打开 odom 订阅器。
    ///
    /// # Errors
    /// 见 [`Subscriber::with_topic`]。
    pub fn with_topic(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        Ok(Self(Subscriber::with_topic(node, topic)?))
    }

    /// 接收一条 odom 消息（见 [`Subscriber::receive`]）。
    ///
    /// # Errors
    /// 见 [`Subscriber::receive`]。
    pub fn receive(&self) -> Result<Option<ReceivedOdom>, firefly_error::Error> {
        self.0.receive()
    }
}

impl CorrectedOdomSubscriber {
    /// 打开校正后里程计订阅器。
    ///
    /// # Errors
    /// 见 [`Subscriber::with_topic`]。
    pub fn new(node: &IpcNode) -> Result<Self, firefly_error::Error> {
        Self::with_topic(node, crate::publish::CORRECTED_ODOM_TOPIC)
    }

    /// 以自定义话题名打开订阅器。
    ///
    /// # Errors
    /// 见 [`Subscriber::with_topic`]。
    pub fn with_topic(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        Ok(Self(Subscriber::with_topic(node, topic)?))
    }

    /// 接收一条校正后里程计（见 [`Subscriber::receive`]）。
    ///
    /// # Errors
    /// 见 [`Subscriber::receive`]。
    pub fn receive(&self) -> Result<Option<ReceivedOdom>, firefly_error::Error> {
        self.0.receive()
    }
}
