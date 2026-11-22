//! 每个 pubsub 话题配对同名事件服务，监听者收到通知后读取数据。
//! 通知不携带数据；trace 上下文由 pubsub 用户头传递。
//! 跨语言事件 ID 为 [`EVENT_ID_SENT_SAMPLE`]；无监听者时 notify 成功返回 0。

use firefly_error::{Error, ErrorKind};
use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::prelude::*;
use iceoryx2::service::ipc::Service;

use crate::node::IpcNode;

/// 事件 id：「该话题有新样本」（当前唯一事件类型）。
pub const EVENT_ID_SENT_SAMPLE: usize = 0;

/// 左右目图像都发布后，生产者用此事件唤醒一次订阅者。图像由各自的数据话题传输。
pub const CAMERA_PAIR_TOPIC: &str = "Firefly/CameraPair";

fn open_event(
    node: &IpcNode,
    topic: &str,
) -> Result<iceoryx2::service::port_factory::event::PortFactory<Service>, firefly_error::Error> {
    node.service_builder(&topic.try_into().map_err(|e| {
        Error::caused_by(ErrorKind::InvalidArgument, "非法话题名", e).with_context("topic", topic)
    })?)
    .event()
    .open_or_create()
    .map_err(|e| {
        Error::caused_by(ErrorKind::Unavailable, "打开/创建事件服务 失败", e)
            .with_context("topic", topic)
    })
}

/// 话题事件通知端：与发布器配对，send 成功后调用 [`Self::notify_sent_sample`]。
#[derive(Debug)]
pub struct TopicNotifier {
    notifier: Notifier<Service>,
}

impl TopicNotifier {
    /// 打开/创建话题同名 event service 的通知端。
    ///
    /// # Errors
    /// event service / notifier 创建失败。
    pub fn with_topic(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        let notifier = open_event(node, topic)?
            .notifier_builder()
            .create()
            .map_err(|e| {
                Error::caused_by(ErrorKind::Unavailable, "创建事件通知端失败", e)
                    .with_context("topic", topic)
            })?;
        Ok(Self { notifier })
    }

    /// 通知监听者读取已发布的样本。通知失败时记录 debug 日志，订阅者仍可轮询数据。
    pub fn notify_sent_sample(&self) {
        if let Err(e) = self
            .notifier
            .notify_with_custom_event_id(EventId::new(EVENT_ID_SENT_SAMPLE))
        {
            log::debug!("事件通知失败（忽略）: {e:?}");
        }
    }
}

/// 话题事件监听端：挂进 WaitSet（notification/interval attachment）等待唤醒。
#[derive(Debug)]
pub struct TopicListener {
    listener: Listener<Service>,
}

impl TopicListener {
    /// 打开/创建话题同名 event service 的监听端。
    ///
    /// # Errors
    /// event service / listener 创建失败。
    pub fn with_topic(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        let listener = open_event(node, topic)?
            .listener_builder()
            .create()
            .map_err(|e| {
                Error::caused_by(ErrorKind::Unavailable, "创建事件监听端失败", e)
                    .with_context("topic", topic)
            })?;
        Ok(Self { listener })
    }

    /// 排空全部未处理通知。官方纪律：WaitSet 唤醒后必须排空监听端，
    /// 否则 fd 持续可读导致 busy-loop。当前只有一种事件 id，计数即排空。
    ///
    /// # Errors
    /// 底层 event 监听失败。
    pub fn drain(&self) -> Result<usize, firefly_error::Error> {
        let count = self.listener.try_wait(|_| {}).map_err(|e| {
            Error::caused_by(ErrorKind::Unavailable, "排空事件通知失败", e)
                .with_status(firefly_error::ErrorStatus::Temporary)
        })?;
        Ok(usize::try_from(count).unwrap_or(usize::MAX))
    }
}

impl FileDescriptorBased for TopicListener {
    fn file_descriptor(&self) -> &FileDescriptor {
        self.listener.file_descriptor()
    }
}

impl SynchronousMultiplexing for TopicListener {}
