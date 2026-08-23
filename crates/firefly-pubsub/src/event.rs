//! 事件唤醒层：pubsub 数据面（拉模型）的推模型补充。
//!
//! 对照 iceoryx2 官方 `examples/rust/event_based_communication`：每个话题
//! 配对一个**同名** event service——发布端 send 后 notify，订阅端把
//! [`TopicListener`] 挂进 `WaitSet` 即到即醒，替代固定节拍轮询。
//!
//! - 通知不携带数据；trace 上下文仍走 pubsub User Header（[`crate::trace`]）
//! - `EventId` 跨语言统一为 [`EVENT_ID_SENT_SAMPLE`]（Python 侧 `iox2.EventId.new(0)`）
//! - notify 无监听者时静默成功（iceoryx2 返回 Ok(0)），订阅方缺席不构成错误

use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::prelude::*;
use iceoryx2::service::ipc::Service;

use crate::node::IpcNode;

/// 事件 id：「该话题有新样本」（当前唯一事件类型）。
pub const EVENT_ID_SENT_SAMPLE: usize = 0;

/// 相机对事件话题（左+右目成对发布完成后单次通知；仅 event 服务，无数据）。
/// 成对通知避免半对唤醒——左右目时间戳配对失败会让消费端白醒一次。
pub const CAMERA_PAIR_TOPIC: &str = "Firefly/CameraPair";

fn open_event(
    node: &IpcNode,
    topic: &str,
) -> Result<iceoryx2::service::port_factory::event::PortFactory<Service>, firefly_error::Error> {
    node.service_builder(&topic.try_into().map_err(|e| {
        firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            format!("非法话题名 `{topic}`: {e:?}"),
        )
    })?)
    .event()
    .open_or_create()
    .map_err(|e| {
        firefly_error::Error::new(
            firefly_error::ErrorKind::Internal,
            format!("打开/创建事件服务 `{topic}` 失败: {e:?}"),
        )
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
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("创建事件通知端失败: {e:?}"),
                )
            })?;
        Ok(Self { notifier })
    }

    /// 通知「有新样本」。无监听者时静默成功；失败仅记 debug 日志、不阻断
    /// 数据路径——唤醒缺失最多退化为订阅端的兜底节拍。
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
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("创建事件监听端失败: {e:?}"),
                )
            })?;
        Ok(Self { listener })
    }

    /// 排空全部未处理通知。官方纪律：WaitSet 唤醒后必须排空监听端，
    /// 否则 fd 持续可读导致 busy-loop。当前只有一种事件 id，计数即排空。
    ///
    /// # Errors
    /// 底层 event 监听失败。
    pub fn drain(&self) -> Result<usize, firefly_error::Error> {
        let mut count = 0usize;
        self.listener.try_wait_all(|_| count += 1).map_err(|e| {
            firefly_error::Error::temporary(
                firefly_error::ErrorKind::Internal,
                format!("排空事件通知失败: {e:?}"),
            )
        })?;
        Ok(count)
    }
}

impl FileDescriptorBased for TopicListener {
    fn file_descriptor(&self) -> &FileDescriptor {
        self.listener.file_descriptor()
    }
}

impl SynchronousMultiplexing for TopicListener {}
