//! 泛型订阅端：统一服务容量与接收缓冲配置，保留 trace 用户头。

use std::fmt::Debug;

use iceoryx2::port::subscriber::Subscriber as Iox2Subscriber;
use iceoryx2::prelude::*;

use crate::node::IpcNode;
use crate::trace::TraceContext;

/// 收到的零拷贝样本（`*sample` 解引用取 payload，`sample.user_header()` 取上下文）。
pub type Received<T> = iceoryx2::sample::Sample<ipc::Service, T, TraceContext>;

/// 接收队列与服务容量；服务容量由首次创建方确定，后续端口只能要求兼容的容量。
#[derive(Debug, Clone, Copy)]
pub struct SubscriberOptions {
    pub buffer_size: usize,
    /// 服务允许的队列上限；None 时队列大于 1 则取队列深度，否则用 iceoryx2 默认值。
    pub service_max_buffer_size: Option<usize>,
    /// 服务允许的发布者数；None 使用 iceoryx2 默认值。
    pub max_publishers: Option<usize>,
}

impl Default for SubscriberOptions {
    fn default() -> Self {
        Self {
            buffer_size: 1,
            service_max_buffer_size: None,
            max_publishers: None,
        }
    }
}

/// 使用调用方的共享节点创建端口；线程与退出约定见 [`crate::node`]。
pub struct Subscriber<T: Debug + IceoryxSend + ?Sized + 'static> {
    subscriber: Iox2Subscriber<ipc::Service, T, TraceContext>,
}

impl<T: Debug + ZeroCopySend + 'static> Subscriber<T> {
    /// # Errors
    /// 缓冲深度无效，或 Service/Subscriber 创建失败（IPC 资源不可用等）。
    pub fn with_topic_and_buffer(
        node: &IpcNode,
        topic: &str,
        buffer_size: usize,
    ) -> Result<Self, firefly_error::Error> {
        Self::with_options(
            node,
            topic,
            SubscriberOptions {
                buffer_size,
                ..Default::default()
            },
        )
    }

    /// # Errors
    /// 队列深度为零、超过声明的服务上限，或服务/订阅器创建失败。
    pub fn with_options(
        node: &IpcNode,
        topic: &str,
        options: SubscriberOptions,
    ) -> Result<Self, firefly_error::Error> {
        Self::open(node, topic, options, None)
    }

    /// Subscribe to a static snapshot, including the current publisher's last value.
    /// # Errors
    /// Returns service or subscriber creation errors.
    pub fn retained(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        Self::open(
            node,
            topic,
            SubscriberOptions {
                buffer_size: 1,
                service_max_buffer_size: Some(1),
                max_publishers: Some(1),
            },
            Some(1),
        )
    }

    fn open(
        node: &IpcNode,
        topic: &str,
        options: SubscriberOptions,
        history: Option<usize>,
    ) -> Result<Self, firefly_error::Error> {
        let SubscriberOptions {
            buffer_size,
            service_max_buffer_size,
            max_publishers,
        } = options;
        if buffer_size == 0 || service_max_buffer_size.is_some_and(|max| buffer_size > max) {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                format!("话题 `{topic}` 的订阅缓冲必须大于零且不超过服务容量"),
            ));
        }
        let mut builder = node
            .service_builder(&topic.try_into().map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::InvalidArgument,
                    format!("非法话题名 `{topic}`: {e:?}"),
                )
            })?)
            .publish_subscribe::<T>()
            .user_header::<TraceContext>();
        if let Some(size) = history {
            builder = builder.history_size(size);
        }
        if let Some(max) =
            service_max_buffer_size.or_else(|| (buffer_size > 1).then_some(buffer_size))
        {
            builder = builder.subscriber_max_buffer_size(max);
        }
        if let Some(max) = max_publishers {
            builder = builder.max_publishers(max);
        }
        let service = builder.open_or_create().map_err(|e| {
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

impl<T: Debug + ZeroCopySend + 'static> Subscriber<[T]> {
    /// 订阅单个原子切片样本，保留统一 trace 用户头。
    /// # Errors
    /// 服务或订阅端创建失败。
    pub fn with_slice_topic(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        let name = topic.try_into().map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                format!("非法话题名: {e:?}"),
            )
        })?;
        let service = node
            .service_builder(&name)
            .publish_subscribe::<[T]>()
            .user_header::<TraceContext>()
            .open_or_create()
            .map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("创建切片服务失败: {e:?}"),
                )
            })?;
        let subscriber = service
            .subscriber_builder()
            .buffer_size(1)
            .create()
            .map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("创建切片订阅器失败: {e:?}"),
                )
            })?;
        Ok(Self { subscriber })
    }

    /// # Errors
    /// 接收 IPC 样本失败。
    pub fn receive(&self) -> Result<Option<Received<[T]>>, firefly_error::Error> {
        self.subscriber.receive().map_err(|e| {
            firefly_error::Error::temporary(
                firefly_error::ErrorKind::Internal,
                format!("接收切片失败: {e:?}"),
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imu::{IMU_SUBSCRIPTION, ImuMessage};
    use crate::publish::Publisher;

    #[test]
    fn imu_capacity_and_queue_work_in_either_startup_order() {
        let node = crate::node::create_node().unwrap();
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        for subscriber_first in [true, false] {
            let topic = format!("Firefly/Test/ImuCapacity/{unique}/{subscriber_first}");
            let publish = || {
                Publisher::<ImuMessage>::with_topic_and_buffer(&node, &topic, Some(128)).unwrap()
            };
            let subscribe =
                || Subscriber::<ImuMessage>::with_options(&node, &topic, IMU_SUBSCRIPTION).unwrap();
            let (publisher, subscriber) = if subscriber_first {
                let subscriber = subscribe();
                (publish(), subscriber)
            } else {
                let publisher = publish();
                (publisher, subscribe())
            };
            assert_eq!(subscriber.subscriber.buffer_size(), 100);
            let full_capacity =
                Subscriber::<ImuMessage>::with_topic_and_buffer(&node, &topic, 128).unwrap();
            assert_eq!(full_capacity.subscriber.buffer_size(), 128);
            for sequence in 0..100 {
                publisher
                    .publish(ImuMessage {
                        timestamp: f64::from(sequence),
                        ..Default::default()
                    })
                    .unwrap();
            }
            for sequence in 0..100 {
                let sample = subscriber.receive().unwrap().expect("完整 IMU 突发");
                assert!((sample.timestamp - f64::from(sequence)).abs() < f64::EPSILON);
            }
            assert!(subscriber.receive().unwrap().is_none());
        }
    }

    #[test]
    fn invalid_queue_limits_are_rejected() {
        let node = crate::node::create_node().unwrap();
        for (buffer_size, service_max_buffer_size) in [(0, None), (2, Some(1))] {
            let result = Subscriber::<ImuMessage>::with_options(
                &node,
                "Firefly/Test/InvalidQueueLimits",
                SubscriberOptions {
                    buffer_size,
                    service_max_buffer_size,
                    ..Default::default()
                },
            );
            assert!(
                matches!(result, Err(error) if error.kind() == firefly_error::ErrorKind::InvalidArgument)
            );
        }
    }
}
