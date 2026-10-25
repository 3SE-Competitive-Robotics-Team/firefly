//! 泛型订阅端：统一服务容量与接收缓冲配置，保留 trace 用户头。
//!
//! 与 [`crate::publish`] 对称：`NodeBuilder` → `service_builder(...)`
//! `.publish_subscribe::<T>()` → `.user_header::<TraceContext>()` →
//! `subscriber_builder().create()`。订阅端必须声明与发布端相同的
//! User Header 类型（iceoryx2 连接时校验签名兼容）。
//!
//! 收到的样本携带发布端 fastrace 活动 span 的上下文
//! （[`TraceContext::continue_span`] 可续接为跨进程 span 树）与双发送
//! 时间戳（可计算端到端延迟）。

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
    /// 当前订阅端的队列深度。
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

/// 泛型零拷贝订阅器（iceoryx2 ipc 服务，User Header 携带 trace 上下文）。
///
/// 消息须满足 iceoryx2 的 `Debug + ZeroCopySend` 约束。
///
/// 节点由调用方持有（进程共享单节点，见 [`crate::node`]），端口只借用其
/// 创建服务；Drop 顺序由调用方作用域保证（节点最后释放）。
pub struct Subscriber<T: Debug + IceoryxSend + ?Sized + 'static> {
    /// 订阅器句柄。
    subscriber: Iox2Subscriber<ipc::Service, T, TraceContext>,
}

impl<T: Debug + ZeroCopySend + 'static> Subscriber<T> {
    /// 以进程共享节点 + 自定义话题名打开订阅器。
    ///
    /// `buffer_size` 控制 iceoryx2 内部环形缓冲区深度：
    /// - IMU 使用 [`crate::imu::IMU_SUBSCRIPTION`]，服务容量与接收队列分开声明；
    /// - 相机（10 Hz）建议 1，只保留最新帧避免左右目交叉。
    ///
    /// 大于 1 的缓冲同时声明服务所需容量，允许订阅端先创建服务。
    ///
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

    /// 以指定队列深度和服务容量创建订阅器；支持订阅端先创建服务。
    ///
    /// # Errors
    /// 队列深度为零、超过声明的服务上限，或服务/订阅器创建失败。
    pub fn with_options(
        node: &IpcNode,
        topic: &str,
        options: SubscriberOptions,
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
