//! 泛型发布端：注入 trace 上下文，并按需发送事件唤醒。

use std::fmt::Debug;

use fastrace::prelude::*;
use iceoryx2::port::publisher::Publisher as Iox2Publisher;
use iceoryx2::prelude::*;
use iceoryx2::sample_mut::SampleMut;

use crate::event::TopicNotifier;
use crate::node::IpcNode;
use crate::trace::TraceContext;

/// 使用调用方的共享节点创建端口；线程与退出约定见 [`crate::node`]。
pub struct Publisher<T: Debug + IceoryxSend + ?Sized + 'static> {
    publisher: Iox2Publisher<ipc::Service, T, TraceContext>,
    /// 事件唤醒端：`Some` 时 publish 成功后自动通知（订阅端 `WaitSet` 即到即醒）。
    notifier: Option<TopicNotifier>,
}

impl<T: Debug + ZeroCopySend + 'static> Publisher<T> {
    /// # Errors
    /// Service/Publisher 创建失败（IPC 资源不可用等）。
    pub fn with_topic(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        Self::with_topic_and_buffer(node, topic, None)
    }

    /// 使用共享节点创建发布器，按需指定订阅缓冲上限；`None` 沿用 iceoryx2 默认值。
    /// 同话题端口的容量须与首次创建的服务兼容；多发布者使用
    /// [`Self::with_topic_buffer_publishers`]。
    ///
    /// # Errors
    /// Service/Publisher 创建失败（IPC 资源不可用等）。
    pub fn with_topic_and_buffer(
        node: &IpcNode,
        topic: &str,
        subscriber_max_buffer_size: Option<usize>,
    ) -> Result<Self, firefly_error::Error> {
        Self::with_topic_buffer_publishers(node, topic, subscriber_max_buffer_size, None)
    }

    /// 创建发布器并指定订阅缓冲与发布者上限；`None` 沿用 iceoryx2 默认值。
    /// 已有服务须支持所请求的容量；日志话题容量见 [`crate::log::LOG_MAX_PUBLISHERS`]。
    ///
    /// # Errors
    /// Service/Publisher 创建失败（IPC 资源不可用等）。
    pub fn with_topic_buffer_publishers(
        node: &IpcNode,
        topic: &str,
        subscriber_max_buffer_size: Option<usize>,
        max_publishers: Option<usize>,
    ) -> Result<Self, firefly_error::Error> {
        Self::open(
            node,
            topic,
            subscriber_max_buffer_size,
            max_publishers,
            None,
        )
    }

    /// Publish a static snapshot that late subscribers can request.
    /// # Errors
    /// Returns service or publisher creation errors.
    pub fn retained(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        Self::open(node, topic, Some(1), Some(1), Some(1))
    }

    /// Deliver retained history to newly connected subscribers while the source is idle.
    /// # Errors
    /// Returns an error when a connection cannot be established.
    pub fn update_connections(&self) -> Result<(), firefly_error::Error> {
        use iceoryx2::port::update_connections::UpdateConnections;
        self.publisher.update_connections().map_err(|error| {
            firefly_error::Error::new(firefly_error::ErrorKind::Internal, error.to_string())
        })
    }

    fn open(
        node: &IpcNode,
        topic: &str,
        subscriber_max_buffer_size: Option<usize>,
        max_publishers: Option<usize>,
        history: Option<usize>,
    ) -> Result<Self, firefly_error::Error> {
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
        if let Some(size) = subscriber_max_buffer_size {
            builder = builder.subscriber_max_buffer_size(size);
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
        let publisher = service.publisher_builder().create().map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::Internal,
                format!("创建 `{topic}` 发布器失败: {e:?}"),
            )
        })?;
        Ok(Self {
            publisher,
            notifier: None,
        })
    }

    /// 以自定义话题名创建**带事件唤醒**的发布器：publish 成功后自动通知
    /// 同名 event service（订阅端把 [`crate::event::TopicListener`]
    /// 挂进 `WaitSet` 即到即醒；无监听者时通知静默丢弃）。
    ///
    /// # Errors
    /// 见 [`Publisher::with_topic`] 与 [`TopicNotifier::with_topic`]。
    pub fn with_topic_notify(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        let notifier = TopicNotifier::with_topic(node, topic)?;
        let mut this = Self::with_topic(node, topic)?;
        this.notifier = Some(notifier);
        Ok(this)
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
        self.publish_counted(msg).map(|(_, ctx)| ctx)
    }

    /// 返回本次收到样本的订阅端数；无人订阅时发送成功，计数为零。
    ///
    /// # Errors
    /// 借出样本失败或发送失败（如发布端超时）。
    pub fn publish_counted(&self, msg: T) -> Result<(usize, TraceContext), firefly_error::Error> {
        let sample = self.publisher.loan_uninit().map_err(|e| {
            firefly_error::Error::temporary(
                firefly_error::ErrorKind::ResourceExhausted,
                format!("借出零拷贝样本失败: {e:?}"),
            )
        })?;
        let sample = sample.write_payload(msg);
        self.send_sample(sample)
    }

    /// 借出由 Default 初始化的共享内存样本，就地填充后直接发送。
    /// trace 上下文与事件通知遵循 [`publish`](Self::publish)。
    ///
    /// # Errors
    /// 借出样本失败或发送失败。
    pub fn publish_with(
        &self,
        fill: impl FnOnce(&mut T),
    ) -> Result<TraceContext, firefly_error::Error>
    where
        T: Default,
    {
        let mut sample = self.publisher.loan().map_err(|e| {
            firefly_error::Error::temporary(
                firefly_error::ErrorKind::ResourceExhausted,
                format!("借出零拷贝样本失败: {e:?}"),
            )
        })?;
        fill(sample.payload_mut());
        self.send_sample(sample).map(|(_, ctx)| ctx)
    }
}

impl<T: Debug + IceoryxSend + ?Sized + 'static> Publisher<T> {
    fn send_sample(
        &self,
        mut sample: SampleMut<ipc::Service, T, TraceContext>,
    ) -> Result<(usize, TraceContext), firefly_error::Error> {
        let ctx = match SpanContext::current_local_parent() {
            Some(sc) => TraceContext::from_span_context(sc),
            None => TraceContext::timestamps_only(),
        };
        *sample.user_header_mut() = ctx;
        sample
            .send()
            .map(|receivers| {
                // 数据路径优先：唤醒失败不阻断发布（见 TopicNotifier::notify_sent_sample）
                if let Some(notifier) = &self.notifier {
                    notifier.notify_sent_sample();
                }
                (receivers, ctx)
            })
            .map_err(|e| {
                firefly_error::Error::temporary(
                    firefly_error::ErrorKind::ResourceExhausted,
                    format!("发送样本失败: {e:?}"),
                )
            })
    }
}

impl<T: Debug + ZeroCopySend + 'static> Publisher<[T]> {
    /// 创建可增长的切片发布器，完整切片以单个 IPC 样本原子发送。
    /// # Errors
    /// 服务或发布器创建失败。
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
        let publisher = service
            .publisher_builder()
            .allocation_strategy(AllocationStrategy::PowerOfTwo)
            .create()
            .map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("创建切片发布器失败: {e:?}"),
                )
            })?;
        Ok(Self {
            publisher,
            notifier: None,
        })
    }

    /// 就地填写变长切片，沿用统一 trace 用户头。
    /// # Errors
    /// 借出共享内存或发送失败。
    pub fn publish_slice_with(
        &self,
        len: usize,
        fill: impl FnMut(usize) -> T,
    ) -> Result<TraceContext, firefly_error::Error> {
        let sample = self
            .publisher
            .loan_slice_uninit(len)
            .map_err(|e| {
                firefly_error::Error::temporary(
                    firefly_error::ErrorKind::ResourceExhausted,
                    format!("借出切片失败: {e:?}"),
                )
            })?
            .write_from_fn(fill);
        self.send_sample(sample).map(|(_, trace)| trace)
    }
}
