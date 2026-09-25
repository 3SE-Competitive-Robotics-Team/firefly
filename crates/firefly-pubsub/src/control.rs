//! 飞控指令消息（`Firefly/Control`）：飞控 → 被控对象，1kHz，被控对象取最新样本。
//!
//! 指令是**4 个电机的推力**（N），不是世界系 wrench：被控对象按机体几何把
//! 4 个推力合成刚体等效 wrench（`firefly-flight::Airframe::realize` 的同一套式子）。
//! 这样电机的饱和/旋向/几何只在分配处（飞控侧）与合成处（被控对象侧）各出现一次，
//! 且换真机只把这里换成归一化电机指令 + 推力曲线。
//!
//! `state_time` 是飞控计算本指令所用的状态采样时刻：被控对象据此判断指令是否陈旧
//! （自己另给最新样本的年龄），`tick` 供连续性/丢包自检。

use iceoryx2::prelude::*;

use crate::node::IpcNode;
use crate::publish::Publisher;
use crate::subscriber::{Received, Subscriber};
use crate::trace::TraceContext;

/// 飞控指令话题（飞控 → 被控对象）。
pub const CONTROL_TOPIC: &str = "Firefly/Control";

/// 指令订阅缓冲区深度：32 条（1kHz 发布、被控对象 200Hz 排空——留 ~32ms 抖动余量）。
const CONTROL_BUFFER_SIZE: usize = 32;
/// 指令服务订阅端上限（先创建方定上限，Python 侧同值）。
const CONTROL_SERVICE_MAX: usize = 32;

/// 飞控指令：4 电机推力 + 所用状态时刻 + tick 计数。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyControlMessage")]
pub struct ControlMessage {
    /// 飞控本次控制所用的状态采样时刻（仿真秒；被控对象据此判陈旧）。
    pub state_time: f64,
    /// 4 电机推力（N，顺序与机体旋翼编号一致；已由飞控侧限幅）。
    pub thrust: [f64; 4],
    /// 飞控 tick 计数（自增；连续性/丢包自检）。
    pub tick: u64,
}

impl Default for ControlMessage {
    fn default() -> Self {
        Self {
            state_time: -1.0,
            thrust: [0.0; 4],
            tick: 0,
        }
    }
}

/// 收到的指令样本。
pub type ReceivedControl = Received<ControlMessage>;

/// 飞控指令发布器（话题 `Firefly/Control`）。
pub struct ControlPublisher(Publisher<ControlMessage>);

/// 飞控指令订阅器（话题 `Firefly/Control`）。
pub struct ControlSubscriber(Subscriber<ControlMessage>);

impl ControlPublisher {
    /// 打开指令话题的发布器。
    ///
    /// # Errors
    /// 见 [`Publisher::with_topic_and_buffer`]。
    pub fn new(node: &IpcNode) -> Result<Self, firefly_error::Error> {
        Self::with_topic(node, CONTROL_TOPIC)
    }

    /// 以自定义话题名打开发布器（声明订阅端缓冲上限，供 Python 侧同值创建）。
    ///
    /// # Errors
    /// 见 [`Publisher::with_topic_and_buffer`]。
    pub fn with_topic(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        Ok(Self(Publisher::with_topic_and_buffer(
            node,
            topic,
            Some(CONTROL_SERVICE_MAX),
        )?))
    }

    /// 发布一条指令（trace 上下文自动注入，见 [`Publisher::publish`]）。
    ///
    /// # Errors
    /// 见 [`Publisher::publish`]。
    pub fn publish(&self, msg: ControlMessage) -> Result<TraceContext, firefly_error::Error> {
        self.0.publish(msg)
    }
}

impl ControlSubscriber {
    /// 打开指令话题的订阅器。
    ///
    /// # Errors
    /// 见 [`Subscriber::with_topic_and_buffer`]。
    pub fn new(node: &IpcNode) -> Result<Self, firefly_error::Error> {
        Self::with_topic(node, CONTROL_TOPIC)
    }

    /// 以自定义话题名打开订阅器（服务上限 [`CONTROL_SERVICE_MAX`]）。
    ///
    /// # Errors
    /// 见 [`Subscriber::with_topic_and_buffer`]。
    pub fn with_topic(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        let name: iceoryx2::service::service_name::ServiceName = topic.try_into().map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                format!("非法话题名 `{topic}`: {e:?}"),
            )
        })?;
        let service = node
            .service_builder(&name)
            .publish_subscribe::<ControlMessage>()
            .user_header::<TraceContext>()
            .subscriber_max_buffer_size(CONTROL_SERVICE_MAX)
            .open_or_create()
            .map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("打开/创建话题 `{topic}` 失败: {e:?}"),
                )
            })?;
        let subscriber = service
            .subscriber_builder()
            .buffer_size(CONTROL_BUFFER_SIZE)
            .create()
            .map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("创建订阅器失败: {e:?}"),
                )
            })?;
        Ok(Self(Subscriber::from_inner(subscriber)))
    }

    /// 接收一条指令（见 [`Subscriber::receive`]）。
    ///
    /// # Errors
    /// 见 [`Subscriber::receive`]。
    pub fn receive(&self) -> Result<Option<ReceivedControl>, firefly_error::Error> {
        self.0.receive()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_message_is_plain_old_data() {
        let m = ControlMessage::default();
        // 8（state_time）+ 32（4 电机推力）+ 8（tick）
        assert_eq!(std::mem::size_of::<ControlMessage>(), 48);
        assert!(m.thrust.iter().all(|t| t.abs() < 1e-12), "{:?}", m.thrust);
        assert!((m.state_time + 1.0).abs() < 1e-9);
    }

    /// 订阅缓冲不变量（编译期）：≤ 服务上限。
    const _: () = {
        assert!(CONTROL_BUFFER_SIZE <= CONTROL_SERVICE_MAX);
    };
}
