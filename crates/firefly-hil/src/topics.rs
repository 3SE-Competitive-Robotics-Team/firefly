//! HIL 唯一 IPC 路由登记：两端共享类型、话题、队列与连续性策略。
//!
//! 新业务消息在协议中定义后，只在 routes! 登记一次；主机和板端通过
//! open / drain / publish 接入。图像与 IMU 保留各自的分片和批次调度。
use crate::protocol::{Kind, Message};
use firefly_pubsub::{
    activity::ActivityMessage,
    attitude::AttitudeMessage,
    camera::{DepthImageMessage, DownImageMessage, GrayImageMessage},
    command::CommandMessage,
    control::{CONTROL_BUFFER_SIZE, ControlMessage},
    event::TopicNotifier,
    goal::GoalMessage,
    imu::ImuMessage,
    landing::LandingTargetMessage,
    log::{LOG_BUFFER_SIZE, LOG_MAX_PUBLISHERS, LogMessage},
    navigation::NavigationMessage,
    node::IpcNode,
    odom::OdomMessage,
    plant::AirframeMessage,
    publish::Publisher,
    reference::ReferenceMessage,
    subscriber::{Subscriber, SubscriberOptions},
    trajectory::PlannerStatusMessage,
    viz::{
        SCALAR_BUFFER_SIZE, ScalarMessage, VIZ_BUFFER_SIZE, VIZ_MAX_PUBLISHERS, VOXEL_BUFFER_SIZE,
        VizMessage, VoxelMessage,
    },
};
use std::{cell::RefCell, collections::BTreeMap, io};

pub(crate) fn error(error: impl std::fmt::Display) -> io::Error {
    io::Error::other(error.to_string())
}
pub(crate) fn topic(prefix: &str, name: &str) -> String {
    format!("{prefix}/{}", name.strip_prefix("Firefly/").unwrap_or(name))
}

#[derive(Default)]
struct Sequences {
    sources: BTreeMap<(u16, u128), u64>,
    outgoing: BTreeMap<u16, u64>,
}

fn sequence(message: &mut Message) -> Option<&mut u64> {
    match message {
        Message::Scalar(value) => Some(&mut value.sequence),
        Message::Log(value) => Some(&mut value.sequence),
        Message::Viz(value) => Some(&mut value.sequence),
        Message::Voxels(value) => Some(&mut value.sequence),
        _ => None,
    }
}

impl Sequences {
    fn observe(
        &mut self,
        message: &mut Message,
        publisher: u128,
        maximum: usize,
    ) -> io::Result<()> {
        let kind = message.kind().id();
        let Some(sequence) = sequence(message) else {
            return Ok(());
        };
        let key = (kind, publisher);
        if let Some(previous) = self.sources.get(&key) {
            if *sequence != previous.wrapping_add(1) {
                return Err(io::Error::other(format!(
                    "HIL diagnostic sequence gap: {previous} -> {sequence}"
                )));
            }
        } else if self.sources.keys().filter(|(k, _)| *k == kind).count() >= maximum {
            return Err(io::Error::other("HIL diagnostic publisher budget exceeded"));
        }
        self.sources.insert(key, *sequence);
        Ok(())
    }
    fn normalize(&mut self, message: &mut Message) {
        let kind = message.kind().id();
        if let Some(sequence) = sequence(message) {
            let next = self.outgoing.entry(kind).or_default();
            *sequence = *next;
            *next = next.wrapping_add(1);
        }
    }
}

macro_rules! routes {
    ($sub:ident, $pub:ident; $( $field:ident: $payload:ty => $variant:ident ($topic:expr, $capacity:expr, $publishers:expr) ),+ $(,)?) => {
        pub struct $sub {
            $(pub $field: Subscriber<$payload>,)+
            sequences: RefCell<Sequences>,
        }
        impl $sub {
            /// # Errors
            /// IPC 服务容量或端口创建失败。
            pub fn open(node: &IpcNode, prefix: &str) -> io::Result<Self> {
                Ok(Self { $($field: Subscriber::with_options(node, &topic(prefix, $topic), SubscriberOptions {
                    buffer_size: $capacity, service_max_buffer_size: Some($capacity), max_publishers: Some($publishers),
                }).map_err(error)?,)+ sequences: RefCell::default() })
            }
            /// 排空已登记流；except 供调度器独占高优先消息，sink 错误结束会话。
            /// # Errors
            /// IPC、连续性或 sink 写入错误。
            pub fn drain(&self, except: Option<Kind>, mut sink: impl FnMut(Message) -> io::Result<()>) -> io::Result<bool> {
                let mut any = false;
                $(if except != Some(Kind::$variant) {
                    while let Some(sample) = self.$field.receive().map_err(error)? {
                        let mut message = Message::$variant((*sample).into());
                        self.sequences.borrow_mut().observe(&mut message, sample.header().publisher_id().value(), $publishers)?;
                        sink(message)?; any = true;
                    }
                })+
                Ok(any)
            }
        }
        pub struct $pub {
            $(pub $field: Publisher<$payload>,)+
            sequences: RefCell<Sequences>,
        }
        impl $pub {
            /// # Errors
            /// IPC 服务容量或端口创建失败。
            pub fn open(node: &IpcNode, prefix: &str) -> io::Result<Self> {
                Ok(Self { $($field: Publisher::with_topic_buffer_publishers(node, &topic(prefix, $topic),
                    Some($capacity), Some($publishers)).map_err(error)?,)+ sequences: RefCell::default() })
            }
            /// 发布到登记的本机话题；诊断流为聚合发布者分配连续序号。
            /// # Errors
            /// 消息方向未登记或 IPC 发布失败。
            pub fn publish(&self, mut message: Message) -> io::Result<()> {
                self.sequences.borrow_mut().normalize(&mut message);
                match message {
                    $(Message::$variant(value) => {
                        let payload: &$payload = std::borrow::Borrow::borrow(&value);
                        self.$field.publish(*payload).map_err(error)?;
                    },)+
                    other => return Err(io::Error::other(format!("unregistered HIL IPC route: {}", other.kind().name()))),
                }
                Ok(())
            }
        }
    }
}

routes! { RequestsSubscribers, RequestsPublishers;
    activity: ActivityMessage => Activity (firefly_pubsub::activity::ACTIVITY_TOPIC, 8, firefly_pubsub::activity::MAX_OWNERS),
    command: CommandMessage => Command (firefly_pubsub::command::COMMAND_TOPIC, 8, 2),
    goal: GoalMessage => Goal (firefly_pubsub::goal::GOAL_TOPIC, 4, 2),
    airframe: AirframeMessage => Airframe (firefly_pubsub::plant::AIRFRAME_TOPIC, 4, 2),
}

routes! { TelemetrySubscribers, TelemetryPublishers;
    control: ControlMessage => Control (firefly_pubsub::control::CONTROL_TOPIC, CONTROL_BUFFER_SIZE, 2),
    odom: OdomMessage => Odometry (firefly_pubsub::odom::ODOM_TOPIC, 2, 2),
    corrected: OdomMessage => CorrectedOdometry (firefly_pubsub::odom::CORRECTED_ODOM_TOPIC, 2, 2),
    attitude: AttitudeMessage => VioAttitude (firefly_pubsub::attitude::ATTITUDE_TOPIC, 2, 2),
    reference: ReferenceMessage => Reference (firefly_pubsub::reference::REFERENCE_TOPIC, 2, 2),
    planner_status: PlannerStatusMessage => PlannerStatus (firefly_pubsub::trajectory::PLANNER_STATUS_TOPIC, 2, 2),
    navigation: NavigationMessage => Navigation (firefly_pubsub::navigation::NAVIGATION_TOPIC, 2, 2),
    landing: LandingTargetMessage => LandingTarget (firefly_pubsub::landing::LANDING_TARGET_TOPIC, 2, 2),
    geometry: VizMessage => Viz (firefly_pubsub::viz::VIZ_TOPIC, VIZ_BUFFER_SIZE, VIZ_MAX_PUBLISHERS),
    voxels: VoxelMessage => Voxels (firefly_pubsub::viz::VOXEL_TOPIC, VOXEL_BUFFER_SIZE, 1),
    scalar: ScalarMessage => Scalar (firefly_pubsub::viz::SCALAR_TOPIC, SCALAR_BUFFER_SIZE, VIZ_MAX_PUBLISHERS),
    log: LogMessage => Log (firefly_pubsub::log::LOG_TOPIC, LOG_BUFFER_SIZE, LOG_MAX_PUBLISHERS),
}

pub struct SensorSubscribers {
    pub requests: RequestsSubscribers,
    pub imu: Subscriber<ImuMessage>,
    pub left: Subscriber<GrayImageMessage>,
    pub right: Subscriber<GrayImageMessage>,
    pub depth: Subscriber<DepthImageMessage>,
    pub down: Subscriber<DownImageMessage>,
}
impl SensorSubscribers {
    /// # Errors
    /// IPC 服务容量或端口创建失败。
    pub fn open(node: &IpcNode, prefix: &str) -> io::Result<Self> {
        let options = |buffer_size, service_max_buffer_size| SubscriberOptions {
            buffer_size,
            service_max_buffer_size: Some(service_max_buffer_size),
            max_publishers: Some(2),
        };
        Ok(Self {
            requests: RequestsSubscribers::open(node, prefix)?,
            imu: Subscriber::with_options(
                node,
                &topic(prefix, firefly_pubsub::imu::IMU_TOPIC),
                options(100, 128),
            )
            .map_err(error)?,
            left: Subscriber::with_options(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_LEFT_TOPIC),
                options(2, 2),
            )
            .map_err(error)?,
            right: Subscriber::with_options(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_RIGHT_TOPIC),
                options(2, 2),
            )
            .map_err(error)?,
            depth: Subscriber::with_options(
                node,
                &topic(prefix, firefly_pubsub::camera::DEPTH_TOPIC),
                options(2, 2),
            )
            .map_err(error)?,
            down: Subscriber::with_options(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_DOWN_TOPIC),
                options(2, 2),
            )
            .map_err(error)?,
        })
    }
}

pub struct SensorPublishers {
    pub requests: RequestsPublishers,
    pub imu: Publisher<ImuMessage>,
    pub imu_notify: TopicNotifier,
    pub left: Publisher<GrayImageMessage>,
    pub right: Publisher<GrayImageMessage>,
    pub depth: Publisher<DepthImageMessage>,
    pub down: Publisher<DownImageMessage>,
}
impl SensorPublishers {
    /// # Errors
    /// IPC 服务容量或端口创建失败。
    pub fn open(node: &IpcNode, prefix: &str) -> io::Result<Self> {
        Ok(Self {
            requests: RequestsPublishers::open(node, prefix)?,
            imu: Publisher::with_topic_and_buffer(
                node,
                &topic(prefix, firefly_pubsub::imu::IMU_TOPIC),
                Some(128),
            )
            .map_err(error)?,
            imu_notify: TopicNotifier::with_topic(
                node,
                &topic(prefix, firefly_pubsub::imu::IMU_TOPIC),
            )
            .map_err(error)?,
            left: Publisher::with_topic_and_buffer(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_LEFT_TOPIC),
                Some(2),
            )
            .map_err(error)?,
            right: Publisher::with_topic_and_buffer(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_RIGHT_TOPIC),
                Some(2),
            )
            .map_err(error)?,
            depth: Publisher::with_topic_and_buffer(
                node,
                &topic(prefix, firefly_pubsub::camera::DEPTH_TOPIC),
                Some(2),
            )
            .map_err(error)?,
            down: Publisher::with_topic_and_buffer(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_DOWN_TOPIC),
                Some(2),
            )
            .map_err(error)?,
        })
    }
    /// 整批发布后只唤醒一次，保留 128 槽的消费者契约。
    /// # Errors
    /// IPC 发布失败。
    pub fn publish_imu(&self, samples: &[ImuMessage]) -> io::Result<usize> {
        for sample in samples {
            self.imu.publish(*sample).map_err(error)?;
        }
        if !samples.is_empty() {
            self.imu_notify.notify_sent_sample();
        }
        Ok(samples.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn diagnostic_sequences_are_per_kind_publisher_and_wrap() {
        let mut sequences = Sequences::default();
        let mut row = ScalarMessage::base(0., "test");
        row.sequence = u64::MAX;
        let mut scalar = Message::Scalar(row);
        sequences.observe(&mut scalar, 1, 2).unwrap();
        *sequence(&mut scalar).unwrap() = 0;
        sequences.observe(&mut scalar, 1, 2).unwrap();
        sequences.observe(&mut scalar, 2, 2).unwrap();
        assert!(sequences.observe(&mut scalar, 1, 2).is_err());
        assert!(sequences.observe(&mut scalar, 3, 2).is_err());
        sequences.normalize(&mut scalar);
        assert_eq!(*sequence(&mut scalar).unwrap(), 0);
        sequences.normalize(&mut scalar);
        assert_eq!(*sequence(&mut scalar).unwrap(), 1);
    }
}
