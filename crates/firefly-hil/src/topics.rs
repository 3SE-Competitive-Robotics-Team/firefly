use crate::protocol::{Kind, Message};
use firefly_pubsub::{
    activity::ActivityMessage,
    attitude::AttitudeMessage,
    camera::{DownImageMessage, GrayImageMessage},
    command::CommandMessage,
    event::TopicNotifier,
    fc_status::{FC_STATUS_BUFFER_SIZE, FC_STATUS_TOPIC, FcStatusMessage},
    goal::GoalMessage,
    imu::ImuMessage,
    landing::LandingTargetMessage,
    log::{LOG_BUFFER_SIZE, LOG_MAX_PUBLISHERS, LogMessage},
    navigation::NavigationMessage,
    node::IpcNode,
    odom::{OdomMessage, VIO_PREDICTION_BUFFER_SIZE, VIO_PREDICTION_TOPIC, VioPrediction},
    plant::AirframeMessage,
    publish::Publisher,
    reference::ReferenceMessage,
    request::{MissionRequest, REQUEST_MAX_PUBLISHERS},
    sticks::{STICKS_BUFFER_SIZE, SticksMessage},
    subscriber::{Subscriber, SubscriberOptions},
    trajectory::PlannerStatusMessage,
    viz::{
        SCALAR_BUFFER_SIZE, ScalarMessage, VIZ_BUFFER_SIZE, VIZ_MAX_PUBLISHERS, VOXEL_BUFFER_SIZE,
        VizMessage, VoxelMessage,
    },
};
use std::{cell::RefCell, collections::BTreeMap, io};

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
        Message::VioPrediction(value) => Some(&mut value.sequence),
        Message::FcStatus(value) => Some(&mut value.sequence),
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
            $($field: Subscriber<$payload>,)+
            sequences: RefCell<Sequences>,
        }
        impl $sub {
            /// # Errors
            /// IPC 服务容量或端口创建失败。
            pub fn open(node: &IpcNode, prefix: &str) -> io::Result<Self> {
                Ok(Self { $($field: Subscriber::with_options(node, &topic(prefix, $topic), SubscriberOptions {
                    buffer_size: $capacity, service_max_buffer_size: Some($capacity), max_publishers: Some($publishers),
                }).map_err(io::Error::other)?,)+ sequences: RefCell::default() })
            }
            /// 排空已登记流；only 指定单个消息种类，None 处理全部，sink 错误结束会话。
            /// # Errors
            /// IPC、连续性或 sink 写入错误。
            pub fn drain(&self, only: Option<Kind>, mut sink: impl FnMut(Message) -> io::Result<()>) -> io::Result<bool> {
                let mut any = false;
                $(if only.is_none_or(|kind| kind == Kind::$variant) {
                    while let Some(sample) = self.$field.receive().map_err(io::Error::other)? {
                        let mut message = Message::$variant((*sample).into());
                        self.sequences.borrow_mut().observe(&mut message, sample.header().publisher_id().value(), $publishers)?;
                        sink(message)?; any = true;
                    }
                })+
                Ok(any)
            }
        }
        pub struct $pub {
            $($field: Publisher<$payload>,)+
            sequences: RefCell<Sequences>,
        }
        impl $pub {
            /// # Errors
            /// IPC 服务容量或端口创建失败。
            pub fn open(node: &IpcNode, prefix: &str) -> io::Result<Self> {
                Ok(Self { $($field: Publisher::with_topic_buffer_publishers(node, &topic(prefix, $topic),
                    Some($capacity), Some($publishers)).map_err(io::Error::other)?,)+ sequences: RefCell::default() })
            }
            /// 发布到登记的本机话题；诊断流为聚合发布者分配连续序号。
            /// # Errors
            /// 消息方向未登记或 IPC 发布失败。
            pub fn publish(&self, mut message: Message) -> io::Result<()> {
                self.sequences.borrow_mut().normalize(&mut message);
                match message {
                    $(Message::$variant(value) => {
                        let payload: &$payload = std::borrow::Borrow::borrow(&value);
                        self.$field.publish(*payload).map_err(io::Error::other)?;
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
    command: CommandMessage => Command (CommandMessage::TOPIC, CommandMessage::BUFFER_SIZE, REQUEST_MAX_PUBLISHERS),
    goal: GoalMessage => Goal (GoalMessage::TOPIC, GoalMessage::BUFFER_SIZE, REQUEST_MAX_PUBLISHERS),
    airframe: AirframeMessage => Airframe (firefly_pubsub::plant::AIRFRAME_TOPIC, 4, 2),
}

routes! { TelemetrySubscribers, TelemetryPublishers;
    fc_status: FcStatusMessage => FcStatus (FC_STATUS_TOPIC, FC_STATUS_BUFFER_SIZE, 1),
    prediction: VioPrediction => VioPrediction (VIO_PREDICTION_TOPIC, VIO_PREDICTION_BUFFER_SIZE, 1),
    sticks: SticksMessage => Sticks (firefly_pubsub::sticks::STICKS_TOPIC, STICKS_BUFFER_SIZE, 2),
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
            .map_err(io::Error::other)?,
            left: Subscriber::with_options(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_LEFT_TOPIC),
                options(2, 2),
            )
            .map_err(io::Error::other)?,
            right: Subscriber::with_options(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_RIGHT_TOPIC),
                options(2, 2),
            )
            .map_err(io::Error::other)?,
            down: Subscriber::with_options(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_DOWN_TOPIC),
                options(2, 2),
            )
            .map_err(io::Error::other)?,
        })
    }
}

pub struct SensorPublishers {
    pub requests: RequestsPublishers,
    pub imu: Publisher<ImuMessage>,
    pub imu_notify: TopicNotifier,
    pub left: Publisher<GrayImageMessage>,
    pub right: Publisher<GrayImageMessage>,
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
            .map_err(io::Error::other)?,
            imu_notify: TopicNotifier::with_topic(
                node,
                &topic(prefix, firefly_pubsub::imu::IMU_TOPIC),
            )
            .map_err(io::Error::other)?,
            left: Publisher::with_topic_and_buffer(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_LEFT_TOPIC),
                Some(2),
            )
            .map_err(io::Error::other)?,
            right: Publisher::with_topic_and_buffer(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_RIGHT_TOPIC),
                Some(2),
            )
            .map_err(io::Error::other)?,
            down: Publisher::with_topic_and_buffer(
                node,
                &topic(prefix, firefly_pubsub::camera::CAMERA_DOWN_TOPIC),
                Some(2),
            )
            .map_err(io::Error::other)?,
        })
    }
    /// 整批发布后只唤醒一次，保留 128 槽的消费者契约。
    /// # Errors
    /// IPC 发布失败。
    pub fn publish_imu(&self, samples: &[ImuMessage]) -> io::Result<usize> {
        for sample in samples {
            self.imu.publish(*sample).map_err(io::Error::other)?;
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
