use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use firefly_error::Result;
use firefly_pubsub::{
    activity::{ACTIVITY_TOPIC, ActivityMessage, LANDING, MAX_OWNERS},
    attitude::{ATTITUDE_TOPIC, AttitudeMessage},
    command::{COMMAND_TOPIC, CommandMessage},
    fc_status::{FC_STATUS_BUFFER_SIZE, FC_STATUS_TOPIC, FcStatusMessage},
    goal::{GOAL_TOPIC, GoalMessage},
    landing::{LANDING_TARGET_TOPIC, LandingTargetMessage},
    node::create_node,
    odom::{ODOM_TOPIC, OdomMessage},
    plant::{AIRFRAME_BUFFER_SIZE, AIRFRAME_TOPIC, AirframeMessage},
    publish::Publisher,
    reference::{REFERENCE_TOPIC, ReferenceMessage},
    subscriber::{Subscriber, SubscriberOptions},
    trajectory::{
        PLANNER_HEARTBEAT_TOPIC, PLANNER_STATUS_TOPIC, PlannerHeartbeatMessage,
        PlannerStatusMessage,
    },
};
use iceoryx2::prelude::ZeroCopySend;

pub fn nanos() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

pub struct Input<T: std::fmt::Debug + ZeroCopySend + 'static> {
    port: Subscriber<T>,
    sample: Option<(T, Instant)>,
}

impl<T: Copy + std::fmt::Debug + ZeroCopySend + 'static> Input<T> {
    fn new(port: Subscriber<T>) -> Self {
        Self { port, sample: None }
    }
    fn pump(&mut self, stamp: impl Fn(T) -> f64) -> Result<()> {
        while let Some(value) = self.port.receive()? {
            if self
                .sample
                .is_none_or(|(old, _)| stamp(*value) != stamp(old))
            {
                self.sample = Some((*value, Instant::now()));
            }
        }
        Ok(())
    }
    pub fn fresh(&self) -> Option<T> {
        self.within(Duration::from_secs(1))
    }
    pub fn within(&self, age: Duration) -> Option<T> {
        self.sample
            .filter(|(_, t)| t.elapsed() < age)
            .map(|(v, _)| v)
    }
}

pub struct Bus {
    pub airframe: Input<AirframeMessage>,
    pub heartbeat: Input<PlannerHeartbeatMessage>,
    pub log: firefly_observability::LogIpc,
    pub odom: Input<OdomMessage>,
    pub attitude: Input<AttitudeMessage>,
    pub fc: Input<FcStatusMessage>,
    pub planner: Input<PlannerStatusMessage>,
    pub reference: Input<ReferenceMessage>,
    pub target: Input<LandingTargetMessage>,
    command: Publisher<CommandMessage>,
    pub goal: Publisher<GoalMessage>,
    activity: Publisher<ActivityMessage>,
    lease_owner: u64,
    lease_sent: Option<Instant>,
}
impl Bus {
    pub fn open() -> Result<Self> {
        let node = create_node()?;
        Ok(Self {
            airframe: Input::new(Subscriber::with_topic_and_buffer(
                &node,
                AIRFRAME_TOPIC,
                AIRFRAME_BUFFER_SIZE,
            )?),
            heartbeat: Input::new(Subscriber::with_topic(&node, PLANNER_HEARTBEAT_TOPIC)?),
            log: firefly_observability::init_ipc(&node, "flight"),
            odom: Input::new(Subscriber::with_topic_and_buffer(&node, ODOM_TOPIC, 2)?),
            attitude: Input::new(Subscriber::with_topic_and_buffer(&node, ATTITUDE_TOPIC, 2)?),
            fc: Input::new(Subscriber::with_options(
                &node,
                FC_STATUS_TOPIC,
                SubscriberOptions {
                    buffer_size: 1,
                    service_max_buffer_size: Some(FC_STATUS_BUFFER_SIZE),
                    max_publishers: Some(1),
                },
            )?),
            planner: Input::new(Subscriber::with_topic_and_buffer(
                &node,
                PLANNER_STATUS_TOPIC,
                2,
            )?),
            reference: Input::new(Subscriber::with_topic_and_buffer(
                &node,
                REFERENCE_TOPIC,
                2,
            )?),
            target: Input::new(Subscriber::with_topic_and_buffer(
                &node,
                LANDING_TARGET_TOPIC,
                2,
            )?),
            command: Publisher::with_topic(&node, COMMAND_TOPIC)?,
            goal: Publisher::with_topic(&node, GOAL_TOPIC)?,
            activity: Publisher::with_topic_buffer_publishers(
                &node,
                ACTIVITY_TOPIC,
                Some(8),
                Some(MAX_OWNERS),
            )?,
            lease_owner: nanos(),
            lease_sent: None,
        })
    }
    pub fn pump(&mut self) -> Result<()> {
        self.airframe.pump(|v| v.timestamp)?;
        self.heartbeat.pump(|v| v.sequence as f64)?;
        self.fc.pump(|v| v.timestamp)?;
        self.attitude.pump(|v| v.timestamp)?;
        self.odom.pump(|v| v.timestamp)?;
        self.planner.pump(|v| v.timestamp)?;
        self.reference.pump(|v| v.timestamp)?;
        self.target.pump(|v| v.timestamp)
    }
    pub fn navigation(&self) -> Option<(OdomMessage, u64, firefly_flight::FlightState)> {
        let pose = self.odom.fresh()?;
        let attitude = self.attitude.fresh()?;
        let fc = self.fc.fresh()?;
        firefly_odometry::Snapshot::from_message(&pose)?;
        (attitude.session != 0
            && (attitude.timestamp - pose.timestamp).abs() < 0.5
            && (fc.timestamp - pose.timestamp).abs() < 0.5)
            .then_some((
                pose,
                attitude.session,
                firefly_flight::FlightState::from_code(fc.mode)?,
            ))
    }
    pub fn command(&self, kind: u8, altitude: f64, sequence: u64) -> Result<()> {
        self.command
            .publish(CommandMessage {
                kind,
                altitude,
                sequence,
                timestamp: sequence as f64 / 1e9,
            })
            .map(|_| ())
    }
    pub fn landing_active(&self) -> bool {
        self.lease_sent.is_some()
    }
    pub fn lease(&mut self, enabled: bool) -> Result<()> {
        if enabled
            && self
                .lease_sent
                .is_none_or(|t| t.elapsed() >= Duration::from_millis(200))
            || !enabled && self.lease_sent.is_some()
        {
            self.activity.publish(ActivityMessage {
                timestamp: self.fc.fresh().map_or(0., |f| f.timestamp),
                owner: self.lease_owner,
                sequence: nanos(),
                mask: if enabled { LANDING } else { 0 },
                lease_ms: if enabled { 1000 } else { 0 },
            })?;
            self.lease_sent = enabled.then(Instant::now);
        }
        Ok(())
    }
}

pub fn speed(p: OdomMessage) -> f64 {
    p.velocity_x.hypot(p.velocity_y).hypot(p.velocity_z)
}
