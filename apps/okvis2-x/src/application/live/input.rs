// SPDX-License-Identifier: BSD-3-Clause
//! Sensor packets are copied into owned buffers before releasing the IPC loan.
use super::{
    config::{Config, invalid},
    output::{self, Output},
};
use crate::runtime::{
    threaded_slam::{PublicationData, SensorInputHandle},
    vi_interface::{ImageMap, TrackingQuality, ViInterface},
};
use firefly_error::Result;
use firefly_pubsub::{
    attitude::{ATTITUDE_TOPIC, AttitudeMessage},
    camera::{
        CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, DEPTH_TOPIC, DepthImageMessage, GrayImageMessage,
        IMAGE_HEIGHT, IMAGE_WIDTH,
    },
    imu::{IMU_SUBSCRIPTION, IMU_TOPIC, ImuMessage},
    navigation::{NAVIGATION_TOPIC, NavigationMessage},
    node::create_node,
    odom::{CORRECTED_ODOM_TOPIC, ODOM_TOPIC, OdomMessage},
    publish::Publisher,
    subscriber::{Subscriber, SubscriberOptions},
    viz::{ScalarMessage, VizMessage, VizPublisher, kind},
};
use jiff::Timestamp;
use nalgebra::Vector3;
use purecv::core::DynamicMatrix;
use std::{
    collections::VecDeque,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc::{Receiver, SyncSender, TrySendError},
    },
    time::Duration,
};

pub(super) struct Depth {
    pub time: Timestamp,
    pub image: Arc<DynamicMatrix>,
}
struct Frame {
    time: Timestamp,
    image: Arc<DynamicMatrix>,
}
pub(super) struct Pairs {
    left: VecDeque<Frame>,
    right: VecDeque<Frame>,
    last: Option<Timestamp>,
    tolerance: f64,
}
impl Pairs {
    pub fn new(tolerance: f64) -> Self {
        Self {
            left: VecDeque::new(),
            right: VecDeque::new(),
            last: None,
            tolerance,
        }
    }
    pub fn push(&mut self, packet: &GrayImageMessage, camera: usize) -> Result<()> {
        validate_image(packet.width, packet.height, packet.sensor_id, camera as i32)?;
        let time = output::time(packet.timestamp)?;
        let queue = if camera == 0 {
            &mut self.left
        } else {
            &mut self.right
        };
        if self.last.is_some_and(|last| time <= last)
            || queue.back().is_some_and(|last| time <= last.time)
        {
            return Ok(());
        }
        let image = Arc::new(
            DynamicMatrix::new_u8(IMAGE_HEIGHT, IMAGE_WIDTH, 1, packet.data.to_vec())
                .map_err(invalid)?,
        );
        queue.push_back(Frame { time, image });
        if queue.len() > 8 {
            queue.pop_front();
            log::warn!("OKVIS stereo pairing queue overrun");
        }
        Ok(())
    }
    pub fn next(
        &mut self,
        imu_time: Option<Timestamp>,
        delay: f64,
    ) -> Option<(Timestamp, ImageMap)> {
        loop {
            let (left, right) = (self.left.front()?, self.right.front()?);
            if (left.time.as_nanosecond() - right.time.as_nanosecond()).unsigned_abs() as f64 * 1e-9
                > self.tolerance
            {
                if left.time < right.time {
                    self.left.pop_front();
                } else {
                    self.right.pop_front();
                }
                continue;
            }
            let time = left.time.max(right.time);
            let covered = imu_time.is_some_and(|imu| {
                imu.as_nanosecond() >= time.as_nanosecond() - (delay * 1e9).round() as i128
            });
            if !covered {
                return None;
            }
            let left = self.left.pop_front()?;
            let right = self.right.pop_front()?;
            self.last = Some(time);
            return Some((time, ImageMap::from([(0, left.image), (1, right.image)])));
        }
    }
}
fn validate_image(width: u32, height: u32, id: i32, expected: i32) -> Result<()> {
    if width as usize != IMAGE_WIDTH || height as usize != IMAGE_HEIGHT || id != expected {
        return Err(invalid(
            "IPC image size or sensor ID does not match calibrated rig",
        ));
    }
    Ok(())
}
fn depth(packet: &DepthImageMessage) -> Result<Depth> {
    validate_image(packet.width, packet.height, packet.sensor_id, 0)?;
    Ok(Depth {
        time: output::time(packet.timestamp)?,
        image: Arc::new(
            DynamicMatrix::new_f32(
                IMAGE_HEIGHT,
                IMAGE_WIDTH,
                1,
                packet
                    .data
                    .iter()
                    .map(|value| {
                        if value.is_finite() && *value > 0.0 {
                            *value
                        } else {
                            0.0
                        }
                    })
                    .collect(),
            )
            .map_err(invalid)?,
        ),
    })
}
pub(super) struct Topics {
    pub imu: String,
    pub left: String,
    pub right: String,
    pub depth: String,
    pub odometry: String,
    pub corrected: String,
    pub navigation: String,
    pub attitude: String,
}
impl Default for Topics {
    fn default() -> Self {
        Self {
            imu: IMU_TOPIC.into(),
            left: CAMERA_LEFT_TOPIC.into(),
            right: CAMERA_RIGHT_TOPIC.into(),
            depth: DEPTH_TOPIC.into(),
            odometry: ODOM_TOPIC.into(),
            corrected: CORRECTED_ODOM_TOPIC.into(),
            navigation: NAVIGATION_TOPIC.into(),
            attitude: ATTITUDE_TOPIC.into(),
        }
    }
}
pub(super) struct Ports {
    pub map_snapshots: super::map_output::Mailbox,
    pub topics: Topics,
    pub config: Config,
    pub sensors: SensorInputHandle,
    pub publications: Receiver<PublicationData>,
    pub depths: SyncSender<Depth>,
    pub wake: SyncSender<()>,
    pub stop: Arc<AtomicBool>,
}
pub(super) fn run(mut ports: Ports) -> Result<()> {
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "firefly-okvis-x");
    let imu = Subscriber::<ImuMessage>::with_options(&node, &ports.topics.imu, IMU_SUBSCRIPTION)?;
    let options = SubscriberOptions {
        buffer_size: 2,
        ..Default::default()
    };
    let left = Subscriber::<GrayImageMessage>::with_options(&node, &ports.topics.left, options)?;
    let right = Subscriber::<GrayImageMessage>::with_options(&node, &ports.topics.right, options)?;
    let depths = ports
        .config
        .mapping
        .then(|| Subscriber::<DepthImageMessage>::with_options(&node, &ports.topics.depth, options))
        .transpose()?;
    let mut publishers = Publishers::open(&node, &ports.config, &ports.topics)?;
    let mut pairs = Pairs::new(ports.config.stereo_tolerance);
    let mut output = Output::new(ports.config.clone(), session())?;
    let mut map_publisher =
        super::map_output::PublisherWorker::start(ports.map_snapshots.clone(), ports.stop.clone())?;
    let mut imu_time = None;
    let mut last_depth = None;

    while !ports.stop.load(Ordering::Acquire) && node.wait(Duration::from_millis(1)).is_ok() {
        firefly_observability::pump_log_ipc(&log_ipc);
        while let Ok(publication) = ports.publications.try_recv() {
            output.posterior(&publication)?;
            map_publisher.update(&output)?;
            publishers.posterior(&publication)?;
        }
        while let Some(packet) = imu.receive()? {
            let trace = *packet.user_header();
            let _span = trace.continue_span("okvis-receive-imu");
            let time = output::time(packet.timestamp)?;
            let alpha = Vector3::new(
                packet.linear_acceleration_x,
                packet.linear_acceleration_y,
                packet.linear_acceleration_z,
            );
            let omega = Vector3::new(
                packet.angular_velocity_x,
                packet.angular_velocity_y,
                packet.angular_velocity_z,
            );
            if alpha
                .iter()
                .chain(omega.iter())
                .any(|value| !value.is_finite())
            {
                return Err(invalid("nonfinite IMU measurement"));
            }
            if imu_time.is_some_and(|last| time <= last) {
                continue;
            }
            firefly_observability::set_sim_time(packet.timestamp);
            imu_time = Some(time);
            if !ports.sensors.add_imu_measurement(time, &alpha, &omega) {
                return Err(invalid("estimator IMU queue rejected measurement"));
            }
            if let Some(sample) = output.imu(time, alpha, omega)? {
                publishers.publish(&sample, packet.timestamp)?;
            }
        }
        for (camera, subscriber) in [(0, &left), (1, &right)] {
            while let Some(packet) = subscriber.receive()? {
                let trace = *packet.user_header();
                let _span = trace.continue_span("okvis-receive-image");
                pairs.push(&packet, camera)?;
            }
        }
        while let Some((time, images)) = pairs.next(imu_time, ports.config.image_delay) {
            if !ports.sensors.add_images(time, &images, &ImageMap::new()) {
                log::warn!("OKVIS camera queue overrun or closed input");
            }
            let _ = ports.wake.try_send(());
        }
        if let Some(subscriber) = &depths {
            while let Some(packet) = subscriber.receive()? {
                let trace = *packet.user_header();
                let _span = trace.continue_span("okvis-receive-depth");
                let frame = depth(&packet)?;
                if last_depth.is_some_and(|last| frame.time <= last) {
                    continue;
                }
                last_depth = Some(frame.time);
                match ports.depths.try_send(frame) {
                    Ok(()) => {}
                    Err(TrySendError::Full(_)) => {
                        log::warn!("OKVIS depth ingress overrun");
                    }
                    Err(TrySendError::Disconnected(_)) => {
                        return Err(invalid("mapping host stopped"));
                    }
                }
            }
        }
    }
    firefly_observability::pump_log_ipc(&log_ipc);
    ports.stop.store(true, Ordering::Release);
    map_publisher.finish()?;
    Ok(())
}

struct Publishers {
    odometry: Publisher<OdomMessage>,
    corrected: Publisher<OdomMessage>,
    navigation: Publisher<NavigationMessage>,
    attitude: Publisher<AttitudeMessage>,
    viz: Option<VizPublisher>,
    last_viz: f64,
    external_localization: bool,
}
impl Publishers {
    fn open(
        node: &firefly_pubsub::node::IpcNode,
        config: &Config,
        topics: &Topics,
    ) -> Result<Self> {
        let odometry = Publisher::<OdomMessage>::with_topic(node, &topics.odometry)?;
        let corrected = Publisher::<OdomMessage>::with_topic(node, &topics.corrected)?;
        let navigation = Publisher::<NavigationMessage>::with_topic(node, &topics.navigation)?;
        let attitude = Publisher::<AttitudeMessage>::with_topic(node, &topics.attitude)?;
        let viz = config
            .diagnostics
            .then(|| VizPublisher::new(node))
            .transpose()?;
        Ok(Self {
            odometry,
            corrected,
            navigation,
            attitude,
            viz,
            last_viz: -1.0,
            external_localization: matches!(
                config.navigation,
                super::config::Navigation::ExternalLocalization
            ),
        })
    }

    fn posterior(&self, publication: &PublicationData) -> Result<()> {
        if let Some(viz) = &self.viz {
            let stamp = publication.state.timestamp.as_nanosecond() as f64 * 1e-9;
            let mut loops = ScalarMessage::base(stamp, "okvis/loop_commits");
            loops.scalars[0] = publication.tracking_state.backend.revision as f64;
            loops.scalar_count = 1;
            viz.publish(loops)?;

            // 跟踪质量用于诊断；IMU 传播的发布条件与上游 Trajectory 一致。
            let mut tracking = ScalarMessage::base(stamp, "okvis/tracking");
            tracking.scalars[0] = match publication.tracking_state.tracking_quality {
                TrackingQuality::Lost => 0.0,
                TrackingQuality::Marginal => 1.0,
                TrackingQuality::Good => 2.0,
            };
            tracking.scalar_count = 1;
            viz.publish(tracking)?;
        }
        Ok(())
    }
    fn publish(&mut self, sample: &output::Sample, stamp: f64) -> Result<()> {
        self.odometry.publish(sample.odom)?;
        if !self.external_localization {
            self.corrected.publish(sample.corrected)?;
            self.navigation.publish(sample.navigation)?;
        }
        if let Some(message) = sample.attitude {
            self.attitude.publish(message)?;
        }
        if stamp - self.last_viz >= 0.1 {
            let mut message = VizMessage::base(kind::POSE, stamp, "okvis/odom");
            message.xyz = [
                sample.odom.position_x,
                sample.odom.position_y,
                sample.odom.position_z,
            ];
            message.quat_xyzw = [
                sample.odom.quat_x,
                sample.odom.quat_y,
                sample.odom.quat_z,
                sample.odom.quat_w,
            ];
            if let Some(viz) = &self.viz {
                viz.publish(message)?;
            }
            self.last_viz = stamp;
        }
        Ok(())
    }
}

fn session() -> u64 {
    (jiff::Timestamp::now().as_nanosecond() as u64).max(1)
}
