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
    event::TopicListener,
    imu::{IMU_SUBSCRIPTION, IMU_TOPIC, ImuMessage},
    navigation::{NAVIGATION_TOPIC, NavigationMessage},
    node::create_node,
    odom::{CORRECTED_ODOM_TOPIC, ODOM_TOPIC, OdomMessage},
    publish::Publisher,
    subscriber::{Subscriber, SubscriberOptions},
    viz::{ScalarMessage, VizMessage, VizPublisher, kind},
};
use iceoryx2::prelude::{CallbackProgression, ipc};
use iceoryx2::waitset::{WaitSetAttachmentId, WaitSetBuilder, WaitSetRunResult};
use jiff::Timestamp;
use nalgebra::Vector3;
use purecv::core::DynamicMatrix;
use std::{
    cell::RefCell,
    collections::VecDeque,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicI64, Ordering},
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
#[derive(Clone)]
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

/// 发送线程输入：IMU 订阅、后验与四路发布所需的状态。
pub(super) struct SendPorts {
    map_snapshots: super::map_output::Mailbox,
    topics: Topics,
    config: Config,
    pub(super) sensors: SensorInputHandle,
    publications: Receiver<PublicationData>,
    stop: Arc<AtomicBool>,
    watermark: ImuWatermark,
}

/// 接收线程输入：双目与深度的订阅和转发通道。
pub(super) struct IngressPorts {
    topics: Topics,
    config: Config,
    pub(super) sensors: SensorInputHandle,
    depths: SyncSender<Depth>,
    wake: SyncSender<()>,
    stop: Arc<AtomicBool>,
    watermark: ImuWatermark,
}

impl Ports {
    pub fn split(self) -> (SendPorts, IngressPorts) {
        let watermark = ImuWatermark::default();
        (
            SendPorts {
                map_snapshots: self.map_snapshots,
                topics: self.topics.clone(),
                config: self.config.clone(),
                sensors: self.sensors.clone(),
                publications: self.publications,
                stop: self.stop.clone(),
                watermark: watermark.clone(),
            },
            IngressPorts {
                topics: self.topics,
                config: self.config,
                sensors: self.sensors,
                depths: self.depths,
                wake: self.wake,
                stop: self.stop,
                watermark,
            },
        )
    }
}

/// 发送线程已入估计器队列的 IMU 时间水位（纳秒）。
///
/// 接收线程只在水位覆盖图像时间后才提交双目帧；未收到 IMU 时保持空。
#[derive(Clone)]
pub(super) struct ImuWatermark(Arc<AtomicI64>);

impl Default for ImuWatermark {
    fn default() -> Self {
        Self(Arc::new(AtomicI64::new(i64::MIN)))
    }
}

impl ImuWatermark {
    pub fn store(&self, time: Timestamp) {
        if let Ok(nanoseconds) = i64::try_from(time.as_nanosecond()) {
            self.0.store(nanoseconds, Ordering::Release);
        }
    }

    pub fn load(&self) -> Option<Timestamp> {
        let nanoseconds = self.0.load(Ordering::Acquire);
        if nanoseconds == i64::MIN {
            return None;
        }
        Timestamp::from_nanosecond(i128::from(nanoseconds)).ok()
    }
}

/// 发送线程：IMU 入队、前向预测与四路发布。
pub(super) fn run_send(ports: SendPorts) -> Result<()> {
    let SendPorts {
        map_snapshots,
        topics,
        config,
        mut sensors,
        publications,
        stop,
        watermark,
    } = ports;
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "firefly-okvis-x");
    let imu = Subscriber::<ImuMessage>::with_options(&node, &topics.imu, IMU_SUBSCRIPTION)?;
    let imu_events = TopicListener::with_topic(&node, &topics.imu)?;
    let mut publishers = Publishers::open(&node, &config, &topics)?;
    let mut output = Output::new(config.clone(), session())?;
    let mut map_publisher = super::map_output::PublisherWorker::start(map_snapshots, stop.clone())?;
    let mut imu_time = None;

    let waitset = WaitSetBuilder::new()
        .create::<ipc::Service>()
        .map_err(|error| invalid(format!("create okvis WaitSet: {error:?}")))?;
    let _imu_guard = waitset
        .attach_notification(&imu_events)
        .map_err(|error| invalid(format!("attach IMU event: {error:?}")))?;
    let _timeout_guard = waitset
        .attach_interval(Duration::from_millis(2))
        .map_err(|error| invalid(format!("attach IMU timeout: {error:?}")))?;

    let failure = RefCell::new(None::<String>);
    let mut ipc_logged = 0u64;
    {
        let mut on_event = |_attachment: WaitSetAttachmentId<ipc::Service>| {
            if stop.load(Ordering::Acquire) {
                return CallbackProgression::Stop;
            }
            let _ = imu_events.drain();
            firefly_observability::pump_log_ipc(&log_ipc);
            let result = (|| -> Result<()> {
                while let Ok(publication) = publications.try_recv() {
                    output.posterior(&publication)?;
                    map_publisher.update(&output)?;
                    publishers.posterior(&publication)?;
                }
                while let Some(packet) = imu.receive()? {
                    handle_imu(
                        &packet,
                        *packet.user_header(),
                        &mut sensors,
                        &watermark,
                        &mut output,
                        &mut publishers,
                        &mut imu_time,
                        &mut ipc_logged,
                    )?;
                }
                Ok(())
            })();
            match result {
                Ok(()) => CallbackProgression::Continue,
                Err(error) => {
                    *failure.borrow_mut() = Some(error.to_string());
                    CallbackProgression::Stop
                }
            }
        };

        loop {
            if stop.load(Ordering::Acquire) {
                break;
            }
            match waitset.wait_and_process(&mut on_event) {
                Ok(WaitSetRunResult::AllEventsHandled) => {}
                Ok(
                    WaitSetRunResult::StopRequest
                    | WaitSetRunResult::Interrupt
                    | WaitSetRunResult::TerminationRequest,
                ) => break,
                Err(error) => return Err(invalid(format!("okvis WaitSet: {error:?}"))),
            }
        }
    }
    firefly_observability::pump_log_ipc(&log_ipc);
    stop.store(true, Ordering::Release);
    if let Some(message) = failure.into_inner() {
        return Err(invalid(message));
    }
    map_publisher.finish()?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn handle_imu(
    packet: &ImuMessage,
    trace: firefly_pubsub::trace::TraceContext,
    sensors: &mut SensorInputHandle,
    watermark: &ImuWatermark,
    output: &mut Output,
    publishers: &mut Publishers,
    imu_time: &mut Option<Timestamp>,
    ipc_logged: &mut u64,
) -> Result<()> {
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
        return Ok(());
    }
    firefly_observability::set_sim_time(packet.timestamp);
    *imu_time = Some(time);
    if !sensors.add_imu_measurement(time, &alpha, &omega) {
        return Err(invalid("estimator IMU queue rejected measurement"));
    }
    watermark.store(time);
    if let Some(sample) = output.imu(time, alpha, omega)? {
        publishers.publish(&sample, packet.timestamp)?;
        *ipc_logged += 1;
        if ipc_logged.is_multiple_of(10) && trace.send_ts_secs > 0 {
            let age_ms = (Timestamp::now().as_nanosecond() - trace.send_timestamp().as_nanosecond())
                as f64
                * 1e-6;
            publishers.ipc(packet.timestamp, age_ms.max(0.0))?;
        }
    }
    Ok(())
}

/// 接收线程：双目配对、图像拷贝、深度转换与转发。
pub(super) fn run_ingress(mut ports: IngressPorts) -> Result<()> {
    let node = create_node()?;
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
    let mut pairs = Pairs::new(ports.config.stereo_tolerance);
    let mut last_depth = None;

    while !ports.stop.load(Ordering::Acquire) && node.wait(Duration::from_millis(1)).is_ok() {
        for (camera, subscriber) in [(0, &left), (1, &right)] {
            while let Some(packet) = subscriber.receive()? {
                let trace = *packet.user_header();
                let _span = trace.continue_span("okvis-receive-image");
                pairs.push(&packet, camera)?;
            }
        }
        let imu_time = ports.watermark.load();
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
    ports.stop.store(true, Ordering::Release);
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
    /// IMU 从发布端发出到四路发布完成的端到端耗时（毫秒）。
    fn ipc(&self, stamp: f64, publish_ms: f64) -> Result<()> {
        if let Some(viz) = &self.viz {
            let mut message = ScalarMessage::base(stamp, "okvis/ipc/imu_publish_ms");
            message.scalars[0] = publish_ms;
            message.scalar_count = 1;
            viz.publish(message)?;
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
