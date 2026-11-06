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
use firefly_imu_propagation::ImuSample;
use firefly_pubsub::{
    attitude::ATTITUDE_TOPIC,
    camera::{
        CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, DEPTH_TOPIC, DepthImageMessage, GrayImageMessage,
        IMAGE_HEIGHT, IMAGE_WIDTH,
    },
    imu::IMU_TOPIC,
    navigation::NAVIGATION_TOPIC,
    node::create_node,
    odom::{CORRECTED_ODOM_TOPIC, ODOM_TOPIC},
    subscriber::{Subscriber, SubscriberOptions},
    viz::ScalarMessage,
};
pub(super) use firefly_vio_runtime::ImuWatermark;
use firefly_vio_runtime::{
    ImuAdapter, Publishers, publish_pose_diagnostic, publish_scalar_diagnostic,
};
use jiff::Timestamp;
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

/// The common runtime owns IMU events, validation, watermark and all four publishers.
pub(super) fn run_send(ports: SendPorts) -> Result<()> {
    let options = firefly_vio_runtime::Options {
        process: "firefly-okvis-x",
        topics: firefly_vio_runtime::Topics {
            imu: ports.topics.imu,
            odometry: ports.topics.odometry,
            corrected: ports.topics.corrected,
            navigation: ports.topics.navigation,
            attitude: ports.topics.attitude,
        },
        diagnostics: ports.config.diagnostics,
        external_localization: matches!(
            ports.config.navigation,
            super::config::Navigation::ExternalLocalization
        ),
        imu_latency_entity: "okvis/ipc/imu_publish_ms",
    };
    firefly_vio_runtime::run_imu(&options, &ports.watermark, &ports.stop, || {
        Ok(Adapter {
            sensors: ports.sensors,
            publications: ports.publications,
            output: Output::new(&ports.config, session())?,
            map_publisher: super::map_output::PublisherWorker::start(
                ports.map_snapshots,
                ports.stop.clone(),
            )?,
        })
    })
}

struct Adapter {
    sensors: SensorInputHandle,
    publications: Receiver<PublicationData>,
    output: Output,
    map_publisher: super::map_output::PublisherWorker,
}
impl ImuAdapter for Adapter {
    fn posteriors(&mut self, publishers: &Publishers) -> Result<()> {
        while let Ok(publication) = self.publications.try_recv() {
            self.output.posterior(&publication)?;
            self.map_publisher.update(&self.output)?;
            posterior_diagnostics(publishers, &publication)?;
        }
        Ok(())
    }
    fn enqueue(&mut self, sample: ImuSample) -> Result<()> {
        let time = Timestamp::from_nanosecond(sample.timestamp_ns).map_err(invalid)?;
        if !self
            .sensors
            .add_imu_measurement(time, &sample.accel, &sample.gyro)
        {
            return Err(invalid("estimator IMU queue rejected measurement"));
        }
        Ok(())
    }
    fn predict(&mut self, sample: ImuSample) -> Result<Option<output::Sample>> {
        self.output.imu(
            Timestamp::from_nanosecond(sample.timestamp_ns).map_err(invalid)?,
            sample.accel,
            sample.gyro,
        )
    }
    fn finish(mut self) -> Result<()> {
        self.map_publisher.finish()
    }
}

fn posterior_diagnostics(publishers: &Publishers, publication: &PublicationData) -> Result<()> {
    if let Some(viz) = publishers.viz() {
        let stamp = publication.state.timestamp.as_nanosecond() as f64 * 1e-9;
        let pose = &publication.state.t_ws;
        let q = pose.quaternion();
        publish_pose_diagnostic(
            viz,
            stamp,
            "vio/posterior/pose_imu_world",
            pose.translation().into(),
            [q.i, q.j, q.k, q.w],
        );
        let speed_bias = publication.state.speed_and_bias();
        publish_scalar_diagnostic(
            viz,
            stamp,
            "vio/posterior/velocity_world",
            speed_bias.fixed_rows::<3>(0).into(),
        );
        publish_scalar_diagnostic(
            viz,
            stamp,
            "vio/posterior/gyro_bias_imu",
            speed_bias.fixed_rows::<3>(3).into(),
        );
        publish_scalar_diagnostic(
            viz,
            stamp,
            "vio/posterior/accel_bias_imu",
            speed_bias.fixed_rows::<3>(6).into(),
        );
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

fn session() -> u64 {
    (jiff::Timestamp::now().as_nanosecond() as u64).max(1)
}
