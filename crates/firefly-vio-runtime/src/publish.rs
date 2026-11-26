use firefly_error::Result;
use firefly_pubsub::{
    attitude::AttitudeMessage,
    navigation::NavigationMessage,
    odom::{OdomMessage, VIO_PREDICTION_BUFFER_SIZE, VIO_PREDICTION_TOPIC, VioPrediction},
    publish::Publisher,
    viz::{ScalarMessage, VizMessage, VizPublisher, kind},
};

pub struct Sample {
    pub odom: OdomMessage,
    pub corrected: OdomMessage,
    pub navigation: NavigationMessage,
    pub attitude: Option<AttitudeMessage>,
}
pub struct Publishers {
    odometry: Publisher<OdomMessage>,
    corrected: Publisher<OdomMessage>,
    navigation: Publisher<NavigationMessage>,
    attitude: Publisher<AttitudeMessage>,
    viz: Option<VizPublisher>,
    prediction: Option<Publisher<VioPrediction>>,
    prediction_sequence: u64,
    last_diagnostic: f64,
    external_localization: bool,
    imu_latency_entity: &'static str,
}
impl Publishers {
    #[must_use]
    pub fn viz(&self) -> Option<&VizPublisher> {
        self.viz.as_ref()
    }

    pub(crate) fn open(
        node: &firefly_pubsub::node::IpcNode,
        options: &crate::Options,
    ) -> Result<Self> {
        let topics = &options.topics;
        let odometry = Publisher::<OdomMessage>::with_topic(node, &topics.odometry)?;
        let corrected = Publisher::<OdomMessage>::with_topic(node, &topics.corrected)?;
        let navigation = Publisher::<NavigationMessage>::with_topic(node, &topics.navigation)?;
        let attitude = Publisher::<AttitudeMessage>::with_topic(node, &topics.attitude)?;
        let viz = options
            .diagnostics
            .then(|| VizPublisher::new(node))
            .transpose()?;
        Ok(Self {
            odometry,
            corrected,
            navigation,
            attitude,
            viz,
            prediction: options
                .diagnostics
                .then(|| {
                    Publisher::with_topic_buffer_publishers(
                        node,
                        VIO_PREDICTION_TOPIC,
                        Some(VIO_PREDICTION_BUFFER_SIZE),
                        Some(1),
                    )
                })
                .transpose()?,
            prediction_sequence: 0,
            last_diagnostic: -1.0,
            external_localization: options.external_localization,
            imu_latency_entity: options.imu_latency_entity,
        })
    }

    /// IMU 从发布端发出到四路发布完成的端到端耗时（毫秒）。
    pub(crate) fn ipc(&self, stamp: f64, publish_ms: f64) -> Result<()> {
        if let Some(viz) = &self.viz {
            let mut message = ScalarMessage::base(stamp, self.imu_latency_entity);
            message.scalars[0] = publish_ms;
            message.scalar_count = 1;
            viz.publish(message)?;
        }
        Ok(())
    }
    pub(crate) fn publish(&mut self, sample: &Sample, stamp: f64) -> Result<()> {
        self.odometry.publish(sample.odom)?;
        if !self.external_localization {
            self.corrected.publish(sample.corrected)?;
            self.navigation.publish(sample.navigation)?;
        }
        if let Some(message) = sample.attitude {
            self.attitude.publish(message)?;
        }
        if stamp - self.last_diagnostic >= 0.02
            && let Some(port) = &self.prediction
        {
            let odom = &sample.odom;
            let message = VioPrediction {
                timestamp: stamp,
                sequence: self.prediction_sequence,
                position: [odom.position_x, odom.position_y, odom.position_z],
                quat_xyzw: [odom.quat_x, odom.quat_y, odom.quat_z, odom.quat_w],
                velocity: [odom.velocity_x, odom.velocity_y, odom.velocity_z],
                has_attitude: u8::from(sample.attitude.is_some()),
                angular_velocity: sample.attitude.map_or([0.; 3], |a| a.angular_velocity),
                posterior_age_s: sample
                    .attitude
                    .map_or(0., |a| a.timestamp - a.posterior_time),
            };
            self.prediction_sequence = self.prediction_sequence.wrapping_add(1);
            if let Err(error) = port.publish(message) {
                log::warn!("VIO prediction observation: {error}");
            }
            self.last_diagnostic = stamp;
        }
        Ok(())
    }
}

pub fn publish_pose_diagnostic(
    viz: &VizPublisher,
    stamp: f64,
    entity: &str,
    position: [f64; 3],
    quaternion_xyzw: [f64; 4],
) {
    let mut message = VizMessage::base(kind::POSE, stamp, entity);
    message.xyz = position;
    message.quat_xyzw = quaternion_xyzw;
    let _ = viz.publish(message);
}

pub fn publish_scalar_diagnostic<const N: usize>(
    viz: &VizPublisher,
    stamp: f64,
    entity: &str,
    values: [f64; N],
) {
    let mut message = ScalarMessage::base(stamp, entity);
    message.scalars[..N].copy_from_slice(&values);
    message.scalar_count = N as u32;
    let _ = viz.publish(message);
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_pubsub::{node::create_node, subscriber::Subscriber};

    #[test]
    fn external_localization_exclusively_owns_map_outputs() {
        let node = create_node().unwrap();
        for external in [false, true] {
            let prefix = format!(
                "Firefly/Test/VioOwnership/{}/{external}",
                std::process::id()
            );
            let topics = crate::Topics {
                imu: format!("{prefix}/Imu"),
                odometry: format!("{prefix}/Odometry"),
                corrected: format!("{prefix}/Corrected"),
                navigation: format!("{prefix}/Navigation"),
                attitude: format!("{prefix}/Attitude"),
            };
            let odometry = Subscriber::<OdomMessage>::with_topic(&node, &topics.odometry).unwrap();
            let corrected =
                Subscriber::<OdomMessage>::with_topic(&node, &topics.corrected).unwrap();
            let navigation =
                Subscriber::<NavigationMessage>::with_topic(&node, &topics.navigation).unwrap();
            let attitude =
                Subscriber::<AttitudeMessage>::with_topic(&node, &topics.attitude).unwrap();
            let mut publishers = Publishers::open(
                &node,
                &crate::Options {
                    process: "vio-ownership-test",
                    topics,
                    diagnostics: false,
                    external_localization: external,
                    imu_latency_entity: "rootslam/ipc/imu_publish_ms",
                },
            )
            .unwrap();
            let odom = OdomMessage {
                timestamp: 2.,
                is_initialized: true,
                ..Default::default()
            };
            publishers
                .publish(
                    &Sample {
                        odom,
                        corrected: odom,
                        navigation: NavigationMessage {
                            odom,
                            ..Default::default()
                        },
                        attitude: Some(AttitudeMessage {
                            timestamp: 2.,
                            posterior_time: 1.9,
                            session: 1,
                            quat_xyzw: [0., 0., 0., 1.],
                            angular_velocity: [0.; 3],
                            gyro_bias: [0.; 3],
                            stereo_landmarks: 12,
                        }),
                    },
                    2.,
                )
                .unwrap();
            assert_eq!(odometry.receive().unwrap().unwrap().timestamp, 2.);
            assert_eq!(attitude.receive().unwrap().unwrap().timestamp, 2.);
            assert_eq!(corrected.receive().unwrap().is_some(), !external);
            assert_eq!(navigation.receive().unwrap().is_some(), !external);
        }
    }
}
