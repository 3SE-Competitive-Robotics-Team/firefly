// SPDX-License-Identifier: BSD-3-Clause
//! Identical publication order, timestamps and diagnostics for all VIO backends.
use firefly_error::Result;
use firefly_pubsub::{
    attitude::AttitudeMessage,
    navigation::NavigationMessage,
    odom::OdomMessage,
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
    last_viz: f64,
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
            last_viz: -1.0,
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
        if stamp - self.last_diagnostic >= 0.02 {
            if let Some(viz) = &self.viz {
                publish_pose_diagnostic(
                    viz,
                    stamp,
                    "vio/prediction/pose_body_odom",
                    [
                        sample.odom.position_x,
                        sample.odom.position_y,
                        sample.odom.position_z,
                    ],
                    [
                        sample.odom.quat_x,
                        sample.odom.quat_y,
                        sample.odom.quat_z,
                        sample.odom.quat_w,
                    ],
                );
                publish_scalar_diagnostic(
                    viz,
                    stamp,
                    "vio/prediction/velocity_body_odom",
                    [
                        sample.odom.velocity_x,
                        sample.odom.velocity_y,
                        sample.odom.velocity_z,
                    ],
                );
                if let Some(attitude) = sample.attitude {
                    publish_scalar_diagnostic(
                        viz,
                        stamp,
                        "vio/prediction/angular_velocity_body",
                        attitude.angular_velocity,
                    );
                    publish_scalar_diagnostic(
                        viz,
                        stamp,
                        "vio/prediction/posterior_age_s",
                        [attitude.timestamp - attitude.posterior_time],
                    );
                }
            }
            self.last_diagnostic = stamp;
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
