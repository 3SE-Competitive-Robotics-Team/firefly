// SPDX-License-Identifier: BSD-3-Clause
//! Bounded IMU replay and coordinate adaptation of committed estimator posteriors.
use super::config::{Config, invalid};
use crate::{
    config::parameters::ImuParameters, frames::frame_typedefs::StateId,
    kinematics::transformation::Transformation, runtime::threaded_slam::PublicationData,
};
use firefly_base::{FrameId, RigidTransform};
use firefly_error::Result;
use firefly_imu_propagation::{Bias, ImuSample, Predictor, State};
use firefly_pubsub::{attitude::AttitudeMessage, navigation::NavigationMessage, odom::OdomMessage};
pub(super) use firefly_vio_runtime::Sample;
use jiff::Timestamp;
use nalgebra::Vector3;
use std::collections::BTreeMap;

pub(super) struct Output {
    imu_parameters: ImuParameters,
    sensor_from_body: Transformation,
    predictor: Predictor,
    anchors: BTreeMap<StateId, Transformation>,
    odom_from_map: Transformation,
    alignment: RigidTransform,
    graph_revision: u64,
    revision: u64,
    session: u64,
}
impl Output {
    pub fn graph_revision(&self) -> u64 {
        self.graph_revision
    }
    pub fn stamp_map(&self, header: &mut firefly_pubsub::map::MapElement, sequence: u64) {
        header.session = self.session;
        header.sequence = sequence;
        header.revision = self.revision;
        header.translation = self.alignment.isometry().translation.vector.into();
        let q = self.alignment.isometry().rotation.quaternion();
        header.rotation = [q.i, q.j, q.k, q.w];
    }
    /// 进程运行标识；只随进程启动变化，跟踪丢失不改变它。
    #[cfg(test)]
    pub fn session_id(&self) -> u64 {
        self.session
    }
    pub fn new(config: &Config, session: u64) -> Result<Self> {
        let imu_parameters = config.imu_parameters()?;
        let sensor_from_body = imu_parameters.t_bs.inverse();
        let map_from_odom = Transformation::identity();
        let alignment = rigid(&map_from_odom)?;
        Ok(Self {
            predictor: Predictor::new(
                config.imu_history_seconds,
                Vector3::new(0., 0., -imu_parameters.g),
            ),
            imu_parameters,
            sensor_from_body,
            anchors: BTreeMap::new(),
            odom_from_map: Transformation::identity(),
            alignment,
            graph_revision: 0,
            revision: 0,
            session,
        })
    }
    pub fn imu(
        &mut self,
        time: Timestamp,
        alpha: Vector3<f64>,
        omega: Vector3<f64>,
    ) -> Result<Option<Sample>> {
        let prediction = self
            .predictor
            .imu(ImuSample {
                timestamp_ns: time.as_nanosecond(),
                accel: alpha.component_mul(&self.imu_parameters.s_a),
                gyro: omega,
            })
            .map_err(|e| invalid(format!("IMU prediction: {e:?}")))?;
        prediction.map(|p| self.sample(time, p)).transpose()
    }
    pub fn posterior(&mut self, publication: &PublicationData) -> Result<()> {
        let state = &publication.state;
        if self
            .predictor
            .posterior_time_ns()
            .is_some_and(|p| state.timestamp.as_nanosecond() < p)
        {
            return Ok(());
        }
        let revision = publication.tracking_state.backend.revision;
        if revision != self.graph_revision {
            let anchor = self.anchors.iter().rev().find_map(|(id, local)| {
                publication
                    .updated_states
                    .get(id)
                    .map(|updated| (local, updated))
            });
            if let Some((local, updated)) = anchor {
                let map_from_odom = updated.t_ws.compose(&local.inverse());
                let alignment = rigid(&map_from_odom)?;
                self.odom_from_map = map_from_odom.inverse();
                self.alignment = alignment;
            } else if !self.anchors.is_empty() {
                return Err(invalid(
                    "full-graph correction has no common odometry anchor",
                ));
            }
            self.graph_revision = revision;
            self.revision = self
                .revision
                .checked_add(1)
                .ok_or_else(|| invalid("navigation revision exhausted"))?;
        }
        self.anchors
            .insert(state.id, self.odom_from_map.compose(&state.t_ws));
        while self.anchors.len() > 512 {
            self.anchors.pop_first();
        }
        let speed_bias = state.speed_and_bias();
        self.predictor.posterior(
            State {
                timestamp_ns: state.timestamp.as_nanosecond(),
                rotation: state.t_ws.quaternion(),
                position: state.t_ws.translation(),
                velocity: speed_bias.fixed_rows::<3>(0).into_owned(),
            },
            Bias {
                gyro: speed_bias.fixed_rows::<3>(3).into_owned(),
                accel: speed_bias.fixed_rows::<3>(6).into_owned(),
            },
        );
        Ok(())
    }
    fn sample(
        &self,
        time: Timestamp,
        predicted: firefly_imu_propagation::Prediction,
    ) -> Result<Sample> {
        let pose = Transformation::new(predicted.state.position, predicted.state.rotation);
        let omega = predicted.angular_velocity;
        let body_from_imu = &self.imu_parameters.t_bs;
        let sensor_from_body = &self.sensor_from_body;
        let map_from_body = pose.compose(sensor_from_body);
        let velocity_map = predicted.state.velocity
            + pose.rotate_vector(&omega.cross(&sensor_from_body.translation()));
        let odom_from_map = &self.odom_from_map;
        let local = odom_from_map.compose(&map_from_body);
        let odom = message(time, &local, &odom_from_map.rotate_vector(&velocity_map));
        let corrected = message(time, &map_from_body, &velocity_map);
        let navigation = NavigationMessage::new(corrected, &self.alignment, self.revision);
        // 上游 OKVIS2-X 不计算状态协方差，姿态照常发布；这里同样不做协方差门控。
        let attitude = Some({
            let q = local.quaternion();
            AttitudeMessage {
                timestamp: timestamp(time),
                posterior_time: self
                    .predictor
                    .posterior_time_ns()
                    .ok_or_else(|| invalid("missing posterior time"))?
                    as f64
                    * 1e-9,
                session: self.session,
                quat_xyzw: [q.i, q.j, q.k, q.w],
                angular_velocity: body_from_imu.rotate_vector(&omega).into(),
                gyro_bias: body_from_imu
                    .rotate_vector(&self.predictor.bias().gyro)
                    .into(),
            }
        });
        Ok(Sample {
            odom,
            corrected,
            navigation,
            attitude,
        })
    }
}
fn message(time: Timestamp, pose: &Transformation, velocity: &Vector3<f64>) -> OdomMessage {
    let p = pose.translation();
    let q = pose.quaternion();
    OdomMessage {
        timestamp: timestamp(time),
        position_x: p.x,
        position_y: p.y,
        position_z: p.z,
        velocity_x: velocity.x,
        velocity_y: velocity.y,
        velocity_z: velocity.z,
        quat_x: q.i,
        quat_y: q.j,
        quat_z: q.k,
        quat_w: q.w,
        is_initialized: true,
    }
}
pub(super) fn rigid(pose: &Transformation) -> Result<RigidTransform> {
    let q = pose.quaternion();
    RigidTransform::from_parts(
        FrameId::MAP,
        FrameId::ODOM,
        pose.translation().into(),
        [q.i, q.j, q.k, q.w],
    )
}
pub(super) fn time(seconds: f64) -> Result<Timestamp> {
    if !seconds.is_finite() || seconds < 0.0 {
        return Err(invalid("invalid sensor timestamp"));
    }
    Timestamp::from_nanosecond((seconds * 1e9).round() as i128).map_err(invalid)
}

pub(super) fn timestamp(time: Timestamp) -> f64 {
    time.as_nanosecond() as f64 * 1e-9
}
