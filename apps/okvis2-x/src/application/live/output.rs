// SPDX-License-Identifier: BSD-3-Clause
//! Bounded IMU replay and coordinate adaptation of committed estimator posteriors.
use super::config::{Config, invalid};
use crate::{
    ceres::imu_error::{Covariance, ImuError, Jacobian},
    frames::frame_typedefs::StateId,
    kinematics::transformation::Transformation,
    runtime::{threaded_slam::PublicationData, vi_interface::TrackingQuality},
    sensors::measurements::{ImuMeasurement, ImuMeasurementDeque, ImuSensorReadings},
};
use firefly_base::{FrameId, RigidTransform};
use firefly_error::Result;
use firefly_pubsub::{attitude::AttitudeMessage, navigation::NavigationMessage, odom::OdomMessage};
use jiff::Timestamp;
use nalgebra::{SVector, Vector3};
use std::collections::{BTreeMap, VecDeque};

pub(super) struct Sample {
    pub odom: OdomMessage,
    pub corrected: OdomMessage,
    pub navigation: NavigationMessage,
    pub attitude: Option<AttitudeMessage>,
}
struct Prediction {
    posterior: Timestamp,
    time: Timestamp,
    pose: Transformation,
    speed_bias: SVector<f64, 9>,
    covariance: Option<Covariance>,
}
pub(super) struct Output {
    config: Config,
    imu: ImuMeasurementDeque,
    prediction: Option<Prediction>,
    anchors: BTreeMap<StateId, Transformation>,
    map_from_odom: Transformation,
    graph_revision: u64,
    revision: u64,
    session: u64,
    last_published: Option<Timestamp>,
}
impl Output {
    /// 进程运行标识；只随进程启动变化，跟踪丢失不改变它。
    #[cfg(test)]
    pub fn session_id(&self) -> u64 {
        self.session
    }
    pub fn new(config: Config, session: u64) -> Self {
        Self {
            config,
            imu: VecDeque::new(),
            prediction: None,
            anchors: BTreeMap::new(),
            map_from_odom: Transformation::identity(),
            graph_revision: 0,
            revision: 0,
            session,
            last_published: None,
        }
    }
    pub fn imu(
        &mut self,
        time: Timestamp,
        alpha: Vector3<f64>,
        omega: Vector3<f64>,
    ) -> Result<Option<Sample>> {
        if self.imu.back().is_some_and(|m| m.time_stamp >= time) {
            return Ok(None);
        }
        let alpha = alpha.component_mul(&Vector3::from(self.config.imu.accelerometer_scale));
        self.imu.push_back(ImuMeasurement::new(
            time,
            ImuSensorReadings::new(omega, alpha),
            -1,
        ));
        while self.imu.len() > 2
            && (self.imu.len() > 10000
                || seconds(time, self.imu[1].time_stamp) > self.config.imu_history_seconds)
        {
            self.imu.pop_front();
        }
        self.predict(time)
    }
    pub fn posterior(&mut self, publication: &PublicationData) -> Result<()> {
        let state = &publication.state;
        if publication.tracking_state.tracking_quality == TrackingQuality::Lost {
            // 跟踪丢失只让预测失效，不改 `session`：`session` 是进程运行标识，
            // 下游（飞控）把它的变化当作进程重启，会永久降级到备用姿态源。
            // 预测清空后不再发 AttitudeMessage，下游按新鲜度超时自然回落并在
            // 恢复后重新接纳；跟踪质量同时经 viz 标量暴露。
            if self.prediction.take().is_some() {
                log::warn!("VIO 跟踪丢失，停止姿态预测（会话不变）");
            }
            return Ok(());
        }
        if self
            .prediction
            .as_ref()
            .is_some_and(|p| state.timestamp < p.posterior)
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
                self.map_from_odom = updated.t_ws.compose(&local.inverse());
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
            .insert(state.id, self.map_from_odom.inverse().compose(&state.t_ws));
        while self.anchors.len() > 512 {
            self.anchors.pop_first();
        }
        self.prediction = Some(Prediction {
            posterior: state.timestamp,
            time: state.timestamp,
            pose: state.t_ws.clone(),
            speed_bias: state.speed_and_bias(),
            covariance: state.covariance,
        });
        Ok(())
    }
    fn predict(&mut self, time: Timestamp) -> Result<Option<Sample>> {
        let Some(prediction) = self.prediction.as_mut() else {
            return Ok(None);
        };
        if time <= prediction.time
            || self.last_published.is_some_and(|last| time <= last)
            || seconds(time, prediction.posterior) > self.config.max_posterior_age
        {
            return Ok(None);
        }
        let start = self
            .imu
            .iter()
            .rposition(|m| m.time_stamp <= prediction.time);
        let Some(start) = start else {
            return Ok(None);
        };
        let measurements: ImuMeasurementDeque = self
            .imu
            .iter()
            .skip(start)
            .take_while(|m| m.time_stamp <= time)
            .cloned()
            .collect();
        if measurements.len() < 2 {
            return Ok(None);
        }
        if measurements
            .iter()
            .zip(measurements.iter().skip(1))
            .any(|(a, b)| seconds(b.time_stamp, a.time_stamp) > self.config.max_imu_gap)
        {
            self.prediction = None;
            return Ok(None);
        }
        let parameters = self.config.imu_parameters()?;
        let mut noise = Covariance::zeros();
        let mut jacobian = Jacobian::identity();
        ImuError::propagation(
            &measurements,
            &parameters,
            &mut prediction.pose,
            &mut prediction.speed_bias,
            (prediction.time, time),
            (Some(&mut noise), Some(&mut jacobian)),
        )
        .map_err(|error| invalid(format!("IMU prediction: {error:?}")))?;
        if let Some(covariance) = &mut prediction.covariance {
            *covariance = jacobian * *covariance * jacobian.transpose() + noise;
        }
        prediction.time = time;
        let omega = self.imu.back().unwrap().measurement.gyroscopes
            - prediction.speed_bias.fixed_rows::<3>(3).into_owned();
        let body_from_imu = self.config.imu.extrinsic.pose()?;
        let sensor_from_body = body_from_imu.inverse();
        let map_from_body = prediction.pose.compose(&sensor_from_body);
        let velocity_map = prediction.speed_bias.fixed_rows::<3>(0).into_owned()
            + prediction
                .pose
                .rotate_vector(&omega.cross(&sensor_from_body.translation()));
        let odom_from_map = self.map_from_odom.inverse();
        let local = odom_from_map.compose(&map_from_body);
        let odom = message(time, &local, &odom_from_map.rotate_vector(&velocity_map));
        let corrected = message(time, &map_from_body, &velocity_map);
        let alignment = rigid(&self.map_from_odom)?;
        let navigation = NavigationMessage::new(corrected, &alignment, self.revision);
        let attitude = prediction.covariance.as_ref().map(|covariance| {
            let rotation = odom_from_map.rotation_matrix();
            let variance = rotation * covariance.fixed_view::<3, 3>(3, 3) * rotation.transpose();
            let q = local.quaternion();
            AttitudeMessage {
                timestamp: timestamp(time),
                posterior_time: timestamp(prediction.posterior),
                session: self.session,
                quat_xyzw: [q.i, q.j, q.k, q.w],
                angular_velocity: body_from_imu.rotate_vector(&omega).into(),
                gyro_bias: body_from_imu
                    .rotate_vector(&prediction.speed_bias.fixed_rows::<3>(3).into_owned())
                    .into(),
                attitude_variance: variance.diagonal().into(),
            }
        });
        self.last_published = Some(time);
        Ok(Some(Sample {
            odom,
            corrected,
            navigation,
            attitude,
        }))
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
fn seconds(end: Timestamp, start: Timestamp) -> f64 {
    (end.as_nanosecond() - start.as_nanosecond()) as f64 * 1e-9
}
fn timestamp(time: Timestamp) -> f64 {
    time.as_nanosecond() as f64 * 1e-9
}
