/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! State records, trajectory reconstruction and sensor-facing estimator interface.
use crate::{
    ceres::imu_error::{IntegrationError, NoiseMode, Preintegration, seconds},
    frames::frame_typedefs::{MapPointVector, SpeedAndBias, StateId},
    kinematics::transformation::{
        Transformation, delta_q, normalized_quaternion, quaternion_from_rotation_matrix,
    },
    sensors::measurements::{ImuMeasurement, ImuMeasurementDeque, ImuSensorReadings},
};
use jiff::Timestamp;
use nalgebra::{Matrix4, Quaternion, Vector3};
use purecv::core::DynamicMatrix;
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::{self, Write},
    path::Path,
    sync::Arc,
};

/// `MatExpr` applies the combined scale in double precision, then rounds to the
/// input depth before the elementwise exponential.
pub fn exp_image_transform(frame: &mut [f32]) {
    for x in frame {
        let scaled = (f64::from(*x) * (-std::f64::consts::LN_2 / 10.0)) as f32;
        *x = f64::from(scaled).exp() as f32;
    }
}
/// `OpenCV` logarithms ignore the sign bit. Exponent-zero and exponent-all-ones
/// inputs retain the HAL's encoded exponent/mantissa interpretation. The
/// standard-library logarithm approximates the finite mantissa; HAL rounding
/// may differ by a few ULPs.
pub fn log_image_transform(frame: &mut [f32]) {
    for x in frame {
        let bits = x.to_bits() & 0x7fff_ffff;
        let exponent = (bits >> 23) as i32;
        let value = if exponent == 0 || exponent == 255 {
            (f64::from(exponent - 127) * std::f64::consts::LN_2
                + (1.0 + f64::from(bits & 0x7f_ffff) / 8_388_608.0).ln()) as f32
        } else {
            f64::from(x.abs()).ln() as f32
        };
        *x = (f64::from(value) * (-10.0 / std::f64::consts::LN_2)) as f32;
        if *x == 0.0 {
            *x = 0.0;
        }
    }
}
pub fn exp_image_transform_f64(frame: &mut [f64]) {
    for x in frame {
        *x = if x.is_nan() {
            0.0
        } else {
            (*x * (-std::f64::consts::LN_2 / 10.0)).exp()
        };
    }
}
pub fn log_image_transform_f64(frame: &mut [f64]) {
    for x in frame {
        let bits = x.to_bits() & 0x7fff_ffff_ffff_ffff;
        let exponent = (bits >> 52) as i32;
        let value = if exponent == 0 || exponent == 2047 {
            f64::from(exponent - 1023) * std::f64::consts::LN_2
                + (1.0 + (bits & 0x000f_ffff_ffff_ffff) as f64 / 4_503_599_627_370_496.0).ln()
        } else {
            x.abs().ln()
        };
        *x = value * (-10.0 / std::f64::consts::LN_2);
        if *x == 0.0 {
            *x = 0.0;
        }
    }
}
#[derive(Clone, Debug)]
pub struct State {
    pub t_ws: Transformation,
    pub v_w: Vector3<f64>,
    pub b_g: Vector3<f64>,
    pub b_a: Vector3<f64>,
    pub omega_s: Vector3<f64>,
    pub timestamp: Timestamp,
    pub id: StateId,
    pub previous_imu_measurements: ImuMeasurementDeque,
    /// Joint [position, orientation, velocity, gyro bias, accel bias] covariance.
    pub covariance: Option<crate::ceres::imu_error::Covariance>,
    pub is_keyframe: bool,
    pub t_ai_w: BTreeMap<u64, Transformation>,
    pub covisible_frame_ids: BTreeSet<StateId>,
    pub state_changed: Option<bool>,
    pub is_online_extrinsics: Option<bool>,
    pub extrinsics: BTreeMap<u64, Transformation>,
    pub t_gw: Transformation,
    pub gps_points: Vec<Vector3<f64>>,
}
impl State {
    #[must_use]
    pub fn new(
        t_ws: Transformation,
        v_w: Vector3<f64>,
        (b_g, b_a): (Vector3<f64>, Vector3<f64>),
        omega_s: Vector3<f64>,
        timestamp: Timestamp,
        id: StateId,
        is_keyframe: bool,
    ) -> Self {
        Self {
            t_ws,
            v_w,
            b_g,
            b_a,
            omega_s,
            timestamp,
            id,
            is_keyframe,
            previous_imu_measurements: ImuMeasurementDeque::new(),
            covariance: None,
            t_ai_w: BTreeMap::new(),
            covisible_frame_ids: BTreeSet::new(),
            state_changed: None,
            is_online_extrinsics: None,
            extrinsics: BTreeMap::new(),
            t_gw: Transformation::default(),
            gps_points: Vec::new(),
        }
    }
    #[must_use]
    pub fn speed_and_bias(&self) -> SpeedAndBias {
        let mut sb = SpeedAndBias::zeros();
        sb.fixed_rows_mut::<3>(0).copy_from(&self.v_w);
        sb.fixed_rows_mut::<3>(3).copy_from(&self.b_g);
        sb.fixed_rows_mut::<3>(6).copy_from(&self.b_a);
        sb
    }
    fn propagated(
        &self,
        pose: Transformation,
        sb: SpeedAndBias,
        omega: Vector3<f64>,
        time: Timestamp,
    ) -> Self {
        let mut out = self.clone();
        out.t_ws = pose;
        out.v_w = sb.fixed_rows::<3>(0).into_owned();
        out.b_g = sb.fixed_rows::<3>(3).into_owned();
        out.b_a = sb.fixed_rows::<3>(6).into_owned();
        out.omega_s = omega;
        out.timestamp = time;
        out.is_keyframe = false;
        out.id = StateId::default();
        out.state_changed = None;
        out.is_online_extrinsics = None;
        out.t_ai_w.clear();
        out.covisible_frame_ids.clear();
        out.extrinsics.clear();
        out.gps_points.clear();
        out.t_gw = Transformation::default();
        out
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackingQuality {
    Good,
    Marginal,
    Lost,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct BackendTrackingState {
    /// Monotonic count of committed full-graph corrections.
    pub revision: u64,
    pub recognised_place: bool,
    pub is_full_graph_optimising: bool,
}
#[derive(Clone, Debug)]
pub struct TrackingState {
    pub id: StateId,
    pub is_keyframe: bool,
    pub is_lidar_keyframe: bool,
    pub tracking_quality: TrackingQuality,
    pub backend: BackendTrackingState,

    pub current_keyframe_id: StateId,
}

#[derive(Clone, Debug)]
pub struct Propagator {
    t_start: Timestamp,
    t_end: Timestamp,
    delta_t: f64,
    integration: Preintegration,
    reference: SpeedAndBias,
    omega_s: Vector3<f64>,
}
impl Propagator {
    #[must_use]
    pub fn new(start: Timestamp) -> Self {
        Self {
            t_start: start,
            t_end: start,
            delta_t: 0.0,
            integration: Preintegration::default(),
            reference: SpeedAndBias::zeros(),
            omega_s: Vector3::zeros(),
        }
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn append_to(
        &mut self,
        measurements: &ImuMeasurementDeque,
        _pose: &Transformation,
        sb: &SpeedAndBias,
        end: Timestamp,
    ) -> Result<bool, IntegrationError> {
        if measurements
            .front()
            .ok_or(IntegrationError::MissingMeasurements)?
            .time_stamp
            > self.t_end
        {
            return Err(IntegrationError::StartNotCovered);
        }
        if measurements
            .back()
            .ok_or(IntegrationError::MissingMeasurements)?
            .time_stamp
            < end
        {
            return Ok(false);
        }
        let mut time = self.t_end;
        self.t_end = end;
        self.reference = *sb;
        let mut started = false;
        for i in 0..measurements.len().saturating_sub(1) {
            let m0 = &measurements[i];
            let m1 = &measurements[i + 1];
            let (mut w0, mut w1) = (m0.measurement.gyroscopes, m1.measurement.gyroscopes);
            let (mut a0, mut a1) = (m0.measurement.accelerometers, m1.measurement.accelerometers);
            let mut next = m1.time_stamp;
            let mut dt = seconds(next, time);
            if end < next {
                let interval = seconds(next, m0.time_stamp);
                next = end;
                dt = seconds(next, time);
                let r = dt / interval;
                w1 = (1.0 - r) * w0 + r * w1;
                a1 = (1.0 - r) * a0 + r * a1;
            }
            if dt <= 0.0 {
                continue;
            }
            if !started {
                started = true;
                let r = dt / seconds(next, m0.time_stamp);
                w0 = r * w0 + (1.0 - r) * w1;
                a0 = r * a0 + (1.0 - r) * a1;
            }
            self.omega_s =
                self.integration
                    .step((w0, w1), (a0, a1), dt, sb, None, (false, NoiseMode::None));
            time = next;
            if next == end {
                break;
            }
        }
        self.delta_t = seconds(self.t_end, self.t_start);
        Ok(true)
    }
    #[must_use]
    pub fn get_state(
        &self,
        pose: &Transformation,
        sb: &SpeedAndBias,
    ) -> (Transformation, SpeedAndBias, Vector3<f64>) {
        propagated(
            &self.integration,
            self.delta_t,
            &self.reference,
            self.omega_s,
            pose,
            sb,
        )
    }
}
fn propagated(
    preintegration: &Preintegration,
    dt: f64,
    reference: &SpeedAndBias,
    omega: Vector3<f64>,
    pose: &Transformation,
    sb: &SpeedAndBias,
) -> (Transformation, SpeedAndBias, Vector3<f64>) {
    let rotation = pose.rotation_matrix();
    let gravity = Vector3::new(0.0, 0.0, 9.81);
    let db = sb.fixed_rows::<6>(3) - reference.fixed_rows::<6>(3);
    let position = pose.translation() + sb.fixed_rows::<3>(0) * dt - 0.5 * gravity * dt * dt
        + rotation
            * (preintegration.acc_doubleintegral + preintegration.dp_db_g * db.fixed_rows::<3>(0)
                - preintegration.c_doubleintegral * db.fixed_rows::<3>(3));
    let dq =
        delta_q(&(-preintegration.dalpha_db_g * db.fixed_rows::<3>(0))) * preintegration.delta_q;
    let velocity = sb.fixed_rows::<3>(0) - gravity * dt
        + rotation
            * (preintegration.acc_integral + preintegration.dv_db_g * db.fixed_rows::<3>(0)
                - preintegration.c_integral * db.fixed_rows::<3>(3));
    let mut out = *sb;
    out.fixed_rows_mut::<3>(0).copy_from(&velocity);
    (
        Transformation::new(position, pose.quaternion() * dq),
        out,
        omega,
    )
}
#[derive(Clone, Debug)]
pub struct BatchedLidarPropagator {
    base: Propagator,
    imu_index: usize,
    interpolated: Preintegration,
    omega_interpolated: Vector3<f64>,
}
impl BatchedLidarPropagator {
    #[must_use]
    pub fn new(start: Timestamp, _measurements: &ImuMeasurementDeque) -> Self {
        Self {
            base: Propagator::new(start),
            imu_index: 0,
            interpolated: Preintegration::default(),
            omega_interpolated: Vector3::zeros(),
        }
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn append_to(
        &mut self,
        ms: &ImuMeasurementDeque,
        _pose: &Transformation,
        sb: &SpeedAndBias,
        end: Timestamp,
    ) -> Result<bool, IntegrationError> {
        if ms
            .front()
            .ok_or(IntegrationError::MissingMeasurements)?
            .time_stamp
            > self.base.t_end
        {
            return Err(IntegrationError::StartNotCovered);
        }
        if ms
            .back()
            .ok_or(IntegrationError::MissingMeasurements)?
            .time_stamp
            < end
        {
            return Ok(false);
        }
        let mut time = self.base.t_end;
        self.base.reference = *sb;
        let mut started = false;
        while self.imu_index + 1 < ms.len() {
            let m0 = &ms[self.imu_index];
            let m1 = &ms[self.imu_index + 1];
            let (mut w0, mut w1) = (m0.measurement.gyroscopes, m1.measurement.gyroscopes);
            let (mut a0, mut a1) = (m0.measurement.accelerometers, m1.measurement.accelerometers);
            let mut next = m1.time_stamp;
            let mut dt = seconds(next, time);
            if end < next {
                let interval = seconds(next, m0.time_stamp);
                next = end;
                dt = seconds(next, time);
                let r = dt / interval;
                w1 = (1.0 - r) * w0 + r * w1;
                a1 = (1.0 - r) * a0 + r * a1;
            }
            if dt <= 0.0 {
                self.imu_index += 1;
                continue;
            }
            if !started {
                started = true;
                let r = dt / seconds(next, m0.time_stamp);
                w0 = r * w0 + (1.0 - r) * w1;
                a0 = r * a0 + (1.0 - r) * a1;
            }
            if next == end {
                self.interpolated = self.base.integration.clone();
                self.omega_interpolated = self.interpolated.step(
                    (w0, w1),
                    (a0, a1),
                    dt,
                    sb,
                    None,
                    (false, NoiseMode::None),
                );
                break;
            }
            self.base.omega_s = self.base.integration.step(
                (w0, w1),
                (a0, a1),
                dt,
                sb,
                None,
                (false, NoiseMode::None),
            );
            time = next;
            self.base.t_end = next;
            self.imu_index += 1;
        }
        self.base.delta_t = seconds(end, self.base.t_start);
        Ok(true)
    }
    #[must_use]
    pub fn get_state(
        &self,
        pose: &Transformation,
        sb: &SpeedAndBias,
    ) -> (Transformation, SpeedAndBias, Vector3<f64>) {
        propagated(
            &self.interpolated,
            self.base.delta_t,
            &self.base.reference,
            self.omega_interpolated,
            pose,
            sb,
        )
    }
}
#[derive(Clone, Debug)]
pub struct ConstantVelocityPropagator {
    start: State,
    end: State,
    time_delta: f64,
    r_s0_s1: Vector3<f64>,
}
impl ConstantVelocityPropagator {
    #[must_use]
    pub fn new(start: State, end: State) -> Self {
        let time_delta = seconds(end.timestamp, start.timestamp);
        let r_s0_s1 = (start.t_ws.inverse().matrix() * end.t_ws.matrix())
            .fixed_view::<3, 1>(0, 3)
            .into_owned();
        Self {
            start,
            end,
            time_delta,
            r_s0_s1,
        }
    }
    #[must_use]
    pub fn extrapolate_angle(&self, time: Timestamp) -> Quaternion<f64> {
        log::warn!(
            "Doing angle extrapolations in the constant velocity model, this is not how this should be used"
        );
        let rel = self.start.t_ws.rotation_matrix().transpose() * self.end.t_ws.rotation_matrix();
        let q = normalized_quaternion(quaternion_from_rotation_matrix(&rel));
        let rot = nalgebra::UnitQuaternion::new_unchecked(q);
        let increment = nalgebra::UnitQuaternion::from_scaled_axis(
            rot.scaled_axis() * (seconds(time, self.start.timestamp) / self.time_delta),
        );
        let origin = if time > self.end.timestamp {
            &self.end
        } else {
            &self.start
        };
        quaternion_from_rotation_matrix(
            &(origin.t_ws.rotation_matrix() * increment.to_rotation_matrix().matrix()),
        )
    }
    #[must_use]
    pub fn get_state(&self, time: Timestamp) -> (Transformation, SpeedAndBias, Vector3<f64>) {
        let factor = seconds(time, self.start.timestamp) / self.time_delta;
        let (pose, reference) = if time >= self.start.timestamp && time <= self.end.timestamp {
            let q0 = nalgebra::UnitQuaternion::new_unchecked(normalized_quaternion(
                self.start.t_ws.quaternion(),
            ));
            let q1 = nalgebra::UnitQuaternion::new_unchecked(normalized_quaternion(
                self.end.t_ws.quaternion(),
            ));
            let relative = self.start.t_ws.rotation_matrix().transpose()
                * q0.slerp(&q1, factor).to_rotation_matrix().matrix();
            let increment = Transformation::new(
                self.r_s0_s1 * factor,
                quaternion_from_rotation_matrix(&relative),
            );
            (self.start.t_ws.compose(&increment), &self.start)
        } else if time > self.end.timestamp {
            let q = self.extrapolate_angle(time);
            let relative = self.end.t_ws.rotation_matrix().transpose()
                * crate::kinematics::transformation::raw_rotation_matrix(&q);
            let increment = Transformation::new(
                self.r_s0_s1 * seconds(time, self.end.timestamp) / self.time_delta,
                quaternion_from_rotation_matrix(&relative),
            );
            (self.end.t_ws.compose(&increment), &self.end)
        } else {
            let q = self.extrapolate_angle(time);
            let relative = self.start.t_ws.rotation_matrix().transpose()
                * crate::kinematics::transformation::raw_rotation_matrix(&q);
            let increment = Transformation::new(
                self.r_s0_s1 * factor,
                quaternion_from_rotation_matrix(&relative),
            );
            (self.end.t_ws.compose(&increment), &self.start)
        };
        (pose, reference.speed_and_bias(), reference.omega_s)
    }
    pub fn new_state(&mut self, state: State) {
        if state.timestamp > self.end.timestamp {
            self.start = self.end.clone();
            self.end = state;
        } else if state.timestamp > self.start.timestamp {
            self.end = state;
        } else if state.timestamp < self.start.timestamp {
            self.end = self.start.clone();
            self.start = state;
        }
        self.time_delta = seconds(self.end.timestamp, self.start.timestamp);
        self.r_s0_s1 = (self.start.t_ws.inverse().matrix() * self.end.t_ws.matrix())
            .fixed_view::<3, 1>(0, 3)
            .into_owned();
    }
}
#[derive(Clone, Debug)]
struct NonKeyframeState {
    t_sk_s: Transformation,
    v_sk: Vector3<f64>,
    omega_s_raw: Vector3<f64>,
    timestamp: Timestamp,
    previous: ImuMeasurementDeque,
}
#[derive(Default)]
pub struct Trajectory {
    keyframes: BTreeMap<StateId, State>,
    keyframe_by_id: BTreeMap<StateId, StateId>,
    ids: BTreeSet<StateId>,
    non_keyframes: BTreeMap<StateId, BTreeMap<StateId, NonKeyframeState>>,
    by_time: BTreeMap<i128, StateId>,
    cache: BTreeMap<StateId, BTreeMap<i128, Propagator>>,
    live: Option<Propagator>,
    imu: ImuMeasurementDeque,
}
impl Trajectory {
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    ///
    /// # Panics
    /// Panics if an internally checked trajectory/state entry is absent from its required queue or
    /// map.
    pub fn update(
        &mut self,
        tracking: &TrackingState,
        updates: &BTreeMap<StateId, State>,
        affected: &mut BTreeSet<StateId>,
    ) -> Result<(), IntegrationError> {
        if updates.is_empty() {
            return Ok(());
        }
        let Some(current) = updates.get(&tracking.id) else {
            return Err(IntegrationError::MissingMeasurements);
        };
        self.ids.insert(tracking.id);
        for (&id, state) in updates {
            affected.insert(state.id);
            if let Some(key) = self.keyframes.get_mut(&id) {
                *key = state.clone();
                if let Some(non) = self.non_keyframes.get(&id) {
                    affected.extend(non.keys().copied());
                }
            }
        }
        if tracking.is_keyframe {
            self.keyframes.insert(tracking.id, current.clone());
        } else if let Some(key) = self.keyframes.get(&tracking.current_keyframe_id) {
            let rel = key.t_ws.inverse().compose(&current.t_ws);
            let v_sk = rel.rotate_vector(&current.t_ws.inverse().rotate_vector(&current.v_w));
            self.non_keyframes
                .entry(tracking.current_keyframe_id)
                .or_default()
                .insert(
                    current.id,
                    NonKeyframeState {
                        t_sk_s: rel,
                        v_sk,
                        omega_s_raw: current.omega_s - current.b_g,
                        timestamp: current.timestamp,
                        previous: current.previous_imu_measurements.clone(),
                    },
                );
            self.keyframe_by_id
                .insert(current.id, tracking.current_keyframe_id);
        }
        self.by_time
            .insert(current.timestamp.as_nanosecond(), current.id);
        self.live = Some(Propagator::new(current.timestamp));
        while self
            .imu
            .front()
            .is_some_and(|m| seconds(current.timestamp, m.time_stamp) >= 0.1)
        {
            self.imu.pop_front();
        }
        if let Some(last) = self.imu.back() {
            self.live.as_mut().unwrap().append_to(
                &self.imu,
                &current.t_ws,
                &current.speed_and_bias(),
                last.time_stamp,
            )?;
        }
        Ok(())
    }
    #[must_use]
    pub fn state_by_id(&self, id: StateId) -> Option<State> {
        if let Some(s) = self.keyframes.get(&id) {
            return Some(s.clone());
        }
        let key_id = self.keyframe_by_id.get(&id)?;
        let non = self.non_keyframes.get(key_id)?.get(&id)?;
        let key = self.keyframes.get(key_id)?;
        let t_ws = key.t_ws.compose(&non.t_sk_s);
        let v_w = key.t_ws.rotate_vector(&non.v_sk);
        let mut out = State::new(
            t_ws,
            v_w,
            (key.b_g, key.b_a),
            non.omega_s_raw - key.b_g,
            non.timestamp,
            StateId::default(),
            false,
        );
        out.previous_imu_measurements.clone_from(&non.previous);
        Some(out)
    }
    #[must_use]
    pub fn state_ids(&self) -> &BTreeSet<StateId> {
        &self.ids
    }
    #[must_use]
    pub fn keyframe_states(&self) -> &BTreeMap<StateId, State> {
        &self.keyframes
    }
    pub fn clear_cache(&mut self, id: StateId) -> bool {
        if let Some(c) = self.cache.get_mut(&id) {
            c.clear();
            true
        } else {
            false
        }
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn add_imu_measurement(
        &mut self,
        time: Timestamp,
        alpha: Vector3<f64>,
        omega: Vector3<f64>,
    ) -> Result<Option<State>, IntegrationError> {
        self.imu.push_back(ImuMeasurement::new(
            time,
            ImuSensorReadings::new(omega, alpha),
            -1,
        ));
        let Some(&id) = self.ids.last() else {
            return Ok(None);
        };
        let Some(base) = self.state_by_id(id) else {
            return Ok(None);
        };
        if time < base.timestamp {
            return Ok(None);
        }
        let Some(live) = self.live.as_mut() else {
            return Ok(None);
        };
        let sb = base.speed_and_bias();
        live.append_to(&self.imu, &base.t_ws, &sb, time)?;
        let (p, s, w) = live.get_state(&base.t_ws, &sb);
        let mut output = base.propagated(p, s, w, time);
        output.previous_imu_measurements.clear();
        Ok(Some(output))
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn get_state(&mut self, time: Timestamp) -> Result<Option<State>, IntegrationError> {
        let ns = time.as_nanosecond();
        if let Some(&id) = self.by_time.get(&ns) {
            return Ok(self.state_by_id(id));
        }
        let Some((&next_ns, &next_id)) = self.by_time.range(ns..).next() else {
            return Ok(None);
        };
        let Some((&_, &prev_id)) = self.by_time.range(..next_ns).next_back() else {
            return Ok(None);
        };
        let (Some(s0), Some(s1)) = (self.state_by_id(prev_id), self.state_by_id(next_id)) else {
            return Ok(None);
        };
        let sb = s0.speed_and_bias();
        let (p, s, w) = if let Some(cache) = self.cache.get(&prev_id).and_then(|c| c.get(&ns)) {
            cache.get_state(&s0.t_ws, &sb)
        } else if s1.previous_imu_measurements.is_empty() {
            ConstantVelocityPropagator::new(s0.clone(), s1).get_state(time)
        } else {
            let mut p = Propagator::new(s0.timestamp);
            p.append_to(&s1.previous_imu_measurements, &s0.t_ws, &sb, time)?;
            let result = p.get_state(&s0.t_ws, &sb);
            self.cache.entry(prev_id).or_default().insert(ns, p);
            result
        };
        Ok(Some(s0.propagated(p, s, w, time)))
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    ///
    /// # Panics
    /// Panics if an internally checked trajectory/state entry is absent from its required queue or
    /// map.
    pub fn get_state_poses(
        &self,
        times: &[Timestamp],
        poses: &mut Vec<Matrix4<f64>>,
    ) -> Result<bool, IntegrationError> {
        let Some(first) = times.first() else {
            return Ok(false);
        };
        let ordered: Vec<_> = self.by_time.iter().collect();
        let Some(mut next_index) = ordered
            .iter()
            .position(|(t, _)| **t >= first.as_nanosecond())
        else {
            return Ok(false);
        };
        if next_index == 0 {
            return Ok(false);
        }
        let mut i = next_index - 1;
        let Some(mut s0) = self.state_by_id(*ordered[i].1) else {
            return Ok(false);
        };
        let Some(mut s1) = self.state_by_id(*ordered[next_index].1) else {
            return Ok(false);
        };
        if *first < s0.timestamp {
            if i == 0 {
                return Ok(false);
            }
            s1 = s0;
            i -= 1;
            s0 = match self.state_by_id(*ordered[i].1) {
                Some(s) => s,
                None => return Ok(false),
            };
        }
        if times.last().unwrap() > &s1.timestamp {
            s0 = s1;
            i += 2;
            if i >= ordered.len() {
                return Ok(false);
            }
            s1 = match self.state_by_id(*ordered[i].1) {
                Some(s) => s,
                None => return Ok(false),
            };
        }
        next_index = i + 1;
        let sb = s0.speed_and_bias();
        let mut batched = BatchedLidarPropagator::new(s0.timestamp, &s1.previous_imu_measurements);
        let constant = ConstantVelocityPropagator::new(s0.clone(), s1.clone());
        let mut previous = Timestamp::UNIX_EPOCH;
        let mut last_pose = Transformation::default().matrix();
        for &time in times {
            if time != previous {
                if next_index < ordered.len() && *ordered[next_index].0 == time.as_nanosecond() {
                    let Some(state) = self.state_by_id(*ordered[next_index].1) else {
                        return Ok(false);
                    };
                    poses.push(state.t_ws.matrix());
                    next_index += 1;
                    if next_index == ordered.len() {
                        break;
                    }
                    continue;
                }
                let (p, _, _) = if s1.previous_imu_measurements.is_empty() {
                    constant.get_state(time)
                } else {
                    batched.append_to(&s1.previous_imu_measurements, &s0.t_ws, &sb, time)?;
                    batched.get_state(&s0.t_ws, &sb)
                };
                last_pose = p.matrix();
                previous = time;
            }
            poses.push(last_pose);
        }
        Ok(true)
    }
}
pub type ImageMap = BTreeMap<usize, Arc<DynamicMatrix>>;
pub type OptimisedGraphCallback = Arc<
    dyn Fn(&State, &TrackingState, Arc<BTreeMap<StateId, State>>, Arc<MapPointVector>)
        + Send
        + Sync,
>;
pub trait ViInterface {
    fn add_images(&mut self, stamp: Timestamp, images: &ImageMap, depth: &ImageMap) -> bool;
    fn add_imu_measurement(
        &mut self,
        stamp: Timestamp,
        alpha: &Vector3<f64>,
        omega: &Vector3<f64>,
    ) -> bool;
    fn add_depth_measurement(
        &mut self,
        stamp: Timestamp,
        depth: &DynamicMatrix,
        sigma: Option<&DynamicMatrix>,
    ) -> bool;
    fn add_lidar_measurement(&mut self, stamp: Timestamp, ray: &Vector3<f64>) -> bool;
    fn set_optimised_graph_callback(&mut self, callback: OptimisedGraphCallback);
    fn set_blocking(&mut self, blocking: bool);
    fn display(&mut self, images: &mut BTreeMap<String, Arc<DynamicMatrix>>);
}
#[derive(Default)]
pub struct ViInterfaceFiles {
    pub optimised_graph_callback: Option<OptimisedGraphCallback>,
    pub csv_imu_file: Option<Box<dyn Write + Send>>,
    pub csv_tracks_files: BTreeMap<usize, Box<dyn Write + Send>>,
}
impl ViInterfaceFiles {
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn set_imu_csv_file(&mut self, writer: impl Write + Send + 'static) -> io::Result<bool> {
        self.csv_imu_file = Some(Box::new(writer));
        self.write_imu_csv_description()
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn set_imu_csv_path(&mut self, path: &Path) -> io::Result<bool> {
        self.set_imu_csv_file(File::create(path)?)
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn set_tracks_csv_file(
        &mut self,
        camera: usize,
        writer: impl Write + Send + 'static,
    ) -> io::Result<bool> {
        self.csv_tracks_files.insert(camera, Box::new(writer));
        self.write_tracks_csv_description(camera)?;
        Ok(true)
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn set_tracks_csv_path(&mut self, camera: usize, path: &Path) -> io::Result<bool> {
        self.set_tracks_csv_file(camera, File::create(path)?)
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn write_imu_csv_description(&mut self) -> io::Result<bool> {
        let Some(w) = self.csv_imu_file.as_mut() else {
            return Ok(false);
        };
        writeln!(
            w,
            "timestamp, omega_tilde_WS_S_x, omega_tilde_WS_S_y, omega_tilde_WS_S_z, a_tilde_WS_S_x, a_tilde_WS_S_y, a_tilde_WS_S_z"
        )?;
        w.flush()?;
        Ok(true)
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn write_tracks_csv_description(&mut self, camera: usize) -> io::Result<bool> {
        let Some(w) = self.csv_tracks_files.get_mut(&camera) else {
            return Ok(false);
        };
        writeln!(
            w,
            "timestamp, landmark_id, z_tilde_x, z_tilde_y, z_tilde_stdev, descriptor"
        )?;
        w.flush()?;
        Ok(false)
    }
}
