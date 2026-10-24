/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENSE file for details
 */
//! IMU preintegration, propagation, analytic factors and noise derivatives from `ImuError.cpp`.
//! Timestamps are differenced in integer nanoseconds before conversion to seconds.
use super::pose_local_parameterization::PoseManifold;
use crate::{
    ceres::pseudo_inverse::PseudoInverse,
    config::parameters::ImuParameters,
    frames::frame_typedefs::SpeedAndBias,
    kinematics::{
        operators::{cross_mx, oplus, plus},
        transformation::{
            Transformation, delta_q, quaternion_inverse, raw_rotation_matrix, right_jacobian,
        },
    },
    sensors::measurements::ImuMeasurementDeque,
};
use jiff::Timestamp;
use nalgebra::{DMatrix, Matrix3, Quaternion, SMatrix, SVector, Vector3};
use std::sync::atomic::{AtomicBool, Ordering};

pub type Covariance = SMatrix<f64, 15, 15>;
pub type Information = Covariance;
pub type Jacobian = Covariance;
pub static REDO_PROPAGATION_ALWAYS: AtomicBool = AtomicBool::new(false);
pub(crate) fn seconds(a: Timestamp, b: Timestamp) -> f64 {
    (a.as_nanosecond() - b.as_nanosecond()) as f64 * 1e-9
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntegrationError {
    MissingMeasurements,
    StartNotCovered,
    EndNotCovered,
    NoIntegrationSteps,
    InvalidCovariance,
}

/// Identity increments and covariance basis matrices before the first sample.
#[derive(Clone, Debug)]
pub(crate) struct Preintegration {
    pub delta_q: Quaternion<f64>,
    pub c_integral: Matrix3<f64>,
    pub c_doubleintegral: Matrix3<f64>,
    pub acc_integral: Vector3<f64>,
    pub acc_doubleintegral: Vector3<f64>,
    pub cross: Matrix3<f64>,
    pub dalpha_db_g: Matrix3<f64>,
    pub dv_db_g: Matrix3<f64>,
    pub dp_db_g: Matrix3<f64>,
    pub p_delta: Covariance,
    pub dp_dsigma: [Covariance; 4],
    pub reference: SpeedAndBias,
}
impl Default for Preintegration {
    fn default() -> Self {
        Self {
            delta_q: Quaternion::identity(),
            c_integral: Matrix3::zeros(),
            c_doubleintegral: Matrix3::zeros(),
            acc_integral: Vector3::zeros(),
            acc_doubleintegral: Vector3::zeros(),
            cross: Matrix3::zeros(),
            dalpha_db_g: Matrix3::zeros(),
            dv_db_g: Matrix3::zeros(),
            dp_db_g: Matrix3::zeros(),
            p_delta: Covariance::zeros(),
            dp_dsigma: [Covariance::zeros(); 4],
            reference: SpeedAndBias::zeros(),
        }
    }
}
#[derive(Clone, Copy)]
pub(crate) enum NoiseMode {
    None,
    Basis,
    Direct,
}
impl Preintegration {
    /// The source propagation and motion compensator use dt*C1 for d(alpha)/db;
    /// the nonlinear IMU/GPS preintegrators include the right Jacobian.
    pub(crate) fn step(
        &mut self,
        (w0, w1): (Vector3<f64>, Vector3<f64>),
        (a0, a1): (Vector3<f64>, Vector3<f64>),
        dt: f64,
        sb: &SpeedAndBias,
        params: Option<&ImuParameters>,
        (right_alpha, noise): (bool, NoiseMode),
    ) -> Vector3<f64> {
        let angular_velocity = 0.5 * (w0 + w1) - sb.fixed_rows::<3>(3);
        let dq = delta_q(&(angular_velocity * dt));
        let q1 = self.delta_q * dq;
        let rotation = raw_rotation_matrix(&self.delta_q);
        let c1 = raw_rotation_matrix(&q1);
        let acceleration = 0.5 * (a0 + a1) - sb.fixed_rows::<3>(6);
        let ci1 = self.c_integral + 0.5 * (rotation + c1) * dt;
        let ai1 = self.acc_integral + 0.5 * (rotation + c1) * acceleration * dt;
        let cross1 = raw_rotation_matrix(&quaternion_inverse(&dq)) * self.cross
            + right_jacobian(&(angular_velocity * dt)) * dt;
        let ax = cross_mx(&acceleration);
        let ax_cross = rotation * ax * self.cross + c1 * ax * cross1;
        let dv1 = self.dv_db_g + 0.5 * dt * ax_cross;
        let mut transition = Covariance::identity();
        transition
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&-cross_mx(
                &(self.acc_integral * dt + 0.25 * (rotation + c1) * acceleration * dt * dt),
            ));
        transition
            .fixed_view_mut::<3, 3>(0, 6)
            .copy_from(&(Matrix3::identity() * dt));
        transition
            .fixed_view_mut::<3, 3>(0, 9)
            .copy_from(&(dt * self.dv_db_g + 0.25 * dt * dt * ax_cross));
        // The positive quadratic term is the exact source expression.
        transition
            .fixed_view_mut::<3, 3>(0, 12)
            .copy_from(&(-self.c_integral * dt + 0.25 * (rotation + c1) * dt * dt));
        transition
            .fixed_view_mut::<3, 3>(3, 9)
            .copy_from(&(-dt * c1));
        transition
            .fixed_view_mut::<3, 3>(6, 3)
            .copy_from(&-cross_mx(&(0.5 * (rotation + c1) * acceleration * dt)));
        transition
            .fixed_view_mut::<3, 3>(6, 9)
            .copy_from(&(0.5 * dt * ax_cross));
        transition
            .fixed_view_mut::<3, 3>(6, 12)
            .copy_from(&(-0.5 * (rotation + c1) * dt));
        self.propagate_noise(params, noise, (w0, w1), (a0, a1), dt, &transition);
        self.c_doubleintegral += self.c_integral * dt + 0.25 * (rotation + c1) * dt * dt;
        self.acc_doubleintegral +=
            self.acc_integral * dt + 0.25 * (rotation + c1) * acceleration * dt * dt;
        self.dalpha_db_g += if right_alpha {
            c1 * right_jacobian(&(angular_velocity * dt)) * dt
        } else {
            dt * c1
        };
        self.dp_db_g += dt * self.dv_db_g + 0.25 * dt * dt * ax_cross;
        self.delta_q = q1;
        self.c_integral = ci1;
        self.acc_integral = ai1;
        self.cross = cross1;
        self.dv_db_g = dv1;
        angular_velocity
    }
    fn propagate_noise(
        &mut self,
        params: Option<&ImuParameters>,
        noise: NoiseMode,
        (w0, w1): (Vector3<f64>, Vector3<f64>),
        (a0, a1): (Vector3<f64>, Vector3<f64>),
        dt: f64,
        transition: &Covariance,
    ) {
        if let Some(p) = params {
            let saturated =
                |first_reading: &Vector3<f64>, second_reading: &Vector3<f64>, limit: f64| {
                    first_reading
                        .iter()
                        .chain(second_reading.iter())
                        .any(|component| component.abs() > limit)
                };
            let gm = if saturated(&w0, &w1, p.g_max) {
                log::warn!("gyr saturation");
                100.0
            } else {
                1.0
            };
            let am = if saturated(&a0, &a1, p.a_max) {
                log::warn!("acc saturation");
                100.0
            } else {
                1.0
            };
            match noise {
                NoiseMode::None => {}
                NoiseMode::Basis => {
                    let mut noise_basis = [Covariance::zeros(); 4];
                    noise_basis[0]
                        .fixed_view_mut::<3, 3>(3, 3)
                        .copy_from(&(Matrix3::identity() * (gm * dt)));
                    noise_basis[1]
                        .fixed_view_mut::<3, 3>(0, 0)
                        .copy_from(&(Matrix3::identity() * (0.5 * dt * dt * dt * am * am * am)));
                    noise_basis[1]
                        .fixed_view_mut::<3, 3>(6, 6)
                        .copy_from(&(Matrix3::identity() * (am * dt)));
                    noise_basis[2]
                        .fixed_view_mut::<3, 3>(9, 9)
                        .copy_from(&(Matrix3::identity() * dt));
                    noise_basis[3]
                        .fixed_view_mut::<3, 3>(12, 12)
                        .copy_from(&(Matrix3::identity() * dt));
                    for (dp, noise_basis) in self.dp_dsigma.iter_mut().zip(noise_basis) {
                        *dp = *transition * (*dp) * transition.transpose() + noise_basis;
                    }
                }
                NoiseMode::Direct => {
                    self.p_delta = *transition * self.p_delta * transition.transpose();
                    let sg = gm * p.sigma_g_c;
                    let sa = am * p.sigma_a_c;
                    let sv = dt * sa * p.sigma_a_c;
                    for (offset, value) in [
                        (0, 0.5 * dt * dt * sv),
                        (3, dt * sg * sg),
                        (6, sv),
                        (9, dt * p.sigma_gw_c * p.sigma_gw_c),
                        (12, dt * p.sigma_aw_c * p.sigma_aw_c),
                    ] {
                        for axis in 0..3 {
                            self.p_delta[(offset + axis, offset + axis)] += value;
                        }
                    }
                }
            }
        }
    }

    pub(crate) fn finish_covariance(&mut self, p: &ImuParameters) {
        for dp in &mut self.dp_dsigma {
            *dp = 0.5 * (*dp) + 0.5 * dp.transpose();
        }
        self.reweight_covariance(p);
    }
    pub(crate) fn reweight_covariance(&mut self, p: &ImuParameters) {
        self.p_delta = self.dp_dsigma[0] * p.sigma_g_c * p.sigma_g_c;
        self.p_delta += self.dp_dsigma[1] * p.sigma_a_c * p.sigma_a_c;
        self.p_delta += self.dp_dsigma[2] * p.sigma_gw_c * p.sigma_gw_c;
        self.p_delta += self.dp_dsigma[3] * p.sigma_aw_c * p.sigma_aw_c;
    }
    pub(crate) fn integrate(
        &mut self,
        measurements: &ImuMeasurementDeque,
        params: Option<&ImuParameters>,
        sb: &SpeedAndBias,
        (start, end): (Timestamp, Timestamp),
        right_alpha: bool,
        noise: NoiseMode,
    ) -> Result<(usize, f64), IntegrationError> {
        let first = measurements
            .front()
            .ok_or(IntegrationError::MissingMeasurements)?;
        let last = measurements
            .back()
            .ok_or(IntegrationError::MissingMeasurements)?;
        if first.time_stamp > start {
            return Err(IntegrationError::StartNotCovered);
        }
        if last.time_stamp < end {
            return Err(IntegrationError::EndNotCovered);
        }
        let mut time = start;
        let mut started = false;
        let mut steps = 0;
        let mut total = 0.0;
        // Only adjacent pairs have both readings defined in the source domain.
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
            self.step((w0, w1), (a0, a1), dt, sb, params, (right_alpha, noise));
            total += dt;
            time = next;
            steps += 1;
            if next == end {
                break;
            }
        }
        Ok((steps, total))
    }
    pub(crate) fn propagation_jacobian(&self, c: &Matrix3<f64>, dt: f64) -> Jacobian {
        let mut f = Jacobian::identity();
        f.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&-cross_mx(&(c * self.acc_doubleintegral)));
        f.fixed_view_mut::<3, 3>(0, 6)
            .copy_from(&(Matrix3::identity() * dt));
        f.fixed_view_mut::<3, 3>(0, 9)
            .copy_from(&(c * self.dp_db_g));
        f.fixed_view_mut::<3, 3>(0, 12)
            .copy_from(&(-c * self.c_doubleintegral));
        f.fixed_view_mut::<3, 3>(3, 9)
            .copy_from(&(-c * self.dalpha_db_g));
        f.fixed_view_mut::<3, 3>(6, 3)
            .copy_from(&-cross_mx(&(c * self.acc_integral)));
        f.fixed_view_mut::<3, 3>(6, 9)
            .copy_from(&(c * self.dv_db_g));
        f.fixed_view_mut::<3, 3>(6, 12)
            .copy_from(&(-c * self.c_integral));
        f
    }
}
pub(crate) fn world_covariance(p: &Covariance, c: &Matrix3<f64>) -> Covariance {
    let mut t = Covariance::identity();
    for offset in [0, 3, 6] {
        t.fixed_view_mut::<3, 3>(offset, offset).copy_from(c);
    }
    t * p * t.transpose()
}
fn sqrt_information(p: &Covariance) -> Information {
    let a = DMatrix::from_column_slice(15, 15, p.as_slice());
    let mut u = DMatrix::zeros(15, 15);
    PseudoInverse::symm_sqrt_u(&a, &mut u, f64::EPSILON, None);
    Information::from_column_slice(u.as_slice())
}

pub trait ImuErrorBase {
    fn t0(&self) -> Timestamp;
    fn t1(&self) -> Timestamp;
    fn set_t0(&mut self, time: Timestamp);
    fn set_t1(&mut self, time: Timestamp);
    fn residual_dim(&self) -> usize {
        15
    }
    fn parameter_blocks(&self) -> usize {
        4
    }
    fn parameter_block_dim(&self, index: usize) -> Option<usize> {
        [7, 9, 7, 9].get(index).copied()
    }
    fn type_info(&self) -> &'static str;
}
#[derive(Clone, Debug)]
pub struct ImuError {
    pub imu_parameters: ImuParameters,
    pub imu_measurements: ImuMeasurementDeque,
    pub t0: Timestamp,
    pub t1: Timestamp,
    pub(crate) preintegration: Preintegration,
    pub information: Option<Information>,
    pub square_root_information: Option<Information>,
    pub redo: bool,
    pub redo_counter: usize,
}
#[derive(Clone, Debug)]
pub struct ImuEvaluation {
    pub residual: SVector<f64, 15>,
    pub pose0: SMatrix<f64, 15, 7>,
    pub speed_bias0: SMatrix<f64, 15, 9>,
    pub pose1: SMatrix<f64, 15, 7>,
    pub speed_bias1: SMatrix<f64, 15, 9>,
    pub pose0_minimal: SMatrix<f64, 15, 6>,
    pub pose1_minimal: SMatrix<f64, 15, 6>,
    pub speed_bias0_minimal: SMatrix<f64, 15, 9>,
    pub speed_bias1_minimal: SMatrix<f64, 15, 9>,
    pub integration_success: bool,
}
impl ImuErrorBase for ImuError {
    fn t0(&self) -> Timestamp {
        self.t0
    }
    fn t1(&self) -> Timestamp {
        self.t1
    }
    fn set_t0(&mut self, t: Timestamp) {
        self.t0 = t;
    }
    fn set_t1(&mut self, t: Timestamp) {
        self.t1 = t;
    }
    fn type_info(&self) -> &'static str {
        "ImuError"
    }
}
impl ImuError {
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn new(
        imu_measurements: ImuMeasurementDeque,
        imu_parameters: ImuParameters,
        t0: Timestamp,
        t1: Timestamp,
    ) -> Result<Self, IntegrationError> {
        let first = imu_measurements
            .front()
            .ok_or(IntegrationError::MissingMeasurements)?;
        let last = imu_measurements
            .back()
            .ok_or(IntegrationError::MissingMeasurements)?;
        if first.time_stamp > t0 {
            return Err(IntegrationError::StartNotCovered);
        }
        if last.time_stamp < t1 {
            return Err(IntegrationError::EndNotCovered);
        }
        // Weighting is assigned by preintegration, before any evaluation.
        Ok(Self {
            imu_parameters,
            imu_measurements,
            t0,
            t1,
            preintegration: Preintegration::default(),
            information: None,
            square_root_information: None,
            redo: true,
            redo_counter: 0,
        })
    }
    pub fn set_imu_measurements(&mut self, m: ImuMeasurementDeque) {
        self.imu_measurements = m;
    }
    pub fn set_imu_parameters(&mut self, p: ImuParameters) {
        self.imu_parameters = p;
        self.preintegration
            .reweight_covariance(&self.imu_parameters);
        self.update_information();
    }
    pub fn sync_from(&mut self, other: &Self) {
        *self = other.clone();
    }
    fn update_information(&mut self) {
        let u = sqrt_information(&self.preintegration.p_delta);
        self.square_root_information = Some(u);
        self.information = Some(u.transpose() * u);
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn redo_preintegration(
        &mut self,
        _pose: &Transformation,
        sb: &SpeedAndBias,
    ) -> Result<usize, IntegrationError> {
        self.preintegration = Preintegration::default();
        let (steps, _) = self.preintegration.integrate(
            &self.imu_measurements,
            Some(&self.imu_parameters),
            sb,
            (self.t0, self.t1),
            true,
            NoiseMode::Basis,
        )?;
        self.preintegration.reference = *sb;
        self.preintegration.finish_covariance(&self.imu_parameters);
        self.update_information();
        Ok(steps)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn append(
        &mut self,
        _pose: &Transformation,
        sb: &SpeedAndBias,
        measurements: &ImuMeasurementDeque,
        end: Timestamp,
    ) -> Result<usize, IntegrationError> {
        let first = measurements
            .front()
            .ok_or(IntegrationError::MissingMeasurements)?;
        let last = measurements
            .back()
            .ok_or(IntegrationError::MissingMeasurements)?;
        if first.time_stamp > self.t1 {
            return Err(IntegrationError::StartNotCovered);
        }
        if last.time_stamp < end {
            return Err(IntegrationError::EndNotCovered);
        }
        let previous_last = self
            .imu_measurements
            .back()
            .ok_or(IntegrationError::MissingMeasurements)?
            .time_stamp;
        self.imu_measurements.extend(
            measurements
                .iter()
                .filter(|m| m.time_stamp > previous_last)
                .cloned(),
        );
        let start = self.t1;
        self.t1 = end;
        let (steps, _) = self.preintegration.integrate(
            measurements,
            Some(&self.imu_parameters),
            sb,
            (start, end),
            true,
            NoiseMode::Basis,
        )?;
        // append intentionally leaves the stored reference unchanged.
        self.preintegration.finish_covariance(&self.imu_parameters);
        self.update_information();
        Ok(steps)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn propagation(
        measurements: &ImuMeasurementDeque,
        params: &ImuParameters,
        pose: &mut Transformation,
        sb: &mut SpeedAndBias,
        (start, end): (Timestamp, Timestamp),
        (covariance, jacobian): (Option<&mut Covariance>, Option<&mut Jacobian>),
    ) -> Result<usize, IntegrationError> {
        let mut p = Preintegration::default();
        let noise = if covariance.is_some() {
            NoiseMode::Direct
        } else {
            NoiseMode::None
        };
        let (steps, dt) =
            p.integrate(measurements, Some(params), sb, (start, end), false, noise)?;
        let c = pose.rotation_matrix();
        let g = Vector3::new(0.0, 0.0, params.g);
        pose.set(
            pose.translation() + sb.fixed_rows::<3>(0) * dt + c * p.acc_doubleintegral
                - 0.5 * g * dt * dt,
            pose.quaternion() * p.delta_q,
        );
        let v = sb.fixed_rows::<3>(0).into_owned() + c * p.acc_integral - g * dt;
        sb.fixed_rows_mut::<3>(0).copy_from(&v);
        if let Some(j) = jacobian {
            *j = p.propagation_jacobian(&c, dt);
        }
        if let Some(cov) = covariance {
            *cov = world_covariance(&p.p_delta, &c);
        }
        Ok(steps)
    }
    pub fn init_pose(measurements: &ImuMeasurementDeque, pose: &mut Transformation) -> bool {
        pose.set_identity();
        if measurements.is_empty() {
            return false;
        }
        let a = measurements
            .iter()
            .fold(Vector3::zeros(), |s, m| s + m.measurement.accelerometers)
            / measurements.len() as f64;
        let normalize = |x: Vector3<f64>| {
            let n = x.norm_squared();
            if n > 0.0 { x / n.sqrt() } else { x }
        };
        let e = normalize(a);
        let z = Vector3::new(0.0, 0.0, 1.0);
        let angle = z.dot(&e).acos();
        let mut increment = SVector::<f64, 6>::zeros();
        increment
            .fixed_rows_mut::<3>(3)
            .copy_from(&(normalize(z.cross(&e)) * angle));
        pose.oplus(&-increment);
        true
    }
    fn unweighted(
        &self,
        p0: &Transformation,
        sb0: &SpeedAndBias,
        p1: &Transformation,
        sb1: &SpeedAndBias,
        db: &SVector<f64, 6>,
        jacobians: Option<(&mut Jacobian, &mut Jacobian)>,
    ) -> SVector<f64, 15> {
        let p = &self.preintegration;
        let ct = p0.rotation_matrix().transpose();
        let dt = seconds(self.t1, self.t0);
        let g = Vector3::new(0.0, 0.0, self.imu_parameters.g);
        let dp =
            p0.translation() - p1.translation() + sb0.fixed_rows::<3>(0) * dt - 0.5 * g * dt * dt;
        let dv = sb0.fixed_rows::<3>(0) - sb1.fixed_rows::<3>(0) - g * dt;
        let dq = delta_q(&(-p.dalpha_db_g * db.fixed_rows::<3>(0))) * p.delta_q;
        let q0 = p0.quaternion();
        let q1i = quaternion_inverse(&p1.quaternion());
        let mut position_bias = SMatrix::<f64, 3, 6>::zeros();
        position_bias
            .fixed_columns_mut::<3>(0)
            .copy_from(&p.dp_db_g);
        position_bias
            .fixed_columns_mut::<3>(3)
            .copy_from(&-p.c_doubleintegral);
        let mut velocity_bias = SMatrix::<f64, 3, 6>::zeros();
        velocity_bias
            .fixed_columns_mut::<3>(0)
            .copy_from(&p.dv_db_g);
        velocity_bias
            .fixed_columns_mut::<3>(3)
            .copy_from(&-p.c_integral);
        if let Some((f0, f1)) = jacobians {
            *f0 = Jacobian::identity();
            *f1 = -Jacobian::identity();
            f0.fixed_view_mut::<3, 3>(0, 0).copy_from(&ct);
            f0.fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&(ct * cross_mx(&dp)));
            f0.fixed_view_mut::<3, 3>(0, 6).copy_from(&(ct * dt));
            f0.fixed_view_mut::<3, 6>(0, 9).copy_from(&position_bias);
            f0.fixed_view_mut::<3, 3>(3, 3)
                .copy_from(&(plus(&(dq * q1i)) * oplus(&q0)).fixed_view::<3, 3>(0, 0));
            f0.fixed_view_mut::<3, 3>(3, 9).copy_from(
                &((oplus(&(q1i * q0)) * oplus(&dq)).fixed_view::<3, 3>(0, 0) * (-p.dalpha_db_g)),
            );
            f0.fixed_view_mut::<3, 3>(6, 3)
                .copy_from(&(ct * cross_mx(&dv)));
            f0.fixed_view_mut::<3, 3>(6, 6).copy_from(&ct);
            f0.fixed_view_mut::<3, 6>(6, 9).copy_from(&velocity_bias);
            f1.fixed_view_mut::<3, 3>(0, 0).copy_from(&-ct);
            f1.fixed_view_mut::<3, 3>(3, 3)
                .copy_from(&-(plus(&dq) * oplus(&q0) * plus(&q1i)).fixed_view::<3, 3>(0, 0));
            f1.fixed_view_mut::<3, 3>(6, 6).copy_from(&-ct);
        }
        let mut e = SVector::<f64, 15>::zeros();
        e.fixed_rows_mut::<3>(0)
            .copy_from(&(ct * dp + p.acc_doubleintegral + position_bias * db));
        e.fixed_rows_mut::<3>(3)
            .copy_from(&(2.0 * (dq * (q1i * q0)).coords.fixed_rows::<3>(0)));
        e.fixed_rows_mut::<3>(6)
            .copy_from(&(ct * dv + p.acc_integral + velocity_bias * db));
        e.fixed_rows_mut::<6>(9)
            .copy_from(&(sb0.fixed_rows::<6>(3) - sb1.fixed_rows::<6>(3)));
        e
    }
    fn evaluate_unweighted(
        &mut self,
        pose0: &SVector<f64, 7>,
        sb0: &SpeedAndBias,
        pose1: &SVector<f64, 7>,
        sb1: &SpeedAndBias,
        jacobians: Option<(&mut Jacobian, &mut Jacobian)>,
    ) -> Result<(SVector<f64, 15>, bool), IntegrationError> {
        let p0 = pose_from_parameters(pose0);
        let p1 = pose_from_parameters(pose1);
        let mut db = sb0.fixed_rows::<6>(3) - self.preintegration.reference.fixed_rows::<6>(3);
        self.redo |= db.fixed_rows::<3>(0).norm() > 0.0003;
        let mut success = true;
        if (self.redo
            && (self.imu_measurements.len() < 50
                || REDO_PROPAGATION_ALWAYS.load(Ordering::Relaxed)))
            || self.redo_counter == 0
        {
            let steps = self.redo_preintegration(&p0, sb0)?;
            success = steps != 0;
            if !success {
                log::warn!("IMU error term computation failed: 0 steps integrated -- disable.");
            }
            self.redo_counter += 1;
            db.fill(0.0);
            self.redo = false;
        }
        let mut e = self.unweighted(&p0, sb0, &p1, sb1, &db, jacobians);
        if !success {
            e.fill(0.0);
        }
        Ok((e, success))
    }
    /// Evaluates the weighted residual and updates the preintegration cache.
    /// # Errors
    /// Returns an error if integration fails or the covariance is invalid.
    pub fn evaluate_residual(
        &mut self,
        pose0: &SVector<f64, 7>,
        sb0: &SpeedAndBias,
        pose1: &SVector<f64, 7>,
        sb1: &SpeedAndBias,
    ) -> Result<SVector<f64, 15>, IntegrationError> {
        let (e, _) = self.evaluate_unweighted(pose0, sb0, pose1, sb1, None)?;
        Ok(self
            .square_root_information
            .ok_or(IntegrationError::InvalidCovariance)?
            * e)
    }
    /// Mutable access serializes cache updates; callers can share behind a mutex.
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn evaluate(
        &mut self,
        pose0: &SVector<f64, 7>,
        sb0: &SpeedAndBias,
        pose1: &SVector<f64, 7>,
        sb1: &SpeedAndBias,
    ) -> Result<ImuEvaluation, IntegrationError> {
        let mut f0 = Jacobian::zeros();
        let mut f1 = Jacobian::zeros();
        let (e, success) =
            self.evaluate_unweighted(pose0, sb0, pose1, sb1, Some((&mut f0, &mut f1)))?;
        let u = self
            .square_root_information
            .ok_or(IntegrationError::InvalidCovariance)?;
        let j0 = u * f0.fixed_columns::<6>(0);
        let j2 = u * f1.fixed_columns::<6>(0);
        let j1 = u * f0.fixed_columns::<9>(6);
        let j3 = u * f1.fixed_columns::<9>(6);
        Ok(ImuEvaluation {
            residual: u * e,
            pose0: j0 * PoseManifold::minus_jacobian(pose0),
            pose1: j2 * PoseManifold::minus_jacobian(pose1),
            speed_bias0: j1,
            speed_bias1: j3,
            pose0_minimal: if success { j0 } else { SMatrix::zeros() },
            pose1_minimal: if success { j2 } else { SMatrix::zeros() },
            speed_bias0_minimal: if success { j1 } else { SMatrix::zeros() },
            speed_bias1_minimal: if success { j3 } else { SMatrix::zeros() },
            integration_success: success,
        })
    }
    /// Derivatives use the four variance basis matrices exactly as in the source.
    /// This entry does not trigger preintegration.
    #[must_use]
    pub fn evaluate_with_sigma_gradient_and_hessian(
        &self,
        pose0: &SVector<f64, 7>,
        sb0: &SpeedAndBias,
        pose1: &SVector<f64, 7>,
        sb1: &SpeedAndBias,
    ) -> Option<(f64, SVector<f64, 4>, SMatrix<f64, 4, 4>)> {
        let information = self.information?;
        let db = sb0.fixed_rows::<6>(3) - self.preintegration.reference.fixed_rows::<6>(3);
        let e = self.unweighted(
            &pose_from_parameters(pose0),
            sb0,
            &pose_from_parameters(pose1),
            sb1,
            &db,
            None,
        );
        let inf_e = information * e;
        let cost_sum = e.dot(&inf_e) + self.preintegration.p_delta.determinant().ln();
        let cost = 0.5 * cost_sum;
        let mut gradient = SVector::<f64, 4>::zeros();
        let mut hessian = SMatrix::<f64, 4, 4>::zeros();
        for j in 0..4 {
            let dj = self.preintegration.dp_dsigma[j];
            let y = information * dj;
            let gradient_sum = -inf_e.dot(&(dj * inf_e)) + y.trace();
            gradient[j] = 0.5 * gradient_sum;
            let hessian_sum = 2.0 * (dj * inf_e).dot(&(information * (dj * inf_e)))
                + (-y * information * dj).trace();
            hessian[(j, j)] = 0.5 * hessian_sum;
            for k in 0..j {
                let dk = self.preintegration.dp_dsigma[k];
                let t0 = (dj * inf_e).dot(&(information * (dk * inf_e)));
                let t1 = (dk * inf_e).dot(&(information * (dj * inf_e)));
                let value = 0.5 * (t0 + t1 + (-y * information * dk).trace());
                hessian[(j, k)] = value;
                hessian[(k, j)] = value;
            }
        }
        Some((cost, gradient, hessian))
    }
}
pub(crate) fn pose_from_parameters(x: &SVector<f64, 7>) -> Transformation {
    Transformation::new(
        x.fixed_rows::<3>(0).into_owned(),
        Quaternion::new(x[6], x[3], x[4], x[5]),
    )
}

#[derive(Clone, Debug)]
pub struct PseudoImuError {
    pub t0: Timestamp,
    pub t1: Timestamp,
}
impl ImuErrorBase for PseudoImuError {
    fn t0(&self) -> Timestamp {
        self.t0
    }
    fn t1(&self) -> Timestamp {
        self.t1
    }
    fn set_t0(&mut self, t: Timestamp) {
        self.t0 = t;
    }
    fn set_t1(&mut self, t: Timestamp) {
        self.t1 = t;
    }
    fn type_info(&self) -> &'static str {
        "PseudoImuError"
    }
}
/// The C++ snapshot declares, but supplies no definition of this propagation.
pub trait PseudoImuPropagation {
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn propagation(
        &self,
        pose: &mut Transformation,
        sb: &mut SpeedAndBias,
        start: Timestamp,
        end: Timestamp,
    ) -> Result<usize, IntegrationError>;
}
impl PseudoImuError {
    #[must_use]
    pub fn new(t0: Timestamp, t1: Timestamp) -> Self {
        Self { t0, t1 }
    }
    pub fn sync_from(&mut self, other: &Self) {
        *self = other.clone();
    }
    #[must_use]
    pub fn evaluate(
        &self,
        pose0: &SVector<f64, 7>,
        sb0: &SpeedAndBias,
        pose1: &SVector<f64, 7>,
        sb1: &SpeedAndBias,
    ) -> ImuEvaluation {
        let p0 = pose_from_parameters(pose0);
        let p1 = pose_from_parameters(pose1);
        let ct = p0.rotation_matrix().transpose();
        let dt = seconds(self.t1, self.t0);
        let dp = p0.translation() + sb0.fixed_rows::<3>(0) * dt - p1.translation();
        let dv = sb0.fixed_rows::<3>(0) - sb1.fixed_rows::<3>(0);
        let qr = quaternion_inverse(&p1.quaternion()) * p0.quaternion();
        let mut error = SVector::<f64, 15>::zeros();
        error.fixed_rows_mut::<3>(0).copy_from(&(ct * dp));
        error
            .fixed_rows_mut::<3>(3)
            .copy_from(&(2.0 * qr.coords.fixed_rows::<3>(0)));
        error.fixed_rows_mut::<3>(6).copy_from(&(ct * dv));
        error
            .fixed_rows_mut::<6>(9)
            .copy_from(&(sb0.fixed_rows::<6>(3) - sb1.fixed_rows::<6>(3)));
        let mut u = Information::identity();
        for (offset, weight) in [(0, 1.0), (3, 1.0), (6, 0.10), (9, 1.0), (12, 1.0)] {
            u.fixed_view_mut::<3, 3>(offset, offset)
                .scale_mut(weight / dt.sqrt());
        }
        let mut jp0 = SMatrix::<f64, 15, 6>::zeros();
        jp0.fixed_view_mut::<3, 3>(0, 0).copy_from(&ct);
        jp0.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(ct * cross_mx(&dp)));
        jp0.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(plus(&qr).fixed_view::<3, 3>(0, 0) * ct));
        jp0.fixed_view_mut::<3, 3>(6, 3)
            .copy_from(&(ct * cross_mx(&dv)));
        let mut jp1 = SMatrix::<f64, 15, 6>::zeros();
        jp1.fixed_view_mut::<3, 3>(0, 0).copy_from(&-ct);
        jp1.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(-plus(&qr).fixed_view::<3, 3>(0, 0) * ct));
        let mut js0 = SMatrix::<f64, 15, 9>::zeros();
        js0.fixed_view_mut::<3, 3>(0, 0).copy_from(&(dt * ct));
        js0.fixed_view_mut::<3, 3>(6, 0).copy_from(&ct);
        js0.fixed_view_mut::<6, 6>(9, 3)
            .copy_from(&SMatrix::<f64, 6, 6>::identity());
        let mut js1 = SMatrix::<f64, 15, 9>::zeros();
        js1.fixed_view_mut::<3, 3>(6, 0).copy_from(&-ct);
        js1.fixed_view_mut::<6, 6>(9, 3)
            .copy_from(&-SMatrix::<f64, 6, 6>::identity());
        let j0 = u * jp0;
        let j2 = u * jp1;
        ImuEvaluation {
            residual: u * error,
            pose0: j0 * PoseManifold::minus_jacobian(pose0),
            speed_bias0: u * js0,
            pose1: j2 * PoseManifold::minus_jacobian(pose1),
            speed_bias1: u * js1,
            pose0_minimal: j0,
            pose1_minimal: j2,
            speed_bias0_minimal: u * js0,
            speed_bias1_minimal: u * js1,
            integration_success: true,
        }
    }
}
