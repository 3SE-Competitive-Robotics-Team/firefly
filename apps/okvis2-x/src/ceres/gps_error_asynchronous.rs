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
//! Asynchronous GPS position factor with IMU propagation from `GpsErrorAsynchronous.cpp`.
use super::{
    imu_error::{
        IntegrationError, NoiseMode, Preintegration, pose_from_parameters, seconds,
        world_covariance,
    },
    pose_error::square_root_information,
    pose_local_parameterization::PoseManifold,
};
use crate::{
    config::parameters::{GpsParameters, ImuParameters},
    frames::frame_typedefs::SpeedAndBias,
    kinematics::{
        operators::cross_mx,
        transformation::{Transformation, delta_q, raw_rotation_matrix},
    },
    sensors::measurements::ImuMeasurementDeque,
};
use jiff::Timestamp;
use nalgebra::{Matrix3, Quaternion, SMatrix, SVector, Vector3};
use std::sync::atomic::{AtomicBool, Ordering};
pub static REDO_PROPAGATION_ALWAYS: AtomicBool = AtomicBool::new(false);
pub static USE_IMU_COVARIANCE: AtomicBool = AtomicBool::new(true);

/// Eigen's fixed-size adjugate inverse has no near-singular cutoff.
fn inverse3(m: &Matrix3<f64>) -> Matrix3<f64> {
    let c0 = m.column(1).cross(&m.column(2));
    let c1 = m.column(2).cross(&m.column(0));
    let c2 = m.column(0).cross(&m.column(1));
    Matrix3::from_rows(&[c0.transpose(), c1.transpose(), c2.transpose()]) / m.column(0).dot(&c0)
}
#[derive(Clone, Debug)]
pub struct GpsErrorAsynchronous {
    pub measurement: Vector3<f64>,
    pub error: Option<Vector3<f64>>,
    pub gps_parameters: GpsParameters,
    pub gps_information: Matrix3<f64>,
    pub gps_covariance: Matrix3<f64>,
    pub imu_parameters: ImuParameters,
    pub imu_measurements: ImuMeasurementDeque,
    pub tk: Timestamp,
    pub tg: Timestamp,
    pub square_root_information: Option<Matrix3<f64>>,
    pub redo: bool,
    pub redo_counter: usize,
    preintegration: Preintegration,
}
#[derive(Clone, Debug)]
pub struct GpsEvaluation {
    pub residual: Vector3<f64>,
    pub pose: SMatrix<f64, 3, 7>,
    pub speed_bias: SMatrix<f64, 3, 9>,
    pub gps_world: SMatrix<f64, 3, 7>,
    pub pose_minimal: SMatrix<f64, 3, 6>,
    pub gps_world_minimal: SMatrix<f64, 3, 6>,
}
impl GpsErrorAsynchronous {
    #[must_use]
    pub fn new(
        measurement: Vector3<f64>,
        information: Matrix3<f64>,
        imu_measurements: ImuMeasurementDeque,
        imu_parameters: ImuParameters,
        tk: Timestamp,
        tg: Timestamp,
        gps_parameters: GpsParameters,
    ) -> Self {
        Self {
            measurement,
            error: None,
            gps_parameters,
            gps_information: information,
            gps_covariance: inverse3(&information),
            imu_parameters,
            imu_measurements,
            tk,
            tg,
            square_root_information: None,
            redo: true,
            redo_counter: 0,
            preintegration: Preintegration::default(),
        }
    }
    #[must_use]
    pub fn from_standard_deviations(
        measurement: Vector3<f64>,
        sigma: Vector3<f64>,
        imu_measurements: ImuMeasurementDeque,
        imu_parameters: ImuParameters,
        tk: Timestamp,
        tg: Timestamp,
        gps_parameters: GpsParameters,
    ) -> Self {
        let information = Matrix3::from_diagonal(&sigma.map(|s| 1.0 / (s * s)));
        Self::new(
            measurement,
            information,
            imu_measurements,
            imu_parameters,
            tk,
            tg,
            gps_parameters,
        )
    }
    pub fn set_measurement(&mut self, x: Vector3<f64>) {
        self.measurement = x;
    }
    pub fn set_information(&mut self, x: Matrix3<f64>) {
        self.gps_information = x;
        self.gps_covariance = inverse3(&x);
    }
    pub fn set_gps_parameters(&mut self, x: GpsParameters) {
        self.gps_parameters = x;
    }
    pub fn set_imu_parameters(&mut self, x: ImuParameters) {
        self.imu_parameters = x;
    }
    pub fn set_imu_measurements(&mut self, x: ImuMeasurementDeque) {
        self.imu_measurements = x;
    }
    pub fn set_tg(&mut self, x: Timestamp) {
        self.tg = x;
    }
    pub fn set_tk(&mut self, x: Timestamp) {
        self.tk = x;
    }
    #[must_use]
    pub fn residual_dim(&self) -> usize {
        3
    }
    #[must_use]
    pub fn parameter_blocks(&self) -> usize {
        3
    }
    #[must_use]
    pub fn parameter_block_dim(&self, index: usize) -> Option<usize> {
        [7, 9, 7].get(index).copied()
    }
    #[must_use]
    pub fn type_info(&self) -> &'static str {
        "GpsErrorAsynchronous"
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
        let use_cov = USE_IMU_COVARIANCE.load(Ordering::Relaxed);
        let (steps, _) = self.preintegration.integrate(
            &self.imu_measurements,
            Some(&self.imu_parameters),
            sb,
            (self.tk, self.tg),
            true,
            if use_cov {
                NoiseMode::Basis
            } else {
                NoiseMode::None
            },
        )?;
        self.preintegration.reference = *sb;
        if use_cov {
            self.preintegration.finish_covariance(&self.imu_parameters);
        }
        Ok(steps)
    }
    /// Applies the cached increment, and marks large gyro-bias changes for the
    /// next evaluation. It does not recompute preintegration itself.
    pub fn apply_pre_int(&mut self, pose: &Transformation, sb: &SpeedAndBias) -> Transformation {
        let db = sb.fixed_rows::<6>(3) - self.preintegration.reference.fixed_rows::<6>(3);
        self.redo |= db.fixed_rows::<3>(0).norm() > 0.0003;
        self.apply_cached(pose, sb, &db)
    }
    fn apply_cached(
        &self,
        pose: &Transformation,
        sb: &SpeedAndBias,
        db: &SVector<f64, 6>,
    ) -> Transformation {
        let p = &self.preintegration;
        let dt = seconds(self.tg, self.tk);
        let g = Vector3::new(0.0, 0.0, self.imu_parameters.g);
        Transformation::new(
            pose.translation() + sb.fixed_rows::<3>(0) * dt - 0.5 * g * dt * dt
                + pose.rotation_matrix()
                    * (p.acc_doubleintegral + p.dp_db_g * db.fixed_rows::<3>(0)
                        - p.c_doubleintegral * db.fixed_rows::<3>(3)),
            pose.quaternion() * p.delta_q * delta_q(&(-p.dalpha_db_g * db.fixed_rows::<3>(0))),
        )
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn evaluate(
        &mut self,
        pose: &SVector<f64, 7>,
        sb: &SpeedAndBias,
        gps_world: &SVector<f64, 7>,
    ) -> Result<GpsEvaluation, IntegrationError> {
        let p0 = pose_from_parameters(pose);
        // GPS/world rotation is read directly without constructing a normalized pose.
        let cgw = raw_rotation_matrix(&Quaternion::new(
            gps_world[6],
            gps_world[3],
            gps_world[4],
            gps_world[5],
        ));
        let rgw = gps_world.fixed_rows::<3>(0);
        let rsa = self.gps_parameters.r_sa;
        let mut db = sb.fixed_rows::<6>(3) - self.preintegration.reference.fixed_rows::<6>(3);
        self.redo |= db.fixed_rows::<3>(0).norm() > 0.0003;
        if REDO_PROPAGATION_ALWAYS.load(Ordering::Relaxed)
            || (self.redo && self.imu_measurements.len() < 50)
            || self.redo_counter == 0
        {
            self.redo_preintegration(&p0, sb)?;
            self.redo_counter += 1;
            db.fill(0.0);
            self.redo = false;
        }
        let pg = self.apply_cached(&p0, sb, &db);
        let c = p0.rotation_matrix();
        let jprop = self
            .preintegration
            .propagation_jacobian(&c, seconds(self.tg, self.tk));
        let covariance = if USE_IMU_COVARIANCE.load(Ordering::Relaxed) {
            let p = world_covariance(&self.preintegration.p_delta, &c);
            let mut jws = SMatrix::<f64, 3, 6>::zeros();
            jws.fixed_view_mut::<3, 3>(0, 0).copy_from(&cgw);
            jws.fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&(-cgw * cross_mx(&(c * rsa))));
            self.gps_covariance + jws * p.fixed_view::<6, 6>(0, 0) * jws.transpose()
        } else {
            self.gps_covariance
        };
        let information = inverse3(&covariance);
        let u = square_root_information(information);
        self.square_root_information = Some(u);
        let antenna = pg.translation() + pg.rotation_matrix() * rsa;
        let error = self.measurement - (cgw * antenna + rgw);
        self.error = Some(error);
        let mut jg = SMatrix::<f64, 3, 6>::zeros();
        jg.fixed_view_mut::<3, 3>(0, 0).copy_from(&-cgw);
        jg.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(cgw * cross_mx(&(pg.rotation_matrix() * rsa))));
        let j0 = u * jg * jprop.fixed_view::<6, 6>(0, 0);
        let j1 = u * jg * jprop.fixed_view::<6, 9>(0, 6);
        let mut j2 = SMatrix::<f64, 3, 6>::zeros();
        j2.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&-Matrix3::identity());
        j2.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&cross_mx(&(cgw * antenna)));
        let j2 = u * j2;
        Ok(GpsEvaluation {
            residual: u * error,
            pose: j0 * PoseManifold::minus_jacobian(pose),
            speed_bias: j1,
            gps_world: j2 * PoseManifold::minus_jacobian(gps_world),
            pose_minimal: j0,
            gps_world_minimal: j2,
        })
    }
}
