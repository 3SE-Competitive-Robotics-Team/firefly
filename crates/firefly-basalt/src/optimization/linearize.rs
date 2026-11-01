// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/optimization/linearize.h`.
use firefly_calibration::calibration::Calibration;
use firefly_calibration::calibration::MocapCalibration;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{Matrix3, Matrix4, SMatrix, SVector, Vector2, Vector3, Vector4};
use std::collections::BTreeMap;

pub const POSE_SIZE: usize = 6;
pub const POS_SIZE: usize = 3;
pub const POS_OFFSET: usize = 0;
pub const ROT_SIZE: usize = 3;
pub const ROT_OFFSET: usize = 3;
pub const ACCEL_BIAS_SIZE: usize = 9;
pub const GYRO_BIAS_SIZE: usize = 12;
pub const G_SIZE: usize = 3;
pub const TIME_SIZE: usize = 1;
pub const ACCEL_BIAS_OFFSET: usize = 0;
pub const GYRO_BIAS_OFFSET: usize = ACCEL_BIAS_SIZE;
pub const G_OFFSET: usize = GYRO_BIAS_OFFSET + GYRO_BIAS_SIZE;

#[derive(Clone, Debug)]
pub struct PoseCalibH<const N: usize> {
    pub h_pose_accum: SMatrix<f64, 6, 6>,
    pub b_pose_accum: SVector<f64, 6>,
    pub h_intr_pose_accum: SMatrix<f64, N, 6>,
    pub h_intr_accum: SMatrix<f64, N, N>,
    pub b_intr_accum: SVector<f64, N>,
}
impl<const N: usize> Default for PoseCalibH<N> {
    fn default() -> Self {
        Self {
            h_pose_accum: SMatrix::zeros(),
            b_pose_accum: SVector::zeros(),
            h_intr_pose_accum: SMatrix::zeros(),
            h_intr_accum: SMatrix::zeros(),
            b_intr_accum: SVector::zeros(),
        }
    }
}

#[derive(Clone)]
#[expect(
    clippy::struct_excessive_bools,
    reason = "Independent original calibration parameter activation flags"
)]
pub struct CalibCommonData<'a, S: BasaltScalar = f64> {
    pub calibration: &'a Calibration<S>,
    pub mocap_calibration: Option<&'a MocapCalibration<S>>,
    pub aprilgrid_corner_pos_3d: &'a [Vector4<f64>],
    pub offset_t_i_c: &'a [usize],
    pub offset_intrinsics: &'a [usize],
    pub mocap_block_offset: usize,
    pub bias_block_offset: usize,
    pub offset_poses: &'a BTreeMap<i64, usize>,
    pub g: Option<&'a Vector3<S>>,
    pub opt_intrinsics: bool,
    pub opt_cam_time_offset: bool,
    pub opt_g: bool,
    pub opt_imu_scale: bool,
    pub pose_var_inv: S,
    pub gyro_var_inv: Vector3<S>,
    pub accel_var_inv: Vector3<S>,
    pub mocap_var_inv: S,
    pub huber_thresh: S,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ReprojectionStats {
    pub error: f64,
    pub reprojection_error: f64,
    pub num_points: usize,
}
impl ReprojectionStats {
    pub fn join(&mut self, other: &Self) {
        self.error += other.error;
        self.reprojection_error += other.reprojection_error;
        self.num_points += other.num_points;
    }
}

impl CalibCommonData<'_, f64> {
    /// Accumulates the original Huber-weighted point normal blocks.
    /// # Panics
    /// Requires `corner_id` to index the configured grid.
    pub fn linearize_point<const N: usize>(
        &self,
        corner_pos: &Vector2<f64>,
        corner_id: usize,
        t_c_w: &Matrix4<f64>,
        project: impl Fn(
            &Vector4<f64>,
            &mut Vector2<f64>,
            Option<&mut SMatrix<f64, 2, 4>>,
            Option<&mut SMatrix<f64, 2, N>>,
        ) -> bool,
        accum: Option<&mut PoseCalibH<N>>,
        stats: &mut ReprojectionStats,
    ) {
        let point = t_c_w * self.aprilgrid_corner_pos_3d[corner_id];
        let mut pixel = Vector2::zeros();
        let mut d_point = SMatrix::<f64, 2, 4>::zeros();
        let mut d_param = SMatrix::<f64, 2, N>::zeros();
        let valid = if accum.is_some() {
            project(&point, &mut pixel, Some(&mut d_point), Some(&mut d_param))
        } else {
            project(&point, &mut pixel, None, None)
        };
        if !valid || pixel.iter().any(|x| !x.is_finite()) {
            return;
        }
        let residual = pixel - corner_pos;
        let e = residual.norm();
        let weight = if e < self.huber_thresh {
            1.0
        } else {
            self.huber_thresh / e
        };
        if let Some(accum) = accum {
            let mut d_xi = SMatrix::<f64, 4, 6>::zeros();
            d_xi.fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&Matrix3::identity());
            d_xi.fixed_view_mut::<3, 3>(0, 3).copy_from(&Matrix3::new(
                0.0, point.z, -point.y, -point.z, 0.0, point.x, point.y, -point.x, 0.0,
            ));
            let jacobian = d_point * d_xi;
            accum.h_pose_accum += weight * jacobian.transpose() * jacobian;
            accum.b_pose_accum += weight * jacobian.transpose() * residual;
            accum.h_intr_pose_accum += weight * d_param.transpose() * jacobian;
            accum.h_intr_accum += weight * d_param.transpose() * d_param;
            accum.b_intr_accum += weight * d_param.transpose() * residual;
        }
        stats.error += weight * e * e * (2.0 - weight);
        stats.reprojection_error += e;
        stats.num_points += 1;
    }
}
