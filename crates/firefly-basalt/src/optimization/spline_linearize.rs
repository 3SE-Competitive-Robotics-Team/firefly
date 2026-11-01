// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original `optimization/spline_linearize.h`, including all five measurement families.
use super::{
    accumulator::{SolverAccumulator, SparseHashAccumulator},
    linearize::{CalibCommonData, PoseCalibH, ReprojectionStats},
    poses_linearize::adjoint,
};
use crate::io::dataset_io::{AccelData, AprilgridCornersData, GyroData, MocapPoseData, PoseData};
use firefly_basalt_headers::{
    camera::generic_camera::GenericCamera,
    spline::{
        se3_spline::{AccelPosSo3Jacobian, PosePosSo3Jacobian, Se3Spline},
        so3_spline::Jacobian as RotationJacobian,
        spline_common::Jacobian as PositionJacobian,
    },
    utils::sophus_utils::{
        right_jacobian_inv_se3_decoupled, se3_inverse as inverse, se3_logd, se3_matrix,
        se3_product as product,
    },
};
use nalgebra::{Matrix3, Matrix4, SMatrix, Vector2, Vector4};
use rayon::prelude::*;

pub struct LinearizeSplineOpt<
    'a,
    const N: usize,
    A: SolverAccumulator<f64> = SparseHashAccumulator<f64>,
> {
    pub accum: A,
    pub stats: ReprojectionStats,
    pub opt_size: usize,
    pub spline: &'a Se3Spline<f64, N>,
    pub common_data: CalibCommonData<'a>,
}
pub struct ComputeErrorSplineOpt<'a, const N: usize> {
    pub stats: ReprojectionStats,
    pub opt_size: usize,
    pub spline: &'a Se3Spline<f64, N>,
    pub common_data: CalibCommonData<'a>,
}
/// Typed equivalent of the five blocked-range overloads.
pub trait SplineMeasurement: Sync {
    fn linearize<const N: usize, A: SolverAccumulator<f64>>(
        &self,
        target: &mut LinearizeSplineOpt<'_, N, A>,
    );
    fn compute_error<const N: usize>(&self, target: &mut ComputeErrorSplineOpt<'_, N>);
}
impl<'a, const N: usize, A: SolverAccumulator<f64>> LinearizeSplineOpt<'a, N, A> {
    #[must_use]
    pub fn new(
        opt_size: usize,
        spline: &'a Se3Spline<f64, N>,
        common_data: CalibCommonData<'a>,
    ) -> Self {
        let mut accum = A::default();
        accum.reset(opt_size);
        Self {
            accum,
            stats: ReprojectionStats::default(),
            opt_size,
            spline,
            common_data,
        }
    }
    pub fn join(&mut self, other: &Self) {
        self.accum.join(&other.accum);
        self.stats.join(&other.stats);
    }
    pub fn linearize<M: SplineMeasurement>(&mut self, measurement: &M) {
        measurement.linearize(self);
    }
    pub fn parallel_linearize<M: SplineMeasurement>(&mut self, measurements: &[M]) {
        let result = measurements
            .par_iter()
            .fold(
                || Self::new(self.opt_size, self.spline, self.common_data.clone()),
                |mut state, measurement| {
                    measurement.linearize(&mut state);
                    state
                },
            )
            .reduce(
                || Self::new(self.opt_size, self.spline, self.common_data.clone()),
                |mut left, right| {
                    left.join(&right);
                    left
                },
            );
        self.join(&result);
    }
    fn camera<const I: usize>(
        &mut self,
        observation: &AprilgridCornersData,
        camera: usize,
        project: impl Fn(
            &Vector4<f64>,
            &mut Vector2<f64>,
            Option<&mut SMatrix<f64, 2, 4>>,
            Option<&mut SMatrix<f64, 2, I>>,
        ) -> bool,
    ) {
        let time = observation.timestamp_ns + self.common_data.calibration.cam_time_offset_ns;
        if time < self.spline.min_time_ns() || time >= self.spline.max_time_ns() {
            return;
        }
        let mut jacobian = PosePosSo3Jacobian::default();
        let pose = self.spline.pose(time, Some(&mut jacobian));
        let time_derivative = self.spline.d_pose_d_t(time);
        let extrinsic = &self.common_data.calibration.t_i_c[camera];
        let transform = se3_matrix(&inverse(&product(&pose, extrinsic)));
        let mut block = PoseCalibH::<I>::default();
        for (corner, &id) in observation.corner_pos.iter().zip(&observation.corner_id) {
            self.common_data.linearize_point(
                corner,
                usize::try_from(id).unwrap(),
                &transform,
                &project,
                Some(&mut block),
                &mut self.stats,
            );
        }
        let adj = -adjoint(&inverse(extrinsic));
        let h_pose = adj.transpose() * block.h_pose_accum * adj;
        let extrinsic_offset = self.common_data.offset_t_i_c[camera];
        let intrinsic_offset = self.common_data.offset_intrinsics[camera];
        let time_offset = self.common_data.mocap_block_offset + 13;
        for i in 0..N {
            let row = (jacobian.start_idx + i) * 6;
            let ji_h = jacobian.d_val_d_knot[i].transpose() * h_pose;
            for j in 0..=i {
                self.accum.add_h(
                    row,
                    (jacobian.start_idx + j) * 6,
                    &(ji_h * jacobian.d_val_d_knot[j]),
                );
            }
            self.accum.add_h(
                extrinsic_offset,
                row,
                &(-block.h_pose_accum * adj * jacobian.d_val_d_knot[i]),
            );
            if self.common_data.opt_intrinsics {
                self.accum.add_h(
                    intrinsic_offset,
                    row,
                    &(block.h_intr_pose_accum * adj * jacobian.d_val_d_knot[i]),
                );
            }
            if self.common_data.opt_cam_time_offset {
                self.accum.add_h(
                    time_offset,
                    row,
                    &(time_derivative.transpose() * h_pose * jacobian.d_val_d_knot[i]),
                );
            }
            self.accum.add_b(
                row,
                &(jacobian.d_val_d_knot[i].transpose() * adj.transpose() * block.b_pose_accum),
            );
        }
        self.accum
            .add_h(extrinsic_offset, extrinsic_offset, &block.h_pose_accum);
        self.accum.add_b(extrinsic_offset, &(-block.b_pose_accum));
        if self.common_data.opt_intrinsics {
            self.accum.add_h(
                intrinsic_offset,
                extrinsic_offset,
                &(-block.h_intr_pose_accum),
            );
            self.accum
                .add_h(intrinsic_offset, intrinsic_offset, &block.h_intr_accum);
            self.accum.add_b(intrinsic_offset, &block.b_intr_accum);
        }
        if self.common_data.opt_cam_time_offset {
            self.accum.add_h(
                time_offset,
                extrinsic_offset,
                &(-time_derivative.transpose() * adj.transpose() * block.h_pose_accum),
            );
            if self.common_data.opt_intrinsics {
                self.accum.add_h(
                    time_offset,
                    intrinsic_offset,
                    &(time_derivative.transpose()
                        * adj.transpose()
                        * block.h_intr_pose_accum.transpose()),
                );
            }
            self.accum.add_h(
                time_offset,
                time_offset,
                &(time_derivative.transpose() * h_pose * time_derivative),
            );
            self.accum.add_b(
                time_offset,
                &(time_derivative.transpose() * adj.transpose() * block.b_pose_accum),
            );
        }
    }
}
impl<'a, const N: usize> ComputeErrorSplineOpt<'a, N> {
    #[must_use]
    pub fn new(
        opt_size: usize,
        spline: &'a Se3Spline<f64, N>,
        common_data: CalibCommonData<'a>,
    ) -> Self {
        Self {
            stats: ReprojectionStats::default(),
            opt_size,
            spline,
            common_data,
        }
    }
    pub fn join(&mut self, other: &Self) {
        self.stats.join(&other.stats);
    }
    pub fn compute_error<M: SplineMeasurement>(&mut self, measurement: &M) {
        measurement.compute_error(self);
    }
    pub fn parallel_error<M: SplineMeasurement>(&mut self, measurements: &[M]) {
        let result = measurements
            .par_iter()
            .fold(
                || Self::new(self.opt_size, self.spline, self.common_data.clone()),
                |mut state, measurement| {
                    measurement.compute_error(&mut state);
                    state
                },
            )
            .reduce(
                || Self::new(self.opt_size, self.spline, self.common_data.clone()),
                |mut left, right| {
                    left.join(&right);
                    left
                },
            );
        self.join(&result);
    }
    fn camera<const I: usize>(
        &mut self,
        observation: &AprilgridCornersData,
        transform: &Matrix4<f64>,
        project: impl Fn(
            &Vector4<f64>,
            &mut Vector2<f64>,
            Option<&mut SMatrix<f64, 2, 4>>,
            Option<&mut SMatrix<f64, 2, I>>,
        ) -> bool,
    ) {
        for (corner, &id) in observation.corner_pos.iter().zip(&observation.corner_id) {
            self.common_data.linearize_point(
                corner,
                usize::try_from(id).unwrap(),
                transform,
                &project,
                None,
                &mut self.stats,
            );
        }
    }
}
impl SplineMeasurement for PoseData {
    fn linearize<const N: usize, A: SolverAccumulator<f64>>(
        &self,
        target: &mut LinearizeSplineOpt<'_, N, A>,
    ) {
        assert!(self.timestamp_ns >= target.spline.min_time_ns());
        let mut position = PositionJacobian::default();
        let mut rotation = RotationJacobian::default();
        let rp = target.spline.position_residual(
            self.timestamp_ns,
            &self.data.translation.vector,
            Some(&mut position),
        );
        let rr = target.spline.orientation_residual(
            self.timestamp_ns,
            &self.data.rotation,
            Some(&mut rotation),
        );
        assert_eq!(position.start_idx, rotation.start_idx);
        let weight = target.common_data.pose_var_inv;
        target.stats.error += weight * (rp.norm_squared() + rr.norm_squared());
        for i in 0..N {
            let row = (position.start_idx + i) * 6;
            assert!(row < target.opt_size);
            for j in 0..=i {
                let col = (position.start_idx + j) * 6;
                assert!(col < target.opt_size);
                target.accum.add_h(
                    row,
                    col,
                    &(Matrix3::identity()
                        * (weight * position.d_val_d_knot[i] * position.d_val_d_knot[j])),
                );
                target.accum.add_h(
                    row + 3,
                    col + 3,
                    &((rotation.d_val_d_knot[i].transpose() * weight) * rotation.d_val_d_knot[j]),
                );
            }
            target
                .accum
                .add_b(row, &(rp * (weight * position.d_val_d_knot[i])));
            target.accum.add_b(
                row + 3,
                &((rotation.d_val_d_knot[i].transpose() * weight) * rr),
            );
        }
    }
    fn compute_error<const N: usize>(&self, target: &mut ComputeErrorSplineOpt<'_, N>) {
        assert!(self.timestamp_ns >= target.spline.min_time_ns());
        let position =
            target
                .spline
                .position_residual(self.timestamp_ns, &self.data.translation.vector, None);
        let rotation =
            target
                .spline
                .orientation_residual(self.timestamp_ns, &self.data.rotation, None);
        target.stats.error +=
            target.common_data.pose_var_inv * (position.norm_squared() + rotation.norm_squared());
    }
}
impl SplineMeasurement for AccelData {
    fn linearize<const N: usize, A: SolverAccumulator<f64>>(
        &self,
        target: &mut LinearizeSplineOpt<'_, N, A>,
    ) {
        assert!(
            self.timestamp_ns >= target.spline.min_time_ns()
                && self.timestamp_ns <= target.spline.max_time_ns()
        );
        let mut jacobian = AccelPosSo3Jacobian::default();
        let mut bias = SMatrix::<f64, 3, 9>::zeros();
        let mut gravity = Matrix3::zeros();
        let residual = target.spline.accel_residual(
            self.timestamp_ns,
            &self.data,
            &target.common_data.calibration.calib_accel_bias,
            target.common_data.g.unwrap(),
            Some(&mut jacobian),
            Some(&mut bias),
            Some(&mut gravity),
        );
        if !target.common_data.opt_imu_scale {
            bias.fixed_view_mut::<3, 6>(0, 3).fill(0.);
        }
        let weight = Matrix3::from_diagonal(&target.common_data.accel_var_inv);
        target.stats.error += (residual.transpose() * weight * residual)[0];
        let bias_offset = target.common_data.bias_block_offset;
        let gravity_offset = bias_offset + 21;
        for i in 0..N {
            let row = (jacobian.start_idx + i) * 6;
            assert!(row < target.opt_size);
            for j in 0..=i {
                let col = (jacobian.start_idx + j) * 6;
                assert!(col < target.opt_size);
                target.accum.add_h(
                    row,
                    col,
                    &(jacobian.d_val_d_knot[i].transpose() * weight * jacobian.d_val_d_knot[j]),
                );
            }
            target.accum.add_h(
                bias_offset,
                row,
                &(bias.transpose() * weight * jacobian.d_val_d_knot[i]),
            );
            if target.common_data.opt_g {
                target.accum.add_h(
                    gravity_offset,
                    row,
                    &(gravity.transpose() * weight * jacobian.d_val_d_knot[i]),
                );
            }
            target.accum.add_b(
                row,
                &(jacobian.d_val_d_knot[i].transpose() * weight * residual),
            );
        }
        target.accum.add_h(
            bias_offset,
            bias_offset,
            &(bias.transpose() * weight * bias),
        );
        if target.common_data.opt_g {
            target.accum.add_h(
                gravity_offset,
                bias_offset,
                &(gravity.transpose() * weight * bias),
            );
            target.accum.add_h(
                gravity_offset,
                gravity_offset,
                &(gravity.transpose() * weight * gravity),
            );
            target
                .accum
                .add_b(gravity_offset, &(gravity.transpose() * weight * residual));
        }
        target
            .accum
            .add_b(bias_offset, &(bias.transpose() * weight * residual));
    }
    fn compute_error<const N: usize>(&self, target: &mut ComputeErrorSplineOpt<'_, N>) {
        assert!(
            self.timestamp_ns >= target.spline.min_time_ns()
                && self.timestamp_ns <= target.spline.max_time_ns()
        );
        let residual = target.spline.accel_residual(
            self.timestamp_ns,
            &self.data,
            &target.common_data.calibration.calib_accel_bias,
            target.common_data.g.unwrap(),
            None,
            None,
            None,
        );
        target.stats.error += (residual.transpose()
            * Matrix3::from_diagonal(&target.common_data.accel_var_inv)
            * residual)[0];
    }
}
impl SplineMeasurement for GyroData {
    fn linearize<const N: usize, A: SolverAccumulator<f64>>(
        &self,
        target: &mut LinearizeSplineOpt<'_, N, A>,
    ) {
        assert!(
            self.timestamp_ns >= target.spline.min_time_ns()
                && self.timestamp_ns <= target.spline.max_time_ns()
        );
        let mut jacobian = RotationJacobian::default();
        let mut bias = SMatrix::<f64, 3, 12>::zeros();
        let residual = target.spline.gyro_residual(
            self.timestamp_ns,
            &self.data,
            &target.common_data.calibration.calib_gyro_bias,
            Some(&mut jacobian),
            Some(&mut bias),
        );
        if !target.common_data.opt_imu_scale {
            bias.fixed_view_mut::<3, 9>(0, 3).fill(0.);
        }
        let weight = Matrix3::from_diagonal(&target.common_data.gyro_var_inv);
        target.stats.error += (residual.transpose() * weight * residual)[0];
        let bias_offset = target.common_data.bias_block_offset + 9;
        for i in 0..N {
            let row = (jacobian.start_idx + i) * 6 + 3;
            assert!(row < target.opt_size);
            for j in 0..=i {
                target.accum.add_h(
                    row,
                    (jacobian.start_idx + j) * 6 + 3,
                    &(jacobian.d_val_d_knot[i].transpose() * weight * jacobian.d_val_d_knot[j]),
                );
            }
            target.accum.add_h(
                bias_offset,
                row,
                &(bias.transpose() * weight * jacobian.d_val_d_knot[i]),
            );
            target.accum.add_b(
                row,
                &(jacobian.d_val_d_knot[i].transpose() * weight * residual),
            );
        }
        target.accum.add_h(
            bias_offset,
            bias_offset,
            &(bias.transpose() * weight * bias),
        );
        target
            .accum
            .add_b(bias_offset, &(bias.transpose() * weight * residual));
    }
    fn compute_error<const N: usize>(&self, target: &mut ComputeErrorSplineOpt<'_, N>) {
        assert!(
            self.timestamp_ns >= target.spline.min_time_ns()
                && self.timestamp_ns <= target.spline.max_time_ns()
        );
        let residual = target.spline.gyro_residual(
            self.timestamp_ns,
            &self.data,
            &target.common_data.calibration.calib_gyro_bias,
            None,
            None,
        );
        target.stats.error += (residual.transpose()
            * Matrix3::from_diagonal(&target.common_data.gyro_var_inv)
            * residual)[0];
    }
}
impl SplineMeasurement for AprilgridCornersData {
    fn linearize<const N: usize, A: SolverAccumulator<f64>>(
        &self,
        target: &mut LinearizeSplineOpt<'_, N, A>,
    ) {
        let camera = usize::try_from(self.cam_id).unwrap();
        macro_rules! visit {
            ($model:expr,$size:literal) => {
                target.camera::<$size>(self, camera, |point, pixel, jp, ji| {
                    $model.project(point, pixel, jp, ji)
                })
            };
        }
        match target.common_data.calibration.intrinsics[camera].clone() {
            GenericCamera::ExtendedUnified(c) => visit!(c, 6),
            GenericCamera::DoubleSphere(c) => visit!(c, 6),
            GenericCamera::KannalaBrandt4(c) => visit!(c, 8),
            GenericCamera::Unified(c) => visit!(c, 5),
            GenericCamera::Pinhole(c) => visit!(c, 4),
            GenericCamera::PinholeRadtan8(c) => visit!(c, 12),
        }
    }
    fn compute_error<const N: usize>(&self, target: &mut ComputeErrorSplineOpt<'_, N>) {
        let time = self.timestamp_ns + target.common_data.calibration.cam_time_offset_ns;
        if time < target.spline.min_time_ns() || time >= target.spline.max_time_ns() {
            return;
        }
        let camera = usize::try_from(self.cam_id).unwrap();
        let transform = se3_matrix(&inverse(&product(
            &target.spline.pose(time, None),
            &target.common_data.calibration.t_i_c[camera],
        )));
        macro_rules! visit {
            ($model:expr,$size:literal) => {
                target.camera::<$size>(self, &transform, |point, pixel, jp, ji| {
                    $model.project(point, pixel, jp, ji)
                })
            };
        }
        match target.common_data.calibration.intrinsics[camera].clone() {
            GenericCamera::ExtendedUnified(c) => visit!(c, 6),
            GenericCamera::DoubleSphere(c) => visit!(c, 6),
            GenericCamera::KannalaBrandt4(c) => visit!(c, 8),
            GenericCamera::Unified(c) => visit!(c, 5),
            GenericCamera::Pinhole(c) => visit!(c, 4),
            GenericCamera::PinholeRadtan8(c) => visit!(c, 12),
        }
    }
}
impl SplineMeasurement for MocapPoseData {
    fn linearize<const N: usize, A: SolverAccumulator<f64>>(
        &self,
        target: &mut LinearizeSplineOpt<'_, N, A>,
    ) {
        let calibration = target.common_data.mocap_calibration.unwrap();
        let time = self.timestamp_ns + calibration.mocap_time_offset_ns;
        if time < target.spline.min_time_ns() || time >= target.spline.max_time_ns() {
            return;
        }
        let mut jacobian = PosePosSo3Jacobian::default();
        let pose = target.spline.pose(time, Some(&mut jacobian));
        let mocap_pose = product(&product(&calibration.t_moc_w, &pose), &calibration.t_i_mark);
        let residual = se3_logd(&product(&inverse(&self.data), &mocap_pose));
        let marker = right_jacobian_inv_se3_decoupled(&residual);
        let pose_j = marker * adjoint(&inverse(&calibration.t_i_mark));
        let world = marker * adjoint(&inverse(&product(&pose, &calibration.t_i_mark)));
        let time_j = pose_j * target.spline.d_pose_d_t(time);
        let world_offset = target.common_data.mocap_block_offset;
        let marker_offset = world_offset + 6;
        let time_offset = world_offset + 12;
        let weight = target.common_data.mocap_var_inv;
        target.stats.error += weight * residual.norm_squared();
        let h_pose = pose_j.transpose() * pose_j;
        for i in 0..N {
            let row = (jacobian.start_idx + i) * 6;
            assert!(row < target.opt_size);
            let ji_h = jacobian.d_val_d_knot[i].transpose() * h_pose;
            for j in 0..=i {
                let col = (jacobian.start_idx + j) * 6;
                assert!(col < target.opt_size);
                target
                    .accum
                    .add_h(row, col, &((ji_h * weight) * jacobian.d_val_d_knot[j]));
            }
            target.accum.add_b(
                row,
                &((jacobian.d_val_d_knot[i].transpose() * weight) * pose_j.transpose() * residual),
            );
            target.accum.add_h(
                world_offset,
                row,
                &((world.transpose() * weight) * pose_j * jacobian.d_val_d_knot[i]),
            );
            target.accum.add_h(
                marker_offset,
                row,
                &((marker.transpose() * weight) * pose_j * jacobian.d_val_d_knot[i]),
            );
            target.accum.add_h(
                time_offset,
                row,
                &((time_j.transpose() * weight) * pose_j * jacobian.d_val_d_knot[i]),
            );
        }
        target.accum.add_h(
            world_offset,
            world_offset,
            &((world.transpose() * weight) * world),
        );
        target.accum.add_h(
            marker_offset,
            world_offset,
            &((marker.transpose() * weight) * world),
        );
        target.accum.add_h(
            marker_offset,
            marker_offset,
            &((marker.transpose() * weight) * marker),
        );
        target.accum.add_h(
            time_offset,
            world_offset,
            &((time_j.transpose() * weight) * world),
        );
        target.accum.add_h(
            time_offset,
            marker_offset,
            &((time_j.transpose() * weight) * marker),
        );
        target.accum.add_h(
            time_offset,
            time_offset,
            &((time_j.transpose() * weight) * time_j),
        );
        target
            .accum
            .add_b(world_offset, &((world.transpose() * weight) * residual));
        target
            .accum
            .add_b(marker_offset, &((marker.transpose() * weight) * residual));
        target
            .accum
            .add_b(time_offset, &((time_j.transpose() * weight) * residual));
    }
    fn compute_error<const N: usize>(&self, target: &mut ComputeErrorSplineOpt<'_, N>) {
        let calibration = target.common_data.mocap_calibration.unwrap();
        let time = self.timestamp_ns + calibration.mocap_time_offset_ns;
        if time < target.spline.min_time_ns() || time >= target.spline.max_time_ns() {
            return;
        }
        let pose = target.spline.pose(time, None);
        let mocap_pose = product(&product(&calibration.t_moc_w, &pose), &calibration.t_i_mark);
        let residual = se3_logd(&product(&inverse(&self.data), &mocap_pose));
        target.stats.error += target.common_data.mocap_var_inv * residual.norm_squared();
    }
}
