// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/optimization/poses_linearize.h`.
use super::{
    accumulator::SparseHashAccumulator,
    linearize::{CalibCommonData, PoseCalibH, ReprojectionStats},
};
use crate::io::dataset_io::AprilgridCornersData;
use crate::utils::sophus_utils::se3_inverse;
use crate::utils::sophus_utils::se3_matrix;
use crate::utils::sophus_utils::se3_product;
use crate::utils::sophus_utils::so3_matrix;
use firefly_calibration::camera::generic_camera::GenericCamera;
use nalgebra::{Isometry3, Matrix3, SMatrix, Vector2, Vector4};
use rayon::prelude::*;
use std::collections::BTreeMap;

pub(crate) fn adjoint(pose: &Isometry3<f64>) -> SMatrix<f64, 6, 6> {
    let r = so3_matrix(&pose.rotation);
    let t = pose.translation.vector;
    let skew = Matrix3::new(0.0, -t.z, t.y, t.z, 0.0, -t.x, -t.y, t.x, 0.0);
    let mut result = SMatrix::<f64, 6, 6>::zeros();
    result.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    result.fixed_view_mut::<3, 3>(3, 3).copy_from(&r);
    result.fixed_view_mut::<3, 3>(0, 3).copy_from(&(skew * r));
    result
}

pub struct LinearizePosesOpt<'a> {
    pub accum: SparseHashAccumulator<f64>,
    pub stats: ReprojectionStats,
    pub opt_size: usize,
    pub timestamp_to_pose: &'a BTreeMap<i64, Isometry3<f64>>,
    pub common_data: CalibCommonData<'a>,
}
impl<'a> LinearizePosesOpt<'a> {
    #[must_use]
    pub fn new(
        opt_size: usize,
        poses: &'a BTreeMap<i64, Isometry3<f64>>,
        common_data: CalibCommonData<'a>,
    ) -> Self {
        let mut accum = SparseHashAccumulator::default();
        accum.reset(opt_size);
        Self {
            accum,
            stats: ReprojectionStats::default(),
            opt_size,
            timestamp_to_pose: poses,
            common_data,
        }
    }
    pub fn join(&mut self, other: &Self) {
        self.accum.join(&other.accum);
        self.stats.join(&other.stats);
    }
    pub fn parallel_linearize(&mut self, observations: &[AprilgridCornersData]) {
        let result = observations
            .par_iter()
            .fold(
                || {
                    Self::new(
                        self.opt_size,
                        self.timestamp_to_pose,
                        self.common_data.clone(),
                    )
                },
                |mut state, observation| {
                    state.linearize(observation);
                    state
                },
            )
            .reduce(
                || {
                    Self::new(
                        self.opt_size,
                        self.timestamp_to_pose,
                        self.common_data.clone(),
                    )
                },
                |mut a, b| {
                    a.join(&b);
                    a
                },
            );
        self.join(&result);
    }
    /// # Panics
    /// Requires a registered pose and valid camera/grid indices.
    pub fn linearize(&mut self, observation: &AprilgridCornersData) {
        let camera_index = usize::try_from(observation.cam_id).expect("nonnegative camera ID");
        macro_rules! visit {
            ($camera:expr, $n:literal) => {
                self.linearize_camera::<$n>(observation, camera_index, |p, pixel, jp, ji| {
                    $camera.project(p, pixel, jp, ji)
                })
            };
        }
        // Clone the camera to keep the immutable calibration borrow separate
        // from the accumulator's mutable borrow.
        match self.common_data.calibration.intrinsics[camera_index].clone() {
            GenericCamera::ExtendedUnified(c) => visit!(c, 6),
            GenericCamera::DoubleSphere(c) => visit!(c, 6),
            GenericCamera::KannalaBrandt4(c) => visit!(c, 8),
            GenericCamera::Unified(c) => visit!(c, 5),
            GenericCamera::Pinhole(c) => visit!(c, 4),
            GenericCamera::PinholeRadtan8(c) => visit!(c, 12),
        }
    }
    fn linearize_camera<const N: usize>(
        &mut self,
        observation: &AprilgridCornersData,
        camera: usize,
        project: impl Fn(
            &Vector4<f64>,
            &mut Vector2<f64>,
            Option<&mut SMatrix<f64, 2, 4>>,
            Option<&mut SMatrix<f64, 2, N>>,
        ) -> bool,
    ) {
        let extrinsic = self.common_data.calibration.t_i_c[camera];
        let t_c_w = se3_matrix(&se3_inverse(&se3_product(
            &self.timestamp_to_pose[&observation.timestamp_ns],
            &extrinsic,
        )));
        let mut block = PoseCalibH::default();
        for (corner, &id) in observation.corner_pos.iter().zip(&observation.corner_id) {
            self.common_data.linearize_point(
                corner,
                usize::try_from(id).expect("nonnegative grid ID"),
                &t_c_w,
                &project,
                Some(&mut block),
                &mut self.stats,
            );
        }
        let adj = -adjoint(&se3_inverse(&extrinsic));
        let po = self.common_data.offset_poses[&observation.timestamp_ns];
        let co = self.common_data.offset_t_i_c[camera];
        let io = self.common_data.offset_intrinsics[camera];
        self.accum
            .add_h(po, po, &(adj.transpose() * block.h_pose_accum * adj));
        self.accum
            .add_b(po, &(adj.transpose() * block.b_pose_accum));
        if camera > 0 {
            self.accum.add_h(co, po, &(-block.h_pose_accum * adj));
            self.accum.add_h(co, co, &block.h_pose_accum);
            self.accum.add_b(co, &(-block.b_pose_accum));
        }
        if self.common_data.opt_intrinsics {
            self.accum.add_h(io, po, &(block.h_intr_pose_accum * adj));
            if camera > 0 {
                self.accum.add_h(io, co, &(-block.h_intr_pose_accum));
            }
            self.accum.add_h(io, io, &block.h_intr_accum);
            self.accum.add_b(io, &block.b_intr_accum);
        }
    }
}

pub struct ComputeErrorPosesOpt<'a> {
    pub stats: ReprojectionStats,
    pub opt_size: usize,
    pub timestamp_to_pose: &'a BTreeMap<i64, Isometry3<f64>>,
    pub common_data: CalibCommonData<'a>,
}
impl<'a> ComputeErrorPosesOpt<'a> {
    #[must_use]
    pub fn new(
        opt_size: usize,
        poses: &'a BTreeMap<i64, Isometry3<f64>>,
        common_data: CalibCommonData<'a>,
    ) -> Self {
        Self {
            stats: ReprojectionStats::default(),
            opt_size,
            timestamp_to_pose: poses,
            common_data,
        }
    }
    pub fn join(&mut self, other: &Self) {
        self.stats.join(&other.stats);
    }
    /// # Panics
    /// Requires valid observation indices and a pose for every timestamp.
    pub fn compute(&mut self, observations: &[AprilgridCornersData]) {
        let stats = observations
            .par_iter()
            .map(|observation| {
                let mut stats = ReprojectionStats::default();
                let ci = usize::try_from(observation.cam_id).expect("nonnegative camera ID");
                let cam = &self.common_data.calibration.intrinsics[ci];
                let transform = se3_matrix(&se3_inverse(&se3_product(
                    &self.timestamp_to_pose[&observation.timestamp_ns],
                    &self.common_data.calibration.t_i_c[ci],
                )));
                for (corner, &id) in observation.corner_pos.iter().zip(&observation.corner_id) {
                    self.common_data.linearize_point::<0>(
                        corner,
                        usize::try_from(id).expect("nonnegative grid ID"),
                        &transform,
                        |p, pixel, jp, _| cam.project(p, pixel, jp),
                        None,
                        &mut stats,
                    );
                }
                stats
            })
            .reduce(ReprojectionStats::default, |mut a, b| {
                a.join(&b);
                a
            });
        self.stats.join(&stats);
    }
}
