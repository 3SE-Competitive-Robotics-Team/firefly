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

//! Weighted camera-depth residual corresponding to `DepthError.hpp`.

use nalgebra::{Matrix3, Matrix4, Quaternion, SMatrix, SVector, Vector4};

use crate::{
    ceres::pose_local_parameterization::PoseManifold,
    kinematics::{
        operators::cross_mx,
        transformation::{normalized_quaternion, raw_rotation_matrix},
    },
};

pub type OneSidedDepthError = DepthErrorT<true>;
pub type DepthError = DepthErrorT<false>;

#[derive(Debug, Default)]
pub struct DepthJacobians<'a> {
    pub pose: Option<&'a mut SMatrix<f64, 1, 7>>,
    pub point: Option<&'a mut SMatrix<f64, 1, 4>>,
    pub extrinsics: Option<&'a mut SMatrix<f64, 1, 7>>,
}

#[derive(Debug, Default)]
pub struct MinimalDepthJacobians<'a> {
    pub pose: Option<&'a mut SMatrix<f64, 1, 6>>,
    pub point: Option<&'a mut SMatrix<f64, 1, 3>>,
    pub extrinsics: Option<&'a mut SMatrix<f64, 1, 6>>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DepthErrorT<const ONE_SIDED: bool> {
    measurement: Option<f64>,
    information: Option<f64>,
    square_root_information: Option<f64>,
}

impl<const ONE_SIDED: bool> DepthErrorT<ONE_SIDED> {
    pub const ONE_SIDED: bool = ONE_SIDED;
    pub const NUM_RESIDUALS: usize = 1;

    #[must_use]
    pub fn new(measurement: f64, stdev: f64) -> Self {
        let information = 1.0 / (stdev * stdev);
        Self {
            measurement: Some(measurement),
            information: Some(information),
            square_root_information: Some(information.sqrt()),
        }
    }

    pub fn set_measurement(&mut self, measurement: f64) {
        self.measurement = Some(measurement);
    }

    pub fn set_stdev(&mut self, stdev: f64) {
        let information = 1.0 / (stdev * stdev);
        self.information = Some(information);
        self.square_root_information = Some(information.sqrt());
    }

    #[must_use]
    pub fn measurement(&self) -> Option<f64> {
        self.measurement
    }
    #[must_use]
    pub fn information(&self) -> Option<f64> {
        self.information
    }

    pub fn evaluate(
        &self,
        parameters: (&SVector<f64, 7>, &Vector4<f64>, &SVector<f64, 7>),
        residual: &mut f64,
        jacobians: Option<DepthJacobians<'_>>,
    ) -> bool {
        self.evaluate_with_minimal_jacobians(parameters, residual, jacobians, None)
    }

    /// Poses use `[tx, ty, tz, qx, qy, qz, qw]`; the point is homogeneous in world coordinates.
    /// Minimal outputs are written only when the corresponding ambient output is requested.
    /// The regular specialization retains `p_z = 0` in its residual branch.
    /// Returns false if measurement or weighting has not been initialized.
    pub fn evaluate_with_minimal_jacobians(
        &self,
        parameters: (&SVector<f64, 7>, &Vector4<f64>, &SVector<f64, 7>),
        residual: &mut f64,
        jacobians: Option<DepthJacobians<'_>>,
        jacobians_minimal: Option<MinimalDepthJacobians<'_>>,
    ) -> bool {
        let (Some(measurement), Some(square_root_information)) =
            (self.measurement, self.square_root_information)
        else {
            return false;
        };
        let (pose, hp_w, extrinsics) = parameters;
        let t_ws_w = pose.fixed_rows::<3>(0).into_owned();
        let t_sc_s = extrinsics.fixed_rows::<3>(0).into_owned();
        let c_sw = rotation_from_parameters(pose).transpose();
        let c_cs = rotation_from_parameters(extrinsics).transpose();
        let mut t_cs = Matrix4::identity();
        t_cs.fixed_view_mut::<3, 3>(0, 0).copy_from(&c_cs);
        t_cs.fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&(-c_cs * t_sc_s));
        let mut t_sw = Matrix4::identity();
        t_sw.fixed_view_mut::<3, 3>(0, 0).copy_from(&c_sw);
        t_sw.fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&(-c_sw * t_ws_w));
        let hp_s = t_sw * hp_w;
        let hp_c = t_cs * hp_s;
        let mut p_z = 0.0;
        let mut ignore = false;
        if hp_c[3].abs() < 1.0e-16 {
            ignore = true;
        } else if ONE_SIDED {
            p_z = hp_c[2] / hp_c[3];
            if p_z > measurement {
                ignore = true;
            }
        }
        let error = measurement - p_z;
        *residual = if ignore {
            0.0
        } else {
            square_root_information * error
        };

        if let Some(jacobians) = jacobians {
            let jh_weighted =
                SMatrix::<f64, 1, 4>::new(0.0, 0.0, square_root_information / hp_c[3], 0.0);
            let DepthJacobians {
                pose: j0,
                point: j1,
                extrinsics: j2,
            } = jacobians;
            let MinimalDepthJacobians {
                pose: j0_minimal,
                point: j1_minimal,
                extrinsics: j2_minimal,
            } = jacobians_minimal.unwrap_or_default();
            if let Some(j0) = j0 {
                if ignore {
                    j0.fill(0.0);
                    if let Some(j0_minimal) = j0_minimal {
                        j0_minimal.fill(0.0);
                    }
                } else {
                    let p = hp_w.fixed_rows::<3>(0) - t_ws_w * hp_w[3];
                    let mut j = SMatrix::<f64, 4, 6>::zeros();
                    j.fixed_view_mut::<3, 3>(0, 0).copy_from(&(c_sw * hp_w[3]));
                    j.fixed_view_mut::<3, 3>(0, 3)
                        .copy_from(&(-c_sw * cross_mx(&p)));
                    let minimal = jh_weighted * t_cs * j;
                    *j0 = minimal * PoseManifold::minus_jacobian(pose);
                    if let Some(j0_minimal) = j0_minimal {
                        *j0_minimal = minimal;
                    }
                }
            }
            Self::evaluate_point_jacobian(ignore, jh_weighted, (t_cs, t_sw), (j1, j1_minimal));
            if let Some(j2) = j2 {
                if ignore {
                    j2.fill(0.0);
                    if let Some(j2_minimal) = j2_minimal {
                        j2_minimal.fill(0.0);
                    }
                } else {
                    let p = hp_s.fixed_rows::<3>(0) - t_sc_s * hp_s[3];
                    let mut j = SMatrix::<f64, 4, 6>::zeros();
                    j.fixed_view_mut::<3, 3>(0, 0).copy_from(&(c_cs * hp_s[3]));
                    j.fixed_view_mut::<3, 3>(0, 3)
                        .copy_from(&(-c_cs * cross_mx(&p)));
                    let minimal = jh_weighted * j;
                    *j2 = minimal * PoseManifold::minus_jacobian(extrinsics);
                    if let Some(j2_minimal) = j2_minimal {
                        *j2_minimal = minimal;
                    }
                }
            }
        }
        true
    }
    fn evaluate_point_jacobian(
        ignore: bool,
        jh_weighted: SMatrix<f64, 1, 4>,
        (t_cs, t_sw): (Matrix4<f64>, Matrix4<f64>),
        (j1, j1_minimal): (
            Option<&mut SMatrix<f64, 1, 4>>,
            Option<&mut SMatrix<f64, 1, 3>>,
        ),
    ) {
        if let Some(j1) = j1 {
            if ignore {
                j1.fill(0.0);
                if let Some(j1_minimal) = j1_minimal {
                    j1_minimal.fill(0.0);
                }
            } else {
                let camera_from_world = t_cs * t_sw;
                *j1 = -jh_weighted * camera_from_world;
                if let Some(j1_minimal) = j1_minimal {
                    let mut s = SMatrix::<f64, 4, 3>::zeros();
                    s.fixed_view_mut::<3, 3>(0, 0)
                        .copy_from(&Matrix3::identity());
                    *j1_minimal = *j1 * s;
                }
            }
        }
    }

    #[must_use]
    pub const fn residual_dim(&self) -> usize {
        Self::NUM_RESIDUALS
    }
    #[must_use]
    pub const fn parameter_blocks(&self) -> usize {
        3
    }
    #[must_use]
    pub fn parameter_block_dim(&self, parameter_block_id: usize) -> usize {
        [7, 4, 7][parameter_block_id]
    }
    #[must_use]
    pub const fn type_info(&self) -> &'static str {
        "DepthError"
    }
}

fn rotation_from_parameters(parameters: &SVector<f64, 7>) -> Matrix3<f64> {
    let q = Quaternion::new(parameters[6], parameters[3], parameters[4], parameters[5]);
    raw_rotation_matrix(&normalized_quaternion(q))
}
