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

//! Occupancy-field alignment factor corresponding to `SubmapIcpError.cpp`.

use super::pose_local_parameterization::PoseManifold;
use crate::kinematics::{
    operators::cross_mx,
    transformation::{Transformation, raw_rotation_matrix},
};
use nalgebra::{Quaternion, SMatrix, SVector, Vector3, Vector4};
use std::cell::Cell;

/// Safe occupancy queries return no value outside the map's supported interpolation domain.
pub trait OccupancySubmap {
    fn grad_field_mean_occup(&self, point: &Vector3<f32>) -> Option<Vector3<f32>>;
    fn interp_field_mean_occup(&self, point: &Vector3<f32>) -> Option<f32>;
    fn log_odd_min(&self) -> f32;
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn snapshot(&self) -> firefly_error::Result<std::sync::Arc<dyn OccupancySubmap + Send + Sync>>;
    /// Stable storage identity permits one frozen map per graph snapshot.
    fn snapshot_identity(&self) -> Option<usize> {
        None
    }
}

#[derive(Debug)]
pub struct SubmapIcpError<'a, M: ?Sized> {
    submap: &'a M,
    point_b: Vector3<f64>,
    sigma_measurement: f64,
    sigma_map: Cell<Option<f64>>,
}
impl<'a, M: OccupancySubmap + ?Sized> SubmapIcpError<'a, M> {
    pub const NUM_RESIDUALS: usize = 1;
    pub fn new(submap: &'a M, point_b: Vector3<f64>, sigma_measurement: f64) -> Self {
        Self {
            submap,
            point_b,
            sigma_measurement,
            sigma_map: Cell::new(None),
        }
    }
    pub fn set_point_b(&mut self, point_b: Vector3<f64>) {
        self.point_b = point_b;
    }
    pub fn submap(&self) -> &M {
        self.submap
    }
    pub fn point_b(&self) -> &Vector3<f64> {
        &self.point_b
    }
    pub fn evaluate(
        &self,
        parameters: [&SVector<f64, 7>; 2],
        residual: &mut f64,
        jacobians: Option<[Option<&mut SMatrix<f64, 1, 7>>; 2]>,
    ) -> bool {
        self.evaluate_with_minimal_jacobians(parameters, residual, jacobians, [None, None])
    }
    /// With no ambient Jacobian array, the residual is zero and map queries are skipped.
    /// Rejected map queries zero ambient outputs and leave minimal outputs untouched.
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn evaluate_with_minimal_jacobians(
        &self,
        parameters: [&SVector<f64, 7>; 2],
        residual: &mut f64,
        jacobians: Option<[Option<&mut SMatrix<f64, 1, 7>>; 2]>,
        jacobians_minimal: [Option<&mut SMatrix<f64, 1, 6>>; 2],
    ) -> bool {
        let Some(jacobians) = jacobians else {
            *residual = 0.0;
            return true;
        };
        let t_ws_a = parameters[0].fixed_rows::<3>(0).into_owned();
        let t_ws_b = parameters[1].fixed_rows::<3>(0).into_owned();
        let c_ws_a = raw_rotation_matrix(&quaternion(parameters[0]));
        let c_ws_b = raw_rotation_matrix(&quaternion(parameters[1]));
        let point_a = self.point_in_a(parameters);
        let point_a_float = point_a.cast::<f32>();
        let gradient_float = self.submap.grad_field_mean_occup(&point_a_float);
        let Some(gradient_float) = gradient_float.filter(|gradient| {
            f64::from(gradient.norm()).partial_cmp(&1e-3) != Some(std::cmp::Ordering::Less)
        }) else {
            *residual = 0.0;
            for jacobian in jacobians.into_iter().flatten() {
                jacobian.fill(0.0);
            }
            return true;
        };
        let gradient = gradient_float.cast::<f64>();
        let gradient_norm = gradient.norm();
        let Some(occupancy_float) = self.submap.interp_field_mean_occup(&point_a_float) else {
            *residual = 0.0;
            for jacobian in jacobians.into_iter().flatten() {
                jacobian.fill(0.0);
            }
            return true;
        };
        let occupancy = f64::from(occupancy_float);
        self.sigma_map.set(Some(
            f64::from(self.submap.log_odd_min()).abs() / (3.0 * gradient_norm),
        ));
        let sigma_map = self
            .sigma_map
            .get()
            .expect("map uncertainty has been evaluated");
        let sigma_total =
            (sigma_map * sigma_map + self.sigma_measurement * self.sigma_measurement).sqrt();
        let square_root_information = 1.0 / sigma_total;
        *residual = square_root_information * occupancy / gradient_norm;
        let [j0, j1] = jacobians;
        let [j0_minimal, j1_minimal] = jacobians_minimal;
        if let Some(j0) = j0 {
            let de_dp_a = (square_root_information * gradient / gradient_norm).transpose();
            let mut dpa_dtwsa = SMatrix::<f64, 3, 6>::zeros();
            dpa_dtwsa
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&(-c_ws_a.transpose()));
            dpa_dtwsa.fixed_view_mut::<3, 3>(0, 3).copy_from(
                &(c_ws_a.transpose() * cross_mx(&(c_ws_b * self.point_b + t_ws_b - t_ws_a))),
            );
            let minimal = de_dp_a * dpa_dtwsa;
            *j0 = minimal * PoseManifold::minus_jacobian(parameters[0]);
            if let Some(j0_minimal) = j0_minimal {
                *j0_minimal = minimal;
            }
        }
        if let Some(j1) = j1 {
            let de_dp_a = (square_root_information * gradient / gradient_norm).transpose();
            let mut dpa_dtwsb = SMatrix::<f64, 3, 6>::zeros();
            dpa_dtwsb
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&c_ws_a.transpose());
            dpa_dtwsb
                .fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&(-c_ws_a.transpose() * cross_mx(&(c_ws_b * self.point_b))));
            let minimal = de_dp_a * dpa_dtwsb;
            *j1 = minimal * PoseManifold::minus_jacobian(parameters[1]);
            if let Some(j1_minimal) = j1_minimal {
                *j1_minimal = minimal;
            }
        }
        true
    }
    pub fn non_zero_field_and_gradient(
        &self,
        parameters: [&SVector<f64, 7>; 2],
        field_value: &mut f32,
        gradient_norm: &mut f32,
    ) -> (bool, bool) {
        let point_a = self.point_in_a(parameters).cast::<f32>();
        let mut non_zero_gradient = true;
        if let Some(gradient) = self.submap.grad_field_mean_occup(&point_a) {
            if f64::from(gradient.norm()) < 1e-3 {
                *gradient_norm = 0.0;
                non_zero_gradient = false;
            } else {
                *gradient_norm = gradient.norm();
            }
        } else {
            *gradient_norm = 0.0;
            non_zero_gradient = false;
        }
        let non_zero_field = if let Some(occupancy) = self.submap.interp_field_mean_occup(&point_a)
        {
            *field_value = occupancy;
            true
        } else {
            *field_value = 0.0;
            false
        };
        (non_zero_field, non_zero_gradient)
    }
    fn point_in_a(&self, parameters: [&SVector<f64, 7>; 2]) -> Vector3<f64> {
        let pose_a = Transformation::new(
            parameters[0].fixed_rows::<3>(0).into_owned(),
            quaternion(parameters[0]),
        );
        let pose_b = Transformation::new(
            parameters[1].fixed_rows::<3>(0).into_owned(),
            quaternion(parameters[1]),
        );
        let a_from_b = pose_a.inverse().compose(&pose_b);
        let point_b = Vector4::new(self.point_b[0], self.point_b[1], self.point_b[2], 1.0);
        (a_from_b.matrix() * point_b)
            .fixed_rows::<3>(0)
            .into_owned()
    }
    pub const fn residual_dim(&self) -> usize {
        1
    }
    pub const fn parameter_blocks(&self) -> usize {
        2
    }
    pub fn parameter_block_dim(&self, block: usize) -> usize {
        [7, 7][block]
    }
    pub const fn type_info(&self) -> &'static str {
        "SubmapIcpError"
    }
}
fn quaternion(parameters: &SVector<f64, 7>) -> Quaternion<f64> {
    Quaternion::new(parameters[6], parameters[3], parameters[4], parameters[5])
}

impl OccupancySubmap for crate::mapping::map_typedefs::OccupancyMap {
    fn grad_field_mean_occup(&self, point: &Vector3<f32>) -> Option<Vector3<f32>> {
        use crate::mapping::submapping_utils::{MeanOccupancyMap, Safe};
        self.grad_mean_occupancy(point, 0, Safe::On)
            .map(|(value, _)| value)
    }
    fn interp_field_mean_occup(&self, point: &Vector3<f32>) -> Option<f32> {
        use crate::mapping::submapping_utils::{MeanOccupancyMap, Safe};
        self.interp_mean_occupancy(point, 0, Safe::On)
            .map(|(value, _)| value)
    }
    fn log_odd_min(&self) -> f32 {
        match self.read() {
            Ok(map) => map.data_config.field.log_odd_min,
            Err(error) => {
                log::error!("submap occupancy configuration failed: {error}");
                f32::NAN
            }
        }
    }
    fn snapshot_identity(&self) -> Option<usize> {
        Some(self.identity())
    }
    fn snapshot(&self) -> firefly_error::Result<std::sync::Arc<dyn OccupancySubmap + Send + Sync>> {
        Ok(std::sync::Arc::new(
            crate::mapping::map_typedefs::OccupancyMap::snapshot(self)?,
        ))
    }
}
