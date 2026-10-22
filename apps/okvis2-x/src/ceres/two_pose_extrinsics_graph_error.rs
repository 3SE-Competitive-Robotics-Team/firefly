/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Two-pose visual Schur factor retaining camera-extrinsics tangent coordinates.

use crate::{
    ceres::{
        error_interface::{ErrorEvaluation, ErrorInterface},
        two_pose_graph_error::{
            Observation, TwoPoseGraphError, TwoPoseGraphErrorConst, evaluate_graph, system_strength,
        },
    },
    frames::frame_typedefs::{KeypointIdentifier, StateId},
    kinematics::transformation::Transformation,
};
use firefly_error::Result;
use nalgebra::{DMatrix, DVector};
use std::ops::{Deref, DerefMut};

pub struct TwoPoseExtrinsicsGraphError {
    pub base: TwoPoseGraphError,
    pub h00: DMatrix<f64>,
    pub b0: DVector<f64>,
    pub delta_x: Option<DVector<f64>>,
    pub j: Option<DMatrix<f64>>,
    pub error_computation_valid: bool,
}
impl Deref for TwoPoseExtrinsicsGraphError {
    type Target = TwoPoseGraphError;
    fn deref(&self) -> &Self::Target {
        &self.base
    }
}
impl DerefMut for TwoPoseExtrinsicsGraphError {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.base
    }
}
impl TwoPoseExtrinsicsGraphError {
    #[must_use]
    pub fn new(
        reference_pose_id: StateId,
        other_pose_id: StateId,
        num_cams: usize,
        stay_const: bool,
    ) -> Self {
        Self {
            base: TwoPoseGraphError::new(reference_pose_id, other_pose_id, num_cams, stay_const),
            h00: DMatrix::zeros(0, 0),
            b0: DVector::zeros(0),
            delta_x: None,
            j: None,
            error_computation_valid: false,
        }
    }
    /// The shared observation kernel builds all pose/extrinsics cross blocks and
    /// eliminates each landmark using the same spectral inverse as the fixed factor.
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn compute(&mut self) -> Result<bool> {
        if self.is_computed {
            return Ok(true);
        }
        let (h, b, j, dx) = self.base.compute_system(true)?;
        self.h00 = h;
        self.b0 = b;
        self.j = Some(j);
        self.delta_x = Some(dx);
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn convert_to_reprojection_errors(
        &mut self,
        observations: &mut Vec<Observation>,
        duplicates: &mut Vec<KeypointIdentifier>,
    ) -> Result<bool> {
        self.base.convert_observations(observations, duplicates)?;
        if let Some(j) = &mut self.j {
            j.fill(0.0);
        }
        self.h00.fill(0.0);
        self.b0.fill(0.0);
        Ok(true)
    }
    #[must_use]
    pub fn strength(&self) -> f64 {
        if self.is_computed {
            system_strength(&self.h00)
        } else {
            0.0
        }
    }
    #[must_use]
    pub fn clone_two_pose_graph_error_const(&self) -> Option<TwoPoseExtrinsicsGraphErrorConst> {
        Some(TwoPoseExtrinsicsGraphErrorConst::new(
            self.delta_x.clone()?,
            self.j.clone()?,
            self.linearisation_point_t_s0s1.clone(),
            self.linearisation_points_t_sc.clone(),
        ))
    }
}
#[derive(Clone)]
pub struct TwoPoseExtrinsicsGraphErrorConst {
    pub delta_x: DVector<f64>,
    pub j: DMatrix<f64>,
    pub linearisation_point_t_s0s1: Transformation,
    pub linearisation_points_t_sc: Vec<Option<Transformation>>,
}
impl TwoPoseExtrinsicsGraphErrorConst {
    #[must_use]
    pub fn new(
        delta_x: DVector<f64>,
        j: DMatrix<f64>,
        linearisation_point_t_s0s1: Transformation,
        linearisation_points_t_sc: Vec<Option<Transformation>>,
    ) -> Self {
        Self {
            delta_x,
            j,
            linearisation_point_t_s0s1,
            linearisation_points_t_sc,
        }
    }
}
impl TwoPoseGraphErrorConst for TwoPoseExtrinsicsGraphErrorConst {
    fn snapshot_send(&self) -> Box<dyn TwoPoseGraphErrorConst + Send> {
        Box::new(self.clone())
    }
}
impl ErrorInterface for TwoPoseExtrinsicsGraphError {
    fn residual_dim(&self) -> usize {
        6 + self.extrinsics_size
    }
    fn parameter_blocks(&self) -> usize {
        2 + self.extrinsics_parameter_block_infos.len()
    }
    fn parameter_block_dim(&self, block: usize) -> usize {
        assert!(block < self.parameter_blocks());
        7
    }
    fn type_info(&self) -> &'static str {
        "TwoPoseExtrinsicsGraphError"
    }
    fn evaluate_with_minimal_jacobians(
        &self,
        parameters: &[&[f64]],
        jacobians: Option<&[bool]>,
        minimal: Option<&[bool]>,
    ) -> Option<ErrorEvaluation> {
        if !self.is_computed {
            return None;
        }
        let indices: Vec<_> = self
            .extrinsics_parameter_block_infos
            .iter()
            .map(|i| i.as_ref().map(|i| i.idx))
            .collect();
        evaluate_graph(
            self.delta_x.as_ref()?,
            self.j.as_ref()?,
            &self.linearisation_point_t_s0s1,
            &self.linearisation_points_t_sc,
            Some(&indices),
            parameters,
            (jacobians, minimal),
        )
    }
}
impl ErrorInterface for TwoPoseExtrinsicsGraphErrorConst {
    fn residual_dim(&self) -> usize {
        self.delta_x.len()
    }
    fn parameter_blocks(&self) -> usize {
        2 + (self.delta_x.len() - 6) / 6
    }
    fn parameter_block_dim(&self, block: usize) -> usize {
        assert!(block < self.parameter_blocks());
        7
    }
    fn type_info(&self) -> &'static str {
        "TwoPoseExtrinsicsGraphErrorConst"
    }
    fn evaluate_with_minimal_jacobians(
        &self,
        parameters: &[&[f64]],
        jacobians: Option<&[bool]>,
        minimal: Option<&[bool]>,
    ) -> Option<ErrorEvaluation> {
        evaluate_graph(
            &self.delta_x,
            &self.j,
            &self.linearisation_point_t_s0s1,
            &self.linearisation_points_t_sc,
            None,
            parameters,
            (jacobians, minimal),
        )
    }
}
