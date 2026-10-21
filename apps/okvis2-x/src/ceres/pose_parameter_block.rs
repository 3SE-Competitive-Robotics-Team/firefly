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

//! Estimate and local update operations corresponding to `PoseParameterBlock.hpp/.cpp`.

use crate::{
    ceres::{
        parameter_block::{Manifold, ParameterBlock, ParameterBlockMetadata},
        parameter_block_sized::ParameterBlockSized,
        pose_local_parameterization::PoseManifold,
    },
    kinematics::transformation::{Transformation, TransformationCacheless},
};
use jiff::Timestamp;
use nalgebra::SVector;
use std::ops::{Deref, DerefMut};

#[derive(Clone)]
pub struct PoseParameterBlock {
    base: ParameterBlockSized<7, 6, TransformationCacheless>,
    timestamp: Timestamp,
}
impl Default for PoseParameterBlock {
    fn default() -> Self {
        let mut base = ParameterBlockSized::default();
        base.set_estimate(TransformationCacheless::default());
        Self {
            base,
            timestamp: Timestamp::UNIX_EPOCH,
        }
    }
}
impl PoseParameterBlock {
    #[must_use]
    pub fn new(t_ws: &Transformation, id: u64, timestamp: Timestamp) -> Self {
        let mut block = Self::default();
        block.set_estimate(TransformationCacheless::from_other(t_ws));
        block.set_id(id);
        block.timestamp = timestamp;
        block
    }
    pub fn set_timestamp(&mut self, timestamp: Timestamp) {
        self.timestamp = timestamp;
    }
    #[must_use]
    pub fn timestamp(&self) -> Timestamp {
        self.timestamp
    }
}
impl Deref for PoseParameterBlock {
    type Target = ParameterBlockSized<7, 6, TransformationCacheless>;
    fn deref(&self) -> &Self::Target {
        &self.base
    }
}
impl DerefMut for PoseParameterBlock {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.base
    }
}
impl ParameterBlock for PoseParameterBlock {
    fn metadata(&self) -> &ParameterBlockMetadata {
        &self.base.metadata
    }
    fn metadata_mut(&mut self) -> &mut ParameterBlockMetadata {
        &mut self.base.metadata
    }
    fn set_parameters(&mut self, parameters: &[f64]) {
        assert_eq!(parameters.len(), 7);
        let mut estimate = TransformationCacheless::default();
        estimate.set_coeffs(&SVector::<f64, 7>::from_column_slice(parameters));
        self.base.estimate = Some(estimate);
    }
    fn parameters(&self) -> Option<&[f64]> {
        self.base
            .estimate
            .as_ref()
            .map(|estimate| estimate.coeffs().as_slice())
    }
    fn parameters_mut(&mut self) -> Option<&mut [f64]> {
        self.base
            .estimate
            .as_mut()
            .map(|estimate| estimate.coeffs_mut().as_mut_slice())
    }
    fn dimension(&self) -> usize {
        7
    }
    fn minimal_dimension(&self) -> usize {
        6
    }
    fn plus(&self, x: &[f64], delta: &[f64], result: &mut [f64]) {
        Manifold::plus(&PoseManifold, x, delta, result);
    }
    fn plus_jacobian(&self, x: &[f64], jacobian: &mut [f64]) {
        Manifold::plus_jacobian(&PoseManifold, x, jacobian);
    }
    fn minus(&self, x: &[f64], y: &[f64], delta: &mut [f64]) {
        Manifold::minus(&PoseManifold, y, x, delta);
    }
    fn lift_jacobian(&self, x: &[f64], jacobian: &mut [f64]) {
        Manifold::minus_jacobian(&PoseManifold, x, jacobian);
    }
    fn type_info(&self) -> &'static str {
        "PoseParameterBlock"
    }
}
