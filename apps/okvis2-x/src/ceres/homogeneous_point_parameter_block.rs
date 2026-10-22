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

//! Estimate and local update operations corresponding to `HomogeneousPointParameterBlock.hpp/.cpp`.

use crate::ceres::{
    homogeneous_point_local_parameterization::HomogeneousPointManifold,
    parameter_block::{Manifold, ParameterBlock, ParameterBlockMetadata},
    parameter_block_sized::ParameterBlockSized,
};
use nalgebra::{Vector3, Vector4};
use std::ops::{Deref, DerefMut};

#[derive(Clone, Default)]
pub struct HomogeneousPointParameterBlock {
    base: ParameterBlockSized<4, 3, Vector4<f64>>,
    initialized: bool,
}
impl HomogeneousPointParameterBlock {
    #[must_use]
    pub fn new(point: Vector4<f64>, id: u64, initialized: bool) -> Self {
        let mut block = Self::default();
        block.set_estimate(point);
        block.set_id(id);
        block.initialized = initialized;
        block
    }
    #[must_use]
    pub fn from_euclidean(point: Vector3<f64>, id: u64, initialized: bool) -> Self {
        Self::new(
            Vector4::new(point[0], point[1], point[2], 1.0),
            id,
            initialized,
        )
    }
    pub fn set_initialized(&mut self, initialized: bool) {
        self.initialized = initialized;
    }
    #[must_use]
    pub fn initialized(&self) -> bool {
        self.initialized
    }
}
impl Deref for HomogeneousPointParameterBlock {
    type Target = ParameterBlockSized<4, 3, Vector4<f64>>;
    fn deref(&self) -> &Self::Target {
        &self.base
    }
}
impl DerefMut for HomogeneousPointParameterBlock {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.base
    }
}
impl ParameterBlock for HomogeneousPointParameterBlock {
    fn metadata(&self) -> &ParameterBlockMetadata {
        &self.base.metadata
    }
    fn metadata_mut(&mut self) -> &mut ParameterBlockMetadata {
        &mut self.base.metadata
    }
    fn set_parameters(&mut self, parameters: &[f64]) {
        assert_eq!(parameters.len(), 4);
        self.base.estimate = Some(Vector4::from_column_slice(parameters));
    }
    fn parameters(&self) -> Option<&[f64]> {
        self.base.estimate.as_ref().map(nalgebra::Matrix::as_slice)
    }
    fn parameters_mut(&mut self) -> Option<&mut [f64]> {
        self.base
            .estimate
            .as_mut()
            .map(nalgebra::Matrix::as_mut_slice)
    }
    fn dimension(&self) -> usize {
        4
    }
    fn minimal_dimension(&self) -> usize {
        3
    }
    fn plus(&self, x: &[f64], delta: &[f64], result: &mut [f64]) {
        Manifold::plus(&HomogeneousPointManifold, x, delta, result);
    }
    fn plus_jacobian(&self, x: &[f64], jacobian: &mut [f64]) {
        Manifold::plus_jacobian(&HomogeneousPointManifold, x, jacobian);
    }
    fn minus(&self, x: &[f64], y: &[f64], delta: &mut [f64]) {
        Manifold::minus(&HomogeneousPointManifold, y, x, delta);
    }
    fn lift_jacobian(&self, x: &[f64], jacobian: &mut [f64]) {
        Manifold::minus_jacobian(&HomogeneousPointManifold, x, jacobian);
    }
    fn type_info(&self) -> &'static str {
        "HomogeneousPointParameterBlock"
    }
}
