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

//! Estimate and local update operations corresponding to `SpeedAndBiasParameterBlock.hpp/.cpp`.

use crate::ceres::{
    parameter_block::{ParameterBlock, ParameterBlockMetadata},
    parameter_block_sized::ParameterBlockSized,
};
pub use crate::frames::frame_typedefs::SpeedAndBias;
use jiff::Timestamp;
use std::ops::{Deref, DerefMut};

#[derive(Clone)]
pub struct SpeedAndBiasParameterBlock {
    base: ParameterBlockSized<9, 9, SpeedAndBias>,
    timestamp: Timestamp,
}
impl Default for SpeedAndBiasParameterBlock {
    fn default() -> Self {
        Self {
            base: ParameterBlockSized::default(),
            timestamp: Timestamp::UNIX_EPOCH,
        }
    }
}
impl SpeedAndBiasParameterBlock {
    #[must_use]
    pub fn new(speed_and_bias: SpeedAndBias, id: u64, timestamp: Timestamp) -> Self {
        let mut block = Self::default();
        block.set_estimate(speed_and_bias);
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
impl Deref for SpeedAndBiasParameterBlock {
    type Target = ParameterBlockSized<9, 9, SpeedAndBias>;
    fn deref(&self) -> &Self::Target {
        &self.base
    }
}
impl DerefMut for SpeedAndBiasParameterBlock {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.base
    }
}
impl ParameterBlock for SpeedAndBiasParameterBlock {
    fn metadata(&self) -> &ParameterBlockMetadata {
        &self.base.metadata
    }
    fn metadata_mut(&mut self) -> &mut ParameterBlockMetadata {
        &mut self.base.metadata
    }
    fn set_parameters(&mut self, parameters: &[f64]) {
        assert_eq!(parameters.len(), 9);
        self.base.estimate = Some(SpeedAndBias::from_column_slice(parameters));
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
        9
    }
    fn minimal_dimension(&self) -> usize {
        9
    }
    fn plus(&self, x: &[f64], delta: &[f64], result: &mut [f64]) {
        assert_eq!(x.len(), 9);
        assert_eq!(delta.len(), 9);
        assert_eq!(result.len(), 9);
        for i in 0..9 {
            result[i] = x[i] + delta[i];
        }
    }
    fn plus_jacobian(&self, _x: &[f64], jacobian: &mut [f64]) {
        assert_eq!(jacobian.len(), 81);
        jacobian.fill(0.0);
        for i in 0..9 {
            jacobian[i * 9 + i] = 1.0;
        }
    }
    fn minus(&self, x: &[f64], y: &[f64], delta: &mut [f64]) {
        assert_eq!(x.len(), 9);
        assert_eq!(y.len(), 9);
        assert_eq!(delta.len(), 9);
        for i in 0..9 {
            delta[i] = y[i] - x[i];
        }
    }
    fn lift_jacobian(&self, x: &[f64], jacobian: &mut [f64]) {
        self.plus_jacobian(x, jacobian);
    }
    fn type_info(&self) -> &'static str {
        "SpeedAndBiasParameterBlock"
    }
}
