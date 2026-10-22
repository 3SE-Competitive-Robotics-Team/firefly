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

//! Typed estimate storage corresponding to `ParameterBlockSized.hpp`.

use crate::ceres::parameter_block::{Manifold, ParameterBlockMetadata};
use std::sync::Arc;

/// Numeric Eigen estimates are absent until assigned; no zero estimate is invented.
#[derive(Clone)]
pub struct ParameterBlockSized<const DIM: usize, const MIN_DIM: usize, T> {
    pub(crate) metadata: ParameterBlockMetadata,
    pub(crate) estimate: Option<T>,
}
impl<const DIM: usize, const MIN_DIM: usize, T> Default for ParameterBlockSized<DIM, MIN_DIM, T> {
    fn default() -> Self {
        Self {
            metadata: ParameterBlockMetadata::default(),
            estimate: None,
        }
    }
}
impl<const DIM: usize, const MIN_DIM: usize, T> ParameterBlockSized<DIM, MIN_DIM, T> {
    pub const DIMENSION: usize = DIM;
    pub const MINIMAL_DIMENSION: usize = MIN_DIM;
    pub fn set_estimate(&mut self, estimate: T) {
        self.estimate = Some(estimate);
    }
    pub fn estimate(&self) -> Option<&T> {
        self.estimate.as_ref()
    }
    pub const fn dimension(&self) -> usize {
        DIM
    }
    pub const fn minimal_dimension(&self) -> usize {
        MIN_DIM
    }
    pub fn set_id(&mut self, id: u64) {
        self.metadata.id = id;
    }
    pub fn id(&self) -> u64 {
        self.metadata.id
    }
    pub fn set_fixed(&mut self, fixed: bool) {
        self.metadata.fixed = fixed;
    }
    pub fn fixed(&self) -> bool {
        self.metadata.fixed
    }
    pub fn set_local_parameterization(&mut self, manifold: Option<Arc<dyn Manifold>>) {
        self.metadata.local_parameterization = manifold;
    }
    pub fn local_parameterization(&self) -> Option<&Arc<dyn Manifold>> {
        self.metadata.local_parameterization.as_ref()
    }
}
