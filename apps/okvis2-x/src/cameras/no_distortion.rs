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

//! Identity distortion model corresponding to `NoDistortion.hpp`.

use super::distortion_base::DistortionBase;
use nalgebra::{DVector, Matrix2, Matrix2xX, Vector2};
use std::sync::Arc;

#[derive(Clone, Copy, Debug, Default)]
pub struct NoDistortion;

impl NoDistortion {
    pub const NUM_DISTORTION_INTRINSICS: usize = 0;
    #[must_use]
    pub fn test_object() -> Self {
        Self
    }
    #[must_use]
    pub fn create_test_object() -> Arc<dyn DistortionBase> {
        Arc::new(Self::test_object())
    }
}

impl DistortionBase for NoDistortion {
    fn set_parameters(&mut self, _parameters: &DVector<f64>) -> bool {
        true
    }
    fn get_parameters(&self, parameters: &mut DVector<f64>) -> bool {
        *parameters = DVector::zeros(0);
        true
    }
    fn type_name(&self) -> &'static str {
        "NoDistortion"
    }
    fn num_distortion_intrinsics(&self) -> usize {
        Self::NUM_DISTORTION_INTRINSICS
    }

    fn distort(&self, point: &Vector2<f64>, output: &mut Vector2<f64>) -> bool {
        *output = *point;
        true
    }

    fn distort_with_jacobians(
        &self,
        point: &Vector2<f64>,
        output: &mut Vector2<f64>,
        point_jacobian: &mut Matrix2<f64>,
        parameter_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> bool {
        *output = *point;
        *point_jacobian = Matrix2::identity();
        if let Some(jacobian) = parameter_jacobian {
            *jacobian = Matrix2xX::zeros(0);
        }
        true
    }

    fn distort_with_external_parameters(
        &self,
        point: &Vector2<f64>,
        _parameters: &DVector<f64>,
        output: &mut Vector2<f64>,
        point_jacobian: Option<&mut Matrix2<f64>>,
        parameter_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> bool {
        *output = *point;
        if let Some(jacobian) = point_jacobian {
            *jacobian = Matrix2::identity();
        }
        if let Some(jacobian) = parameter_jacobian {
            *jacobian = Matrix2xX::zeros(0);
        }
        true
    }

    fn undistort(&self, point: &Vector2<f64>, output: &mut Vector2<f64>) -> bool {
        *output = *point;
        true
    }

    fn undistort_with_jacobian(
        &self,
        point: &Vector2<f64>,
        output: &mut Vector2<f64>,
        point_jacobian: &mut Matrix2<f64>,
    ) -> bool {
        *output = *point;
        *point_jacobian = Matrix2::identity();
        true
    }
}
