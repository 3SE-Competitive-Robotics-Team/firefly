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

//! Parameter storage and local-update interfaces corresponding to `ParameterBlock.hpp`.

use crate::ceres::{
    homogeneous_point_local_parameterization::HomogeneousPointManifold,
    pose_local_parameterization::{PoseManifold, PoseManifold4d},
};
use nalgebra::{SMatrix, SVector};
use std::sync::Arc;

/// Local parameterisation capability accepted by a parameter block.
/// Jacobian buffers contain row-major coefficients, as in the Ceres interface.
pub trait Manifold: Send + Sync {
    fn ambient_size(&self) -> usize;
    fn tangent_size(&self) -> usize;
    fn plus(&self, x: &[f64], delta: &[f64], result: &mut [f64]);
    fn plus_jacobian(&self, x: &[f64], jacobian: &mut [f64]);
    fn minus(&self, y: &[f64], x: &[f64], delta: &mut [f64]);
    fn minus_jacobian(&self, x: &[f64], jacobian: &mut [f64]);
}

#[derive(Clone, Default)]
pub struct ParameterBlockMetadata {
    pub id: u64,
    pub fixed: bool,
    pub local_parameterization: Option<Arc<dyn Manifold>>,
}

/// Object-safe parameter block interface. An absent estimate has no parameter slice.
/// Input and output slice lengths must match the declared dimensions.
pub trait ParameterBlock {
    fn metadata(&self) -> &ParameterBlockMetadata;
    fn metadata_mut(&mut self) -> &mut ParameterBlockMetadata;
    fn set_parameters(&mut self, parameters: &[f64]);
    fn parameters(&self) -> Option<&[f64]>;
    fn parameters_mut(&mut self) -> Option<&mut [f64]>;
    fn dimension(&self) -> usize;
    fn minimal_dimension(&self) -> usize;
    fn plus(&self, x: &[f64], delta: &[f64], result: &mut [f64]);
    fn plus_jacobian(&self, x: &[f64], jacobian: &mut [f64]);
    /// Computes `y minus x`; the ordering matches the parameter-block C++ API.
    fn minus(&self, x: &[f64], y: &[f64], delta: &mut [f64]);
    fn lift_jacobian(&self, x: &[f64], jacobian: &mut [f64]);
    fn type_info(&self) -> &'static str;

    fn set_id(&mut self, id: u64) {
        self.metadata_mut().id = id;
    }
    fn id(&self) -> u64 {
        self.metadata().id
    }
    fn set_fixed(&mut self, fixed: bool) {
        self.metadata_mut().fixed = fixed;
    }
    fn fixed(&self) -> bool {
        self.metadata().fixed
    }
    fn set_local_parameterization(&mut self, manifold: Option<Arc<dyn Manifold>>) {
        self.metadata_mut().local_parameterization = manifold;
    }
    fn local_parameterization(&self) -> Option<&Arc<dyn Manifold>> {
        self.metadata().local_parameterization.as_ref()
    }
}

pub(crate) fn write_row_major<const R: usize, const C: usize>(
    matrix: &SMatrix<f64, R, C>,
    output: &mut [f64],
) {
    assert_eq!(output.len(), R * C);
    for r in 0..R {
        for c in 0..C {
            output[r * C + c] = matrix[(r, c)];
        }
    }
}

macro_rules! implement_manifold {
    ($type:ty, $dim:literal, $min_dim:literal) => {
        impl Manifold for $type {
            fn ambient_size(&self) -> usize {
                $dim
            }
            fn tangent_size(&self) -> usize {
                $min_dim
            }
            fn plus(&self, x: &[f64], delta: &[f64], result: &mut [f64]) {
                assert_eq!(x.len(), $dim);
                assert_eq!(delta.len(), $min_dim);
                assert_eq!(result.len(), $dim);
                result.copy_from_slice(
                    <$type>::plus(
                        &SVector::<f64, $dim>::from_column_slice(x),
                        &SVector::<f64, $min_dim>::from_column_slice(delta),
                    )
                    .as_slice(),
                );
            }
            fn plus_jacobian(&self, x: &[f64], jacobian: &mut [f64]) {
                assert_eq!(x.len(), $dim);
                write_row_major(
                    &<$type>::plus_jacobian(&SVector::<f64, $dim>::from_column_slice(x)),
                    jacobian,
                );
            }
            fn minus(&self, y: &[f64], x: &[f64], delta: &mut [f64]) {
                assert_eq!(x.len(), $dim);
                assert_eq!(y.len(), $dim);
                assert_eq!(delta.len(), $min_dim);
                delta.copy_from_slice(
                    <$type>::minus(
                        &SVector::<f64, $dim>::from_column_slice(y),
                        &SVector::<f64, $dim>::from_column_slice(x),
                    )
                    .as_slice(),
                );
            }
            fn minus_jacobian(&self, x: &[f64], jacobian: &mut [f64]) {
                assert_eq!(x.len(), $dim);
                write_row_major(
                    &<$type>::minus_jacobian(&SVector::<f64, $dim>::from_column_slice(x)),
                    jacobian,
                );
            }
        }
    };
}
implement_manifold!(PoseManifold, 7, 6);
implement_manifold!(PoseManifold4d, 7, 4);
implement_manifold!(HomogeneousPointManifold, 4, 3);
