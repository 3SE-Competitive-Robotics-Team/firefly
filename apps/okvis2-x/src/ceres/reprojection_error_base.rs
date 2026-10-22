/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Reprojection interfaces and an object-safe adapter for the camera-generic factor.

use crate::{
    cameras::camera_base::CameraBase,
    ceres::{
        error_interface::{ErrorEvaluation, ErrorInterface},
        reprojection_error::{
            MinimalReprojectionJacobians, ReprojectionError, ReprojectionJacobians,
        },
    },
};
use nalgebra::{DMatrix, DVector, Matrix2, SMatrix, SVector, Vector2, Vector4};
use std::sync::Arc;

/// Camera identity and the residual/parameter sizes of a reprojection cost.
pub trait ReprojectionErrorBase: ErrorInterface {
    fn camera_id(&self) -> Option<u64>;
    fn set_camera_id(&mut self, camera_id: u64);
}
pub trait ReprojectionError2dBase: ReprojectionErrorBase {
    fn set_measurement(&mut self, measurement: Vector2<f64>);
    fn set_information(&mut self, information: Matrix2<f64>);
    fn measurement(&self) -> Option<&Vector2<f64>>;
    fn information(&self) -> Option<&Matrix2<f64>>;
    fn covariance(&self) -> Option<&Matrix2<f64>>;
    fn clone_box(&self) -> Box<dyn ReprojectionError2dBase>;
    /// Owns calibrated camera data for an independent numerical worker.
    fn snapshot_send(&self) -> Option<Box<dyn ReprojectionError2dBase + Send>>;
}
impl<G: CameraBase + ?Sized> ErrorInterface for ReprojectionError<G> {
    fn residual_dim(&self) -> usize {
        2
    }
    fn parameter_blocks(&self) -> usize {
        3
    }
    fn parameter_block_dim(&self, block: usize) -> usize {
        [7, 4, 7][block]
    }
    fn type_info(&self) -> &'static str {
        "ReprojectionError"
    }
    fn evaluate_with_minimal_jacobians(
        &self,
        parameters: &[&[f64]],
        jacobians: Option<&[bool]>,
        minimal: Option<&[bool]>,
    ) -> Option<ErrorEvaluation> {
        if parameters.len() != 3
            || parameters[0].len() != 7
            || parameters[1].len() != 4
            || parameters[2].len() != 7
        {
            return None;
        }
        if jacobians.is_some_and(|x| x.len() != 3) || minimal.is_some_and(|x| x.len() != 3) {
            return None;
        }
        self.measurement()?;
        self.information()?;
        let p0 = SVector::from_column_slice(parameters[0]);
        let p1 = Vector4::from_column_slice(parameters[1]);
        let p2 = SVector::from_column_slice(parameters[2]);
        let mut residual = Vector2::zeros();
        let mut j0 = SMatrix::<f64, 2, 7>::zeros();
        let mut j1 = SMatrix::<f64, 2, 4>::zeros();
        let mut j2 = SMatrix::<f64, 2, 7>::zeros();
        let mut m0 = SMatrix::<f64, 2, 6>::zeros();
        let mut m1 = SMatrix::<f64, 2, 3>::zeros();
        let mut m2 = SMatrix::<f64, 2, 6>::zeros();
        let js = jacobians.map(|mask| ReprojectionJacobians {
            pose: mask[0].then_some(&mut j0),
            point: mask[1].then_some(&mut j1),
            extrinsics: mask[2].then_some(&mut j2),
        });
        let ms = minimal.map(|mask| MinimalReprojectionJacobians {
            pose: mask[0].then_some(&mut m0),
            point: mask[1].then_some(&mut m1),
            extrinsics: mask[2].then_some(&mut m2),
        });
        if !ReprojectionError::evaluate_with_minimal_jacobians(
            self,
            (&p0, &p1, &p2),
            &mut residual,
            js,
            ms,
        ) {
            return None;
        }
        let matrices = [
            DMatrix::from_column_slice(2, 7, j0.as_slice()),
            DMatrix::from_column_slice(2, 4, j1.as_slice()),
            DMatrix::from_column_slice(2, 7, j2.as_slice()),
        ];
        let mins = [
            DMatrix::from_column_slice(2, 6, m0.as_slice()),
            DMatrix::from_column_slice(2, 3, m1.as_slice()),
            DMatrix::from_column_slice(2, 6, m2.as_slice()),
        ];
        Some(ErrorEvaluation {
            residuals: DVector::from_column_slice(residual.as_slice()),
            jacobians: matrices
                .into_iter()
                .enumerate()
                .map(|(i, m)| jacobians.is_some_and(|x| x[i]).then_some(m))
                .collect(),
            minimal_jacobians: mins
                .into_iter()
                .enumerate()
                .map(|(i, m)| {
                    (jacobians.is_some_and(|x| x[i]) && minimal.is_some_and(|x| x[i])).then_some(m)
                })
                .collect(),
        })
    }
}
impl<G: CameraBase + ?Sized> ReprojectionErrorBase for ReprojectionError<G> {
    fn camera_id(&self) -> Option<u64> {
        self.camera_id()
    }
    fn set_camera_id(&mut self, camera_id: u64) {
        self.set_camera_id(camera_id);
    }
}
impl<G: CameraBase + ?Sized + 'static> ReprojectionError2dBase for ReprojectionError<G> {
    fn set_measurement(&mut self, measurement: Vector2<f64>) {
        self.set_measurement(measurement);
    }
    fn set_information(&mut self, information: Matrix2<f64>) {
        self.set_information(information);
    }
    fn measurement(&self) -> Option<&Vector2<f64>> {
        self.measurement()
    }
    fn information(&self) -> Option<&Matrix2<f64>> {
        self.information()
    }
    fn covariance(&self) -> Option<&Matrix2<f64>> {
        self.covariance()
    }
    fn snapshot_send(&self) -> Option<Box<dyn ReprojectionError2dBase + Send>> {
        self.detached_snapshot()
    }
    fn clone_box(&self) -> Box<dyn ReprojectionError2dBase> {
        match Arc::try_unwrap(self.clone_factor()) {
            Ok(factor) => Box::new(factor),
            Err(_) => unreachable!("clone_factor returns one owned reference"),
        }
    }
}
