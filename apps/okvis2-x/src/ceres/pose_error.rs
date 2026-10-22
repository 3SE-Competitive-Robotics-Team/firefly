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

//! Absolute pose prior corresponding to `PoseError.cpp`.

use nalgebra::{Isometry3, Matrix3, Quaternion, SMatrix, SVector};

use super::pose_local_parameterization::PoseManifold;
use crate::kinematics::{
    operators::plus,
    transformation::{Transformation, from_isometry_coefficients, normalized_quaternion},
};

/// Prior with translation followed by rotation residuals.
///
/// Unassigned weighting fields are `None`. The diagonal-information constructor
/// leaves covariance unassigned, matching the fields written by its C++ counterpart.
#[derive(Clone, Debug)]
pub struct PoseError {
    measurement: Isometry3<f64>,
    information: Option<SMatrix<f64, 6, 6>>,
    square_root_information: Option<SMatrix<f64, 6, 6>>,
    covariance: Option<SMatrix<f64, 6, 6>>,
}

impl Default for PoseError {
    fn default() -> Self {
        Self {
            measurement: Isometry3::identity(),
            information: None,
            square_root_information: None,
            covariance: None,
        }
    }
}

impl PoseError {
    pub const NUM_RESIDUALS: usize = 6;

    /// Constructs a prior with the weighting produced by Eigen's LLT path.
    #[must_use]
    pub fn new(measurement: Isometry3<f64>, information: SMatrix<f64, 6, 6>) -> Self {
        let mut prior = Self {
            measurement,
            ..Self::default()
        };
        prior.set_information(information);
        prior
    }

    /// Sets diagonal information and its square root; covariance remains `None`.
    pub fn from_information_diagonal(
        measurement: Isometry3<f64>,
        information_diagonal: SVector<f64, 6>,
    ) -> Self {
        Self {
            measurement,
            information: Some(SMatrix::from_diagonal(&information_diagonal)),
            square_root_information: Some(SMatrix::from_diagonal(
                &information_diagonal.map(f64::sqrt),
            )),
            covariance: None,
        }
    }

    /// Both variances must be positive and finite.
    #[must_use]
    pub fn from_variances(
        measurement: Isometry3<f64>,
        translation_variance: f64,
        rotation_variance: f64,
    ) -> Self {
        let mut information = SMatrix::<f64, 6, 6>::zeros();
        information
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(Matrix3::identity() * 1.0 / translation_variance));
        information
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(Matrix3::identity() * 1.0 / rotation_variance));
        Self::new(measurement, information)
    }

    pub fn set_measurement(&mut self, measurement: Isometry3<f64>) {
        self.measurement = measurement;
    }

    /// Sets information and its upper Cholesky factor. A singular covariance is absent.
    /// Failed LLT keeps the partial lower factor, as the source's unchecked Eigen LLT does.
    pub fn set_information(&mut self, information: SMatrix<f64, 6, 6>) {
        self.information = Some(information);
        self.covariance = information.try_inverse();
        self.square_root_information = Some(square_root_information(information));
    }

    #[must_use]
    pub fn measurement(&self) -> &Isometry3<f64> {
        &self.measurement
    }
    #[must_use]
    pub fn information(&self) -> Option<&SMatrix<f64, 6, 6>> {
        self.information.as_ref()
    }
    #[must_use]
    pub fn covariance(&self) -> Option<&SMatrix<f64, 6, 6>> {
        self.covariance.as_ref()
    }

    pub fn evaluate(
        &self,
        parameters: &SVector<f64, 7>,
        residuals: &mut SVector<f64, 6>,
        jacobian: Option<&mut SMatrix<f64, 6, 7>>,
    ) -> bool {
        self.evaluate_with_minimal_jacobians(parameters, residuals, jacobian, None)
    }

    /// Parameters are `[tx, ty, tz, qx, qy, qz, qw]`.
    ///
    /// Returns false if information has not been set. The minimal Jacobian is written
    /// only when the ambient Jacobian is requested, matching C++.
    pub fn evaluate_with_minimal_jacobians(
        &self,
        parameters: &SVector<f64, 7>,
        residuals: &mut SVector<f64, 6>,
        jacobian: Option<&mut SMatrix<f64, 6, 7>>,
        jacobian_minimal: Option<&mut SMatrix<f64, 6, 6>>,
    ) -> bool {
        let q = normalized_quaternion(Quaternion::new(
            parameters[6],
            parameters[3],
            parameters[4],
            parameters[5],
        ));
        let t_ws = Transformation::new(parameters.fixed_rows::<3>(0).into_owned(), q);
        let dp = from_isometry_coefficients(&self.measurement).compose(&t_ws.inverse());
        let mut error = SVector::<f64, 6>::zeros();
        error
            .fixed_rows_mut::<3>(0)
            .copy_from(&(self.measurement.translation.vector - t_ws.translation()));
        error
            .fixed_rows_mut::<3>(3)
            .copy_from(&(2.0 * dp.quaternion().coords.fixed_rows::<3>(0)));
        let Some(square_root_information) = self.square_root_information.as_ref() else {
            return false;
        };
        *residuals = square_root_information * error;

        if let Some(jacobian) = jacobian {
            let mut j_minimal = -SMatrix::<f64, 6, 6>::identity();
            j_minimal
                .fixed_view_mut::<3, 3>(3, 3)
                .copy_from(&(-plus(&dp.quaternion()).fixed_view::<3, 3>(0, 0)));
            let j_minimal = square_root_information * j_minimal;
            let j_lift = PoseManifold::minus_jacobian(parameters);
            *jacobian = j_minimal * j_lift;
            if let Some(jacobian_minimal) = jacobian_minimal {
                *jacobian_minimal = j_minimal;
            }
        }
        true
    }

    #[must_use]
    pub const fn residual_dim(&self) -> usize {
        Self::NUM_RESIDUALS
    }
    #[must_use]
    pub const fn parameter_blocks(&self) -> usize {
        1
    }
    #[must_use]
    pub fn parameter_block_dim(&self, parameter_block_id: usize) -> usize {
        [7][parameter_block_id]
    }
    #[must_use]
    pub const fn type_info(&self) -> &'static str {
        "PoseError"
    }
}

/// Eigen stops at the first nonpositive pivot and exposes the remaining lower
/// triangle unchanged when the caller reads matrixL without checking `info()`.
pub(crate) fn square_root_information<const N: usize>(
    information: SMatrix<f64, N, N>,
) -> SMatrix<f64, N, N> {
    if let Some(cholesky) = information.cholesky() {
        return cholesky.l().transpose();
    }
    let mut lower = information;
    for column in 0..N {
        let diagonal =
            lower[(column, column)] - (0..column).map(|k| lower[(column, k)].powi(2)).sum::<f64>();
        if diagonal <= 0.0 {
            break;
        }
        lower[(column, column)] = diagonal.sqrt();
        for row in column + 1..N {
            let product = (0..column)
                .map(|k| lower[(row, k)] * lower[(column, k)])
                .sum::<f64>();
            lower[(row, column)] = (lower[(row, column)] - product) / lower[(column, column)];
        }
    }
    SMatrix::from_fn(|row, column| {
        if row <= column {
            lower[(column, row)]
        } else {
            0.0
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn information_square_root_matches_eigen_success_and_partial_failure() {
        let indefinite = SMatrix::<f64, 3, 3>::new(4., 2., 1., 2., 1., 0., 1., 0., 3.);
        let partial = SMatrix::<f64, 3, 3>::new(2., 1., 0.5, 0., 1., 0., 0., 0., 3.);
        assert_eq!(square_root_information(indefinite), partial);
        let positive = SMatrix::<f64, 3, 3>::new(4., 2., 1., 2., 5., 0., 1., 0., 3.);
        let expected =
            SMatrix::<f64, 3, 3>::new(2., 1., 0.5, 0., 2., -0.25, 0., 0., 1.639_359_631_075_5);
        assert!((square_root_information(positive) - expected).amax() < 1e-15);
        let zero = SMatrix::<f64, 9, 9>::zeros();
        assert_eq!(square_root_information(zero), zero);
    }
    #[test]
    fn priors_accept_zero_and_indefinite_weights_without_panicking() {
        use crate::ceres::{
            gps_error_synchronous::GpsErrorSynchronous,
            homogeneous_point_error::HomogeneousPointError, relative_pose_error::RelativePoseError,
            speed_and_bias_error::SpeedAndBiasError,
        };
        let mut identity = SVector::<f64, 7>::zeros();
        identity[6] = 1.;
        let mut r6 = SVector::<f64, 6>::repeat(99.);
        let relative = RelativePoseError::new(SMatrix::zeros(), Isometry3::identity());
        assert!(relative.covariance().is_none());
        assert!(relative.evaluate([&identity, &identity], &mut r6, [None, None]));
        assert_eq!(r6, SVector::<f64, 6>::zeros());
        let speed = SpeedAndBiasError::new(SVector::repeat(1.), SMatrix::zeros());
        let mut r9 = SVector::<f64, 9>::repeat(99.);
        assert!(speed.covariance().is_none());
        assert!(speed.evaluate(&SVector::zeros(), &mut r9, None));
        assert_eq!(r9, SVector::<f64, 9>::zeros());
        let information = SMatrix::<f64, 3, 3>::new(4., 2., 1., 2., 1., 0., 1., 0., 3.);
        let point = HomogeneousPointError::new(nalgebra::Vector4::new(0., 0., 0., 1.), information);
        let mut r3 = nalgebra::Vector3::zeros();
        assert!(point.evaluate(&nalgebra::Vector4::repeat(1.), &mut r3, None));
        assert_eq!(r3, nalgebra::Vector3::new(3.5, 1., 3.));
        let gps = GpsErrorSynchronous::new(
            0,
            nalgebra::Vector3::repeat(1.),
            information,
            crate::config::parameters::GpsParameters::default(),
        );
        assert!(gps.evaluate([&identity, &identity], &mut r3, [None, None]));
        assert_eq!(r3, nalgebra::Vector3::new(3.5, 1., 3.));
    }

    #[test]
    fn uninitialized_pose_and_depth_return_false_without_touching_residuals() {
        let mut identity = SVector::<f64, 7>::zeros();
        identity[6] = 1.;
        let mut pose_residual = SVector::<f64, 6>::repeat(12.);
        assert!(!PoseError::default().evaluate(&identity, &mut pose_residual, None));
        assert_eq!(pose_residual, SVector::<f64, 6>::repeat(12.));
        let mut depth_residual = 12.;
        assert!(!crate::ceres::depth_error::DepthError::default().evaluate(
            (
                &identity,
                &nalgebra::Vector4::new(0., 0., 2., 1.),
                &identity
            ),
            &mut depth_residual,
            None
        ));
        assert_eq!(depth_residual, 12.);
    }
}
