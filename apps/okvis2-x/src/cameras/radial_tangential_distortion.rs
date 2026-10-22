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

//! Distortion and inverse-map formulas corresponding to `RadialTangentialDistortion.hpp`.

use crate::cameras::distortion_base::{DistortionBase, inverse_2x2};
use nalgebra::{DVector, Matrix2, Matrix2xX, SVector, Vector2};
use std::sync::Arc;

#[derive(Clone, Copy, Debug)]
pub struct RadialTangentialDistortion {
    parameters: SVector<f64, 4>,
    k1: f64,
    k2: f64,
    p1: f64,
    p2: f64,
}

impl Default for RadialTangentialDistortion {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0)
    }
}

impl RadialTangentialDistortion {
    pub const NUM_DISTORTION_INTRINSICS: usize = 4;
    #[must_use]
    pub fn new(k1: f64, k2: f64, p1: f64, p2: f64) -> Self {
        Self {
            parameters: SVector::<f64, 4>::from_row_slice(&[k1, k2, p1, p2]),
            k1,
            k2,
            p1,
            p2,
        }
    }
    #[must_use]
    pub fn test_object() -> Self {
        Self::new(-0.16, 0.15, 0.0003, 0.0002)
    }
    #[must_use]
    pub fn create_test_object() -> Arc<dyn DistortionBase> {
        Arc::new(Self::test_object())
    }

    fn evaluate_distortion(
        &self,
        point: &Vector2<f64>,
        output: &mut Vector2<f64>,
        point_jacobian: Option<&mut Matrix2<f64>>,
        parameter_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> bool {
        let k1 = self.k1;
        let k2 = self.k2;
        let p1 = self.p1;
        let p2 = self.p2;
        let u0 = point[0];
        let u1 = point[1];
        let mx_u = u0 * u0;
        let squared_y = u1 * u1;
        let coordinate_product = u0 * u1;
        let rho_u = mx_u + squared_y;
        let rad_dist_u = k1 * rho_u + k2 * rho_u * rho_u;
        output[0] =
            u0 + u0 * rad_dist_u + 2.0 * p1 * coordinate_product + p2 * (rho_u + 2.0 * mx_u);
        output[1] =
            u1 + u1 * rad_dist_u + 2.0 * p2 * coordinate_product + p1 * (rho_u + 2.0 * squared_y);
        if let Some(jacobian) = point_jacobian {
            jacobian[(0, 0)] = 1.0
                + rad_dist_u
                + k1 * 2.0 * mx_u
                + k2 * rho_u * 4.0 * mx_u
                + 2.0 * p1 * u1
                + 6.0 * p2 * u0;
            jacobian[(1, 0)] =
                k1 * 2.0 * u0 * u1 + k2 * 4.0 * rho_u * u0 * u1 + p1 * 2.0 * u0 + 2.0 * p2 * u1;
            jacobian[(0, 1)] = jacobian[(1, 0)];
            jacobian[(1, 1)] = 1.0
                + rad_dist_u
                + k1 * 2.0 * squared_y
                + k2 * rho_u * 4.0 * squared_y
                + 6.0 * p1 * u1
                + 2.0 * p2 * u0;
        }
        if let Some(parameters_jacobian) = parameter_jacobian {
            *parameters_jacobian = Matrix2xX::zeros(Self::NUM_DISTORTION_INTRINSICS);
            let r2 = rho_u;
            let r4 = r2 * r2;
            parameters_jacobian[(0, 0)] = u0 * r2;
            parameters_jacobian[(0, 1)] = u0 * r4;
            parameters_jacobian[(0, 2)] = 2.0 * u0 * u1;
            parameters_jacobian[(0, 3)] = r2 + 2.0 * u0 * u0;
            parameters_jacobian[(1, 0)] = u1 * r2;
            parameters_jacobian[(1, 1)] = u1 * r4;
            parameters_jacobian[(1, 2)] = r2 + 2.0 * u1 * u1;
            parameters_jacobian[(1, 3)] = 2.0 * u0 * u1;
        }
        true
    }
}

impl DistortionBase for RadialTangentialDistortion {
    fn set_parameters(&mut self, parameters: &DVector<f64>) -> bool {
        if parameters.len() != Self::NUM_DISTORTION_INTRINSICS {
            return false;
        }
        self.parameters.copy_from(parameters);
        self.k1 = parameters[0];
        self.k2 = parameters[1];
        self.p1 = parameters[2];
        self.p2 = parameters[3];
        true
    }
    fn get_parameters(&self, parameters: &mut DVector<f64>) -> bool {
        *parameters = DVector::from_column_slice(self.parameters.as_slice());
        true
    }
    fn type_name(&self) -> &'static str {
        "RadialTangentialDistortion"
    }
    fn num_distortion_intrinsics(&self) -> usize {
        Self::NUM_DISTORTION_INTRINSICS
    }

    fn distort(&self, point: &Vector2<f64>, output: &mut Vector2<f64>) -> bool {
        self.evaluate_distortion(point, output, None, None)
    }
    fn distort_with_jacobians(
        &self,
        point: &Vector2<f64>,
        output: &mut Vector2<f64>,
        point_jacobian: &mut Matrix2<f64>,
        parameter_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> bool {
        self.evaluate_distortion(point, output, Some(point_jacobian), parameter_jacobian)
    }
    fn distort_with_external_parameters(
        &self,
        point: &Vector2<f64>,
        parameters: &DVector<f64>,
        output: &mut Vector2<f64>,
        point_jacobian: Option<&mut Matrix2<f64>>,
        parameter_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> bool {
        if parameters.len() < Self::NUM_DISTORTION_INTRINSICS {
            return false;
        }
        let Some(point_jacobian) = point_jacobian else {
            return false;
        };
        let model = Self::new(parameters[0], parameters[1], parameters[2], parameters[3]);
        model.evaluate_distortion(point, output, Some(point_jacobian), parameter_jacobian)
    }

    fn undistort(&self, point: &Vector2<f64>, output: &mut Vector2<f64>) -> bool {
        let mut x_bar = *point;
        let mut e_jacobian = Matrix2::zeros();
        let mut success = false;
        for _ in 0..5 {
            let mut x_tmp = Vector2::zeros();
            self.distort_with_jacobians(&x_bar, &mut x_tmp, &mut e_jacobian, None);
            let e = point - x_tmp;
            let delta =
                inverse_2x2(&(e_jacobian.transpose() * e_jacobian)) * e_jacobian.transpose() * e;
            x_bar += delta;
            let chi2 = e.dot(&e);
            if chi2 < 1e-6 {
                success = true;
            }
            if chi2 < 1e-15 {
                success = true;
                break;
            }
        }
        *output = x_bar;
        if !success {
            log::info!("undistortion failed");
        }
        success
    }

    fn undistort_with_jacobian(
        &self,
        point: &Vector2<f64>,
        output: &mut Vector2<f64>,
        point_jacobian: &mut Matrix2<f64>,
    ) -> bool {
        let mut x_bar = *point;
        let mut e_jacobian = Matrix2::zeros();
        let mut success = false;
        for _ in 0..5 {
            let mut x_tmp = Vector2::zeros();
            self.distort_with_jacobians(&x_bar, &mut x_tmp, &mut e_jacobian, None);
            let e = point - x_tmp;
            let delta =
                inverse_2x2(&(e_jacobian.transpose() * e_jacobian)) * e_jacobian.transpose() * e;
            x_bar += delta;
            let chi2 = e.dot(&e);
            if chi2 < 1e-4 {
                success = true;
            }
            if chi2 < 1e-15 {
                success = true;
                break;
            }
        }
        *output = x_bar;
        *point_jacobian = inverse_2x2(&e_jacobian);
        success
    }
}
