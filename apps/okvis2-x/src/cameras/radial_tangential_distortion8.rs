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

//! Distortion and inverse-map formulas corresponding to `RadialTangentialDistortion8.hpp`.

use crate::cameras::distortion_base::{DistortionBase, inverse_2x2};
use nalgebra::{DVector, Matrix2, Matrix2xX, SVector, Vector2};
use std::sync::Arc;

#[derive(Clone, Copy, Debug)]
pub struct RadialTangentialDistortion8 {
    parameters: SVector<f64, 8>,
    k1: f64,
    k2: f64,
    p1: f64,
    p2: f64,
    k3: f64,
    k4: f64,
    k5: f64,
    k6: f64,
}

impl Default for RadialTangentialDistortion8 {
    fn default() -> Self {
        Self::new((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    }
}

impl RadialTangentialDistortion8 {
    pub const NUM_DISTORTION_INTRINSICS: usize = 8;
    #[must_use]
    pub fn new((k1, k2, p1, p2, k3, k4, k5, k6): (f64, f64, f64, f64, f64, f64, f64, f64)) -> Self {
        Self {
            parameters: SVector::<f64, 8>::from_row_slice(&[k1, k2, p1, p2, k3, k4, k5, k6]),
            k1,
            k2,
            p1,
            p2,
            k3,
            k4,
            k5,
            k6,
        }
    }
    #[must_use]
    pub fn test_object() -> Self {
        Self::new((
            0.6261, 0.001, -0.0002, 0.0001, 0.0001, 0.9541, 0.1151, -0.0075,
        ))
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
        let k3 = self.k3;
        let k4 = self.k4;
        let k5 = self.k5;
        let k6 = self.k6;

        let u0 = point[0];
        let u1 = point[1];
        let mx_u = u0 * u0;
        let squared_y = u1 * u1;
        let coordinate_product = u0 * u1;
        let rho_u = mx_u + squared_y;

        if rho_u > 9.0 {
            return false;
        }

        let c = rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0;
        let c2 = c * c;

        let rad_dist_u = (1.0 + ((k3 * rho_u + k2) * rho_u + k1) * rho_u)
            / (1.0 + ((k6 * rho_u + k5) * rho_u + k4) * rho_u);
        output[0] = u0 * rad_dist_u + 2.0 * p1 * coordinate_product + p2 * (rho_u + 2.0 * mx_u);
        output[1] =
            u1 * rad_dist_u + 2.0 * p2 * coordinate_product + p1 * (rho_u + 2.0 * squared_y);

        if let Some(jacobian) = point_jacobian {
            jacobian[(0, 0)] = p1 * u1 * 2.0
                + p2 * u0 * 6.0
                + (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0)
                    / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0)
                + (u0
                    * (rho_u * (u0 * (k2 + k3 * rho_u) * 2.0 + k3 * u0 * rho_u * 2.0)
                        + u0 * (k1 + rho_u * (k2 + k3 * rho_u)) * 2.0))
                    / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0)
                - u0 * (rho_u * (u0 * (k5 + k6 * rho_u) * 2.0 + k6 * u0 * rho_u * 2.0)
                    + u0 * (k4 + rho_u * (k5 + k6 * rho_u)) * 2.0)
                    * (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0)
                    * 1.0
                    / c2;
            jacobian[(0, 1)] = p1 * u0 * 2.0
                + p2 * u1 * 2.0
                + (u0
                    * (rho_u * (u1 * (k2 + k3 * rho_u) * 2.0 + k3 * u1 * rho_u * 2.0)
                        + u1 * (k1 + rho_u * (k2 + k3 * rho_u)) * 2.0))
                    / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0)
                - u0 * (rho_u * (u1 * (k5 + k6 * rho_u) * 2.0 + k6 * u1 * rho_u * 2.0)
                    + u1 * (k4 + rho_u * (k5 + k6 * rho_u)) * 2.0)
                    * (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0)
                    * 1.0
                    / c2;
            jacobian[(1, 0)] = p1 * u0 * 2.0
                + p2 * u1 * 2.0
                + (u1
                    * (rho_u * (u0 * (k2 + k3 * rho_u) * 2.0 + k3 * u0 * rho_u * 2.0)
                        + u0 * (k1 + rho_u * (k2 + k3 * rho_u)) * 2.0))
                    / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0)
                - u1 * (rho_u * (u0 * (k5 + k6 * rho_u) * 2.0 + k6 * u0 * rho_u * 2.0)
                    + u0 * (k4 + rho_u * (k5 + k6 * rho_u)) * 2.0)
                    * (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0)
                    * 1.0
                    / c2;
            jacobian[(1, 1)] = p1 * u1 * 6.0
                + p2 * u0 * 2.0
                + (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0)
                    / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0)
                + (u1
                    * (rho_u * (u1 * (k2 + k3 * rho_u) * 2.0 + k3 * u1 * rho_u * 2.0)
                        + u1 * (k1 + rho_u * (k2 + k3 * rho_u)) * 2.0))
                    / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0)
                - u1 * (rho_u * (u1 * (k5 + k6 * rho_u) * 2.0 + k6 * u1 * rho_u * 2.0)
                    + u1 * (k4 + rho_u * (k5 + k6 * rho_u)) * 2.0)
                    * (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0)
                    * 1.0
                    / c2;
        }
        self.evaluate_parameter_jacobian(point, rho_u, c2, parameter_jacobian);
        true
    }
    fn evaluate_parameter_jacobian(
        &self,
        point: &Vector2<f64>,
        rho_u: f64,
        c2: f64,
        parameter_jacobian: Option<&mut Matrix2xX<f64>>,
    ) {
        let (u0, u1) = (point[0], point[1]);
        let (k1, k2, k3, k4, k5, k6) = (self.k1, self.k2, self.k3, self.k4, self.k5, self.k6);
        if let Some(parameters_jacobian) = parameter_jacobian {
            *parameters_jacobian = Matrix2xX::zeros(Self::NUM_DISTORTION_INTRINSICS);
            let rho_u2 = rho_u * rho_u;
            let rho_u3 = rho_u * rho_u2;
            parameters_jacobian[(0, 0)] =
                (u0 * rho_u) / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0);
            parameters_jacobian[(0, 1)] =
                (u0 * rho_u2) / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0);
            parameters_jacobian[(0, 2)] = u0 * u1 * 2.0;
            parameters_jacobian[(0, 3)] = (u0 * u0) * 3.0 + u1 * u1;
            parameters_jacobian[(0, 4)] =
                (u0 * rho_u3) / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0);
            parameters_jacobian[(0, 5)] =
                -u0 * rho_u * (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0) * 1.0 / c2;
            parameters_jacobian[(0, 6)] =
                -u0 * rho_u2 * (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0) * 1.0 / c2;
            parameters_jacobian[(0, 7)] =
                -u0 * rho_u3 * (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0) * 1.0 / c2;
            parameters_jacobian[(1, 0)] =
                (u1 * rho_u) / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0);
            parameters_jacobian[(1, 1)] =
                (u1 * rho_u2) / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0);
            parameters_jacobian[(1, 2)] = u0 * u0 + (u1 * u1) * 3.0;
            parameters_jacobian[(1, 3)] = u0 * u1 * 2.0;
            parameters_jacobian[(1, 4)] =
                (u1 * rho_u3) / (rho_u * (k4 + rho_u * (k5 + k6 * rho_u)) + 1.0);
            parameters_jacobian[(1, 5)] =
                -u1 * rho_u * (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0) * 1.0 / c2;
            parameters_jacobian[(1, 6)] =
                -u1 * rho_u2 * (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0) * 1.0 / c2;
            parameters_jacobian[(1, 7)] =
                -u1 * rho_u3 * (rho_u * (k1 + rho_u * (k2 + k3 * rho_u)) + 1.0) * 1.0 / c2;
        }
    }
}

impl DistortionBase for RadialTangentialDistortion8 {
    fn set_parameters(&mut self, parameters: &DVector<f64>) -> bool {
        if parameters.len() != Self::NUM_DISTORTION_INTRINSICS {
            return false;
        }
        self.parameters.copy_from(parameters);
        self.k1 = parameters[0];
        self.k2 = parameters[1];
        self.p1 = parameters[2];
        self.p2 = parameters[3];
        self.k3 = parameters[4];
        self.k4 = parameters[5];
        self.k5 = parameters[6];
        self.k6 = parameters[7];
        true
    }
    fn get_parameters(&self, parameters: &mut DVector<f64>) -> bool {
        *parameters = DVector::from_column_slice(self.parameters.as_slice());
        true
    }
    fn type_name(&self) -> &'static str {
        "RadialTangentialDistortion8"
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
        let model = Self::new((
            parameters[0],
            parameters[1],
            parameters[2],
            parameters[3],
            parameters[4],
            parameters[5],
            parameters[6],
            parameters[7],
        ));
        model.evaluate_distortion(point, output, Some(point_jacobian), parameter_jacobian)
    }

    fn undistort(&self, point: &Vector2<f64>, output: &mut Vector2<f64>) -> bool {
        let mut x_bar = *point;
        let mut e_jacobian = Matrix2::zeros();
        let mut success = false;
        for _ in 0..5 {
            let mut x_tmp = Vector2::zeros();
            if !self.distort_with_jacobians(&x_bar, &mut x_tmp, &mut e_jacobian, None) {
                return false;
            }
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
            if !self.distort_with_jacobians(&x_bar, &mut x_tmp, &mut e_jacobian, None) {
                return false;
            }
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
