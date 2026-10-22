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

//! Distortion and inverse-map formulas corresponding to `EquidistantDistortion.hpp`.

use crate::cameras::distortion_base::{DistortionBase, inverse_2x2};
use nalgebra::{DVector, Matrix2, Matrix2xX, SVector, Vector2};
use std::sync::Arc;

#[derive(Clone, Copy, Debug)]
pub struct EquidistantDistortion {
    parameters: SVector<f64, 4>,
    k1: f64,
    k2: f64,
    k3: f64,
    k4: f64,
}

impl Default for EquidistantDistortion {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0)
    }
}

impl EquidistantDistortion {
    pub const NUM_DISTORTION_INTRINSICS: usize = 4;
    #[must_use]
    pub fn new(k1: f64, k2: f64, k3: f64, k4: f64) -> Self {
        Self {
            parameters: SVector::<f64, 4>::from_row_slice(&[k1, k2, k3, k4]),
            k1,
            k2,
            k3,
            k4,
        }
    }
    #[must_use]
    pub fn test_object() -> Self {
        Self::new(-0.21, 0.14, 0.0006, 0.0003)
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
        let k3 = self.k3;
        let k4 = self.k4;

        let u0 = point[0];
        let u1 = point[1];
        let r = (u0 * u0 + u1 * u1).sqrt();
        let theta = (r).atan();
        let theta2 = theta * theta;
        let theta4 = theta2 * theta2;
        let theta6 = theta4 * theta2;
        let theta8 = theta4 * theta4;
        let distorted_angle = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);

        let scaling = if r > 1e-8 { distorted_angle / r } else { 1.0 };
        output[0] = scaling * u0;
        output[1] = scaling * u1;

        if let Some(jacobian) = point_jacobian {
            if r > 1e-8 {
                let t2 = u0 * u0;
                let t3 = u1 * u1;
                let t4 = t2 + t3;
                let t6 = ((t4).sqrt()).atan();
                let t7 = t6 * t6;
                let t8 = 1.0 / (t4).sqrt();
                let t9 = t7 * t7;
                let t11 = 1.0 / ((t2 + t3) + 1.0);
                let t17 = (((k1 * t7 + k2 * t9) + k3 * t7 * t9) + k4 * (t9 * t9)) + 1.0;
                let t18 = 1.0 / t4;
                let t19 = 1.0 / (t4 * t4 * t4).sqrt();
                let t20 = t6 * t8 * t17;
                let t25 = ((k2 * t6 * t7 * t8 * t11 * u1 * 4.0
                    + k3 * t6 * t8 * t9 * t11 * u1 * 6.0)
                    + k4 * t6 * t7 * t8 * t9 * t11 * u1 * 8.0)
                    + k1 * t6 * t8 * t11 * u1 * 2.0;
                let t4 = ((k2 * t6 * t7 * t8 * t11 * u0 * 4.0
                    + k3 * t6 * t8 * t9 * t11 * u0 * 6.0)
                    + k4 * t6 * t7 * t8 * t9 * t11 * u0 * 8.0)
                    + k1 * t6 * t8 * t11 * u0 * 2.0;
                let t7 = t11 * t17 * t18 * u0 * u1;
                jacobian[(0, 1)] = (t7 + t6 * t8 * t25 * u0) - t6 * t17 * t19 * u0 * u1;
                jacobian[(1, 1)] =
                    ((t20 - t3 * t6 * t17 * t19) + t3 * t11 * t17 * t18) + t6 * t8 * t25 * u1;
                jacobian[(0, 0)] =
                    ((t20 - t2 * t6 * t17 * t19) + t2 * t11 * t17 * t18) + t6 * t8 * t4 * u0;
                jacobian[(1, 0)] = (t7 + t6 * t8 * t4 * u1) - t6 * t17 * t19 * u0 * u1;
            } else {
                *jacobian = Matrix2::identity();
            }
        }
        if let Some(parameters_jacobian) = parameter_jacobian {
            *parameters_jacobian = Matrix2xX::zeros(Self::NUM_DISTORTION_INTRINSICS);
            if r > 1e-8 {
                let t6 = u0 * u0 + u1 * u1;
                let t2 = ((t6).sqrt()).atan();
                let t3 = t2 * t2;
                let t8 = t3 * t3;
                let t6 = 1.0 / (t6).sqrt();
                let t10 = t8 * t8;
                parameters_jacobian[(0, 0)] = t2 * t3 * t6 * u0;
                parameters_jacobian[(1, 0)] = t2 * t3 * t6 * u1;
                parameters_jacobian[(0, 1)] = t2 * t8 * t6 * u0;
                parameters_jacobian[(1, 1)] = t2 * t8 * t6 * u1;
                parameters_jacobian[(0, 2)] = t2 * t3 * t8 * t6 * u0;
                parameters_jacobian[(1, 2)] = t2 * t3 * t8 * t6 * u1;
                parameters_jacobian[(0, 3)] = t2 * t6 * t10 * u0;
                parameters_jacobian[(1, 3)] = t2 * t6 * t10 * u1;
            }
        }

        true
    }
}

impl DistortionBase for EquidistantDistortion {
    fn set_parameters(&mut self, parameters: &DVector<f64>) -> bool {
        if parameters.len() != Self::NUM_DISTORTION_INTRINSICS {
            return false;
        }
        self.parameters.copy_from(parameters);
        self.k1 = parameters[0];
        self.k2 = parameters[1];
        self.k3 = parameters[2];
        self.k4 = parameters[3];
        true
    }
    fn get_parameters(&self, parameters: &mut DVector<f64>) -> bool {
        *parameters = DVector::from_column_slice(self.parameters.as_slice());
        true
    }
    fn type_name(&self) -> &'static str {
        "EquidistantDistortion"
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
        for _ in 0..20 {
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
            if chi2 < 1e-2 {
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
