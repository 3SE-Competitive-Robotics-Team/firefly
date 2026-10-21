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

//! Extended unified camera model corresponding to `implementation/EucmCamera.hpp`.

use crate::cameras::camera_base::{
    CameraAwarenessMaps, CameraBase, CameraState, ProjectionStatus, RectifyMaps,
    make_awareness_maps,
};
use nalgebra::{DVector, Matrix2xX, SMatrix, SVector, Vector2, Vector3, Vector4};
use std::sync::Arc;

#[derive(Clone, Debug)]
pub struct EucmCamera {
    state: CameraState,
    intrinsics: SVector<f64, 6>,
    awareness_maps: Option<CameraAwarenessMaps>,
}

impl EucmCamera {
    pub const NUM_PROJECTION_INTRINSICS: usize = 6;
    pub const NUM_INTRINSICS: usize = 6;
    #[must_use]
    pub fn new(
        (width, height): (i32, i32),
        (fu, fv, cu, cv, alpha, beta): (f64, f64, f64, f64, f64, f64),
        id: u64,
    ) -> Self {
        Self {
            state: CameraState::new(width, height, id),
            intrinsics: SVector::from_row_slice(&[fu, fv, cu, cv, alpha, beta]),
            awareness_maps: None,
        }
    }
    #[must_use]
    pub fn test_object() -> Self {
        Self::new(
            (640, 480),
            (320.0, 320.0, 320.0, 240.0, 0.75, 0.99),
            u64::MAX,
        )
    }
    #[must_use]
    pub fn create_test_object() -> Arc<dyn CameraBase> {
        Arc::new(Self::test_object())
    }
    #[must_use]
    pub fn focal_length_u(&self) -> f64 {
        self.intrinsics[0]
    }
    #[must_use]
    pub fn focal_length_v(&self) -> f64 {
        self.intrinsics[1]
    }
    #[must_use]
    pub fn image_center_u(&self) -> f64 {
        self.intrinsics[2]
    }
    #[must_use]
    pub fn image_center_v(&self) -> f64 {
        self.intrinsics[3]
    }
    #[must_use]
    pub fn alpha(&self) -> f64 {
        self.intrinsics[4]
    }
    #[must_use]
    pub fn beta(&self) -> f64 {
        self.intrinsics[5]
    }
    fn status(&self, image: &Vector2<f64>) -> ProjectionStatus {
        if !self.state.is_in_image(image) {
            return ProjectionStatus::OutsideImage;
        }
        if self.state.is_masked(image) {
            return ProjectionStatus::Masked;
        }
        ProjectionStatus::Successful
    }
    fn projection(
        &self,
        point: &Vector3<f64>,
        parameters: &[f64],
        image: &mut Vector2<f64>,
        point_jacobian: Option<&mut SMatrix<f64, 2, 3>>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> ProjectionStatus {
        let (fu, fv, cu, cv, alpha, beta) = (
            parameters[0],
            parameters[1],
            parameters[2],
            parameters[3],
            parameters[4],
            parameters[5],
        );
        let rho = (beta * (point[0] * point[0] + point[1] * point[1]) + point[2] * point[2]).sqrt();
        let rz = 1.0 / (alpha * rho + (1.0 - alpha) * point[2]);
        let image_plane = Vector2::new(point[0] * rz, point[1] * rz);
        *image = Vector2::new(fu * image_plane[0] + cu, fv * image_plane[1] + cv);
        if let Some(j) = intrinsics_jacobian {
            let den = alpha * rho + (1.0 - alpha) * point[2];
            *j = Matrix2xX::zeros(6);
            j[(0, 0)] = point[0] / den;
            j[(1, 1)] = point[1] / den;
            j[(0, 2)] = 1.0;
            j[(1, 3)] = 1.0;
            j[(0, 4)] = -fu * point[0] * (rho - point[2]) / (den * den);
            j[(1, 4)] = -fv * point[1] * (rho - point[2]) / (den * den);
            j[(0, 5)] = -alpha * fu * point[0] * (point[0] * point[0] + point[1] * point[1])
                / (2.0 * rho * den * den);
            j[(1, 5)] = -alpha * fv * point[1] * (point[0] * point[0] + point[1] * point[1])
                / (2.0 * rho * den * den);
        }
        if let Some(j) = point_jacobian {
            j[(0, 0)] = rz - (alpha * beta * point[0] * point[0]) * rz * rz / rho;
            j[(0, 0)] *= fu;
            j[(0, 1)] = -(alpha * beta * point[0] * point[1]) * rz * rz / rho;
            j[(0, 1)] *= fu;
            j[(0, 2)] = -point[0] * rz * rz * (1.0 - alpha + (alpha * point[2]) / rho);
            j[(0, 2)] *= fu;
            j[(1, 0)] = -(alpha * beta * point[0] * point[1]) * rz * rz / rho;
            j[(1, 0)] *= fv;
            j[(1, 1)] = rz - (alpha * beta * point[1] * point[1]) * rz * rz / rho;
            j[(1, 1)] *= fv;
            j[(1, 2)] = -point[1] * rz * rz * (1.0 - alpha + (alpha * point[2]) / rho);
            j[(1, 2)] *= fv;
        }
        self.status(image)
    }
    fn back_projection(&self, image: &Vector2<f64>, direction: &mut Vector3<f64>) -> bool {
        let (fu, fv, cu, cv, alpha, beta) = (
            self.intrinsics[0],
            self.intrinsics[1],
            self.intrinsics[2],
            self.intrinsics[3],
            self.intrinsics[4],
            self.intrinsics[5],
        );
        let mx = (image[0] - cu) / fu;
        let my = (image[1] - cv) / fv;
        let r_squared = mx * mx + my * my;
        let mz = (1.0 - beta * alpha * alpha * r_squared)
            / (alpha * (1.0 - (2.0 * alpha - 1.0) * beta * r_squared).sqrt() + (1.0 - alpha));
        let scale = 1.0 / (mx * mx + my * my + mz * mz).sqrt();
        *direction = Vector3::new(scale * mx, scale * my, scale * mz);
        if alpha > 0.5 && r_squared > 1.0 / (beta * (2.0 * alpha - 1.0)) {
            return false;
        }
        true
    }
}

impl CameraBase for EucmCamera {
    fn set_rectify_map(&mut self, _maps: RectifyMaps) {}
    fn get_rectify_map(&self, _maps: &mut Option<RectifyMaps>) {}

    fn state(&self) -> &CameraState {
        &self.state
    }
    fn state_mut(&mut self) -> &mut CameraState {
        &mut self.state
    }
    fn get_intrinsics(&self, intrinsics: &mut DVector<f64>) {
        *intrinsics = DVector::from_column_slice(self.intrinsics.as_slice());
    }
    fn set_intrinsics(&mut self, intrinsics: &DVector<f64>) -> bool {
        if intrinsics.len() != 6 {
            return false;
        }
        self.intrinsics.copy_from(intrinsics);
        true
    }
    fn num_intrinsics(&self) -> usize {
        6
    }
    fn type_name(&self) -> String {
        "EUCMCamera".to_owned()
    }
    fn distortion_type(&self) -> &'static str {
        "NoDistortion"
    }
    fn project(&self, point: &Vector3<f64>, image: &mut Vector2<f64>) -> ProjectionStatus {
        self.projection(point, self.intrinsics.as_slice(), image, None, None)
    }
    fn project_with_jacobians(
        &self,
        point: &Vector3<f64>,
        image: &mut Vector2<f64>,
        point_jacobian: &mut SMatrix<f64, 2, 3>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> ProjectionStatus {
        self.projection(
            point,
            self.intrinsics.as_slice(),
            image,
            Some(point_jacobian),
            intrinsics_jacobian,
        )
    }
    fn project_with_external_parameters(
        &self,
        point: &Vector3<f64>,
        parameters: &DVector<f64>,
        image: &mut Vector2<f64>,
        point_jacobian: Option<&mut SMatrix<f64, 2, 3>>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> ProjectionStatus {
        if parameters.len() < Self::NUM_INTRINSICS {
            return ProjectionStatus::Invalid;
        }
        let Some(point_jacobian) = point_jacobian else {
            return ProjectionStatus::Invalid;
        };
        self.projection(
            point,
            parameters.as_slice(),
            image,
            Some(point_jacobian),
            intrinsics_jacobian,
        )
    }
    fn project_homogeneous(
        &self,
        point: &Vector4<f64>,
        image: &mut Vector2<f64>,
    ) -> ProjectionStatus {
        self.project(&(point.fixed_rows::<3>(0) / point[3]), image)
    }
    fn project_homogeneous_with_jacobians(
        &self,
        point: &Vector4<f64>,
        image: &mut Vector2<f64>,
        point_jacobian: &mut SMatrix<f64, 2, 4>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> ProjectionStatus {
        let point3d = point.fixed_rows::<3>(0) / point[3];
        let mut j = SMatrix::<f64, 2, 3>::repeat(f64::NAN);
        let status = self.project_with_jacobians(&point3d, image, &mut j, intrinsics_jacobian);
        point_jacobian.column_mut(3).fill(0.0);
        point_jacobian
            .fixed_columns_mut::<3>(0)
            .copy_from(&(j / point[3]));
        status
    }
    fn project_homogeneous_with_external_parameters(
        &self,
        point: &Vector4<f64>,
        parameters: &DVector<f64>,
        image: &mut Vector2<f64>,
        point_jacobian: &mut SMatrix<f64, 2, 4>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> ProjectionStatus {
        let point3d = point.fixed_rows::<3>(0) / point[3];
        let mut j = SMatrix::<f64, 2, 3>::repeat(f64::NAN);
        let status = self.project_with_external_parameters(
            &point3d,
            parameters,
            image,
            Some(&mut j),
            intrinsics_jacobian,
        );
        point_jacobian.column_mut(3).fill(0.0);
        point_jacobian
            .fixed_columns_mut::<3>(0)
            .copy_from(&(j / point[3]));
        status
    }
    fn back_project(&self, image: &Vector2<f64>, direction: &mut Vector3<f64>) -> bool {
        self.back_projection(image, direction)
    }
    /// The Jacobian output is zero and an error is logged on the successful branch.
    fn back_project_with_jacobian(
        &self,
        image: &Vector2<f64>,
        direction: &mut Vector3<f64>,
        point_jacobian: &mut SMatrix<f64, 3, 2>,
    ) -> bool {
        if !self.back_projection(image, direction) {
            return false;
        }
        point_jacobian.fill(0.0);
        log::error!("pointJacobian for EUCM Camera back-projection not implemented.");
        true
    }
    fn initialise_camera_awareness_maps(&mut self) -> bool {
        self.awareness_maps = Some(make_awareness_maps(self));
        true
    }
    fn camera_awareness_maps(&self) -> Option<&CameraAwarenessMaps> {
        self.awareness_maps
            .as_ref()
            .filter(|maps| !maps.rays.data.is_empty() && !maps.image_jacobians.data.is_empty())
    }
}
