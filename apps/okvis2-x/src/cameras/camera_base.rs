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

//! Camera interface, masks and sampling corresponding to `CameraBase.hpp` and `CameraBase.cpp`.

use nalgebra::{DVector, Matrix2xX, Matrix3xX, Matrix4xX, SMatrix, Vector2, Vector3, Vector4};
use purecv::core::Matrix;
use rand::RngExt;
use std::{any::Any, sync::Arc};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProjectionStatus {
    Successful,
    OutsideImage,
    Masked,
    Behind,
    Invalid,
}

#[derive(Clone, Debug, Default)]
pub struct CameraState {
    pub image_width: i32,
    pub image_height: i32,
    pub id: u64,
    mask: Option<Arc<Matrix<u8>>>,
}

impl CameraState {
    #[must_use]
    pub fn new(image_width: i32, image_height: i32, id: u64) -> Self {
        Self {
            image_width,
            image_height,
            id,
            mask: None,
        }
    }
    pub fn set_mask(&mut self, mask: Arc<Matrix<u8>>) -> bool {
        if mask.channels != 1
            || mask.rows != self.image_height as usize
            || mask.cols != self.image_width as usize
        {
            return false;
        }
        self.mask = Some(mask);
        true
    }
    pub fn remove_mask(&mut self) -> bool {
        self.mask = None;
        true
    }
    #[must_use]
    pub fn has_mask(&self) -> bool {
        self.mask.as_ref().is_some_and(|mask| !mask.data.is_empty())
    }
    #[must_use]
    pub fn mask(&self) -> Option<&Arc<Matrix<u8>>> {
        self.mask.as_ref()
    }
    #[must_use]
    pub fn is_in_image(&self, point: &Vector2<f64>) -> bool {
        if point[0] < 0.0 || point[1] < 0.0 {
            return false;
        }
        if point[0] >= f64::from(self.image_width) || point[1] >= f64::from(self.image_height) {
            return false;
        }
        true
    }
    /// A nonzero mask byte marks a point as masked.
    #[must_use]
    /// # Panics
    /// Panics if a required resource/invariant is absent: `mask is present`.
    pub fn is_masked(&self, point: &Vector2<f64>) -> bool {
        if !self.is_in_image(point) {
            return true;
        }
        if !self.has_mask() {
            return false;
        }
        let mask = self.mask.as_ref().expect("mask is present");
        mask.data[point[1] as usize * mask.cols + point[0] as usize] != 0
    }
}

/// Pixelwise ray and Jacobian data; unwritten Jacobian entries have no value.
#[derive(Clone, Debug)]
pub struct CameraAwarenessMaps {
    pub rays: Arc<Matrix<f32>>,
    pub image_jacobians: Arc<Matrix<Option<f32>>>,
}

/// Fixed-point maps: signed integer coordinates and packed 5-bit fractions.
#[derive(Clone, Debug)]
pub struct RectifyMaps {
    pub coordinates: Arc<Matrix<i16>>,
    pub fractions: Arc<Matrix<u16>>,
}

pub trait CameraBase: Any + Send + Sync {
    fn state(&self) -> &CameraState;
    fn state_mut(&mut self) -> &mut CameraState;
    fn get_intrinsics(&self, intrinsics: &mut DVector<f64>);
    fn set_intrinsics(&mut self, intrinsics: &DVector<f64>) -> bool;
    fn set_rectify_map(&mut self, maps: RectifyMaps);
    fn get_rectify_map(&self, maps: &mut Option<RectifyMaps>);
    fn num_intrinsics(&self) -> usize;
    fn type_name(&self) -> String;
    fn distortion_type(&self) -> &'static str;
    fn project(&self, point: &Vector3<f64>, image: &mut Vector2<f64>) -> ProjectionStatus;
    fn project_with_jacobians(
        &self,
        point: &Vector3<f64>,
        image: &mut Vector2<f64>,
        point_jacobian: &mut SMatrix<f64, 2, 3>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> ProjectionStatus;
    fn project_with_external_parameters(
        &self,
        point: &Vector3<f64>,
        parameters: &DVector<f64>,
        image: &mut Vector2<f64>,
        point_jacobian: Option<&mut SMatrix<f64, 2, 3>>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> ProjectionStatus;
    fn project_homogeneous(
        &self,
        point: &Vector4<f64>,
        image: &mut Vector2<f64>,
    ) -> ProjectionStatus;
    fn project_homogeneous_with_jacobians(
        &self,
        point: &Vector4<f64>,
        image: &mut Vector2<f64>,
        point_jacobian: &mut SMatrix<f64, 2, 4>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> ProjectionStatus;
    fn project_homogeneous_with_external_parameters(
        &self,
        point: &Vector4<f64>,
        parameters: &DVector<f64>,
        image: &mut Vector2<f64>,
        point_jacobian: &mut SMatrix<f64, 2, 4>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> ProjectionStatus;
    fn back_project(&self, image: &Vector2<f64>, direction: &mut Vector3<f64>) -> bool;
    fn back_project_with_jacobian(
        &self,
        image: &Vector2<f64>,
        direction: &mut Vector3<f64>,
        point_jacobian: &mut SMatrix<f64, 3, 2>,
    ) -> bool;
    fn initialise_camera_awareness_maps(&mut self) -> bool;
    fn camera_awareness_maps(&self) -> Option<&CameraAwarenessMaps>;

    fn set_id(&mut self, id: u64) {
        self.state_mut().id = id;
    }
    fn id(&self) -> u64 {
        self.state().id
    }
    fn image_width(&self) -> u32 {
        self.state().image_width as u32
    }
    fn image_height(&self) -> u32 {
        self.state().image_height as u32
    }
    fn set_mask(&mut self, mask: Arc<Matrix<u8>>) -> bool {
        self.state_mut().set_mask(mask)
    }
    fn remove_mask(&mut self) -> bool {
        self.state_mut().remove_mask()
    }
    fn has_mask(&self) -> bool {
        self.state().has_mask()
    }
    fn mask(&self) -> Option<&Arc<Matrix<u8>>> {
        self.state().mask()
    }

    fn project_batch(
        &self,
        points: &Matrix3xX<f64>,
        images: &mut Matrix2xX<f64>,
        mut statuses: Option<&mut Vec<ProjectionStatus>>,
    ) {
        for i in 0..points.ncols() {
            let mut image = images.column(i).into_owned();
            let status = self.project(&points.column(i).into_owned(), &mut image);
            images.set_column(i, &image);
            if let Some(statuses) = statuses.as_mut() {
                statuses.push(status);
            }
        }
    }
    fn project_homogeneous_batch(
        &self,
        points: &Matrix4xX<f64>,
        images: &mut Matrix2xX<f64>,
        mut statuses: Option<&mut Vec<ProjectionStatus>>,
    ) {
        for i in 0..points.ncols() {
            let mut image = images.column(i).into_owned();
            let status = self.project_homogeneous(&points.column(i).into_owned(), &mut image);
            images.set_column(i, &image);
            if let Some(statuses) = statuses.as_mut() {
                statuses.push(status);
            }
        }
    }
    fn back_project_batch(
        &self,
        images: &Matrix2xX<f64>,
        directions: &mut Matrix3xX<f64>,
        mut success: Option<&mut Vec<bool>>,
    ) -> bool {
        for i in 0..images.ncols() {
            let mut direction = directions.column(i).into_owned();
            let result = self.back_project(&images.column(i).into_owned(), &mut direction);
            if let Some(success) = success.as_mut() {
                success.push(result);
            }
            directions.set_column(i, &direction);
        }
        true
    }
    fn back_project_homogeneous(&self, image: &Vector2<f64>, direction: &mut Vector4<f64>) -> bool {
        let mut ray = direction.fixed_rows::<3>(0).into_owned();
        let success = self.back_project(image, &mut ray);
        direction.fixed_rows_mut::<3>(0).copy_from(&ray);
        direction[3] = 1.0;
        success
    }
    fn back_project_homogeneous_with_jacobian(
        &self,
        image: &Vector2<f64>,
        direction: &mut Vector4<f64>,
        point_jacobian: &mut SMatrix<f64, 4, 2>,
    ) -> bool {
        let mut ray = direction.fixed_rows::<3>(0).into_owned();
        let mut j = point_jacobian.fixed_view::<3, 2>(0, 0).into_owned();
        let success = self.back_project_with_jacobian(image, &mut ray, &mut j);
        direction.fixed_rows_mut::<3>(0).copy_from(&ray);
        direction[3] = 1.0;
        point_jacobian.row_mut(3).fill(0.0);
        point_jacobian.fixed_view_mut::<3, 2>(0, 0).copy_from(&j);
        success
    }
    fn back_project_homogeneous_batch(
        &self,
        images: &Matrix2xX<f64>,
        directions: &mut Matrix4xX<f64>,
        mut success: Option<&mut Vec<bool>>,
    ) -> bool {
        directions.row_mut(3).fill(1.0);
        for i in 0..images.ncols() {
            let mut direction = directions.fixed_view::<3, 1>(0, i).into_owned();
            let result = self.back_project(&images.column(i).into_owned(), &mut direction);
            if let Some(success) = success.as_mut() {
                success.push(result);
            }
            directions
                .fixed_view_mut::<3, 1>(0, i)
                .copy_from(&direction);
        }
        true
    }
    fn create_random_image_point(&self) -> Vector2<f64> {
        let mut rng = rand::rng();
        let random = Vector2::new(rng.random_range(-1.0..=1.0), rng.random_range(-1.0..=1.0));
        let mut point = (random + Vector2::repeat(1.0)) * 0.5;
        point[0] *= f64::from(self.state().image_width) - 0.022;
        point[0] += 0.011;
        point[1] *= f64::from(self.state().image_height) - 0.022;
        point[1] += 0.011;
        point
    }
    fn create_random_visible_point(&self, min_distance: f64, max_distance: f64) -> Vector3<f64> {
        let image = self.create_random_image_point();
        let mut rng = rand::rng();
        let depth: Vector2<f64> =
            Vector2::new(rng.random_range(-1.0..=1.0), rng.random_range(-1.0..=1.0));
        let mut ray = Vector3::zeros();
        self.back_project(&image, &mut ray);
        let norm_squared = ray.norm_squared();
        if norm_squared > 0.0 {
            ray /= norm_squared.sqrt();
        }
        ray *= 0.5 * (max_distance - min_distance) * (depth[0] + 1.0) + min_distance;
        ray
    }
    fn create_random_visible_homogeneous_point(
        &self,
        min_distance: f64,
        max_distance: f64,
    ) -> Vector4<f64> {
        let point = self.create_random_visible_point(min_distance, max_distance);
        Vector4::new(point[0], point[1], point[2], 1.0)
    }
}

pub(crate) fn make_awareness_maps<C: CameraBase>(camera: &C) -> CameraAwarenessMaps {
    let rows = camera.image_height() as usize;
    let cols = camera.image_width() as usize;
    let mut rays = Matrix::<f32>::new(rows, cols, 3);
    let mut image_jacobians = Matrix::<Option<f32>>::new(rows, cols, 6);
    for v in 0..rows {
        for u in 0..cols {
            let mut ray = Vector3::zeros();
            if camera.back_project(&Vector2::new(u as f64, v as f64), &mut ray) {
                let norm_squared = ray.norm_squared();
                if norm_squared > 0.0 {
                    ray /= norm_squared.sqrt();
                }
            } else {
                ray.fill(0.0);
            }
            let offset = (v * cols + u) * 3;
            for c in 0..3 {
                rays.data[offset + c] = ray[c] as f32;
            }
            let mut image = Vector2::zeros();
            let mut jacobian = SMatrix::<f64, 2, 3>::zeros();
            if camera.project_with_jacobians(&ray, &mut image, &mut jacobian, None)
                == ProjectionStatus::Successful
            {
                let offset = (v * cols + u) * 6;
                for row in 0..2 {
                    for col in 0..3 {
                        image_jacobians.data[offset + row * 3 + col] =
                            Some(jacobian[(row, col)] as f32);
                    }
                }
            }
        }
    }
    CameraAwarenessMaps {
        rays: Arc::new(rays),
        image_jacobians: Arc::new(image_jacobians),
    }
}
