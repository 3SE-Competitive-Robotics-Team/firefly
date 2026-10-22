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

//! Pinhole camera model corresponding to `implementation/PinholeCamera.hpp`.

pub use crate::cameras::camera_base::RectifyMaps;
use crate::cameras::{
    camera_base::{
        CameraAwarenessMaps, CameraBase, CameraState, ProjectionStatus, make_awareness_maps,
    },
    distortion_base::DistortionBase,
    no_distortion::NoDistortion,
};
use nalgebra::{DVector, Matrix2, Matrix2xX, SMatrix, Vector2, Vector3, Vector4};
use purecv::core::{
    DynamicData, DynamicMatrix, Matrix,
    types::{BorderTypes, Scalar},
};
use purecv::imgproc::geometric::{InterpolationFlags, remap};
use std::sync::Arc;

/// Distortion fixtures used by the camera model's test-object factories.
pub trait DistortionFixture: DistortionBase {
    fn test_distortion() -> Self;
}
impl DistortionFixture for NoDistortion {
    fn test_distortion() -> Self {
        Self::test_object()
    }
}
impl DistortionFixture
    for crate::cameras::radial_tangential_distortion::RadialTangentialDistortion
{
    fn test_distortion() -> Self {
        Self::test_object()
    }
}
impl DistortionFixture
    for crate::cameras::radial_tangential_distortion8::RadialTangentialDistortion8
{
    fn test_distortion() -> Self {
        Self::test_object()
    }
}
impl DistortionFixture for crate::cameras::equidistant_distortion::EquidistantDistortion {
    fn test_distortion() -> Self {
        Self::test_object()
    }
}

impl<D: DistortionFixture> PinholeCamera<D> {
    #[must_use]
    pub fn test_object() -> Self {
        Self::new(
            (752, 480),
            (350.0, 360.0, 378.0, 238.0),
            D::test_distortion(),
            u64::MAX,
        )
    }
    #[must_use]
    pub fn create_test_object() -> Arc<dyn CameraBase>
    where
        D: 'static,
    {
        Arc::new(Self::test_object())
    }
}

#[derive(Clone, Debug)]
pub struct PinholeCamera<D> {
    state: CameraState,
    distortion: D,
    intrinsics: DVector<f64>,
    fu: f64,
    fv: f64,
    cu: f64,
    cv: f64,
    one_over_fu: f64,
    one_over_fv: f64,
    rectify_maps: Option<RectifyMaps>,
    undistorted_parameters: (i32, i32, f64, f64, f64, f64),
    awareness_maps: Option<CameraAwarenessMaps>,
}

impl<D: DistortionBase> PinholeCamera<D> {
    pub const NUM_PROJECTION_INTRINSICS: usize = 4;
    pub fn new(
        (width, height): (i32, i32),
        (fu, fv, cu, cv): (f64, f64, f64, f64),
        distortion: D,
        id: u64,
    ) -> Self {
        let mut intrinsics = DVector::zeros(4 + distortion.num_distortion_intrinsics());
        intrinsics
            .fixed_rows_mut::<4>(0)
            .copy_from(&Vector4::new(fu, fv, cu, cv));
        Self {
            state: CameraState::new(width, height, id),
            distortion,
            intrinsics,
            fu,
            fv,
            cu,
            cv,
            one_over_fu: 1.0 / fu,
            one_over_fv: 1.0 / fv,
            rectify_maps: None,
            undistorted_parameters: (0, 0, 0.0, 0.0, 0.0, 0.0),
            awareness_maps: None,
        }
    }
    pub fn focal_length_u(&self) -> f64 {
        self.fu
    }
    pub fn focal_length_v(&self) -> f64 {
        self.fv
    }
    pub fn image_center_u(&self) -> f64 {
        self.cu
    }
    pub fn image_center_v(&self) -> f64 {
        self.cv
    }
    pub fn distortion(&self) -> &D {
        &self.distortion
    }
    pub fn initialise_undistort_maps(&mut self) -> bool {
        let focal_sum = self.fu + self.fv;
        let f = focal_sum * 0.5;
        self.initialise_undistort_maps_with_parameters(
            self.state.image_width,
            self.state.image_height,
            f,
            f,
            f64::from(self.state.image_width) * 0.5 + 0.5,
            f64::from(self.state.image_height) * 0.5 + 0.5,
        )
    }
    pub fn initialise_undistort_maps_with_parameters(
        &mut self,
        width: i32,
        height: i32,
        fu: f64,
        fv: f64,
        cu: f64,
        cv: f64,
    ) -> bool {
        self.undistorted_parameters = (width, height, fu, fv, cu, cv);
        let inverse_k = Matrix2::new(1.0 / fu, 0.0, 0.0, 1.0 / fv);
        let actual_k = Matrix2::new(self.fu, 0.0, 0.0, self.fv);
        let actual_center = Vector2::new(self.cu, self.cv);
        let center = Vector2::new(cu, cv);
        let mut coordinates = Matrix::<i16>::new(height as usize, width as usize, 2);
        let mut fractions = Matrix::<u16>::new(height as usize, width as usize, 1);
        for y in 0..height as usize {
            for x in 0..width as usize {
                let plane = inverse_k * (Vector2::new(x as f64, y as f64) - center);
                let ray = 0.25 * Vector3::new(plane[0], plane[1], 1.0);
                let homogeneous = Vector4::new(ray[0], ray[1], ray[2], 1.0);
                let transformed = homogeneous.fixed_rows::<3>(0) / homogeneous[3];
                let projected = transformed.fixed_rows::<2>(0) / transformed[2];
                let mut distorted = Vector2::repeat(f64::NAN);
                self.distortion.distort(&projected, &mut distorted);
                let mapped = actual_k * distorted + actual_center;
                // Float maps are quantized to OpenCV's 1/32-pixel fixed-point grid.
                let sx = ((mapped[0] as f32) * 32.0).round_ties_even() as i32;
                let sy = ((mapped[1] as f32) * 32.0).round_ties_even() as i32;
                let i = y * width as usize + x;
                coordinates.data[2 * i] =
                    (sx >> 5).clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16;
                coordinates.data[2 * i + 1] =
                    (sy >> 5).clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16;
                fractions.data[i] = (((sy & 31) << 5) + (sx & 31)) as u16;
            }
        }
        self.rectify_maps = Some(RectifyMaps {
            coordinates: Arc::new(coordinates),
            fractions: Arc::new(fractions),
        });
        true
    }
    /// # Panics
    /// Panics if the required assertion fails: `maps.coordinates.cols, 0`.
    pub fn undistorted_pinhole_camera(&self) -> PinholeCamera<NoDistortion> {
        let maps = self
            .rectify_maps
            .as_ref()
            .expect("undistortion maps are not initialized");
        assert_ne!(maps.coordinates.cols, 0);
        let (width, height, fu, fv, cu, cv) = self.undistorted_parameters;
        PinholeCamera::new((width, height), (fu, fv, cu, cv), NoDistortion, u64::MAX)
    }
    /// # Panics
    /// Panics if a required resource/invariant is absent: `undistortion maps are not initialized`.
    pub fn undistort_image(&self, source: &DynamicMatrix, destination: &mut DynamicMatrix) -> bool {
        let maps = self
            .rectify_maps
            .as_ref()
            .expect("undistortion maps are not initialized");
        let mut map_x = Matrix::<f32>::new(maps.coordinates.rows, maps.coordinates.cols, 1);
        let mut map_y = map_x.clone();
        for i in 0..map_x.data.len() {
            map_x.data[i] = f32::from(maps.coordinates.data[2 * i])
                + f32::from(maps.fractions.data[i] & 31) / 32.0;
            map_y.data[i] = f32::from(maps.coordinates.data[2 * i + 1])
                + f32::from((maps.fractions.data[i] >> 5) & 31) / 32.0;
        }
        macro_rules! remap_data {
            ($image:expr, $variant:ident, $zero:expr) => {
                match remap(
                    $image,
                    &map_x,
                    &map_y,
                    InterpolationFlags::Linear,
                    BorderTypes::Constant,
                    Scalar::all($zero),
                ) {
                    Ok(image) => DynamicData::$variant(image),
                    Err(error) => {
                        log::error!("image remapping failed: {error}");
                        return false;
                    }
                }
            };
        }
        destination.data = match &source.data {
            DynamicData::U8(image) => remap_data!(image, U8, 0_u8),
            DynamicData::U16(image) => remap_data!(image, U16, 0_u16),
            DynamicData::I16(image) => remap_data!(image, I16, 0_i16),
            DynamicData::F32(image) => remap_data!(image, F32, 0.0_f32),
            DynamicData::F64(image) => remap_data!(image, F64, 0.0_f64),
            DynamicData::I8(_) | DynamicData::I32(_) => {
                log::error!("image depth is not supported by OpenCV remap");
                return false;
            }
        };
        true
    }
    fn status(&self, image: &Vector2<f64>, z: f64) -> ProjectionStatus {
        if !self.state.is_in_image(image) {
            return ProjectionStatus::OutsideImage;
        }
        if self.state.is_masked(image) {
            return ProjectionStatus::Masked;
        }
        if z > 0.0 {
            ProjectionStatus::Successful
        } else {
            ProjectionStatus::Behind
        }
    }
    fn projection_jacobian(
        point: &Vector3<f64>,
        distortion: &Matrix2<f64>,
        fu: f64,
        fv: f64,
    ) -> SMatrix<f64, 2, 3> {
        let rz = 1.0 / point[2];
        let rz2 = rz * rz;
        SMatrix::<f64, 2, 3>::new(
            fu * distortion[(0, 0)] * rz,
            fu * distortion[(0, 1)] * rz,
            -fu * (point[0] * distortion[(0, 0)] + point[1] * distortion[(0, 1)]) * rz2,
            fv * distortion[(1, 0)] * rz,
            fv * distortion[(1, 1)] * rz,
            -fv * (point[0] * distortion[(1, 0)] + point[1] * distortion[(1, 1)]) * rz2,
        )
    }
    fn project_derivatives(
        &self,
        point: &Vector3<f64>,
        image: &mut Vector2<f64>,
        point_jacobian: Option<&mut SMatrix<f64, 2, 3>>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
        external: Option<&DVector<f64>>,
    ) -> ProjectionStatus
    where
        D: 'static,
    {
        if point[2].abs() < 1.0e-12 {
            return ProjectionStatus::Invalid;
        }
        if external.is_some_and(|parameters| parameters.len() < self.num_intrinsics()) {
            return ProjectionStatus::Invalid;
        }
        let (fu, fv, cu, cv) = external.map_or((self.fu, self.fv, self.cu, self.cv), |p| {
            (p[0], p[1], p[2], p[3])
        });
        let rz = 1.0 / point[2];
        let plane = Vector2::new(point[0] * rz, point[1] * rz);
        let mut distorted = Vector2::repeat(f64::NAN);
        let mut distortion_jacobian = Matrix2::repeat(f64::NAN);
        let mut distortion_parameters_jacobian =
            Matrix2xX::zeros(self.distortion.num_distortion_intrinsics());
        let parameters_requested = intrinsics_jacobian.is_some();
        let parameters_jacobian = if parameters_requested {
            Some(&mut distortion_parameters_jacobian)
        } else {
            None
        };
        let success = if let Some(parameters) = external {
            let count = self.distortion.num_distortion_intrinsics();
            let distortion_parameters = parameters
                .rows(parameters.len() - count, count)
                .into_owned();
            self.distortion.distort_with_external_parameters(
                &plane,
                &distortion_parameters,
                &mut distorted,
                Some(&mut distortion_jacobian),
                parameters_jacobian,
            )
        } else {
            self.distortion.distort_with_jacobians(
                &plane,
                &mut distorted,
                &mut distortion_jacobian,
                parameters_jacobian,
            )
        };
        if let Some(jacobian) = intrinsics_jacobian {
            *jacobian = Matrix2xX::zeros(self.num_intrinsics());
            jacobian[(0, 0)] = distorted[0];
            jacobian[(1, 1)] = distorted[1];
            jacobian[(0, 2)] = 1.0;
            jacobian[(1, 3)] = 1.0;
            if self.distortion.num_distortion_intrinsics() > 0 {
                let block = Matrix2::new(fu, 0.0, 0.0, fv) * distortion_parameters_jacobian;
                jacobian.columns_mut(4, block.ncols()).copy_from(&block);
            }
        }
        if let Some(jacobian) = point_jacobian {
            *jacobian = Self::projection_jacobian(point, &distortion_jacobian, fu, fv);
        }
        *image = Vector2::new(fu * distorted[0] + cu, fv * distorted[1] + cv);
        if !success {
            return ProjectionStatus::Invalid;
        }
        self.status(image, point[2])
    }
}

impl<D: DistortionBase + 'static> CameraBase for PinholeCamera<D> {
    fn set_rectify_map(&mut self, maps: RectifyMaps) {
        self.rectify_maps = Some(maps);
    }
    fn get_rectify_map(&self, maps: &mut Option<RectifyMaps>) {
        *maps = self.rectify_maps.as_ref().map(|stored| RectifyMaps {
            coordinates: Arc::new((*stored.coordinates).clone()),
            fractions: Arc::new((*stored.fractions).clone()),
        });
    }

    fn state(&self) -> &CameraState {
        &self.state
    }
    fn state_mut(&mut self) -> &mut CameraState {
        &mut self.state
    }
    fn get_intrinsics(&self, intrinsics: &mut DVector<f64>) {
        *intrinsics = self.intrinsics.clone();
        let count = self.distortion.num_distortion_intrinsics();
        if count > 0 {
            let mut parameters = DVector::zeros(count);
            self.distortion.get_parameters(&mut parameters);
            intrinsics.rows_mut(4, count).copy_from(&parameters);
        }
    }
    fn set_intrinsics(&mut self, intrinsics: &DVector<f64>) -> bool {
        if intrinsics.len() != self.num_intrinsics() {
            return false;
        }
        self.intrinsics = intrinsics.clone();
        self.fu = intrinsics[0];
        self.fv = intrinsics[1];
        self.cu = intrinsics[2];
        self.cv = intrinsics[3];
        let count = self.distortion.num_distortion_intrinsics();
        self.distortion.set_parameters(
            &intrinsics
                .rows(intrinsics.len() - count, count)
                .into_owned(),
        );
        self.one_over_fu = 1.0 / self.fu;
        self.one_over_fv = 1.0 / self.fv;
        true
    }
    fn num_intrinsics(&self) -> usize {
        4 + self.distortion.num_distortion_intrinsics()
    }
    fn type_name(&self) -> String {
        format!("PinholeCamera<{}>", self.distortion.type_name())
    }
    fn distortion_type(&self) -> &'static str {
        self.distortion.type_name()
    }
    fn project(&self, point: &Vector3<f64>, image: &mut Vector2<f64>) -> ProjectionStatus {
        if point[2].abs() < 1.0e-12 {
            return ProjectionStatus::Invalid;
        }
        let rz = 1.0 / point[2];
        let plane = Vector2::new(point[0] * rz, point[1] * rz);
        let mut distorted = Vector2::repeat(f64::NAN);
        if !self.distortion.distort(&plane, &mut distorted) {
            return ProjectionStatus::Invalid;
        }
        *image = Vector2::new(
            self.fu * distorted[0] + self.cu,
            self.fv * distorted[1] + self.cv,
        );
        self.status(image, point[2])
    }
    fn project_with_jacobians(
        &self,
        point: &Vector3<f64>,
        image: &mut Vector2<f64>,
        point_jacobian: &mut SMatrix<f64, 2, 3>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> ProjectionStatus {
        self.project_derivatives(
            point,
            image,
            Some(point_jacobian),
            intrinsics_jacobian,
            None,
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
        self.project_derivatives(
            point,
            image,
            point_jacobian,
            intrinsics_jacobian,
            Some(parameters),
        )
    }
    fn project_homogeneous(
        &self,
        point: &Vector4<f64>,
        image: &mut Vector2<f64>,
    ) -> ProjectionStatus {
        let head = point.fixed_rows::<3>(0).into_owned();
        if point[3] < 0.0 {
            self.project(&(-head), image)
        } else {
            self.project(&head, image)
        }
    }
    fn project_homogeneous_with_jacobians(
        &self,
        point: &Vector4<f64>,
        image: &mut Vector2<f64>,
        point_jacobian: &mut SMatrix<f64, 2, 4>,
        intrinsics_jacobian: Option<&mut Matrix2xX<f64>>,
    ) -> ProjectionStatus {
        let sign = if point[3] < 0.0 { -1.0 } else { 1.0 };
        let head = sign * point.fixed_rows::<3>(0).into_owned();
        let mut j = SMatrix::<f64, 2, 3>::repeat(f64::NAN);
        let status = self.project_with_jacobians(&head, image, &mut j, intrinsics_jacobian);
        point_jacobian.column_mut(3).fill(0.0);
        point_jacobian
            .fixed_columns_mut::<3>(0)
            .copy_from(&(sign * j));
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
        let sign = if point[3] < 0.0 { -1.0 } else { 1.0 };
        let head = sign * point.fixed_rows::<3>(0).into_owned();
        let mut j = SMatrix::<f64, 2, 3>::repeat(f64::NAN);
        let status = self.project_with_external_parameters(
            &head,
            parameters,
            image,
            Some(&mut j),
            intrinsics_jacobian,
        );
        point_jacobian.column_mut(3).fill(0.0);
        point_jacobian
            .fixed_columns_mut::<3>(0)
            .copy_from(&(sign * j));
        status
    }
    fn back_project(&self, image: &Vector2<f64>, direction: &mut Vector3<f64>) -> bool {
        let plane = Vector2::new(
            (image[0] - self.cu) * self.one_over_fu,
            (image[1] - self.cv) * self.one_over_fv,
        );
        let mut undistorted = Vector2::repeat(f64::NAN);
        let success = self.distortion.undistort(&plane, &mut undistorted);
        *direction = Vector3::new(undistorted[0], undistorted[1], 1.0);
        success
    }
    fn back_project_with_jacobian(
        &self,
        image: &Vector2<f64>,
        direction: &mut Vector3<f64>,
        point_jacobian: &mut SMatrix<f64, 3, 2>,
    ) -> bool {
        let plane = Vector2::new(
            (image[0] - self.cu) * self.one_over_fu,
            (image[1] - self.cv) * self.one_over_fv,
        );
        let mut undistorted = Vector2::repeat(f64::NAN);
        let mut undistortion_jacobian = Matrix2::repeat(f64::NAN);
        let success = self.distortion.undistort_with_jacobian(
            &plane,
            &mut undistorted,
            &mut undistortion_jacobian,
        );
        *direction = Vector3::new(undistorted[0], undistorted[1], 1.0);
        point_jacobian.fill(0.0);
        point_jacobian.fixed_view_mut::<2, 2>(0, 0).copy_from(
            &(undistortion_jacobian * Matrix2::new(self.one_over_fu, 0.0, 0.0, self.one_over_fv)),
        );
        success
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
