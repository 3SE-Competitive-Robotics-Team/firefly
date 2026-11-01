// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/camera/generic_camera.hpp` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
use super::{
    double_sphere_camera::DoubleSphereCamera, extended_camera::ExtendedUnifiedCamera,
    kannala_brandt_camera4::KannalaBrandtCamera4, pinhole_camera::PinholeCamera,
    pinhole_radtan8_camera::PinholeRadtan8Camera, unified_camera::UnifiedCamera,
};
use crate::scalar::BasaltScalar;
use nalgebra::{DVector, Matrix4, SMatrix, SVector, Vector2, Vector4};

/// The six alternatives in the upstream variant, in their original order.
#[derive(Clone, Debug)]
pub enum GenericCamera<S: BasaltScalar = f64> {
    ExtendedUnified(ExtendedUnifiedCamera<S>),
    DoubleSphere(DoubleSphereCamera<S>),
    KannalaBrandt4(KannalaBrandtCamera4<S>),
    Unified(UnifiedCamera<S>),
    Pinhole(PinholeCamera<S>),
    PinholeRadtan8(PinholeRadtan8Camera<S>),
}
macro_rules! visit {
    ($value:expr,$camera:ident,$expression:expr) => {
        match $value {
            GenericCamera::ExtendedUnified($camera) => $expression,
            GenericCamera::DoubleSphere($camera) => $expression,
            GenericCamera::KannalaBrandt4($camera) => $expression,
            GenericCamera::Unified($camera) => $expression,
            GenericCamera::Pinhole($camera) => $expression,
            GenericCamera::PinholeRadtan8($camera) => $expression,
        }
    };
}
impl<S: BasaltScalar> Default for GenericCamera<S> {
    fn default() -> Self {
        Self::ExtendedUnified(ExtendedUnifiedCamera::default())
    }
}
impl<S: BasaltScalar> GenericCamera<S> {
    /// Construct by upstream model name; unknown names retain the default EUCM.
    #[must_use]
    pub fn from_name(name: &str) -> Self {
        match name {
            "ds" => Self::DoubleSphere(DoubleSphereCamera::default()),
            "kb4" => Self::KannalaBrandt4(KannalaBrandtCamera4::default()),
            "ucm" => Self::Unified(UnifiedCamera::default()),
            "pinhole" => Self::Pinhole(PinholeCamera::default()),
            "pinhole-radtan8" => Self::PinholeRadtan8(PinholeRadtan8Camera::default()),
            _ => Self::default(),
        }
    }
    #[must_use]
    pub fn name(&self) -> &'static str {
        match self {
            Self::ExtendedUnified(_) => ExtendedUnifiedCamera::<S>::NAME,
            Self::DoubleSphere(_) => DoubleSphereCamera::<S>::NAME,
            Self::KannalaBrandt4(_) => KannalaBrandtCamera4::<S>::NAME,
            Self::Unified(_) => UnifiedCamera::<S>::NAME,
            Self::Pinhole(_) => PinholeCamera::<S>::NAME,
            Self::PinholeRadtan8(_) => PinholeRadtan8Camera::<S>::NAME,
        }
    }
    #[must_use]
    pub fn num_parameters(&self) -> usize {
        visit!(self, camera, camera.param().len())
    }
    #[must_use]
    pub fn param(&self) -> DVector<S> {
        visit!(
            self,
            camera,
            DVector::from_column_slice(camera.param().as_slice())
        )
    }
    pub fn set_from_init(&mut self, init: Vector4<S>) {
        visit!(self, camera, camera.set_from_init(init));
    }
    /// # Panics
    /// Panics if the increment length differs from the camera's parameter count.
    pub fn apply_increment(&mut self, inc: &DVector<S>) {
        assert_eq!(inc.len(), self.num_parameters());
        visit!(
            self,
            camera,
            camera.increment(&SVector::from_column_slice(inc.as_slice()))
        );
    }
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> GenericCamera<T> {
        match self {
            Self::ExtendedUnified(camera) => GenericCamera::ExtendedUnified(camera.cast()),
            Self::DoubleSphere(camera) => GenericCamera::DoubleSphere(camera.cast()),
            Self::KannalaBrandt4(camera) => GenericCamera::KannalaBrandt4(camera.cast()),
            Self::Unified(camera) => GenericCamera::Unified(camera.cast()),
            Self::Pinhole(camera) => GenericCamera::Pinhole(camera.cast()),
            Self::PinholeRadtan8(camera) => GenericCamera::PinholeRadtan8(camera.cast()),
        }
    }
    /// # Panics
    /// Panics unless the point dimension is three or four.
    pub fn project<const D: usize>(
        &self,
        point: &SVector<S, D>,
        pixel: &mut Vector2<S>,
        jacobian: Option<&mut SMatrix<S, 2, D>>,
    ) -> bool {
        visit!(self, camera, camera.project(point, pixel, jacobian, None))
    }
    /// # Panics
    /// Panics unless the point dimension is three or four.
    pub fn unproject<const D: usize>(
        &self,
        pixel: &Vector2<S>,
        point: &mut SVector<S, D>,
        jacobian: Option<&mut SMatrix<S, D, 2>>,
    ) -> bool {
        visit!(self, camera, camera.unproject(pixel, point, jacobian, None))
    }
    /// Batch projection without a frame transform.
    pub fn project_points(
        &self,
        points: &[Vector4<S>],
        pixels: &mut Vec<Vector2<S>>,
        valid: &mut Vec<bool>,
    ) {
        pixels.resize(points.len(), Vector2::zeros());
        valid.resize(points.len(), false);
        visit!(self, camera, {
            for (i, point) in points.iter().enumerate() {
                valid[i] = camera.project(point, &mut pixels[i], None, None);
            }
        });
    }
    /// Transform world points into camera coordinates and project them.
    /// Three-dimensional inputs receive homogeneous coordinate one.
    /// # Panics
    /// Panics unless the point dimension is three or four.
    pub fn project_transformed<const D: usize>(
        &self,
        points: &[SVector<S, D>],
        camera_from_world: &Matrix4<S>,
        pixels: &mut Vec<Vector2<S>>,
        valid: &mut Vec<bool>,
    ) {
        assert!(D == 3 || D == 4);
        pixels.resize(points.len(), Vector2::zeros());
        valid.resize(points.len(), false);
        visit!(self, camera, {
            for (i, point) in points.iter().enumerate() {
                let homogeneous = Vector4::new(
                    point[0],
                    point[1],
                    point[2],
                    if D == 4 { point[3] } else { S::one() },
                );
                valid[i] = camera.project(
                    &(camera_from_world * homogeneous),
                    &mut pixels[i],
                    None,
                    None,
                );
            }
        });
    }
    /// Batch projection with polar and azimuthal angles, using upstream atan2(x,y).
    pub fn project_with_angles(
        &self,
        points: &[Vector4<S>],
        camera_from_world: &Matrix4<S>,
        pixels: &mut Vec<Vector2<S>>,
        valid: &mut Vec<bool>,
        angles: &mut Vec<Vector2<S>>,
    ) {
        pixels.resize(points.len(), Vector2::zeros());
        valid.resize(points.len(), false);
        angles.resize(points.len(), Vector2::zeros());
        visit!(self, camera, {
            for (i, point) in points.iter().enumerate() {
                let transformed = camera_from_world * point;
                valid[i] = camera.project(&transformed, &mut pixels[i], None, None);
                let radius =
                    (transformed[0] * transformed[0] + transformed[1] * transformed[1]).sqrt();
                angles[i] = Vector2::new(
                    radius.atan2(transformed[2]),
                    transformed[0].atan2(transformed[1]),
                );
            }
        });
    }
    pub fn unproject_points(
        &self,
        pixels: &[Vector2<S>],
        points: &mut Vec<Vector4<S>>,
        valid: &mut Vec<bool>,
    ) {
        points.resize(pixels.len(), Vector4::zeros());
        valid.resize(pixels.len(), false);
        visit!(self, camera, {
            for (i, pixel) in pixels.iter().enumerate() {
                valid[i] = camera.unproject(pixel, &mut points[i], None, None);
            }
        });
    }
}
