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

//! Keypoint reprojection factor corresponding to `implementation/ReprojectionError.hpp`.

use crate::{
    cameras::camera_base::CameraBase,
    ceres::{pose_error::square_root_information, pose_local_parameterization::PoseManifold},
    kinematics::{
        operators::cross_mx,
        transformation::{normalized_quaternion, raw_rotation_matrix},
    },
};
use nalgebra::{Matrix2, Matrix3, Matrix4, Quaternion, SMatrix, SVector, Vector2, Vector4};
use std::sync::Arc;

#[derive(Debug, Default)]
pub struct ReprojectionJacobians<'a> {
    pub pose: Option<&'a mut SMatrix<f64, 2, 7>>,
    pub point: Option<&'a mut SMatrix<f64, 2, 4>>,
    pub extrinsics: Option<&'a mut SMatrix<f64, 2, 7>>,
}
#[derive(Debug, Default)]
pub struct MinimalReprojectionJacobians<'a> {
    pub pose: Option<&'a mut SMatrix<f64, 2, 6>>,
    pub point: Option<&'a mut SMatrix<f64, 2, 3>>,
    pub extrinsics: Option<&'a mut SMatrix<f64, 2, 6>>,
}

#[derive(Debug)]
pub struct ReprojectionError<G: ?Sized> {
    camera_geometry: Arc<G>,
    camera_id: Option<u64>,
    measurement: Option<Vector2<f64>>,
    information: Option<Matrix2<f64>>,
    square_root_information: Option<Matrix2<f64>>,
    covariance: Option<Matrix2<f64>>,
}
impl<G: CameraBase + Default> Default for ReprojectionError<G> {
    fn default() -> Self {
        Self {
            camera_geometry: Arc::new(G::default()),
            camera_id: None,
            measurement: None,
            information: None,
            square_root_information: None,
            covariance: None,
        }
    }
}
impl<G: CameraBase + ?Sized> ReprojectionError<G> {
    pub const NUM_RESIDUALS: usize = 2;
    pub fn new(
        camera_geometry: Arc<G>,
        camera_id: u64,
        measurement: Vector2<f64>,
        information: Matrix2<f64>,
    ) -> Self {
        let mut factor = Self {
            camera_geometry,
            camera_id: Some(camera_id),
            measurement: Some(measurement),
            information: None,
            square_root_information: None,
            covariance: None,
        };
        factor.set_information(information);
        factor
    }
    #[must_use]
    pub fn camera_id(&self) -> Option<u64> {
        self.camera_id
    }
    pub fn set_camera_id(&mut self, id: u64) {
        self.camera_id = Some(id);
    }
    pub fn set_measurement(&mut self, measurement: Vector2<f64>) {
        self.measurement = Some(measurement);
    }
    pub fn set_camera_geometry(&mut self, camera_geometry: Arc<G>) {
        self.camera_geometry = camera_geometry;
    }
    /// Uses Eigen-compatible partial LLT weighting; singular covariance is absent.
    pub fn set_information(&mut self, information: Matrix2<f64>) {
        self.information = Some(information);
        self.covariance = information.try_inverse();
        self.square_root_information = Some(square_root_information(information));
    }
    #[must_use]
    pub fn measurement(&self) -> Option<&Vector2<f64>> {
        self.measurement.as_ref()
    }
    #[must_use]
    pub fn information(&self) -> Option<&Matrix2<f64>> {
        self.information.as_ref()
    }
    #[must_use]
    pub fn covariance(&self) -> Option<&Matrix2<f64>> {
        self.covariance.as_ref()
    }
    pub fn clone_factor(&self) -> Arc<Self> {
        Arc::new(Self {
            camera_geometry: Arc::clone(&self.camera_geometry),
            camera_id: self.camera_id,
            measurement: self.measurement,
            information: self.information,
            square_root_information: self.information.map(square_root_information),
            covariance: self.information.and_then(nalgebra::Matrix::try_inverse),
        })
    }
    pub fn evaluate(
        &self,
        parameters: (&SVector<f64, 7>, &Vector4<f64>, &SVector<f64, 7>),
        residuals: &mut Vector2<f64>,
        jacobians: Option<ReprojectionJacobians<'_>>,
    ) -> bool {
        self.evaluate_with_minimal_jacobians(parameters, residuals, jacobians, None)
    }
    /// Poses are `[tx,ty,tz,qx,qy,qz,qw]`; the point is homogeneous in world coordinates.
    /// Projection status does not gate the residual or Jacobians in this factor.
    /// Minimal outputs are written only for requested ambient Jacobians.
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn evaluate_with_minimal_jacobians(
        &self,
        parameters: (&SVector<f64, 7>, &Vector4<f64>, &SVector<f64, 7>),
        residuals: &mut Vector2<f64>,
        jacobians: Option<ReprojectionJacobians<'_>>,
        jacobians_minimal: Option<MinimalReprojectionJacobians<'_>>,
    ) -> bool {
        let (Some(measurement), Some(square_root_information)) =
            (self.measurement, self.square_root_information)
        else {
            return false;
        };
        let (pose, hp_w, extrinsics) = parameters;
        let t_ws_w = pose.fixed_rows::<3>(0).into_owned();
        let t_sc_s = extrinsics.fixed_rows::<3>(0).into_owned();
        let c_sw = raw_rotation_matrix(&normalized_quaternion(quaternion(pose))).transpose();
        let c_cs = raw_rotation_matrix(&normalized_quaternion(quaternion(extrinsics))).transpose();
        let mut t_cs = Matrix4::identity();
        t_cs.fixed_view_mut::<3, 3>(0, 0).copy_from(&c_cs);
        t_cs.fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&(-c_cs * t_sc_s));
        let mut t_sw = Matrix4::identity();
        t_sw.fixed_view_mut::<3, 3>(0, 0).copy_from(&c_sw);
        t_sw.fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&(-c_sw * t_ws_w));
        let hp_s = t_sw * hp_w;
        let hp_c = t_cs * hp_s;
        let mut keypoint = Vector2::repeat(f64::NAN);
        let mut jh = SMatrix::<f64, 2, 4>::repeat(f64::NAN);
        let jh_weighted = if jacobians.is_some() {
            self.camera_geometry.project_homogeneous_with_jacobians(
                &hp_c,
                &mut keypoint,
                &mut jh,
                None,
            );
            Some(square_root_information * jh)
        } else {
            self.camera_geometry
                .project_homogeneous(&hp_c, &mut keypoint);
            None
        };
        *residuals = square_root_information * (measurement - keypoint);
        if let Some(jacobians) = jacobians {
            let jh_weighted = jh_weighted.expect("projection Jacobian was requested");
            let ReprojectionJacobians {
                pose: j0,
                point: j1,
                extrinsics: j2,
            } = jacobians;
            let MinimalReprojectionJacobians {
                pose: j0_minimal,
                point: j1_minimal,
                extrinsics: j2_minimal,
            } = jacobians_minimal.unwrap_or_default();
            if let Some(j0) = j0 {
                let p = hp_w.fixed_rows::<3>(0) - t_ws_w * hp_w[3];
                let mut j = SMatrix::<f64, 4, 6>::zeros();
                j.fixed_view_mut::<3, 3>(0, 0).copy_from(&(c_sw * hp_w[3]));
                j.fixed_view_mut::<3, 3>(0, 3)
                    .copy_from(&(-c_sw * cross_mx(&p)));
                let minimal = jh_weighted * t_cs * j;
                *j0 = minimal * PoseManifold::minus_jacobian(pose);
                if let Some(j0_minimal) = j0_minimal {
                    *j0_minimal = minimal;
                }
            }
            if let Some(j1) = j1 {
                let camera_from_world = t_cs * t_sw;
                *j1 = -jh_weighted * camera_from_world;
                if let Some(j1_minimal) = j1_minimal {
                    let mut s = SMatrix::<f64, 4, 3>::zeros();
                    s.fixed_view_mut::<3, 3>(0, 0)
                        .copy_from(&Matrix3::identity());
                    *j1_minimal = *j1 * s;
                }
            }
            if let Some(j2) = j2 {
                let p = hp_s.fixed_rows::<3>(0) - t_sc_s * hp_s[3];
                let mut j = SMatrix::<f64, 4, 6>::zeros();
                j.fixed_view_mut::<3, 3>(0, 0).copy_from(&(c_cs * hp_s[3]));
                j.fixed_view_mut::<3, 3>(0, 3)
                    .copy_from(&(-c_cs * cross_mx(&p)));
                let minimal = jh_weighted * j;
                *j2 = minimal * PoseManifold::minus_jacobian(extrinsics);
                if let Some(j2_minimal) = j2_minimal {
                    *j2_minimal = minimal;
                }
            }
        }
        true
    }
    #[must_use]
    pub const fn residual_dim(&self) -> usize {
        2
    }
    #[must_use]
    pub const fn parameter_blocks(&self) -> usize {
        3
    }
    #[must_use]
    pub fn parameter_block_dim(&self, block: usize) -> usize {
        [7, 4, 7][block]
    }
    #[must_use]
    pub const fn type_info(&self) -> &'static str {
        "ReprojectionError"
    }
}
fn quaternion(parameters: &SVector<f64, 7>) -> Quaternion<f64> {
    Quaternion::new(parameters[6], parameters[3], parameters[4], parameters[5])
}

impl<G: CameraBase + ?Sized> ReprojectionError<G> {
    /// Copies the supported calibration model; no graph or camera Rc crosses threads.
    #[must_use]
    pub fn detached_snapshot(
        &self,
    ) -> Option<Box<dyn crate::ceres::reprojection_error_base::ReprojectionError2dBase + Send>>
    {
        use crate::cameras::{
            equidistant_distortion::EquidistantDistortion, eucm_camera::EucmCamera,
            no_distortion::NoDistortion, pinhole_camera::PinholeCamera,
            radial_tangential_distortion::RadialTangentialDistortion,
            radial_tangential_distortion8::RadialTangentialDistortion8,
        };
        let mut k = nalgebra::DVector::zeros(self.camera_geometry.num_intrinsics());
        self.camera_geometry.get_intrinsics(&mut k);
        let state = self.camera_geometry.state().clone();
        let id = self.camera_id?;
        let measurement = self.measurement?;
        let information = self.information?;
        macro_rules! make {
            ($camera:expr) => {{
                let mut camera = $camera;
                *camera.state_mut() = state;
                return Some(Box::new(Self::copy_with_camera(
                    self,
                    camera,
                    id,
                    measurement,
                    information,
                )));
            }};
        }
        let (w, h) = (state.image_width, state.image_height);
        if self.camera_geometry.type_name() == "EUCMCamera" {
            if k.len() != 6 {
                return None;
            }
            make!(EucmCamera::new(
                (w, h),
                (k[0], k[1], k[2], k[3], k[4], k[5]),
                state.id
            ));
        }
        if !self
            .camera_geometry
            .type_name()
            .starts_with("PinholeCamera<")
            || k.len() < 4
        {
            return None;
        }
        match self.camera_geometry.distortion_type() {
            "NoDistortion" if k.len() == 4 => make!(PinholeCamera::new(
                (w, h),
                (k[0], k[1], k[2], k[3]),
                NoDistortion,
                state.id
            )),
            "EquidistantDistortion" if k.len() == 8 => make!(PinholeCamera::new(
                (w, h),
                (k[0], k[1], k[2], k[3]),
                EquidistantDistortion::new(k[4], k[5], k[6], k[7]),
                state.id
            )),
            "RadialTangentialDistortion" if k.len() == 8 => make!(PinholeCamera::new(
                (w, h),
                (k[0], k[1], k[2], k[3]),
                RadialTangentialDistortion::new(k[4], k[5], k[6], k[7]),
                state.id
            )),
            "RadialTangentialDistortion8" if k.len() == 12 => make!(PinholeCamera::new(
                (w, h),
                (k[0], k[1], k[2], k[3]),
                RadialTangentialDistortion8::new((
                    k[4], k[5], k[6], k[7], k[8], k[9], k[10], k[11]
                )),
                state.id
            )),
            _ => None,
        }
    }
    fn copy_with_camera<C: CameraBase>(
        source: &Self,
        camera: C,
        id: u64,
        measurement: Vector2<f64>,
        information: Matrix2<f64>,
    ) -> ReprojectionError<C> {
        let mut out = ReprojectionError::new(Arc::new(camera), id, measurement, information);
        out.square_root_information = source.square_root_information;
        out.covariance = source.covariance;
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cameras::{no_distortion::NoDistortion, pinhole_camera::PinholeCamera};

    #[test]
    fn uninitialized_and_degenerate_reprojection_weights_are_explicit() {
        let camera = Arc::new(PinholeCamera::new(
            (20, 20),
            (1., 1., 0., 0.),
            NoDistortion,
            0,
        ));
        let mut factor = ReprojectionError {
            camera_geometry: camera,
            camera_id: None,
            measurement: None,
            information: None,
            square_root_information: None,
            covariance: None,
        };
        let mut pose = SVector::<f64, 7>::zeros();
        pose[6] = 1.;
        let point = Vector4::new(0., 0., 2., 1.);
        let mut residual = Vector2::repeat(12.);
        assert!(
            !factor
                .clone_factor()
                .evaluate((&pose, &point, &pose), &mut residual, None)
        );
        assert_eq!(residual, Vector2::repeat(12.));
        factor.set_measurement(Vector2::repeat(1.));
        factor.set_information(Matrix2::zeros());
        assert!(factor.covariance().is_none());
        assert!(factor.evaluate((&pose, &point, &pose), &mut residual, None));
        assert_eq!(residual, Vector2::zeros());
        factor.set_information(Matrix2::new(4., 2., 2., 1.));
        assert!(factor.covariance().is_none());
        assert!(
            factor
                .clone_factor()
                .evaluate((&pose, &point, &pose), &mut residual, None)
        );
        assert_eq!(residual, Vector2::new(3., 1.));
    }
}
