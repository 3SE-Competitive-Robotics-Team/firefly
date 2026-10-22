/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Multi-camera geometry and rotation-only field-of-view overlaps.

use crate::{
    cameras::camera_base::{CameraBase, ProjectionStatus},
    kinematics::transformation::Transformation,
};
use nalgebra::{Quaternion, Vector2, Vector3};
use purecv::core::Matrix;
use std::{collections::BTreeMap, sync::Arc};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum DistortionType {
    Equidistant = 0,
    RadialTangential = 1,
    NoDistortion = 2,
    RadialTangential8 = 3,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DepthCreation {
    pub create_virtual: bool,
    pub create_depth: bool,
}
#[derive(Clone, Debug)]
pub struct DepthType {
    pub is_depth_camera: bool,
    pub baseline: Vector3<f64>,
    pub creation: DepthCreation,

    pub sigma_pixels: f64,
    pub sigma_depth: f64,
    pub need_rectify: bool,
}
impl Default for DepthType {
    fn default() -> Self {
        Self {
            is_depth_camera: false,
            baseline: Vector3::zeros(),
            creation: crate::cameras::n_camera_system::DepthCreation {
                create_virtual: false,
                create_depth: false,
            },

            sigma_pixels: 1.0,
            sigma_depth: 0.01,
            need_rectify: false,
        }
    }
}
#[derive(Clone, Debug)]
pub struct CameraType {
    pub is_colour: bool,
    pub is_used: bool,
    pub is_used_mapping: bool,
    pub depth_type: DepthType,
}
impl Default for CameraType {
    fn default() -> Self {
        Self {
            is_colour: true,
            is_used: true,
            is_used_mapping: false,
            depth_type: DepthType::default(),
        }
    }
}

#[derive(Clone, Default)]
pub struct NCameraSystem {
    t_sc: Vec<Arc<Transformation>>,
    camera_geometries: Vec<Arc<dyn CameraBase>>,
    distortion_types: Vec<DistortionType>,
    overlap_mats: Vec<Vec<Arc<Matrix<u8>>>>,
    overlaps: Vec<Vec<bool>>,
    camera_types: Vec<CameraType>,
    virtual_t_sc: Vec<Arc<Transformation>>,
    virtual_camera_geometries: Vec<Arc<dyn CameraBase>>,
    virtual_distortion_types: Vec<DistortionType>,
    rectify_t_sc: BTreeMap<usize, Arc<Transformation>>,
    rectify_camera_geometries: BTreeMap<usize, Arc<dyn CameraBase>>,
    num_used_cameras: usize,
}
impl NCameraSystem {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
    pub fn add_camera(
        &mut self,
        t_sc: &Arc<Transformation>,
        geometry: Arc<dyn CameraBase>,
        distortion: DistortionType,
        compute_overlaps: bool,
        camera_type: CameraType,
    ) {
        self.t_sc.push(t_sc.clone());
        self.camera_geometries.push(geometry.clone());
        self.distortion_types.push(distortion);
        self.num_used_cameras += usize::from(camera_type.is_used);
        if camera_type.depth_type.is_depth_camera && camera_type.depth_type.creation.create_virtual
        {
            let t_cc_virtual =
                Transformation::new(camera_type.depth_type.baseline, Quaternion::identity());
            self.virtual_t_sc
                .push(Arc::new(t_sc.compose(&t_cc_virtual)));
            self.virtual_camera_geometries.push(geometry);
            self.virtual_distortion_types.push(distortion);
        }
        self.camera_types.push(camera_type);
        if compute_overlaps {
            self.compute_overlaps();
        }
    }
    pub fn add_rectify_camera(
        &mut self,
        index: usize,
        t_sc: Arc<Transformation>,
        geometry: Arc<dyn CameraBase>,
    ) {
        self.rectify_t_sc.insert(index, t_sc);
        self.rectify_camera_geometries.insert(index, geometry);
    }
    #[must_use]
    pub fn num_cameras(&self) -> usize {
        self.camera_geometries.len()
    }
    #[must_use]
    pub fn num_used_cameras(&self) -> usize {
        self.num_used_cameras
    }
    #[must_use]
    pub fn t_sc(&self, index: usize) -> &Arc<Transformation> {
        &self.t_sc[index]
    }
    #[must_use]
    /// # Panics
    /// Panics if a required resource/invariant is absent: `rectified camera index out of range`.
    pub fn rectify_t_sc(&self, index: usize) -> &Arc<Transformation> {
        self.rectify_t_sc
            .get(&index)
            .expect("rectified camera index out of range")
    }
    pub fn set_extrinsics(&mut self, index: usize, t_sc: Transformation) {
        self.t_sc[index] = Arc::new(t_sc);
    }
    #[must_use]
    pub fn camera_geometry(&self, index: usize) -> &Arc<dyn CameraBase> {
        &self.camera_geometries[index]
    }
    #[must_use]
    /// # Panics
    /// Panics if a required resource/invariant is absent: `rectified camera index out of range`.
    pub fn rectify_camera_geometry(&self, index: usize) -> &Arc<dyn CameraBase> {
        self.rectify_camera_geometries
            .get(&index)
            .expect("rectified camera index out of range")
    }
    #[must_use]
    pub fn distortion_type(&self, index: usize) -> DistortionType {
        self.distortion_types[index]
    }
    #[must_use]
    pub fn camera_type(&self, index: usize) -> &CameraType {
        &self.camera_types[index]
    }
    #[must_use]
    pub fn is_depth_camera(&self, index: usize) -> bool {
        self.camera_types[index].depth_type.is_depth_camera
    }
    #[must_use]
    pub fn is_camera_configured(&self, index: usize) -> bool {
        index < self.camera_types.len()
    }
    #[must_use]
    /// # Panics
    /// Panics if the required assertion fails: `self.t_sc.len(), n, "extrinsics/camera model count
    /// mismatch"`.
    pub fn overlap_computation_valid(&self) -> bool {
        let n = self.num_cameras();
        assert_eq!(self.t_sc.len(), n, "extrinsics/camera model count mismatch");
        self.overlaps.len() == n
            && self.overlap_mats.len() == n
            && self.overlaps.iter().all(|r| r.len() == n)
            && self.overlap_mats.iter().all(|r| r.len() == n)
    }
    #[must_use]
    /// # Panics
    /// Panics if the required assertion fails: `seen_by < self.num_cameras() && index <
    /// self.num_cameras(); self.overlap_computation_valid(), "overlap computation invalid"`.
    pub fn overlap(&self, seen_by: usize, index: usize) -> &Arc<Matrix<u8>> {
        assert!(seen_by < self.num_cameras() && index < self.num_cameras());
        assert!(
            self.overlap_computation_valid(),
            "overlap computation invalid"
        );
        &self.overlap_mats[seen_by][index]
    }
    #[must_use]
    /// # Panics
    /// Panics if the required assertion fails: `seen_by < self.num_cameras() && index <
    /// self.num_cameras(); self.overlap_computation_valid(), "overlap computation invalid"`.
    pub fn has_overlap(&self, seen_by: usize, index: usize) -> bool {
        assert!(seen_by < self.num_cameras() && index < self.num_cameras());
        assert!(
            self.overlap_computation_valid(),
            "overlap computation invalid"
        );
        self.overlaps[seen_by][index]
    }
    /// # Panics
    /// Panics if the required assertion fails: `self.t_sc.len(), n, "extrinsics/camera model count
    /// mismatch"`.
    pub fn compute_overlaps(&mut self) {
        let n = self.num_cameras();
        assert_eq!(self.t_sc.len(), n, "extrinsics/camera model count mismatch");
        self.overlap_mats.resize_with(n, Vec::new);
        self.overlaps.resize_with(n, Vec::new);
        for seen_by in 0..n {
            self.overlap_mats[seen_by].resize_with(n, || Arc::new(Matrix::new(0, 0, 1)));
            // C++ resize preserves existing visibility flags on recomputation.
            self.overlaps[seen_by].resize(n, false);
            for index in 0..n {
                let camera = &self.camera_geometries[index];
                let width = camera.image_width() as usize;
                let height = camera.image_height() as usize;
                let mut mask = Matrix::new(height, width, 1);
                if index == seen_by {
                    mask.data.fill(1);
                    self.overlaps[seen_by][index] = true;
                } else {
                    let other = &self.camera_geometries[seen_by];
                    let t_other_c = self.t_sc[seen_by].inverse().compose(&self.t_sc[index]);
                    for u in 0..width {
                        for v in 0..height {
                            let mut ray = Vector3::repeat(f64::NAN);
                            camera.back_project(&Vector2::new(u as f64, v as f64), &mut ray);
                            let ray_other = t_other_c.rotate_vector(&ray);
                            let mut image = Vector2::repeat(f64::NAN);
                            if other.project(&ray_other, &mut image) == ProjectionStatus::Successful
                            {
                                let mut verification = Vector3::repeat(f64::NAN);
                                other.back_project(&image, &mut verification);
                                let mut normalized_ray = ray_other;
                                let mut normalized_verification = verification;
                                for vector in [&mut normalized_ray, &mut normalized_verification] {
                                    let squared = vector.norm_squared();
                                    if squared > 0.0 {
                                        *vector /= squared.sqrt();
                                    }
                                }
                                if (normalized_ray.dot(&normalized_verification) - 1.0).abs()
                                    < 1.0e-10
                                {
                                    mask.data[v * width + u] = 1;
                                    self.overlaps[seen_by][index] = true;
                                }
                            }
                        }
                    }
                }
                self.overlap_mats[seen_by][index] = Arc::new(mask);
            }
        }
    }
}
