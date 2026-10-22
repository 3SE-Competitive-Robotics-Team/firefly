/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Synchronized collection of camera frames and their calibration.

use crate::{
    cameras::{camera_base::CameraBase, n_camera_system::NCameraSystem},
    frames::frame::{DescriptorExtractor, FeatureDetector, Frame, Image, KeyPoint, Network},
    kinematics::transformation::Transformation,
};
use firefly_error::Result;
use jiff::Timestamp;
use nalgebra::{Vector2, Vector3, Vector4};
use purecv::core::Matrix;
use std::sync::Arc;

#[derive(Clone)]
pub struct MultiFrame {
    timestamp: Timestamp,
    id: u64,
    frames: Vec<Frame>,
    camera_system: NCameraSystem,
}
impl Default for MultiFrame {
    fn default() -> Self {
        Self {
            timestamp: Timestamp::UNIX_EPOCH,
            id: 0,
            frames: Vec::new(),
            camera_system: NCameraSystem::default(),
        }
    }
}
impl MultiFrame {
    #[must_use]
    pub fn new(camera_system: &NCameraSystem, timestamp: Timestamp, id: u64) -> Self {
        let mut frame = Self {
            timestamp,
            id,
            ..Self::default()
        };
        frame.reset_camera_system_and_frames(camera_system);
        frame
    }
    pub fn reset_camera_system_and_frames(&mut self, camera_system: &NCameraSystem) {
        self.camera_system = camera_system.clone();
        self.frames = (0..camera_system.num_cameras())
            .map(|c| {
                let mut frame = Frame::default();
                frame.set_geometry(camera_system.camera_geometry(c).clone());
                frame
            })
            .collect();
    }
    #[must_use]
    pub fn camera_system(&self) -> &NCameraSystem {
        &self.camera_system
    }
    pub fn set_timestamp(&mut self, timestamp: Timestamp) {
        self.timestamp = timestamp;
    }
    #[must_use]
    pub fn timestamp(&self) -> Timestamp {
        self.timestamp
    }
    pub fn set_id(&mut self, id: u64) {
        self.id = id;
    }
    #[must_use]
    pub fn id(&self) -> u64 {
        self.id
    }
    #[must_use]
    pub fn num_frames(&self) -> usize {
        self.frames.len()
    }
    #[must_use]
    pub fn t_sc(&self, camera: usize) -> &Arc<Transformation> {
        self.camera_system.t_sc(camera)
    }
    pub fn set_extrinsics(&mut self, camera: usize, t_sc: Transformation) {
        self.camera_system.set_extrinsics(camera, t_sc);
    }
    #[must_use]
    pub fn frame(&self, camera: usize) -> &Frame {
        &self.frames[camera]
    }
    pub fn frame_mut(&mut self, camera: usize) -> &mut Frame {
        &mut self.frames[camera]
    }
    pub fn total_keypoints(&self) -> usize {
        self.frames.iter().map(Frame::num_keypoints).sum()
    }
    #[must_use]
    pub fn overlap(&self, seen_by: usize, camera: usize) -> &Arc<Matrix<u8>> {
        self.camera_system.overlap(seen_by, camera)
    }
    #[must_use]
    pub fn has_overlap(&self, seen_by: usize, camera: usize) -> bool {
        self.camera_system.has_overlap(seen_by, camera)
    }
    pub fn set_image(&mut self, camera: usize, image: Image) {
        self.frames[camera].set_image(image);
    }
    pub fn set_depth_image(&mut self, camera: usize, image: Image) {
        self.frames[camera].set_depth_image(image);
    }
    pub fn set_geometry(&mut self, camera: usize, geometry: Arc<dyn CameraBase>) {
        self.frames[camera].set_geometry(geometry);
    }
    pub fn set_detector(&mut self, camera: usize, detector: Arc<dyn FeatureDetector>) {
        self.frames[camera].set_detector(detector);
    }
    pub fn set_extractor(&mut self, camera: usize, extractor: Arc<dyn DescriptorExtractor>) {
        self.frames[camera].set_extractor(extractor);
    }
    pub fn set_network(&mut self, camera: usize, network: Arc<dyn Network>) {
        self.frames[camera].set_network(network);
    }
    /// # Errors
    /// Returns an error if required image/camera/feature resources are absent, indices or
    /// image/descriptor shapes are invalid, or the detector, extractor or model fails.
    pub fn detect(&mut self, camera: usize) -> Result<usize> {
        self.frames[camera].detect()
    }
    /// # Errors
    /// Returns an error if required image/camera/feature resources are absent, indices or
    /// image/descriptor shapes are invalid, or the detector, extractor or model fails.
    pub fn describe(&mut self, camera: usize) -> Result<usize> {
        self.frames[camera].describe()
    }
    pub fn compute_back_projections(&mut self, camera: usize) -> usize {
        self.frames[camera].compute_back_projections()
    }
    /// # Errors
    /// Returns an error if required image/camera/feature resources are absent, indices or
    /// image/descriptor shapes are invalid, or the detector, extractor or model fails.
    pub fn compute_classifications(
        &mut self,
        camera: usize,
        size_u: usize,
        size_v: usize,
    ) -> Result<usize> {
        self.frames[camera].compute_classifications(size_u, size_v, 1)
    }
    pub fn set_landmark_id(&mut self, camera: usize, index: usize, id: u64) -> bool {
        self.frames[camera].set_landmark_id(index, id)
    }
    pub fn set_landmark(
        &mut self,
        camera: usize,
        index: usize,
        landmark: Vector4<f64>,
        initialized: bool,
    ) -> bool {
        self.frames[camera].set_landmark(index, landmark, initialized)
    }
    pub fn reset_keypoints(&mut self, camera: usize, keypoints: &[KeyPoint]) -> bool {
        self.frames[camera].reset_keypoints(keypoints)
    }
    pub fn reset_descriptors(&mut self, camera: usize, descriptors: Arc<Matrix<u8>>) -> bool {
        self.frames[camera].reset_descriptors(descriptors)
    }
    #[must_use]
    pub fn image(&self, camera: usize) -> Option<&Image> {
        self.frames[camera].image()
    }
    #[must_use]
    pub fn depth_image(&self, camera: usize) -> Option<&Image> {
        self.frames[camera].depth_image()
    }
    #[must_use]
    pub fn geometry(&self, camera: usize) -> &Arc<dyn CameraBase> {
        self.frames[camera].geometry()
    }
    #[must_use]
    pub fn geometry_as<G: CameraBase>(&self, camera: usize) -> &G {
        self.frames[camera].geometry_as::<G>()
    }

    pub fn get_back_projection(&self, camera: usize, index: usize, ray: &mut Vector3<f64>) -> bool {
        self.frames[camera].get_back_projection(index, ray)
    }
    pub fn get_classification(
        &self,
        camera: usize,
        index: usize,
        classification: &mut [f32; 19],
    ) -> bool {
        self.frames[camera].get_classification(index, classification)
    }
    #[must_use]
    pub fn is_classified(&self, camera: usize) -> bool {
        self.frames[camera].is_classified()
    }
    pub fn get_cv_keypoint(&self, camera: usize, index: usize, keypoint: &mut KeyPoint) -> bool {
        self.frames[camera].get_cv_keypoint(index, keypoint)
    }
    pub fn get_keypoint(&self, camera: usize, index: usize, keypoint: &mut Vector2<f64>) -> bool {
        self.frames[camera].get_keypoint(index, keypoint)
    }
    pub fn get_keypoint_size(&self, camera: usize, index: usize, size: &mut f64) -> bool {
        self.frames[camera].get_keypoint_size(index, size)
    }
    #[must_use]
    pub fn keypoint_descriptor(&self, camera: usize, index: usize) -> &[u8] {
        self.frames[camera].keypoint_descriptor(index)
    }
    #[must_use]
    pub fn landmark_id(&self, camera: usize, index: usize) -> u64 {
        self.frames[camera].landmark_id(index)
    }
    pub fn get_landmark(
        &self,
        camera: usize,
        index: usize,
        landmark: &mut Vector4<f64>,
        initialized: &mut bool,
    ) -> bool {
        self.frames[camera].get_landmark(index, landmark, initialized)
    }
    #[must_use]
    pub fn num_keypoints(&self, camera: usize) -> usize {
        self.frames[camera].num_keypoints()
    }
}
