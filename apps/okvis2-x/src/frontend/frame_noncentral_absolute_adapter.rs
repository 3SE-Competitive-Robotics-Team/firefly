/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Noncentral absolute-pose correspondences from observed estimator landmarks.

use crate::{
    cameras::n_camera_system::NCameraSystem,
    frames::{frame_typedefs::LandmarkId, multi_frame::MultiFrame},
    frontend::frame_relative_adapter::{EstimatorView, focal_length_u, normalize},
};
use nalgebra::{Matrix3, Vector3};

/// Capabilities shared by the two supported absolute adapters.
pub trait AbsoluteAdapter {
    fn bearing_vector(&self, index: usize) -> Vector3<f64>;
    fn point(&self, index: usize) -> Vector3<f64>;
    fn cam_offset(&self, index: usize) -> Vector3<f64>;
    fn cam_rotation(&self, index: usize) -> Matrix3<f64>;
    fn number_correspondences(&self) -> usize;
    fn sigma_angle(&self, index: usize) -> f64;
    fn cam_index(&self, index: usize) -> usize;
    fn keypoint_index(&self, index: usize) -> usize;
    fn weight(&self, _index: usize) -> f64 {
        1.0
    }
    fn translation(&self) -> Vector3<f64>;
    fn rotation(&self) -> Matrix3<f64>;
    fn set_translation(&mut self, value: Vector3<f64>);
    fn set_rotation(&mut self, value: Matrix3<f64>);
}

#[derive(Default)]
pub(crate) struct AbsoluteCorrespondences {
    pub bearing_vectors: Vec<Vector3<f64>>,
    pub points: Vec<Vector3<f64>>,
    pub cam_indices: Vec<usize>,
    pub keypoint_indices: Vec<usize>,
    pub cam_offsets: Vec<Vector3<f64>>,
    pub cam_rotations: Vec<Matrix3<f64>>,
    pub sigma_angles: Vec<f64>,
}
impl AbsoluteCorrespondences {
    pub fn push(
        &mut self,
        frame: &MultiFrame,
        camera: usize,
        keypoint: usize,
        point: Vector3<f64>,
        fu: f64,
    ) {
        self.points.push(point);
        let mut size = 0.0;
        frame.get_keypoint_size(camera, keypoint, &mut size);
        let stddev = 0.8 * size / 12.0;
        let mut bearing = Vector3::repeat(f64::NAN);
        if !frame.get_back_projection(camera, keypoint, &mut bearing) {
            bearing = Vector3::new(1.0, 0.0, 0.0);
        }
        self.sigma_angles
            .push(2.0_f64.sqrt() * stddev * stddev / (fu * fu));
        normalize(&mut bearing);
        self.bearing_vectors.push(bearing);
        self.cam_indices.push(camera);
        self.keypoint_indices.push(keypoint);
    }
}
pub struct FrameNoncentralAbsoluteAdapter {
    data: AbsoluteCorrespondences,
    translation: Vector3<f64>,
    rotation: Matrix3<f64>,
}
impl FrameNoncentralAbsoluteAdapter {
    /// # Panics
    /// Panics if the required assertion fails: `distortion, camera_system.distortion_type(camera),
    /// "mixed frame types are not supported"`.
    pub fn new(
        estimator: &impl EstimatorView,
        camera_system: &NCameraSystem,
        frame: &MultiFrame,
    ) -> Self {
        let distortion = camera_system.distortion_type(0);
        for camera in 1..camera_system.num_cameras() {
            assert_eq!(
                distortion,
                camera_system.distortion_type(camera),
                "mixed frame types are not supported"
            );
        }
        let mut data = AbsoluteCorrespondences::default();
        for camera in 0..camera_system.num_cameras() {
            data.cam_offsets.push(frame.t_sc(camera).translation());
            data.cam_rotations
                .push(frame.t_sc(camera).rotation_matrix());
            let fu = focal_length_u(frame.geometry(camera).as_ref(), distortion);
            for keypoint in 0..frame.num_keypoints(camera) {
                let id = frame.landmark_id(camera, keypoint);
                if id == 0 || !estimator.is_landmark_added(LandmarkId::new(id)) {
                    continue;
                }
                let landmark = estimator.landmark(LandmarkId::new(id));
                if landmark.observations.len() < 2 {
                    continue;
                }
                let hp = landmark.point;
                if hp[3].abs() < 1.0e-8 {
                    continue;
                }
                data.push(
                    frame,
                    camera,
                    keypoint,
                    hp.fixed_rows::<3>(0).into_owned() / hp[3],
                    fu,
                );
            }
        }
        Self {
            data,
            translation: Vector3::zeros(),
            rotation: Matrix3::identity(),
        }
    }
}
impl AbsoluteAdapter for FrameNoncentralAbsoluteAdapter {
    fn bearing_vector(&self, index: usize) -> Vector3<f64> {
        self.data.bearing_vectors[index]
    }
    fn point(&self, index: usize) -> Vector3<f64> {
        self.data.points[index]
    }
    fn cam_offset(&self, index: usize) -> Vector3<f64> {
        self.data.cam_offsets[self.data.cam_indices[index]]
    }
    fn cam_rotation(&self, index: usize) -> Matrix3<f64> {
        self.data.cam_rotations[self.data.cam_indices[index]]
    }
    fn number_correspondences(&self) -> usize {
        self.data.points.len()
    }
    fn sigma_angle(&self, index: usize) -> f64 {
        self.data.sigma_angles[index]
    }
    fn cam_index(&self, index: usize) -> usize {
        self.data.cam_indices[index]
    }
    fn keypoint_index(&self, index: usize) -> usize {
        self.data.keypoint_indices[index]
    }
    fn translation(&self) -> Vector3<f64> {
        self.translation
    }
    fn rotation(&self) -> Matrix3<f64> {
        self.rotation
    }
    fn set_translation(&mut self, value: Vector3<f64>) {
        self.translation = value;
    }
    fn set_rotation(&mut self, value: Matrix3<f64>) {
        self.rotation = value;
    }
}
