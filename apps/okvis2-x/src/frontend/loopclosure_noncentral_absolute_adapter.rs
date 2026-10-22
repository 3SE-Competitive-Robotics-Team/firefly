/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Noncentral absolute-pose correspondences for explicit loop-closure matches.

use crate::{
    cameras::n_camera_system::NCameraSystem,
    frames::{frame_typedefs::KeypointIdentifier, multi_frame::MultiFrame},
    frontend::{
        frame_noncentral_absolute_adapter::{AbsoluteAdapter, AbsoluteCorrespondences},
        frame_relative_adapter::focal_length_u,
    },
};
use nalgebra::{Matrix3, Vector3, Vector4};
use std::collections::BTreeMap;

pub type Points = BTreeMap<u64, Vector4<f64>>;

pub struct LoopclosureNoncentralAbsoluteAdapter {
    data: AbsoluteCorrespondences,
    translation: Vector3<f64>,
    rotation: Matrix3<f64>,
}
impl LoopclosureNoncentralAbsoluteAdapter {
    #[must_use]
    /// # Panics
    /// Panics if the required assertion fails: `distortion, camera_system.distortion_type(camera),
    /// "mixed frame types are not supported"`.
    pub fn new(
        points: &Points,
        matches: &BTreeMap<KeypointIdentifier, u64>,
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
                let kid = KeypointIdentifier::new(frame.id(), camera, keypoint);
                let Some(&id) = matches.get(&kid) else {
                    continue;
                };
                if id == 0 {
                    continue;
                }
                let hp = points
                    .get(&id)
                    .expect("matched landmark missing from points");
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
impl AbsoluteAdapter for LoopclosureNoncentralAbsoluteAdapter {
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
