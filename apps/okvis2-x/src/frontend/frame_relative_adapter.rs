/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Central relative-pose correspondences from two estimator frames.

use crate::{
    cameras::{
        camera_base::CameraBase,
        n_camera_system::{DistortionType, NCameraSystem},
    },
    frames::{
        frame_typedefs::{LandmarkId, MapPoint2, Match, StateId},
        multi_frame::MultiFrame,
    },
};
use nalgebra::{DVector, Matrix3, Vector3};
use std::collections::BTreeMap;

/// Read capabilities used from `ViSlamBackend`. Missing frame/landmark access has
/// the estimator's checked-access semantics; no landmark state is synthesized.
pub trait EstimatorView {
    fn multi_frame(&self, id: StateId) -> &MultiFrame;
    fn is_landmark_added(&self, id: LandmarkId) -> bool;
    fn landmark(&self, id: LandmarkId) -> &MapPoint2;
}

pub struct FrameRelativeAdapter {
    bearing_vectors1: Vec<Option<Vector3<f64>>>,
    bearing_vectors2: Vec<Option<Vector3<f64>>>,
    sigma_angles1: Vec<Option<f64>>,
    sigma_angles2: Vec<Option<f64>>,
    matches: Vec<Match>,
    t12: Vector3<f64>,
    r12: Matrix3<f64>,
}
impl FrameRelativeAdapter {
    pub fn new(
        estimator: &impl EstimatorView,
        camera_system: &NCameraSystem,
        frame_a_id: u64,
        camera_a: usize,
        other_frame_id: u64,
        camera_b: usize,
    ) -> Self {
        let a = estimator.multi_frame(StateId::new(frame_a_id));
        let b = estimator.multi_frame(StateId::new(other_frame_id));
        let fu1 = focal_length_u(
            a.geometry(camera_a).as_ref(),
            camera_system.distortion_type(camera_a),
        );
        // The source selects frame A's second camera geometry for fu2.
        let fu2 = focal_length_u(
            a.geometry(camera_b).as_ref(),
            camera_system.distortion_type(camera_b),
        );
        let na = a.num_keypoints(camera_a);
        let nb = b.num_keypoints(camera_b);
        let mut result = Self {
            bearing_vectors1: vec![None; na],
            bearing_vectors2: vec![None; nb],
            sigma_angles1: vec![None; na],
            sigma_angles2: vec![None; nb],
            matches: Vec::with_capacity(na.min(nb)),
            t12: Vector3::zeros(),
            r12: Matrix3::identity(),
        };
        let mut ids = BTreeMap::new();
        for k in 0..nb {
            let id = b.landmark_id(camera_b, k);
            if id == 0 || !estimator.is_landmark_added(LandmarkId::new(id)) {
                continue;
            }
            ids.entry(id).or_insert(k);
        }
        for k in 0..na {
            let id = a.landmark_id(camera_a, k);
            if id == 0 {
                continue;
            }
            if let Some(&index_b) = ids.get(&id) {
                result.matches.push(Match {
                    idx_a: k,
                    idx_b: index_b,
                    distance: 0.0,
                });
            }
        }
        for m in &result.matches {
            let i = m.idx_a;
            let j = m.idx_b;
            let mut size = 0.0;
            a.get_keypoint_size(camera_a, i, &mut size);
            let stddev = 0.8 * size / 12.0;
            result.sigma_angles1[i] = Some(2.0_f64.sqrt() * stddev * stddev / (fu1 * fu1));
            let mut bearing1 = Vector3::repeat(f64::NAN);
            if !a.get_back_projection(camera_a, i, &mut bearing1) {
                bearing1 = Vector3::new(1.0, 0.0, 0.0);
            }
            normalize(&mut bearing1);
            result.bearing_vectors1[i] = Some(bearing1);
            b.get_keypoint_size(camera_b, j, &mut size);
            let stddev = 0.8 * size / 12.0;
            result.sigma_angles2[j] = Some(2.0_f64.sqrt() * stddev * stddev / (fu2 * fu2));
            let mut bearing2 = Vector3::repeat(f64::NAN);
            if !b.get_back_projection(camera_b, j, &mut bearing2) {
                // The source's failure branch writes the first frame's ray.
                result.bearing_vectors1[i] = Some(Vector3::new(1.0, 0.0, 0.0));
            }
            normalize(&mut bearing2);
            result.bearing_vectors2[j] = Some(bearing2);
        }
        result
    }
    #[must_use]
    /// # Panics
    /// Panics if a required resource/invariant is absent: `matched bearing not computed`.
    pub fn bearing_vector1(&self, index: usize) -> Vector3<f64> {
        self.bearing_vectors1[self.matches[index].idx_a].expect("matched bearing not computed")
    }
    #[must_use]
    /// # Panics
    /// Panics if a required resource/invariant is absent: `matched bearing not computed`.
    pub fn bearing_vector2(&self, index: usize) -> Vector3<f64> {
        self.bearing_vectors2[self.matches[index].idx_b].expect("matched bearing not computed")
    }
    #[must_use]
    pub fn cam_offset1(&self, _index: usize) -> Vector3<f64> {
        Vector3::zeros()
    }
    #[must_use]
    pub fn cam_offset2(&self, _index: usize) -> Vector3<f64> {
        Vector3::zeros()
    }
    #[must_use]
    pub fn cam_rotation1(&self, _index: usize) -> Matrix3<f64> {
        Matrix3::identity()
    }
    #[must_use]
    pub fn cam_rotation2(&self, _index: usize) -> Matrix3<f64> {
        Matrix3::identity()
    }
    #[must_use]
    pub fn number_correspondences(&self) -> usize {
        self.matches.len()
    }
    #[must_use]
    /// # Panics
    /// Panics if a required resource/invariant is absent: `matched uncertainty not computed`.
    pub fn sigma_angle1(&self, index: usize) -> f64 {
        self.sigma_angles1[self.matches[index].idx_a].expect("matched uncertainty not computed")
    }
    #[must_use]
    /// # Panics
    /// Panics if a required resource/invariant is absent: `matched uncertainty not computed`.
    pub fn sigma_angle2(&self, index: usize) -> f64 {
        self.sigma_angles2[self.matches[index].idx_b].expect("matched uncertainty not computed")
    }
    #[must_use]
    pub fn match_keypoint_idx_a(&self, index: usize) -> usize {
        self.matches[index].idx_a
    }
    #[must_use]
    pub fn match_keypoint_idx_b(&self, index: usize) -> usize {
        self.matches[index].idx_b
    }
    #[must_use]
    pub fn weight(&self, _index: usize) -> f64 {
        1.0
    }
    pub fn set_t12(&mut self, translation: Vector3<f64>) {
        self.t12 = translation;
    }
    pub fn set_r12(&mut self, rotation: Matrix3<f64>) {
        self.r12 = rotation;
    }
    #[must_use]
    pub fn t12(&self) -> Vector3<f64> {
        self.t12
    }
    #[must_use]
    pub fn r12(&self) -> Matrix3<f64> {
        self.r12
    }
}

/// All supported source camera classes store fu as the first intrinsic.
pub(crate) fn focal_length_u(camera: &dyn CameraBase, distortion: DistortionType) -> f64 {
    let expected = match distortion {
        DistortionType::Equidistant => "PinholeCamera<EquidistantDistortion>",
        DistortionType::RadialTangential => "PinholeCamera<RadialTangentialDistortion>",
        DistortionType::RadialTangential8 => "PinholeCamera<RadialTangentialDistortion8>",
        DistortionType::NoDistortion => "EUCMCamera",
    };
    assert_eq!(
        camera.type_name(),
        expected,
        "incorrect camera geometry type"
    );
    let mut intrinsics = DVector::zeros(camera.num_intrinsics());
    camera.get_intrinsics(&mut intrinsics);
    intrinsics[0]
}
/// Eigen's normalize leaves zero vectors unchanged.
pub(crate) fn normalize(vector: &mut Vector3<f64>) {
    let squared = vector.norm_squared();
    if squared > 0.0 {
        *vector /= squared.sqrt();
    }
}
