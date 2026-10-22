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

//! Frame identifiers, associations and map point records from `FrameTypedefs.hpp`.

use crate::kinematics::transformation::Transformation;
use nalgebra::{SVector, Vector2, Vector4};
use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet, HashMap},
    hash::{Hash, Hasher},
    marker::PhantomData,
    ops::Add,
};

/// A distinct identifier domain; zero denotes an uninitialised ID.
#[derive(Debug)]
pub struct NamedId<P> {
    value: u64,
    parameter: PhantomData<fn() -> P>,
}
impl<P> NamedId<P> {
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self {
            value,
            parameter: PhantomData,
        }
    }
    #[must_use]
    pub const fn value(&self) -> u64 {
        self.value
    }
    pub fn value_mut(&mut self) -> &mut u64 {
        &mut self.value
    }
    #[must_use]
    pub const fn is_initialised(&self) -> bool {
        self.value != 0
    }
}
impl<P> Default for NamedId<P> {
    fn default() -> Self {
        Self::new(0)
    }
}
impl<P> Clone for NamedId<P> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<P> Copy for NamedId<P> {}
impl<P> PartialEq for NamedId<P> {
    fn eq(&self, rhs: &Self) -> bool {
        self.value == rhs.value
    }
}
impl<P> Eq for NamedId<P> {}
impl<P> PartialOrd for NamedId<P> {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        Some(self.cmp(rhs))
    }
}
impl<P> Ord for NamedId<P> {
    fn cmp(&self, rhs: &Self) -> Ordering {
        self.value.cmp(&rhs.value)
    }
}
impl<P> Hash for NamedId<P> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.value.hash(state);
    }
}
impl<P> Add<u64> for NamedId<P> {
    type Output = Self;
    fn add(self, rhs: u64) -> Self {
        Self::new(self.value.wrapping_add(rhs))
    }
}
#[derive(Debug)]
pub struct LandmarkIdParameter;
#[derive(Debug)]
pub struct StateIdParameter;
pub type LandmarkId = NamedId<LandmarkIdParameter>;
pub type StateId = NamedId<StateIdParameter>;

/// Comparison is lexicographic: frame ID, camera index, keypoint index.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct KeypointIdentifier {
    pub frame_id: u64,
    pub camera_index: usize,
    pub keypoint_index: usize,
}
impl KeypointIdentifier {
    #[must_use]
    pub const fn new(frame_id: u64, camera_index: usize, keypoint_index: usize) -> Self {
        Self {
            frame_id,
            camera_index,
            keypoint_index,
        }
    }
    #[must_use]
    pub const fn frame_id(&self) -> u64 {
        self.frame_id
    }
    pub fn set_frame_id(&mut self, frame_id: u64) {
        self.frame_id = frame_id;
    }
    #[must_use]
    pub fn is_binary_equal(&self, rhs: &Self) -> bool {
        self == rhs
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Match {
    pub idx_a: usize,
    pub idx_b: usize,
    pub distance: f32,
}
impl Match {
    #[must_use]
    pub const fn new(idx_a: usize, idx_b: usize, distance: f32) -> Self {
        Self {
            idx_a,
            idx_b,
            distance,
        }
    }
}
pub type Matches = Vec<Match>;

#[derive(Clone, Debug)]
pub struct MapPoint {
    pub id: u64,
    pub point: Vector4<f64>,
    pub quality: f64,
    pub state_id: u64,
}
impl MapPoint {
    #[must_use]
    pub fn new(id: u64, point: Vector4<f64>, quality: f64, state_id: u64) -> Self {
        Self {
            id,
            point,
            quality,
            state_id,
        }
    }
}
impl Default for MapPoint {
    fn default() -> Self {
        Self::new(0, Vector4::new(0.0, 0.0, 0.0, 1.0), 0.0, 0)
    }
}

#[derive(Clone, Debug)]
pub struct MapPoint2 {
    pub id: LandmarkId,
    pub point: Vector4<f64>,
    pub observations: BTreeSet<KeypointIdentifier>,
    pub is_initialised: bool,
    pub quality: f64,
    pub classification: i32,
}

pub type AlignedMap<K, V> = BTreeMap<K, V>;
pub type AlignedUnorderedMap<K, V> = HashMap<K, V>;
pub type AlignedVector<V> = Vec<V>;
pub type AlignedSet<V> = BTreeSet<V>;
pub type MapPointVector = Vec<MapPoint>;
pub type MapPoints = BTreeMap<LandmarkId, MapPoint2>;
pub type TransformationMap = BTreeMap<u64, Transformation>;

#[derive(Clone, Debug)]
pub struct Observation {
    pub keypoint_idx: usize,
    pub camera_idx: usize,
    pub frame_id: u64,
    /// Absent for the C++ default constructor's unwritten Eigen vector.
    pub keypoint_measurement: Option<Vector2<f64>>,
    pub keypoint_size: f64,
    pub landmark_w: Option<Vector4<f64>>,
    pub landmark_id: u64,
    pub is_initialized: bool,
    pub classification: i32,
}
impl Observation {
    #[must_use]
    pub fn new(
        (keypoint_idx, keypoint_measurement, keypoint_size): (usize, Vector2<f64>, f64),
        camera_idx: usize,
        frame_id: u64,
        (landmark_w, landmark_id, is_initialized): (Vector4<f64>, u64, bool),
        classification: i32,
    ) -> Self {
        Self {
            keypoint_idx,
            camera_idx,
            frame_id,
            keypoint_measurement: Some(keypoint_measurement),
            keypoint_size,
            landmark_w: Some(landmark_w),
            landmark_id,
            is_initialized,
            classification,
        }
    }
}
impl Default for Observation {
    fn default() -> Self {
        Self {
            keypoint_idx: 0,
            camera_idx: usize::MAX,
            frame_id: 0,
            keypoint_measurement: None,
            keypoint_size: 0.0,
            landmark_w: None,
            landmark_id: 0,
            is_initialized: false,
            classification: -1,
        }
    }
}
pub type ObservationVector = Vec<Observation>;
pub type SpeedAndBias = SVector<f64, 9>;
