// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// See docs/licenses/README.md.
//! `include/basalt/vi_estimator/landmark_database.h` and
//! `src/vi_estimator/landmark_database.cpp`, upstream
//! `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

use crate::utils::common_types::FrameId;
use firefly_calibration::observation::TimeCamId;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::Vector2;
use std::collections::{BTreeMap, BTreeSet, HashMap};

#[derive(Clone, Debug)]
pub struct KeypointObservation<S: BasaltScalar> {
    pub kpt_id: i32,
    pub pos: Vector2<S>,
}

/// Inverse-distance landmark hosted in a particular camera frame.
#[derive(Clone, Debug)]
pub struct Keypoint<S: BasaltScalar> {
    pub direction: Vector2<S>,
    pub inv_dist: S,
    pub host_kf_id: TimeCamId,
    pub obs: BTreeMap<TimeCamId, Vector2<S>>,
    backup_parameters: Option<(Vector2<S>, S)>,
}

impl<S: BasaltScalar> Keypoint<S> {
    #[must_use]
    pub fn new(direction: Vector2<S>, inv_dist: S, host_kf_id: TimeCamId) -> Self {
        Self {
            direction,
            inv_dist,
            host_kf_id,
            obs: BTreeMap::new(),
            backup_parameters: None,
        }
    }

    pub fn backup(&mut self) {
        self.backup_parameters = Some((self.direction, self.inv_dist));
    }

    /// Restores only position parameters, as in the reference.
    /// # Panics
    /// Requires a preceding call to [`Self::backup`].
    pub fn restore(&mut self) {
        (self.direction, self.inv_dist) = self
            .backup_parameters
            .expect("landmark must be backed up before restore");
    }
}

pub type ObservationIndex = HashMap<TimeCamId, BTreeMap<TimeCamId, BTreeSet<usize>>>;

/// Landmark storage and host-to-target observation index.
#[derive(Clone, Debug)]
pub struct LandmarkDatabase<S: BasaltScalar> {
    kpts: HashMap<usize, Keypoint<S>>,
    observations: ObservationIndex,
}

impl<S: BasaltScalar> Default for LandmarkDatabase<S> {
    fn default() -> Self {
        Self {
            kpts: HashMap::new(),
            observations: HashMap::new(),
        }
    }
}

impl<S: BasaltScalar> LandmarkDatabase<S> {
    /// Inserts geometry; existing observations and backup parameters are retained.
    pub fn add_landmark(&mut self, id: usize, position: &Keypoint<S>) {
        let landmark = self.kpts.entry(id).or_insert_with(|| {
            Keypoint::new(position.direction, position.inv_dist, position.host_kf_id)
        });
        landmark.direction = position.direction;
        landmark.inv_dist = position.inv_dist;
        landmark.host_kf_id = position.host_kf_id;
    }

    /// # Panics
    /// Requires the landmark to exist.
    pub fn add_observation(&mut self, target: TimeCamId, observation: &KeypointObservation<S>) {
        let id = observation.kpt_id as usize;
        let landmark = self.kpts.get_mut(&id).expect("unknown landmark");
        landmark.obs.insert(target, observation.pos);
        self.observations
            .entry(landmark.host_kf_id)
            .or_default()
            .entry(target)
            .or_default()
            .insert(id);
    }

    /// # Panics
    /// Requires the landmark to exist.
    #[must_use]
    pub fn landmark(&self, id: usize) -> &Keypoint<S> {
        self.kpts.get(&id).expect("unknown landmark")
    }

    /// # Panics
    /// Requires the landmark to exist. Host and observation topology must remain
    /// consistent with the database index when mutating geometry through this API.
    pub fn landmark_mut(&mut self, id: usize) -> &mut Keypoint<S> {
        self.kpts.get_mut(&id).expect("unknown landmark")
    }

    #[must_use]
    pub fn host_kfs(&self) -> Vec<TimeCamId> {
        self.observations.keys().copied().collect()
    }

    /// Returns one entry per indexed observation, retaining upstream duplicates.
    /// # Panics
    /// Requires an indexed host and consistent landmark topology.
    #[must_use]
    pub fn landmarks_for_host(&self, host: TimeCamId) -> Vec<&Keypoint<S>> {
        self.observations
            .get(&host)
            .expect("unknown host")
            .values()
            .flat_map(|ids| ids.iter().map(|id| &self.kpts[id]))
            .collect()
    }

    #[must_use]
    pub fn observations(&self) -> &ObservationIndex {
        &self.observations
    }
    #[must_use]
    pub fn landmarks(&self) -> &HashMap<usize, Keypoint<S>> {
        &self.kpts
    }
    #[must_use]
    pub fn landmark_exists(&self, id: i32) -> bool {
        self.kpts.contains_key(&(id as usize))
    }
    #[must_use]
    pub fn num_landmarks(&self) -> usize {
        self.kpts.len()
    }
    #[must_use]
    pub fn num_observations(&self) -> usize {
        self.observations
            .values()
            .flat_map(BTreeMap::values)
            .map(BTreeSet::len)
            .sum()
    }
    /// # Panics
    /// Requires the landmark to exist.
    #[must_use]
    pub fn num_landmark_observations(&self, id: usize) -> usize {
        self.landmark(id).obs.len()
    }

    fn remove_index_entry(&mut self, host: TimeCamId, target: TimeCamId, id: usize) {
        let targets = self
            .observations
            .get_mut(&host)
            .expect("missing host observation index");
        let ids = targets
            .get_mut(&target)
            .expect("missing target observation index");
        ids.remove(&id);
        if ids.is_empty() {
            targets.remove(&target);
        }
        if targets.is_empty() {
            self.observations.remove(&host);
        }
    }

    pub fn remove_landmark(&mut self, id: usize) {
        if let Some(landmark) = self.kpts.remove(&id) {
            for target in landmark.obs.keys() {
                self.remove_index_entry(landmark.host_kf_id, *target, id);
            }
        }
    }

    /// # Panics
    /// Requires the landmark to exist and observation indices to be consistent.
    pub fn remove_observations(&mut self, id: usize, targets: &BTreeSet<TimeCamId>) {
        let landmark = self.landmark(id);
        let host = landmark.host_kf_id;
        let removed: Vec<_> = landmark
            .obs
            .keys()
            .filter(|target| targets.contains(target))
            .copied()
            .collect();
        for target in removed {
            self.remove_index_entry(host, target, id);
            self.landmark_mut(id).obs.remove(&target);
        }
        if self.landmark(id).obs.len() < 2 {
            self.remove_landmark(id);
        }
    }

    pub fn remove_frame(&mut self, frame: FrameId) {
        let ids: Vec<_> = self.kpts.keys().copied().collect();
        for id in ids {
            let targets = self.kpts[&id]
                .obs
                .keys()
                .filter(|target| target.frame_id == frame)
                .copied()
                .collect();
            self.remove_observations(id, &targets);
        }
    }

    pub fn remove_keyframes(
        &mut self,
        keyframes: &BTreeSet<FrameId>,
        poses: &BTreeSet<FrameId>,
        states: &BTreeSet<FrameId>,
    ) {
        let ids: Vec<_> = self.kpts.keys().copied().collect();
        for id in ids {
            if keyframes.contains(&self.kpts[&id].host_kf_id.frame_id) {
                self.remove_landmark(id);
            } else {
                let targets = self.kpts[&id]
                    .obs
                    .keys()
                    .filter(|target| {
                        keyframes.contains(&target.frame_id)
                            || poses.contains(&target.frame_id)
                            || states.contains(&target.frame_id)
                    })
                    .copied()
                    .collect();
                self.remove_observations(id, &targets);
            }
        }
    }

    pub fn backup(&mut self) {
        for landmark in self.kpts.values_mut() {
            landmark.backup();
        }
    }

    /// # Panics
    /// Every current landmark must have a saved parameter backup.
    pub fn restore(&mut self) {
        for landmark in self.kpts.values_mut() {
            landmark.restore();
        }
    }
}
