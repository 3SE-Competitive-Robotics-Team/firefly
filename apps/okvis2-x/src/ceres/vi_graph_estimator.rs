/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! State elimination, reversible visual factor compression and graph synchronization.
use crate::{
    cameras::n_camera_system::DistortionType,
    ceres::{
        component::{ComponentGraph, ComponentLandmark, ComponentState, MultiFramePtr},
        imu_error::ImuError,
        parameter_block::ParameterBlock,
        two_pose_extrinsics_graph_error::TwoPoseExtrinsicsGraphError,
        two_pose_graph_error::{
            Observation, SharedPose, TwoPoseGraphErrorConst, TwoPoseStandardGraphError,
        },
        vi_graph::{
            GraphFactor, ManifoldKind, Parameter, RobustLoss, ViGraph, VisualFactor, graph_error,
        },
    },
    frames::{
        frame_typedefs::{KeypointIdentifier, LandmarkId, StateId},
        multi_frame::MultiFrame,
    },
};
use firefly_error::Result;
use nalgebra::Vector4;
use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet},
    ops::{Deref, DerefMut},
    rc::Rc,
};
#[derive(Clone)]
pub struct PoseGraphEdge {
    pub pose_graph_error: Rc<RefCell<VisualFactor>>,
    pub reference_id: StateId,
    pub other_id: StateId,
}
pub struct MstConversion {
    pub created_edges: Vec<PoseGraphEdge>,
    pub removed_edges: Vec<(StateId, StateId)>,
    pub removed_observations: Vec<KeypointIdentifier>,
    pub converted: bool,
}
#[derive(Default)]
pub struct ObservationConversion {
    pub created_landmarks: BTreeSet<LandmarkId>,
    pub connected_states: BTreeSet<StateId>,
    pub observations: Vec<Observation>,
}
pub struct ViGraphEstimator {
    pub graph: ViGraph,
    pub mst_frame_ids: Vec<StateId>,
    pub mst_graph_indices: BTreeMap<StateId, usize>,
    pub mst_edges: Vec<(usize, usize)>,
}
impl Default for ViGraphEstimator {
    fn default() -> Self {
        Self {
            graph: ViGraph::new(),
            mst_frame_ids: Vec::new(),
            mst_graph_indices: BTreeMap::new(),
            mst_edges: Vec::new(),
        }
    }
}
impl Deref for ViGraphEstimator {
    type Target = ViGraph;
    fn deref(&self) -> &ViGraph {
        &self.graph
    }
}
impl DerefMut for ViGraphEstimator {
    fn deref_mut(&mut self) -> &mut ViGraph {
        &mut self.graph
    }
}
impl ViGraphEstimator {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn remove_all_observations(&mut self, id: StateId) -> Result<bool> {
        let keys: Vec<_> = self
            .states
            .get(&id)
            .ok_or_else(|| graph_error("state absent"))?
            .observations
            .keys()
            .copied()
            .collect();
        for key in keys {
            self.remove_observation(key)?;
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn eliminate_state_by_imu_merge(
        &mut self,
        id: StateId,
        reference: StateId,
    ) -> Result<bool> {
        let info = self
            .state_info
            .get(&id)
            .ok_or_else(|| graph_error("state absent"))?
            .clone();
        if !info.two_pose_links.is_empty() || !info.two_pose_const_links.is_empty() {
            return Err(graph_error("cannot eliminate state with two pose links"));
        }
        let previous = self
            .states
            .range(..id)
            .next_back()
            .map(|(&i, _)| i)
            .ok_or_else(|| graph_error("cannot eliminate first state"))?;
        let next = self
            .states
            .range((std::ops::Bound::Excluded(id), std::ops::Bound::Unbounded))
            .next()
            .map(|(&i, _)| i)
            .ok_or_else(|| graph_error("cannot eliminate last state"))?;
        for &other in info.relative_pose_links.keys() {
            self.remove_relative_pose_constraint(id, other)?;
        }
        let pose = self.pose(id)?;
        let sb = self.speed_and_bias(id)?;
        let previous_link = self.state_info[&previous]
            .next_imu
            .ok_or_else(|| graph_error("previous IMU link absent"))?;
        let next_link = info
            .next_imu
            .ok_or_else(|| graph_error("next IMU link absent"))?;
        let previous_factor = self
            .problem
            .residuals
            .get(&previous_link)
            .ok_or_else(|| graph_error("previous IMU residual absent"))?
            .factor
            .clone();
        let next_factor = self
            .problem
            .residuals
            .get(&next_link)
            .ok_or_else(|| graph_error("next IMU residual absent"))?
            .factor
            .clone();
        let factor = match (previous_factor.as_ref(), next_factor.as_ref()) {
            (GraphFactor::Imu(left), GraphFactor::Imu(right)) => {
                let right = right.borrow();
                let mut left = left.borrow_mut();
                left.append(&pose, &sb, &right.imu_measurements, right.t1)
                    .map_err(|e| graph_error(&format!("append IMU: {e:?}")))?;
                let copy = Rc::new(left.clone());
                self.states
                    .get_mut(&previous)
                    .expect("state exists")
                    .next_imu_link = Some(copy.clone());
                self.states
                    .get_mut(&next)
                    .expect("state exists")
                    .previous_imu_link = Some(copy);
                GraphFactor::Imu(Rc::new(RefCell::new(left.clone())))
            }
            (GraphFactor::PseudoImu(left), GraphFactor::PseudoImu(right)) => {
                let mut copy = left.borrow().clone();
                crate::ceres::imu_error::ImuErrorBase::set_t1(
                    &mut copy,
                    crate::ceres::imu_error::ImuErrorBase::t1(&*right.borrow()),
                );
                GraphFactor::PseudoImu(Rc::new(RefCell::new(copy)))
            }
            _ => return Err(graph_error("incompatible IMU links")),
        };
        self.finish_state_elimination(
            (id, previous, next, reference),
            &pose,
            factor,
            (next_link, previous_link),
        )
    }
    fn finish_state_elimination(
        &mut self,
        (id, previous, next, reference): (StateId, StateId, StateId, StateId),
        pose: &crate::kinematics::transformation::Transformation,
        factor: GraphFactor,
        (next_link, previous_link): (
            crate::ceres::vi_graph::ResidualId,
            crate::ceres::vi_graph::ResidualId,
        ),
    ) -> Result<bool> {
        self.problem.residuals.remove(&next_link);
        self.problem.residuals.remove(&previous_link);
        let pose_block = self.pose_block(id)?;
        let sb_block = self.speed_bias_block(id)?;
        self.problem.remove_parameter(&Parameter::Pose(pose_block));
        self.problem
            .remove_parameter(&Parameter::SpeedBias(sb_block));
        let parameters = vec![
            Parameter::Pose(self.pose_block(previous)?),
            Parameter::SpeedBias(self.speed_bias_block(previous)?),
            Parameter::Pose(self.pose_block(next)?),
            Parameter::SpeedBias(self.speed_bias_block(next)?),
        ];
        let rid = self.problem.add(factor, parameters, None);
        self.state_info
            .get_mut(&previous)
            .expect("state exists")
            .next_imu = Some(rid);
        self.state_info
            .get_mut(&next)
            .expect("state exists")
            .previous_imu = Some(rid);
        for camera in 0..self.states[&id].extrinsics.len() {
            let ext = self.extrinsics_block(id, camera)?;
            if !ext.borrow().fixed()
                && ext.borrow().id() != self.extrinsics_block(previous, camera)?.borrow().id()
            {
                return Err(graph_error("varying extrinsics not supported"));
            }
        }
        if !reference.is_initialised() {
            return Err(graph_error("no reference keyframe"));
        }
        let t_sk_w = self.pose(reference)?.inverse();
        let v = t_sk_w.rotation_matrix() * self.speed_and_bias(reference)?.fixed_rows::<3>(0);
        let any = self
            .any_state
            .get_mut(&id)
            .ok_or_else(|| graph_error("historical state absent"))?;
        any.keyframe_id = Some(reference);
        any.t_sk_s = t_sk_w.compose(pose);
        any.v_sk = v;
        self.states.remove(&id);
        self.state_info.remove(&id);
        self.gps_states.remove(&id);
        self.gps_reinit_states.remove(&id);
        self.gps_init_map.remove(&id);
        Ok(true)
    }

    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn merge_landmark(
        &mut self,
        from: LandmarkId,
        into: LandmarkId,
        frames: &mut BTreeMap<StateId, MultiFramePtr>,
    ) -> Result<bool> {
        if !self.landmarks.contains_key(&from) || !self.landmarks.contains_key(&into) {
            return Err(graph_error("landmark absent"));
        }
        let keys = self.landmark_info[&from].observations.clone();
        for key in keys {
            let obs = self.observations[&key].clone();
            let cauchy = self.problem.residuals[&obs.residual].loss.is_some();
            let factor = obs.core.error_term.borrow().clone_box();
            self.remove_observation(key)?;
            self.add_external_observation(factor.as_ref(), into, key, cauchy)?;
            frames
                .get(&StateId::new(key.frame_id))
                .ok_or_else(|| graph_error("multiframe absent"))?
                .borrow_mut()
                .set_landmark_id(key.camera_index, key.keypoint_index, into.value());
        }
        self.remove_landmark(from)?;
        Ok(true)
    }
    fn freeze_blocks(&mut self, id: StateId, remove: bool, speed_bias: bool) -> Result<bool> {
        if !self.states.contains_key(&id) {
            return Err(graph_error("state absent"));
        }
        let ids: Vec<_> = self.states.range(..=id).rev().map(|(&id, _)| id).collect();
        let mut first = true;
        for id in ids {
            let block = if speed_bias {
                Parameter::SpeedBias(self.speed_bias_block(id)?)
            } else {
                Parameter::Pose(self.pose_block(id)?)
            };
            if block.fixed() {
                if remove {
                    self.problem.remove_parameter(&block);
                }
                break;
            }
            block.set_fixed(true);
            if first || !remove {
                first = false;
            } else {
                self.problem.remove_parameter(&block);
            }
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn freeze_poses_until(&mut self, id: StateId, remove: bool) -> Result<bool> {
        self.freeze_blocks(id, remove, false)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn freeze_speed_and_biases_until(&mut self, id: StateId, remove: bool) -> Result<bool> {
        self.freeze_blocks(id, remove, true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn unfreeze_poses_from(&mut self, id: StateId) -> Result<bool> {
        self.unfreeze_blocks(id, false)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn unfreeze_speed_and_biases_from(&mut self, id: StateId) -> Result<bool> {
        self.unfreeze_blocks(id, true)
    }
    fn unfreeze_blocks(&mut self, id: StateId, speed_bias: bool) -> Result<bool> {
        if !self.states.contains_key(&id) {
            return Err(graph_error("state absent"));
        }
        for &sid in self.states.range(id..).map(|(id, _)| id) {
            let block = if speed_bias {
                Parameter::SpeedBias(self.speed_bias_block(sid)?)
            } else {
                Parameter::Pose(self.pose_block(sid)?)
            };
            if !self.problem.parameters.contains_key(&block.key()) {
                return Err(graph_error("removed parameter cannot be unfrozen"));
            }
            block.set_fixed(false);
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn freeze_extrinsics_until(&mut self, id: StateId, remove: bool) -> Result<bool> {
        if remove {
            return Err(graph_error("removing extrinsics not supported"));
        }
        if !self.states.contains_key(&id) {
            return Err(graph_error("state absent"));
        }
        for state in self.states.range(..=id).rev().map(|(_, s)| s) {
            for ext in &state.extrinsics {
                let ext = ext
                    .as_ref()
                    .ok_or_else(|| graph_error("extrinsics absent"))?;
                if ext.borrow().fixed() {
                    break;
                }
                ext.borrow_mut().set_fixed(true);
            }
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn unfreeze_extrinsics_from(&mut self, id: StateId) -> Result<bool> {
        if !self.states.contains_key(&id) {
            return Err(graph_error("state absent"));
        }
        for state in self.states.range(id..).map(|(_, s)| s) {
            for ext in &state.extrinsics {
                ext.as_ref()
                    .ok_or_else(|| graph_error("extrinsics absent"))?
                    .borrow_mut()
                    .set_fixed(false);
            }
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn build_mst(&mut self, states: &BTreeSet<StateId>, keyframes_only: bool) -> Result<bool> {
        fn find(parents: &mut [usize], i: usize) -> usize {
            if parents[i] != i {
                parents[i] = find(parents, parents[i]);
            }
            parents[i]
        }

        if !self.compute_covisibilities() {
            return Err(graph_error("covisibilities could not be computed"));
        }
        self.mst_frame_ids = states.iter().copied().collect();
        self.mst_graph_indices = self
            .mst_frame_ids
            .iter()
            .enumerate()
            .map(|(i, &id)| (id, i))
            .collect();
        self.mst_edges.clear();
        let mut edges = Vec::new();
        for (&a, counts) in &self.co_observation_counts {
            for (&b, &count) in counts {
                let (a, b) = (StateId::new(a), StateId::new(b));
                if (!keyframes_only || (self.is_keyframe(a) && self.is_keyframe(b)))
                    && self.mst_graph_indices.contains_key(&a)
                    && self.mst_graph_indices.contains_key(&b)
                {
                    edges.push((
                        -count,
                        (self.mst_graph_indices[&a], self.mst_graph_indices[&b]),
                    ));
                }
            }
        }
        edges.sort_unstable();
        let mut parents: Vec<_> = (0..self.mst_frame_ids.len()).collect();
        let mut ranks = vec![0; parents.len()];

        for (_, (a, b)) in edges {
            let pa = find(&mut parents, a);
            let pb = find(&mut parents, b);
            if pa != pb {
                self.mst_edges.push((a, b));
                if ranks[pa] > ranks[pb] {
                    parents[pb] = pa;
                } else {
                    parents[pa] = pb;
                    if ranks[pa] == ranks[pb] {
                        ranks[pb] += 1;
                    }
                }
            }
        }
        Ok(!self.mst_edges.is_empty())
    }
    fn new_visual_factor(&self, a: StateId, b: StateId) -> Result<VisualFactor> {
        let cameras = self.camera_parameters.len();
        if self
            .camera_parameters
            .first()
            .ok_or_else(|| graph_error("camera parameters absent"))?
            .online_calibration
            .do_extrinsics
        {
            Ok(VisualFactor::Extrinsics(TwoPoseExtrinsicsGraphError::new(
                a, b, cameras, false,
            )))
        } else {
            Ok(VisualFactor::Standard(TwoPoseStandardGraphError::new(
                a, b, cameras, false,
            )))
        }
    }
    fn visual_parameters(&self, a: StateId, b: StateId) -> Result<Vec<Parameter>> {
        let mut parameters = vec![
            Parameter::Pose(self.pose_block(a)?),
            Parameter::Pose(self.pose_block(b)?),
        ];
        if self
            .camera_parameters
            .first()
            .is_some_and(|c| c.online_calibration.do_extrinsics)
        {
            for i in 0..self.camera_parameters.len() {
                parameters.push(Parameter::Pose(self.extrinsics_block(a, i)?));
            }
        }
        Ok(parameters)
    }
    fn shared_landmarks(&self, a: StateId, b: StateId) -> BTreeSet<LandmarkId> {
        let left: BTreeSet<_> = self.states[&a]
            .observations
            .values()
            .map(|o| o.landmark_id)
            .collect();
        let right: BTreeSet<_> = self.states[&b]
            .observations
            .values()
            .map(|o| o.landmark_id)
            .collect();
        left.intersection(&right).copied().collect()
    }
    fn add_visual_observation(
        &self,
        factor: &mut VisualFactor,
        key: KeypointIdentifier,
        duplicate: bool,
        weight: f64,
    ) -> Result<()> {
        let obs = self
            .observations
            .get(&key)
            .ok_or_else(|| graph_error("observation absent"))?;
        let id = StateId::new(key.frame_id);
        factor.base_mut().add_observation(
            key,
            obs.core.error_term.borrow().as_ref(),
            Some(Rc::new(RobustLoss::Cauchy(1.0))),
            (
                self.pose_block(id)?,
                self.landmarks[&obs.core.landmark_id].h_point.clone(),
                self.extrinsics_block(id, key.camera_index)?,
            ),
            duplicate,
            weight,
        )?;
        Ok(())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn convert_to_pose_graph_mst(
        &mut self,
        states: &BTreeSet<StateId>,
        considered: &BTreeSet<StateId>,
    ) -> Result<MstConversion> {
        self.build_mst(considered, true)?;
        let mut output = MstConversion {
            created_edges: Vec::new(),
            removed_edges: Vec::new(),
            removed_observations: Vec::new(),
            converted: false,
        };
        if self.mst_edges.is_empty() {
            return Ok(output);
        }
        let mut counts: BTreeMap<StateId, usize> = BTreeMap::new();
        for &(a, b) in &self.mst_edges {
            *counts.entry(self.mst_frame_ids[a]).or_default() += 1;
            *counts.entry(self.mst_frame_ids[b]).or_default() += 1;
        }
        let mut edges: Vec<_> = self
            .mst_edges
            .iter()
            .copied()
            .filter(|&(a, b)| {
                states.contains(&self.mst_frame_ids[a]) || states.contains(&self.mst_frame_ids[b])
            })
            .collect();
        let oldest = *self.mst_frame_ids.first().expect("MST has nodes");
        let newest = *self.mst_frame_ids.last().expect("MST has nodes");
        let old_index = self.mst_graph_indices[&oldest];
        let new_index = self.mst_graph_indices[&newest];
        if states.contains(&oldest)
            && oldest != newest
            && self.covisibilities(oldest, newest)? >= 2
            && !edges.iter().any(|&(a, b)| {
                (a == old_index && b == new_index) || (a == new_index && b == old_index)
            })
        {
            edges.push((old_index, new_index));
            *counts.entry(oldest).or_default() += 1;
            *counts.entry(newest).or_default() += 1;
        }
        self.convert_mst_edges(states, edges, &mut counts, &mut output)?;
        output.converted = true;
        Ok(output)
    }
    fn convert_mst_edges(
        &mut self,
        states: &BTreeSet<StateId>,
        edges: Vec<(usize, usize)>,
        counts: &mut BTreeMap<StateId, usize>,
        output: &mut MstConversion,
    ) -> Result<()> {
        for (i, j) in edges {
            let (a, b) = (
                self.mst_frame_ids[i].min(self.mst_frame_ids[j]),
                self.mst_frame_ids[i].max(self.mst_frame_ids[j]),
            );
            if !self.is_keyframe(a) || !self.is_keyframe(b) {
                return Err(graph_error("MST endpoint must be keyframe"));
            }
            let mut factor = self.new_visual_factor(a, b)?;
            if let Some(rid) = self.state_info[&a].two_pose_links.get(&b).copied() {
                let existing = self.problem.residuals[&rid].factor.clone();
                if let GraphFactor::Visual(existing) = existing.as_ref() {
                    let mut observations = Vec::new();
                    existing
                        .borrow_mut()
                        .convert(&mut observations, &mut Vec::new())?;
                    for obs in observations {
                        factor.base_mut().add_observation(
                            obs.keypoint_identifier,
                            obs.reprojection_error.borrow().as_ref(),
                            obs.loss_function,
                            (obs.pose, obs.h_point, obs.extrinsics),
                            obs.is_duplication,
                            1.0,
                        )?;
                    }
                }
                self.remove_visual_link(a, b, rid);
                output.removed_edges.push((a, b));
            }
            let keep_a = counts.get(&a).copied().unwrap_or(0) > 1 || !states.contains(&a);
            let keep_b = counts.get(&b).copied().unwrap_or(0) > 1 || !states.contains(&b);
            let shared = self.shared_landmarks(a, b);
            for (id, keep) in [(a, keep_a), (b, keep_b)] {
                let keys: Vec<_> = self.states[&id].observations.keys().copied().collect();
                for key in keys {
                    let lm = self.observations[&key].core.landmark_id;
                    if shared.contains(&lm) {
                        self.add_visual_observation(&mut factor, key, keep, 1.0)?;
                    }
                    if !keep {
                        self.remove_observation(key)?;
                        output.removed_observations.push(key);
                    }
                }
            }
            if !shared.is_empty() {
                factor.compute()?;
                let shared = Rc::new(RefCell::new(factor));
                let parameters = self.visual_parameters(a, b)?;
                let rid = self
                    .problem
                    .add(GraphFactor::Visual(shared.clone()), parameters, None);
                self.state_info
                    .get_mut(&a)
                    .expect("state exists")
                    .two_pose_links
                    .insert(b, rid);
                self.state_info
                    .get_mut(&b)
                    .expect("state exists")
                    .two_pose_links
                    .insert(a, rid);
                output.created_edges.push(PoseGraphEdge {
                    pose_graph_error: shared,
                    reference_id: a,
                    other_id: b,
                });
            }
            *counts
                .get_mut(&a)
                .ok_or_else(|| graph_error("MST degree absent"))? -= 1;
            *counts
                .get_mut(&b)
                .ok_or_else(|| graph_error("MST degree absent"))? -= 1;
        }
        Ok(())
    }

    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn obtain_pose_graph_mst(&mut self) -> Result<Vec<PoseGraphEdge>> {
        self.compute_covisibilities();
        let visible = self.visible_frames.clone();
        self.build_mst(&visible, false)?;
        if self.mst_edges.is_empty() {
            return Err(graph_error("MST not created"));
        }
        let mut counts: BTreeMap<StateId, usize> = BTreeMap::new();
        for &(a, b) in &self.mst_edges {
            *counts.entry(self.mst_frame_ids[a]).or_default() += 1;
            *counts.entry(self.mst_frame_ids[b]).or_default() += 1;
        }
        let mut output = Vec::new();
        for (i, j) in self.mst_edges.clone() {
            let (a, b) = (
                self.mst_frame_ids[i].min(self.mst_frame_ids[j]),
                self.mst_frame_ids[i].max(self.mst_frame_ids[j]),
            );
            let mut factor = self.new_visual_factor(a, b)?;
            let shared = self.shared_landmarks(a, b);
            for id in [a, b] {
                let duplicate = counts[&id] > 1 || !visible.contains(&id);
                for &key in self.states[&id].observations.keys() {
                    if shared.contains(&self.observations[&key].core.landmark_id) {
                        self.add_visual_observation(&mut factor, key, duplicate, 1.0)?;
                    }
                }
            }
            if !shared.is_empty() {
                factor.compute()?;
                output.push(PoseGraphEdge {
                    pose_graph_error: Rc::new(RefCell::new(factor)),
                    reference_id: a,
                    other_id: b,
                });
            }
            *counts.get_mut(&a).expect("degree exists") -= 1;
            *counts.get_mut(&b).expect("degree exists") -= 1;
        }
        Ok(output)
    }
    fn remove_visual_link(&mut self, a: StateId, b: StateId, rid: u64) {
        self.problem.residuals.remove(&rid);
        if let Some(info) = self.state_info.get_mut(&a) {
            info.two_pose_links.remove(&b);
        }
        if let Some(info) = self.state_info.get_mut(&b) {
            info.two_pose_links.remove(&a);
        }
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn add_external_two_pose_link(
        &mut self,
        error: Rc<dyn TwoPoseGraphErrorConst>,
        a: StateId,
        b: StateId,
    ) -> Result<bool> {
        if self
            .state_info
            .get(&a)
            .is_some_and(|s| s.two_pose_const_links.contains_key(&b))
            || self
                .state_info
                .get(&b)
                .is_some_and(|s| s.two_pose_const_links.contains_key(&a))
        {
            return Err(graph_error("duplicate constant pose link"));
        }
        let parameters = self.visual_parameters(a, b)?;
        let rid = self
            .problem
            .add(GraphFactor::VisualConst(error), parameters, None);
        self.state_info
            .get_mut(&a)
            .ok_or_else(|| graph_error("state absent"))?
            .two_pose_const_links
            .insert(b, rid);
        self.state_info
            .get_mut(&b)
            .ok_or_else(|| graph_error("state absent"))?
            .two_pose_const_links
            .insert(a, rid);
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn remove_two_pose_const_links(&mut self, id: StateId) -> Result<bool> {
        let others: Vec<_> = self
            .state_info
            .get(&id)
            .ok_or_else(|| graph_error("state absent"))?
            .two_pose_const_links
            .keys()
            .copied()
            .collect();
        for other in others {
            self.remove_two_pose_const_link(id, other)?;
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn remove_two_pose_const_link(&mut self, a: StateId, b: StateId) -> Result<bool> {
        let rid = self
            .state_info
            .get_mut(&a)
            .ok_or_else(|| graph_error("state absent"))?
            .two_pose_const_links
            .remove(&b)
            .ok_or_else(|| graph_error("constant pose link absent"))?;
        self.state_info
            .get_mut(&b)
            .ok_or_else(|| graph_error("state absent"))?
            .two_pose_const_links
            .remove(&a)
            .ok_or_else(|| graph_error("reverse constant pose link absent"))?;
        self.problem.residuals.remove(&rid);
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn convert_to_observations(&mut self, id: StateId) -> Result<ObservationConversion> {
        let links = self
            .state_info
            .get(&id)
            .ok_or_else(|| graph_error("state absent"))?
            .two_pose_links
            .clone();
        let mut output = ObservationConversion::default();
        for (other, rid) in links {
            let factor = self
                .problem
                .residuals
                .get(&rid)
                .ok_or_else(|| graph_error("visual residual absent"))?
                .factor
                .clone();
            let GraphFactor::Visual(factor) = factor.as_ref() else {
                return Err(graph_error("visual factor invalid"));
            };
            self.state_info
                .get_mut(&other)
                .ok_or_else(|| graph_error("connected state absent"))?
                .two_pose_links
                .remove(&id);
            output.connected_states.insert(other);
            let mut observations = Vec::new();
            factor
                .borrow_mut()
                .convert(&mut observations, &mut Vec::new())?;
            for obs in observations {
                let key = obs.keypoint_identifier;
                if self.observations.contains_key(&key) {
                    continue;
                }
                let landmark_id = LandmarkId::new(obs.h_point.borrow().id());
                if !self.landmarks.contains_key(&landmark_id) {
                    let point = obs
                        .h_point
                        .borrow()
                        .estimate()
                        .copied()
                        .ok_or_else(|| graph_error("compressed point absent"))?;
                    let initialised = obs.h_point.borrow().initialized();
                    self.add_landmark(landmark_id, point, initialised)?;
                }
                output.created_landmarks.insert(landmark_id);
                self.add_external_observation(
                    obs.reprojection_error.borrow().as_ref(),
                    landmark_id,
                    key,
                    obs.loss_function.is_some(),
                )?;
                output.observations.push(obs);
            }
            self.problem.residuals.remove(&rid);
        }
        self.state_info
            .get_mut(&id)
            .expect("state checked")
            .two_pose_links
            .clear();
        Ok(output)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn remove_speed_and_bias_prior(&mut self, id: StateId) -> Result<bool> {
        if let Some(rid) = self
            .state_info
            .get_mut(&id)
            .ok_or_else(|| graph_error("state absent"))?
            .speed_bias_prior
            .take()
        {
            self.problem.residuals.remove(&rid);
            Ok(true)
        } else {
            Ok(false)
        }
    }
    pub fn clear(&mut self) {
        self.mst_frame_ids.clear();
        self.mst_graph_indices.clear();
        self.mst_edges.clear();
        self.states.clear();
        self.state_info.clear();
        self.landmarks.clear();
        self.landmark_info.clear();
        self.observations.clear();
        self.problem = crate::ceres::vi_graph::GraphProblem::default();
        self.co_observation_counts.clear();
        self.covisibilities_computed = true;
        self.visible_frames.clear();
        self.any_state.clear();
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn check_synched_states(&self, other: &Self) -> Result<()> {
        for &id in other.states.keys() {
            if !self.states.contains_key(&id) || !self.states_approx(other, id)? {
                return Err(graph_error("state estimates or keyframe flags differ"));
            }
        }
        Ok(())
    }
    fn states_approx(&self, other: &Self, id: StateId) -> Result<bool> {
        Ok(self.is_keyframe(id) == other.is_keyframe(id)
            && eigen_approx(
                self.pose(id)?.matrix().as_slice(),
                other.pose(id)?.matrix().as_slice(),
            )
            && eigen_approx(
                self.speed_and_bias(id)?.as_slice(),
                other.speed_and_bias(id)?.as_slice(),
            ))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn is_observationless_synched(&self, other: &Self) -> Result<bool> {
        for (&id, right) in &other.state_info {
            let Some(left) = self.state_info.get(&id) else {
                return Ok(false);
            };
            if !self.states_approx(other, id)? {
                return Ok(false);
            }
            if left.two_pose_links.len() != right.two_pose_links.len()
                && left.two_pose_const_links.len() != right.two_pose_links.len()
                && left.two_pose_links.len() != right.two_pose_const_links.len()
            {
                return Ok(false);
            }
            if left.relative_pose_links.len() != right.relative_pose_links.len() {
                return Ok(false);
            }
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn is_synched(&self, other: &Self) -> Result<bool> {
        if self.observations.keys().ne(other.observations.keys()) {
            return Ok(false);
        }
        for (&key, left) in &self.observations {
            let right = &other.observations[&key];
            let l = left.core.error_term.borrow();
            let r = right.core.error_term.borrow();
            if !eigen_approx(
                l.information()
                    .ok_or_else(|| graph_error("observation information absent"))?
                    .as_slice(),
                r.information()
                    .ok_or_else(|| graph_error("observation information absent"))?
                    .as_slice(),
            ) {
                return Ok(false);
            }
        }
        let mut errors = 0;
        for (&id, left) in &self.landmark_info {
            let Some(right) = other.landmark_info.get(&id) else {
                return Ok(false);
            };
            for &key in &right.observations {
                if !left.observations.contains(&key) {
                    return Ok(false);
                }
                let r0 = other
                    .observation_evaluation(key, false)?
                    .residuals
                    .norm_squared();
                let r1 = self
                    .observation_evaluation(key, false)?
                    .residuals
                    .norm_squared();
                errors += 1;
                if (r0 - r1).abs() > 1.0e-10 {
                    return Ok(false);
                }
            }
        }
        for (&id, right) in &other.landmark_info {
            let Some(left) = self.landmark_info.get(&id) else {
                return Ok(false);
            };
            if !eigen_approx(
                self.landmark(id)?.as_slice(),
                other.landmark(id)?.as_slice(),
            ) {
                return Ok(false);
            }
            for &key in &left.observations {
                if !right.observations.contains(&key) {
                    return Ok(false);
                }
                let r0 = other
                    .observation_evaluation(key, false)?
                    .residuals
                    .norm_squared();
                let r1 = self
                    .observation_evaluation(key, false)?
                    .residuals
                    .norm_squared();
                if (r0 - r1).abs() > 1.0e-10 {
                    return Ok(false);
                }
            }
        }
        self.state_factors_synched(other, errors)
    }
    fn state_factors_synched(&self, other: &Self, mut errors: usize) -> Result<bool> {
        let mut visual_errors = 0;
        for (&id, right) in &other.state_info {
            let Some(left) = self.state_info.get(&id) else {
                return Ok(false);
            };
            if !self.states_approx(other, id)? {
                return Ok(false);
            }
            if left.two_pose_links.len() != right.two_pose_links.len() {
                if left.two_pose_const_links.len() != right.two_pose_links.len()
                    && left.two_pose_links.len() != right.two_pose_const_links.len()
                {
                    return Ok(false);
                }
                let left_links: Vec<_> = left
                    .two_pose_links
                    .values()
                    .chain(left.two_pose_const_links.values())
                    .copied()
                    .collect();
                let right_links: Vec<_> = right
                    .two_pose_links
                    .values()
                    .chain(right.two_pose_const_links.values())
                    .copied()
                    .collect();
                if left_links.len() != right_links.len() {
                    return Err(graph_error(
                        "source visual link comparison indexes beyond its vector",
                    ));
                }
                for (l, r) in left_links.iter().zip(&right_links) {
                    let lr = &self.problem.residuals[l];
                    let rr = &other.problem.residuals[r];
                    let lp: Vec<_> = lr
                        .parameters
                        .iter()
                        .take(2)
                        .map(Parameter::values)
                        .collect::<Result<_>>()?;
                    let rp: Vec<_> = rr
                        .parameters
                        .iter()
                        .take(2)
                        .map(Parameter::values)
                        .collect::<Result<_>>()?;
                    let r0 = factor_norm(lr.factor.as_ref(), &lp)?;
                    let r1 = factor_norm(rr.factor.as_ref(), &rp)?;
                    visual_errors += 1;
                    if (r0 - r1).abs() > 1.0e-10 {
                        return Ok(false);
                    }
                }
            }
            if left.relative_pose_links.len() != right.relative_pose_links.len() {
                return Ok(false);
            }
            for (left_id, right_id) in [
                (left.previous_imu, right.previous_imu),
                (left.pose_prior, right.pose_prior),
                (left.speed_bias_prior, right.speed_bias_prior),
            ] {
                if let Some(l) = left_id {
                    let r = right_id.ok_or_else(|| graph_error("comparison factor absent"))?;
                    let right_residual = &other.problem.residuals[&r];
                    let parameters: Vec<_> = right_residual
                        .parameters
                        .iter()
                        .map(Parameter::values)
                        .collect::<Result<_>>()?;
                    let r0 = factor_norm(self.problem.residuals[&l].factor.as_ref(), &parameters)?;
                    let r1 = factor_norm(right_residual.factor.as_ref(), &parameters)?;
                    errors += 1;
                    if (r0 - r1).abs() > 1.0e-10 {
                        return Ok(false);
                    }
                }
            }
        }
        errors += visual_errors / 2;
        Ok(
            self.problem.residuals.len() == other.problem.residuals.len()
                && self.problem.residuals.len() == errors,
        )
    }
}
impl ComponentGraph for ViGraphEstimator {
    fn states(&self) -> &BTreeMap<StateId, ComponentState> {
        &self.states
    }
    fn states_mut(&mut self) -> &mut BTreeMap<StateId, ComponentState> {
        &mut self.states
    }
    fn landmarks(&self) -> &BTreeMap<LandmarkId, ComponentLandmark> {
        &self.landmarks
    }
    fn register_pose(&mut self, pose: SharedPose) -> Result<()> {
        let id = StateId::new(pose.borrow().id());
        self.state_info.entry(id).or_default();
        self.problem
            .register(Parameter::Pose(pose), ManifoldKind::Pose);
        Ok(())
    }
    fn register_speed_and_bias(
        &mut self,
        p: crate::ceres::vi_graph::SharedSpeedBias,
    ) -> Result<()> {
        self.problem
            .register(Parameter::SpeedBias(p), ManifoldKind::Euclidean);
        Ok(())
    }
    fn register_extrinsics(&mut self, p: SharedPose) -> Result<()> {
        self.problem
            .register(Parameter::Pose(p), ManifoldKind::Pose);
        Ok(())
    }
    fn add_landmark(&mut self, id: LandmarkId, p: Vector4<f64>, initialized: bool) -> Result<()> {
        self.graph.add_landmark(id, p, initialized)?;
        Ok(())
    }
    fn set_landmark_quality(&mut self, id: LandmarkId, quality: f64) -> Result<()> {
        self.graph.set_landmark_quality(id, quality)?;
        Ok(())
    }
    fn add_observation(
        &mut self,
        frame: &MultiFrame,
        landmark: LandmarkId,
        keypoint: KeypointIdentifier,
        distortion: DistortionType,
        robust: bool,
    ) -> Result<()> {
        self.graph
            .add_observation(frame, landmark, keypoint, distortion, robust)?;
        Ok(())
    }
    fn register_imu_error(
        &mut self,
        previous: StateId,
        current: StateId,
        error: Rc<ImuError>,
    ) -> Result<()> {
        let parameters = vec![
            Parameter::Pose(self.pose_block(previous)?),
            Parameter::SpeedBias(self.speed_bias_block(previous)?),
            Parameter::Pose(self.pose_block(current)?),
            Parameter::SpeedBias(self.speed_bias_block(current)?),
        ];
        let rid = self.problem.add(
            GraphFactor::Imu(Rc::new(RefCell::new((*error).clone()))),
            parameters,
            None,
        );
        self.state_info.entry(previous).or_default().next_imu = Some(rid);
        self.state_info.entry(current).or_default().previous_imu = Some(rid);
        Ok(())
    }
}

fn eigen_approx(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).map(|(a, b)| (a - b) * (a - b)).sum::<f64>()
            <= 1.0e-24
                * a.iter()
                    .map(|v| v * v)
                    .sum::<f64>()
                    .min(b.iter().map(|v| v * v).sum::<f64>())
}
fn factor_norm(factor: &GraphFactor, p: &[Vec<f64>]) -> Result<f64> {
    use crate::ceres::error_interface::ErrorInterface;
    use nalgebra::SVector;
    let slices: Vec<_> = p.iter().map(Vec::as_slice).collect();
    match factor {
        GraphFactor::PosePrior(f) => {
            let mut r = SVector::<f64, 6>::zeros();
            f.evaluate(&SVector::from_column_slice(&p[0]), &mut r, None);
            Ok(r.norm_squared())
        }
        GraphFactor::SpeedBiasPrior(f) => {
            let mut r = SVector::<f64, 9>::zeros();
            f.evaluate(&SVector::from_column_slice(&p[0]), &mut r, None);
            Ok(r.norm_squared())
        }
        GraphFactor::Imu(f) => Ok(f
            .borrow_mut()
            .evaluate(
                &SVector::from_column_slice(&p[0]),
                &SVector::from_column_slice(&p[1]),
                &SVector::from_column_slice(&p[2]),
                &SVector::from_column_slice(&p[3]),
            )
            .map_err(|e| graph_error(&format!("IMU check: {e:?}")))?
            .residual
            .norm_squared()),
        GraphFactor::PseudoImu(f) => Ok(f
            .borrow()
            .evaluate(
                &SVector::from_column_slice(&p[0]),
                &SVector::from_column_slice(&p[1]),
                &SVector::from_column_slice(&p[2]),
                &SVector::from_column_slice(&p[3]),
            )
            .residual
            .norm_squared()),
        GraphFactor::Visual(f) => {
            let f = f.borrow();
            let evaluation = match &*f {
                VisualFactor::Standard(f) => f.evaluate_with_minimal_jacobians(&slices, None, None),
                VisualFactor::Extrinsics(f) => {
                    f.evaluate_with_minimal_jacobians(&slices, None, None)
                }
            };
            Ok(evaluation
                .ok_or_else(|| graph_error("visual factor comparison failed"))?
                .residuals
                .norm_squared())
        }
        GraphFactor::VisualConst(f) => Ok(f
            .evaluate_with_minimal_jacobians(&slices, None, None)
            .ok_or_else(|| graph_error("constant visual factor comparison failed"))?
            .residuals
            .norm_squared()),
        _ => Err(graph_error("unexpected comparison factor")),
    }
}
