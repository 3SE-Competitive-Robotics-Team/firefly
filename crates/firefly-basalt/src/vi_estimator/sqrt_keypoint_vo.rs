// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Official `sqrt_keypoint_vo.h` / `src/vi_estimator/sqrt_keypoint_vo.cpp`.
use super::{
    ba_base::BundleAdjustmentBase,
    landmark_database::{Keypoint, KeypointObservation},
    marg_helper::MargHelper,
    vio_estimator::{
        VioEstimatorEngine, VioStatus, VioVisualizationData, VioWorkerInputs, VioWorkerOutputs,
    },
};
use crate::imu::imu_types::PoseVelBiasState;
use crate::{
    linearization::linearization_base::{LinearizationBase, Options, is_linearization_sqrt},
    optical_flow::OpticalFlowResult,
    utils::{
        imu_types::{AbsOrderMap, MargData, MargLinData, PoseStateWithLin},
        ldlt::Ldlt,
        time_utils::{ExecutionStats, Timer},
        vio_config::VioConfig,
    },
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::stereographic_param::StereographicParam;
use firefly_calibration::observation::TimeCamId;
use firefly_calibration::scalar::BasaltScalar;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{DMatrix, DVector, Isometry3, Vector3, Vector4};
use std::{
    collections::{BTreeMap, BTreeSet, HashSet},
    sync::{Arc, atomic::Ordering},
};

pub struct SqrtKeypointVoEstimator<S: BasaltScalar> {
    pub base: BundleAdjustmentBase<S>,
    pub marg_data: MargLinData<S>,
    nullspace_marg_data: MargLinData<S>,
    config: VioConfig,
    take_kf: bool,
    frames_after_kf: i32,
    kf_ids: BTreeSet<i64>,
    last_state_t_ns: i64,
    prev_opt_flow_res: BTreeMap<i64, Arc<OpticalFlowResult>>,
    num_points_kf: BTreeMap<i64, i32>,
    max_states: usize,
    max_kfs: usize,
    t_w_i_init: Isometry3<S>,
    initialized: bool,
    lambda: S,
    min_lambda: S,
    max_lambda: S,
    lambda_vee: S,
    pub stats_all: ExecutionStats,
    pub stats_sums: ExecutionStats,
}
fn scalar<S: BasaltScalar>(value: f64) -> S {
    S::from_f64_lossy(value)
}
fn add_order(order: &mut AbsOrderMap, time: i64) {
    if !order.abs_order_map.contains_key(&time) {
        order
            .abs_order_map
            .insert(time, (order.total_size as i32, 6));
        order.total_size += 6;
        order.items += 1;
    }
}
impl<S: BasaltScalar> SqrtKeypointVoEstimator<S> {
    pub fn get_state(&self) -> &PoseVelBiasState<S> {
        // Original legacy getter indexes IMU states even for VO.
        self.base.frame_states[&self.last_state_t_ns].state()
    }
    pub fn get_t_ns(&self) -> i64 {
        self.get_state().state.pose.t_ns
    }
    pub fn get_t_w_i(&self) -> &Isometry3<S> {
        &self.get_state().state.pose.t_w_i
    }
    pub fn get_vel_w_i(&self) -> &Vector3<S> {
        &self.get_state().state.vel_w_i
    }
    pub fn get_state_at(&self, time: i64) -> PoseVelBiasState<S> {
        if let Some(state) = self.base.frame_states.get(&time) {
            return state.state().clone();
        }
        let mut state = PoseVelBiasState::new(
            0,
            Isometry3::identity(),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        if let Some(pose) = self.base.frame_poses.get(&time) {
            state.state.pose.t_w_i = *pose.pose();
        }
        state
    }
    pub fn get_frame_states(&self) -> Vec<Isometry3<S>> {
        self.base
            .frame_states
            .values()
            .map(|s| s.state().state.pose.t_w_i)
            .collect()
    }
    pub fn get_frame_poses(&self) -> Vec<Isometry3<S>> {
        self.base.frame_poses.values().map(|s| *s.pose()).collect()
    }
    pub fn get_all_poses_map(&self) -> BTreeMap<i64, Isometry3<S>> {
        self.base
            .frame_poses
            .iter()
            .map(|(&t, s)| (t, *s.pose()))
            .chain(
                self.base
                    .frame_states
                    .iter()
                    .map(|(&t, s)| (t, s.state().state.pose.t_w_i)),
            )
            .collect()
    }
    pub fn check_marg_nullspace(&self) -> DVector<f64> {
        let random = DVector::from_fn(self.nullspace_marg_data.order.total_size, |_, _| {
            rand::random::<f64>() * 2. - 1.
        });
        super::sqrt_ba_base::check_nullspace(
            &self.nullspace_marg_data,
            &self.base.frame_states,
            &self.base.frame_poses,
            &random,
        )
    }
    pub fn check_marg_eigenvalues(&self) -> DVector<f64> {
        super::sqrt_ba_base::check_eigenvalues(&self.nullspace_marg_data)
    }
    pub fn log_marg_nullspace(&mut self) {
        self.nullspace_marg_data.order = self.marg_data.order.clone();
        let nullspace = self.check_marg_nullspace();
        let eigenvalues = self.check_marg_eigenvalues();
        self.stats_sums.add_vector("marg_ns", nullspace.as_slice());
        self.stats_sums
            .add_vector("marg_ev", eigenvalues.as_slice());
    }
    /// # Errors
    /// Returns an error if the original statistics files cannot be written.
    pub fn debug_finalize(&self, directory: &std::path::Path) -> Result<()> {
        self.stats_all.print();
        self.stats_sums.print();
        self.stats_all.save_json(directory.join("stats_all.json"))?;
        self.stats_sums.save_json(directory.join("stats_sums.json"))
    }
    #[must_use]
    pub fn new(calibration: &Calibration<f64>, config: &VioConfig) -> Self {
        let mut prior = MargLinData {
            is_sqrt: config.vio_sqrt_marg,
            h: DMatrix::zeros(6, 6),
            b: DVector::zeros(6),
            order: AbsOrderMap::default(),
        };
        let nullspace_marg_data = prior.clone();
        let weight = scalar::<S>(config.vio_init_pose_weight);
        let weight = if prior.is_sqrt { weight.sqrt() } else { weight };
        for i in 0..6 {
            prior.h[(i, i)] = weight;
        }
        Self {
            base: BundleAdjustmentBase::new(
                calibration.cast(),
                scalar(config.vio_obs_std_dev),
                scalar(config.vio_obs_huber_thresh),
            ),
            marg_data: prior,
            nullspace_marg_data,
            config: config.clone(),
            take_kf: true,
            frames_after_kf: 0,
            kf_ids: BTreeSet::new(),
            last_state_t_ns: -1,
            prev_opt_flow_res: BTreeMap::new(),
            num_points_kf: BTreeMap::new(),
            max_states: config.vio_max_states as usize,
            max_kfs: config.vio_max_kfs as usize,
            t_w_i_init: Isometry3::identity(),
            initialized: false,
            lambda: scalar(config.vio_lm_lambda_initial),
            min_lambda: scalar(config.vio_lm_lambda_min),
            max_lambda: scalar(config.vio_lm_lambda_max),
            lambda_vee: scalar(2.),
            stats_all: ExecutionStats::default(),
            stats_sums: ExecutionStats::default(),
        }
    }
    fn initialize_pose(&mut self, time: i64, pose: Isometry3<S>) {
        self.initialized = true;
        self.t_w_i_init = pose;
        self.last_state_t_ns = time;
        self.base
            .frame_poses
            .insert(time, PoseStateWithLin::new(time, pose, true));
        add_order(&mut self.marg_data.order, time);
    }
    fn options(&self) -> Options<S> {
        Options {
            lb_options: crate::linearization::landmark_block::Options {
                huber_parameter: self.base.huber_thresh,
                obs_std_dev: self.base.obs_std_dev,
                ..Default::default()
            },
            linearization_type: self.config.vio_linearization_type,
        }
    }
    /// # Panics
    /// Requires initialized poses, synchronized observations and valid window topology.
    /// # Errors
    /// Propagates processing errors from the estimator pipeline.
    #[expect(
        clippy::too_many_lines,
        reason = "Preserves source measurement, triangulation and output phases"
    )]
    pub fn measure(
        &mut self,
        measurement: Arc<OpticalFlowResult>,
        add_pose: bool,
        outputs: &VioWorkerOutputs,
        status: &VioStatus,
    ) -> Result<bool> {
        let total = Timer::default();
        self.stats_sums
            .add("frame_id", measurement.t_ns as f64)
            .format("none");
        if add_pose {
            let pose = *self.base.frame_poses[&self.last_state_t_ns].pose();
            self.last_state_t_ns = measurement.t_ns;
            self.base.frame_poses.insert(
                measurement.t_ns,
                PoseStateWithLin::new(measurement.t_ns, pose, false),
            );
        }
        assert_eq!(measurement.t_ns, self.last_state_t_ns);
        assert_eq!(
            self.base.frame_poses.last_key_value().unwrap().0,
            &self.last_state_t_ns
        );
        self.prev_opt_flow_res
            .insert(measurement.t_ns, measurement.clone());
        let mut connected0 = 0;
        let mut connected = BTreeMap::new();
        let mut new_tracks = HashSet::new();
        let mut tracked = vec![Vec::new(); measurement.observations.len()];
        for (camera, observations) in measurement.observations.iter().enumerate() {
            let target = TimeCamId::new(measurement.t_ns, camera);
            for (&id, observation) in observations {
                if self.base.lmdb.landmark_exists(id as i32) {
                    let host = self.base.lmdb.landmark(id).host_kf_id;
                    self.base.lmdb.add_observation(
                        target,
                        &KeypointObservation {
                            kpt_id: id as i32,
                            pos: observation.translation.map(|v| scalar(f64::from(v))),
                        },
                    );
                    *connected.entry(host.frame_id).or_insert(0) += 1;
                    tracked[camera].push(id as i32);
                    if camera == 0 {
                        connected0 += 1;
                    }
                } else if camera == 0 {
                    new_tracks.insert(id);
                }
            }
        }
        if scalar::<S>(f64::from(connected0))
            / scalar::<S>((connected0 as usize + new_tracks.len()) as f64)
            < scalar(f64::from(self.config.vio_new_kf_keypoints_thresh))
            && self.frames_after_kf > self.config.vio_min_frames_after_kf
        {
            self.take_kf = true;
        }
        let mut pose = self.base.frame_poses[&self.last_state_t_ns].clone();
        self.base.optimize_single_frame_pose(&mut pose, &tracked);
        self.base.frame_poses.insert(self.last_state_t_ns, pose);
        if self.take_kf {
            self.take_kf = false;
            self.frames_after_kf = 0;
            self.kf_ids.insert(self.last_state_t_ns);
            let host = TimeCamId::new(measurement.t_ns, 0);
            let mut added = 0;
            for id in new_tracks {
                let mut observations = BTreeMap::new();
                for (&time, frame) in &self.prev_opt_flow_res {
                    for (camera, tracks) in frame.observations.iter().enumerate() {
                        if let Some(observation) = tracks.get(&id) {
                            observations.insert(
                                TimeCamId::new(time, camera),
                                KeypointObservation {
                                    kpt_id: id as i32,
                                    pos: observation.translation.map(|v| scalar(f64::from(v))),
                                },
                            );
                        }
                    }
                }
                let minimum = scalar::<S>(
                    self.config.vio_min_triangulation_dist * self.config.vio_min_triangulation_dist,
                );
                let mut valid = false;
                for (&target, observation) in &observations {
                    let origin = measurement.observations[0][&id]
                        .translation
                        .map(|v| scalar(f64::from(v)));
                    let mut ray0 = Vector4::zeros();
                    let mut ray1 = Vector4::zeros();
                    if !self.base.calib.intrinsics[0].unproject(&origin, &mut ray0, None)
                        || !self.base.calib.intrinsics[target.cam_id].unproject(
                            &observation.pos,
                            &mut ray1,
                            None,
                        )
                    {
                        continue;
                    }
                    let relative = self
                        .base
                        .get_pose_state_with_lin(host.frame_id)
                        .pose()
                        .inverse()
                        * self.base.get_pose_state_with_lin(target.frame_id).pose();
                    let transform = self.base.calib.t_i_c[0].inverse()
                        * relative
                        * self.base.calib.t_i_c[target.cam_id];
                    if transform.translation.vector.norm_squared() < minimum {
                        continue;
                    }
                    let point = BundleAdjustmentBase::triangulate(
                        &ray0.fixed_rows::<3>(0).into_owned(),
                        &ray1.fixed_rows::<3>(0).into_owned(),
                        &transform,
                    );
                    if point.iter().all(nalgebra::ComplexField::is_finite)
                        && point[3] > S::zero()
                        && point[3] < scalar(3.)
                    {
                        let direction = StereographicParam::project(&point, None);
                        self.base
                            .lmdb
                            .add_landmark(id, &Keypoint::new(direction, point[3], host));
                        added += 1;
                        valid = true;
                        break;
                    }
                }
                if valid {
                    for (target, observation) in observations {
                        self.base.lmdb.add_observation(target, &observation);
                    }
                }
            }
            self.num_points_kf.insert(measurement.t_ns, added);
        } else {
            self.frames_after_kf += 1;
        }
        let lost = if self.config.vio_marg_lost_landmarks {
            self.base
                .lmdb
                .landmarks()
                .keys()
                .filter(|id| !measurement.observations.iter().any(|o| o.contains_key(id)))
                .copied()
                .collect()
        } else {
            HashSet::new()
        };
        let previous = if outputs.local_map.is_some() {
            super::local_map::LocalMapSnapshot::observation_ids(&self.base)
        } else {
            BTreeMap::new()
        };
        self.optimize();
        if let Some(queue) = &outputs.local_map {
            let mut snapshot = super::local_map::LocalMapSnapshot::capture(
                self.last_state_t_ns,
                &self.base,
                &self.kf_ids,
                &self.prev_opt_flow_res,
            );
            snapshot.record_rejections(&previous);
            let _ = queue.send(Some(Arc::new(snapshot)));
        }
        self.marginalize(&connected, &lost, outputs);
        if outputs.state.is_some() || outputs.state_bounded.is_some() {
            let p = self.base.frame_poses[&self.last_state_t_ns].cast::<f64>();
            let state = PoseVelBiasState::new(
                p.t_ns(),
                *p.pose(),
                Vector3::zeros(),
                Vector3::zeros(),
                Vector3::zeros(),
            );
            outputs.send_state(Arc::new(state));
        }
        if let Some(queue) = &outputs.vis {
            let mut points = Vec::new();
            let mut point_ids = Vec::new();
            self.base
                .get_current_points::<f64>(&mut points, &mut point_ids);
            let mut projections = vec![Vec::new(); measurement.observations.len()];
            self.base
                .compute_projections(&mut projections, self.last_state_t_ns);
            let data = VioVisualizationData {
                t_ns: self.last_state_t_ns,
                states: Vec::new(),
                frames: self
                    .base
                    .frame_poses
                    .values()
                    .map(|p| *p.cast::<f64>().pose())
                    .collect(),
                points,
                point_ids,
                opt_flow_res: Some(measurement),
                projections,
            };
            let _ = queue.send(Some(Arc::new(data)));
        }
        status
            .last_processed_t_ns
            .store(self.last_state_t_ns, Ordering::Release);
        self.stats_sums.add("measure", total.elapsed()).format("ms");
        Ok(true)
    }
    /// # Panics
    /// Requires a pure-VO window retaining at least two keyframes and a consistent prior.
    #[expect(
        clippy::too_many_lines,
        reason = "Preserves original keyframe selection and prior transformation phases"
    )]
    pub fn marginalize(
        &mut self,
        connected: &BTreeMap<i64, i32>,
        lost: &HashSet<usize>,
        outputs: &VioWorkerOutputs,
    ) {
        let total = Timer::default();
        assert!(self.base.frame_states.is_empty());
        let non_keyframes: Vec<_> = self
            .base
            .frame_poses
            .keys()
            .filter(|&&t| t != self.last_state_t_ns && !self.kf_ids.contains(&t))
            .copied()
            .collect();
        for time in non_keyframes {
            self.base.frame_poses.remove(&time);
            self.base.lmdb.remove_frame(time);
            self.prev_opt_flow_res.remove(&time);
        }
        let all_keyframes = self.kf_ids.clone();
        let mut removed = BTreeSet::new();
        while self.kf_ids.len() > self.max_kfs {
            let candidates: Vec<_> = self
                .kf_ids
                .iter()
                .take(self.kf_ids.len().saturating_sub(2))
                .copied()
                .collect();
            let mut selected = candidates
                .iter()
                .find(|&&t| {
                    !connected.contains_key(&t)
                        || scalar::<S>(f64::from(connected[&t]))
                            / scalar::<S>(f64::from(self.num_points_kf[&t]))
                            < scalar(self.config.vio_kf_marg_feature_ratio)
                })
                .copied();
            if selected.is_none() {
                let newest = *self.kf_ids.last().unwrap();
                let mut best = scalar::<S>(f64::MAX);
                for &candidate in &candidates {
                    let position = self.base.frame_poses[&candidate].pose().translation.vector;
                    let denominator = candidates.iter().fold(S::zero(), |sum, t| {
                        sum + S::one()
                            / ((position - self.base.frame_poses[t].pose().translation.vector)
                                .norm()
                                + scalar(1e-5))
                    });
                    let score = (position
                        - self.base.frame_poses[&newest].pose().translation.vector)
                        .norm()
                        .sqrt()
                        * denominator;
                    if score < best {
                        best = score;
                        selected = Some(candidate);
                    }
                }
            }
            let selected = selected
                .expect("No keyframe candidate; maximum keyframes must preserve the newest two");
            removed.insert(selected);
            self.kf_ids.remove(&selected);
        }
        let mut order = self.marg_data.order.clone();
        for &time in self.base.frame_poses.keys() {
            if self.base.lmdb.observations().iter().any(|(host, targets)| {
                removed.contains(&host.frame_id)
                    && targets.keys().any(|target| target.frame_id == time)
            }) {
                add_order(&mut order, time);
            }
        }
        if self.config.vio_marg_lost_landmarks {
            for &id in lost {
                let landmark = self.base.lmdb.landmark(id);
                add_order(&mut order, landmark.host_kf_id.frame_id);
                for target in landmark.obs.keys() {
                    add_order(&mut order, target.frame_id);
                }
            }
        }
        let disconnected: Vec<_> = removed
            .iter()
            .filter(|t| !order.abs_order_map.contains_key(t))
            .copied()
            .collect();
        for time in disconnected {
            self.base.frame_poses.remove(&time);
            self.prev_opt_flow_res.remove(&time);
            self.base.lmdb.remove_keyframes(
                &BTreeSet::from([time]),
                &BTreeSet::new(),
                &BTreeSet::new(),
            );
            removed.remove(&time);
        }
        if !removed.is_empty() {
            assert!(all_keyframes.contains(&self.last_state_t_ns));
            let actual = Timer::default();
            let square_root =
                is_linearization_sqrt(self.config.vio_linearization_type) && self.marg_data.is_sqrt;
            let linearize_timer = Timer::default();
            let mut l = LinearizationBase::<S, 6>::create(
                &self.base,
                &order,
                self.options(),
                Some(&self.marg_data),
                None,
                Some(&removed),
                Some(lost),
                -1,
            );
            let _ = l.linearize_problem(&self.base);
            l.perform_qr();
            let (mut h, mut b) = if square_root {
                l.get_dense_q2jp_q2r(&self.base)
            } else {
                l.get_dense_h_b(&self.base)
            };
            drop(l);
            self.stats_sums
                .add("marg_linearize", linearize_timer.elapsed())
                .format("ms");
            if outputs.has_marginalization_output() {
                let (abs_h, abs_b) = if square_root {
                    (h.transpose() * &h, h.transpose() * &b)
                } else {
                    (h.clone(), b.clone())
                };
                let archive = MargData {
                    aom: order.clone(),
                    abs_h: abs_h.map(BasaltScalar::as_f64),
                    abs_b: abs_b.map(BasaltScalar::as_f64),
                    frame_states: self
                        .base
                        .frame_states
                        .iter()
                        .map(|(&t, s)| (t, s.cast()))
                        .collect(),
                    frame_poses: self
                        .base
                        .frame_poses
                        .iter()
                        .map(|(&t, s)| (t, s.cast()))
                        .collect(),
                    kfs_all: all_keyframes.clone(),
                    kfs_to_marg: removed.clone(),
                    use_imu: false,
                    opt_flow_res: all_keyframes
                        .iter()
                        .map(|t| self.prev_opt_flow_res[t].clone())
                        .collect(),
                };
                outputs.send_marginalization(Arc::new(archive));
            }
            let mut keep = BTreeSet::new();
            let mut eliminate = BTreeSet::new();
            for (&time, &(offset, size)) in &order.abs_order_map {
                assert_eq!(size, 6);
                let set = if removed.contains(&time) {
                    &mut eliminate
                } else {
                    &mut keep
                };
                set.extend(offset as usize..offset as usize + 6);
            }
            let reduce = |h: &mut DMatrix<S>, b: &mut DVector<S>| {
                if square_root {
                    MargHelper::marginalize_sqrt_to_sqrt(h, b, &keep, &eliminate)
                } else if self.marg_data.is_sqrt {
                    MargHelper::marginalize_sq_to_sqrt(h, b, &keep, &eliminate)
                } else {
                    MargHelper::marginalize_sq_to_sq(h, b, &keep, &eliminate)
                }
            };
            if self.config.vio_debug || self.config.vio_extended_logging {
                self.nullspace_marg_data.order = self.marg_data.order.clone();
                let mut l = LinearizationBase::<S, 6>::create(
                    &self.base,
                    &order,
                    self.options(),
                    Some(&self.nullspace_marg_data),
                    None,
                    Some(&removed),
                    Some(lost),
                    -1,
                );
                let _ = l.linearize_problem(&self.base);
                l.perform_qr();
                let (mut nh, mut nb) = if square_root {
                    l.get_dense_q2jp_q2r(&self.base)
                } else {
                    l.get_dense_h_b(&self.base)
                };
                drop(l);
                (self.nullspace_marg_data.h, self.nullspace_marg_data.b) = reduce(&mut nh, &mut nb);
            }
            let helper_timer = Timer::default();
            let (new_h, new_b) = reduce(&mut h, &mut b);
            self.stats_sums
                .add("marg_helper", helper_timer.elapsed())
                .format("ms");
            for (&time, state) in &mut self.base.frame_poses {
                if order.abs_order_map.contains_key(&time) && !state.is_linearized() {
                    state.set_lin_true();
                }
            }
            for time in &removed {
                self.base.frame_poses.remove(time);
                self.prev_opt_flow_res.remove(time);
            }
            self.base
                .lmdb
                .remove_keyframes(&removed, &removed, &removed);
            if self.config.vio_marg_lost_landmarks {
                for &id in lost {
                    self.base.lmdb.remove_landmark(id);
                }
            }
            let mut new_order = AbsOrderMap::default();
            for &time in self.base.frame_poses.keys() {
                if order.abs_order_map.contains_key(&time) {
                    add_order(&mut new_order, time);
                }
            }
            self.marg_data.h = new_h;
            self.marg_data.b = new_b;
            self.marg_data.order = new_order;
            assert_eq!(self.marg_data.h.ncols(), self.marg_data.order.total_size);
            let delta = self.base.compute_delta(&self.marg_data.order);
            self.marg_data.b -= &self.marg_data.h * &delta;
            if self.config.vio_debug || self.config.vio_extended_logging {
                self.nullspace_marg_data.b -= &self.nullspace_marg_data.h * &delta;
            }
            self.stats_sums
                .add("marg_total", actual.elapsed())
                .format("ms");
            if self.config.vio_debug || self.config.vio_extended_logging {
                let log_timer = Timer::default();
                self.log_marg_nullspace();
                self.stats_sums
                    .add("marg_log", log_timer.elapsed())
                    .format("ms");
            }
        }
        self.stats_sums
            .add("marginalize", total.elapsed())
            .format("ms");
    }
    /// # Panics
    /// Requires a pure-VO window and finite residual linearizations.
    #[expect(
        clippy::too_many_lines,
        reason = "Preserves the original nested LM acceptance and retry control flow"
    )]
    pub fn optimize(&mut self) {
        let total = Timer::default();
        let mut stats = ExecutionStats::default();
        let mut order = self.marg_data.order.clone();
        for &time in self.base.frame_poses.keys() {
            add_order(&mut order, time);
        }
        assert!(self.base.frame_states.is_empty());
        self.lambda = scalar(self.config.vio_lm_lambda_initial);
        stats
            .add("num_cams", self.base.frame_poses.len() as f64)
            .format("count");
        stats
            .add("num_lms", self.base.lmdb.num_landmarks() as f64)
            .format("count");
        stats
            .add("num_obs", self.base.lmdb.num_observations() as f64)
            .format("count");
        let mut timer = Timer::default();
        let mut linearization = LinearizationBase::<S, 6>::create(
            &self.base,
            &order,
            self.options(),
            Some(&self.marg_data),
            None,
            None,
            None,
            -1,
        );
        stats.add("allocateLMB", timer.reset()).format("ms");
        let mut iteration = 0;
        let mut rejected = 0;
        let mut terminated = false;
        while iteration <= self.config.vio_max_iterations && !terminated {
            let mut iteration_timer = Timer::default();
            let (error, valid) = linearization.linearize_problem(&self.base);
            assert!(valid, "numerical failure during linearization");
            stats.add("linearizeProblem", timer.reset()).format("ms");
            linearization.perform_qr();
            stats.add("performQR", timer.reset()).format("ms");
            while iteration <= self.config.vio_max_iterations && !terminated {
                let (h, b) = linearization.get_dense_h_b(&self.base);
                stats.add("get_dense_H_b", timer.reset()).format("ms");
                let mut increment = DVector::zeros(b.len());
                for _ in 0..3 {
                    let mut damped = h.clone();
                    for i in 0..h.nrows() {
                        damped[(i, i)] += (h[(i, i)] * self.lambda).max(self.min_lambda);
                    }
                    increment = Ldlt::new(damped).solve_vector(&b);
                    stats.add("solve", timer.reset()).format("ms");
                    if increment.iter().all(nalgebra::ComplexField::is_finite) {
                        break;
                    }
                    self.lambda *= self.lambda_vee;
                    self.lambda_vee *= scalar(2.);
                }
                self.base.backup();
                increment = -increment;
                let predicted = linearization.back_substitute(&mut self.base, &increment);
                stats.add("backSubstitute", timer.reset()).format("ms");
                for (&time, state) in &mut self.base.frame_poses {
                    state.apply_inc(
                        &increment
                            .fixed_rows::<6>(order.abs_order_map[&time].0 as usize)
                            .into_owned(),
                    );
                }
                let step = increment.amax();
                let after = self.base.compute_error(None, scalar(-1.))
                    + self.base.compute_marg_prior_error(&self.marg_data);
                stats.add("computerError2", timer.reset()).format("ms");
                let decrease = error - after;
                let ratio = decrease / predicted;
                stats.add("iteration", iteration_timer.reset()).format("ms");
                if predicted > S::zero() && ratio > S::zero() {
                    self.lambda *= scalar::<S>(1. / 3.)
                        .max(S::one() - (scalar::<S>(2.) * ratio - S::one()).powi(3));
                    self.lambda = self.min_lambda.max(self.lambda);
                    self.lambda_vee = scalar(2.);
                    iteration += 1;
                    if (decrease > S::zero() && decrease < scalar(1e-6)) || step < scalar(1e-4) {
                        terminated = true;
                    }
                    break;
                }
                self.lambda *= self.lambda_vee;
                self.lambda_vee *= scalar(2.);
                self.base.restore();
                iteration += 1;
                rejected += 1;
                if self.lambda > self.max_lambda {
                    terminated = true;
                }
            }
        }
        stats.add("optimize", total.elapsed()).format("ms");
        stats.add("num_it", f64::from(iteration)).format("count");
        stats
            .add("num_it_rejected", f64::from(rejected))
            .format("count");
        self.stats_all.merge_all(&stats);
        self.stats_sums.merge_sums(&stats);
    }
}
impl<S: BasaltScalar> VioEstimatorEngine for SqrtKeypointVoEstimator<S> {
    fn debug_finalize(&self, directory: &std::path::Path) -> Result<()> {
        Self::debug_finalize(self, directory)
    }
    fn get_state_at(&self, time: i64) -> PoseVelBiasState<f64> {
        Self::get_state_at(self, time).cast::<f64>()
    }
    fn get_all_poses_map(&self) -> BTreeMap<i64, Isometry3<f64>> {
        Self::get_all_poses_map(self)
            .into_iter()
            .map(|(time, pose)| {
                (
                    time,
                    PoseStateWithLin::new(time, pose, false)
                        .cast::<f64>()
                        .pose()
                        .to_owned(),
                )
            })
            .collect()
    }
    fn initialize_state(
        &mut self,
        time: i64,
        pose: Isometry3<f64>,
        _velocity: Vector3<f64>,
        _bg: Vector3<f64>,
        _ba: Vector3<f64>,
    ) {
        let p = PoseStateWithLin::new(time, pose, false).cast::<S>();
        self.initialize_pose(time, *p.pose());
    }
    fn initialize_bias(&mut self, _bg: Vector3<f64>, _ba: Vector3<f64>) {}
    fn run(
        &mut self,
        inputs: VioWorkerInputs,
        outputs: VioWorkerOutputs,
        status: Arc<VioStatus>,
    ) -> Result<()> {
        let mut previous = false;
        while let Ok(mut frame) = inputs.vision.recv() {
            if self.config.vio_enforce_realtime {
                while let Ok(next) = inputs.vision.try_recv() {
                    frame = next;
                }
            }
            let Some(frame) = frame else { break };
            for _ in inputs.imu.try_iter() {}
            self.max_states = status.max_states.load(Ordering::Acquire);
            self.max_kfs = status.max_kfs.load(Ordering::Acquire);
            if !self.initialized {
                self.initialize_pose(frame.t_ns, self.t_w_i_init);
                self.nullspace_marg_data.order = self.marg_data.order.clone();
                if self.config.vio_debug || self.config.vio_extended_logging {
                    self.log_marg_nullspace();
                }
            }
            *status.initial_pose.lock().map_err(|_| {
                Error::new(ErrorKind::Internal, "Initial pose mutex poisoned")
                    .with_operation("basalt.vo")
            })? = *PoseStateWithLin::new(frame.t_ns, self.t_w_i_init, false)
                .cast::<f64>()
                .pose();
            self.measure(frame, previous, &outputs, &status)?;
            previous = true;
        }
        Ok(())
    }
}
