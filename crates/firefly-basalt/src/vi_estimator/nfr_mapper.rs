// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are in ../../LICENSE.
//! Original Basalt `vi_estimator/nfr_mapper.h` and `src/vi_estimator/nfr_mapper.cpp`.
use super::{
    ba_base::BundleAdjustmentBase,
    landmark_database::{Keypoint, KeypointObservation},
    marg_helper::MargHelper,
    sc_ba_base,
};
use crate::{
    hash_bow::HashBow,
    optical_flow::OpticalFlowInput,
    optimization::accumulator::SparseHashAccumulator,
    utils::{
        common_types::{Corners, FeatureTracks, MatchData, Matches},
        full_piv_householder_qr::FullPivHouseholderQr,
        imu_types::{AbsOrderMap, MargData, PoseStateWithLin, RelPoseFactor, RollPitchFactor},
        keypoints,
        ldlt::Ldlt,
        nfr::{abs_position_error, rel_pose_error, roll_pitch_error, yaw_error},
        tracks::TrackBuilder,
        vio_config::VioConfig,
    },
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::stereographic_param::StereographicParam;
use firefly_calibration::observation::TimeCamId;
use firefly_error::Result;
use nalgebra::{DMatrix, SMatrix, Vector3, Vector4};
use rayon::prelude::*;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    sync::Arc,
};

pub struct NfrMapper {
    pub base: BundleAdjustmentBase<f64>,
    pub roll_pitch_factors: Vec<RollPitchFactor>,
    pub rel_pose_factors: Vec<RelPoseFactor>,
    pub img_data: HashMap<i64, Arc<OpticalFlowInput>>,
    pub feature_corners: Corners,
    pub feature_matches: Matches,
    pub feature_tracks: FeatureTracks,
    pub hash_bow_database: HashBow<256>,
    pub config: VioConfig,
    pub lambda: f64,
    pub min_lambda: f64,
    pub max_lambda: f64,
    pub lambda_vee: f64,
}
impl NfrMapper {
    #[must_use]
    pub fn new(calibration: &Calibration<f64>, config: &VioConfig) -> Self {
        Self {
            base: BundleAdjustmentBase::new(
                calibration.clone(),
                config.mapper_obs_std_dev,
                config.mapper_obs_huber_thresh,
            ),
            roll_pitch_factors: Vec::new(),
            rel_pose_factors: Vec::new(),
            img_data: HashMap::new(),
            feature_corners: Corners::default(),
            feature_matches: Matches::default(),
            feature_tracks: FeatureTracks::default(),
            hash_bow_database: HashBow::new(config.mapper_bow_num_bits as usize),
            config: config.clone(),
            lambda: config.mapper_lm_lambda_min,
            min_lambda: config.mapper_lm_lambda_min,
            max_lambda: config.mapper_lm_lambda_max,
            lambda_vee: 2.,
        }
    }
    pub fn add_marg_data(&mut self, data: &mut MargData) {
        self.process_marg_data(data);
        if self.extract_nonlinear_factors(data) {
            for (&time, state) in &data.frame_poses {
                self.base
                    .frame_poses
                    .insert(time, PoseStateWithLin::new(time, *state.pose(), false));
            }
            for (&time, state) in &data.frame_states {
                if data.kfs_all.contains(&time) {
                    self.base.frame_poses.insert(
                        time,
                        PoseStateWithLin::new(time, state.state().state.pose.t_w_i, false),
                    );
                }
            }
        }
    }
    /// # Panics
    /// Requires consistent state/order matrices and pose (6) or IMU (15) block sizes.
    pub fn process_marg_data(&mut self, data: &mut MargData) {
        assert_eq!(data.aom.total_size, data.abs_h.ncols());
        let mut order = AbsOrderMap::default();
        let (mut keep, mut remove) = (BTreeSet::new(), BTreeSet::new());
        for (&time, &(offset, size)) in &data.aom.abs_order_map {
            let offset = offset as usize;
            match size {
                6 => {
                    keep.extend(offset..offset + 6);
                    order.abs_order_map.insert(time, (offset as i32, 6));
                    order.total_size += 6;
                }
                15 => {
                    if data.kfs_all.contains(&time) {
                        keep.extend(offset..offset + 6);
                        remove.extend(offset + 6..offset + 15);
                        order
                            .abs_order_map
                            .insert(time, (order.total_size as i32, 6));
                        order.total_size += 6;
                        data.frame_poses.insert(
                            time,
                            PoseStateWithLin::from_imu_state(&data.frame_states[&time]),
                        );
                    } else {
                        remove.extend(offset..offset + 15);
                    }
                    data.frame_states.remove(&time);
                }
                _ => panic!("Unknown marginalization block size"),
            }
        }
        if !remove.is_empty() {
            (data.abs_h, data.abs_b) =
                MargHelper::marginalize_sq_to_sq(&mut data.abs_h, &mut data.abs_b, &keep, &remove);
            data.aom = order;
        }
        assert_eq!(data.aom.total_size, data.abs_h.ncols());
        for frame in &data.opt_flow_res {
            if let Some(images) = &frame.input_images {
                self.img_data.insert(frame.t_ns, images.clone());
            }
        }
    }
    /// # Panics
    /// Requires a marginalized keyframe, all referenced poses and invertible roll-pitch covariance.
    pub fn extract_nonlinear_factors(&mut self, data: &MargData) -> bool {
        let size = data.aom.total_size;
        let qr = FullPivHouseholderQr::new(data.abs_h.clone());
        if qr.rank() != data.abs_h.ncols() {
            return false;
        }
        let covariance = qr.solve(&DMatrix::identity(size, size));
        let keyframe = *data
            .kfs_to_marg
            .first()
            .expect("NFR needs a marginalized keyframe");
        let start = data.aom.abs_order_map[&keyframe].0 as usize;
        let pose = data.frame_poses[&keyframe].pose();
        let mut position_j = SMatrix::<f64, 3, 6>::zeros();
        let mut yaw_j = SMatrix::<f64, 1, 6>::zeros();
        let mut rp_j = SMatrix::<f64, 2, 6>::zeros();
        let _ = abs_position_error(pose, &pose.translation.vector, Some(&mut position_j));
        let _ = yaw_error(
            pose,
            &(pose.rotation.inverse() * Vector3::x()),
            Some(&mut yaw_j),
        );
        let _ = roll_pitch_error(pose, &pose.rotation, Some(&mut rp_j));
        let mut j = DMatrix::zeros(6, size);
        j.view_mut((0, start), (3, 6)).copy_from(&position_j);
        j.view_mut((3, start), (1, 6)).copy_from(&yaw_j);
        j.view_mut((4, start), (2, 6)).copy_from(&rp_j);
        let cov_new = &j * &covariance * j.transpose();
        let cov_inv = if self.config.mapper_no_factor_weights {
            SMatrix::<f64, 2, 2>::identity()
        } else {
            cov_new
                .fixed_view::<2, 2>(4, 4)
                .try_inverse()
                .expect("NFR roll-pitch covariance invertible")
        };
        if data.use_imu {
            self.roll_pitch_factors.push(RollPitchFactor {
                t_ns: keyframe,
                r_w_i_meas: pose.rotation,
                cov_inv,
            });
        }
        for &other in &data.kfs_all {
            if other == keyframe {
                continue;
            }
            let Some(other_state) = data.frame_poses.get(&other) else {
                continue;
            };
            let other_pose = other_state.pose();
            let measurement = pose.inverse() * other_pose;
            let (mut ji, mut jj) = (SMatrix::<f64, 6, 6>::zeros(), SMatrix::<f64, 6, 6>::zeros());
            let _ = rel_pose_error(&measurement, pose, other_pose, Some(&mut ji), Some(&mut jj));
            let mut j = DMatrix::zeros(6, size);
            j.view_mut((0, start), (6, 6)).copy_from(&ji);
            j.view_mut((0, data.aom.abs_order_map[&other].0 as usize), (6, 6))
                .copy_from(&jj);
            let cov_new = &j * &covariance * j.transpose();
            let cov_inv = if self.config.mapper_no_factor_weights {
                SMatrix::<f64, 6, 6>::identity()
            } else {
                Ldlt::new(cov_new)
                    .solve(&DMatrix::identity(6, 6))
                    .fixed_view::<6, 6>(0, 0)
                    .into_owned()
            };
            self.rel_pose_factors.push(RelPoseFactor {
                t_i_ns: keyframe,
                t_j_ns: other,
                t_i_j: measurement,
                cov_inv,
            });
        }
        true
    }
    pub fn get_frame_poses(&mut self) -> &mut BTreeMap<i64, PoseStateWithLin<f64>> {
        &mut self.base.frame_poses
    }
    pub fn compute_rel_pose(&self) -> f64 {
        self.rel_pose_factors
            .iter()
            .map(|factor| {
                let residual = rel_pose_error(
                    &factor.t_i_j,
                    self.base.frame_poses[&factor.t_i_ns].pose(),
                    self.base.frame_poses[&factor.t_j_ns].pose(),
                    None,
                    None,
                );
                residual.dot(&(factor.cov_inv * residual))
            })
            .sum()
    }
    pub fn compute_roll_pitch(&self) -> f64 {
        self.roll_pitch_factors
            .iter()
            .map(|factor| {
                let residual = roll_pitch_error(
                    self.base.frame_poses[&factor.t_ns].pose(),
                    &factor.r_w_i_meas,
                    None,
                );
                residual.dot(&(factor.cov_inv * residual))
            })
            .sum()
    }
    /// # Panics
    /// Requires non-linearized poses and valid factor and observation references.
    #[expect(
        clippy::too_many_lines,
        reason = "Retains the source factor assembly and nested LM acceptance loop"
    )]
    pub fn optimize(&mut self, iterations: usize) {
        let mut order = AbsOrderMap::default();
        for &time in self.base.frame_poses.keys() {
            order
                .abs_order_map
                .insert(time, (order.total_size as i32, 6));
            order.total_size += 6;
        }
        for _ in 0..iterations {
            let (data, vision_error) =
                sc_ba_base::linearize_helper(self.base.lmdb.observations(), &self.base);
            let empty = || {
                let mut value = SparseHashAccumulator::<f64>::default();
                value.reset(order.total_size);
                value
            };
            let mut accumulator = data
                .par_iter()
                .fold(empty, |mut accumulator, block| {
                    let (h, b) = sc_ba_base::linearize_rel(block);
                    sc_ba_base::linearize_rel_to_abs(&h, &b, &block.base, &order, &mut accumulator);
                    accumulator
                })
                .reduce(empty, |mut left, right| {
                    left.join(&right);
                    left
                });
            let mut factor_error = 0.;
            if self.config.mapper_use_factors {
                for factor in &self.roll_pitch_factors {
                    let mut j = SMatrix::<f64, 2, 6>::zeros();
                    let residual = roll_pitch_error(
                        self.base.frame_poses[&factor.t_ns].pose(),
                        &factor.r_w_i_meas,
                        Some(&mut j),
                    );
                    let offset = order.abs_order_map[&factor.t_ns].0 as usize;
                    accumulator.add_h(offset, offset, &(j.transpose() * factor.cov_inv * j));
                    accumulator.add_b(offset, &(j.transpose() * factor.cov_inv * residual));
                    factor_error += residual.dot(&(factor.cov_inv * residual));
                }
                for factor in &self.rel_pose_factors {
                    let (mut ji, mut jj) =
                        (SMatrix::<f64, 6, 6>::zeros(), SMatrix::<f64, 6, 6>::zeros());
                    let residual = rel_pose_error(
                        &factor.t_i_j,
                        self.base.frame_poses[&factor.t_i_ns].pose(),
                        self.base.frame_poses[&factor.t_j_ns].pose(),
                        Some(&mut ji),
                        Some(&mut jj),
                    );
                    let oi = order.abs_order_map[&factor.t_i_ns].0 as usize;
                    let oj = order.abs_order_map[&factor.t_j_ns].0 as usize;
                    for (offset, j) in [(oi, ji), (oj, jj)] {
                        for (other_offset, other_j) in [(oi, ji), (oj, jj)] {
                            accumulator.add_h(
                                offset,
                                other_offset,
                                &(j.transpose() * factor.cov_inv * other_j),
                            );
                        }
                        accumulator.add_b(offset, &(j.transpose() * factor.cov_inv * residual));
                    }
                    factor_error += residual.dot(&(factor.cov_inv * residual));
                }
            }
            accumulator.iterative_solver = true;
            accumulator.print_info = true;
            accumulator.setup_solver();
            let diagonal = accumulator.h_diagonal();
            let mut converged = false;
            let retries = if self.config.mapper_use_lm { 10 } else { 1 };
            for _ in 0..retries {
                let damping = if self.config.mapper_use_lm {
                    self.lambda
                } else {
                    self.min_lambda
                };
                let diagonal_lambda = diagonal.map(|value| (value * damping).max(self.min_lambda));
                let increment = accumulator.solve(Some(&diagonal_lambda));
                converged = increment.amax() < 1e-5;
                if self.config.mapper_use_lm {
                    self.base.backup();
                }
                for (&time, state) in &mut self.base.frame_poses {
                    assert!(!state.is_linearized());
                    state.apply_inc(
                        &(-increment
                            .fixed_rows::<6>(order.abs_order_map[&time].0 as usize)
                            .into_owned()),
                    );
                }
                for block in &data {
                    sc_ba_base::update_points(&order, block, &increment, &mut self.base.lmdb, None);
                }
                if !self.config.mapper_use_lm {
                    break;
                }
                let after = self.base.compute_error(None, -1.)
                    + if self.config.mapper_use_factors {
                        self.compute_rel_pose() + self.compute_roll_pitch()
                    } else {
                        0.
                    };
                if vision_error + factor_error - after < 0. {
                    self.lambda = self.max_lambda.min(self.lambda_vee * self.lambda);
                    self.lambda_vee *= 2.;
                    self.base.restore();
                } else {
                    self.lambda = self.min_lambda.max(self.lambda / 3.);
                    self.lambda_vee = 2.;
                    break;
                }
                if converged {
                    break;
                }
            }
            if converged {
                break;
            }
        }
    }
    /// # Panics
    /// Requires unpoisoned feature storage and calibration for every image.
    /// # Errors
    /// Returns image or corner detection errors.
    pub fn detect_keypoints(&mut self) -> Result<()> {
        let corners = self.feature_corners.read().expect("feature lock poisoned");
        let images: Vec<_> = self
            .img_data
            .iter()
            .filter(|(time, _)| self.base.frame_poses.contains_key(time))
            .collect();
        let processed: Result<Vec<_>> = images
            .par_iter()
            .map(|&(&time, input)| {
                let mut records = Vec::new();
                for (camera, image) in input.img_data.iter().enumerate() {
                    let id = TimeCamId {
                        frame_id: time,
                        cam_id: camera,
                    };
                    let mut data = corners.get(&id).cloned().unwrap_or_default();
                    let Some(image) = &image.img else {
                        records.push((id, data));
                        continue;
                    };
                    let image = image.view();
                    keypoints::detect_keypoints_mapping(
                        &image,
                        &mut data,
                        self.config.mapper_detection_num_points,
                    )?;
                    keypoints::compute_angles(&image, &mut data, true);
                    keypoints::compute_descriptors(&image, &mut data);
                    data.corners_3d = data
                        .corners
                        .iter()
                        .map(|pixel| {
                            let mut bearing = Vector4::zeros();
                            self.base.calib.intrinsics[camera].unproject(pixel, &mut bearing, None);
                            bearing
                        })
                        .collect();
                    self.hash_bow_database.compute_bow(
                        &data.corner_descriptors,
                        &mut data.hashes,
                        &mut data.bow_vector,
                    );
                    self.hash_bow_database.add_to_database(id, &data.bow_vector);
                    records.push((id, data));
                }
                Ok(records)
            })
            .collect();
        drop(corners);
        self.feature_corners
            .write()
            .expect("feature lock poisoned")
            .extend(processed?.into_iter().flatten());
        Ok(())
    }
    /// # Panics
    /// Requires two calibrated cameras and unpoisoned feature/match storage.
    pub fn match_stereo(&mut self) {
        let transform = self.base.calib.t_i_c[0].inverse() * self.base.calib.t_i_c[1];
        let essential = keypoints::compute_essential(&transform);
        let mut corners = self.feature_corners.write().expect("feature lock poisoned");
        let mut matches = self.feature_matches.write().expect("match lock poisoned");
        for &time in self.img_data.keys() {
            let (left, right) = (
                TimeCamId {
                    frame_id: time,
                    cam_id: 0,
                },
                TimeCamId {
                    frame_id: time,
                    cam_id: 1,
                },
            );
            corners.entry(left).or_default();
            corners.entry(right).or_default();
            let (a, b) = (&corners[&left], &corners[&right]);
            let mut data = MatchData {
                t_i_j: transform,
                ..Default::default()
            };
            keypoints::match_descriptors(
                &a.corner_descriptors,
                &b.corner_descriptors,
                &mut data.matches,
                self.config.mapper_max_hamming_distance as i32,
                self.config.mapper_second_best_test_ratio,
            );
            keypoints::find_inliers_essential(a, b, &essential, 1e-3, &mut data);
            if data.inliers.len() > 16 {
                matches.insert((left, right), data);
            }
        }
    }
    /// # Panics
    /// Requires unpoisoned storage and corner records for database entries.
    /// # Errors
    /// Returns geometric verification errors.
    pub fn match_all(&mut self) -> Result<()> {
        let corners = self.feature_corners.read().expect("feature lock poisoned");
        let records: Vec<_> = corners.iter().collect();
        let threshold = self.config.mapper_frames_to_match_threshold;
        let pairs: Vec<_> = records
            .par_iter()
            .flat_map_iter(|&(&id, data)| {
                let mut candidates = Vec::new();
                self.hash_bow_database.query_database(
                    &data.bow_vector,
                    self.config.mapper_num_frames_to_match as usize,
                    &mut candidates,
                    Some(id.frame_id),
                );
                candidates.into_iter().filter_map(move |(other, score)| {
                    (other.frame_id != id.frame_id && score > threshold).then_some((id, other))
                })
            })
            .collect();
        let matches: Result<Vec<_>> = pairs
            .par_iter()
            .map(|&(id, other)| {
                let data = &corners[&id];
                let candidate = &corners[&other];
                let mut pair = MatchData::default();
                keypoints::match_descriptors(
                    &data.corner_descriptors,
                    &candidate.corner_descriptors,
                    &mut pair.matches,
                    70,
                    1.2,
                );
                if pair.matches.len() > self.config.mapper_min_matches as usize {
                    keypoints::find_inliers_ransac(
                        data,
                        candidate,
                        self.config.mapper_ransac_threshold,
                        self.config.mapper_min_matches as usize,
                        &mut pair,
                    )?;
                }
                Ok((!pair.inliers.is_empty()).then_some(((id, other), pair)))
            })
            .collect();
        self.feature_matches
            .write()
            .expect("match lock poisoned")
            .extend(matches?.into_iter().flatten());
        Ok(())
    }
    pub fn build_tracks(&mut self) {
        let mut builder = TrackBuilder::default();
        builder.build(&self.feature_matches);
        builder.filter(self.config.mapper_min_track_length as usize);
        builder.export(&mut self.feature_tracks);
    }
    /// # Panics
    /// Requires unpoisoned storage and valid feature, pose and camera references.
    pub fn setup_opt(&mut self) {
        let minimum = self.config.mapper_min_triangulation_dist.powi(2);
        let corners = self.feature_corners.read().expect("feature lock poisoned");
        for (&id, track) in &self.feature_tracks {
            if track.len() < 2 {
                continue;
            }
            let (&host, &feature) = track.first_key_value().expect("nonempty track");
            let mut host_ray = Vector4::zeros();
            self.base.calib.intrinsics[host.cam_id].unproject(
                &corners[&host].corners[feature as usize],
                &mut host_ray,
                None,
            );
            for (&other, &feature) in track.iter().skip(1) {
                let mut ray = Vector4::zeros();
                self.base.calib.intrinsics[other.cam_id].unproject(
                    &corners[&other].corners[feature as usize],
                    &mut ray,
                    None,
                );
                let host_pose = self.base.frame_poses[&host.frame_id].pose()
                    * self.base.calib.t_i_c[host.cam_id];
                let other_pose = self.base.frame_poses[&other.frame_id].pose()
                    * self.base.calib.t_i_c[other.cam_id];
                let relative = host_pose.inverse() * other_pose;
                if relative.translation.vector.norm_squared() < minimum {
                    continue;
                }
                let point = BundleAdjustmentBase::<f64>::triangulate(
                    &host_ray.fixed_rows::<3>(0).into_owned(),
                    &ray.fixed_rows::<3>(0).into_owned(),
                    &relative,
                );
                if !point.iter().all(|v| v.is_finite()) || point.w <= 0. || point.w > 2. {
                    continue;
                }
                let direction = StereographicParam::<f64>::project(&point, None);
                self.base
                    .lmdb
                    .add_landmark(id as usize, &Keypoint::new(direction, point.w, host));
                for (&image, &feature) in track {
                    self.base.lmdb.add_observation(
                        image,
                        &KeypointObservation {
                            kpt_id: id as i32,
                            pos: corners[&image].corners[feature as usize],
                        },
                    );
                }
                break;
            }
        }
    }
}
