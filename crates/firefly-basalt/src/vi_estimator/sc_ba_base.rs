// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original Basalt `vi_estimator/sc_ba_base.h/.cpp`: host-grouped Schur blocks.
use super::{
    ba_base::BundleAdjustmentBase,
    landmark_database::{LandmarkDatabase, ObservationIndex},
};
use crate::{
    optimization::accumulator::BlockAccumulator,
    utils::{
        ba_utils::{compute_rel_pose, linearize_point},
        common_types::TimeCamId,
        imu_types::{AbsOrderMap, PoseStateWithLin, PoseVelBiasStateWithLin},
        ldlt::Ldlt,
    },
};
use firefly_basalt_headers::{
    imu::preintegration::IntegratedImuMeasurement,
    scalar::BasaltScalar,
    utils::sophus_utils::{hat, se3_matrix},
};
use nalgebra::{DMatrix, DVector, Matrix3, Matrix4, SMatrix, SVector, Vector2, Vector3};
use rayon::prelude::*;
use std::collections::{BTreeMap, HashMap};
type Mat6<S> = SMatrix<S, 6, 6>;
type Mat63<S> = SMatrix<S, 6, 3>;
type Vec6<S> = SVector<S, 6>;

#[derive(Clone, Debug)]
pub struct RelLinDataBase<S: BasaltScalar> {
    pub order: Vec<(TimeCamId, TimeCamId)>,
    pub d_rel_d_h: Vec<Mat6<S>>,
    pub d_rel_d_t: Vec<Mat6<S>>,
}
#[derive(Clone, Debug)]
pub struct FrameRelLinData<S: BasaltScalar> {
    pub hpp: Mat6<S>,
    pub bp: Vec6<S>,
    pub lm_id: Vec<usize>,
    pub hpl: Vec<Mat63<S>>,
}
impl<S: BasaltScalar> Default for FrameRelLinData<S> {
    fn default() -> Self {
        Self {
            hpp: Mat6::zeros(),
            bp: Vec6::zeros(),
            lm_id: Vec::new(),
            hpl: Vec::new(),
        }
    }
}
#[derive(Clone, Debug)]
pub struct RelLinData<S: BasaltScalar> {
    pub base: RelLinDataBase<S>,
    pub hll: HashMap<usize, Matrix3<S>>,
    pub hll_inv: HashMap<usize, Matrix3<S>>,
    pub bl: HashMap<usize, Vector3<S>>,
    pub lm_to_obs: HashMap<usize, Vec<(usize, usize)>>,
    pub hpppl: Vec<FrameRelLinData<S>>,
    pub error: S,
}
impl<S: BasaltScalar> RelLinData<S> {
    #[must_use]
    pub fn new(num_keypoints: usize, num_poses: usize) -> Self {
        Self {
            base: RelLinDataBase {
                order: Vec::with_capacity(num_poses),
                d_rel_d_h: Vec::with_capacity(num_poses),
                d_rel_d_t: Vec::with_capacity(num_poses),
            },
            hll: HashMap::with_capacity(num_keypoints),
            hll_inv: HashMap::with_capacity(num_keypoints),
            bl: HashMap::with_capacity(num_keypoints),
            lm_to_obs: HashMap::with_capacity(num_keypoints),
            hpppl: Vec::with_capacity(num_poses),
            error: S::zero(),
        }
    }
    pub fn invert_keypoint_hessians(&mut self) {
        invert_hessians(&self.hll, &mut self.hll_inv);
    }
}
#[derive(Clone, Debug)]
pub struct FrameAbsLinData<S: BasaltScalar> {
    pub hphph: Mat6<S>,
    pub bph: Vec6<S>,
    pub hptpt: Mat6<S>,
    pub bpt: Vec6<S>,
    pub hphpt: Mat6<S>,
    pub lm_id: Vec<usize>,
    pub hphl: Vec<Mat63<S>>,
    pub hptl: Vec<Mat63<S>>,
}
impl<S: BasaltScalar> Default for FrameAbsLinData<S> {
    fn default() -> Self {
        Self {
            hphph: Mat6::zeros(),
            bph: Vec6::zeros(),
            hptpt: Mat6::zeros(),
            bpt: Vec6::zeros(),
            hphpt: Mat6::zeros(),
            lm_id: Vec::new(),
            hphl: Vec::new(),
            hptl: Vec::new(),
        }
    }
}
#[derive(Clone, Debug)]
pub struct AbsLinData<S: BasaltScalar> {
    pub order: Vec<(TimeCamId, TimeCamId)>,
    pub hll: HashMap<usize, Matrix3<S>>,
    pub hll_inv: HashMap<usize, Matrix3<S>>,
    pub bl: HashMap<usize, Vector3<S>>,
    pub lm_to_obs: HashMap<usize, Vec<(usize, usize)>>,
    pub hpppl: Vec<FrameAbsLinData<S>>,
    pub error: S,
}
impl<S: BasaltScalar> AbsLinData<S> {
    #[must_use]
    pub fn new(num_keypoints: usize, num_poses: usize) -> Self {
        Self {
            order: Vec::with_capacity(num_poses),
            hll: HashMap::with_capacity(num_keypoints),
            hll_inv: HashMap::with_capacity(num_keypoints),
            bl: HashMap::with_capacity(num_keypoints),
            lm_to_obs: HashMap::with_capacity(num_keypoints),
            hpppl: Vec::with_capacity(num_poses),
            error: S::zero(),
        }
    }
    pub fn invert_keypoint_hessians(&mut self) {
        invert_hessians(&self.hll, &mut self.hll_inv);
    }
}
fn invert_hessians<S: BasaltScalar>(
    input: &HashMap<usize, Matrix3<S>>,
    output: &mut HashMap<usize, Matrix3<S>>,
) {
    for (&id, hessian) in input {
        let inverse = Ldlt::new(DMatrix::from_column_slice(3, 3, hessian.as_slice()))
            .solve(&DMatrix::identity(3, 3));
        output.insert(id, Matrix3::from_column_slice(inverse.as_slice()));
    }
}
fn offset(order: &AbsOrderMap, time: i64) -> usize {
    usize::try_from(order.abs_order_map[&time].0).expect("nonnegative state offset")
}
fn relative_pose<S: BasaltScalar>(
    estimator: &BundleAdjustmentBase<S>,
    host: TimeCamId,
    target: TimeCamId,
) -> (Matrix4<S>, Mat6<S>, Mat6<S>) {
    if host == target {
        return (Matrix4::identity(), Mat6::zeros(), Mat6::zeros());
    }
    let h = estimator.get_pose_state_with_lin(host.frame_id);
    let t = estimator.get_pose_state_with_lin(target.frame_id);
    let mut dh = Mat6::zeros();
    let mut dt = Mat6::zeros();
    let mut pose = compute_rel_pose(
        h.pose_lin(),
        &estimator.calib.t_i_c[host.cam_id],
        t.pose_lin(),
        &estimator.calib.t_i_c[target.cam_id],
        Some(&mut dh),
        Some(&mut dt),
    );
    if h.is_linearized() || t.is_linearized() {
        pose = compute_rel_pose(
            h.pose(),
            &estimator.calib.t_i_c[host.cam_id],
            t.pose(),
            &estimator.calib.t_i_c[target.cam_id],
            None,
            None,
        );
    }
    (se3_matrix(&pose), dh, dt)
}

/// Group observations by host, retaining the upstream relative-pose block layout.
pub fn linearize_helper<S: BasaltScalar>(
    observations: &ObservationIndex,
    estimator: &BundleAdjustmentBase<S>,
) -> (Vec<RelLinData<S>>, S) {
    let data: Vec<_> = observations
        .par_iter()
        .map(|(&host, targets)| {
            let mut result = RelLinData::new(estimator.lmdb.num_landmarks(), targets.len());
            for (&target, ids) in targets {
                let (transform, dh, dt) = relative_pose(estimator, host, target);
                result.base.order.push((host, target));
                result.base.d_rel_d_h.push(dh);
                result.base.d_rel_d_t.push(dt);
                let mut frame = FrameRelLinData::default();
                for &id in ids {
                    let point = estimator.lmdb.landmark(id);
                    let mut residual = Vector2::zeros();
                    let mut jp = SMatrix::<S, 2, 6>::zeros();
                    let mut jl = SMatrix::<S, 2, 3>::zeros();
                    if linearize_point(
                        &point.obs[&target],
                        point,
                        &transform,
                        &estimator.calib.intrinsics[target.cam_id],
                        &mut residual,
                        Some(&mut jp),
                        Some(&mut jl),
                        None,
                    ) {
                        let norm = residual.norm();
                        let robust = if norm < estimator.huber_thresh {
                            S::one()
                        } else {
                            estimator.huber_thresh / norm
                        };
                        let weight = robust / (estimator.obs_std_dev * estimator.obs_std_dev);
                        result.error += S::from_f64_lossy(0.5)
                            * (S::from_f64_lossy(2.) - robust)
                            * weight
                            * residual.norm_squared();
                        *result.hll.entry(id).or_insert_with(Matrix3::zeros) +=
                            jl.transpose() * weight * jl;
                        *result.bl.entry(id).or_insert_with(Vector3::zeros) +=
                            jl.transpose() * weight * residual;
                        frame.hpp += jp.transpose() * weight * jp;
                        frame.bp += jp.transpose() * weight * residual;
                        frame.hpl.push(jp.transpose() * weight * jl);
                        frame.lm_id.push(id);
                        result
                            .lm_to_obs
                            .entry(id)
                            .or_default()
                            .push((result.hpppl.len(), frame.lm_id.len() - 1));
                    }
                }
                result.hpppl.push(frame);
            }
            result.invert_keypoint_hessians();
            result
        })
        .collect();
    let error = data.iter().fold(S::zero(), |sum, entry| sum + entry.error);
    (data, error)
}

pub fn linearize_helper_abs<S: BasaltScalar>(
    observations: &ObservationIndex,
    estimator: &BundleAdjustmentBase<S>,
) -> (Vec<AbsLinData<S>>, S) {
    let data: Vec<_> = observations
        .par_iter()
        .map(|(&host, targets)| {
            let mut result = AbsLinData::new(estimator.lmdb.num_landmarks(), targets.len());
            for (&target, ids) in targets {
                let (transform, dh, dt) = relative_pose(estimator, host, target);
                result.order.push((host, target));
                let mut frame = FrameAbsLinData::default();
                for &id in ids {
                    let point = estimator.lmdb.landmark(id);
                    let mut residual = Vector2::zeros();
                    let mut jp = SMatrix::<S, 2, 6>::zeros();
                    let mut jl = SMatrix::<S, 2, 3>::zeros();
                    if linearize_point(
                        &point.obs[&target],
                        point,
                        &transform,
                        &estimator.calib.intrinsics[target.cam_id],
                        &mut residual,
                        Some(&mut jp),
                        Some(&mut jl),
                        None,
                    ) {
                        let jh = jp * dh;
                        let jt = jp * dt;
                        let norm = residual.norm();
                        let robust = if norm < estimator.huber_thresh {
                            S::one()
                        } else {
                            estimator.huber_thresh / norm
                        };
                        let weight = robust / (estimator.obs_std_dev * estimator.obs_std_dev);
                        result.error += S::from_f64_lossy(0.5)
                            * (S::from_f64_lossy(2.) - robust)
                            * weight
                            * residual.norm_squared();
                        *result.hll.entry(id).or_insert_with(Matrix3::zeros) +=
                            jl.transpose() * weight * jl;
                        *result.bl.entry(id).or_insert_with(Vector3::zeros) +=
                            jl.transpose() * weight * residual;
                        frame.hphph += jh.transpose() * weight * jh;
                        frame.hptpt += jt.transpose() * weight * jt;
                        frame.hphpt += jh.transpose() * weight * jt;
                        frame.bph += jh.transpose() * weight * residual;
                        frame.bpt += jt.transpose() * weight * residual;
                        frame.hphl.push(jh.transpose() * weight * jl);
                        frame.hptl.push(jt.transpose() * weight * jl);
                        frame.lm_id.push(id);
                        result
                            .lm_to_obs
                            .entry(id)
                            .or_default()
                            .push((result.hpppl.len(), frame.lm_id.len() - 1));
                    }
                }
                result.hpppl.push(frame);
            }
            result.invert_keypoint_hessians();
            result
        })
        .collect();
    let error = data.iter().fold(S::zero(), |sum, entry| sum + entry.error);
    (data, error)
}

#[must_use]
pub fn linearize_rel<S: BasaltScalar>(data: &RelLinData<S>) -> (DMatrix<S>, DVector<S>) {
    let size = data.base.order.len() * 6;
    let mut h = DMatrix::zeros(size, size);
    let mut b = DVector::zeros(size);
    for (i, frame) in data.hpppl.iter().enumerate() {
        for row in 0..6 {
            b[i * 6 + row] += frame.bp[row];
            for col in 0..6 {
                h[(i * 6 + row, i * 6 + col)] += frame.hpp[(row, col)];
            }
        }
        for (j, &id) in frame.lm_id.iter().enumerate() {
            let cross = frame.hpl[j] * data.hll_inv[&id];
            let rhs = cross * data.bl[&id];
            for row in 0..6 {
                b[i * 6 + row] -= rhs[row];
            }
            for &(other, landmark) in &data.lm_to_obs[&id] {
                let reduced = cross * data.hpppl[other].hpl[landmark].transpose();
                for row in 0..6 {
                    for col in 0..6 {
                        h[(i * 6 + row, other * 6 + col)] -= reduced[(row, col)];
                    }
                }
            }
        }
    }
    (h, b)
}

/// # Panics
/// Requires all relative blocks to share their host and a complete state ordering.
pub fn linearize_rel_to_abs<S: BasaltScalar>(
    hessian: &DMatrix<S>,
    rhs: &DVector<S>,
    data: &RelLinDataBase<S>,
    order: &AbsOrderMap,
    accumulator: &mut impl BlockAccumulator<S>,
) {
    for (i, &(host, target)) in data.order.iter().enumerate() {
        let hi = offset(order, host.frame_id);
        let ti = offset(order, target.frame_id);
        let b = rhs.fixed_rows::<6>(i * 6);
        accumulator.add_b(hi, &(data.d_rel_d_h[i].transpose() * b));
        accumulator.add_b(ti, &(data.d_rel_d_t[i].transpose() * b));
        for (j, &(other_host, other_target)) in data.order.iter().enumerate() {
            assert_eq!(host, other_host);
            if host.frame_id == target.frame_id || host.frame_id == other_target.frame_id {
                continue;
            }
            let tj = offset(order, other_target.frame_id);
            let h = hessian.fixed_view::<6, 6>(i * 6, j * 6);
            accumulator.add_h(
                hi,
                hi,
                &(data.d_rel_d_h[i].transpose() * h * data.d_rel_d_h[j]),
            );
            accumulator.add_h(
                ti,
                hi,
                &(data.d_rel_d_t[i].transpose() * h * data.d_rel_d_h[j]),
            );
            accumulator.add_h(
                hi,
                tj,
                &(data.d_rel_d_h[i].transpose() * h * data.d_rel_d_t[j]),
            );
            accumulator.add_h(
                ti,
                tj,
                &(data.d_rel_d_t[i].transpose() * h * data.d_rel_d_t[j]),
            );
        }
    }
}

/// # Panics
/// Requires landmark observations to share their host and a complete state ordering.
pub fn linearize_abs<S: BasaltScalar>(
    data: &AbsLinData<S>,
    order: &AbsOrderMap,
    accumulator: &mut impl BlockAccumulator<S>,
) {
    for (i, &(host, target)) in data.order.iter().enumerate() {
        let hi = offset(order, host.frame_id);
        let ti = offset(order, target.frame_id);
        let frame = &data.hpppl[i];
        accumulator.add_h(hi, hi, &frame.hphph);
        accumulator.add_h(ti, ti, &frame.hptpt);
        accumulator.add_h(hi, ti, &frame.hphpt);
        accumulator.add_h(ti, hi, &frame.hphpt.transpose());
        accumulator.add_b(hi, &frame.bph);
        accumulator.add_b(ti, &frame.bpt);
        for (j, &id) in frame.lm_id.iter().enumerate() {
            let host_cross = frame.hphl[j] * data.hll_inv[&id];
            let target_cross = frame.hptl[j] * data.hll_inv[&id];
            accumulator.add_b(hi, &(-host_cross * data.bl[&id]));
            accumulator.add_b(ti, &(-target_cross * data.bl[&id]));
            for &(other, landmark) in &data.lm_to_obs[&id] {
                assert_eq!(host, data.order[other].0);
                let tj = offset(order, data.order[other].1.frame_id);
                let other_frame = &data.hpppl[other];
                let h = other_frame.hphl[landmark].transpose();
                let t = other_frame.hptl[landmark].transpose();
                accumulator.add_h(hi, hi, &(-host_cross * h));
                accumulator.add_h(ti, hi, &(-target_cross * h));
                accumulator.add_h(hi, tj, &(-host_cross * t));
                accumulator.add_h(ti, tj, &(-target_cross * t));
            }
        }
    }
}

pub fn update_points<S: BasaltScalar>(
    order: &AbsOrderMap,
    data: &RelLinData<S>,
    increment: &DVector<S>,
    landmarks: &mut LandmarkDatabase<S>,
    mut decrease: Option<&mut S>,
) {
    let half = S::from_f64_lossy(0.5);
    let mut relative = DVector::zeros(data.base.order.len() * 6);
    for (i, &(host, target)) in data.base.order.iter().enumerate() {
        if host.frame_id == target.frame_id {
            continue;
        }
        let inc = data.base.d_rel_d_h[i] * increment.fixed_rows::<6>(offset(order, host.frame_id))
            + data.base.d_rel_d_t[i] * increment.fixed_rows::<6>(offset(order, target.frame_id));
        relative.rows_mut(i * 6, 6).copy_from(&inc);
        if let Some(value) = decrease.as_deref_mut() {
            let frame = &data.hpppl[i];
            *value += inc.dot(&(frame.bp - frame.hpp * inc * half));
        }
    }
    for (&id, observations) in &data.lm_to_obs {
        let mut cross = Vector3::zeros();
        for &(frame, landmark) in observations {
            cross +=
                data.hpppl[frame].hpl[landmark].transpose() * relative.fixed_rows::<6>(frame * 6);
        }
        let inc = -(data.hll_inv[&id] * (data.bl[&id] - cross));
        let point = landmarks.landmark_mut(id);
        point.direction += inc.fixed_rows::<2>(0);
        let distance = point.inv_dist + inc[2];
        point.inv_dist = if distance > S::zero() {
            distance
        } else {
            S::zero()
        };
        if let Some(value) = decrease.as_deref_mut() {
            *value -= inc.dot(&(data.bl[&id] + data.hll[&id] * inc * half - cross));
        }
    }
}

pub fn update_points_abs<S: BasaltScalar>(
    order: &AbsOrderMap,
    data: &AbsLinData<S>,
    increment: &DVector<S>,
    landmarks: &mut LandmarkDatabase<S>,
    mut decrease: Option<&mut S>,
) {
    let half = S::from_f64_lossy(0.5);
    if let Some(value) = decrease.as_deref_mut() {
        for (i, &(host, target)) in data.order.iter().enumerate() {
            let h = increment.fixed_rows::<6>(offset(order, host.frame_id));
            let t = increment.fixed_rows::<6>(offset(order, target.frame_id));
            let frame = &data.hpppl[i];
            *value += h.dot(&(frame.bph - frame.hphph * h * half - frame.hphpt * t))
                + t.dot(&(frame.bpt - frame.hptpt * t * half));
        }
    }
    for (&id, observations) in &data.lm_to_obs {
        let mut cross = Vector3::zeros();
        for &(frame, landmark) in observations {
            let (host, target) = data.order[frame];
            cross += data.hpppl[frame].hphl[landmark].transpose()
                * increment.fixed_rows::<6>(offset(order, host.frame_id));
            cross += data.hpppl[frame].hptl[landmark].transpose()
                * increment.fixed_rows::<6>(offset(order, target.frame_id));
        }
        let inc = -(data.hll_inv[&id] * (data.bl[&id] - cross));
        let point = landmarks.landmark_mut(id);
        point.direction += inc.fixed_rows::<2>(0);
        let distance = point.inv_dist + inc[2];
        point.inv_dist = if distance > S::zero() {
            distance
        } else {
            S::zero()
        };
        if let Some(value) = decrease.as_deref_mut() {
            *value -= inc.dot(&(data.bl[&id] + data.hll[&id] * inc * half - cross));
        }
    }
}

/// # Panics
/// Requires representable IMU endpoint timestamps and both endpoint states.
pub fn compute_imu_error<S: BasaltScalar>(
    order: &AbsOrderMap,
    states: &BTreeMap<i64, PoseVelBiasStateWithLin<S>>,
    measurements: &BTreeMap<i64, IntegratedImuMeasurement<S>>,
    gyro_weight: &Vector3<S>,
    accel_weight: &Vector3<S>,
    gravity: &Vector3<S>,
) -> (S, S, S) {
    let (mut imu_error, mut gyro_error, mut accel_error) = (S::zero(), S::zero(), S::zero());
    let half = S::from_f64_lossy(0.5);
    for measurement in measurements.values() {
        if measurement.dt_ns() == 0 {
            continue;
        }
        let start = measurement.start_t_ns();
        let end = start
            .checked_add(measurement.dt_ns())
            .expect("IMU timestamp overflow");
        if !order.abs_order_map.contains_key(&start) || !order.abs_order_map.contains_key(&end) {
            continue;
        }
        let initial = states[&start].state();
        let next = states[&end].state();
        let residual = measurement.residual(
            &initial.state,
            gravity,
            &next.state,
            &initial.bias_gyro,
            &initial.bias_accel,
            None,
            None,
            None,
            None,
        );
        imu_error += half * residual.dot(&(measurement.cov_inv() * residual));
        let dt = S::from_i64_lossy(measurement.dt_ns()) * S::from_f64_lossy(1e-9);
        let gyro = initial.bias_gyro - next.bias_gyro;
        let accel = initial.bias_accel - next.bias_accel;
        gyro_error += half * gyro.dot(&(gyro_weight / dt).component_mul(&gyro));
        accel_error += half * accel.dot(&(accel_weight / dt).component_mul(&accel));
    }
    (imu_error, gyro_error, accel_error)
}

/// Nullspace diagnostic directions, with the upstream random direction supplied explicitly.
/// # Panics
/// Requires state sizes six or fifteen and nonnegative ordering offsets.
pub fn nullspace_increments<S: BasaltScalar>(
    order: &AbsOrderMap,
    states: &BTreeMap<i64, PoseVelBiasStateWithLin<S>>,
    poses: &BTreeMap<i64, PoseStateWithLin<S>>,
    random: &DVector<f64>,
) -> [DVector<f64>; 7] {
    let mut result = std::array::from_fn(|_| DVector::zeros(order.total_size));
    let mut mean = Vector3::<f64>::zeros();
    for (&time, &(_, size)) in &order.abs_order_map {
        mean += match size {
            6 => poses[&time]
                .pose_lin()
                .translation
                .vector
                .map(BasaltScalar::as_f64),
            15 => states[&time]
                .state_lin()
                .state
                .pose
                .t_w_i
                .translation
                .vector
                .map(BasaltScalar::as_f64),
            _ => panic!("unknown state size"),
        };
    }
    mean /= order.abs_order_map.len() as f64;
    for (&time, &(index, size)) in &order.abs_order_map {
        let index = usize::try_from(index).unwrap();
        for axis in 0..3 {
            result[axis][index + axis] = 0.01;
            result[axis + 3][index + axis + 3] = 0.01;
        }
        let translation = if size == 6 {
            poses[&time]
                .pose_lin()
                .translation
                .vector
                .map(BasaltScalar::as_f64)
        } else {
            states[&time]
                .state_lin()
                .state
                .pose
                .t_w_i
                .translation
                .vector
                .map(BasaltScalar::as_f64)
        };
        let derivative = -hat(&(translation - mean)) * 0.01;
        for axis in 0..3 {
            result[axis + 3]
                .rows_mut(index, 3)
                .copy_from(&derivative.column(axis));
        }
        if size == 15 {
            let velocity = states[&time]
                .state_lin()
                .state
                .vel_w_i
                .map(BasaltScalar::as_f64);
            let derivative = -hat(&velocity) * 0.01;
            for axis in 0..3 {
                result[axis + 3]
                    .rows_mut(index + 6, 3)
                    .copy_from(&derivative.column(axis));
            }
        }
    }
    result[6] = random.clone();
    for direction in &mut result {
        direction.normalize_mut();
    }
    result
}
/// # Panics
/// Requires the Hessian dimension to equal the ordering size.
pub fn check_nullspace<S: BasaltScalar>(
    hessian: &DMatrix<S>,
    rhs: &DVector<S>,
    order: &AbsOrderMap,
    states: &BTreeMap<i64, PoseVelBiasStateWithLin<S>>,
    poses: &BTreeMap<i64, PoseStateWithLin<S>>,
    random: &DVector<f64>,
) -> DVector<f64> {
    assert_eq!(hessian.ncols(), order.total_size);
    let h = hessian.map(BasaltScalar::as_f64);
    let b = rhs.map(BasaltScalar::as_f64);
    let directions = nullspace_increments(order, states, poses, random);
    DVector::from_fn(7, |i, _| {
        directions[i].dot(&(&h * &directions[i])) + directions[i].dot(&b)
    })
}
pub fn check_eigenvalues<S: BasaltScalar>(hessian: &DMatrix<S>) -> DVector<f64> {
    let mut eigenvalues = hessian
        .map(BasaltScalar::as_f64)
        .symmetric_eigen()
        .eigenvalues;
    eigenvalues.as_mut_slice().sort_by(f64::total_cmp);
    eigenvalues
}
