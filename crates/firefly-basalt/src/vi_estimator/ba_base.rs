// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// See the crate LICENSE-BSD-3-Clause.
//! Original Basalt `vi_estimator/ba_base.h` and `ba_base.cpp`, revision
//! `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

use super::landmark_database::LandmarkDatabase;
use crate::utils::{
    ba_utils::{compute_rel_pose, linearize_point},
    common_types::TimeCamId,
    imu_types::{AbsOrderMap, MargLinData, PoseStateWithLin, PoseVelBiasStateWithLin},
};
use firefly_basalt_headers::{
    calibration::Calibration,
    camera::stereographic_param::StereographicParam,
    scalar::BasaltScalar,
    utils::sophus_utils::{so3_inverse, so3_product},
};
use nalgebra::{
    DMatrix, DVector, Isometry3, Matrix4, SMatrix, SVector, Translation3, Vector2, Vector3, Vector4,
};
use std::collections::{BTreeMap, BTreeSet};

pub type Outliers<S> = BTreeMap<usize, Vec<(TimeCamId, S)>>;

#[derive(Clone, Debug)]
pub struct BundleAdjustmentBase<S: BasaltScalar> {
    pub frame_states: BTreeMap<i64, PoseVelBiasStateWithLin<S>>,
    pub frame_poses: BTreeMap<i64, PoseStateWithLin<S>>,
    pub lmdb: LandmarkDatabase<S>,
    pub obs_std_dev: S,
    pub huber_thresh: S,
    pub calib: Calibration<S>,
}

// Fixed real specializations of Eigen JacobiSVD / RealSvd2x2 / LDLT.
// Copyright (C) 2008-2016 Gael Guennebaud, Benoit Jacob, Keir Mierle,
// and Timothy E. Holy. These helper implementations are MPL-2.0:
// https://mozilla.org/MPL/2.0/ .

#[expect(
    clippy::many_single_char_names,
    reason = "Retains Eigen Jacobi rotation notation"
)]
fn rotate_left<S: BasaltScalar>(a: &mut Matrix4<S>, p: usize, q: usize, c: S, s: S) {
    for j in 0..4 {
        let x = a[(p, j)];
        let y = a[(q, j)];
        a[(p, j)] = c * x + s * y;
        a[(q, j)] = -s * x + c * y;
    }
}
#[expect(
    clippy::many_single_char_names,
    reason = "Retains Eigen Jacobi rotation notation"
)]
fn rotate_right<S: BasaltScalar>(a: &mut Matrix4<S>, p: usize, q: usize, c: S, s: S) {
    for i in 0..4 {
        let x = a[(i, p)];
        let y = a[(i, q)];
        a[(i, p)] = c * x - s * y;
        a[(i, q)] = s * x + c * y;
    }
}

#[expect(
    clippy::many_single_char_names,
    reason = "Mechanical Eigen two-sided Jacobi SVD specialization"
)]
fn jacobi_smallest_vector<S: BasaltScalar>(mut a: Matrix4<S>) -> Vector4<S> {
    let mut scale = S::zero();
    for value in a.iter() {
        if !value.is_finite() {
            return Vector4::repeat(S::from_f64_lossy(f64::NAN));
        }
        scale = scale.max(value.abs());
    }
    if scale == S::zero() {
        scale = S::one();
    }
    a /= scale;
    let mut v = Matrix4::identity();
    let mut max_diag = S::zero();
    for i in 0..4 {
        max_diag = max_diag.max(a[(i, i)].abs());
    }
    let precision = S::default_epsilon() * S::from_f64_lossy(2.0);
    loop {
        let mut finished = true;
        for p in 1..4 {
            for q in 0..p {
                let threshold = S::min_positive_value().max(precision * max_diag);
                if a[(p, q)].abs() <= threshold && a[(q, p)].abs() <= threshold {
                    continue;
                }
                finished = false;
                let t = a[(p, p)] + a[(q, q)];
                let d = a[(q, p)] - a[(p, q)];
                let (c, s) = if d.abs() < S::min_positive_value() {
                    (S::one(), S::zero())
                } else {
                    let u = t / d;
                    let tmp = (S::one() + u * u).sqrt();
                    (u / tmp, S::one() / tmp)
                };
                let x = c * a[(p, p)] + s * a[(q, p)];
                let y = c * a[(p, q)] + s * a[(q, q)];
                let z = -s * a[(p, q)] + c * a[(q, q)];
                let deno = S::from_f64_lossy(2.0) * y.abs();
                let (cr, sr) = if deno < S::min_positive_value() {
                    (S::one(), S::zero())
                } else {
                    let tau = (x - z) / deno;
                    let w = (tau * tau + S::one()).sqrt();
                    let tangent = if tau > S::zero() {
                        S::one() / (tau + w)
                    } else {
                        S::one() / (tau - w)
                    };
                    let sign = if tangent > S::zero() {
                        S::one()
                    } else {
                        -S::one()
                    };
                    let n = S::one() / (tangent * tangent + S::one()).sqrt();
                    (n, -sign * (y / y.abs()) * tangent.abs() * n)
                };
                let cl = c * cr + s * sr;
                let sl = -c * sr + s * cr;
                rotate_left(&mut a, p, q, cl, sl);
                rotate_right(&mut a, p, q, cr, sr);
                rotate_right(&mut v, p, q, cr, sr);
                max_diag = max_diag.max(a[(p, p)].abs()).max(a[(q, q)].abs());
            }
        }
        if finished {
            break;
        }
    }
    let mut singular = Vector4::from_fn(|i, _| a[(i, i)].abs() * scale);
    for i in 0..4 {
        let mut largest = i;
        for j in i + 1..4 {
            if singular[j] > singular[largest] {
                largest = j;
            }
        }
        if singular[largest] == S::zero() {
            break;
        }
        singular.swap_rows(i, largest);
        v.swap_columns(i, largest);
    }
    v.column(3).into_owned()
}

impl<S: BasaltScalar> BundleAdjustmentBase<S> {
    /// Noise parameters are explicit: the original base leaves them to its owner.
    pub fn new(calib: Calibration<S>, obs_std_dev: S, huber_thresh: S) -> Self {
        Self {
            frame_states: BTreeMap::new(),
            frame_poses: BTreeMap::new(),
            lmdb: LandmarkDatabase::default(),
            obs_std_dev,
            huber_thresh,
            calib,
        }
    }

    pub fn get_pose_state_with_lin(&self, time: i64) -> PoseStateWithLin<S> {
        self.frame_poses
            .get(&time)
            .cloned()
            .unwrap_or_else(|| PoseStateWithLin::from_imu_state(&self.frame_states[&time]))
    }

    fn relative(&self, host: TimeCamId, target: TimeCamId) -> Matrix4<S> {
        if host == target {
            return Matrix4::identity();
        }
        compute_rel_pose(
            self.get_pose_state_with_lin(host.frame_id).pose(),
            &self.calib.t_i_c[host.cam_id],
            self.get_pose_state_with_lin(target.frame_id).pose(),
            &self.calib.t_i_c[target.cam_id],
            None,
            None,
        )
        .to_homogeneous()
    }

    pub fn compute_error(&self, mut outliers: Option<&mut Outliers<S>>, threshold: S) -> S {
        if let Some(out) = outliers.as_deref_mut() {
            out.clear();
        }
        let mut error = S::zero();
        for (&host, targets) in self.lmdb.observations() {
            for (&target, ids) in targets {
                let transform = self.relative(host, target);
                for &id in ids {
                    let point = self.lmdb.landmark(id);
                    let mut residual = Vector2::zeros();
                    let valid = linearize_point(
                        &point.obs[&target],
                        point,
                        &transform,
                        &self.calib.intrinsics[target.cam_id],
                        &mut residual,
                        None,
                        None,
                        None,
                    );
                    let e = residual.norm();
                    if let Some(out) = outliers.as_deref_mut()
                        && (!valid || e > threshold)
                    {
                        let value = if host == target {
                            S::from_f64_lossy(-2.0)
                        } else if valid {
                            e
                        } else {
                            -S::one()
                        };
                        out.entry(id).or_default().push((target, value));
                    }
                    if valid {
                        let weight = if e < self.huber_thresh {
                            S::one()
                        } else {
                            self.huber_thresh / e
                        };
                        error += S::from_f64_lossy(0.5)
                            * (S::from_f64_lossy(2.0) - weight)
                            * (weight / (self.obs_std_dev * self.obs_std_dev))
                            * residual.norm_squared();
                    }
                }
            }
        }
        error
    }

    pub fn filter_outliers(&mut self, threshold: S, minimum_observations: i32) {
        let mut outliers = BTreeMap::new();
        self.compute_error(Some(&mut outliers), threshold);
        for (id, entries) in outliers {
            let remaining = self.lmdb.num_landmark_observations(id) as i32 - entries.len() as i32;
            if remaining < minimum_observations
                || entries.iter().any(|(_, e)| *e == S::from_f64_lossy(-2.0))
            {
                self.lmdb.remove_landmark(id);
            } else {
                self.lmdb.remove_observations(
                    id,
                    &entries
                        .into_iter()
                        .map(|(target, _)| target)
                        .collect::<BTreeSet<_>>(),
                );
            }
        }
    }

    /// # Panics
    /// Requires existing connected landmarks hosted in a different stored pose.
    pub fn optimize_single_frame_pose(
        &self,
        state: &mut PoseStateWithLin<S>,
        connected: &[Vec<i32>],
    ) {
        for _ in 0..2 {
            let mut h = SMatrix::<S, 6, 6>::zeros();
            let mut b = SVector::<S, 6>::zeros();
            let mut relative = BTreeMap::new();
            for (camera, ids) in connected.iter().enumerate() {
                let target = TimeCamId {
                    frame_id: state.t_ns(),
                    cam_id: camera,
                };
                for &id in ids {
                    let point = self.lmdb.landmark(id as usize);
                    let (transform, derivative) = relative
                        .entry((point.host_kf_id, target))
                        .or_insert_with(|| {
                            assert_ne!(point.host_kf_id.frame_id, state.t_ns());
                            let mut derivative = SMatrix::<S, 6, 6>::zeros();
                            let transform = compute_rel_pose(
                                self.frame_poses[&point.host_kf_id.frame_id].pose(),
                                &self.calib.t_i_c[point.host_kf_id.cam_id],
                                state.pose(),
                                &self.calib.t_i_c[camera],
                                None,
                                Some(&mut derivative),
                            );
                            (transform.to_homogeneous(), derivative)
                        });
                    let mut residual = Vector2::zeros();
                    let mut jacobian = SMatrix::<S, 2, 6>::zeros();
                    if linearize_point(
                        &point.obs[&target],
                        point,
                        transform,
                        &self.calib.intrinsics[camera],
                        &mut residual,
                        Some(&mut jacobian),
                        None,
                        None,
                    ) {
                        // Upstream computes a robust diagnostic cost but intentionally
                        // assembles these two Gauss-Newton iterations without weights.
                        jacobian *= *derivative;
                        h += jacobian.transpose() * jacobian;
                        b += jacobian.transpose() * residual;
                    }
                }
            }
            let lambda = S::from_f64_lossy(1e-6);
            for i in 0..6 {
                let damping = (lambda * h[(i, i)]).max(lambda);
                h[(i, i)] += damping;
            }
            let solution =
                crate::utils::ldlt::Ldlt::new(DMatrix::from_column_slice(6, 6, h.as_slice()))
                    .solve_vector(&DVector::from_column_slice(b.as_slice()));
            state.apply_inc(&-SVector::<S, 6>::from_column_slice(solution.as_slice()));
        }
    }

    pub fn get_current_points<T: BasaltScalar>(
        &self,
        points: &mut Vec<Vector3<T>>,
        ids: &mut Vec<i32>,
    ) {
        points.clear();
        ids.clear();
        for host in self.lmdb.host_kfs() {
            let pose = if let Some(state) = self.frame_states.get(&host.frame_id) {
                state.state().state.pose.t_w_i
            } else {
                *self.frame_poses[&host.frame_id].pose()
            };
            let camera = &self.calib.t_i_c[host.cam_id];
            let world = Isometry3::from_parts(
                Translation3::from(
                    pose.translation.vector + pose.rotation * camera.translation.vector,
                ),
                so3_product(&pose.rotation, &camera.rotation),
            )
            .to_homogeneous();
            for point in self.lmdb.landmarks_for_host(host) {
                let mut local = StereographicParam::unproject(&point.direction, None);
                local[3] = point.inv_dist;
                let global = world * local;
                points.push(Vector3::from_fn(|i, _| {
                    T::from_f64_lossy((global[i] / global[3]).as_f64())
                }));
                ids.push(1);
            }
        }
    }

    /// # Panics
    /// Requires ordered 6/15-dimensional states with frozen linearization points.
    pub fn compute_delta(&self, order: &AbsOrderMap) -> DVector<S> {
        let mut delta = DVector::zeros(order.total_size);
        for (&time, &(offset, size)) in &order.abs_order_map {
            match size {
                6 => {
                    let state = &self.frame_poses[&time];
                    assert!(state.is_linearized());
                    delta.rows_mut(offset as usize, 6).copy_from(state.delta());
                }
                15 => {
                    let state = &self.frame_states[&time];
                    assert!(state.is_linearized());
                    delta.rows_mut(offset as usize, 15).copy_from(state.delta());
                }
                _ => panic!("invalid marginalization state size"),
            }
        }
        delta
    }

    /// # Panics
    /// Requires the prior to occupy the same leading state order in the system.
    pub fn linearize_marg_prior(
        &self,
        prior: &MargLinData<S>,
        order: &AbsOrderMap,
        h: &mut DMatrix<S>,
        b: &mut DVector<S>,
    ) -> S {
        assert_eq!(prior.h.ncols(), prior.order.total_size);
        for (time, entry) in &prior.order.abs_order_map {
            assert_eq!(&order.abs_order_map[time], entry);
        }
        let delta = self.compute_delta(&prior.order);
        let n = prior.order.total_size;
        let (prior_h, prior_b) = if prior.is_sqrt {
            (
                prior.h.transpose() * &prior.h,
                prior.h.transpose() * (&prior.b + &prior.h * delta),
            )
        } else {
            (prior.h.clone(), &prior.h * delta + &prior.b)
        };
        for i in 0..n {
            b[i] += prior_b[i];
            for j in 0..n {
                h[(i, j)] += prior_h[(i, j)];
            }
        }
        self.compute_marg_prior_error(prior)
    }

    /// # Panics
    /// Requires matching prior dimensions and linearized states in its order.
    pub fn compute_marg_prior_error(&self, prior: &MargLinData<S>) -> S {
        assert_eq!(prior.h.ncols(), prior.order.total_size);
        let delta = self.compute_delta(&prior.order);
        let half = S::from_f64_lossy(0.5);
        if prior.is_sqrt {
            let transformed = &prior.h * delta;
            transformed.dot(&(transformed.clone() * half + &prior.b))
        } else {
            delta.dot(&((&prior.h * &delta) * half + &prior.b))
        }
    }

    pub fn compute_marg_prior_model_cost_change(
        &self,
        prior: &MargLinData<S>,
        scaling: &DVector<S>,
        increment: &DVector<S>,
    ) -> S {
        let delta = self.compute_delta(&prior.order);
        let inc = if scaling.is_empty() {
            increment.clone()
        } else {
            scaling.component_mul(increment)
        };
        let half = S::from_f64_lossy(0.5);
        if prior.is_sqrt {
            let j_inc = &prior.h * inc;
            -j_inc.dot(&(&prior.h * delta + &prior.b + &j_inc * half))
        } else {
            -inc.dot(&(&prior.h * (delta + &inc * half) + &prior.b))
        }
    }

    pub fn compute_model_cost_change(
        &self,
        h: &DMatrix<S>,
        b: &DVector<S>,
        increment: &DVector<S>,
    ) -> S {
        -increment.dot(&((h * increment) * S::from_f64_lossy(0.5) + b))
    }

    /// Invalid projection diagnostics are NaNs; upstream leaves those entries
    /// uninitialized. Valid projections and appended landmark IDs are preserved.
    pub fn compute_projections<T: BasaltScalar>(
        &self,
        data: &mut [Vec<Vector4<T>>],
        last_time: i64,
    ) {
        for (&host, targets) in self.lmdb.observations() {
            for (&target, ids) in targets {
                if target.frame_id != last_time {
                    continue;
                }
                let transform = self.relative(host, target);
                for &id in ids {
                    let mut residual = Vector2::zeros();
                    let mut projection = Vector4::repeat(S::from_f64_lossy(f64::NAN));
                    linearize_point(
                        &Vector2::zeros(),
                        self.lmdb.landmark(id),
                        &transform,
                        &self.calib.intrinsics[target.cam_id],
                        &mut residual,
                        None,
                        None,
                        Some(&mut projection),
                    );
                    projection[3] = S::from_f64_lossy(id as f64);
                    data[target.cam_id].push(projection.map(|x| T::from_f64_lossy(x.as_f64())));
                }
            }
        }
    }

    pub fn triangulate(
        first: &Vector3<S>,
        second: &Vector3<S>,
        transform: &Isometry3<S>,
    ) -> Vector4<S> {
        let rotation = so3_inverse(&transform.rotation);
        let inverse = Isometry3::from_parts(
            Translation3::from(-(rotation * transform.translation.vector)),
            rotation,
        )
        .to_homogeneous();
        let identity = Matrix4::<S>::identity();
        let mut a = Matrix4::zeros();
        a.set_row(
            0,
            &(identity.row(2) * first[0] - identity.row(0) * first[2]),
        );
        a.set_row(
            1,
            &(identity.row(2) * first[1] - identity.row(1) * first[2]),
        );
        a.set_row(
            2,
            &(inverse.row(2) * second[0] - inverse.row(0) * second[2]),
        );
        a.set_row(
            3,
            &(inverse.row(2) * second[1] - inverse.row(1) * second[2]),
        );
        let mut result = jacobi_smallest_vector(a);
        result /= result.fixed_rows::<3>(0).norm();
        if first.dot(&result.fixed_rows::<3>(0)) < S::zero() {
            result = -result;
        }
        result
    }

    pub fn backup(&mut self) {
        for state in self.frame_states.values_mut() {
            state.backup();
        }
        for pose in self.frame_poses.values_mut() {
            pose.backup();
        }
        self.lmdb.backup();
    }
    pub fn restore(&mut self) {
        for state in self.frame_states.values_mut() {
            state.restore();
        }
        for pose in self.frame_poses.values_mut() {
            pose.restore();
        }
        self.lmdb.restore();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn triangulate_exact_two_camera_geometry() {
        let point = Vector3::new(0.4, -0.3, 4.0);
        let transform = Isometry3::translation(0.2, 0.0, 0.0);
        let first = point.normalize();
        let second = (point - transform.translation.vector).normalize();
        let result = BundleAdjustmentBase::<f64>::triangulate(&first, &second, &transform);
        assert!((result.fixed_rows::<3>(0) / result[3] - point).norm() < 1e-10);
    }
}
