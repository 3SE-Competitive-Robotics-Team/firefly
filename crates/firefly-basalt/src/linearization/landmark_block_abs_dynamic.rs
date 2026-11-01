// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
//! Original Basalt `linearization/landmark_block_abs_dynamic.hpp`.
//! Landmark references are passed explicitly at linearization and back-substitution
//! instead of retaining C++ raw pointers across the solver's cost evaluation.
use super::landmark_block::{Options, RelPoseLin, State};
use crate::{
    utils::{
        ba_utils::linearize_point,
        complete_orthogonal_decomposition::{apply_householder_left, make_householder},
        imu_types::AbsOrderMap,
    },
    vi_estimator::landmark_database::Keypoint,
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::observation::TimeCamId;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector, SMatrix, Vector2, Vector3};
use std::collections::{BTreeMap, BTreeSet, HashMap};

#[derive(Clone, Debug)]
struct Observation<S: BasaltScalar> {
    target: TimeCamId,
    host_offset: usize,
    target_offset: Option<usize>,
    relative: RelPoseLin<S>,
}
#[derive(Clone, Debug)]
pub struct LandmarkBlockAbsDynamic<S: BasaltScalar, const POSE_SIZE: usize> {
    pub landmark_id: usize,
    host: TimeCamId,
    options: Options<S>,
    state: State,
    storage: DMatrix<S>,
    landmark_scaling: Vector3<S>,
    damping_rotations: Vec<(S, S)>,
    observations: Vec<Observation<S>>,
    residuals_by_pose: BTreeMap<usize, BTreeSet<usize>>,
    padding_index: usize,
    landmark_index: usize,
    residual_index: usize,
}
impl<S: BasaltScalar, const POSE_SIZE: usize> LandmarkBlockAbsDynamic<S, POSE_SIZE> {
    /// Allocate the original padded dense block; no landmark state is copied.
    /// # Panics
    /// Requires a host in the ordering, every relative pose, and pose size six.
    #[must_use]
    pub fn new(
        landmark_id: usize,
        point: &Keypoint<S>,
        relative: &HashMap<(TimeCamId, TimeCamId), RelPoseLin<S>>,
        ordering: &AbsOrderMap,
        options: Options<S>,
    ) -> Self {
        assert_eq!(POSE_SIZE, 6);
        let host_offset = usize::try_from(ordering.abs_order_map[&point.host_kf_id.frame_id].0)
            .expect("nonnegative ordering");
        let mut observations = Vec::new();
        let mut residuals_by_pose: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
        for &target in point.obs.keys() {
            let index = observations.len();
            let target_offset = ordering
                .abs_order_map
                .get(&target.frame_id)
                .map(|entry| usize::try_from(entry.0).expect("nonnegative ordering"));
            residuals_by_pose
                .entry(host_offset)
                .or_default()
                .insert(index);
            if let Some(offset) = target_offset {
                residuals_by_pose.entry(offset).or_default().insert(index);
            }
            observations.push(Observation {
                target,
                host_offset,
                target_offset,
                relative: relative[&(point.host_kf_id, target)].clone(),
            });
        }
        let padding_index = ordering.total_size;
        let landmark_index = padding_index + (4 - padding_index % 4) % 4;
        let residual_index = landmark_index + 3;
        let storage = DMatrix::zeros(observations.len() * 2 + 3, residual_index + 1);
        Self {
            landmark_id,
            host: point.host_kf_id,
            options,
            state: State::Allocated,
            storage,
            landmark_scaling: Vector3::repeat(S::one()),
            damping_rotations: Vec::with_capacity(6),
            observations,
            residuals_by_pose,
            padding_index,
            landmark_index,
            residual_index,
        }
    }
    #[must_use]
    pub fn state(&self) -> State {
        self.state
    }
    #[must_use]
    pub fn is_numerical_failure(&self) -> bool {
        self.state == State::NumericalFailure
    }
    #[must_use]
    pub fn host_keyframe(&self) -> TimeCamId {
        self.host
    }
    #[must_use]
    pub fn num_q2_rows(&self) -> usize {
        self.storage.nrows() - 3
    }
    #[must_use]
    pub fn storage(&self) -> &DMatrix<S> {
        &self.storage
    }
    #[must_use]
    pub fn has_landmark_damping(&self) -> bool {
        !self.damping_rotations.is_empty()
    }
    /// Refresh the data reached through the original block's relative-pose pointers.
    /// # Panics
    /// Requires every allocated host-target pair to remain present.
    pub fn update_relative_poses(
        &mut self,
        relative: &HashMap<(TimeCamId, TimeCamId), RelPoseLin<S>>,
    ) {
        for observation in &mut self.observations {
            observation.relative = relative[&(self.host, observation.target)].clone();
        }
    }
    /// # Panics
    /// Requires the same landmark and observation layout as allocation.
    pub fn linearize_landmark(&mut self, point: &Keypoint<S>, calibration: &Calibration<S>) -> S {
        assert_ne!(self.state, State::Uninitialized);
        assert_eq!(point.host_kf_id, self.host);
        assert_eq!(point.obs.len(), self.observations.len());
        self.storage.fill(S::zero());
        self.damping_rotations.clear();
        let mut numerically_valid = true;
        let mut error_sum = S::zero();
        for (index, observation) in self.observations.iter().enumerate() {
            let Some(target_offset) = observation.target_offset else {
                continue;
            };
            let mut residual = Vector2::zeros();
            // The C++ invalid-projection branch leaves these uninitialized when
            // use_valid_projections_only is disabled. Safe Rust uses NaN to
            // report NumericalFailure deterministically for that undefined case.
            let mut pose_derivative = SMatrix::<S, 2, 6>::repeat(S::from_f64_lossy(f64::NAN));
            let mut point_derivative = SMatrix::<S, 2, 3>::repeat(S::from_f64_lossy(f64::NAN));
            let valid = linearize_point(
                &point.obs[&observation.target],
                point,
                &observation.relative.t_t_h,
                &calibration.intrinsics[observation.target.cam_id],
                &mut residual,
                Some(&mut pose_derivative),
                Some(&mut point_derivative),
                None,
            );
            if self.options.use_valid_projections_only && !valid {
                continue;
            }
            numerically_valid &= pose_derivative
                .iter()
                .chain(point_derivative.iter())
                .all(nalgebra::ComplexField::is_finite);
            let squared = residual.norm_squared();
            let huber = self.options.huber_parameter;
            let weight = if huber > S::zero() && squared > huber * huber {
                huber / squared.sqrt()
            } else {
                S::one()
            };
            let half = S::from_f64_lossy(0.5);
            let error = if huber > S::zero() {
                half * (S::from_f64_lossy(2.) - weight) * weight * squared
            } else {
                half * squared
            };
            let sqrt_weight = weight.sqrt() / self.options.obs_std_dev;
            error_sum += error / (self.options.obs_std_dev * self.options.obs_std_dev);
            self.storage
                .view_mut((index * 2, self.landmark_index), (2, 3))
                .copy_from(&(point_derivative * sqrt_weight));
            self.storage
                .view_mut((index * 2, self.residual_index), (2, 1))
                .copy_from(&(residual * sqrt_weight));
            pose_derivative *= sqrt_weight;
            let host = pose_derivative * observation.relative.d_rel_d_h;
            let target = pose_derivative * observation.relative.d_rel_d_t;
            for r in 0..2 {
                for c in 0..6 {
                    self.storage[(index * 2 + r, observation.host_offset + c)] += host[(r, c)];
                    self.storage[(index * 2 + r, target_offset + c)] += target[(r, c)];
                }
            }
        }
        self.state = if numerically_valid {
            State::Linearized
        } else {
            State::NumericalFailure
        };
        error_sum
    }
    /// # Panics
    /// Requires a successfully linearized block with at least two observations.
    pub fn perform_qr(&mut self) {
        assert_eq!(self.state, State::Linearized);
        let rows = self.num_q2_rows();
        assert!(rows >= 3);
        if self.options.use_householder {
            for column in 0..3 {
                let remaining = rows - column;
                let mut reflector = self
                    .storage
                    .view((column, self.landmark_index + column), (remaining, 1))
                    .column(0)
                    .into_owned();
                let (tau, _) = make_householder(&mut reflector);
                let essential = reflector.rows(1, remaining - 1).into_owned();
                let mut block = self.storage.rows(column, remaining).into_owned();
                apply_householder_left(&mut block, 0, 0, &essential, tau);
                self.storage.rows_mut(column, remaining).copy_from(&block);
            }
        } else {
            for column in 0..3 {
                for row in (column + 1..rows).rev() {
                    let rotation = givens(
                        self.storage[(row - 1, self.landmark_index + column)],
                        self.storage[(row, self.landmark_index + column)],
                    );
                    rotate(&mut self.storage, row, row - 1, rotation);
                }
            }
        }
        self.state = State::Marginalized;
    }
    /// # Panics
    /// Requires a marginalized block and nonnegative damping.
    pub fn set_landmark_damping(&mut self, lambda: S) {
        assert_eq!(self.state, State::Marginalized);
        assert!(lambda >= S::zero());
        let start = self.num_q2_rows();
        if self.has_landmark_damping() {
            assert_eq!(self.damping_rotations.len(), 6);
            for n in (0..3).rev() {
                for m in (0..=n).rev() {
                    let (c, s) = self.damping_rotations.pop().unwrap();
                    rotate(&mut self.storage, start + n - m, n, (c, -s));
                }
            }
        }
        if lambda == S::zero() {
            for i in 0..3 {
                self.storage[(start + i, self.landmark_index + i)] = S::zero();
            }
        } else {
            assert!(
                self.landmark_scaling
                    .iter()
                    .all(nalgebra::ComplexField::is_finite)
            );
            for i in 0..3 {
                self.storage[(start + i, self.landmark_index + i)] = lambda.sqrt();
            }
            for n in 0..3 {
                for m in 0..=n {
                    let rotation = givens(
                        self.storage[(n, self.landmark_index + n)],
                        self.storage[(start + n - m, self.landmark_index + n)],
                    );
                    self.damping_rotations.push(rotation);
                    rotate(&mut self.storage, start + n - m, n, rotation);
                }
            }
        }
    }
    /// # Panics
    /// Requires a linearized block.
    pub fn scale_landmark_columns(&mut self) {
        assert_eq!(self.state, State::Linearized);
        let rows = self.num_q2_rows();
        for c in 0..3 {
            // Upstream storage is row-major: this column reduction is strided
            // and uses the scalar Eigen traversal rather than a packed dot.
            let mut squared_norm = S::zero();
            for row in 0..rows {
                let value = self.storage[(row, self.landmark_index + c)];
                squared_norm += value * value;
            }
            self.landmark_scaling[c] =
                S::one() / (self.options.jacobi_scaling_eps + squared_norm.sqrt());
            for r in 0..rows {
                self.storage[(r, self.landmark_index + c)] *= self.landmark_scaling[c];
            }
        }
    }
    /// # Panics
    /// Requires a marginalized, undamped block and the full pose scaling vector.
    pub fn scale_pose_columns(&mut self, scaling: &DVector<S>) {
        assert_eq!(self.state, State::Marginalized);
        assert!(!self.has_landmark_damping());
        assert_eq!(scaling.len(), self.padding_index);
        for c in 0..self.padding_index {
            for r in 0..self.num_q2_rows() {
                self.storage[(r, c)] *= scaling[c];
            }
        }
    }
    /// # Panics
    /// Requires a linearized block and a full pose-sized output.
    pub fn add_pose_diagonal_squared(&self, output: &mut DVector<S>) {
        assert_eq!(self.state, State::Linearized);
        assert!(
            self.observations
                .iter()
                .all(|observation| observation.target_offset.is_some()),
            "upstream ordering lookup requires every observed target"
        );
        assert_eq!(output.len(), self.padding_index);
        for (&offset, indices) in &self.residuals_by_pose {
            for &index in indices {
                for c in 0..POSE_SIZE {
                    output[offset + c] += self.storage[(2 * index, offset + c)]
                        * self.storage[(2 * index, offset + c)]
                        + self.storage[(2 * index + 1, offset + c)]
                            * self.storage[(2 * index + 1, offset + c)];
                }
            }
        }
    }
    /// # Panics
    /// Requires output space for the reduced rows at `start`.
    pub fn get_dense_q2jp_q2r(
        &self,
        jacobian: &mut DMatrix<S>,
        residual: &mut DVector<S>,
        start: usize,
    ) {
        assert_eq!(jacobian.ncols(), self.padding_index);
        let rows = self.num_q2_rows();
        jacobian
            .view_mut((start, 0), (rows, self.padding_index))
            .copy_from(&self.storage.view((3, 0), (rows, self.padding_index)));
        residual
            .rows_mut(start, rows)
            .copy_from(&self.storage.view((3, self.residual_index), (rows, 1)));
    }
    pub fn add_dense_h_b(&self, hessian: &mut DMatrix<S>, rhs: &mut DVector<S>) {
        let rows = self.num_q2_rows();
        let jacobian = self.storage.view((3, 0), (rows, self.padding_index));
        let residual = self.storage.view((3, self.residual_index), (rows, 1));
        *hessian += jacobian.transpose() * jacobian;
        *rhs += jacobian.transpose() * residual;
    }
    /// # Panics
    /// Requires a marginalized block, matching landmark and full pose increment.
    pub fn back_substitute(
        &mut self,
        point: &mut Keypoint<S>,
        pose_increment: &DVector<S>,
        model_decrease: &mut S,
    ) {
        assert_eq!(self.state, State::Marginalized);
        assert_eq!(point.host_kf_id, self.host);
        assert_eq!(pose_increment.len(), self.padding_index);
        let mut increment = Vector3::zeros();
        let rhs = self.storage.view((0, self.residual_index), (3, 1))
            + self.storage.view((0, 0), (3, self.padding_index)) * pose_increment;
        for r in (0..3).rev() {
            let mut sum = S::zero();
            for c in r + 1..3 {
                sum += self.storage[(r, self.landmark_index + c)] * increment[c];
            }
            increment[r] = (rhs[r] - sum) / self.storage[(r, self.landmark_index + r)];
        }
        increment = -increment;
        self.set_landmark_damping(S::zero());
        let rows = self.num_q2_rows();
        let mut projected = self.storage.view((0, 0), (rows, self.padding_index)) * pose_increment;
        for r in 0..3 {
            let mut sum = S::zero();
            for c in r..3 {
                sum += self.storage[(r, self.landmark_index + c)] * increment[c];
            }
            projected[r] += sum;
        }
        let mut change = S::zero();
        for r in 0..rows {
            change += projected[r]
                * (S::from_f64_lossy(0.5) * projected[r] + self.storage[(r, self.residual_index)]);
        }
        *model_decrease -= change;
        if !increment
            .iter()
            .chain(point.direction.iter())
            .all(nalgebra::ComplexField::is_finite)
            || !point.inv_dist.is_finite()
        {
            log::error!("Numerical failure in landmark back-substitution");
        }
        increment.component_mul_assign(&self.landmark_scaling);
        point.direction += increment.fixed_rows::<2>(0);
        let distance = point.inv_dist + increment[2];
        point.inv_dist = if distance > S::zero() {
            distance
        } else {
            S::zero()
        };
    }
}

// Real specialization of Eigen 5.0.1 JacobiRotation::makeGivens, MPL-2.0.
#[expect(
    clippy::many_single_char_names,
    reason = "Uses the original Givens rotation notation"
)]
fn givens<S: BasaltScalar>(p: S, q: S) -> (S, S) {
    if q == S::zero() {
        (if p < S::zero() { -S::one() } else { S::one() }, S::zero())
    } else if p == S::zero() {
        (S::zero(), if q < S::zero() { S::one() } else { -S::one() })
    } else if p.abs() > q.abs() {
        let t = q / p;
        let mut u = (S::one() + t * t).sqrt();
        if p < S::zero() {
            u = -u;
        }
        let c = S::one() / u;
        (c, -t * c)
    } else {
        let t = p / q;
        let mut u = (S::one() + t * t).sqrt();
        if q < S::zero() {
            u = -u;
        }
        let s = -S::one() / u;
        (-t * s, s)
    }
}
fn rotate<S: BasaltScalar>(matrix: &mut DMatrix<S>, first: usize, second: usize, (c, s): (S, S)) {
    for col in 0..matrix.ncols() {
        let x = matrix[(first, col)];
        let y = matrix[(second, col)];
        matrix[(first, col)] = c * x + s * y;
        matrix[(second, col)] = -s * x + c * y;
    }
}
