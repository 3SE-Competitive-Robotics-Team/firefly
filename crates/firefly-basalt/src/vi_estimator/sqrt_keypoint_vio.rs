// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original Basalt `sqrt_keypoint_vio.h/.cpp`, revision
//! `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
use super::{
    ba_base::BundleAdjustmentBase,
    landmark_database::{Keypoint, KeypointObservation},
    marg_helper::MargHelper,
    vio_estimator::{
        VioEstimatorEngine, VioFrameDiagnostics, VioStatus, VioVisualizationData, VioWorkerInputs,
        VioWorkerOutputs,
    },
};
use crate::imu::imu_types::ImuData;
use crate::imu::imu_types::PoseState;
use crate::imu::imu_types::PoseVelBiasState;
use crate::imu::preintegration::IntegratedImuMeasurement;
use crate::{
    linearization::{
        landmark_block,
        linearization_base::{LinearizationBase, Options, is_linearization_sqrt},
    },
    optical_flow::OpticalFlowResult,
    utils::{
        ba_utils::compute_rel_pose,
        imu_types::{
            AbsOrderMap, ImuLinData, MargData, MargLinData, PoseStateWithLin,
            PoseVelBiasStateWithLin,
        },
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
use nalgebra::{DMatrix, DVector, Isometry3, Quaternion, UnitQuaternion, Vector3, Vector4};
use std::{
    collections::{BTreeMap, BTreeSet, HashSet},
    sync::{Arc, atomic::Ordering},
};

pub struct SqrtKeypointVioEstimator<S: BasaltScalar> {
    pub ba: BundleAdjustmentBase<S>,
    pub marg_data: MargLinData<S>,
    pub nullspace_marg_data: MargLinData<S>,
    pub last_state_t_ns: i64,
    pub imu_meas: BTreeMap<i64, IntegratedImuMeasurement<S>>,
    pub kf_ids: BTreeSet<i64>,
    pub prev_opt_flow_res: BTreeMap<i64, Arc<OpticalFlowResult>>,
    pub num_points_kf: BTreeMap<i64, i32>,
    pub g: Vector3<S>,
    pub gyro_bias_sqrt_weight: Vector3<S>,
    pub accel_bias_sqrt_weight: Vector3<S>,
    pub config: VioConfig,
    pub max_states: usize,
    pub max_kfs: usize,
    pub t_w_i_init: Isometry3<S>,
    pub initialized: bool,
    pub opt_started: bool,
    take_kf: bool,
    frames_after_kf: i32,
    bg_init: Vector3<S>,
    ba_init: Vector3<S>,
    lambda: S,
    min_lambda: S,
    max_lambda: S,
    lambda_vee: S,
    pub stats_all: ExecutionStats,
    pub stats_sums: ExecutionStats,
}

// Eigen Quaternion::setFromTwoVectors and the needed column of its 2x3
// JacobiSVD full V. Eigen's wide-matrix preconditioner computes the pivoted
// QR of the transpose. All subsequent 2x2 rotations and singular-value sorting
// touch only columns 0 and 1, so column 2 remains Q.col(2) exactly.
// Eigen implementation copyright (C) 2009-2016 Gael Guennebaud and Benoit Jacob,
// MPL-2.0, https://mozilla.org/MPL/2.0/ .
fn rotation_from_two_vectors<S: BasaltScalar>(
    first: &Vector3<S>,
    second: &Vector3<S>,
) -> UnitQuaternion<S> {
    use crate::utils::complete_orthogonal_decomposition::{
        apply_householder_left, make_householder,
    };
    use crate::utils::sophus_utils::so3_from_quaternion;
    let normalize = |v: &Vector3<S>| {
        let squared = v.norm_squared();
        if squared > S::zero() {
            v / squared.sqrt()
        } else {
            *v
        }
    };
    let first = normalize(first);
    let second = normalize(second);
    let mut cosine = second.dot(&first);
    let dummy = S::from_f64_lossy(if std::mem::size_of::<S>() == 4 {
        1e-5
    } else {
        1e-12
    });
    let quaternion = if cosine < -S::one() + dummy {
        cosine = cosine.max(-S::one());
        let scale = first
            .iter()
            .chain(second.iter())
            .fold(S::zero(), |m, x| m.max(x.abs()));
        let mut matrix = DMatrix::from_fn(3, 2, |r, c| {
            if c == 0 {
                first[r] / scale
            } else {
                second[r] / scale
            }
        });
        if matrix.column(1).norm() > matrix.column(0).norm() {
            matrix.swap_columns(0, 1);
        }
        let mut reflectors = Vec::new();
        for column in 0..2 {
            let mut vector = matrix.column(column).rows(column, 3 - column).into_owned();
            let (tau, beta) = make_householder(&mut vector);
            let tail = vector.rows(1, vector.len() - 1).into_owned();
            apply_householder_left(&mut matrix, column, column + 1, &tail, tau);
            matrix[(column, column)] = beta;
            reflectors.push((tau, tail));
        }
        let mut axis = DMatrix::from_column_slice(3, 1, &[S::zero(), S::zero(), S::one()]);
        for column in (0..2).rev() {
            let (tau, tail) = &reflectors[column];
            apply_householder_left(&mut axis, column, 0, tail, *tau);
        }
        let scalar_squared = (S::one() + cosine) * S::from_f64_lossy(0.5);
        let vector = axis.column(0) * (S::one() - scalar_squared).sqrt();
        Quaternion::new(scalar_squared.sqrt(), vector[0], vector[1], vector[2])
    } else {
        let axis = first.cross(&second);
        let scalar = ((S::one() + cosine) * S::from_f64_lossy(2.0)).sqrt();
        let vector = axis / scalar;
        Quaternion::new(
            scalar * S::from_f64_lossy(0.5),
            vector[0],
            vector[1],
            vector[2],
        )
    };
    so3_from_quaternion(quaternion)
}

impl<S: BasaltScalar> SqrtKeypointVioEstimator<S> {
    /// The first six diagnostic directions are deterministic; the seventh uses
    /// Rust's RNG rather than Eigen's process-global C `rand()` sequence.
    pub fn check_marg_nullspace(&self) -> DVector<f64> {
        use rand::RngExt;
        let prior = &self.nullspace_marg_data;
        let mut rng = rand::rng();
        let random = DVector::from_fn(prior.order.total_size, |_, _| rng.random_range(-1.0..=1.0));
        super::sqrt_ba_base::check_nullspace(
            prior,
            &self.ba.frame_states,
            &self.ba.frame_poses,
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
    /// Returns errors writing the requested statistics JSON data products.
    pub fn debug_finalize(&self, directory: &std::path::Path) -> Result<()> {
        self.stats_all.print();
        self.stats_sums.print();
        self.stats_all.save_json(directory.join("stats_all.json"))?;
        self.stats_sums.save_json(directory.join("stats_sums.json"))
    }

    fn pop_imu(&self, inputs: &VioWorkerInputs) -> Option<ImuData<S>> {
        let mut data = inputs.imu.recv().ok().flatten()?.cast::<S>();
        data.accel = self.ba.calib.calib_accel_bias.calibrated(&data.accel);
        data.gyro = self.ba.calib.calib_gyro_bias.calibrated(&data.gyro);
        Some(data)
    }

    #[expect(
        clippy::needless_pass_by_value,
        reason = "Worker owns input/output handles until stream processing finishes"
    )]
    fn process_stream(
        &mut self,
        inputs: VioWorkerInputs,
        outputs: VioWorkerOutputs,
        status: Arc<VioStatus>,
    ) -> Result<()> {
        let accel_noise = self.ba.calib.dicrete_time_accel_noise_std();
        let gyro_noise = self.ba.calib.dicrete_time_gyro_noise_std();
        let accel_cov = accel_noise.component_mul(&accel_noise);
        let gyro_cov = gyro_noise.component_mul(&gyro_noise);
        let mut data = self
            .pop_imu(&inputs)
            .ok_or_else(|| Error::new(ErrorKind::Internal, "first IMU measurement is absent"))?;
        let mut previous: Option<Arc<OpticalFlowResult>> = None;
        while let Ok(mut incoming) = inputs.vision.recv() {
            if self.config.vio_enforce_realtime {
                while let Ok(next) = inputs.vision.try_recv() {
                    incoming = next;
                }
            }
            let Some(frame) = incoming else {
                break;
            };
            if !self.initialized {
                while data.t_ns < frame.t_ns {
                    let Some(next) = self.pop_imu(&inputs) else {
                        return Ok(());
                    };
                    data = next;
                }
                let rotation = rotation_from_two_vectors(&data.accel, &Vector3::z());
                let mut pose = self.t_w_i_init;
                pose.rotation = rotation;
                self.initialize_at(
                    frame.t_ns,
                    pose,
                    Vector3::zeros(),
                    self.bg_init,
                    self.ba_init,
                );
                if self.config.vio_debug || self.config.vio_extended_logging {
                    self.log_marg_nullspace();
                }
            }
            *status
                .initial_pose
                .lock()
                .map_err(|_| Error::new(ErrorKind::Internal, "initial pose mutex poisoned"))? =
                PoseState::new(0, self.t_w_i_init).cast::<f64>().t_w_i;
            let mut imu_finished = false;
            let measurement = if let Some(previous) = &previous {
                assert!(previous.t_ns < frame.t_ns, "duplicate frame timestamps");
                let state = self.get_state();
                let mut measurement =
                    IntegratedImuMeasurement::new(previous.t_ns, state.bias_gyro, state.bias_accel);
                while data.t_ns <= previous.t_ns {
                    let Some(next) = self.pop_imu(&inputs) else {
                        return Ok(());
                    };
                    data = next;
                }
                while data.t_ns <= frame.t_ns {
                    measurement.integrate(&data, &accel_cov, &gyro_cov);
                    let Some(next) = self.pop_imu(&inputs) else {
                        imu_finished = true;
                        break;
                    };
                    data = next;
                }
                if measurement.start_t_ns() + measurement.dt_ns() < frame.t_ns {
                    if imu_finished {
                        return Ok(());
                    }
                    let time = data.t_ns;
                    data.t_ns = frame.t_ns;
                    measurement.integrate(&data, &accel_cov, &gyro_cov);
                    data.t_ns = time;
                }
                Some(measurement)
            } else {
                None
            };
            self.measure(&frame, measurement, &outputs, &status)?;
            previous = Some(frame);
            if imu_finished {
                break;
            }
        }
        Ok(())
    }
    fn options(&self) -> Options<S> {
        Options {
            lb_options: landmark_block::Options {
                huber_parameter: self.ba.huber_thresh,
                obs_std_dev: self.ba.obs_std_dev,
                ..Default::default()
            },
            linearization_type: self.config.vio_linearization_type,
        }
    }

    fn full_order(&self) -> AbsOrderMap {
        let mut order = AbsOrderMap::default();
        for &time in self.ba.frame_poses.keys() {
            let entry = (order.total_size as i32, 6);
            assert_eq!(self.marg_data.order.abs_order_map[&time], entry);
            order.abs_order_map.insert(time, entry);
            order.total_size += 6;
            order.items += 1;
        }
        for &time in self.ba.frame_states.keys() {
            let entry = (order.total_size as i32, 15);
            if order.items < self.marg_data.order.abs_order_map.len() {
                assert_eq!(self.marg_data.order.abs_order_map[&time], entry);
            }
            order.abs_order_map.insert(time, entry);
            order.total_size += 15;
            order.items += 1;
        }
        order
    }

    /// # Errors
    /// Rejects nonfinite residual linearization, as the original assertion does.
    #[expect(
        clippy::too_many_lines,
        reason = "Preserves the original nested LM acceptance and retry control flow"
    )]
    pub fn optimize(&mut self) -> Result<()> {
        if !self.opt_started && self.ba.frame_states.len() <= 4 {
            return Ok(());
        }
        self.opt_started = true;
        let order = self.full_order();
        self.lambda = S::from_f64_lossy(self.config.vio_lm_lambda_initial);
        let timer = Timer::default();
        let mut stats = ExecutionStats::default();
        stats
            .add("num_cams", self.ba.frame_poses.len() as f64)
            .format("count");
        stats
            .add("num_lms", self.ba.lmdb.num_landmarks() as f64)
            .format("count");
        stats
            .add("num_obs", self.ba.lmdb.num_observations() as f64)
            .format("count");
        let imu = ImuLinData {
            g: &self.g,
            gyro_bias_weight_sqrt: &self.gyro_bias_sqrt_weight,
            accel_bias_weight_sqrt: &self.accel_bias_sqrt_weight,
            imu_meas: self.imu_meas.iter().map(|(&t, m)| (t, m)).collect(),
        };
        let mut linearization = LinearizationBase::<S, 6>::create(
            &self.ba,
            &order,
            self.options(),
            Some(&self.marg_data),
            Some(&imu),
            None,
            None,
            i64::MAX,
        );
        let mut iteration = 0;
        let mut rejected = 0;
        let mut terminated = false;
        while iteration <= self.config.vio_max_iterations && !terminated {
            let mut stage = Timer::default();
            let (before, valid) = linearization.linearize_problem(&self.ba);
            if !valid {
                return Err(Error::new(
                    ErrorKind::Internal,
                    "numerical failure during VIO linearization",
                ));
            }
            stats.add("linearizeProblem", stage.reset()).format("ms");
            linearization.perform_qr();
            stats.add("performQR", stage.reset()).format("ms");
            while iteration <= self.config.vio_max_iterations && !terminated {
                let iteration_timer = Timer::default();
                let (h, b) = linearization.get_dense_h_b(&self.ba);
                stats.add("get_dense_H_b", stage.reset()).format("ms");
                let mut inc = DVector::zeros(order.total_size);
                for _ in 0..3 {
                    let mut damped = h.clone();
                    for i in 0..h.nrows() {
                        damped[(i, i)] += (h[(i, i)] * self.lambda).max(self.min_lambda);
                    }
                    inc = Ldlt::new(damped).solve_vector(&b);
                    stats.add("solve", stage.reset()).format("ms");
                    if inc.iter().all(nalgebra::ComplexField::is_finite) {
                        break;
                    }
                    self.lambda *= self.lambda_vee;
                    self.lambda_vee *= S::from_f64_lossy(2.0);
                }
                self.ba.backup();
                inc = -inc;
                let model_decrease = linearization.back_substitute(&mut self.ba, &inc);
                stats.add("backSubstitute", stage.reset()).format("ms");
                for (&time, state) in &mut self.ba.frame_poses {
                    state.apply_inc(
                        &inc.fixed_rows::<6>(order.abs_order_map[&time].0 as usize)
                            .into_owned(),
                    );
                }
                for (&time, state) in &mut self.ba.frame_states {
                    state.apply_inc(
                        &inc.fixed_rows::<15>(order.abs_order_map[&time].0 as usize)
                            .into_owned(),
                    );
                }
                let step = inc.iter().fold(S::zero(), |m, v| m.max(v.abs()));
                let (inertial, gyro, accel) = super::sc_ba_base::compute_imu_error(
                    &order,
                    &self.ba.frame_states,
                    &self.imu_meas,
                    &self
                        .gyro_bias_sqrt_weight
                        .component_mul(&self.gyro_bias_sqrt_weight),
                    &self
                        .accel_bias_sqrt_weight
                        .component_mul(&self.accel_bias_sqrt_weight),
                    &self.g,
                );
                let after = self.ba.compute_error(None, S::zero())
                    + self.ba.compute_marg_prior_error(&self.marg_data)
                    + inertial
                    + gyro
                    + accel;
                stats.add("computerError2", stage.reset()).format("ms");
                let decrease = before - after;
                let quality = decrease / model_decrease;
                stats
                    .add("iteration", iteration_timer.elapsed())
                    .format("ms");
                iteration += 1;
                if model_decrease > S::zero() && quality > S::zero() {
                    self.lambda *= (S::one() / S::from_f64_lossy(3.0))
                        .max(S::one() - (S::from_f64_lossy(2.0) * quality - S::one()).powi(3));
                    self.lambda = self.lambda.max(self.min_lambda);
                    self.lambda_vee = S::from_f64_lossy(2.0);
                    terminated = (decrease > S::zero() && decrease < S::from_f64_lossy(1e-6))
                        || step < S::from_f64_lossy(1e-4);
                    break;
                }
                self.lambda *= self.lambda_vee;
                self.lambda_vee *= S::from_f64_lossy(2.0);
                self.ba.restore();
                rejected += 1;
                if self.lambda > self.max_lambda {
                    terminated = true;
                }
            }
        }
        stats.add("optimize", timer.elapsed()).format("ms");
        stats.add("num_it", f64::from(iteration)).format("count");
        stats
            .add("num_it_rejected", f64::from(rejected))
            .format("count");
        self.stats_all.merge_all(&stats);
        self.stats_sums.merge_sums(&stats);
        Ok(())
    }

    /// # Errors
    /// Propagates numerical linearization failures from the optimizer.
    pub fn optimize_and_marg(
        &mut self,
        connected: &BTreeMap<i64, i32>,
        lost: &HashSet<usize>,
        outputs: &VioWorkerOutputs,
    ) -> Result<()> {
        let previous = if outputs.local_map.is_some() {
            super::local_map::LocalMapSnapshot::observation_ids(&self.ba)
        } else {
            BTreeMap::new()
        };
        let diagnostic_before = outputs
            .diagnostics
            .as_ref()
            .map(|_| (self.get_state().cast::<f64>(), self.frame_diagnostics()));
        self.optimize()?;
        if let (Some(queue), Some((before, mut data))) = (&outputs.diagnostics, diagnostic_before) {
            let after = self.get_state().cast::<f64>();
            data.residual_after = self.frame_residuals();
            data.position_delta = (after.state.pose.t_w_i.translation.vector
                - before.state.pose.t_w_i.translation.vector)
                .into();
            data.velocity_delta = (after.state.vel_w_i - before.state.vel_w_i).into();
            data.rotation_delta = (before.state.pose.t_w_i.rotation.inverse()
                * after.state.pose.t_w_i.rotation)
                .scaled_axis()
                .into();
            let _ = queue.send(data);
        }
        if let Some(queue) = &outputs.local_map {
            let mut snapshot = super::local_map::LocalMapSnapshot::capture(
                self.last_state_t_ns,
                &self.ba,
                &self.kf_ids,
                &self.prev_opt_flow_res,
            );
            snapshot.record_rejections(&previous);
            let _ = queue.send(Some(Arc::new(snapshot)));
        }
        self.marginalize(connected, lost, outputs);
        Ok(())
    }

    fn frame_residuals(&self) -> [f64; 4] {
        let flow = &self.prev_opt_flow_res[&self.last_state_t_ns];
        let mut projections = vec![Vec::<Vector4<f64>>::new(); flow.observations.len()];
        self.ba
            .compute_projections(&mut projections, self.last_state_t_ns);
        let mut sums = [0.; 4];
        for (camera, points) in projections.iter().enumerate() {
            for point in points {
                if let Some(observation) = flow.observations[camera].get(&(point[3] as usize))
                    && point[0].is_finite()
                    && point[1].is_finite()
                {
                    let x = f64::from(observation.translation.x) - point[0];
                    let y = f64::from(observation.translation.y) - point[1];
                    sums[0] += 1.;
                    sums[1] += x * x + y * y;
                    sums[2] += x;
                    sums[3] += y;
                }
            }
        }
        if sums[0] > 0. {
            sums[1] = (sums[1] / sums[0]).sqrt();
            sums[2] /= sums[0];
            sums[3] /= sums[0];
        }
        sums
    }

    fn frame_diagnostics(&self) -> VioFrameDiagnostics {
        let flow = &self.prev_opt_flow_res[&self.last_state_t_ns];
        let left = &flow.observations[0];
        let stereo = flow.observations.get(1).map_or(0, |right| {
            left.keys().filter(|id| right.contains_key(id)).count()
        });
        let previous = self
            .prev_opt_flow_res
            .range(..self.last_state_t_ns)
            .next_back();
        let tracked = previous.map_or(0, |(_, previous)| {
            left.keys()
                .filter(|id| previous.observations[0].contains_key(id))
                .count()
        });
        let active_ids: Vec<_> = left
            .keys()
            .filter(|&&id| self.ba.lmdb.landmark_exists(id as i32))
            .copied()
            .collect();
        let added = active_ids
            .iter()
            .filter(|&&id| self.ba.lmdb.landmark(id).host_kf_id.frame_id == self.last_state_t_ns)
            .count();
        VioFrameDiagnostics {
            t_ns: self.last_state_t_ns,
            tracks: [
                left.len() as f64,
                tracked as f64,
                stereo as f64,
                active_ids.len() as f64,
            ],
            graph: [
                self.ba.lmdb.num_landmarks() as f64,
                self.ba.lmdb.num_observations() as f64,
                self.ba.frame_states.len() as f64,
                self.ba.frame_poses.len() as f64,
            ],
            keyframes: [
                f64::from(self.kf_ids.contains(&self.last_state_t_ns)),
                (active_ids.len() - added) as f64,
                added as f64,
                self.kf_ids
                    .first()
                    .map_or(0., |first| (self.last_state_t_ns - first) as f64 * 1e-9),
            ],
            active_ids,
            residual_before: self.frame_residuals(),
            flow: Some(Arc::clone(flow)),
            ..Default::default()
        }
    }

    #[expect(
        clippy::similar_names,
        reason = "Upstream lost-landmark and last-frame arguments"
    )]
    fn marginal_system(
        &self,
        order: &AbsOrderMap,
        prior: &MargLinData<S>,
        keyframes: &BTreeSet<i64>,
        lost: &HashSet<usize>,
        last: i64,
    ) -> (DMatrix<S>, DVector<S>) {
        let imu = ImuLinData {
            g: &self.g,
            gyro_bias_weight_sqrt: &self.gyro_bias_sqrt_weight,
            accel_bias_weight_sqrt: &self.accel_bias_sqrt_weight,
            imu_meas: self
                .imu_meas
                .iter()
                .filter(|(_, m)| {
                    order.abs_order_map.contains_key(&m.start_t_ns())
                        && order
                            .abs_order_map
                            .contains_key(&(m.start_t_ns() + m.dt_ns()))
                })
                .map(|(&t, m)| (t, m))
                .collect(),
        };
        let mut linearization = LinearizationBase::<S, 6>::create(
            &self.ba,
            order,
            self.options(),
            Some(prior),
            Some(&imu),
            Some(keyframes),
            Some(lost),
            last,
        );
        linearization.linearize_problem(&self.ba);
        linearization.perform_qr();
        if is_linearization_sqrt(self.config.vio_linearization_type) && self.marg_data.is_sqrt {
            linearization.get_dense_q2jp_q2r(&self.ba)
        } else {
            linearization.get_dense_h_b(&self.ba)
        }
    }

    fn reduce_marginal(
        &self,
        mut h: DMatrix<S>,
        mut b: DVector<S>,
        keep: &BTreeSet<usize>,
        remove: &BTreeSet<usize>,
    ) -> (DMatrix<S>, DVector<S>) {
        if is_linearization_sqrt(self.config.vio_linearization_type) && self.marg_data.is_sqrt {
            MargHelper::marginalize_sqrt_to_sqrt(&mut h, &mut b, keep, remove)
        } else if self.marg_data.is_sqrt {
            MargHelper::marginalize_sq_to_sqrt(&mut h, &mut b, keep, remove)
        } else {
            MargHelper::marginalize_sq_to_sq(&mut h, &mut b, keep, remove)
        }
    }

    /// # Panics
    /// Requires consistent state/prior ordering and a retained inertial state.
    #[expect(
        clippy::too_many_lines,
        clippy::similar_names,
        reason = "Preserves original keyframe selection and marginalization ordering"
    )]
    pub fn marginalize(
        &mut self,
        connected: &BTreeMap<i64, i32>,
        lost: &HashSet<usize>,
        outputs: &VioWorkerOutputs,
    ) {
        if !self.opt_started {
            return;
        }
        let timer = Timer::default();
        if self.ba.frame_poses.len() > self.max_kfs || self.ba.frame_states.len() >= self.max_states
        {
            let num_remove = self.ba.frame_states.len() as isize - self.max_states as isize + 1;
            let last = *self
                .ba
                .frame_states
                .keys()
                .nth(num_remove.max(0) as usize)
                .expect("marginalization requires retained IMU state");
            let mut order = AbsOrderMap::default();
            let mut poses_remove = BTreeSet::new();
            let mut states_remove = BTreeSet::new();
            let mut velocity_remove = BTreeSet::new();
            for &time in self.ba.frame_poses.keys() {
                let entry = (order.total_size as i32, 6);
                assert_eq!(self.marg_data.order.abs_order_map[&time], entry);
                order.abs_order_map.insert(time, entry);
                order.total_size += 6;
                order.items += 1;
                if !self.kf_ids.contains(&time) {
                    poses_remove.insert(time);
                }
            }
            for &time in self
                .ba
                .frame_states
                .keys()
                .take_while(|&&time| time <= last)
            {
                if time != last {
                    if self.kf_ids.contains(&time) {
                        velocity_remove.insert(time);
                    } else {
                        states_remove.insert(time);
                    }
                }
                let entry = (order.total_size as i32, 15);
                if order.items < self.marg_data.order.abs_order_map.len() {
                    assert_eq!(self.marg_data.order.abs_order_map[&time], entry);
                }
                order.abs_order_map.insert(time, entry);
                order.total_size += 15;
                order.items += 1;
            }
            let all_keyframes = self.kf_ids.clone();
            let mut keyframes_remove = BTreeSet::new();
            while self.kf_ids.len() > self.max_kfs && !velocity_remove.is_empty() {
                let candidates: Vec<_> = self
                    .kf_ids
                    .iter()
                    .take(self.kf_ids.len().saturating_sub(2))
                    .copied()
                    .collect();
                let selected = candidates.iter().copied().find(|time| {
                    !connected.contains_key(time)
                        || (connected[time] as f32 / self.num_points_kf[time] as f32)
                            < self.config.vio_kf_marg_feature_ratio as f32
                });
                let selected = selected
                    .or_else(|| {
                        let latest = *self.kf_ids.last()?;
                        let mut best = None;
                        let mut min = S::from_f64_lossy(f64::MAX);
                        for &first in &candidates {
                            let position = self.ba.frame_poses[&first].pose().translation.vector;
                            let mut denominator = S::zero();
                            for second in &candidates {
                                denominator += S::one()
                                    / ((position
                                        - self.ba.frame_poses[second].pose().translation.vector)
                                        .norm()
                                        + S::from_f64_lossy(1e-5));
                            }
                            let score = (position
                                - self.ba.frame_states[&latest]
                                    .state()
                                    .state
                                    .pose
                                    .t_w_i
                                    .translation
                                    .vector)
                                .norm()
                                .sqrt()
                                * denominator;
                            if score < min {
                                min = score;
                                best = Some(first);
                            }
                        }
                        best
                    })
                    .expect("upstream keyframe marginalization selection failed");
                keyframes_remove.insert(selected);
                poses_remove.insert(selected);
                self.kf_ids.remove(&selected);
            }
            let actual_timer = Timer::default();
            let linear_timer = Timer::default();
            let (h, b) =
                self.marginal_system(&order, &self.marg_data, &keyframes_remove, lost, last);
            self.stats_sums
                .add("marg_linearize", linear_timer.elapsed())
                .format("ms");
            if outputs.has_marginalization_output() && !keyframes_remove.is_empty() {
                let sqrt = is_linearization_sqrt(self.config.vio_linearization_type)
                    && self.marg_data.is_sqrt;
                let abs_h = if sqrt { h.transpose() * &h } else { h.clone() };
                let abs_b = if sqrt { h.transpose() * &b } else { b.clone() };
                let data = MargData {
                    aom: order.clone(),
                    abs_h: abs_h.map(BasaltScalar::as_f64),
                    abs_b: abs_b.map(BasaltScalar::as_f64),
                    frame_states: self
                        .ba
                        .frame_states
                        .iter()
                        .map(|(&t, s)| (t, s.cast()))
                        .collect(),
                    frame_poses: self
                        .ba
                        .frame_poses
                        .iter()
                        .map(|(&t, s)| (t, s.cast()))
                        .collect(),
                    kfs_all: all_keyframes.clone(),
                    kfs_to_marg: keyframes_remove.clone(),
                    use_imu: true,
                    opt_flow_res: all_keyframes
                        .iter()
                        .map(|t| Arc::clone(&self.prev_opt_flow_res[t]))
                        .collect(),
                };
                outputs.send_marginalization(Arc::new(data));
            }
            let mut keep = BTreeSet::new();
            let mut remove = BTreeSet::new();
            for (&time, &(start, size)) in &order.abs_order_map {
                for i in 0..size {
                    let eliminated = if size == 6 {
                        poses_remove.contains(&time)
                    } else {
                        states_remove.contains(&time) || (velocity_remove.contains(&time) && i >= 6)
                    };
                    if eliminated {
                        remove.insert((start + i) as usize);
                    } else {
                        keep.insert((start + i) as usize);
                    }
                }
            }
            if self.config.vio_debug || self.config.vio_extended_logging {
                self.nullspace_marg_data.order = self.marg_data.order.clone();
                let (nh, nb) = self.marginal_system(
                    &order,
                    &self.nullspace_marg_data,
                    &keyframes_remove,
                    lost,
                    last,
                );
                (self.nullspace_marg_data.h, self.nullspace_marg_data.b) =
                    self.reduce_marginal(nh, nb, &keep, &remove);
            }
            let helper_timer = Timer::default();
            let (new_h, new_b) = self.reduce_marginal(h, b, &keep, &remove);
            self.stats_sums
                .add("marg_helper", helper_timer.elapsed())
                .format("ms");
            let state = self.ba.frame_states.get_mut(&last).expect("retained state");
            assert!(!state.is_linearized());
            state.set_lin_true();
            for &time in &states_remove {
                self.ba.frame_states.remove(&time);
                self.imu_meas.remove(&time);
                self.prev_opt_flow_res.remove(&time);
            }
            for &time in &velocity_remove {
                let state = self.ba.frame_states.remove(&time).expect("state to pose");
                self.ba
                    .frame_poses
                    .insert(time, PoseStateWithLin::from_imu_state(&state));
                self.imu_meas.remove(&time);
            }
            for &time in &poses_remove {
                self.ba.frame_poses.remove(&time);
                self.prev_opt_flow_res.remove(&time);
            }
            self.ba
                .lmdb
                .remove_keyframes(&keyframes_remove, &poses_remove, &states_remove);
            if self.config.vio_marg_lost_landmarks {
                for &id in lost {
                    self.ba.lmdb.remove_landmark(id);
                }
            }
            let mut new_order = AbsOrderMap::default();
            for &time in self.ba.frame_poses.keys() {
                new_order
                    .abs_order_map
                    .insert(time, (new_order.total_size as i32, 6));
                new_order.total_size += 6;
                new_order.items += 1;
            }
            new_order
                .abs_order_map
                .insert(last, (new_order.total_size as i32, 15));
            new_order.total_size += 15;
            new_order.items += 1;
            self.marg_data.h = new_h;
            self.marg_data.b = new_b;
            self.marg_data.order = new_order;
            assert_eq!(self.marg_data.h.ncols(), self.marg_data.order.total_size);
            let delta = self.ba.compute_delta(&self.marg_data.order);
            self.marg_data.b -= &self.marg_data.h * &delta;
            if self.config.vio_debug || self.config.vio_extended_logging {
                self.nullspace_marg_data.b -= &self.nullspace_marg_data.h * &delta;
                self.log_marg_nullspace();
            }
            self.stats_sums
                .add("marg", actual_timer.elapsed())
                .format("ms");
        }
        self.stats_sums
            .add("marginalize", timer.elapsed())
            .format("ms");
    }
    pub fn new(gravity: &Vector3<f64>, calibration: &Calibration<f64>, config: &VioConfig) -> Self {
        let calib = calibration.cast::<S>();
        let mut prior = MargLinData {
            is_sqrt: config.vio_sqrt_marg,
            order: AbsOrderMap::default(),
            h: DMatrix::zeros(15, 15),
            b: DVector::zeros(15),
        };
        let nullspace = prior.clone();
        for (indices, weight) in [
            (vec![0, 1, 2], config.vio_init_pose_weight),
            (
                vec![5],
                config
                    .vio_init_yaw_weight
                    .unwrap_or(config.vio_init_pose_weight),
            ),
            (vec![9, 10, 11], config.vio_init_ba_weight),
            (vec![12, 13, 14], config.vio_init_bg_weight),
        ] {
            let weight = S::from_f64_lossy(weight);
            for i in indices {
                prior.h[(i, i)] = if prior.is_sqrt { weight.sqrt() } else { weight };
            }
        }
        Self {
            gyro_bias_sqrt_weight: calib.gyro_bias_std.map(|v| S::one() / v),
            accel_bias_sqrt_weight: calib.accel_bias_std.map(|v| S::one() / v),
            ba: BundleAdjustmentBase::new(
                calib,
                S::from_f64_lossy(config.vio_obs_std_dev),
                S::from_f64_lossy(config.vio_obs_huber_thresh),
            ),
            marg_data: prior,
            nullspace_marg_data: nullspace,
            last_state_t_ns: 0,
            imu_meas: BTreeMap::new(),
            kf_ids: BTreeSet::new(),
            prev_opt_flow_res: BTreeMap::new(),
            num_points_kf: BTreeMap::new(),
            g: gravity.map(S::from_f64_lossy),
            config: config.clone(),
            max_states: config.vio_max_states as usize,
            max_kfs: config.vio_max_kfs as usize,
            t_w_i_init: Isometry3::identity(),
            initialized: false,
            opt_started: false,
            take_kf: true,
            frames_after_kf: 0,
            bg_init: Vector3::zeros(),
            ba_init: Vector3::zeros(),
            lambda: S::from_f64_lossy(config.vio_lm_lambda_initial),
            min_lambda: S::from_f64_lossy(config.vio_lm_lambda_min),
            max_lambda: S::from_f64_lossy(config.vio_lm_lambda_max),
            lambda_vee: S::from_f64_lossy(2.0),
            stats_all: ExecutionStats::default(),
            stats_sums: ExecutionStats::default(),
        }
    }

    fn initialize_at(
        &mut self,
        time: i64,
        pose: Isometry3<S>,
        velocity: Vector3<S>,
        bg: Vector3<S>,
        ba: Vector3<S>,
    ) {
        self.initialized = true;
        self.t_w_i_init = pose;
        self.last_state_t_ns = time;
        self.imu_meas
            .insert(time, IntegratedImuMeasurement::new(time, bg, ba));
        self.ba.frame_states.insert(
            time,
            PoseVelBiasStateWithLin::new(PoseVelBiasState::new(time, pose, velocity, bg, ba), true),
        );
        self.marg_data.order = AbsOrderMap {
            abs_order_map: BTreeMap::from([(time, (0, 15))]),
            items: 1,
            total_size: 15,
        };
        self.nullspace_marg_data.order = self.marg_data.order.clone();
    }

    pub fn get_state(&self) -> &PoseVelBiasState<S> {
        self.ba.frame_states[&self.last_state_t_ns].state()
    }
    pub fn get_state_at(&self, time: i64) -> PoseVelBiasState<S> {
        if let Some(state) = self.ba.frame_states.get(&time) {
            return state.state().clone();
        }
        let mut state = PoseVelBiasState::new(
            0,
            Isometry3::identity(),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        if let Some(pose) = self.ba.frame_poses.get(&time) {
            state.state.pose.t_w_i = *pose.pose();
        }
        state
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
    pub fn get_frame_states(&self) -> Vec<Isometry3<S>> {
        self.ba
            .frame_states
            .values()
            .map(|s| s.state().state.pose.t_w_i)
            .collect()
    }
    pub fn get_frame_poses(&self) -> Vec<Isometry3<S>> {
        self.ba.frame_poses.values().map(|s| *s.pose()).collect()
    }
    pub fn get_all_poses_map(&self) -> BTreeMap<i64, Isometry3<S>> {
        let mut poses: BTreeMap<_, _> = self
            .ba
            .frame_poses
            .iter()
            .map(|(&t, s)| (t, *s.pose()))
            .collect();
        poses.extend(
            self.ba
                .frame_states
                .iter()
                .map(|(&t, s)| (t, s.state().state.pose.t_w_i)),
        );
        poses
    }

    /// # Errors
    /// Propagates optimizer numerical failures.
    /// # Panics
    /// Requires increasing frame timestamps, matching preintegration, valid
    /// calibration camera indices and a consistent initialized local window.
    #[expect(
        clippy::too_many_lines,
        reason = "Retains original association, triangulation and optimization sequence"
    )]
    pub fn measure(
        &mut self,
        flow: &Arc<OpticalFlowResult>,
        measurement: Option<IntegratedImuMeasurement<S>>,
        outputs: &VioWorkerOutputs,
        status: &VioStatus,
    ) -> Result<bool> {
        let timer = Timer::default();
        self.stats_sums
            .add("frame_id", flow.t_ns as f64)
            .format("none");
        if let Some(measurement) = measurement {
            assert_eq!(self.get_t_ns(), measurement.start_t_ns());
            assert_eq!(flow.t_ns, measurement.start_t_ns() + measurement.dt_ns());
            assert!(measurement.dt_ns() > 0);
            let mut next = self.get_state().clone();
            measurement.predict_state(&self.get_state().state, &self.g, &mut next.state);
            self.last_state_t_ns = flow.t_ns;
            next.state.pose.t_ns = flow.t_ns;
            self.ba
                .frame_states
                .insert(flow.t_ns, PoseVelBiasStateWithLin::new(next, false));
            self.imu_meas.insert(measurement.start_t_ns(), measurement);
        }
        self.prev_opt_flow_res.insert(flow.t_ns, Arc::clone(flow));
        let mut connected0 = 0;
        let mut connected = BTreeMap::<i64, i32>::new();
        let mut unconnected = BTreeSet::new();
        for (camera, observations) in flow.observations.iter().enumerate() {
            let target = TimeCamId {
                frame_id: flow.t_ns,
                cam_id: camera,
            };
            for (&id, observation) in observations {
                if self.ba.lmdb.landmark_exists(id as i32) {
                    let host = self.ba.lmdb.landmark(id).host_kf_id;
                    self.ba.lmdb.add_observation(
                        target,
                        &KeypointObservation {
                            kpt_id: id as i32,
                            pos: observation
                                .translation
                                .map(|v| S::from_f64_lossy(f64::from(v))),
                        },
                    );
                    *connected.entry(host.frame_id).or_default() += 1;
                    if camera == 0 {
                        connected0 += 1;
                    }
                } else if camera == 0 {
                    unconnected.insert(id);
                }
            }
        }
        let ratio = S::from_f64_lossy(connected0 as f64)
            / S::from_f64_lossy((connected0 + unconnected.len()) as f64);
        if ratio < S::from_f64_lossy(f64::from(self.config.vio_new_kf_keypoints_thresh))
            && self.frames_after_kf > self.config.vio_min_frames_after_kf
        {
            self.take_kf = true;
        }
        if self.take_kf {
            self.take_kf = false;
            self.frames_after_kf = 0;
            self.kf_ids.insert(self.last_state_t_ns);
            let host = TimeCamId {
                frame_id: flow.t_ns,
                cam_id: 0,
            };
            let mut added = 0;
            for id in unconnected {
                let mut observations = BTreeMap::new();
                for (&time, previous) in &self.prev_opt_flow_res {
                    for (camera, camera_obs) in previous.observations.iter().enumerate() {
                        if let Some(observation) = camera_obs.get(&id) {
                            observations.insert(
                                TimeCamId {
                                    frame_id: time,
                                    cam_id: camera,
                                },
                                KeypointObservation {
                                    kpt_id: id as i32,
                                    pos: observation
                                        .translation
                                        .map(|v| S::from_f64_lossy(f64::from(v))),
                                },
                            );
                        }
                    }
                }
                let mut valid = false;
                for (&target, observation) in &observations {
                    let p0 = flow.observations[0][&id]
                        .translation
                        .map(|v| S::from_f64_lossy(f64::from(v)));
                    let mut f0 = Vector4::zeros();
                    let mut f1 = Vector4::zeros();
                    if !self.ba.calib.intrinsics[0].unproject(&p0, &mut f0, None)
                        || !self.ba.calib.intrinsics[target.cam_id].unproject(
                            &observation.pos,
                            &mut f1,
                            None,
                        )
                    {
                        continue;
                    }
                    let relative = compute_rel_pose(
                        self.ba.get_pose_state_with_lin(target.frame_id).pose(),
                        &self.ba.calib.t_i_c[target.cam_id],
                        self.ba.get_pose_state_with_lin(host.frame_id).pose(),
                        &self.ba.calib.t_i_c[0],
                        None,
                        None,
                    );
                    if relative.translation.vector.norm_squared()
                        < S::from_f64_lossy(self.config.vio_min_triangulation_dist.powi(2))
                    {
                        continue;
                    }
                    let point = BundleAdjustmentBase::triangulate(
                        &f0.fixed_rows::<3>(0).into_owned(),
                        &f1.fixed_rows::<3>(0).into_owned(),
                        &relative,
                    );
                    if point.iter().all(nalgebra::ComplexField::is_finite)
                        && point[3] > S::zero()
                        && point[3] < S::from_f64_lossy(3.0)
                    {
                        self.ba.lmdb.add_landmark(
                            id,
                            &Keypoint::new(
                                StereographicParam::project(&point, None),
                                point[3],
                                host,
                            ),
                        );
                        added += 1;
                        valid = true;
                        break;
                    }
                }
                if valid {
                    for (target, observation) in observations {
                        self.ba.lmdb.add_observation(target, &observation);
                    }
                }
            }
            self.num_points_kf.insert(flow.t_ns, added);
        } else {
            self.frames_after_kf += 1;
        }
        let lost: HashSet<usize> = if self.config.vio_marg_lost_landmarks {
            self.ba
                .lmdb
                .landmarks()
                .keys()
                .filter(|&&id| {
                    !flow
                        .observations
                        .iter()
                        .any(|camera| camera.contains_key(&id))
                })
                .copied()
                .collect()
        } else {
            HashSet::new()
        };
        self.max_states = status.max_states.load(Ordering::Relaxed);
        self.max_kfs = status.max_kfs.load(Ordering::Relaxed);
        self.optimize_and_marg(&connected, &lost, outputs)?;
        if outputs.state.is_some() || outputs.state_bounded.is_some() {
            outputs.send_state(Arc::new(self.get_state().cast::<f64>()));
        }
        if let Some(queue) = &outputs.vis {
            let mut data = VioVisualizationData {
                t_ns: self.last_state_t_ns,
                states: self
                    .ba
                    .frame_states
                    .values()
                    .map(|s| s.state().cast::<f64>().state.pose.t_w_i)
                    .collect(),
                frames: self
                    .ba
                    .frame_poses
                    .values()
                    .map(|s| *s.cast::<f64>().pose())
                    .collect(),
                opt_flow_res: Some(Arc::clone(flow)),
                projections: vec![Vec::new(); flow.observations.len()],
                ..Default::default()
            };
            self.ba
                .get_current_points(&mut data.points, &mut data.point_ids);
            self.ba
                .compute_projections(&mut data.projections, self.last_state_t_ns);
            let _ = queue.send(Some(Arc::new(data)));
        }
        status
            .last_processed_t_ns
            .store(self.last_state_t_ns, Ordering::Release);
        self.stats_sums.add("measure", timer.elapsed()).format("ms");
        Ok(true)
    }
}

impl<S: BasaltScalar> VioEstimatorEngine for SqrtKeypointVioEstimator<S> {
    fn debug_finalize(&self, directory: &std::path::Path) -> Result<()> {
        Self::debug_finalize(self, directory)
    }
    fn get_state_at(&self, time: i64) -> PoseVelBiasState<f64> {
        Self::get_state_at(self, time).cast::<f64>()
    }
    fn get_all_poses_map(&self) -> BTreeMap<i64, Isometry3<f64>> {
        Self::get_all_poses_map(self)
            .into_iter()
            .map(|(time, pose)| (time, PoseState::new(time, pose).cast::<f64>().t_w_i))
            .collect()
    }
    fn initialize_state(
        &mut self,
        time: i64,
        pose: Isometry3<f64>,
        velocity: Vector3<f64>,
        bg: Vector3<f64>,
        ba: Vector3<f64>,
    ) {
        self.initialize_at(
            time,
            PoseState::new(time, pose).cast::<S>().t_w_i,
            velocity.map(S::from_f64_lossy),
            bg.map(S::from_f64_lossy),
            ba.map(S::from_f64_lossy),
        );
        self.initialize_bias(bg, ba);
    }
    fn initialize_bias(&mut self, bg: Vector3<f64>, ba: Vector3<f64>) {
        self.bg_init = bg.map(S::from_f64_lossy);
        self.ba_init = ba.map(S::from_f64_lossy);
    }
    fn run(
        &mut self,
        inputs: VioWorkerInputs,
        outputs: VioWorkerOutputs,
        status: Arc<VioStatus>,
    ) -> Result<()> {
        self.process_stream(inputs, outputs, status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{optical_flow::AffineCompact2, utils::vio_config::LinearizationType};
    use firefly_calibration::camera::generic_camera::GenericCamera;
    use nalgebra::{Matrix2, Vector2};

    fn initialization_oracle<S: BasaltScalar>(key: &str, tolerance: f64) {
        let reference: serde_json::Value =
            serde_json::from_str(include_str!("../../tests/reference/initialization.json"))
                .unwrap();
        for (i, input) in [
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
            [0.000_001, 0.0, -1.0],
            [0.0001, -0.0003, -1.0],
            [1.0, 2.0, 3.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, -1e8],
        ]
        .into_iter()
        .enumerate()
        {
            let input = Vector3::from(input).map(S::from_f64_lossy);
            let rotation = rotation_from_two_vectors(&input, &Vector3::z());
            for (j, value) in rotation.quaternion().coords.iter().enumerate() {
                let expected = reference[key][i][j].as_f64().unwrap();
                assert!(
                    (value.as_f64() - expected).abs() < tolerance,
                    "{key} case {i} coord {j}: {} vs {expected}",
                    value.as_f64()
                );
            }
        }
    }

    #[test]
    fn gravity_alignment_matches_eigen_including_opposite_and_zero() {
        initialization_oracle::<f32>("f32", 1e-5);
        initialization_oracle::<f64>("f64", 1e-10);
    }

    #[test]
    #[expect(
        clippy::too_many_lines,
        reason = "Analytic stereo/IMU fixture checks state and diagnostic outputs together"
    )]
    fn stationary_stereo_imu_runs_optimization_and_marginalization() {
        for kind in [
            LinearizationType::AbsQr,
            LinearizationType::AbsSc,
            LinearizationType::RelSc,
        ] {
            let mut camera = GenericCamera::from_name("pinhole");
            camera.set_from_init(Vector4::new(300.0, 300.0, 320.0, 240.0));
            let calib = Calibration::<f64> {
                t_i_c: vec![Isometry3::identity(), Isometry3::translation(0.2, 0.0, 0.0)],
                intrinsics: vec![camera.clone(), camera],
                ..Default::default()
            };
            let config = VioConfig {
                vio_max_states: 3,
                vio_max_kfs: 3,
                vio_linearization_type: kind,
                ..Default::default()
            };
            let mut estimator = SqrtKeypointVioEstimator::<f64>::new(
                &Vector3::new(0.0, 0.0, -9.81),
                &calib,
                &config,
            );
            let start = 1_000_000_000;
            estimator.initialize_at(
                start,
                Isometry3::identity(),
                Vector3::zeros(),
                Vector3::zeros(),
                Vector3::zeros(),
            );
            let status = VioStatus::new(3, 3);
            let (diagnostics, recorded) = std::sync::mpsc::channel();
            let outputs = VioWorkerOutputs {
                diagnostics: Some(diagnostics),
                ..Default::default()
            };
            for frame in 0..9 {
                let time = start + frame * 20_000_000;
                let observations = (0..2)
                    .map(|camera| {
                        (0..24)
                            .map(|id| {
                                let world = Vector3::new(
                                    (id % 6) as f64 * 0.2 - 0.5,
                                    (id / 6) as f64 * 0.2 - 0.3,
                                    3.0 + (id % 3) as f64 * 0.2,
                                );
                                let local = world - calib.t_i_c[camera].translation.vector;
                                (
                                    id,
                                    AffineCompact2 {
                                        linear: Matrix2::identity(),
                                        translation: Vector2::new(
                                            (300.0 * local.x / local.z + 320.0) as f32,
                                            (300.0 * local.y / local.z + 240.0) as f32,
                                        ),
                                    },
                                )
                            })
                            .collect()
                    })
                    .collect();
                let flow = Arc::new(OpticalFlowResult {
                    t_ns: time,
                    observations,
                    pyramid_levels: Vec::new(),
                    input_images: None,
                });
                let measurement = if frame == 0 {
                    None
                } else {
                    let mut measurement = IntegratedImuMeasurement::new(
                        time - 20_000_000,
                        Vector3::zeros(),
                        Vector3::zeros(),
                    );
                    for sample in 1..=4 {
                        measurement.integrate(
                            &ImuData {
                                t_ns: time - 20_000_000 + sample * 5_000_000,
                                accel: Vector3::new(0.0, 0.0, 9.81),
                                gyro: Vector3::zeros(),
                            },
                            &Vector3::repeat(0.01),
                            &Vector3::repeat(0.001),
                        );
                    }
                    Some(measurement)
                };
                estimator
                    .measure(&flow, measurement, &outputs, &status)
                    .unwrap();
                let data = recorded.try_recv().unwrap();
                assert_eq!(data.t_ns, time);
                assert_eq!(
                    data.tracks,
                    [24., if frame == 0 { 0. } else { 24. }, 24., 24.]
                );
                assert_eq!(data.residual_before[0], 48.);
                assert_eq!(data.residual_after[0], 48.);
                assert!(data.residual_before[1] < 1e-3);
                assert!(data.residual_after[1] < 1e-3);
                assert!(Vector3::from(data.position_delta).norm() < 1e-5);
                assert!(Vector3::from(data.velocity_delta).norm() < 1e-4);
            }
            assert!(estimator.opt_started);
            assert!(estimator.ba.frame_states.len() < 3);
            assert_eq!(estimator.ba.lmdb.num_landmarks(), 24);
            assert!(estimator.get_t_w_i().translation.vector.norm() < 1e-5);
            assert!(estimator.marg_data.h.iter().all(|x| x.is_finite()));
        }
    }
}
