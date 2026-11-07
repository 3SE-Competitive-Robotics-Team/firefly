// SPDX-License-Identifier: BSD-3-Clause
//! Read-only observations and objective terms from the live Basalt optimizer.

use super::SqrtKeypointVioEstimator;
use crate::{
    imu::imu_types::PoseVelBiasState,
    utils::{
        ba_utils::{compute_rel_pose, linearize_point},
        imu_types::AbsOrderMap,
    },
    vi_estimator::vio_estimator::VioPointDiagnostics,
};
use firefly_calibration::{
    camera::stereographic_param::StereographicParam, observation::TimeCamId, scalar::BasaltScalar,
};
use nalgebra::Vector2;
use serde_json::{Value, json};
use std::collections::BTreeSet;

pub(super) fn enabled() -> bool {
    log::log_enabled!(target: "vio-diagnostics", log::Level::Debug)
}

pub(super) struct FrameObservations {
    pub points: Vec<VioPointDiagnostics>,
    pub support: [f64; 4],
    pub effective_left: Vec<usize>,
    pub stereo_effective: usize,
    pub residual: [f64; 4],
}

impl<S: BasaltScalar> SqrtKeypointVioEstimator<S> {
    pub(super) fn observation_diagnostics(&self, include_points: bool) -> FrameObservations {
        let flow = &self.prev_opt_flow_res[&self.last_state_t_ns];
        let mut result = FrameObservations {
            points: Vec::new(),
            support: [0.; 4],
            effective_left: Vec::new(),
            stereo_effective: 0,
            residual: [0.; 4],
        };
        let mut effective = vec![BTreeSet::new(); flow.observations.len()];
        result.support[0] = flow
            .observations
            .first()
            .map_or(0., |left| left.len() as f64);
        for (camera, observations) in flow.observations.iter().enumerate() {
            for (index, (&id, observed)) in observations.iter().enumerate() {
                let data = self.point_diagnostic(
                    id,
                    camera,
                    index,
                    observed.translation.map(f64::from).into(),
                );
                if camera == 0 && data.flags & 1 != 0 {
                    result.support[1] += 1.;
                }
                if data.flags & 8 != 0 {
                    effective[camera].insert(id);
                    if camera == 0 {
                        result.support[2] += 1.;
                        if data.flags & 2 != 0 {
                            result.support[3] += 1.;
                        }
                    }
                }
                if data.flags & 4 != 0 {
                    result.residual[0] += 1.;
                    result.residual[1] += data.residual[0].powi(2) + data.residual[1].powi(2);
                    result.residual[2] += data.residual[0];
                    result.residual[3] += data.residual[1];
                }
                if include_points {
                    result.points.push(data);
                }
            }
        }
        if result.residual[0] > 0. {
            result.residual[1] = (result.residual[1] / result.residual[0]).sqrt();
            result.residual[2] /= result.residual[0];
            result.residual[3] /= result.residual[0];
        }
        if let Some(left) = effective.first() {
            result.effective_left = left.iter().copied().collect();
            result.stereo_effective = effective
                .get(1)
                .map_or(0, |right| left.intersection(right).count());
        }
        result
    }

    fn point_diagnostic(
        &self,
        id: usize,
        camera: usize,
        index: usize,
        pixel: [f64; 2],
    ) -> VioPointDiagnostics {
        let mut data = VioPointDiagnostics {
            track_id: id as u64,
            camera: camera as u32,
            keypoint_index: index as u32,
            flags: 0,
            pixel,
            depth_z: f64::NAN,
            range: f64::NAN,
            inverse_range: f64::NAN,
            residual: [f64::NAN; 2],
            whitened_squared: f64::NAN,
            sqrt_information: [[f64::NAN; 2]; 2],
            robust_derivative: f64::NAN,
            robust_cost: f64::NAN,
        };
        let Some(point) = self.ba.lmdb.landmarks().get(&id) else {
            return data;
        };
        let host = point.host_kf_id;
        let target = TimeCamId {
            frame_id: self.last_state_t_ns,
            cam_id: camera,
        };
        let in_graph = self
            .ba
            .lmdb
            .observations()
            .get(&host)
            .and_then(|targets| targets.get(&target))
            .is_some_and(|ids| ids.contains(&id));
        let transform = compute_rel_pose(
            self.ba.get_pose_state_with_lin(host.frame_id).pose(),
            &self.ba.calib.t_i_c[host.cam_id],
            self.ba.get_pose_state_with_lin(target.frame_id).pose(),
            &self.ba.calib.t_i_c[target.cam_id],
            None,
            None,
        )
        .to_homogeneous();
        let mut ray = StereographicParam::unproject(&point.direction, None);
        ray[3] = point.inv_dist;
        let local = transform * ray;
        let inverse = point.inv_dist.as_f64();
        if inverse.is_finite() && inverse > 0. {
            data.depth_z = (local[2] / point.inv_dist).as_f64();
            data.range = (local.fixed_rows::<3>(0).norm() / point.inv_dist).as_f64();
            data.inverse_range = 1. / data.range;
            if data.depth_z.is_finite() && data.depth_z > 0. {
                data.flags |= 2;
            }
        } else if inverse == 0. {
            data.inverse_range = 0.;
        }
        if !in_graph {
            return data;
        }
        data.flags |= 1;
        let mut residual = Vector2::zeros();
        if linearize_point(
            &point.obs[&target],
            point,
            &transform,
            &self.ba.calib.intrinsics[camera],
            &mut residual,
            None,
            None,
            None,
        ) {
            fill_factor(
                &mut data,
                &residual,
                self.ba.obs_std_dev,
                self.ba.huber_thresh,
            );
        }
        data
    }

    pub(super) fn diagnostic_cost(&self, order: &AbsOrderMap) -> Value {
        let (imu, gyro, accel) = super::super::sc_ba_base::compute_imu_error(
            order,
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
        let visual = self.ba.compute_error(None, S::zero()).as_f64();
        let marginalization = self.ba.compute_marg_prior_error(&self.marg_data).as_f64();
        json!({"visual": visual, "imu": imu.as_f64(), "gyro_bias": gyro.as_f64(),
            "accel_bias": accel.as_f64(), "marginalization": marginalization,
            "total": visual + marginalization + imu.as_f64() + gyro.as_f64() + accel.as_f64()})
    }
}

fn fill_factor<S: BasaltScalar>(
    data: &mut VioPointDiagnostics,
    residual: &Vector2<S>,
    sigma: S,
    huber: S,
) {
    data.flags |= 4;
    data.residual = (-residual).map(BasaltScalar::as_f64).into();
    let squared = residual.norm_squared();
    let weight = if huber > S::zero() && squared > huber * huber {
        huber / squared.sqrt()
    } else {
        S::one()
    };
    let whitening = (S::one() / sigma).as_f64();
    data.sqrt_information = [[whitening, 0.], [0., whitening]];
    data.whitened_squared = (squared / (sigma * sigma)).as_f64();
    data.robust_derivative = weight.as_f64();
    data.robust_cost =
        (S::from_f64_lossy(0.5) * (S::from_f64_lossy(2.) - weight) * weight * squared
            / (sigma * sigma))
            .as_f64();
    if whitening.is_finite()
        && whitening > 0.
        && data.robust_derivative.is_finite()
        && data.robust_derivative > 0.
    {
        data.flags |= 8;
    }
}

/// Solver records are formatted after solving; every record is bounded by the IPC text limit.
pub(super) struct SolverDiagnostics {
    frame_ns: i64,
    pub records: Vec<Value>,
}

impl SolverDiagnostics {
    pub fn new(frame_ns: i64) -> Option<Self> {
        enabled().then(|| Self {
            frame_ns,
            records: Vec::new(),
        })
    }

    pub fn push(&mut self, kind: &str, mut record: Value) {
        record["schema"] = json!(1);
        record["backend"] = json!("rootslam");
        record["frame_ns"] = json!(self.frame_ns);
        record["kind"] = json!(kind);
        self.records.push(record);
    }

    pub fn push_state(&mut self, phase: &str, state: &PoseVelBiasState<f64>) {
        let pose = &state.state.pose.t_w_i;
        let quaternion = pose.rotation.quaternion();
        self.push(
            "state_pose",
            json!({"stage":phase,
            "position":pose.translation.vector.as_slice(),
            "quaternion_xyzw":[quaternion.i, quaternion.j, quaternion.k, quaternion.w]}),
        );
        self.push(
            "state_inertial",
            json!({"stage":phase,
            "velocity":state.state.vel_w_i.as_slice(),
            "gyro_bias":state.bias_gyro.as_slice(), "accel_bias":state.bias_accel.as_slice()}),
        );
    }

    pub fn finish(self, reason: &str, iterations: i32, rejected: i32) {
        let mut records = self;
        records.push(
            "stop",
            json!({"reason": reason, "iterations": iterations, "rejected": rejected}),
        );
        records.emit();
    }

    fn emit(self) {
        for text in self.messages() {
            log::debug!(target: "vio-diagnostics", "{text}");
        }
    }

    fn messages(self) -> Vec<String> {
        let total = self.records.len();
        self.records
            .into_iter()
            .enumerate()
            .map(|(index, mut record)| {
                record["ri"] = json!(index);
                record["rn"] = json!(total);
                let text = record.to_string();
                assert!(
                    text.len() <= 512,
                    "VIO solver diagnostic exceeds log IPC capacity"
                );
                text
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        optical_flow::{AffineCompact2, OpticalFlowResult},
        utils::vio_config::VioConfig,
        vi_estimator::landmark_database::{Keypoint, KeypointObservation},
    };
    use firefly_calibration::{calibration::Calibration, camera::generic_camera::GenericCamera};
    use nalgebra::{Isometry3, Matrix2, Vector3, Vector4};
    use std::{collections::BTreeMap, sync::Arc};

    fn fixture() -> SqrtKeypointVioEstimator<f64> {
        let mut camera = GenericCamera::from_name("pinhole");
        camera.set_from_init(Vector4::new(300., 300., 320., 240.));
        let calibration = Calibration {
            t_i_c: vec![Isometry3::identity(); 2],
            intrinsics: vec![camera.clone(), camera],
            ..Default::default()
        };
        let mut estimator = SqrtKeypointVioEstimator::new(
            &Vector3::new(0., 0., -9.81),
            &calibration,
            &VioConfig::default(),
        );
        estimator.initialize_at(
            1,
            Isometry3::identity(),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let host = TimeCamId {
            frame_id: 1,
            cam_id: 0,
        };
        for (id, direction, inverse) in [
            (0, Vector2::zeros(), 1. / 3.),
            (1, Vector2::zeros(), 0.),
            (2, Vector2::new(2., 0.), 1. / 3.),
            (3, Vector2::zeros(), 1. / 3.),
        ] {
            estimator
                .ba
                .lmdb
                .add_landmark(id, &Keypoint::new(direction, inverse, host));
        }
        let mut observations = vec![BTreeMap::new(), BTreeMap::new()];
        for (camera, points) in observations.iter_mut().enumerate() {
            for id in 0..if camera == 0 { 5 } else { 1 } {
                let pixel = Vector2::new(if id == 0 && camera == 0 { 322. } else { 320. }, 240.);
                points.insert(
                    id,
                    AffineCompact2 {
                        linear: Matrix2::identity(),
                        translation: pixel.map(|v| v as f32),
                    },
                );
                if id < 3 {
                    estimator.ba.lmdb.add_observation(
                        TimeCamId {
                            frame_id: 1,
                            cam_id: camera,
                        },
                        &KeypointObservation {
                            kpt_id: id as i32,
                            pos: pixel,
                        },
                    );
                }
            }
        }
        estimator.prev_opt_flow_res.insert(
            1,
            Arc::new(OpticalFlowResult {
                t_ns: 1,
                observations,
                pyramid_levels: Vec::new(),
                input_images: None,
            }),
        );
        estimator
    }

    #[test]
    fn effective_support_keeps_direction_factors_and_rejects_invalid_projections() {
        let estimator = fixture();
        let report = estimator.observation_diagnostics(true);
        assert_eq!(report.support, [5., 3., 2., 1.]);
        assert_eq!(report.effective_left, [0, 1]);
        assert_eq!(report.stereo_effective, 1);
        let finite = &report.points[0];
        assert_eq!(finite.flags, 15);
        assert_eq!(finite.depth_z, 3.);
        assert_eq!(finite.range, 3.);
        assert_eq!(finite.residual, [2., 0.]);
        assert_eq!(finite.sqrt_information, [[2., 0.], [0., 2.]]);
        assert_eq!(finite.whitened_squared, 16.);
        assert_eq!(finite.robust_derivative, 0.5);
        assert_eq!(finite.robust_cost, 6.);
        let direction = &report.points[1];
        assert_eq!(direction.flags, 1 | 4 | 8);
        assert_eq!(direction.inverse_range, 0.);
        assert!(direction.depth_z.is_nan());
        let invalid = &report.points[2];
        assert_eq!(invalid.flags, 1);
        assert!(invalid.robust_cost.is_nan());
        assert_eq!(report.points[3].flags, 2);
        assert!(report.points[3].whitened_squared.is_nan());
        assert_eq!(report.points[4].flags, 0);
        assert_eq!(report.residual[0], 3.);
        assert!((report.residual[1] - (4_f64 / 3.).sqrt()).abs() < 1e-12);
    }

    #[test]
    fn cost_breakdown_matches_analytic_huber_and_existing_objective() {
        let estimator = fixture();
        let report = estimator.diagnostic_cost(&estimator.full_order());
        assert_eq!(report["visual"], 6.);
        assert_eq!(report["total"], 6.);
        for term in ["imu", "gyro_bias", "accel_bias", "marginalization"] {
            assert_eq!(report[term], 0.);
        }
        let points = estimator.observation_diagnostics(true);
        let sum: f64 = points
            .points
            .iter()
            .map(|point| point.robust_cost)
            .filter(|value| value.is_finite())
            .sum();
        assert_eq!(sum, estimator.ba.compute_error(None, 0.));
    }

    #[test]
    fn diagnostic_records_fit_log_transport_and_keep_nonfinite_values_missing() {
        let mut diagnostic = SolverDiagnostics {
            frame_ns: i64::MAX,
            records: Vec::new(),
        };
        let wide = -1.234_567_890_123_456_7e-123;
        diagnostic.push(
            "cost",
            json!({"stage":"trial", "iteration":i32::MAX, "accepted":false,
            "visual":wide, "imu":wide, "gyro_bias":wide, "accel_bias":wide,
            "marginalization":wide, "total":wide}),
        );
        diagnostic.push(
            "iteration",
            json!({"iteration":i32::MAX, "accepted":false,
            "linearized_before":wide, "cost_after":wide, "model_decrease":wide,
            "actual_decrease":wide, "step_inf":wide, "lambda":wide}),
        );
        diagnostic.push(
            "stop",
            json!({"reason":"numerical_failure", "iterations":i32::MAX, "rejected":i32::MAX}),
        );
        diagnostic.push("cost", json!({"stage":"after", "total":f64::NAN}));
        diagnostic.push("state_pose", json!({"stage":"before_opt", "position":[wide,wide,wide], "quaternion_xyzw":[wide,wide,wide,wide]}));
        diagnostic.push("state_inertial", json!({"stage":"before_opt", "velocity":[wide,wide,wide], "gyro_bias":[wide,wide,wide], "accel_bias":[wide,wide,wide]}));
        diagnostic.push(
            "iteration_pose",
            json!({"iteration":i32::MAX, "accepted":true, "position_delta":[wide,wide,wide]}),
        );
        assert!(diagnostic.records[3]["total"].is_null());
        let messages = diagnostic.messages();
        for (index, text) in messages.iter().enumerate() {
            assert!(text.len() <= 512, "{text}");
            let record: Value = serde_json::from_str(text).unwrap();
            assert_eq!(record["ri"], index);
            assert_eq!(record["rn"], messages.len());
        }
    }
}
