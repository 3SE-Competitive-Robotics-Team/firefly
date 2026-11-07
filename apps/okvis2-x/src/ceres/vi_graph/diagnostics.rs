//! Read-only numerical evidence collected alongside the production solver.

pub const GRAPH: u32 = 1;
pub const DEPTH_READY: u32 = 2;
pub const PROJECTABLE: u32 = 4;
pub const EFFECTIVE: u32 = 8;

#[derive(Clone, Debug)]
pub struct ObservationDiagnostic {
    pub track_id: u64,
    pub camera: u32,
    pub keypoint_index: u32,
    pub flags: u32,
    pub pixel: [f64; 2],
    pub depth_z: f64,
    pub range: f64,
    pub inverse_range: f64,
    pub residual: [f64; 2],
    pub whitened_squared: f64,
    pub sqrt_information: [[f64; 2]; 2],
    pub robust_derivative: f64,
    pub robust_cost: f64,
}

#[must_use]
pub fn residual_summary(points: &[ObservationDiagnostic]) -> [f64; 4] {
    let mut sums = [0.0; 4];
    for point in points.iter().filter(|p| p.flags & EFFECTIVE != 0) {
        sums[0] += 1.0;
        sums[1] += point.residual[0].powi(2) + point.residual[1].powi(2);
        sums[2] += point.residual[0];
        sums[3] += point.residual[1];
    }
    if sums[0] > 0.0 {
        sums[1] = (sums[1] / sums[0]).sqrt();
        sums[2] /= sums[0];
        sums[3] /= sums[0];
    }
    sums
}

impl super::ViGraph {
    /// Reads support and weights from the registered optimization observations.
    /// # Errors
    /// Returns an error if a referenced pose, extrinsic or landmark estimate is absent.
    pub fn frame_observations(
        &self,
        frame: &super::MultiFrame,
    ) -> super::Result<Vec<ObservationDiagnostic>> {
        use super::{GraphFactor, KeypointIdentifier, LandmarkId, LossFunction, StateId, Vector2};
        use crate::cameras::camera_base::ProjectionStatus;
        let state = StateId::new(frame.id());
        let pose = self.pose(state)?;
        let mut points = Vec::with_capacity(frame.total_keypoints());
        for camera in 0..frame.num_frames() {
            let camera_from_world = pose.compose(&self.extrinsics(state, camera)?).inverse();
            for index in 0..frame.num_keypoints(camera) {
                let track_id = frame.landmark_id(camera, index);
                let landmark = LandmarkId::new(track_id);
                let mut pixel = Vector2::repeat(f64::NAN);
                frame.get_keypoint(camera, index, &mut pixel);
                let mut sample = ObservationDiagnostic {
                    track_id,
                    camera: camera as u32,
                    keypoint_index: index as u32,
                    flags: 0,
                    pixel: pixel.into(),
                    depth_z: f64::NAN,
                    range: f64::NAN,
                    inverse_range: f64::NAN,
                    residual: [f64::NAN; 2],
                    whitened_squared: f64::NAN,
                    sqrt_information: [[f64::NAN; 2]; 2],
                    robust_derivative: f64::NAN,
                    robust_cost: f64::NAN,
                };
                if track_id != 0 && self.is_landmark_added(landmark) {
                    let hp = camera_from_world.transform_homogeneous(&self.landmark(landmark)?);
                    if hp.w != 0.0 {
                        let p = hp.fixed_rows::<3>(0) / hp.w;
                        sample.depth_z = p.z;
                        sample.range = p.norm();
                        sample.inverse_range = 1.0 / sample.range;
                    }
                    if self.is_landmark_initialised(landmark)?
                        && sample.depth_z.is_finite()
                        && sample.depth_z > 0.0
                    {
                        sample.flags |= DEPTH_READY;
                    }
                    let mut predicted = Vector2::repeat(f64::NAN);
                    if frame
                        .geometry(camera)
                        .project_homogeneous(&hp, &mut predicted)
                        == ProjectionStatus::Successful
                        && predicted.iter().all(|x| x.is_finite())
                    {
                        sample.flags |= PROJECTABLE;
                        sample.residual = (pixel - predicted).into();
                    }
                    let key = KeypointIdentifier::new(frame.id(), camera, index);
                    if let Some(observation) = self.observations.get(&key)
                        && observation.core.landmark_id == landmark
                        && let Some(record) = self.problem.residuals.get(&observation.residual)
                        && let GraphFactor::Reprojection(factor) = record.factor.as_ref()
                    {
                        sample.flags |= GRAPH;
                        let factor = factor.borrow();
                        if let Some(information) = factor.information() {
                            let whitening =
                                crate::ceres::pose_error::square_root_information(*information);
                            sample.sqrt_information = [
                                [whitening[(0, 0)], whitening[(0, 1)]],
                                [whitening[(1, 0)], whitening[(1, 1)]],
                            ];
                            if sample.flags & PROJECTABLE != 0 {
                                let raw =
                                    factor.measurement().copied().unwrap_or(pixel) - predicted;
                                sample.residual = raw.into();
                                let squared = (whitening * raw).norm_squared();
                                sample.whitened_squared = squared;
                                let rho = record
                                    .loss
                                    .map_or([squared, 1.0, 0.0], |loss| loss.evaluate(squared));
                                sample.robust_derivative = rho[1];
                                sample.robust_cost = 0.5 * rho[0];
                                if whitening.iter().all(|x| x.is_finite())
                                    && whitening.norm_squared() > 0.0
                                    && rho[1].is_finite()
                                    && rho[1] > 0.0
                                    && squared.is_finite()
                                {
                                    sample.flags |= EFFECTIVE;
                                }
                            }
                        }
                    }
                }
                points.push(sample);
            }
        }
        Ok(points)
    }
}

/// Costs include constant factors and use the production robust objective.
#[derive(Clone, Copy, Debug, Default, serde::Serialize)]
pub struct FactorCosts {
    pub visual: f64,
    pub imu: f64,
    /// OKVIS condensed two-pose visual factors, rather than a VI marginal prior.
    pub marginalization: f64,
    pub prior: f64,
}

#[derive(Clone, Debug)]
pub struct PoseStep {
    /// Internal parameter identity; never serialized into the recording.
    pub key: usize,
    pub translation: [f64; 3],
    pub rotation: [f64; 3],
}

#[derive(Clone, Debug)]
pub struct SolverIteration {
    pub iteration: usize,
    pub before: FactorCosts,
    pub candidate: Option<FactorCosts>,
    pub accepted: bool,
    pub reason: &'static str,
    pub predicted_decrease: Option<f64>,
    pub quality: Option<f64>,
    pub pose_steps: Vec<PoseStep>,
}

pub enum FrameEvent {
    Solve {
        frame_ns: i128,
        solve: &'static str,
        pose_key: usize,
        summary: super::SolverSummary,
    },
    State {
        frame_ns: i128,
        stage: &'static str,
        position: [f64; 3],
        quaternion_xyzw: [f64; 4],
        speed_bias: [f64; 9],
    },
}

pub fn log_frame_events(events: &[FrameEvent]) {
    for record in frame_records(events) {
        emit(&record);
    }
}

fn frame_records(events: &[FrameEvent]) -> Vec<serde_json::Value> {
    let mut records = Vec::new();
    let mut solve_index = 0;
    for (sequence, event) in events.iter().enumerate() {
        match event {
            FrameEvent::Solve {
                frame_ns,
                solve,
                pose_key,
                summary,
            } => {
                for mut record in solver_records(*frame_ns, solve, *pose_key, summary) {
                    record["solve_index"] = serde_json::json!(solve_index);
                    records.push(record);
                }
                solve_index += 1;
            }
            FrameEvent::State {
                frame_ns,
                stage,
                position,
                quaternion_xyzw,
                speed_bias,
            } => {
                for record in state_records(
                    *frame_ns,
                    stage,
                    sequence,
                    position,
                    quaternion_xyzw,
                    speed_bias,
                ) {
                    records.push(record);
                }
            }
        }
    }
    let count = records.len();
    for (index, record) in records.iter_mut().enumerate() {
        record["ri"] = serde_json::json!(index);
        record["rn"] = serde_json::json!(count);
    }
    records
}

fn state_records(
    frame_ns: i128,
    stage: &str,
    sequence: usize,
    position: &[f64; 3],
    quaternion_xyzw: &[f64; 4],
    speed_bias: &[f64; 9],
) -> [serde_json::Value; 2] {
    [
        serde_json::json!({
            "kind":"state_pose", "solver":"okvis", "frame_ns":frame_ns,
            "stage":stage, "sequence":sequence, "position":position, "quaternion_xyzw":quaternion_xyzw,
        }),
        serde_json::json!({
            "kind":"state_inertial", "solver":"okvis", "frame_ns":frame_ns,
            "stage":stage, "sequence":sequence, "velocity":&speed_bias[..3],
            "gyro_bias":&speed_bias[3..6], "accel_bias":&speed_bias[6..],
        }),
    ]
}

fn emit(value: &serde_json::Value) {
    let message = value.to_string();
    debug_assert!(message.len() < 512, "diagnostic exceeds IPC log capacity");
    log::debug!(target: "vio-diagnostics", "{message}");
}

fn cost_record(
    frame_ns: i128,
    solve: &str,
    iteration: usize,
    stage: &str,
    cost: FactorCosts,
) -> serde_json::Value {
    serde_json::json!({
        "kind": "solver_cost", "solver": "okvis", "frame_ns": frame_ns,
        "solve": solve, "iteration": iteration, "stage": stage,
        "visual": cost.visual, "imu": cost.imu,
        "condensed_visual": cost.marginalization, "prior": cost.prior,
        "total": cost.visual + cost.imu + cost.marginalization + cost.prior,
    })
}

fn solver_records(
    frame_ns: i128,
    solve: &str,
    pose_key: usize,
    summary: &super::SolverSummary,
) -> Vec<serde_json::Value> {
    let mut records = Vec::new();
    if let Some(cost) = summary.diagnostic_initial {
        records.push(cost_record(frame_ns, solve, 0, "initial", cost));
    }
    for entry in &summary.diagnostics {
        records.push(cost_record(
            frame_ns,
            solve,
            entry.iteration,
            "before",
            entry.before,
        ));
        if let Some(cost) = entry.candidate {
            records.push(cost_record(frame_ns, solve, entry.iteration, "trial", cost));
        }
        let after = if entry.accepted {
            entry.candidate.unwrap_or(entry.before)
        } else {
            entry.before
        };
        records.push(cost_record(
            frame_ns,
            solve,
            entry.iteration,
            "after",
            after,
        ));
        records.push(serde_json::json!({
            "kind": "solver_iteration", "solver": "okvis", "frame_ns": frame_ns,
            "solve": solve, "iteration": entry.iteration, "accepted": entry.accepted,
            "reason": entry.reason, "predicted_decrease": entry.predicted_decrease,
            "quality": entry.quality,
        }));
        if let Some(pose) = entry.pose_steps.iter().find(|p| p.key == pose_key) {
            records.push(serde_json::json!({
                "kind": "solver_pose_step", "solver": "okvis", "frame_ns": frame_ns,
                "solve": solve, "iteration": entry.iteration, "accepted": entry.accepted,
                "translation": pose.translation, "rotation": pose.rotation,
            }));
        }
    }
    if let Some(cost) = summary.diagnostic_final {
        records.push(cost_record(
            frame_ns,
            solve,
            summary.iterations,
            "final",
            cost,
        ));
    }
    records.push(serde_json::json!({
        "kind": "solver_stop", "solver": "okvis", "frame_ns": frame_ns,
        "solve": solve, "iterations": summary.iterations, "reason": summary.report,
        "successful": summary.successful, "final_cost": summary.final_cost,
    }));
    records
}

#[cfg(test)]
#[path = "diagnostics_tests.rs"]
mod tests;
