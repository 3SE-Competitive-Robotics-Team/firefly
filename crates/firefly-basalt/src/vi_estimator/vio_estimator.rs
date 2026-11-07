// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original `vi_estimator/vio_estimator.h` and `src/vi_estimator/vio_estimator.cpp`.
use crate::imu::imu_types::ImuData;
use crate::imu::imu_types::PoseVelBiasState;
use crate::{optical_flow::OpticalFlowResult, utils::imu_types::MargData};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Isometry3, Matrix3, Vector3, Vector4};
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicI64, AtomicUsize, Ordering},
        mpsc::{self, Receiver, Sender, SyncSender},
    },
    thread::JoinHandle,
};

#[derive(Clone, Debug, Default)]
pub struct VioVisualizationData {
    pub t_ns: i64,
    pub states: Vec<Isometry3<f64>>,
    pub frames: Vec<Isometry3<f64>>,
    pub points: Vec<Vector3<f64>>,
    pub point_ids: Vec<i32>,
    pub opt_flow_res: Option<Arc<OpticalFlowResult>>,
    pub projections: Vec<Vec<Vector4<f64>>>,
}

/// Current-frame, unweighted pixel residuals and optimizer changes before marginalization.
#[derive(Clone, Debug, Default)]
pub struct VioFrameDiagnostics {
    pub t_ns: i64,
    pub tracks: [f64; 4],
    /// Associated left IDs, current graph observations, effective observations, depth-ready effective observations.
    pub support: [f64; 4],
    pub stereo_effective: usize,
    pub points_before: Vec<VioPointDiagnostics>,
    pub points_after: Vec<VioPointDiagnostics>,
    pub graph: [f64; 4],
    /// Current keyframe flag, continued landmarks, newly hosted landmarks, retained span (seconds).
    pub keyframes: [f64; 4],
    pub active_ids: Vec<usize>,
    /// Count, RMS norm, mean horizontal residual, mean vertical residual.
    pub residual_before: [f64; 4],
    pub residual_after: [f64; 4],
    pub position_delta: [f64; 3],
    pub velocity_delta: [f64; 3],
    pub rotation_delta: [f64; 3],
    pub flow: Option<Arc<OpticalFlowResult>>,
}

/// One frontend observation with its actual optimizer factor and geometry state.
#[derive(Clone, Debug)]
pub struct VioPointDiagnostics {
    pub track_id: u64,
    pub camera: u32,
    pub keypoint_index: u32,
    /// GRAPH=1, `DEPTH_READY=2`, PROJECTABLE=4, EFFECTIVE=8.
    pub flags: u32,
    pub pixel: [f64; 2],
    pub depth_z: f64,
    pub range: f64,
    pub inverse_range: f64,
    /// Observed minus predicted pixels, matching the live diagnostic convention.
    pub residual: [f64; 2],
    pub whitened_squared: f64,
    pub sqrt_information: [[f64; 2]; 2],
    pub robust_derivative: f64,
    pub robust_cost: f64,
}
pub struct VioWorkerInputs {
    pub vision: Receiver<Option<Arc<OpticalFlowResult>>>,
    pub imu: Receiver<Option<Arc<ImuData<f64>>>>,
}
#[derive(Clone, Default)]
pub struct VioWorkerOutputs {
    pub diagnostics: Option<Sender<VioFrameDiagnostics>>,
    pub local_map: Option<Sender<Option<Arc<super::local_map::LocalMapSnapshot>>>>,
    pub state: Option<Sender<Option<Arc<PoseVelBiasState<f64>>>>>,
    pub state_bounded: Option<SyncSender<Option<Arc<PoseVelBiasState<f64>>>>>,
    pub marg: Option<Sender<Option<Arc<MargData>>>>,
    pub marg_bounded: Option<SyncSender<Option<Arc<MargData>>>>,
    pub vis: Option<Sender<Option<Arc<VioVisualizationData>>>>,
}
impl VioWorkerOutputs {
    pub(crate) fn send_state(&self, state: Arc<PoseVelBiasState<f64>>) {
        if let Some(queue) = &self.state {
            let _ = queue.send(Some(Arc::clone(&state)));
        }
        if let Some(queue) = &self.state_bounded {
            let _ = queue.send(Some(state));
        }
    }
    #[must_use]
    pub(crate) fn has_marginalization_output(&self) -> bool {
        self.marg.is_some() || self.marg_bounded.is_some()
    }
    pub(crate) fn send_marginalization(&self, data: Arc<MargData>) {
        if let Some(queue) = &self.marg {
            let _ = queue.send(Some(Arc::clone(&data)));
        }
        if let Some(queue) = &self.marg_bounded {
            let _ = queue.send(Some(data));
        }
    }
    pub fn finish(&self) {
        if let Some(q) = &self.local_map {
            let _ = q.send(None);
        }
        if let Some(q) = &self.vis {
            let _ = q.send(None);
        }
        if let Some(q) = &self.marg {
            let _ = q.send(None);
        }
        if let Some(q) = &self.marg_bounded {
            let _ = q.send(None);
        }
        if let Some(q) = &self.state {
            let _ = q.send(None);
        }
        if let Some(q) = &self.state_bounded {
            let _ = q.send(None);
        }
    }
}
struct WorkerEnd<'a> {
    outputs: &'a VioWorkerOutputs,
    status: &'a VioStatus,
}
impl Drop for WorkerEnd<'_> {
    fn drop(&mut self) {
        self.outputs.finish();
        self.status.finished.store(true, Ordering::Release);
    }
}
pub struct VioStatus {
    pub last_processed_t_ns: AtomicI64,
    pub finished: AtomicBool,
    pub initial_pose: Mutex<Isometry3<f64>>,
    pub max_states: AtomicUsize,
    pub max_kfs: AtomicUsize,
}
impl VioStatus {
    #[must_use]
    pub fn new(max_states: usize, max_kfs: usize) -> Self {
        Self {
            last_processed_t_ns: AtomicI64::new(0),
            finished: AtomicBool::new(false),
            initial_pose: Mutex::new(Isometry3::identity()),
            max_states: AtomicUsize::new(max_states),
            max_kfs: AtomicUsize::new(max_kfs),
        }
    }
}
/// Algorithms own their state; the common handle owns the processing thread.
pub trait VioEstimatorEngine: Send {
    fn initialize_state(
        &mut self,
        t_ns: i64,
        pose: Isometry3<f64>,
        velocity: Vector3<f64>,
        bg: Vector3<f64>,
        ba: Vector3<f64>,
    );
    fn initialize_bias(&mut self, bg: Vector3<f64>, ba: Vector3<f64>);
    /// # Errors
    /// Returns a processing or input-stream error.
    fn run(
        &mut self,
        inputs: VioWorkerInputs,
        outputs: VioWorkerOutputs,
        status: Arc<VioStatus>,
    ) -> Result<()>;
    /// # Errors
    /// Returns an error if statistics cannot be written.
    fn debug_finalize(&self, directory: &std::path::Path) -> Result<()>;
    fn get_state_at(&self, time: i64) -> PoseVelBiasState<f64>;
    fn get_all_poses_map(&self) -> std::collections::BTreeMap<i64, Isometry3<f64>>;
}
/// Bounded inputs preserve the original capacities and `None` end markers.
pub struct VioEstimatorBase {
    vision: Option<SyncSender<Option<Arc<OpticalFlowResult>>>>,
    imu: Option<SyncSender<Option<Arc<ImuData<f64>>>>>,
    inputs: Option<VioWorkerInputs>,
    engine: Option<Box<dyn VioEstimatorEngine>>,
    thread: Option<JoinHandle<WorkerCompletion>>,
    pub outputs: VioWorkerOutputs,
    pub status: Arc<VioStatus>,
}
struct WorkerCompletion {
    engine: Box<dyn VioEstimatorEngine>,
    result: Result<()>,
}
impl VioEstimatorBase {
    #[must_use]
    pub fn new(engine: Box<dyn VioEstimatorEngine>, max_states: usize, max_kfs: usize) -> Self {
        let (vision, vision_rx) = mpsc::sync_channel(10);
        let (imu, imu_rx) = mpsc::sync_channel(300);
        Self {
            vision: Some(vision),
            imu: Some(imu),
            inputs: Some(VioWorkerInputs {
                vision: vision_rx,
                imu: imu_rx,
            }),
            engine: Some(engine),
            thread: None,
            outputs: VioWorkerOutputs::default(),
            status: Arc::new(VioStatus::new(max_states, max_kfs)),
        }
    }
    /// # Panics
    /// Panics after shutdown has started.
    #[must_use]
    pub fn vision_data_queue(&self) -> &SyncSender<Option<Arc<OpticalFlowResult>>> {
        self.vision.as_ref().expect("estimator shutdown")
    }
    /// # Panics
    /// Panics after shutdown has started.
    #[must_use]
    pub fn imu_data_queue(&self) -> &SyncSender<Option<Arc<ImuData<f64>>>> {
        self.imu.as_ref().expect("estimator shutdown")
    }
    /// # Errors
    /// Returns an error if already initialized or thread creation fails.
    pub fn initialize_state(
        &mut self,
        t_ns: i64,
        pose: Isometry3<f64>,
        velocity: Vector3<f64>,
        bg: Vector3<f64>,
        ba: Vector3<f64>,
    ) -> Result<()> {
        if self.inputs.is_none() {
            return Err(already_started());
        }
        self.engine
            .as_mut()
            .ok_or_else(already_started)?
            .initialize_state(t_ns, pose, velocity, bg, ba);
        *self
            .status
            .initial_pose
            .lock()
            .map_err(|_| worker_error("Initial pose mutex poisoned"))? = pose;
        self.start()
    }
    /// # Errors
    /// Returns an error if already initialized or thread creation fails.
    pub fn initialize(&mut self, bg: Vector3<f64>, ba: Vector3<f64>) -> Result<()> {
        if self.inputs.is_none() {
            return Err(already_started());
        }
        self.engine
            .as_mut()
            .ok_or_else(already_started)?
            .initialize_bias(bg, ba);
        self.start()
    }
    fn start(&mut self) -> Result<()> {
        let mut engine = self.engine.take().ok_or_else(already_started)?;
        let inputs = self.inputs.take().ok_or_else(already_started)?;
        let outputs = self.outputs.clone();
        let status = self.status.clone();
        self.thread = Some(
            std::thread::Builder::new()
                .name("basalt-estimator".into())
                .spawn(move || {
                    let _ending = WorkerEnd {
                        outputs: &outputs,
                        status: &status,
                    };
                    let result = engine.run(inputs, outputs.clone(), status.clone());
                    WorkerCompletion { engine, result }
                })
                .map_err(|e| worker_error("Cannot start estimator worker").with_source(e))?,
        );
        Ok(())
    }
    /// # Errors
    /// Returns the worker's processing error or a panic error.
    pub fn maybe_join(&mut self) -> Result<()> {
        if let Some(thread) = self.thread.take() {
            let WorkerCompletion { engine, result } = thread
                .join()
                .map_err(|_| worker_error("Estimator worker panicked"))?;
            self.engine = Some(engine);
            result?;
        }
        Ok(())
    }
    /// Access algorithm state before starting or after joining the worker.
    /// # Errors
    /// Returns an error while the worker owns the algorithm, or after a panic.
    pub fn engine(&self) -> Result<&dyn VioEstimatorEngine> {
        self.engine
            .as_deref()
            .ok_or_else(|| worker_error("Join estimator before accessing algorithm state"))
    }
    /// # Errors
    /// Returns an error before joining or if statistics cannot be written.
    pub fn debug_finalize(&self, directory: &std::path::Path) -> Result<()> {
        self.engine()?.debug_finalize(directory)
    }
    /// # Errors
    /// Returns an error if the initial pose mutex is poisoned.
    pub fn initial_pose(&self) -> Result<Isometry3<f64>> {
        self.status
            .initial_pose
            .lock()
            .map(|v| *v)
            .map_err(|_| worker_error("Initial pose mutex poisoned"))
    }
    pub fn set_max_states(&self, value: usize) {
        self.status.max_states.store(value, Ordering::Release);
    }
    pub fn set_max_kfs(&self, value: usize) {
        self.status.max_kfs.store(value, Ordering::Release);
    }
    /// Drop all locally owned producers; the worker sees disconnection once
    /// external producers have also been released.
    /// # Errors
    /// Returns worker errors. External queue clones must be dropped first.
    pub fn shutdown(mut self) -> Result<()> {
        self.close_inputs();
        self.maybe_join()
    }

    /// Close local producers before joining the optical-flow/estimator workers.
    pub fn close_inputs(&mut self) {
        self.vision.take();
        self.imu.take();
    }
}
fn already_started() -> Error {
    worker_error("Estimator already initialized")
}
impl Drop for VioEstimatorBase {
    fn drop(&mut self) {
        self.vision.take();
        self.imu.take();
        let _ = self.maybe_join();
    }
}
fn worker_error(message: &str) -> Error {
    Error::new(ErrorKind::Internal, message).with_operation("basalt.estimator")
}

pub struct VioEstimatorFactory;
impl VioEstimatorFactory {
    /// Selects the original VO/VIO and float/double implementations.
    /// # Errors
    /// Reserved for construction failures reported by the estimator.
    pub fn get_vio_estimator(
        config: &crate::utils::vio_config::VioConfig,
        calibration: &firefly_calibration::calibration::Calibration<f64>,
        gravity: &Vector3<f64>,
        use_imu: bool,
        use_double: bool,
    ) -> Result<VioEstimatorBase> {
        use super::{
            sqrt_keypoint_vio::SqrtKeypointVioEstimator, sqrt_keypoint_vo::SqrtKeypointVoEstimator,
        };
        let engine: Box<dyn VioEstimatorEngine> = match (use_imu, use_double) {
            (true, true) => Box::new(SqrtKeypointVioEstimator::<f64>::new(
                gravity,
                calibration,
                config,
            )),
            (true, false) => Box::new(SqrtKeypointVioEstimator::<f32>::new(
                gravity,
                calibration,
                config,
            )),
            (false, true) => Box::new(SqrtKeypointVoEstimator::<f64>::new(calibration, config)),
            (false, false) => Box::new(SqrtKeypointVoEstimator::<f32>::new(calibration, config)),
        };
        Ok(VioEstimatorBase::new(
            engine,
            config.vio_max_states as usize,
            config.vio_max_kfs as usize,
        ))
    }
}

/// Original rigid SVD trajectory alignment, including 110 ms association cutoff.
/// Mutates the full ground-truth trajectory into the estimate coordinate system.
/// # Panics
/// Panics for mismatched arrays, invalid intervals or no timestamp associations.
#[must_use]
pub fn align_svd(
    filter_times: &[i64],
    filter_positions: &[Vector3<f64>],
    gt_times: &[i64],
    gt_positions: &mut [Vector3<f64>],
) -> f64 {
    assert_eq!(filter_times.len(), filter_positions.len());
    assert_eq!(gt_times.len(), gt_positions.len());
    let mut associations = Vec::new();
    for (time, position) in filter_times.iter().zip(filter_positions) {
        let upper = gt_times
            .iter()
            .position(|t| t > time)
            .unwrap_or(gt_times.len());
        if upper == 0 || upper >= gt_times.len() {
            continue;
        }
        let lower = upper - 1;
        let elapsed = (*time - gt_times[lower]) as f64;
        let interval = (gt_times[upper] - gt_times[lower]) as f64;
        assert!(elapsed >= 0. && interval > 0.);
        if interval > 1.1e8 {
            continue;
        }
        let ratio = elapsed / interval;
        assert!((0. ..1.).contains(&ratio));
        associations.push((
            *position,
            gt_positions[lower] * (1. - ratio) + gt_positions[upper] * ratio,
        ));
    }
    assert_ne!(associations.len(), 0, "No timestamp associations");
    let count = associations.len() as f64;
    let mean_est = associations.iter().map(|p| p.0).sum::<Vector3<f64>>() / count;
    let mean_gt = associations.iter().map(|p| p.1).sum::<Vector3<f64>>() / count;
    let covariance = associations
        .iter()
        .map(|(est, gt)| (gt - mean_gt) * (est - mean_est).transpose())
        .sum::<Matrix3<f64>>();
    let svd = covariance.svd(true, true);
    let u = svd.u.expect("U requested");
    let vt = svd.v_t.expect("V requested");
    let mut sign = Matrix3::identity();
    if u.determinant() * vt.determinant() < 0. {
        sign[(2, 2)] = -1.;
    }
    let rotation = u * sign * vt;
    let translation = mean_gt - rotation * mean_est;
    for position in gt_positions {
        *position = rotation.transpose() * (*position - translation);
    }
    (associations
        .iter()
        .map(|(est, gt)| (rotation * est + translation - gt).norm_squared())
        .sum::<f64>()
        / count)
        .sqrt()
}

#[cfg(test)]
mod worker_end_tests {
    use super::*;

    #[test]
    fn normal_and_panicking_worker_send_one_end_to_each_output() {
        for panic in [false, true] {
            let (state, estimates) = mpsc::channel();
            let (bounded_state, bounded_estimates) = mpsc::sync_channel(1);
            let (vis, visuals) = mpsc::channel();
            let (marg, margins) = mpsc::sync_channel(1);
            let (map, maps) = mpsc::channel();
            let outputs = VioWorkerOutputs {
                state: Some(state),
                state_bounded: Some(bounded_state),
                vis: Some(vis),
                marg_bounded: Some(marg),
                local_map: Some(map),
                ..Default::default()
            };
            let status = VioStatus::new(3, 7);
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let _end = WorkerEnd {
                    outputs: &outputs,
                    status: &status,
                };
                assert!(!panic, "injected estimator panic");
            }));
            assert_eq!(result.is_err(), panic);
            assert!(status.finished.load(Ordering::Acquire));
            assert!(estimates.try_recv().unwrap().is_none());
            assert!(bounded_estimates.try_recv().unwrap().is_none());
            assert!(visuals.try_recv().unwrap().is_none());
            assert!(margins.try_recv().unwrap().is_none());
            assert!(maps.try_recv().unwrap().is_none());
            assert!(estimates.try_recv().is_err());
            assert!(bounded_estimates.try_recv().is_err());
            assert!(visuals.try_recv().is_err());
            assert!(margins.try_recv().is_err());
            assert!(maps.try_recv().is_err());
        }
    }

    #[test]
    fn failed_archive_consumer_releases_blocked_producer() {
        let (marg, margins) = mpsc::sync_channel(1);
        marg.send(None).unwrap();
        let (completed, completion) = mpsc::channel();
        let worker = std::thread::spawn(move || {
            let outputs = VioWorkerOutputs {
                marg_bounded: Some(marg),
                ..Default::default()
            };
            let status = VioStatus::new(3, 7);
            {
                let _end = WorkerEnd {
                    outputs: &outputs,
                    status: &status,
                };
            }
            completed
                .send(status.finished.load(Ordering::Acquire))
                .unwrap();
        });
        drop(margins);
        assert!(
            completion
                .recv_timeout(std::time::Duration::from_secs(2))
                .unwrap()
        );
        worker.join().unwrap();
    }
}
