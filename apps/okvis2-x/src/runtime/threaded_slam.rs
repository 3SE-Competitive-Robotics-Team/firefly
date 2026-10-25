/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Thread-owned estimation runtime, bounded sensor queues, publication and visualization.
//! Non-Send camera and solver resources are constructed on their owning worker.
use crate::{
    ceres::{
        component::MultiFramePtr,
        imu_error::{ImuError, seconds},
        vi_slam_backend::StrategyResult,
    },
    config::parameters::{CameraParameters, GpsParameters, ImuParameters, ViParameters},
    frames::{
        frame_typedefs::{
            LandmarkId, MapPoint, MapPointVector, Observation, SpeedAndBias, StateId,
        },
        multi_frame::MultiFrame,
    },
    frontend::{
        frame_absolute_pose_sac_problem::AbsolutePoseSolver,
        frame_relative_pose_sac_problem::CentralRelativePoseSolver,
        frame_rotation_only_sac_problem::RotationOnlySolver,
        pipeline::{BriskFeatureFactory, Frontend, FrontendGraph, LoopPoseSolver},
    },
    kinematics::transformation::Transformation,
    mapping::{
        config_mapping::SubMapConfig,
        lidar_motion_undistortion::{LidarInitialState, LidarMotionUndistortion},
        map_typedefs::SupereightMapType,
        submapping_utils::{
            MeanOccupancyMap, Safe, downsample_points_uncertainty, grad_field_mean_occup,
            interp_field_mean_occup,
        },
    },
    runtime::vi_interface::{
        ImageMap, OptimisedGraphCallback, State, TrackingQuality, TrackingState, Trajectory,
        ViInterface,
    },
    sensors::measurements::{
        CameraData, CameraMeasurement, DepthMeasurementDeque, GpsMeasurement, GpsMeasurementDeque,
        GpsSensorReadings, ImuMeasurement, ImuMeasurementDeque, ImuSensorReadings,
        LidarMeasurement, LidarMeasurementDeque, LidarSensorReadings, Measurement,
    },
};
use firefly_error::{Error, ErrorKind, Result};
use jiff::{SignedDuration, Timestamp};
use nalgebra::{DVector, Quaternion, Vector2, Vector3};
use purecv::core::{DynamicMatrix, Matrix};
use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet, VecDeque},
    io::Write,
    path::{Path, PathBuf},
    rc::Rc,
    sync::{
        Arc, Condvar, Mutex,
        atomic::{AtomicBool, AtomicUsize, Ordering},
        mpsc,
    },
    thread::{self, JoinHandle},
    time::Instant,
};
const IMU_TEMPORAL_OVERLAP: f64 = 0.02;
/// Solver/graph primitives shared with Frontend; graph ownership stays on one thread.
/// These source header declarations have no definition in the C++ snapshot.
pub trait Se2MapExport {
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn export_se2_map(&mut self) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn create_final_se2_map(&mut self) -> Result<bool>;
}
pub trait ThreadedGraph: FrontendGraph {
    type FullGraphResult: Send + 'static;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn add_imu(&mut self, parameters: &ImuParameters) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn add_camera(&mut self, parameters: &CameraParameters) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn add_gps(&mut self, parameters: &GpsParameters) -> Result<()>;
    fn set_detector_uniformity_radius(&mut self, radius: f64);
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn set_optimisation_time_limit(&mut self, seconds: f64, min_iterations: i32) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn add_states(
        &mut self,
        frame: MultiFramePtr,
        imu: &ImuMeasurementDeque,
        keyframe: bool,
    ) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn clear(&mut self) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn set_keyframe(&mut self, id: StateId, value: bool) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn speed_and_bias(&self, id: StateId) -> Result<SpeedAndBias>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn timestamp(&self, id: StateId) -> Result<Timestamp>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn optimise_realtime_graph(
        &mut self,
        iterations: i32,
        threads: i32,
        initialized: bool,
    ) -> Result<Vec<StateId>>;
    /// The task owns a Send numerical snapshot and a solver created on its worker;
    /// it must not capture Rc graph blocks. Commit replays the backend's touched-state backlog.
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn prepare_full_graph_task(
        &mut self,
        iterations: i32,
        threads: i32,
    ) -> Result<Box<dyn FnOnce() -> Result<Self::FullGraphResult> + Send>>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn finish_full_graph_task(&mut self, result: Self::FullGraphResult) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn synchronise_realtime_and_full_graph(&mut self) -> Result<Vec<StateId>>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn add_gps_measurements(
        &mut self,
        gps: &GpsMeasurementDeque,
        imu: &ImuMeasurementDeque,
    ) -> Result<()>;
    fn observed_ids(&self, id: StateId) -> BTreeSet<StateId>;
    fn most_overlapped_state_id(&self, id: StateId) -> StateId;
    fn closed_loop(&self, id: StateId) -> bool;
    fn component_alignments(&self, id: StateId) -> BTreeMap<u64, Transformation>;
    fn gps_points(&self, id: StateId) -> Vec<Vector3<f64>>;
    fn t_gw(&self) -> Transformation;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn apply_strategy(
        &mut self,
        keyframes: usize,
        loop_frames: usize,
        imu_frames: usize,
    ) -> Result<StrategyResult>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn do_final_ba(
        &mut self,
        iterations: i32,
        sigma_r: f64,
        sigma_alpha: f64,
        threads: i32,
    ) -> Result<Vec<StateId>>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    /// # Errors
    /// Returns graph serialization or file output errors.
    fn write_final_csv_trajectory(&self, path: &Path, rpg: bool) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn write_global_csv_trajectory(&self, path: &Path) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn save_map(&mut self, path: &Path) -> Result<bool>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn dump_gps_residuals(&mut self, path: &Path) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn write_debug_statistics_csv(&self, prefix: &Path) -> Result<()>;
}
#[derive(Clone, Debug)]
pub struct AlignmentTerm {
    pub submap_a: Option<u64>,
    pub submap_b: u64,
    pub frame_a_id: u64,
    pub frame_b_id: u64,
    pub points_b: Vec<Vector3<f32>>,
    pub sensor_errors: Vec<f32>,
}
/// Map handles retain the identity/lifetime of the source's shared submap pointers.
pub trait MappingRuntime<G: ThreadedGraph> {
    type Map: SupereightMapType + MeanOccupancyMap + Clone;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn map(&self, handle: u64) -> Result<Self::Map>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn add_constraints(
        &mut self,
        graph: &mut G,
        map: u64,
        frames: (u64, u64),
        samples: (&[Vector3<f32>], &[f32]),
        lidar: bool,
        loss: &str,
    ) -> Result<()>;
}
pub type AlignmentPublishCallback =
    Arc<dyn Fn(Timestamp, &Transformation, &[Vector3<f32>], bool) + Send + Sync>;
#[derive(Clone)]
pub struct PublicationData {
    pub state: State,
    pub tracking_state: TrackingState,
    pub updated_states: Arc<BTreeMap<StateId, State>>,
    pub landmarks: Arc<MapPointVector>,
}
#[derive(Clone)]
pub struct VisualisationData {
    pub publication: PublicationData,
    pub images: ImageMap,
    pub observations: Vec<Observation>,
    pub extrinsics: Vec<Transformation>,
}
/// Drawing receives an owned image/numerical snapshot and never accesses graph blocks.
pub trait VisualisationRenderer: Send + 'static {
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn draw_matches(
        &mut self,
        data: &VisualisationData,
    ) -> Result<BTreeMap<String, Arc<DynamicMatrix>>>;
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    fn draw_overhead(&mut self, data: &PublicationData) -> Result<Option<Arc<DynamicMatrix>>>;
}
#[derive(Clone, Debug)]
pub struct SensorInputConfig {
    pub used_cameras: Vec<bool>,
    pub colour_cameras: Vec<bool>,
    pub image_delay: f64,
    pub use_imu: bool,
    pub accelerometer_scale: Vector3<f64>,
    pub use_alignment: bool,
    pub downsampling: usize,
    pub depth_scale: f32,
}
impl SensorInputConfig {
    #[must_use]
    pub fn new(parameters: &ViParameters, submap: &SubMapConfig) -> Self {
        Self {
            used_cameras: (0..parameters.n_camera_system.num_cameras())
                .map(|c| parameters.n_camera_system.camera_type(c).is_used)
                .collect(),
            colour_cameras: (0..parameters.n_camera_system.num_cameras())
                .map(|c| parameters.n_camera_system.camera_type(c).is_colour)
                .collect(),
            image_delay: parameters.camera.image_delay,
            use_imu: parameters.imu.r#use,
            accelerometer_scale: parameters.imu.s_a,
            use_alignment: submap.factors.use_map2live_factors
                && parameters.output.enable_submapping,
            downsampling: submap.sensor_measurement_downsampling,
            depth_scale: submap.depth_scaling_factor,
        }
    }
}
struct QueueState<T> {
    items: VecDeque<T>,
    shutdown: bool,
}
/// The nonblocking push drops the oldest value and accepts the newest, as `ThreadsafeQueue` does.
pub struct ThreadsafeQueue<T> {
    state: Mutex<QueueState<T>>,
    readable: Condvar,
    writable: Condvar,
}
impl<T> Default for ThreadsafeQueue<T> {
    fn default() -> Self {
        Self {
            state: Mutex::new(QueueState {
                items: VecDeque::new(),
                shutdown: false,
            }),
            readable: Condvar::new(),
            writable: Condvar::new(),
        }
    }
}
impl<T> ThreadsafeQueue<T> {
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn push(&self, value: T, capacity: usize, blocking: bool) -> Result<Option<bool>> {
        if capacity == 0 {
            return Err(runtime_error("queue capacity must be positive"));
        }
        let mut state = self
            .state
            .lock()
            .map_err(|_| runtime_error("queue mutex poisoned"))?;
        while blocking && state.items.len() >= capacity && !state.shutdown {
            state = self
                .writable
                .wait(state)
                .map_err(|_| runtime_error("queue mutex poisoned"))?;
        }
        if state.shutdown {
            return Ok(None);
        }
        let dropped = state.items.len() >= capacity;
        if dropped {
            state.items.pop_front();
        }
        state.items.push_back(value);
        self.readable.notify_one();
        Ok(Some(dropped))
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn pop(&self, blocking: bool) -> Result<Option<T>> {
        let mut state = self
            .state
            .lock()
            .map_err(|_| runtime_error("queue mutex poisoned"))?;
        while blocking && state.items.is_empty() && !state.shutdown {
            state = self
                .readable
                .wait(state)
                .map_err(|_| runtime_error("queue mutex poisoned"))?;
        }
        let result = state.items.pop_front();
        if result.is_some() {
            self.writable.notify_one();
        }
        Ok(result)
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn front(&self) -> Result<Option<T>>
    where
        T: Clone,
    {
        Ok(self
            .state
            .lock()
            .map_err(|_| runtime_error("queue mutex poisoned"))?
            .items
            .front()
            .cloned())
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn back(&self) -> Result<Option<T>>
    where
        T: Clone,
    {
        Ok(self
            .state
            .lock()
            .map_err(|_| runtime_error("queue mutex poisoned"))?
            .items
            .back()
            .cloned())
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn len(&self) -> Result<usize> {
        Ok(self
            .state
            .lock()
            .map_err(|_| runtime_error("queue mutex poisoned"))?
            .items
            .len())
    }
    /// # Errors
    /// Returns an error if the queue mutex is poisoned.
    pub fn is_empty(&self) -> Result<bool> {
        Ok(self.len()? == 0)
    }
    pub fn shutdown(&self) {
        if let Ok(mut state) = self.state.lock() {
            state.shutdown = true;
            self.readable.notify_all();
            self.writable.notify_all();
        }
    }
}
enum Command {
    Process(mpsc::Sender<Result<bool>>),
    NeedsLidarKeyframe(mpsc::Sender<Result<bool>>),
    Drain(mpsc::Sender<Result<()>>),
    FinalBa(mpsc::Sender<Result<()>>),
    SaveMap(PathBuf, mpsc::Sender<Result<bool>>),
    WriteTrajectory(PathBuf, bool, mpsc::Sender<Result<()>>),
    WriteGlobalTrajectory(PathBuf, mpsc::Sender<Result<()>>),
    GpsResiduals(PathBuf, mpsc::Sender<Result<()>>),
    DebugStatistics(PathBuf, mpsc::Sender<Result<()>>),
    Exit,
}
struct Shared {
    cameras: ThreadsafeQueue<Vec<CameraMeasurement>>,
    imu: ThreadsafeQueue<ImuMeasurement>,
    lidar: ThreadsafeQueue<LidarMeasurement>,
    depth: ThreadsafeQueue<CameraMeasurement>,
    gps: ThreadsafeQueue<GpsMeasurement>,
    alignments: ThreadsafeQueue<AlignmentTerm>,
    commands: ThreadsafeQueue<Command>,
    publications: ThreadsafeQueue<PublicationData>,
    visualisation: ThreadsafeQueue<VisualisationData>,
    images: ThreadsafeQueue<BTreeMap<String, Arc<DynamicMatrix>>>,
    blocking: AtomicBool,
    paused: AtomicBool,
    started: AtomicBool,
    shutdown: AtomicBool,
    downsampling_counter: AtomicUsize,
    callback: Mutex<Option<OptimisedGraphCallback>>,
    lossless_publications: AtomicBool,
    alignment_callback: Mutex<Option<AlignmentPublishCallback>>,
    errors: Mutex<Vec<String>>,
    pause: Mutex<()>,
    resume: Condvar,
}
impl Default for Shared {
    fn default() -> Self {
        Self {
            cameras: ThreadsafeQueue::default(),
            imu: ThreadsafeQueue::default(),
            lidar: ThreadsafeQueue::default(),
            depth: ThreadsafeQueue::default(),
            gps: ThreadsafeQueue::default(),
            alignments: ThreadsafeQueue::default(),
            commands: ThreadsafeQueue::default(),
            publications: ThreadsafeQueue::default(),
            visualisation: ThreadsafeQueue::default(),
            images: ThreadsafeQueue::default(),
            blocking: AtomicBool::new(false),
            paused: AtomicBool::new(false),
            started: AtomicBool::new(false),
            shutdown: AtomicBool::new(false),
            downsampling_counter: AtomicUsize::new(0),
            callback: Mutex::new(None),
            lossless_publications: AtomicBool::new(false),
            alignment_callback: Mutex::new(None),
            errors: Mutex::new(Vec::new()),
            pause: Mutex::new(()),
            resume: Condvar::new(),
        }
    }
}
impl Shared {
    fn error(&self, error: &(impl ToString + ?Sized)) {
        if let Ok(mut errors) = self.errors.lock() {
            errors.push(error.to_string());
        }
    }
    fn close_sensors(&self) {
        self.cameras.shutdown();
        self.imu.shutdown();
        self.lidar.shutdown();
        self.depth.shutdown();
        self.gps.shutdown();
        self.alignments.shutdown();
    }
    fn check_errors(&self) -> Result<()> {
        let mut errors = self
            .errors
            .lock()
            .map_err(|_| runtime_error("error mutex poisoned"))?;
        if errors.is_empty() {
            Ok(())
        } else {
            Err(runtime_error(std::mem::take(&mut *errors).join("; ")))
        }
    }
}
/// Send sensor handle. The factory is invoked inside the processing worker.
pub struct ThreadedSlam {
    input: SensorInputConfig,
    shared: Arc<Shared>,
    processing: Option<JoinHandle<Result<()>>>,
    publishing: Option<JoinHandle<Result<()>>>,
    visualisation: Option<JoinHandle<Result<()>>>,
    drained: bool,
    final_trajectory_csv_file_name: Option<PathBuf>,
    map_csv_file_name: Option<PathBuf>,
    rpg: bool,
}
impl ThreadedSlam {
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn new<G, S, B, M>(
        input: SensorInputConfig,
        factory: impl FnOnce() -> Result<ProcessingCore<G, S, B, M>> + Send + 'static,
        renderer: Option<Box<dyn VisualisationRenderer>>,
    ) -> Result<Self>
    where
        G: ThreadedGraph + 'static,
        S: AbsolutePoseSolver
            + CentralRelativePoseSolver
            + RotationOnlySolver
            + LoopPoseSolver
            + 'static,
        B: BriskFeatureFactory + 'static,
        M: MappingRuntime<G> + 'static,
    {
        if input.downsampling == 0 {
            return Err(runtime_error("sensor downsampling must be positive"));
        }
        let shared = Arc::new(Shared::default());
        let worker_shared = shared.clone();
        let processing = thread::spawn(move || Self::processing_worker(&worker_shared, factory));
        let publishing_shared = shared.clone();
        let publishing = thread::spawn(move || {
            while let Some(data) = publishing_shared.publications.pop(true)? {
                let callback = publishing_shared
                    .callback
                    .lock()
                    .map_err(|_| runtime_error("callback mutex poisoned"))?
                    .clone();
                if let Some(callback) = callback {
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        callback(
                            &data.state,
                            &data.tracking_state,
                            data.updated_states,
                            data.landmarks,
                        );
                    }));
                    if result.is_err() {
                        publishing_shared.publications.shutdown();
                        publishing_shared.error("publication callback panicked");
                        return Err(runtime_error("publication callback panicked"));
                    }
                }
            }
            Ok(())
        });
        let visualisation = renderer.map(|mut renderer| {
            let visual_shared = shared.clone();
            thread::spawn(move || {
                while let Some(data) = visual_shared.visualisation.pop(true)? {
                    let mut images = renderer.draw_matches(&data)?;
                    if let Some(overhead) = renderer.draw_overhead(&data.publication)? {
                        images.insert("Top Debug View".into(), overhead);
                    }
                    visual_shared.images.push(images, 1, false)?;
                }
                Ok(())
            })
        });
        Ok(Self {
            input,
            shared,
            processing: Some(processing),
            publishing: Some(publishing),
            visualisation,
            drained: false,
            final_trajectory_csv_file_name: None,
            map_csv_file_name: None,
            rpg: false,
        })
    }
    fn processing_worker<G, S, B, M>(
        worker_shared: &Shared,
        factory: impl FnOnce() -> Result<ProcessingCore<G, S, B, M>> + Send + 'static,
    ) -> Result<()>
    where
        G: ThreadedGraph + 'static,
        S: AbsolutePoseSolver
            + CentralRelativePoseSolver
            + RotationOnlySolver
            + LoopPoseSolver
            + 'static,
        B: BriskFeatureFactory + 'static,
        M: MappingRuntime<G> + 'static,
    {
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut core = factory()?;
            core.initialise()?;
            loop {
                let Some(command) = worker_shared.commands.pop(true)? else {
                    break;
                };
                match command {
                    Command::Process(reply) => {
                        let mut lock = worker_shared
                            .pause
                            .lock()
                            .map_err(|_| runtime_error("pause mutex poisoned"))?;
                        while worker_shared.paused.load(Ordering::Acquire)
                            && !worker_shared.shutdown.load(Ordering::Acquire)
                        {
                            lock = worker_shared
                                .resume
                                .wait(lock)
                                .map_err(|_| runtime_error("pause mutex poisoned"))?;
                        }
                        drop(lock);
                        let result = core.process_frame(worker_shared);
                        let fatal = result.as_ref().err().map(ToString::to_string);
                        let _ = reply.send(result);
                        if let Some(error) = fatal {
                            return Err(runtime_error(error));
                        }
                    }
                    Command::NeedsLidarKeyframe(reply) => {
                        let _ = reply.send(Ok(core.needs_new_lidar_keyframe()));
                    }
                    Command::Drain(reply) => {
                        let result = core.drain(worker_shared);
                        let _ = reply.send(result);
                    }
                    Command::FinalBa(reply) => {
                        let result = core.final_ba(worker_shared);
                        let _ = reply.send(result);
                    }
                    Command::SaveMap(path, reply) => {
                        let _ = reply.send(core.graph.save_map(&path));
                    }
                    Command::WriteTrajectory(path, rpg, reply) => {
                        let _ = reply.send(core.graph.write_final_csv_trajectory(&path, rpg));
                    }
                    Command::WriteGlobalTrajectory(path, reply) => {
                        let _ = reply.send(core.graph.write_global_csv_trajectory(&path));
                    }
                    Command::GpsResiduals(path, reply) => {
                        let _ = reply.send(core.graph.dump_gps_residuals(&path));
                    }
                    Command::DebugStatistics(path, reply) => {
                        let _ = reply.send(core.graph.write_debug_statistics_csv(&path));
                    }
                    Command::Exit => {
                        core.drain(worker_shared)?;
                        break;
                    }
                }
            }
            Ok(())
        }))
        .unwrap_or_else(|_| Err(runtime_error("processing worker panicked")));
        worker_shared.close_sensors();
        worker_shared.commands.shutdown();
        while let Ok(Some(command)) = worker_shared.commands.pop(false) {
            drop(command);
        }
        if let Err(error) = &outcome {
            worker_shared.error(&error);
        }
        outcome
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn needs_new_lidar_keyframe(&self) -> Result<bool> {
        self.command(Command::NeedsLidarKeyframe)
    }
    #[must_use]
    pub fn has_started(&self) -> bool {
        self.shared.started.load(Ordering::Acquire)
    }
    pub fn set_paused(&self, paused: bool) {
        let _guard = self
            .shared
            .pause
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        self.shared.paused.store(paused, Ordering::Release);
        if !paused {
            self.shared.resume.notify_all();
        }
    }
    #[must_use]
    pub fn is_paused(&self) -> bool {
        self.shared.paused.load(Ordering::Acquire)
    }
    fn command<T>(&self, make: impl FnOnce(mpsc::Sender<Result<T>>) -> Command) -> Result<T> {
        self.shared.check_errors()?;
        let (tx, rx) = mpsc::channel();
        if self.shared.commands.push(make(tx), 8, true)?.is_none() {
            return Err(runtime_error("processing worker stopped"));
        }
        rx.recv()
            .map_err(|_| runtime_error("processing worker disconnected"))?
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn process_frame(&self) -> Result<bool> {
        self.command(Command::Process)
    }
    /// Drains work and joins graph/CNN tasks while retaining the thread-owned graph for final output.
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn stop_threading(&mut self) -> Result<()> {
        if self.drained {
            return self.shared.check_errors();
        }
        self.set_paused(false);
        self.shared.close_sensors();
        let mut failure = self.command(Command::Drain).err();
        self.shared.shutdown.store(true, Ordering::Release);
        self.shared.publications.shutdown();
        self.shared.visualisation.shutdown();
        if let Some(worker) = self.publishing.take()
            && let Err(error) = join_worker(worker)
        {
            failure.get_or_insert(error);
        }
        if let Some(worker) = self.visualisation.take()
            && let Err(error) = join_worker(worker)
        {
            failure.get_or_insert(error);
        }
        self.drained = true;
        failure.map_or(Ok(()), Err)
    }
    pub fn set_final_trajectory_csv_file(&mut self, path: impl Into<PathBuf>, rpg: bool) {
        let path = path.into();
        self.final_trajectory_csv_file_name = if path.as_os_str().is_empty() {
            None
        } else {
            Some(path)
        };
        self.rpg = rpg;
    }
    pub fn set_map_csv_file(&mut self, path: impl Into<PathBuf>) {
        let path = path.into();
        self.map_csv_file_name = if path.as_os_str().is_empty() {
            None
        } else {
            Some(path)
        };
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn write_final_trajectory_csv(&mut self) -> Result<()> {
        self.stop_threading()?;
        if let Some(path) = self.final_trajectory_csv_file_name.clone() {
            self.command(|reply| Command::WriteTrajectory(path, self.rpg, reply))?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn save_map(&mut self) -> Result<bool> {
        self.stop_threading()?;
        if self.final_trajectory_csv_file_name.is_none() {
            return Ok(false);
        }
        let path = self.map_csv_file_name.clone().unwrap_or_default();
        self.command(|reply| Command::SaveMap(path, reply))
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn set_alignment_publish_callback(&self, callback: AlignmentPublishCallback) -> Result<()> {
        *self
            .shared
            .alignment_callback
            .lock()
            .map_err(|_| runtime_error("alignment callback mutex poisoned"))? = Some(callback);
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn do_final_ba(&mut self) -> Result<()> {
        self.stop_threading()?;
        self.command(Command::FinalBa)
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn dump_gps_residuals(&mut self, path: &Path) -> Result<()> {
        self.stop_threading()?;
        self.command(|reply| Command::GpsResiduals(path.into(), reply))
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn write_debug_statistics_csv(&self, prefix: &Path) -> Result<()> {
        self.command(|reply| Command::DebugStatistics(prefix.into(), reply))
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn write_global_trajectory_csv(&self, path: &Path) -> Result<()> {
        self.command(|reply| Command::WriteGlobalTrajectory(path.into(), reply))
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn add_gps_measurement(
        &self,
        stamp: Timestamp,
        position: &Vector3<f64>,
        sigma: &Vector3<f64>,
    ) -> Result<bool> {
        self.push(
            &self.shared.gps,
            Measurement::new(
                stamp,
                GpsSensorReadings::from_cartesian_sigmas(*position, sigma.x, sigma.y, sigma.z),
                -1,
            ),
            5000,
        )
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn add_geodetic_gps_measurement(
        &self,
        stamp: Timestamp,
        lat: f64,
        lon: f64,
        height: f64,
        h_acc: f64,
        v_acc: f64,
    ) -> Result<bool> {
        self.push(
            &self.shared.gps,
            Measurement::new(
                stamp,
                GpsSensorReadings::from_geodetic(lat, lon, height, h_acc, v_acc),
                -1,
            ),
            5000,
        )
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn add_submap_alignment_constraints(&self, term: AlignmentTerm) -> Result<bool> {
        if term.submap_a.is_some() && term.points_b.len() != term.sensor_errors.len() {
            return Err(runtime_error(
                "alignment point and uncertainty counts differ",
            ));
        }
        self.shared
            .alignments
            .push(term, usize::MAX, false)
            .map(|r| r.is_some())
    }
    fn push<T>(&self, queue: &ThreadsafeQueue<T>, value: T, capacity: usize) -> Result<bool> {
        let blocking = self.shared.blocking.load(Ordering::Acquire);
        Ok(queue
            .push(value, capacity, blocking)?
            .is_some_and(|dropped| !dropped))
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn shutdown(&mut self) -> Result<()> {
        let mut failure = self.stop_threading().err();
        self.shared.shutdown.store(true, Ordering::Release);
        self.set_paused(false);
        let _ = self.shared.commands.push(Command::Exit, 8, false);
        self.shared.commands.shutdown();
        if let Some(worker) = self.processing.take()
            && let Err(error) = join_worker(worker)
        {
            failure.get_or_insert(error);
        }
        self.shared.publications.shutdown();
        self.shared.visualisation.shutdown();
        if let Some(worker) = self.publishing.take()
            && let Err(error) = join_worker(worker)
        {
            failure.get_or_insert(error);
        }
        if let Some(worker) = self.visualisation.take()
            && let Err(error) = join_worker(worker)
        {
            failure.get_or_insert(error);
        }
        self.shared.images.shutdown();
        if let Err(error) = self.shared.check_errors() {
            failure.get_or_insert(error);
        }
        failure.map_or(Ok(()), Err)
    }
}
impl Drop for ThreadedSlam {
    fn drop(&mut self) {
        if let Err(error) = self.shutdown() {
            log::error!("{error}");
        }
    }
}
/// Independently cloneable input endpoint; blocking sensor pushes never lock the processing handle.
#[derive(Clone)]
pub struct SensorInputHandle {
    input: SensorInputConfig,
    shared: Arc<Shared>,
}
impl ThreadedSlam {
    /// Backpressures the graph owner instead of dropping committed state updates.
    pub fn enable_lossless_publications(&self) {
        self.shared
            .lossless_publications
            .store(true, Ordering::Release);
    }
    #[must_use]
    pub fn sensor_input_handle(&self) -> SensorInputHandle {
        SensorInputHandle {
            input: self.input.clone(),
            shared: self.shared.clone(),
        }
    }
}
impl SensorInputHandle {
    /// Closes ingestion and wakes estimator waits for sensor coverage.
    pub fn close_inputs(&self) {
        self.shared.close_sensors();
        self.shared.resume.notify_all();
    }

    /// Pending IMU, stereo-pair and depth measurement counts, in that order.
    /// # Errors
    /// Returns an error if a sensor queue mutex is poisoned.
    pub fn pending_measurements(&self) -> Result<[usize; 3]> {
        Ok([
            self.shared.imu.len()?,
            self.shared.cameras.len()?,
            self.shared.depth.len()?,
        ])
    }

    fn push<T>(&self, queue: &ThreadsafeQueue<T>, value: T, capacity: usize) -> Result<bool> {
        let blocking = self.shared.blocking.load(Ordering::Acquire);
        Ok(queue
            .push(value, capacity, blocking)?
            .is_some_and(|dropped| !dropped))
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn add_gps_measurement(
        &self,
        stamp: Timestamp,
        position: &Vector3<f64>,
        sigma: &Vector3<f64>,
    ) -> Result<bool> {
        self.push(
            &self.shared.gps,
            Measurement::new(
                stamp,
                GpsSensorReadings::from_cartesian_sigmas(*position, sigma.x, sigma.y, sigma.z),
                -1,
            ),
            5000,
        )
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn add_geodetic_gps_measurement(
        &self,
        stamp: Timestamp,
        lat: f64,
        lon: f64,
        height: f64,
        h_acc: f64,
        v_acc: f64,
    ) -> Result<bool> {
        self.push(
            &self.shared.gps,
            Measurement::new(
                stamp,
                GpsSensorReadings::from_geodetic(lat, lon, height, h_acc, v_acc),
                -1,
            ),
            5000,
        )
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn add_submap_alignment_constraints(&self, term: AlignmentTerm) -> Result<bool> {
        if term.submap_a.is_some() && term.points_b.len() != term.sensor_errors.len() {
            return Err(runtime_error(
                "alignment point and uncertainty counts differ",
            ));
        }
        self.shared
            .alignments
            .push(term, usize::MAX, false)
            .map(|r| r.is_some())
    }
}
impl ViInterface for ThreadedSlam {
    fn add_images(&mut self, t: Timestamp, images: &ImageMap, depth: &ImageMap) -> bool {
        self.sensor_input_handle().add_images(t, images, depth)
    }
    fn add_imu_measurement(
        &mut self,
        t: Timestamp,
        alpha: &Vector3<f64>,
        omega: &Vector3<f64>,
    ) -> bool {
        self.sensor_input_handle()
            .add_imu_measurement(t, alpha, omega)
    }
    fn add_lidar_measurement(&mut self, t: Timestamp, ray: &Vector3<f64>) -> bool {
        self.sensor_input_handle().add_lidar_measurement(t, ray)
    }
    fn add_depth_measurement(
        &mut self,
        t: Timestamp,
        depth: &DynamicMatrix,
        sigma: Option<&DynamicMatrix>,
    ) -> bool {
        self.sensor_input_handle()
            .add_depth_measurement(t, depth, sigma)
    }
    fn set_optimised_graph_callback(&mut self, callback: OptimisedGraphCallback) {
        self.sensor_input_handle()
            .set_optimised_graph_callback(callback);
    }
    fn set_blocking(&mut self, blocking: bool) {
        self.sensor_input_handle().set_blocking(blocking);
    }
    fn display(&mut self, images: &mut BTreeMap<String, Arc<DynamicMatrix>>) {
        self.sensor_input_handle().display(images);
    }
}
impl ViInterface for SensorInputHandle {
    fn add_images(&mut self, stamp: Timestamp, images: &ImageMap, depth: &ImageMap) -> bool {
        let result = (|| {
            let stamp = shift_time(stamp, -self.input.image_delay)?;
            let mut frames = (0..self.input.used_cameras.len())
                .map(|c| Measurement::new(stamp, CameraData::default(), c as i32))
                .collect::<Vec<_>>();
            let mut use_frame = false;
            for (&c, image) in images {
                if self.input.used_cameras.get(c).copied().unwrap_or(false) {
                    frames[c].measurement.image = Some(grayscale(image)?);
                    frames[c].measurement.delivers_keypoints = Some(false);
                    use_frame = true;
                }
            }
            for (&c, image) in depth {
                if self.input.used_cameras.get(c).copied().unwrap_or(false) {
                    frames[c].measurement.depth_image = Some(image.clone());
                    frames[c].measurement.delivers_keypoints = Some(false);
                }
            }
            if !use_frame {
                return Ok(true);
            }
            let blocking = self.shared.blocking.load(Ordering::Acquire);
            Ok(self
                .shared
                .cameras
                .push(frames, if blocking { 1 } else { 2 }, blocking)?
                .is_some_and(|dropped| !dropped))
        })();
        sensor_result(&self.shared, result)
    }
    fn add_imu_measurement(
        &mut self,
        stamp: Timestamp,
        alpha: &Vector3<f64>,
        omega: &Vector3<f64>,
    ) -> bool {
        if !self.input.use_imu {
            return false;
        }
        let reading =
            ImuSensorReadings::new(*omega, self.input.accelerometer_scale.component_mul(alpha));
        sensor_result(
            &self.shared,
            self.push(&self.shared.imu, Measurement::new(stamp, reading, -1), 5000),
        )
    }
    fn add_lidar_measurement(&mut self, stamp: Timestamp, ray: &Vector3<f64>) -> bool {
        if !self.input.use_alignment {
            return false;
        }
        let count = self
            .shared
            .downsampling_counter
            .fetch_add(1, Ordering::AcqRel)
            + 1;
        if !count.is_multiple_of(self.input.downsampling) {
            return true;
        }
        let blocking = self.shared.blocking.load(Ordering::Acquire);
        let result = self
            .shared
            .lidar
            .push(
                Measurement::new(stamp, LidarSensorReadings::from_ray(*ray), -1),
                500_000,
                blocking,
            )
            .map(|value| value.is_some_and(|dropped| if blocking { true } else { dropped }));
        sensor_result(&self.shared, result)
    }
    fn add_depth_measurement(
        &mut self,
        stamp: Timestamp,
        depth: &DynamicMatrix,
        sigma: Option<&DynamicMatrix>,
    ) -> bool {
        let count = self
            .shared
            .downsampling_counter
            .fetch_add(1, Ordering::AcqRel)
            + 1;
        if !count.is_multiple_of(self.input.downsampling) {
            return true;
        }
        self.shared.downsampling_counter.store(0, Ordering::Release);
        let result = (|| {
            let source = depth
                .as_matrix_f32()
                .ok_or_else(|| runtime_error("depth image requires f32 pixels"))?;
            let scaled = DynamicMatrix::new_f32(
                source.rows,
                source.cols,
                source.channels,
                source
                    .data
                    .iter()
                    .map(|v| v * self.input.depth_scale)
                    .collect(),
            )
            .map_err(|e| runtime_error(e.to_string()))?;
            let data = CameraData {
                depth_image: Some(Arc::new(scaled)),
                sigma_image: sigma.map(|s| Arc::new(s.clone())),
                ..CameraData::default()
            };
            let blocking = self.shared.blocking.load(Ordering::Acquire);
            Ok(self
                .shared
                .depth
                .push(
                    Measurement::new(shift_time(stamp, -self.input.image_delay)?, data, -1),
                    4,
                    blocking,
                )?
                .is_some_and(|dropped| if blocking { true } else { dropped }))
        })();
        sensor_result(&self.shared, result)
    }
    fn set_optimised_graph_callback(&mut self, callback: OptimisedGraphCallback) {
        if let Ok(mut slot) = self.shared.callback.lock() {
            *slot = Some(callback);
        }
    }
    fn set_blocking(&mut self, blocking: bool) {
        self.shared.blocking.store(blocking, Ordering::Release);
    }
    fn display(&mut self, images: &mut BTreeMap<String, Arc<DynamicMatrix>>) {
        match self.shared.images.pop(false) {
            Ok(Some(out)) => images.extend(out),
            Ok(None) => {}
            Err(error) => self.shared.error(&error),
        }
    }
}
struct DepthCamera {
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
}
impl DepthCamera {
    fn back_project(&self, x: usize, y: usize) -> Vector3<f32> {
        Vector3::new(
            (x as f32 - self.cx) / self.fx,
            (y as f32 - self.cy) / self.fy,
            1.0,
        )
    }
}
pub struct ProcessingCore<G: ThreadedGraph, S, B, M> {
    pub graph: G,
    pub frontend: Frontend<G, S, B>,
    pub parameters: ViParameters,
    pub submap_config: SubMapConfig,
    pub mapping: M,
    imu: ImuMeasurementDeque,
    lidar: LidarMeasurementDeque,
    depth: DepthMeasurementDeque,
    gps: GpsMeasurementDeque,
    imu_by_frame: BTreeMap<StateId, ImuMeasurementDeque>,
    last_optimised: Option<State>,
    pre_last_optimised: Option<State>,
    first_frame: bool,
    graph_revision: u64,
    trajectory: Trajectory,
    affected_states: BTreeSet<StateId>,
    lidar_keyframes: BTreeSet<StateId>,
    previous_submap: Option<(u64, StateId)>,
    no_overlap_counter: usize,
    use_alignment_factors: bool,
    t_sd: Transformation,
    t_rect: Transformation,
    depth_camera: Option<DepthCamera>,
    full_graph: Option<JoinHandle<Result<G::FullGraphResult>>>,
}
impl<G, S, B, M> ProcessingCore<G, S, B, M>
where
    G: ThreadedGraph,
    S: AbsolutePoseSolver + CentralRelativePoseSolver + RotationOnlySolver + LoopPoseSolver,
    B: BriskFeatureFactory,
    M: MappingRuntime<G>,
{
    pub fn new(
        graph: G,
        frontend: Frontend<G, S, B>,
        parameters: ViParameters,
        submap_config: SubMapConfig,
        mapping: M,
    ) -> Self {
        let use_alignment_factors = submap_config.factors.use_map2live_factors;
        Self {
            graph,
            frontend,
            parameters,
            submap_config,
            mapping,
            imu: VecDeque::new(),
            lidar: VecDeque::new(),
            depth: VecDeque::new(),
            gps: VecDeque::new(),
            imu_by_frame: BTreeMap::new(),
            last_optimised: None,
            pre_last_optimised: None,
            first_frame: true,
            graph_revision: 0,
            trajectory: Trajectory::default(),
            affected_states: BTreeSet::new(),
            lidar_keyframes: BTreeSet::new(),
            previous_submap: None,
            no_overlap_counter: 0,
            use_alignment_factors,
            t_sd: Transformation::identity(),
            t_rect: Transformation::identity(),
            depth_camera: None,
            full_graph: None,
        }
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn initialise(&mut self) -> Result<()> {
        let n = self.parameters.n_camera_system.num_cameras();
        if n == 0 {
            return Err(runtime_error("at least one camera is required"));
        }
        let frontend = &self.parameters.frontend;
        self.frontend
            .set_brisk_detection_octaves(frontend.octaves as usize);
        self.frontend
            .set_brisk_detection_threshold(frontend.detection_threshold);
        self.frontend
            .set_brisk_detection_absolute_threshold(frontend.absolute_threshold);
        self.frontend
            .set_brisk_matching_threshold(frontend.matching_threshold);
        self.frontend
            .set_brisk_detection_maximum_keypoints(frontend.max_num_keypoints as usize);
        self.frontend
            .set_keyframe_insertion_overlap_threshold(frontend.keyframe_overlap as f32);
        self.graph.add_imu(&self.parameters.imu)?;
        for _ in 0..n {
            self.graph.add_camera(&self.parameters.camera)?;
        }
        if let Some(gps) = &self.parameters.gps {
            self.graph.add_gps(gps)?;
        }
        self.graph
            .set_detector_uniformity_radius(frontend.detection_threshold);
        if self.parameters.estimator.enforce_realtime {
            self.graph.set_optimisation_time_limit(
                self.parameters.estimator.realtime_time_limit,
                self.parameters.estimator.realtime_min_iterations,
            )?;
        }
        if self.parameters.output.enable_submapping {
            if let Some(lidar) = &self.parameters.lidar {
                self.t_sd = lidar.t_sl.clone();
            } else {
                let mut found = false;
                for c in 0..n {
                    let system = &self.parameters.n_camera_system;
                    let ct = system.camera_type(c);
                    if !ct.is_used_mapping {
                        continue;
                    }
                    found = true;
                    let camera;
                    if ct.depth_type.need_rectify {
                        camera = system.rectify_camera_geometry(c);
                        self.t_sd = system.rectify_t_sc(c).as_ref().clone();
                        self.t_rect = system.t_sc(c).inverse().compose(&self.t_sd);
                    } else {
                        camera = system.camera_geometry(c);
                        self.t_sd = system.t_sc(c).as_ref().clone();
                    }
                    let mut intrinsics = DVector::zeros(camera.num_intrinsics());
                    camera.get_intrinsics(&mut intrinsics);
                    let scale = self.submap_config.depth_image_res_downsampling as f32;
                    if scale <= 0.0 {
                        return Err(runtime_error(
                            "depth resolution downsampling must be positive",
                        ));
                    }
                    self.depth_camera = Some(DepthCamera {
                        fx: intrinsics[0] as f32 / scale,
                        fy: intrinsics[1] as f32 / scale,
                        cx: (intrinsics[2] as f32 + 0.5) / scale - 0.5,
                        cy: (intrinsics[3] as f32 + 0.5) / scale - 0.5,
                    });
                    break;
                }
                if !found {
                    self.use_alignment_factors = false;
                }
            }
        } else {
            self.use_alignment_factors = false;
        }
        Ok(())
    }
    fn get_next_frame(&self, shared: &Shared) -> Result<Option<MultiFramePtr>> {
        let Some(frames) = shared.cameras.front()? else {
            return Ok(None);
        };
        if frames.is_empty() {
            return Ok(None);
        }
        let mut frame = MultiFrame::new(&self.parameters.n_camera_system, frames[0].time_stamp, 0);
        let mut success = false;
        for (c, measurement) in frames.iter().enumerate() {
            if let Some(image) = &measurement.measurement.image {
                if image.rows() > 0 && image.cols() > 0 {
                    success = true;
                }
                frame.set_image(c, image.clone());
            }
            if let Some(depth) = &measurement.measurement.depth_image {
                frame.set_depth_image(c, depth.clone());
            }
        }
        Ok(success.then(|| Rc::new(RefCell::new(frame))))
    }
    fn detect(&self, frame: &MultiFramePtr, pose: &Transformation) -> Result<()> {
        let n = frame.borrow().num_frames();
        for c in 0..n {
            if frame
                .borrow()
                .image(c)
                .is_some_and(|i| i.rows() > 0 && i.cols() > 0)
            {
                let t_wc = pose.compose(frame.borrow().t_sc(c));
                self.frontend.detect_and_describe(c, frame, &t_wc, None)?;
            }
        }
        Ok(())
    }
    fn process_frame(&mut self, shared: &Shared) -> Result<bool> {
        self.finish_full_graph(false)?;
        let Some(frame) = self.get_next_frame(shared)? else {
            return Ok(false);
        };
        let time = frame.borrow().timestamp();
        if !self.collect_frame_imu(shared, time)? {
            return Ok(false);
        }
        if self.first_frame {
            while shared.lidar.front()?.is_some_and(|m| m.time_stamp < time) {
                shared.lidar.pop(false)?;
            }
            while shared.gps.front()?.is_some_and(|m| m.time_stamp < time) {
                shared.gps.pop(false)?;
            }
            while shared.depth.front()?.is_some_and(|m| m.time_stamp < time) {
                shared.depth.pop(false)?;
            }
            self.first_frame = false;
        }
        let mut pose = Transformation::identity();
        let mut ran_detection = false;
        if let Some(last) = &self.last_optimised {
            pose = self.predicted_frame_pose(last, time)?;
            while shared.gps.front()?.is_some_and(|m| m.time_stamp < time) {
                if let Some(m) = shared.gps.pop(false)? {
                    self.gps.push_back(m);
                }
            }
            while shared.lidar.front()?.is_some_and(|m| m.time_stamp < time) {
                if let Some(m) = shared.lidar.pop(false)? {
                    self.lidar.push_back(m);
                }
            }
            while shared.depth.front()?.is_some_and(|m| m.time_stamp <= time) {
                if let Some(m) = shared.depth.pop(false)? {
                    self.depth.push_back(m);
                }
            }
        } else {
            if self.parameters.imu.r#use {
                if !ImuError::init_pose(&self.imu, &mut pose) {
                    return Err(runtime_error(
                        "pose could not be initialized from IMU measurements",
                    ));
                }
            } else {
                let s = 2.0f64.sqrt();
                let t_wc = Transformation::new(Vector3::zeros(), Quaternion::new(-s, s, 0.0, 0.0));
                pose = t_wc.compose(&self.parameters.n_camera_system.t_sc(0).inverse());
            }
            self.detect(&frame, &pose)?;
            ran_detection = true;
            if !self.frontend.is_initialized() && frame.borrow().total_keypoints() < 15 {
                shared.cameras.pop(false)?;
                return Ok(false);
            }
        }
        shared.cameras.pop(false)?;
        if !ran_detection {
            self.detect(&frame, &pose)?;
            if !self.frontend.is_initialized() && frame.borrow().total_keypoints() < 15 {
                return Ok(true);
            }
        }
        let ids = self.graph.frame_ids();
        if let Some(&id) = ids.last() {
            self.last_optimised = Some(self.make_state(id, false)?);
            if ids.len() > 1 {
                self.pre_last_optimised = Some(self.make_state(ids[ids.len() - 2], false)?);
            }
        }
        if let Some(last) = &self.last_optimised {
            let oldest = shift_time(last.timestamp, -IMU_TEMPORAL_OVERLAP)?;
            while self.imu.len() > 1 && self.imu[1].time_stamp <= oldest {
                self.imu.pop_front();
            }
        }
        self.associate_and_optimise_frame(shared, &frame, time)
    }
    fn predicted_frame_pose(&self, last: &State, time: Timestamp) -> Result<Transformation> {
        let mut pose;
        pose = last.t_ws.clone();
        if self.parameters.imu.r#use {
            let mut sb = last.speed_and_bias();
            ImuError::propagation(
                &self.imu,
                &self.parameters.imu,
                &mut pose,
                &mut sb,
                (last.timestamp, time),
                (None, None),
            )
            .map_err(integration_error)?;
        } else if let Some(previous) = &self.pre_last_optimised {
            let ratio = seconds(time, last.timestamp) / seconds(last.timestamp, previous.timestamp);
            let translation = last.t_ws.translation()
                + ratio * (last.t_ws.translation() - previous.t_ws.translation());
            let old = previous.t_ws.quaternion();
            let delta = last.t_ws.quaternion() * old.conjugate() / old.norm_squared();
            let axis = crate::ceres::vi_graph::quaternion_angle_axis(&delta) * ratio;
            let increment = crate::kinematics::transformation::delta_q(&axis);
            pose.set(translation, last.t_ws.quaternion() * increment);
        }
        Ok(pose)
    }

    fn collect_frame_imu(&mut self, shared: &Shared, time: Timestamp) -> Result<bool> {
        if self.parameters.imu.r#use {
            let before = shift_time(time, -IMU_TEMPORAL_OVERLAP)?;
            let after = shift_time(time, IMU_TEMPORAL_OVERLAP)?;

            if self.first_frame {
                let Some(first) = shared.imu.front()? else {
                    return Ok(false);
                };
                if before <= first.time_stamp {
                    shared.cameras.pop(false)?;
                    return Ok(false);
                }
                if shared
                    .imu
                    .back()?
                    .is_none_or(|last| last.time_stamp < after)
                {
                    return Ok(false);
                }
            }
            while self.imu.back().is_none_or(|m| m.time_stamp < after) {
                let Some(measurement) = shared.imu.pop(false)? else {
                    return Ok(false);
                };
                self.imu.push_back(measurement);
            }
        }
        Ok(true)
    }

    fn associate_and_optimise_frame(
        &mut self,
        shared: &Shared,
        frame: &MultiFramePtr,
        time: Timestamp,
    ) -> Result<bool> {
        let matching_start = Instant::now();
        if !self.graph.add_states(frame.clone(), &self.imu, false)? {
            return Ok(true);
        }
        let id = StateId::new(frame.borrow().id());
        if self.use_alignment_factors {
            self.add_live_factors(shared, id, time)?;
        }
        self.imu_by_frame.insert(id, self.imu.clone());
        let kf_prior = self.submap_config.factors.use_map2live_factors
            && self.parameters.lidar.is_some()
            && self.needs_new_lidar_keyframe();
        if kf_prior {
            self.lidar_keyframes.insert(id);
        }
        let mut as_keyframe = false;
        if !self.frontend.data_association_and_initialization(
            &mut self.graph,
            &self.parameters,
            frame,
            kf_prior,
            &mut as_keyframe,
        )? && !self.frontend.is_initialized()
        {
            self.frontend.clear()?;
            self.graph.clear()?;
            return Ok(false);
        }
        self.graph.set_keyframe(id, as_keyframe)?;
        self.graph.add_gps_measurements(&self.gps, &self.imu)?;
        while self.gps.front().is_some_and(|m| m.time_stamp < time) {
            self.gps.pop_front();
        }
        while self.lidar.front().is_some_and(|m| m.time_stamp < time) {
            self.lidar.pop_front();
        }
        while self.depth.front().is_some_and(|m| m.time_stamp <= time) {
            self.depth.pop_front();
        }
        self.add_alignment_terms(shared, false)?;
        let mut gyro = Vector3::zeros();
        for m in self.imu.iter().rev() {
            if m.time_stamp <= time {
                break;
            }
            gyro = m.measurement.gyroscopes;
        }
        if self.parameters.estimator.enforce_realtime {
            let dt = self.parameters.estimator.realtime_time_limit
                - matching_start.elapsed().as_secs_f64();
            self.graph.set_optimisation_time_limit(
                if dt < 0.0 { 0.01 } else { dt },
                self.parameters.estimator.realtime_min_iterations,
            )?;
        }
        if self.graph.needs_full_graph_optimisation() {
            self.finish_full_graph(true)?;
            let task = self.graph.prepare_full_graph_task(
                self.parameters.estimator.full_graph_iterations,
                self.parameters.estimator.full_graph_num_threads,
            )?;
            self.full_graph = Some(thread::spawn(task));
        }
        self.optimise_publish_marginalise(shared, frame, gyro)?;
        Ok(true)
    }

    fn needs_new_lidar_keyframe(&mut self) -> bool {
        if self.previous_submap.is_some() && self.no_overlap_counter > 1 {
            self.no_overlap_counter = 0;
            true
        } else {
            false
        }
    }
    fn add_live_factors(&mut self, shared: &Shared, id: StateId, time: Timestamp) -> Result<()> {
        let Some((map, submap_id)) = self.previous_submap else {
            return Ok(());
        };
        let pose = self.graph.pose(id);
        if self.parameters.lidar.is_some() {
            let Some(initial) = self.last_optimised.as_ref() else {
                return Ok(());
            };
            let mut undistortion = LidarMotionUndistortion::new(
                initial,
                pose.clone(),
                self.t_sd.clone(),
                self.lidar.clone(),
                self.imu.clone(),
            );
            undistortion.deskew().map_err(integration_error)?;
            let observed =
                undistortion.filter_observed(&self.mapping.map(map)?, &self.graph.pose(submap_id));
            if observed < self.submap_config.num_submap_factors {
                self.no_overlap_counter += 1;
            } else {
                self.no_overlap_counter = 0;
            }
            undistortion.downsample(
                self.submap_config.num_submap_factors,
                f64::from(self.submap_config.voxel_grid_resolution),
            );
            let points = undistortion.deskewed_downsampled_point_cloud();
            publish_alignment(shared, time, &pose, points, false)?;
            let errors = vec![self.submap_config.sensor_error; points.len()];
            self.mapping.add_constraints(
                &mut self.graph,
                map,
                (submap_id.value(), id.value()),
                (points, &errors),
                true,
                "Tukey",
            )?;
        } else {
            let (points, sigmas) = self.compute_live_depth_measurements(&pose, time)?;
            if points.len() as f64 > 0.05 * self.submap_config.num_submap_factors as f64 {
                publish_alignment(shared, time, &pose, &points, false)?;
                self.mapping.add_constraints(
                    &mut self.graph,
                    map,
                    (submap_id.value(), id.value()),
                    (&points, &sigmas),
                    false,
                    "Tukey",
                )?;
            }
        }
        Ok(())
    }
    fn add_alignment_terms(&mut self, shared: &Shared, final_ba: bool) -> Result<()> {
        while let Some(term) = shared.alignments.pop(false)? {
            let lidar = self.parameters.lidar.is_some();
            if let Some(map) = term.submap_a {
                self.mapping.add_constraints(
                    &mut self.graph,
                    map,
                    (term.frame_a_id, term.frame_b_id),
                    (&term.points_b, &term.sensor_errors),
                    lidar,
                    "Tukey",
                )?;
            }
            if final_ba || lidar {
                self.previous_submap = Some((term.submap_b, StateId::new(term.frame_b_id)));
            } else if term.frame_a_id != 0 {
                self.previous_submap = term
                    .submap_a
                    .map(|map| (map, StateId::new(term.frame_a_id)));
            } else {
                self.previous_submap = Some((term.submap_b, StateId::new(term.frame_b_id)));
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn compute_live_depth_measurements(
        &mut self,
        pose: &Transformation,
        time: Timestamp,
    ) -> Result<(Vec<Vector3<f32>>, Vec<f32>)> {
        let mut live_points = Vec::new();
        let mut live_sigmas = Vec::new();
        let Some((handle, submap_id)) = self.previous_submap else {
            return Ok((live_points, live_sigmas));
        };
        let Some(camera) = self.depth_camera.as_ref() else {
            return Ok((live_points, live_sigmas));
        };
        let t_wd = pose.compose(&self.t_sd);
        let map_from_depth = self.graph.pose(submap_id).inverse().compose(&t_wd);
        let map = self.mapping.map(handle)?;
        for measurement in &self.depth {
            let source = measurement
                .measurement
                .depth_image
                .as_ref()
                .and_then(|d| d.as_matrix_f32())
                .ok_or_else(|| runtime_error("depth measurement must contain f32 depth image"))?;
            let factor = self.submap_config.depth_image_res_downsampling;
            if factor == 0 {
                return Err(runtime_error("depth downsampling must be positive"));
            }
            let depth = nearest_downsample(source, factor);
            let sigma = if self.submap_config.use_uncertainty {
                measurement
                    .measurement
                    .sigma_image
                    .as_ref()
                    .and_then(|s| s.as_matrix_f32())
                    .map(|s| nearest_downsample(s, factor))
            } else {
                None
            };
            let delta = Self::depth_alignment_delta(
                &mut self.trajectory,
                &self.t_sd,
                pose,
                time,
                measurement.time_stamp,
                &t_wd,
            )?;
            let map_transform = map_from_depth.compose(&delta).matrix_3x4().cast::<f32>();
            let sensor_transform = self.t_sd.compose(&delta).matrix_3x4().cast::<f32>();
            let mut points = Vec::new();
            let mut sigmas = Vec::new();
            for x in (0..depth.cols).step_by(10) {
                for y in (0..depth.rows).step_by(10) {
                    let z = depth.data[y * depth.cols + x];
                    let sigma = sigma
                        .as_ref()
                        .map_or(self.submap_config.sensor_error, |s| s.data[y * s.cols + x]);
                    if z < self.submap_config.near_plane
                        || z > self.submap_config.far_plane
                        || sigma > 3.0
                    {
                        continue;
                    }
                    let point = camera.back_project(x, y) * z;
                    let map_point = map_transform * point.push(1.0);
                    let occupancy = interp_field_mean_occup(&map, &map_point, 0, Safe::On, None);
                    let gradient = grad_field_mean_occup(&map, &map_point, 0, Safe::On, None);
                    if let (Some(occupancy), Some(gradient)) = (occupancy, gradient)
                        && occupancy > -2.0
                        && occupancy < 2.0
                        && gradient.norm() >= 1.0e-3
                    {
                        points.push(sensor_transform * point.push(1.0));
                        sigmas.push(sigma);
                    }
                }
            }
            if points.len() > self.submap_config.num_submap_factors {
                if !downsample_points_uncertainty(
                    &points,
                    &sigmas,
                    &mut live_points,
                    &mut live_sigmas,
                    self.submap_config.num_submap_factors,
                ) {
                    return Err(runtime_error("invalid uncertainty sample"));
                }
            } else {
                live_points = points;
                live_sigmas = sigmas;
            }
        }
        Ok((live_points, live_sigmas))
    }
    fn depth_alignment_delta(
        trajectory: &mut Trajectory,
        t_sd: &Transformation,
        pose: &Transformation,
        time: Timestamp,
        measurement_time: Timestamp,
        world_from_depth: &Transformation,
    ) -> Result<Transformation> {
        let mut delta = Transformation::identity();
        if measurement_time != time {
            let state = if let Some(state) = trajectory
                .get_state(measurement_time)
                .map_err(integration_error)?
            {
                Some(state.t_ws)
            } else if let Some(&latest) = trajectory.state_ids().last() {
                let latest = trajectory
                    .state_by_id(latest)
                    .ok_or_else(|| runtime_error("trajectory state missing"))?;
                let dt = seconds(time, latest.timestamp);
                let dt0 = seconds(measurement_time, latest.timestamp);
                let weight = (dt0 / dt) as f32;
                if weight > 0.0 && weight < 1.0 {
                    let w = f64::from(weight);
                    Some(Transformation::new(
                        (1.0 - w) * latest.t_ws.translation() + w * pose.translation(),
                        slerp(latest.t_ws.quaternion(), pose.quaternion(), w),
                    ))
                } else {
                    None
                }
            } else {
                None
            };
            let Some(state) = state else {
                return Err(runtime_error(
                    "depth timestamp has no trajectory/interpolation pose",
                ));
            };
            delta = world_from_depth.inverse().compose(&state.compose(t_sd));
        }
        Ok(delta)
    }

    fn finish_full_graph(&mut self, wait: bool) -> Result<()> {
        if self
            .full_graph
            .as_ref()
            .is_some_and(|worker| wait || worker.is_finished())
        {
            let worker = self.full_graph.take().unwrap();
            let result = worker
                .join()
                .map_err(|_| runtime_error("full graph optimization worker panicked"))??;
            self.graph.finish_full_graph_task(result)?;
        }
        Ok(())
    }
    fn optimise_publish_marginalise(
        &mut self,
        shared: &Shared,
        frame: &MultiFramePtr,
        gyro: Vector3<f64>,
    ) -> Result<()> {
        let updated = self.graph.optimise_realtime_graph(
            self.parameters.estimator.realtime_max_iterations,
            self.parameters.estimator.realtime_num_threads,
            self.frontend.is_initialized(),
        )?;
        self.finish_full_graph(false)?;
        let updated = if self.graph.is_loop_closure_available() {
            if self.graph.is_loop_closing() {
                return Err(runtime_error(
                    "loop closure available while optimization is still running",
                ));
            }
            let synchronized = self.graph.synchronise_realtime_and_full_graph()?;
            self.graph_revision = self
                .graph_revision
                .checked_add(1)
                .ok_or_else(|| runtime_error("graph revision exhausted"))?;
            if synchronized.is_empty() {
                updated
            } else {
                synchronized
            }
        } else {
            updated
        };
        let id = StateId::new(frame.borrow().id());
        let mut state = self.make_state(id, false)?;
        state.omega_s = gyro - state.b_g;
        state.is_online_extrinsics = Some(self.parameters.camera.online_calibration.do_extrinsics);
        if self.parameters.camera.online_calibration.do_extrinsics && self.depth_camera.is_some() {
            for c in 0..self.parameters.n_camera_system.num_cameras() {
                let camera = self.parameters.n_camera_system.camera_type(c);
                if camera.is_used_mapping {
                    let extrinsics = self.graph.extrinsics(id, c)?;
                    let extrinsics = if camera.depth_type.need_rectify {
                        extrinsics.compose(&self.t_rect)
                    } else {
                        extrinsics
                    };
                    state.extrinsics.insert(c as u64, extrinsics.clone());
                    self.t_sd = extrinsics;
                }
            }
        }
        state.t_ai_w = self
            .graph
            .component_alignments(id)
            .into_iter()
            .map(|(component, alignment)| (component, alignment.compose(&state.t_ws.inverse())))
            .collect();
        let publication = self.prepare_publication(state, &updated, false)?;
        shared.started.store(true, Ordering::Release);
        Self::publish(shared, publication.clone())?;
        if self.parameters.output.display.display_matches
            || self.parameters.output.display.display_overhead
        {
            let data = self.prepare_visualisation(frame, publication)?;
            shared.visualisation.push(data, 1, false)?;
        }
        let result = self.graph.apply_strategy(
            self.parameters.estimator.num_keyframes as usize,
            self.parameters.estimator.num_loop_closure_frames as usize,
            self.parameters.estimator.num_imu_frames as usize,
        )?;
        self.affected_states = result.affected_states;
        for id in result.eliminated_states {
            self.imu_by_frame.remove(&id);
        }
        Ok(())
    }
    fn make_state(&self, id: StateId, changed: bool) -> Result<State> {
        let pose = self.graph.pose(id);
        let sb = self.graph.speed_and_bias(id)?;
        let timestamp = self.graph.timestamp(id)?;
        let imu = self
            .imu_by_frame
            .get(&id)
            .cloned()
            .ok_or_else(|| runtime_error("state has no recorded IMU measurement deque"))?;
        let omega = imu
            .iter()
            .rev()
            .find(|m| m.time_stamp < timestamp)
            .map_or(Vector3::zeros(), |m| {
                m.measurement.gyroscopes - sb.fixed_rows::<3>(3).into_owned()
            });
        let mut state = State::new(
            pose.clone(),
            sb.fixed_rows::<3>(0).into_owned(),
            (
                sb.fixed_rows::<3>(3).into_owned(),
                sb.fixed_rows::<3>(6).into_owned(),
            ),
            omega,
            timestamp,
            id,
            self.graph.is_keyframe(id),
        );
        state.previous_imu_measurements = imu;
        state.state_changed = Some(changed);
        state.is_online_extrinsics = Some(false);
        state.covisible_frame_ids = self.graph.observed_ids(id);
        state.gps_points = self.graph.gps_points(id);
        state.t_gw = self.graph.t_gw();
        Ok(state)
    }
    fn prepare_publication(
        &mut self,
        mut state: State,
        updated: &[StateId],
        final_output: bool,
    ) -> Result<PublicationData> {
        let id = state.id;
        state.state_changed = None;
        state.t_gw = Transformation::identity();
        state.gps_points.clear();
        if final_output {
            state.covisible_frame_ids.clear();
            state.is_online_extrinsics = None;
        }
        let quality = self.graph.tracking_quality(id);
        let tracking = TrackingState {
            id,
            is_keyframe: state.is_keyframe,
            is_lidar_keyframe: self.lidar_keyframes.contains(&id),
            tracking_quality: if quality < 0.01 {
                TrackingQuality::Lost
            } else if quality < 0.3 {
                TrackingQuality::Marginal
            } else {
                TrackingQuality::Good
            },
            backend: crate::runtime::vi_interface::BackendTrackingState {
                revision: self.graph_revision,
                recognised_place: self.graph.closed_loop(id),
                is_full_graph_optimising: self.full_graph.is_some(),
            },

            current_keyframe_id: self.graph.most_overlapped_state_id(id),
        };
        let mut states = BTreeMap::new();
        for &id in updated {
            states.insert(id, self.make_state(id, true)?);
        }
        for &id in &self.affected_states {
            states.insert(id, self.make_state(id, false)?);
        }
        self.affected_states.clear();
        if final_output {
            for state in states.values_mut() {
                state.t_gw = Transformation::identity();
                state.gps_points.clear();
            }
        }
        let landmarks = self
            .graph
            .landmarks()
            .into_iter()
            .map(|(id, lm)| {
                let latest = if final_output {
                    0
                } else {
                    lm.observations.last().map_or(0, |obs| obs.frame_id)
                };
                MapPoint::new(id.value(), lm.point, lm.quality, latest)
            })
            .collect();
        let publication = PublicationData {
            state,
            tracking_state: tracking,
            updated_states: Arc::new(states),
            landmarks: Arc::new(landmarks),
        };
        if !publication.updated_states.is_empty() {
            let mut affected = BTreeSet::new();
            self.trajectory
                .update(
                    &publication.tracking_state,
                    &publication.updated_states,
                    &mut affected,
                )
                .map_err(integration_error)?;
        }
        Ok(publication)
    }
    fn publish(shared: &Shared, data: PublicationData) -> Result<()> {
        let blocking = shared.blocking.load(Ordering::Acquire)
            || shared.lossless_publications.load(Ordering::Acquire);
        if shared.shutdown.load(Ordering::Acquire) {
            let callback = shared
                .callback
                .lock()
                .map_err(|_| runtime_error("callback mutex poisoned"))?
                .clone();
            if let Some(callback) = callback {
                callback(
                    &data.state,
                    &data.tracking_state,
                    data.updated_states,
                    data.landmarks,
                );
            }
            return Ok(());
        }
        let dropped = shared
            .publications
            .push(data, if blocking { 1 } else { 3 }, blocking)?;
        match dropped {
            Some(true) => log::error!("publication full update overrun: dropping oldest update"),
            Some(false) => {}
            None => return Err(runtime_error("publication worker stopped")),
        }
        Ok(())
    }
    fn prepare_visualisation(
        &self,
        frame: &MultiFramePtr,
        publication: PublicationData,
    ) -> Result<VisualisationData> {
        let mf = frame.borrow();
        let points = self.graph.landmarks();
        let mut images = BTreeMap::new();
        let mut observations = Vec::with_capacity(mf.total_keypoints());
        let mut extrinsics = Vec::new();
        for c in 0..mf.num_frames() {
            if let Some(image) = mf.image(c) {
                images.insert(c, image.clone());
            }
            extrinsics.push(self.graph.extrinsics(StateId::new(mf.id()), c)?);
            for k in 0..mf.num_keypoints(c) {
                let mut pixel = Vector2::zeros();
                mf.get_keypoint(c, k, &mut pixel);
                let mut size = 0.0;
                mf.get_keypoint_size(c, k, &mut size);
                let id = LandmarkId::new(mf.landmark_id(c, k));
                let point = points.get(&id).filter(|p| {
                    p.observations.contains(
                        &crate::frames::frame_typedefs::KeypointIdentifier::new(mf.id(), c, k),
                    )
                });
                let observation = Observation::new(
                    (k, pixel, size),
                    c,
                    mf.id(),
                    (
                        point.map_or(nalgebra::Vector4::zeros(), |p| p.point),
                        id.value(),
                        point.is_some_and(|p| p.is_initialised),
                    ),
                    point.map_or(-1, |p| p.classification),
                );
                observations.push(observation);
            }
        }
        Ok(VisualisationData {
            publication,
            images,
            observations,
            extrinsics,
        })
    }
    fn drain(&mut self, shared: &Shared) -> Result<()> {
        while self.process_frame(shared)? {}
        self.frontend.end_cnn_threads()?;
        self.finish_full_graph(true)?;
        if self.graph.is_loop_closure_available() {
            let updated = self.graph.synchronise_realtime_and_full_graph()?;
            if let Some(&id) = self.graph.frame_ids().last() {
                let mut state = self.make_state(id, false)?;
                state.omega_s = Vector3::zeros();
                for id in &updated {
                    self.affected_states.remove(id);
                }
                let publication = self.prepare_publication(state, &updated, true)?;
                Self::publish(shared, publication)?;
            }
        }
        Ok(())
    }
    fn final_ba(&mut self, shared: &Shared) -> Result<()> {
        self.add_alignment_terms(shared, true)?;
        self.drain(shared)?;
        let calibration = &self.parameters.camera.online_calibration;
        let (sigma_r, sigma_alpha) = if calibration.do_extrinsics_final_ba {
            (
                calibration.sigma_r_final_ba,
                calibration.sigma_alpha_final_ba,
            )
        } else {
            (0.0, 0.0)
        };
        let updated = self.graph.do_final_ba(
            100,
            sigma_r,
            sigma_alpha,
            self.parameters.estimator.realtime_num_threads
                + self.parameters.estimator.full_graph_num_threads,
        )?;
        if let Some(&id) = self.graph.frame_ids().last() {
            let mut state = self.make_state(id, false)?;
            state.omega_s = Vector3::zeros();
            let publication = self.prepare_publication(state, &updated, true)?;
            Self::publish(shared, publication)?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid/missing state, frame or measurement data, or a failed
    /// estimator, graph, mapper, queue, callback or worker operation.
    pub fn save_aligned_points(
        &self,
        id: StateId,
        points: &[Vector3<f32>],
        path: &Path,
    ) -> Result<()> {
        let pose = self.graph.pose(id);
        let mut file =
            std::io::BufWriter::new(std::fs::File::create(path).map_err(|error| io_error(&error))?);
        writeln!(file,"ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nend_header",points.len()).map_err(|error| io_error(&error))?;
        for point in points {
            let p = pose.rotation_matrix() * point.cast::<f64>() + pose.translation();
            writeln!(file, "{} {} {}", p.x, p.y, p.z).map_err(|error| io_error(&error))?;
        }
        file.flush().map_err(|error| io_error(&error))
    }
}
impl<G: ThreadedGraph, S, B, M> Drop for ProcessingCore<G, S, B, M> {
    fn drop(&mut self) {
        if let Some(worker) = self.full_graph.take() {
            match worker.join() {
                Ok(Ok(_)) => {}
                Ok(Err(error)) => log::error!("{error}"),
                Err(_) => log::error!("full graph optimization worker panicked"),
            }
        }
    }
}
impl LidarInitialState for State {
    fn id(&self) -> StateId {
        self.id
    }
    fn timestamp(&self) -> Timestamp {
        self.timestamp
    }
    fn t_ws(&self) -> &Transformation {
        &self.t_ws
    }
    fn v_w(&self) -> Vector3<f64> {
        self.v_w
    }
    fn b_g(&self) -> Vector3<f64> {
        self.b_g
    }
    fn b_a(&self) -> Vector3<f64> {
        self.b_a
    }
}
fn publish_alignment(
    shared: &Shared,
    time: Timestamp,
    pose: &Transformation,
    points: &[Vector3<f32>],
    is_submap: bool,
) -> Result<()> {
    let callback = shared
        .alignment_callback
        .lock()
        .map_err(|_| runtime_error("alignment callback mutex poisoned"))?
        .clone();
    if let Some(callback) = callback {
        callback(time, pose, points, is_submap);
    }
    Ok(())
}
fn runtime_error(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::Internal, message.into()).with_operation("ThreadedSlam")
}
fn io_error(error: &std::io::Error) -> Error {
    runtime_error(error.to_string())
}
fn integration_error(error: impl std::fmt::Debug) -> Error {
    runtime_error(format!("{error:?}"))
}
fn join_worker(worker: JoinHandle<Result<()>>) -> Result<()> {
    worker
        .join()
        .map_err(|_| runtime_error("SLAM worker panicked"))?
}
fn sensor_result(shared: &Shared, result: Result<bool>) -> bool {
    match result {
        Ok(value) => value,
        Err(error) => {
            shared.error(&error);
            false
        }
    }
}
fn shift_time(time: Timestamp, seconds: f64) -> Result<Timestamp> {
    if !seconds.is_finite() || seconds.abs() > i64::MAX as f64 / 1e9 {
        return Err(runtime_error("timestamp offset is invalid"));
    }
    time.checked_add(SignedDuration::from_nanos((seconds * 1e9).round() as i64))
        .map_err(|e| runtime_error(e.to_string()))
}
fn grayscale(image: &Arc<DynamicMatrix>) -> Result<Arc<DynamicMatrix>> {
    let matrix = image
        .as_matrix_u8()
        .ok_or_else(|| runtime_error("camera image must contain u8 pixels"))?;
    if matrix.channels == 1 {
        return Ok(image.clone());
    }
    let gray = purecv::imgproc::cvt_color_bgr_to_gray(matrix).map_err(runtime_error)?;
    DynamicMatrix::new_u8(gray.rows, gray.cols, 1, gray.data.clone())
        .map(Arc::new)
        .map_err(|e| runtime_error(e.to_string()))
}
fn nearest_downsample(image: &Matrix<f32>, factor: usize) -> Matrix<f32> {
    let cols = (image.cols as f64 / factor as f64).round_ties_even() as usize;
    let rows = (image.rows as f64 / factor as f64).round_ties_even() as usize;
    let mut out = Matrix::new(rows, cols, image.channels);
    for y in 0..rows {
        for x in 0..cols {
            for c in 0..image.channels {
                out.data[(y * cols + x) * image.channels + c] =
                    image.data[((y * factor).min(image.rows - 1) * image.cols
                        + (x * factor).min(image.cols - 1))
                        * image.channels
                        + c];
            }
        }
    }
    out
}
fn slerp(a: Quaternion<f64>, b: Quaternion<f64>, weight: f64) -> Quaternion<f64> {
    let dot = a.coords.dot(&b.coords);
    let abs_dot = dot.abs();
    let (scale_a, mut scale_b) = if abs_dot >= 1.0 - f64::EPSILON {
        (1.0 - weight, weight)
    } else {
        let theta = abs_dot.acos();
        let sin = theta.sin();
        (
            ((1.0 - weight) * theta).sin() / sin,
            (weight * theta).sin() / sin,
        )
    };
    if dot < 0.0 {
        scale_b = -scale_b;
    }
    a * scale_a + b * scale_b
}

/// A registry of shared occupancy-map handles used by the graph owner.
#[derive(Clone)]
pub struct OccupancyMappingRuntime {
    pub maps: crate::mapping::map_typedefs::SharedOccupancyMaps,
}
impl OccupancyMappingRuntime {
    pub fn new(maps: crate::mapping::map_typedefs::SharedOccupancyMaps) -> Self {
        Self { maps }
    }
}
impl MappingRuntime<crate::ceres::vi_slam_backend::ViSlamBackend> for OccupancyMappingRuntime {
    type Map = crate::mapping::map_typedefs::OccupancyMap;
    fn map(&self, handle: u64) -> Result<Self::Map> {
        self.maps
            .read()
            .map_err(|_| runtime_error("occupancy registry lock poisoned"))?
            .get(&handle)
            .cloned()
            .ok_or_else(|| runtime_error("occupancy map handle absent"))
    }
    fn add_constraints(
        &mut self,
        graph: &mut crate::ceres::vi_slam_backend::ViSlamBackend,
        map: u64,
        (frame_a, frame_b): (u64, u64),
        (points, sigmas): (&[Vector3<f32>], &[f32]),
        lidar: bool,
        loss: &str,
    ) -> Result<()> {
        let map =
            <Self as MappingRuntime<crate::ceres::vi_slam_backend::ViSlamBackend>>::map(self, map)?;
        graph.add_submap_alignment_constraints(
            &(Rc::new(map) as Rc<dyn crate::ceres::submap_icp_error::OccupancySubmap>),
            (StateId::new(frame_a), StateId::new(frame_b)),
            points,
            sigmas,
            lidar,
            loss,
        )?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        application::live::config::Config,
        ceres::vi_slam_backend::ViSlamBackend,
        frontend::{
            frame_absolute_pose_sac_problem::OpenGvSolver, pipeline::RustBriskFeatureFactory,
        },
    };

    fn marginalisation_core()
    -> ProcessingCore<ViSlamBackend, OpenGvSolver, RustBriskFeatureFactory, OccupancyMappingRuntime>
    {
        let config = Config {
            mapping: false,
            ..Config::default()
        };
        let mut parameters = config.parameters().unwrap();
        parameters.estimator.enforce_realtime = false;
        parameters.estimator.num_imu_frames = 2;
        parameters.estimator.realtime_max_iterations = 1;
        let solver: OpenGvSolver = OpenGvSolver::default();
        let frontend = Frontend::with_vocabulary(
            parameters.n_camera_system.num_cameras(),
            &firefly_dbow2::templated_vocabulary::TemplatedVocabulary::default(),
            solver,
            RustBriskFeatureFactory,
        );
        let mut core = ProcessingCore::new(
            ViSlamBackend::default(),
            frontend,
            parameters,
            SubMapConfig::default(),
            OccupancyMappingRuntime::new(Arc::default()),
        );
        core.initialise().unwrap();
        core
    }

    fn check_marginalised_frame_imu(full_graph_busy: bool) {
        let mut core = marginalisation_core();
        let shared = Shared::default();
        let mut publications = Vec::new();
        let mut task = None;
        for index in 1..=6 {
            let time = Timestamp::from_millisecond((index - 1) * 50).unwrap();
            let frame = Rc::new(RefCell::new(MultiFrame::new(
                &core.parameters.n_camera_system,
                time,
                index as u64,
            )));
            let imu = (-1..=index * 10)
                .map(|i| {
                    ImuMeasurement::new(
                        Timestamp::from_millisecond(i * 5).unwrap(),
                        ImuSensorReadings::new(Vector3::zeros(), Vector3::z() * 9.81),
                        -1,
                    )
                })
                .collect();
            assert!(
                core.graph
                    .add_states(frame.clone(), &imu, index == 1 || index == 3)
                    .unwrap()
            );
            let id = StateId::new(frame.borrow().id());
            core.imu_by_frame.insert(id, imu);
            core.optimise_publish_marginalise(&shared, &frame, Vector3::zeros())
                .unwrap();
            publications.push(shared.publications.pop(false).unwrap().unwrap());
            if full_graph_busy && index == 3 {
                task = Some(core.graph.prepare_full_graph_task(1, 1).unwrap());
            }
        }
        assert_eq!(
            core.imu_by_frame
                .keys()
                .map(StateId::value)
                .collect::<Vec<_>>(),
            vec![1, 3, 5, 6]
        );
        for value in [2, 4] {
            let id = StateId::new(value);
            assert!(!core.graph.realtime_graph.states.contains_key(&id));
            assert!(core.graph.realtime_graph.any_state.contains_key(&id));
            assert!(
                !publications[value as usize - 1]
                    .state
                    .previous_imu_measurements
                    .is_empty()
            );
            assert!(
                !core
                    .trajectory
                    .state_by_id(id)
                    .unwrap()
                    .previous_imu_measurements
                    .is_empty()
            );
            assert_eq!(
                core.graph.full_graph.states.contains_key(&id),
                full_graph_busy && value == 2
            );
            assert_eq!(
                core.graph.eliminate_states.contains_key(&id),
                full_graph_busy
            );
        }
        if let Some(task) = task {
            core.graph.finish_full_graph_task(task().unwrap()).unwrap();
            let updated = core.graph.synchronise_realtime_and_full_graph().unwrap();
            assert!(core.graph.eliminate_states.is_empty());
            assert!(!updated.contains(&StateId::new(2)));
            assert!(!updated.contains(&StateId::new(4)));
            let state = core.make_state(StateId::new(6), false).unwrap();
            let publication = core.prepare_publication(state, &updated, true).unwrap();
            assert!(!publication.updated_states.is_empty());
        }
        for id in core.graph.realtime_graph.states.keys() {
            assert!(
                !core
                    .make_state(*id, false)
                    .unwrap()
                    .previous_imu_measurements
                    .is_empty()
            );
        }
        let removed = StateId::new(5);
        let result = core.graph.apply_strategy(5, 5, 1, true).unwrap();
        assert_eq!(result.eliminated_states, BTreeSet::from([removed]));
        assert!(!result.affected_states.contains(&removed));
        assert!(
            core.graph
                .apply_strategy(5, 5, 1, true)
                .unwrap()
                .eliminated_states
                .is_empty()
        );
    }

    #[test]
    fn marginalisation_releases_only_eliminated_frame_imu() {
        check_marginalised_frame_imu(false);
    }

    #[test]
    fn marginalisation_releases_frame_imu_before_full_graph_sync() {
        check_marginalised_frame_imu(true);
    }

    #[test]
    fn depth_pose_slerp_uses_eigens_near_equal_linear_branch() {
        let a = Quaternion::identity();
        let b = Quaternion::new(1., 1e-9, 0., 0.);
        let interpolated = slerp(a, b, 0.5);
        assert_eq!(interpolated, Quaternion::new(1., 0.5e-9, 0., 0.));
        assert_eq!(slerp(a, -b, 0.5), interpolated);
    }

    #[test]
    fn queue_fifo_overload_and_zero_capacity_are_explicit() {
        let queue = ThreadsafeQueue::default();
        assert!(queue.push(0, 0, false).is_err());
        for value in 0..20 {
            assert_eq!(queue.push(value, 3, false).unwrap(), Some(value >= 3));
        }
        assert_eq!(queue.front().unwrap(), Some(17));
        assert_eq!(queue.back().unwrap(), Some(19));
        for value in 17..20 {
            assert_eq!(queue.pop(false).unwrap(), Some(value));
        }
        assert_eq!(queue.pop(false).unwrap(), None);
    }

    #[test]
    fn blocked_producer_resumes_after_pop_and_shutdown_wakes_waiting_consumers() {
        let queue = Arc::new(ThreadsafeQueue::default());
        queue.push(1, 1, false).unwrap();
        let writer = queue.clone();
        let (ready, started) = mpsc::channel();
        let (done, result) = mpsc::channel();
        let producer = thread::spawn(move || {
            ready.send(()).unwrap();
            done.send(writer.push(2, 1, true).unwrap()).unwrap();
        });
        started
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap();
        assert_eq!(
            result.recv_timeout(std::time::Duration::from_millis(20)),
            Err(mpsc::RecvTimeoutError::Timeout)
        );
        assert_eq!(queue.pop(false).unwrap(), Some(1));
        assert_eq!(
            result
                .recv_timeout(std::time::Duration::from_secs(5))
                .unwrap(),
            Some(false)
        );
        producer.join().unwrap();
        assert_eq!(queue.pop(false).unwrap(), Some(2));
        let reader = queue.clone();
        let (ready, started) = mpsc::channel();
        let (done, result) = mpsc::channel();
        let consumer = thread::spawn(move || {
            ready.send(()).unwrap();
            done.send(reader.pop(true).unwrap()).unwrap();
        });
        started
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap();
        assert_eq!(
            result.recv_timeout(std::time::Duration::from_millis(20)),
            Err(mpsc::RecvTimeoutError::Timeout)
        );
        queue.shutdown();
        assert_eq!(
            result
                .recv_timeout(std::time::Duration::from_secs(5))
                .unwrap(),
            None
        );
        consumer.join().unwrap();
        assert_eq!(queue.push(3, 1, true).unwrap(), None);
    }

    #[test]
    fn shutdown_drains_queued_items_and_unblocks_a_full_queue_writer() {
        let queue = Arc::new(ThreadsafeQueue::default());
        queue.push(1, 1, false).unwrap();
        let writer = queue.clone();
        let (ready, started) = mpsc::channel();
        let (done, result) = mpsc::channel();
        let producer = thread::spawn(move || {
            ready.send(()).unwrap();
            done.send(writer.push(2, 1, true).unwrap()).unwrap();
        });
        started
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap();
        assert_eq!(
            result.recv_timeout(std::time::Duration::from_millis(20)),
            Err(mpsc::RecvTimeoutError::Timeout)
        );
        queue.shutdown();
        assert_eq!(
            result
                .recv_timeout(std::time::Duration::from_secs(5))
                .unwrap(),
            None
        );
        producer.join().unwrap();
        assert_eq!(queue.pop(true).unwrap(), Some(1));
        assert_eq!(queue.pop(true).unwrap(), None);
    }

    fn sensor_handle() -> SensorInputHandle {
        SensorInputHandle {
            input: SensorInputConfig {
                used_cameras: vec![true, true],
                colour_cameras: vec![false, false],
                image_delay: 0.005,
                use_imu: true,
                accelerometer_scale: Vector3::new(2.0, 3.0, 4.0),
                use_alignment: false,
                downsampling: 1,
                depth_scale: 0.001,
            },
            shared: Arc::new(Shared::default()),
        }
    }

    #[test]
    fn image_input_preserves_submitted_order_and_missing_camera_without_fabricating_images() {
        let mut handle = sensor_handle();
        let image = Arc::new(DynamicMatrix::new_u8(3, 4, 1, vec![17; 12]).unwrap());
        let images = BTreeMap::from([(0, image)]);
        for millis in [30, 10, 10, 50] {
            let stamp = Timestamp::from_millisecond(millis).unwrap();
            assert!(handle.add_images(stamp, &images, &BTreeMap::new()));
            let frames = handle.shared.cameras.pop(false).unwrap().unwrap();
            assert_eq!(frames.len(), 2);
            assert_eq!(
                frames[0].time_stamp,
                Timestamp::from_millisecond(millis - 5).unwrap()
            );
            assert!(frames[0].measurement.image.is_some());
            assert!(frames[1].measurement.image.is_none());
        }
        assert!(handle.add_images(Timestamp::UNIX_EPOCH, &BTreeMap::new(), &BTreeMap::new()));
        assert!(handle.shared.cameras.is_empty().unwrap());
        handle.shared.close_sensors();
        assert!(!handle.add_images(Timestamp::UNIX_EPOCH, &images, &BTreeMap::new()));
    }

    #[test]
    fn sensor_inputs_apply_accelerometer_and_depth_units_once_and_reject_wrong_depth_type() {
        let mut handle = sensor_handle();
        let stamp = Timestamp::UNIX_EPOCH;
        assert!(handle.add_imu_measurement(
            stamp,
            &Vector3::new(1.0, 2.0, 3.0),
            &Vector3::new(0.1, 0.2, 0.3)
        ));
        let imu = handle.shared.imu.pop(false).unwrap().unwrap();
        assert_eq!(imu.measurement.accelerometers, Vector3::new(2.0, 6.0, 12.0));
        assert_eq!(imu.measurement.gyroscopes, Vector3::new(0.1, 0.2, 0.3));
        let depth = DynamicMatrix::new_f32(2, 2, 1, vec![1000.0, 2000.0, 0.0, 3000.0]).unwrap();
        handle.add_depth_measurement(stamp, &depth, None);
        let result = handle.shared.depth.pop(false).unwrap().unwrap();
        assert_eq!(result.time_stamp, Timestamp::from_millisecond(-5).unwrap());
        let data = result.measurement.depth_image.unwrap();
        for (&a, b) in data
            .as_matrix_f32()
            .unwrap()
            .data
            .iter()
            .zip([1.0, 2.0, 0.0, 3.0])
        {
            assert!((a - b).abs() < 5e-7);
        }
        let invalid = DynamicMatrix::new_u8(2, 2, 1, vec![1; 4]).unwrap();
        assert!(!handle.add_depth_measurement(stamp, &invalid, None));
        assert!(handle.shared.depth.is_empty().unwrap());
        assert!(handle.shared.check_errors().is_err());
    }
}
