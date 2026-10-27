/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Realtime and full graph orchestration, window policy, loop closure and alignment.
use crate::{
    ceres::{
        component::{Component, MultiFramePtr},
        parameter_block::ParameterBlock,
        pose_error::PoseError,
        pseudo_inverse::PseudoInverse,
        submap_icp_error::OccupancySubmap,
        vi_graph::{
            GraphFactor, Parameter, SolverStage, SolverSummary, SolverUpdate, graph_error,
            io_error, pose_isometry, quaternion_angle_axis, quaternion_angular_distance, seconds,
        },
        vi_graph_estimator::{ObservationConversion, ViGraphEstimator},
    },
    config::parameters::{CameraParameters, GpsParameters, ImuParameters},
    frames::frame_typedefs::{
        KeypointIdentifier, LandmarkId, MapPoint2, MapPoints, SpeedAndBias, StateId,
    },
    kinematics::transformation::{Transformation, delta_q, from_isometry_coefficients},
    sensors::measurements::{GpsMeasurement, GpsMeasurementDeque, ImuMeasurementDeque},
};
use firefly_error::Result;
use jiff::Timestamp;
use nalgebra::{DMatrix, Matrix2, SMatrix, SVector, SymmetricEigen, Vector2, Vector3, Vector4};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::{BufWriter, Write},
    path::Path,
    rc::Rc,
    sync::atomic::{AtomicBool, Ordering},
};
const MIN_DELTA_T: f64 = 2.0;
const NUM_POSE_GRAPH_FRAMES: usize = 12;
#[derive(Clone, Copy, Debug)]
pub struct PlaceRecognitionState {
    pub closed_loop: bool,
    pub is_lost: bool,
    pub is_place_recognition_frame: bool,
}
impl Default for PlaceRecognitionState {
    fn default() -> Self {
        Self {
            closed_loop: false,
            is_lost: false,
            is_place_recognition_frame: true,
        }
    }
}
#[derive(Clone)]
pub struct AuxiliaryState {
    pub is_keyframe: bool,
    pub is_imu_frame: bool,
    pub is_pose_graph_frame: bool,
    pub recognition: PlaceRecognitionState,

    pub loop_id: StateId,
    pub recent_loop_closure_frames: BTreeSet<StateId>,
}
impl Default for AuxiliaryState {
    fn default() -> Self {
        Self {
            is_keyframe: false,
            is_imu_frame: false,
            is_pose_graph_frame: false,
            recognition: crate::ceres::vi_slam_backend::PlaceRecognitionState {
                closed_loop: false,
                is_lost: false,
                is_place_recognition_frame: true,
            },

            loop_id: StateId::default(),
            recent_loop_closure_frames: BTreeSet::new(),
        }
    }
}
#[derive(Default)]
pub struct Component0 {
    pub pose_ids: BTreeSet<StateId>,
    pub reference_id: Option<StateId>,
    pub connected_reference_id: Option<StateId>,
}
pub struct AddStatesBacklog {
    pub timestamp: Timestamp,
    pub id: StateId,
    pub imu_measurements: ImuMeasurementDeque,
}
pub struct AddGpsBacklog {
    pub id: StateId,
    pub measurement: GpsMeasurement,
    pub imu_measurements: ImuMeasurementDeque,
    pub reinit: bool,
}
pub struct AddSubmapAlignmentBacklog {
    pub map: Rc<dyn OccupancySubmap>,
    pub a: StateId,
    pub b: StateId,
    pub points: Vec<Vector3<f32>>,
    pub sensor_error: Vec<f32>,
    pub is_lidar: bool,
    pub robust: String,
}
pub struct RelativePoseInfo {
    pub t_si_sj: Transformation,
    pub information: SMatrix<f64, 6, 6>,
    pub pose_i: StateId,
    pub pose_j: StateId,
}
/// Anti-aliased overlay and image encoding capabilities absent from `PureCV`.
pub trait GraphRaster {
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn draw(&mut self, rows: usize, cols: usize, primitives: &[DrawPrimitive]) -> Result<()>;
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn write_image(&mut self, path: &Path) -> Result<()>;
}
#[derive(Clone)]
pub enum DrawPrimitive {
    Circle {
        center: Vector2<f64>,
        radius: i32,
        bgr: [f64; 3],
        thickness: i32,
    },
    Line {
        a: Vector2<f64>,
        b: Vector2<f64>,
        bgr: [f64; 3],
        thickness: i32,
    },
    Text {
        at: Vector2<f64>,
        text: String,
        scale: f64,
        bgr: [f64; 3],
    },
    Diamond {
        center: Vector2<f64>,
        size: i32,
        bgr: [f64; 3],
        thickness: i32,
    },
}
pub struct FinalBaStatistics {
    pub summary: SolverSummary,
    pub updated_states: BTreeSet<StateId>,
    pub bias_mean: SVector<f64, 6>,
    pub bias_std: SVector<f64, 6>,
    pub error_histograms: Vec<[usize; 12]>,
    pub inlier_counts: Vec<usize>,
    pub outlier_counts: Vec<usize>,
}
/// State updates and removals produced by the window policy.
pub struct StrategyResult {
    /// Retained states whose published data needs updating.
    pub affected_states: BTreeSet<StateId>,
    /// Non-keyframes removed from the realtime graph and multiframe storage.
    pub eliminated_states: BTreeSet<StateId>,
}
pub struct ViSlamBackend {
    pub realtime_graph: ViGraphEstimator,
    pub full_graph: ViGraphEstimator,
    pub multi_frames: BTreeMap<StateId, MultiFramePtr>,
    pub auxiliary_states: BTreeMap<StateId, AuxiliaryState>,
    pub components: Vec<Component0>,
    pub current_component_index: usize,
    pub loop_closure_frames: BTreeSet<StateId>,
    pub imu_frames: BTreeSet<StateId>,
    pub keyframes: BTreeSet<StateId>,
    pub current_loop_closure_frames: BTreeSet<StateId>,
    pub needs_full_graph_optimisation: AtomicBool,
    pub is_loop_closing: AtomicBool,
    pub is_loop_closure_available: AtomicBool,
    pub gps_observability: AtomicBool,
    pub add_states_backlog: Vec<AddStatesBacklog>,
    pub eliminate_states: BTreeMap<StateId, StateId>,
    pub touched_states: BTreeSet<StateId>,
    pub touched_landmarks: BTreeSet<LandmarkId>,
    pub updated_states_loop_closure_attempt: BTreeSet<StateId>,
    pub full_graph_relative_pose_constraints: Vec<RelativePoseInfo>,
    pub last_freeze: Option<StateId>,
    pub keypoint_radius: f64,
    pub num_realtime_pose_graph_frames: usize,
    pub add_gps_backlog: Vec<AddGpsBacklog>,
    pub add_submap_alignment_backlog: Vec<AddSubmapAlignmentBacklog>,
    pub t_ai_s: BTreeMap<StateId, BTreeMap<u64, Transformation>>,
    pub raster: Option<Box<dyn GraphRaster>>,
    pending_full_graph_residuals: Vec<(StateId, StateId)>,
}
impl Default for ViSlamBackend {
    fn default() -> Self {
        Self {
            realtime_graph: ViGraphEstimator::new(),
            full_graph: ViGraphEstimator::new(),
            multi_frames: BTreeMap::new(),
            auxiliary_states: BTreeMap::new(),
            components: vec![Component0::default()],
            current_component_index: 0,
            loop_closure_frames: BTreeSet::new(),
            imu_frames: BTreeSet::new(),
            keyframes: BTreeSet::new(),
            current_loop_closure_frames: BTreeSet::new(),
            needs_full_graph_optimisation: AtomicBool::new(false),
            is_loop_closing: AtomicBool::new(false),
            is_loop_closure_available: AtomicBool::new(false),
            gps_observability: AtomicBool::new(false),
            add_states_backlog: Vec::new(),
            eliminate_states: BTreeMap::new(),
            touched_states: BTreeSet::new(),
            touched_landmarks: BTreeSet::new(),
            updated_states_loop_closure_attempt: BTreeSet::new(),
            full_graph_relative_pose_constraints: Vec::new(),
            last_freeze: None,
            keypoint_radius: 0.09,
            num_realtime_pose_graph_frames: 12,
            add_gps_backlog: Vec::new(),
            add_submap_alignment_backlog: Vec::new(),
            t_ai_s: BTreeMap::new(),
            raster: None,
            pending_full_graph_residuals: Vec::new(),
        }
    }
}
impl ViSlamBackend {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
    pub fn full_graph_busy(&self) -> bool {
        self.is_loop_closing.load(Ordering::Acquire)
            || self.is_loop_closure_available.load(Ordering::Acquire)
    }
    pub fn add_camera(&mut self, p: CameraParameters) -> usize {
        self.full_graph.add_camera(p.clone());
        self.realtime_graph.add_camera(p)
    }
    pub fn add_imu(&mut self, p: ImuParameters) -> i32 {
        self.full_graph.add_imu(p.clone());
        self.realtime_graph.add_imu(p)
    }
    pub fn add_gps(&mut self, p: GpsParameters) -> i32 {
        self.full_graph.add_gps(p.clone());
        self.realtime_graph.add_gps(p)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn multi_frame(&self, id: StateId) -> Result<MultiFramePtr> {
        self.multi_frames
            .get(&id)
            .cloned()
            .ok_or_else(|| graph_error("multiframe absent"))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn current_state_id(&self) -> Result<StateId> {
        self.realtime_graph.current_state_id()
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn timestamp(&self, id: StateId) -> Result<Timestamp> {
        self.realtime_graph.timestamp(id)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn pose(&self, id: StateId) -> Result<Transformation> {
        self.realtime_graph.pose(id)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn speed_and_bias(&self, id: StateId) -> Result<SpeedAndBias> {
        self.realtime_graph.speed_and_bias(id)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn extrinsics(&self, id: StateId, camera: usize) -> Result<Transformation> {
        self.realtime_graph.extrinsics(id, camera)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn get_landmark(&self, id: LandmarkId) -> Result<MapPoint2> {
        self.realtime_graph.get_landmark(id)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn get_landmarks(&self) -> Result<MapPoints> {
        self.realtime_graph.get_landmarks()
    }
    pub fn is_landmark_added(&self, id: LandmarkId) -> bool {
        self.realtime_graph.is_landmark_added(id)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn is_landmark_initialised(&self, id: LandmarkId) -> Result<bool> {
        self.realtime_graph.is_landmark_initialised(id)
    }
    pub fn is_observed(&self, key: KeypointIdentifier) -> bool {
        self.realtime_graph.observations.contains_key(&key)
    }
    pub fn is_keyframe(&self, id: StateId) -> bool {
        self.realtime_graph.is_keyframe(id)
    }
    pub fn is_in_imu_window(&self, id: StateId) -> bool {
        self.auxiliary_states[&id].is_imu_frame
    }
    pub fn is_pose_graph_frame(&self, id: StateId) -> bool {
        self.auxiliary_states[&id].is_pose_graph_frame
    }
    pub fn is_place_recognition_frame(&self, id: StateId) -> bool {
        self.auxiliary_states[&id]
            .recognition
            .is_place_recognition_frame
    }
    pub fn closed_loop(&self, id: StateId) -> bool {
        self.auxiliary_states[&id].recognition.closed_loop
    }
    pub fn recent_loop_closure_frames(&self, id: StateId) -> &BTreeSet<StateId> {
        &self.auxiliary_states[&id].recent_loop_closure_frames
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn state_id_by_age(&self, age: usize) -> Result<StateId> {
        self.realtime_graph.state_id_by_age(age)
    }
    pub fn num_frames(&self) -> usize {
        self.multi_frames.len()
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn t_gw(&self) -> Result<Transformation> {
        self.realtime_graph.t_gw(None)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn gps_measurements(&self, id: StateId) -> Result<Vec<Vector3<f64>>> {
        self.realtime_graph.gps_measurements(id)
    }
    pub fn is_loop_closing(&self) -> bool {
        self.is_loop_closing.load(Ordering::Acquire)
    }
    pub fn is_loop_closure_available(&self) -> bool {
        self.is_loop_closure_available.load(Ordering::Acquire) && !self.is_loop_closing()
    }
    pub fn needs_full_graph_optimisation(&self) -> bool {
        self.needs_full_graph_optimisation.load(Ordering::Acquire)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn is_synched(&self) -> Result<bool> {
        self.full_graph.is_synched(&self.realtime_graph)
    }
    pub fn keyframes(&self) -> &BTreeSet<StateId> {
        &self.keyframes
    }
    pub fn imu_frames(&self) -> &BTreeSet<StateId> {
        &self.imu_frames
    }
    pub fn loop_closure_frames(&self) -> &BTreeSet<StateId> {
        &self.loop_closure_frames
    }
    pub fn is_loop_closure_frame(&self, id: StateId) -> bool {
        self.loop_closure_frames.contains(&id)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn are_landmarks_in_front_of_cameras(&self) -> Result<bool> {
        self.realtime_graph.are_landmarks_in_front_of_cameras()
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn add_states(
        &mut self,
        frame: MultiFramePtr,
        imu: &ImuMeasurementDeque,
        keyframe: bool,
    ) -> Result<bool> {
        let first = self.multi_frames.is_empty();
        let timestamp = frame.borrow().timestamp();
        let id = if first {
            if self.full_graph_busy() {
                return Err(graph_error("cannot initialize while loop closing"));
            }
            let id = self.realtime_graph.add_states_initialise(
                timestamp,
                imu,
                frame.borrow().camera_system(),
            )?;
            self.full_graph.add_states_initialise(
                timestamp,
                imu,
                frame.borrow().camera_system(),
            )?;
            id
        } else {
            let id = self
                .realtime_graph
                .add_states_propagate(timestamp, imu, keyframe)?;
            let num_cameras = frame.borrow().num_frames();
            for camera in 0..num_cameras {
                if self.realtime_graph.camera_parameters[camera]
                    .online_calibration
                    .do_extrinsics
                {
                    let ext = self.extrinsics(id, camera)?;
                    frame.borrow_mut().set_extrinsics(camera, ext);
                }
            }
            if self.full_graph_busy() {
                self.add_states_backlog.push(AddStatesBacklog {
                    timestamp,
                    id,
                    imu_measurements: imu.clone(),
                });
                self.touched_states.insert(id);
            } else {
                self.full_graph
                    .add_states_propagate(timestamp, imu, keyframe)?;
            }
            id
        };
        frame.borrow_mut().set_id(id.value());
        self.multi_frames.insert(id, frame);
        self.auxiliary_states.insert(
            id,
            AuxiliaryState {
                is_keyframe: keyframe,
                is_imu_frame: true,
                loop_id: id,
                ..AuxiliaryState::default()
            },
        );
        self.imu_frames.insert(id);
        Ok(id.is_initialised() && (!first || id.value() == 1))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_pose(&mut self, id: StateId, p: &Transformation) -> Result<bool> {
        if self.full_graph_busy() {
            self.touched_states.insert(id);
        } else {
            self.full_graph.set_pose(id, p)?;
        }
        self.realtime_graph.set_pose(id, p)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_speed_and_bias(&mut self, id: StateId, sb: SpeedAndBias) -> Result<bool> {
        if self.full_graph_busy() {
            self.touched_states.insert(id);
        } else {
            self.full_graph.set_speed_and_bias(id, sb)?;
        }
        self.realtime_graph.set_speed_and_bias(id, sb)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_extrinsics(
        &mut self,
        id: StateId,
        camera: usize,
        p: &Transformation,
    ) -> Result<bool> {
        if self.full_graph_busy() {
            self.touched_states.insert(id);
        } else {
            self.full_graph.set_extrinsics(id, camera, p)?;
        }
        self.realtime_graph.set_extrinsics(id, camera, p)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_keyframe(&mut self, id: StateId, keyframe: bool) -> Result<bool> {
        self.auxiliary_states
            .get_mut(&id)
            .ok_or_else(|| graph_error("auxiliary state absent"))?
            .is_keyframe = keyframe;
        if self.full_graph_busy() {
            self.touched_states.insert(id);
        } else {
            self.full_graph.set_keyframe(id, keyframe)?;
        }
        self.realtime_graph.set_keyframe(id, keyframe)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn add_landmark(
        &mut self,
        id: LandmarkId,
        point: Vector4<f64>,
        initialised: bool,
    ) -> Result<bool> {
        self.realtime_graph.add_landmark(id, point, initialised)?;
        if self.full_graph_busy() {
            self.touched_landmarks.insert(id);
        } else {
            self.full_graph.add_landmark(id, point, initialised)?;
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn add_landmark_auto(
        &mut self,
        point: Vector4<f64>,
        initialised: bool,
    ) -> Result<LandmarkId> {
        let id = self.realtime_graph.add_landmark_auto(point, initialised)?;
        if self.full_graph_busy() {
            self.touched_landmarks.insert(id);
        } else {
            self.full_graph.add_landmark(id, point, initialised)?;
        }
        Ok(id)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_landmark(
        &mut self,
        id: LandmarkId,
        point: Vector4<f64>,
        initialised: bool,
    ) -> Result<bool> {
        self.realtime_graph
            .set_landmark(id, point, Some(initialised))?;
        if self.full_graph_busy() {
            self.touched_landmarks.insert(id);
        } else {
            self.full_graph.set_landmark(id, point, Some(initialised))?;
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_landmark_classification(
        &mut self,
        id: LandmarkId,
        classification: i32,
    ) -> Result<bool> {
        let Some(lm) = self.realtime_graph.landmark_info.get_mut(&id) else {
            return Ok(false);
        };
        lm.classification = classification;
        if self.full_graph_busy() {
            self.touched_landmarks.insert(id);
        } else {
            self.full_graph
                .landmark_info
                .get_mut(&id)
                .ok_or_else(|| graph_error("full graph landmark absent"))?
                .classification = classification;
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn add_observation(
        &mut self,
        landmark: LandmarkId,
        id: StateId,
        camera: usize,
        keypoint: usize,
        cauchy: bool,
    ) -> Result<bool> {
        if self
            .auxiliary_states
            .get(&id)
            .is_some_and(|s| s.is_pose_graph_frame)
        {
            return Err(graph_error("pose graph frame cannot have observations"));
        }
        let frame = self.multi_frame(id)?;
        let frame = frame.borrow();
        let key = KeypointIdentifier::new(id.value(), camera, keypoint);
        let distortion = frame.camera_system().distortion_type(camera);
        self.realtime_graph
            .graph
            .add_observation(&frame, landmark, key, distortion, cauchy)?;
        if self.full_graph_busy() {
            self.touched_states.insert(id);
            self.touched_landmarks.insert(landmark);
        } else {
            self.full_graph
                .graph
                .add_observation(&frame, landmark, key, distortion, cauchy)?;
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn remove_observation(
        &mut self,
        id: StateId,
        camera: usize,
        keypoint: usize,
    ) -> Result<bool> {
        let key = KeypointIdentifier::new(id.value(), camera, keypoint);
        let lm = self
            .realtime_graph
            .observations
            .get(&key)
            .ok_or_else(|| graph_error("observation absent"))?
            .core
            .landmark_id;
        if self.full_graph_busy() {
            self.touched_states.insert(id);
            self.touched_landmarks.insert(lm);
        } else {
            self.full_graph.remove_observation(key)?;
        }
        self.realtime_graph.remove_observation(key)?;
        self.multi_frame(id)?
            .borrow_mut()
            .set_landmark_id(camera, keypoint, 0);
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_observation_information(
        &mut self,
        id: StateId,
        camera: usize,
        keypoint: usize,
        information: Matrix2<f64>,
    ) -> Result<bool> {
        let key = KeypointIdentifier::new(id.value(), camera, keypoint);
        let observation = self
            .realtime_graph
            .observations
            .get(&key)
            .ok_or_else(|| graph_error("observation absent"))?;
        let lm = observation.core.landmark_id;
        observation
            .core
            .error_term
            .borrow_mut()
            .set_information(information);
        if self.full_graph_busy() {
            self.touched_states.insert(id);
            self.touched_landmarks.insert(lm);
        } else {
            self.full_graph
                .observations
                .get(&key)
                .ok_or_else(|| graph_error("full observation absent"))?
                .core
                .error_term
                .borrow_mut()
                .set_information(information);
        }
        Ok(true)
    }
    pub fn set_detector_uniformity_radius(&mut self, radius: f64) {
        self.keypoint_radius = 0.09 * radius / 36.0;
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn tracking_quality(&self, id: StateId) -> Result<f64> {
        let frame = self.multi_frame(id)?;
        let frame = frame.borrow();
        let mut intersection = 0i64;
        let mut union = 0i64;
        let mut match_mask = 0;
        for camera in 0..frame.num_frames() {
            // 深度相机只有深度图。上游对每路相机都取 image，缺失时是空 Mat（0×0），
            // 对交集/并集贡献为零；这里沿用同一语义跳过，与 overlap_fraction 一致。
            let Some(image) = frame.image(camera) else {
                continue;
            };
            let (rows, cols) = (image.rows() / 10, image.cols() / 10);
            let radius = rows.min(cols) as f64 * self.keypoint_radius;
            let mut circles = Vec::new();
            for keypoint in 0..frame.num_keypoints(camera) {
                let lm = LandmarkId::new(frame.landmark_id(camera, keypoint));
                if lm.is_initialised()
                    && self.realtime_graph.landmark_info.get(&lm).is_some_and(|l| {
                        l.observations.iter().any(|key| key.frame_id != id.value())
                    })
                {
                    let mut point = Vector2::zeros();
                    frame.get_keypoint(camera, keypoint, &mut point);
                    circles.push((point * 0.1, radius as i32));
                    match_mask += 1;
                }
            }
            let mask = crate::frontend::pipeline::filled_circle_mask(rows, cols, &circles)?;
            let point_area = (radius * radius * std::f64::consts::PI) as i64;
            intersection += (mask.iter().filter(|&&v| v).count() as i64 - point_area).max(0);
            union += rows as i64 * cols as i64 - point_area;
        }
        Ok(if match_mask < 8 || union <= 0 {
            0.0
        } else {
            intersection as f64 / union as f64
        })
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn overlap_fraction(&self, a: &MultiFramePtr, b: &MultiFramePtr) -> Result<f64> {
        let a = a.borrow();
        let b = b.borrow();
        if a.num_frames() != b.num_frames() {
            return Err(graph_error("frame camera counts differ"));
        }
        let frames = [&*a, &*b];
        let mut landmarks: [BTreeSet<u64>; 2] = [BTreeSet::new(), BTreeSet::new()];
        for (i, frame) in frames.iter().enumerate() {
            for camera in 0..frame.num_frames() {
                if frame.image(camera).is_none() {
                    continue;
                }
                for key in 0..frame.num_keypoints(camera) {
                    let id = frame.landmark_id(camera, key);
                    if id != 0 {
                        landmarks[i].insert(id);
                    }
                }
            }
        }
        let shared: BTreeSet<_> = landmarks[0].intersection(&landmarks[1]).copied().collect();
        if shared.is_empty() {
            return Ok(0.0);
        }
        let mut overlaps = [0.0; 2];
        for (i, frame) in frames.iter().enumerate() {
            let (mut intersection, mut union) = (0usize, 0usize);
            for camera in 0..frame.num_frames() {
                let Some(image) = frame.image(camera) else {
                    continue;
                };
                let (rows, cols) = (image.rows() / 10, image.cols() / 10);
                let radius = rows.min(cols) as f64 * self.keypoint_radius;
                let mut detections = Vec::new();
                let mut matches = Vec::new();
                for key in 0..frame.num_keypoints(camera) {
                    let mut point = Vector2::zeros();
                    frame.get_keypoint(camera, key, &mut point);
                    detections.push((point * 0.1, radius as i32));
                    if shared.contains(&frame.landmark_id(camera, key)) {
                        matches.push((point * 0.1, radius as i32));
                    }
                }
                let detection =
                    crate::frontend::pipeline::filled_circle_mask(rows, cols, &detections)?;
                let match_mask =
                    crate::frontend::pipeline::filled_circle_mask(rows, cols, &matches)?;
                if detection.len() != rows * cols || match_mask.len() != rows * cols {
                    return Err(graph_error("raster mask size differs"));
                }
                for (&d, &m) in detection.iter().zip(&match_mask) {
                    intersection += usize::from(d && m);
                    union += usize::from(d || m);
                }
            }
            overlaps[i] = intersection as f64 / union as f64;
        }
        Ok(overlaps[0].min(overlaps[1]))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn most_overlapped_state_id(
        &self,
        frame: StateId,
        consider_loop: bool,
    ) -> Result<Option<StateId>> {
        let mut all: BTreeSet<_> = self.keyframes.union(&self.imu_frames).copied().collect();
        all.extend(&self.loop_closure_frames);
        if !consider_loop {
            for id in &self.current_loop_closure_frames {
                all.remove(id);
            }
        }
        let mut result = None;
        let mut overlap = 0.0;
        for id in all {
            if id == frame || !self.is_keyframe(id) {
                continue;
            }
            let value = self.overlap_fraction(&self.multi_frame(id)?, &self.multi_frame(frame)?)?;
            if value >= overlap {
                result = Some(id);
                overlap = value;
            }
        }
        Ok(result)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn current_keyframe_state_id(&self, consider_loop: bool) -> Result<Option<StateId>> {
        self.most_overlapped_state_id(self.current_state_id()?, consider_loop)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn current_loopclosure_state_id(&self) -> Result<Option<StateId>> {
        let current = self.current_state_id()?;
        let mut result = None;
        let mut overlap = 0.0;
        for &id in &self.loop_closure_frames {
            if id == current || !self.is_keyframe(id) {
                continue;
            }
            let value =
                self.overlap_fraction(&self.multi_frame(id)?, &self.multi_frame(current)?)?;
            if value >= overlap {
                result = Some(id);
                overlap = value;
            }
        }
        Ok(if overlap > 0.5 { result } else { None })
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn get_observed_ids(&self, id: StateId) -> Result<BTreeSet<StateId>> {
        let state = self
            .realtime_graph
            .states
            .get(&id)
            .ok_or_else(|| graph_error("state absent"))?;
        let mut result: BTreeSet<_> = self.realtime_graph.state_info[&id]
            .two_pose_links
            .keys()
            .copied()
            .collect();
        if !result.is_empty() {
            result.insert(id);
        }
        if self.keyframes.contains(&id)
            || self.imu_frames.contains(&id)
            || self.loop_closure_frames.contains(&id)
        {
            for obs in state.observations.values() {
                result.extend(
                    self.realtime_graph.landmark_info[&obs.landmark_id]
                        .observations
                        .iter()
                        .map(|k| StateId::new(k.frame_id)),
                );
            }
        }
        if let Some((&next, _)) = self
            .realtime_graph
            .states
            .range((std::ops::Bound::Excluded(id), std::ops::Bound::Unbounded))
            .next()
        {
            result.insert(next);
        }
        if let Some((&previous, _)) = self.realtime_graph.states.range(..id).next_back() {
            result.insert(previous);
        }
        Ok(result)
    }
    pub fn is_recent_loop_closure_frame(&self, id: StateId) -> bool {
        self.keyframes
            .union(&self.loop_closure_frames)
            .any(|current| {
                self.auxiliary_states[current]
                    .recent_loop_closure_frames
                    .contains(&id)
            })
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn prune_place_recognition_frames(&mut self) -> Result<usize> {
        self.realtime_graph.compute_covisibilities();
        let Some(&latest) = self.keyframes.last() else {
            return Ok(0);
        };
        let all: BTreeSet<_> = self
            .keyframes
            .union(&self.loop_closure_frames)
            .copied()
            .collect();
        for id in all {
            if latest <= id
                || !self.auxiliary_states[&latest]
                    .recognition
                    .is_place_recognition_frame
                || !self.auxiliary_states[&id]
                    .recognition
                    .is_place_recognition_frame
                || self.realtime_graph.covisibilities(latest, id)? < 10
            {
                continue;
            }
            if self.overlap_fraction(&self.multi_frame(latest)?, &self.multi_frame(id)?)? > 0.6 {
                self.auxiliary_states
                    .get_mut(&latest)
                    .expect("state exists")
                    .recognition
                    .is_place_recognition_frame = false;
                return Ok(1);
            }
        }
        Ok(0)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    #[fastrace::trace]
    pub fn convert_to_pose_graph_mst(
        &mut self,
        convert: &BTreeSet<StateId>,
        consider: &BTreeSet<StateId>,
    ) -> Result<BTreeSet<StateId>> {
        for &id in convert {
            let inverse = self.pose(id)?.inverse();
            let frame = self.multi_frame(id)?;
            let mut frame = frame.borrow_mut();
            for camera in 0..frame.num_frames() {
                for key in 0..frame.num_keypoints(camera) {
                    let lm = LandmarkId::new(frame.landmark_id(camera, key));
                    if lm.is_initialised() && self.is_landmark_added(lm) {
                        let point =
                            inverse.transform_homogeneous(&self.realtime_graph.landmark(lm)?);
                        let initialised = self.is_landmark_initialised(lm)?;
                        frame.set_landmark(camera, key, point, initialised);
                    }
                }
            }
        }
        let output = self
            .realtime_graph
            .convert_to_pose_graph_mst(convert, consider)?;
        let mut affected = BTreeSet::new();
        for edge in &output.created_edges {
            affected.insert(edge.reference_id);
            affected.insert(edge.other_id);
        }
        for &(a, b) in &output.removed_edges {
            affected.insert(a);
            affected.insert(b);
        }
        if self.full_graph_busy() {
            self.touched_states.extend(&affected);
            for key in output.removed_observations {
                self.touched_states.insert(StateId::new(key.frame_id));
                if let Some(obs) = self.full_graph.observations.get(&key)
                    && obs.core.landmark_id.is_initialised()
                {
                    self.touched_landmarks.insert(obs.core.landmark_id);
                }
            }
        } else {
            for key in output.removed_observations {
                self.full_graph.remove_observation(key)?;
            }
            for (a, b) in output.removed_edges {
                self.full_graph.remove_two_pose_const_link(a, b)?;
            }
            for edge in output.created_edges {
                self.full_graph.add_external_two_pose_link(
                    edge.pose_graph_error.borrow().clone_const()?,
                    edge.reference_id,
                    edge.other_id,
                )?;
            }
        }
        Ok(affected)
    }
    fn replicate_observations(
        &mut self,
        id: StateId,
        conversion: &ObservationConversion,
    ) -> Result<()> {
        for &lm in &conversion.created_landmarks {
            if !self.full_graph.landmark_exists(lm) {
                let record = self.realtime_graph.get_landmark(lm)?;
                self.full_graph
                    .add_landmark(lm, record.point, record.is_initialised)?;
                self.full_graph.set_landmark_quality(lm, record.quality)?;
            }
        }
        self.full_graph.remove_two_pose_const_links(id)?;
        for obs in &conversion.observations {
            let lm = LandmarkId::new(obs.h_point.borrow().id());
            self.full_graph.add_external_observation(
                obs.reprojection_error.borrow().as_ref(),
                lm,
                obs.keypoint_identifier,
                obs.loss_function.is_some(),
            )?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn expand_keyframe(&mut self, id: StateId) -> Result<usize> {
        if self.imu_frames.contains(&id)
            || (!self.keyframes.contains(&id) && !self.loop_closure_frames.contains(&id))
            || self.realtime_graph.state_info[&id]
                .two_pose_links
                .is_empty()
        {
            return Err(graph_error("expansion requires frontier keyframe"));
        }
        let output = self.realtime_graph.convert_to_observations(id)?;
        if self.full_graph_busy() {
            self.touched_states.extend(&output.connected_states);
            self.touched_landmarks.extend(&output.created_landmarks);
        } else {
            self.replicate_observations(id, &output)?;
        }
        let mut count = 0;
        for connected in output.connected_states {
            let posegraph = self.auxiliary_states[&connected].is_pose_graph_frame;
            if !self.loop_closure_frames.contains(&connected) && posegraph {
                self.keyframes.insert(connected);
                self.auxiliary_states
                    .get_mut(&connected)
                    .expect("state exists")
                    .is_pose_graph_frame = false;
                count += 1;
            }
            if !self.keyframes.contains(&connected)
                && self.auxiliary_states[&connected].is_pose_graph_frame
            {
                self.loop_closure_frames.insert(connected);
                self.auxiliary_states
                    .get_mut(&connected)
                    .expect("state exists")
                    .is_pose_graph_frame = false;
                count += 1;
            }
        }
        Ok(count)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    #[fastrace::trace]
    pub fn eliminate_imu_frames(&mut self, keep: usize) -> Result<BTreeSet<StateId>> {
        let mut eliminated = BTreeSet::new();
        let ids = self.imu_frames.clone();
        for id in ids {
            if self.imu_frames.len() <= keep {
                break;
            }
            if self.is_keyframe(id) {
                self.imu_frames.remove(&id);
                self.keyframes.insert(id);
                self.components[self.current_component_index]
                    .pose_ids
                    .insert(id);
                self.loop_closure_frames.remove(&id);
                self.auxiliary_states
                    .get_mut(&id)
                    .expect("state exists")
                    .is_imu_frame = false;
            } else {
                let reference = self
                    .most_overlapped_state_id(id, false)?
                    .ok_or_else(|| graph_error("IMU elimination reference absent"))?;
                let observations = self.realtime_graph.states[&id].observations.clone();
                self.realtime_graph.remove_all_observations(id)?;
                self.realtime_graph
                    .eliminate_state_by_imu_merge(id, reference)?;
                if self.loop_closure_frames.contains(&id) {
                    return Err(graph_error("non-keyframe belongs to loop closure set"));
                }
                if self.full_graph_busy() {
                    self.touched_landmarks
                        .extend(observations.values().map(|o| o.landmark_id));
                    self.eliminate_states.insert(id, reference);
                } else {
                    self.full_graph.remove_all_observations(id)?;
                    self.full_graph
                        .eliminate_state_by_imu_merge(id, reference)?;
                }
                self.imu_frames.remove(&id);
                self.multi_frames.remove(&id);
                eliminated.insert(id);
                self.auxiliary_states.remove(&id);
            }
        }
        Ok(eliminated)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    #[fastrace::trace]
    pub fn apply_strategy(
        &mut self,
        num_keyframes: usize,
        num_loop_frames: usize,
        num_imu_frames: usize,
        expand: bool,
    ) -> Result<StrategyResult> {
        if self.imu_frames.len() > 1 && !self.keyframes.is_empty() {
            let id = *self
                .auxiliary_states
                .keys()
                .next_back()
                .expect("IMU states exist");
            let quality = self.tracking_quality(id)?;
            if quality < 0.01 {
                log::warn!("tracking quality weak: {quality}");
            }
        }
        let eliminated_states = self.eliminate_imu_frames(num_imu_frames)?;
        let mut affected = BTreeSet::new();
        if self.current_keyframe_state_id(true)?.is_none() {
            return Ok(StrategyResult {
                affected_states: affected,
                eliminated_states,
            });
        }
        let mut eliminated = false;
        let mut converted = 0;
        while self.keyframes.len() > num_keyframes {
            self.realtime_graph.compute_covisibilities();
            let current = self.current_state_id()?;
            let keyframe = self
                .current_keyframe_state_id(true)?
                .ok_or_else(|| graph_error("current keyframe absent"))?;
            let mut selected = None;
            let mut minimum = 100_000;
            for &id in &self.keyframes {
                let count = self
                    .realtime_graph
                    .covisibilities(current, id)?
                    .max(self.realtime_graph.covisibilities(keyframe, id)?);
                if Some(&id) == self.keyframes.first() && count >= 2 {
                    continue;
                }
                if count < minimum {
                    selected = Some(id);
                    minimum = count;
                }
            }
            let id = selected.ok_or_else(|| graph_error("no keyframe can be removed"))?;
            let (consider, max_count) = self.frames_for_conversion(id)?;
            self.auxiliary_states
                .get_mut(&id)
                .expect("state exists")
                .is_pose_graph_frame = true;
            self.keyframes.remove(&id);
            let recent = self.keyframes.clone();
            self.auxiliary_states
                .get_mut(&id)
                .expect("state exists")
                .recent_loop_closure_frames
                .extend(recent);
            eliminated = true;
            if max_count == 0 {
                self.drop_state_observations(id, &mut affected)?;
                continue;
            }
            affected.extend(self.convert_to_pose_graph_mst(&BTreeSet::from([id]), &consider)?);
            converted += 1;
            if !self.realtime_graph.states[&id].observations.is_empty() {
                return Err(graph_error("converted frame retains observations"));
            }
            if converted >= 3 {
                break;
            }
        }
        self.freeze_eliminated_window(eliminated, num_keyframes, num_imu_frames)?;
        let loop_converted = self.convert_loop_frames(num_loop_frames, &mut affected)?;
        if expand && loop_converted < 3 && converted < 3 {
            if let Some(id) = self.current_keyframe_state_id(true)?
                && !self.realtime_graph.state_info[&id]
                    .two_pose_links
                    .is_empty()
            {
                self.expand_keyframe(id)?;
            }
            if let Some(id) = self.current_loopclosure_state_id()?
                && !self.realtime_graph.state_info[&id]
                    .two_pose_links
                    .is_empty()
            {
                self.expand_keyframe(id)?;
            }
        }
        Ok(StrategyResult {
            affected_states: affected,
            eliminated_states,
        })
    }
    fn convert_loop_frames(
        &mut self,
        num_loop_frames: usize,
        affected: &mut BTreeSet<StateId>,
    ) -> Result<usize> {
        let mut loop_converted = 0;
        loop {
            if self.loop_closure_frames.is_empty() {
                break;
            }
            self.realtime_graph.compute_covisibilities();
            let current = self.current_state_id()?;
            let keyframe = self
                .current_keyframe_state_id(true)?
                .ok_or_else(|| graph_error("current keyframe absent"))?;
            let mut selected = None;
            let mut minimum = 100_000;
            for &id in &self.loop_closure_frames {
                let count = self
                    .realtime_graph
                    .covisibilities(current, id)?
                    .max(self.realtime_graph.covisibilities(keyframe, id)?);
                if count < minimum {
                    minimum = count;
                    selected = Some(id);
                }
            }
            if minimum != 0 && self.loop_closure_frames.len() <= num_loop_frames {
                break;
            }
            let id = selected.ok_or_else(|| graph_error("loop frame selection failed"))?;
            let (consider, max_count) = self.frames_for_conversion(id)?;
            self.auxiliary_states
                .get_mut(&id)
                .expect("state exists")
                .is_pose_graph_frame = true;
            self.loop_closure_frames.remove(&id);
            if max_count == 0 {
                self.drop_state_observations(id, affected)?;
            } else {
                affected.extend(self.convert_to_pose_graph_mst(&BTreeSet::from([id]), &consider)?);
                loop_converted += 1;
                if loop_converted >= 3 {
                    break;
                }
            }
            if self.loop_closure_frames.len() <= num_loop_frames {
                break;
            }
        }
        Ok(loop_converted)
    }

    fn freeze_eliminated_window(
        &mut self,
        eliminated: bool,
        num_keyframes: usize,
        num_imu_frames: usize,
    ) -> Result<()> {
        if eliminated {
            let newest = self.current_state_id()?;
            let time = self.timestamp(newest)?;
            let ids: Vec<_> = self.auxiliary_states.keys().rev().copied().collect();
            let offset = num_keyframes + num_imu_frames + self.num_realtime_pose_graph_frames;
            if let Some(&candidate) = ids.get(offset) {
                let eligible: Vec<_> = self
                    .realtime_graph
                    .states
                    .range(..=candidate)
                    .rev()
                    .map(|(&id, _)| id)
                    .collect();
                let mut freeze = None;
                for id in eligible {
                    freeze = Some(id);
                    if seconds(time, self.timestamp(id)?) >= MIN_DELTA_T {
                        break;
                    }
                }
                if let Some(freeze) = freeze {
                    let first = *self
                        .realtime_graph
                        .states
                        .keys()
                        .next()
                        .expect("states exist");
                    if freeze != first {
                        let freeze = self.last_freeze.map_or(freeze, |old| old.max(freeze));
                        self.last_freeze = Some(freeze);
                        if freeze != first {
                            self.realtime_graph.freeze_poses_until(freeze, false)?;
                        }
                        self.realtime_graph
                            .freeze_speed_and_biases_until(freeze, false)?;
                    }
                }
            }
        }
        Ok(())
    }

    fn drop_state_observations(
        &mut self,
        id: StateId,
        affected: &mut BTreeSet<StateId>,
    ) -> Result<()> {
        self.realtime_graph.remove_all_observations(id)?;
        if self.full_graph_busy() {
            self.touched_states.insert(id);
            affected.insert(id);
        } else {
            self.full_graph.remove_all_observations(id)?;
        }
        Ok(())
    }
    fn frames_for_conversion(&self, id: StateId) -> Result<(BTreeSet<StateId>, i32)> {
        let mut frames = BTreeSet::new();
        let mut maximum = 0;
        let mut best = None;
        for &other in self.keyframes.union(&self.loop_closure_frames) {
            let count = self.realtime_graph.covisibilities(id, other)?;
            if count >= maximum {
                maximum = count;
                best = Some(other);
            }
            if !self.realtime_graph.state_info[&other]
                .two_pose_links
                .is_empty()
            {
                frames.insert(other);
            }
        }
        frames.insert(id);
        if let Some(other) = best {
            frames.insert(other);
        }
        Ok((frames, maximum))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    #[fastrace::trace]
    pub fn optimise_realtime_graph(
        &mut self,
        iterations: usize,
        threads: usize,
        verbose: bool,
        only_newest: bool,
        initialised: bool,
    ) -> Result<Vec<StateId>> {
        let newest = self.current_state_id()?;
        let fixation = self.initial_pose_fixation(newest, initialised)?;
        let (unfreeze, frozen) = self.freeze_newest_window(only_newest, newest)?;
        self.realtime_graph.options.linear_solver = "DENSE_SCHUR";
        let solve = {
            let _phase = fastrace::local::LocalSpan::enter_with_local_parent(if only_newest {
                "newest-state-solve"
            } else {
                "window-solve"
            });
            self.realtime_graph.optimise(iterations, threads, verbose)
        };
        if only_newest {
            return self.restore_newest_solve(newest, (unfreeze, frozen), fixation, solve);
        }
        if let Err(error) = solve {
            if let Some(rid) = fixation {
                self.realtime_graph.problem.residuals.remove(&rid);
            }
            return Err(error);
        }
        self.realtime_graph.update_landmarks()?;
        let _sync_span =
            fastrace::local::LocalSpan::enter_with_local_parent("realtime-full-graph-sync");
        let mut updated: Vec<_> = self
            .updated_states_loop_closure_attempt
            .iter()
            .copied()
            .collect();
        let ids: Vec<_> = self.realtime_graph.states.keys().rev().copied().collect();
        for id in &ids {
            if self.realtime_graph.pose_block(*id)?.borrow().fixed()
                && self.realtime_graph.speed_bias_block(*id)?.borrow().fixed()
            {
                break;
            }
            if !self.updated_states_loop_closure_attempt.contains(id) {
                updated.push(*id);
            }
            if !self.full_graph_busy() {
                self.full_graph.set_pose(*id, &self.pose(*id)?)?;
                self.full_graph
                    .set_speed_and_bias(*id, self.speed_and_bias(*id)?)?;
                let global = self.realtime_graph.t_gw(Some(*id))?;
                self.full_graph.state_info[id]
                    .t_gw
                    .as_ref()
                    .ok_or_else(|| graph_error("GPS transform absent"))?
                    .borrow_mut()
                    .set_estimate(
                        crate::kinematics::transformation::TransformationCacheless::from_other(
                            &global,
                        ),
                    );
            }
        }
        if !self.gps_observability.load(Ordering::Acquire) {
            if !self.full_graph_busy() {
                self.full_graph
                    .set_gps_extrinsics(&self.realtime_graph.t_gw(None)?)?;
            }
        } else if !self.realtime_graph.gps_flags.gps_fixed && !self.full_graph_busy() {
            self.realtime_graph.freeze_gps_extrinsics()?;
            self.full_graph
                .set_gps_extrinsics(&self.realtime_graph.t_gw(None)?)?;
            self.full_graph.freeze_gps_extrinsics()?;
        }
        self.updated_states_loop_closure_attempt.clear();
        if !self.full_graph_busy() {
            for (&id, lm) in &self.realtime_graph.landmarks {
                self.full_graph.set_landmark(
                    id,
                    *lm.h_point
                        .borrow()
                        .estimate()
                        .ok_or_else(|| graph_error("point absent"))?,
                    Some(lm.h_point.borrow().initialized()),
                )?;
                self.full_graph.set_landmark_quality(id, lm.quality)?;
            }
            for id in ids {
                sync_imu(&self.realtime_graph, &self.full_graph, id)?;
                if self.realtime_graph.pose_block(id)?.borrow().fixed() {
                    break;
                }
            }
        }
        if let Some(rid) = fixation {
            self.realtime_graph.problem.residuals.remove(&rid);
        }
        Ok(updated)
    }
    fn freeze_newest_window(
        &mut self,
        only_newest: bool,
        newest: StateId,
    ) -> Result<(Option<StateId>, bool)> {
        let mut unfreeze = None;
        let mut frozen = false;
        if only_newest {
            for &id in self.realtime_graph.states.keys().rev() {
                if self.realtime_graph.pose_block(id)?.borrow().fixed() {
                    break;
                }
                unfreeze = Some(id);
            }
            if let Ok(previous) = self.realtime_graph.state_id_by_age(1) {
                self.realtime_graph.freeze_poses_until(previous, false)?;
                self.realtime_graph
                    .freeze_speed_and_biases_until(previous, false)?;
                frozen = true;
            }
            for landmark in self.realtime_graph.landmarks.values() {
                landmark.h_point.borrow_mut().set_fixed(true);
            }
            for camera in 0..self.realtime_graph.camera_parameters.len() {
                if self.realtime_graph.camera_parameters[camera]
                    .online_calibration
                    .do_extrinsics
                {
                    self.realtime_graph
                        .extrinsics_block(newest, camera)?
                        .borrow_mut()
                        .set_fixed(true);
                }
            }
        }
        Ok((unfreeze, frozen))
    }

    fn restore_newest_solve(
        &mut self,
        newest: StateId,
        (unfreeze, frozen): (Option<StateId>, bool),
        fixation: Option<crate::ceres::vi_graph::ResidualId>,
        solve: Result<()>,
    ) -> Result<Vec<StateId>> {
        for camera in 0..self.realtime_graph.camera_parameters.len() {
            if self.realtime_graph.camera_parameters[camera]
                .online_calibration
                .do_extrinsics
            {
                self.realtime_graph
                    .extrinsics_block(newest, camera)?
                    .borrow_mut()
                    .set_fixed(false);
            }
        }
        if frozen {
            let id = unfreeze.ok_or_else(|| graph_error("unfreeze state absent"))?;
            self.realtime_graph.unfreeze_poses_from(id)?;
            self.realtime_graph.unfreeze_speed_and_biases_from(id)?;
        }
        for landmark in self.realtime_graph.landmarks.values() {
            landmark.h_point.borrow_mut().set_fixed(false);
        }
        if let Some(rid) = fixation {
            self.realtime_graph.problem.residuals.remove(&rid);
        }
        solve?;
        if !self.full_graph_busy() {
            self.full_graph.set_pose(newest, &self.pose(newest)?)?;
            self.full_graph
                .set_speed_and_bias(newest, self.speed_and_bias(newest)?)?;
        }
        Ok(Vec::new())
    }
    fn initial_pose_fixation(
        &mut self,
        newest: StateId,
        initialised: bool,
    ) -> Result<Option<crate::ceres::vi_graph::ResidualId>> {
        let mut fixation = None;
        if !initialised {
            let first = self.realtime_graph.pose(StateId::new(1))?;
            let pose = self.pose(newest)?;
            let parameter = Parameter::Pose(self.realtime_graph.pose_block(newest)?);
            let information =
                SVector::<f64, 6>::from_row_slice(&[1.0e8, 1.0e8, 1.0e8, 0.0, 0.0, 0.0]);
            fixation = Some(self.realtime_graph.problem.add(
                GraphFactor::PosePrior(Box::new(PoseError::from_information_diagonal(
                    pose_isometry(&first),
                    information,
                ))),
                vec![parameter],
                None,
            ));
            self.realtime_graph.set_pose(
                newest,
                &Transformation::new(first.translation(), pose.quaternion()),
            )?;
        }
        Ok(fixation)
    }

    pub fn set_optimisation_time_limit(&mut self, limit: f64, min_iterations: usize) -> bool {
        self.realtime_graph
            .set_optimisation_time_limit(limit, min_iterations)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn clean_unobserved_landmarks(&mut self) -> Result<usize> {
        let removed = self.realtime_graph.clean_unobserved_landmarks()?;
        for keys in removed.values() {
            for key in keys {
                self.multi_frame(StateId::new(key.frame_id))?
                    .borrow_mut()
                    .set_landmark_id(key.camera_index, key.keypoint_index, 0);
            }
        }
        if self.full_graph_busy() {
            for (&lm, keys) in &removed {
                self.touched_landmarks.insert(lm);
                self.touched_states
                    .extend(keys.iter().map(|k| StateId::new(k.frame_id)));
            }
        } else {
            let other = self.full_graph.clean_unobserved_landmarks()?;
            if other.len() != removed.len() {
                return Err(graph_error("inconsistent landmark cleanup"));
            }
        }
        Ok(removed.len())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn merge_landmark(&mut self, from: LandmarkId, into: LandmarkId) -> Result<bool> {
        let mut success = self
            .realtime_graph
            .merge_landmark(from, into, &mut self.multi_frames)?;
        let target = self
            .realtime_graph
            .landmark_info
            .get(&into)
            .ok_or_else(|| graph_error("landmark absent"))?;
        let keys = target.observations.clone();
        for key in &keys {
            self.multi_frame(StateId::new(key.frame_id))?
                .borrow_mut()
                .set_landmark_id(key.camera_index, key.keypoint_index, into.value());
        }
        if self.full_graph_busy() {
            self.touched_states
                .extend(keys.iter().map(|k| StateId::new(k.frame_id)));
            self.touched_landmarks.insert(from);
            self.touched_landmarks.insert(into);
        } else {
            success &= self
                .full_graph
                .merge_landmark(from, into, &mut self.multi_frames)?;
        }
        Ok(success)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn merge_landmarks(&mut self, from: &[LandmarkId], into: &[LandmarkId]) -> Result<usize> {
        if from.len() != into.len() {
            return Err(graph_error("landmark merge vectors differ"));
        }
        if self.full_graph_busy() {
            return Err(graph_error("cannot merge while loop closing"));
        }
        let mut changes = BTreeMap::new();
        let mut count = 0;
        for (&from, &into) in from.iter().zip(into) {
            let (mut from, mut into) = (from, into);
            while let Some(&next) = changes.get(&from) {
                from = next;
            }
            while let Some(&next) = changes.get(&into) {
                into = next;
            }
            if from == into {
                continue;
            }
            if self
                .realtime_graph
                .merge_landmark(from, into, &mut self.multi_frames)?
            {
                count += 1;
            }
            self.full_graph
                .merge_landmark(from, into, &mut self.multi_frames)?;
            changes.insert(from, into);
            let target = self
                .realtime_graph
                .landmark_info
                .get(&into)
                .ok_or_else(|| graph_error("landmark absent"))?;
            let keys = target.observations.clone();
            for key in keys {
                self.multi_frame(StateId::new(key.frame_id))?
                    .borrow_mut()
                    .set_landmark_id(key.camera_index, key.keypoint_index, into.value());
            }
        }
        Ok(count)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn optimise_full_graph(
        &mut self,
        iterations: usize,
        threads: usize,
        verbose: bool,
    ) -> Result<SolverSummary> {
        if self.is_loop_closing.swap(true, Ordering::AcqRel) {
            return Err(graph_error("full graph already optimizing"));
        }
        self.needs_full_graph_optimisation
            .store(false, Ordering::Release);
        let result = (|| {
            let constraints = std::mem::take(&mut self.full_graph_relative_pose_constraints);
            if !constraints.is_empty() {
                for item in &constraints {
                    self.full_graph.add_relative_pose_constraint(
                        item.pose_i,
                        item.pose_j,
                        &item.t_si_sj,
                        item.information * 100.0,
                    )?;
                }
                self.full_graph.options.function_tolerance = 0.001;
                let solve = self.full_graph.optimise(iterations / 3, threads, verbose);
                for item in &constraints {
                    self.full_graph
                        .remove_relative_pose_constraint(item.pose_i, item.pose_j)?;
                }
                solve?;
            }
            self.full_graph.options.function_tolerance = 1.0e-6;
            self.full_graph.optimise(iterations, threads, verbose)?;
            self.full_graph.update_landmarks()?;
            self.is_loop_closure_available
                .store(true, Ordering::Release);
            Ok(self.full_graph.summary.clone())
        })();
        self.is_loop_closing.store(false, Ordering::Release);
        result
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn add_loop_closure_frame(
        &mut self,
        id: StateId,
        skip_full_optimisation: bool,
    ) -> Result<BTreeSet<LandmarkId>> {
        if self.full_graph_busy() {
            return Err(graph_error("cannot add loop closure while loop closing"));
        }
        if self.loop_closure_frames.contains(&id) || self.keyframes.contains(&id) {
            return Ok(BTreeSet::new());
        }
        if !self
            .auxiliary_states
            .get(&id)
            .is_some_and(|s| s.is_pose_graph_frame)
        {
            return Err(graph_error("loop closure frame must be pose graph frame"));
        }
        self.loop_closure_frames.insert(id);
        self.current_loop_closure_frames.insert(id);
        let conversion = self.realtime_graph.convert_to_observations(id)?;
        self.replicate_observations(id, &conversion)?;
        for &connected in &conversion.connected_states {
            if self.auxiliary_states[&connected].is_pose_graph_frame
                && !self.keyframes.contains(&connected)
            {
                self.loop_closure_frames.insert(connected);
                self.current_loop_closure_frames.insert(connected);
                self.auxiliary_states
                    .get_mut(&connected)
                    .expect("state exists")
                    .is_pose_graph_frame = false;
            }
        }
        self.auxiliary_states
            .get_mut(&id)
            .expect("state exists")
            .is_pose_graph_frame = false;
        let first_loop = self.auxiliary_states[&id].loop_id;
        for (_, state) in self.auxiliary_states.range_mut(first_loop..) {
            state.loop_id = first_loop;
        }
        let freeze = self.full_freeze_boundary(
            first_loop,
            self.full_graph.timestamp(id)?,
            self.last_freeze,
        )?;
        self.unfreeze_full_from_boundary(freeze)?;
        let recent = self.loop_closure_frames.clone();
        for current in self.keyframes.union(&self.loop_closure_frames) {
            self.auxiliary_states
                .get_mut(current)
                .expect("state exists")
                .recent_loop_closure_frames
                .extend(&recent);
        }
        if !skip_full_optimisation {
            self.needs_full_graph_optimisation
                .store(true, Ordering::Release);
        }
        Ok(conversion.created_landmarks)
    }
    fn full_freeze_boundary(
        &self,
        id: StateId,
        time: Timestamp,
        last_freeze: Option<StateId>,
    ) -> Result<StateId> {
        let ids: Vec<_> = self
            .full_graph
            .states
            .range(..=id)
            .rev()
            .map(|(&id, _)| id)
            .collect();
        let mut index = NUM_POSE_GRAPH_FRAMES.min(ids.len().saturating_sub(1));
        while index + 1 < ids.len()
            && seconds(time, self.full_graph.timestamp(ids[index])?) < MIN_DELTA_T
        {
            index += 1;
        }
        if let Some(last) = last_freeze {
            while index + 1 < ids.len() && ids[index] > last {
                index += 1;
            }
        }
        ids.get(index)
            .copied()
            .ok_or_else(|| graph_error("freeze boundary absent"))
    }
    fn unfreeze_full_from_boundary(&mut self, id: StateId) -> Result<()> {
        let first = *self
            .full_graph
            .states
            .keys()
            .next()
            .ok_or_else(|| graph_error("full graph empty"))?;
        self.full_graph.unfreeze_poses_from(id)?;
        if id != first {
            self.full_graph.freeze_poses_until(id, false)?;
        }
        self.full_graph.unfreeze_speed_and_biases_from(id)?;
        if id != first {
            self.full_graph.freeze_speed_and_biases_until(id, false)?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn add_gps_alignment_frame(&mut self, id: StateId) -> Result<()> {
        if self.full_graph_busy() {
            return Err(graph_error("cannot add GPS alignment while loop closing"));
        }
        let freeze = self.full_freeze_boundary(id, self.full_graph.timestamp(id)?, None)?;
        self.unfreeze_full_from_boundary(freeze)?;
        self.needs_full_graph_optimisation
            .store(true, Ordering::Release);
        Ok(())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn add_gps_measurements_on_all_graphs(
        &mut self,
        input: &mut GpsMeasurementDeque,
        imu: &ImuMeasurementDeque,
    ) -> Result<bool> {
        if self.realtime_graph.gps_parameters.is_empty() {
            return Ok(false);
        }
        let mut gps = if self
            .realtime_graph
            .gps_parameters
            .last()
            .expect("parameters exist")
            .robust_gps_init
        {
            self.realtime_graph.check_valid_gps_measurements(input)?
        } else {
            input.clone()
        };
        if gps.is_empty() {
            return Ok(false);
        }
        let reinit = self.realtime_graph.needs_gps_reinit()?;
        if reinit {
            self.realtime_graph.reinit_gps_extrinsics();
        }
        self.full_graph.gps_status = self.realtime_graph.gps_status;
        if self.full_graph_busy() {
            let ids = self.realtime_graph.add_gps_measurements(&mut gps, imu)?;
            if ids.len() != gps.len() {
                return Err(graph_error("buffered GPS state count differs"));
            }
            for (id, measurement) in ids.into_iter().zip(&gps) {
                self.add_gps_backlog.push(AddGpsBacklog {
                    id,
                    measurement: measurement.clone(),
                    imu_measurements: imu.clone(),
                    reinit,
                });
            }
        } else {
            self.realtime_graph.add_gps_measurements(&mut gps, imu)?;
            self.full_graph.add_gps_measurements(&mut gps, imu)?;
        }
        if let Some(global) = self.realtime_graph.initialization_strategy()? {
            self.realtime_graph.add_gps_init_factors()?;
            self.full_graph.add_gps_init_factors()?;
            self.realtime_graph.set_gps_extrinsics(&global)?;
            self.full_graph.set_gps_extrinsics(&global)?;
        }
        if !self.full_graph_busy() {
            self.try_gps_alignment()?;
        }
        if !self.gps_observability.load(Ordering::Acquire) {
            self.gps_observability.store(
                self.realtime_graph.gps_flags.gps_observability,
                Ordering::Release,
            );
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn try_gps_alignment(&mut self) -> Result<bool> {
        if self
            .realtime_graph
            .alignment_requests
            .needs_initial_alignment
        {
            self.full_graph
                .set_gps_extrinsics(&self.realtime_graph.t_gw(None)?)?;
            self.add_gps_alignment_frame(StateId::new(1))?;
            self.realtime_graph.reset_initial_gps_alignment();
            self.full_graph.reset_initial_gps_alignment();
            return Ok(true);
        }
        let _ = self.full_graph.needs_full_gps_alignment()?;
        if let Some((loss, align, new_global)) = self.realtime_graph.needs_full_gps_alignment()? {
            self.attempt_full_gps_alignment(loss, align, &new_global)?;
            self.optimise_realtime_graph(50, 1, false, false, true)?;
            self.add_gps_alignment_frame(loss)?;
            self.realtime_graph.reset_full_gps_alignment();
            self.full_graph.reset_full_gps_alignment();
            return Ok(true);
        }
        if let Some((loss, align, error)) = self.realtime_graph.needs_pos_gps_alignment()? {
            self.attempt_pos_gps_alignment(loss, align, error)?;
            self.add_gps_alignment_frame(loss)?;
            self.realtime_graph.reset_pos_gps_alignment();
            self.full_graph.reset_pos_gps_alignment();
            return Ok(true);
        }
        Ok(false)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn synchronise_realtime_and_full_graph(&mut self) -> Result<Vec<StateId>> {
        if self.is_loop_closing.load(Ordering::Acquire) {
            return Err(graph_error("cannot synchronize while loop closing"));
        }
        self.keyframes.append(&mut self.loop_closure_frames);
        let removed: Vec<_> = self
            .full_graph
            .landmarks
            .keys()
            .filter(|id| !self.realtime_graph.landmarks.contains_key(id))
            .copied()
            .collect();
        for id in removed {
            self.full_graph.remove_landmark(id)?;
        }
        let old_id = self
            .full_graph
            .states
            .keys()
            .rev()
            .find(|id| self.realtime_graph.states.contains_key(id))
            .copied()
            .ok_or_else(|| graph_error("graphs have no common state"))?;
        let change = self
            .full_graph
            .pose(old_id)?
            .compose(&self.realtime_graph.pose(old_id)?.inverse());
        self.replay_state_backlog(old_id, &change)?;
        for (id, reference) in std::mem::take(&mut self.eliminate_states) {
            self.full_graph.remove_all_observations(id)?;
            self.full_graph
                .eliminate_state_by_imu_merge(id, reference)?;
        }
        let full_ids: Vec<_> = self.full_graph.states.keys().rev().copied().collect();
        for &id in &full_ids {
            let both_fixed = self.full_graph.pose_block(id)?.borrow().fixed()
                && self.full_graph.speed_bias_block(id)?.borrow().fixed()
                && self.realtime_graph.pose_block(id)?.borrow().fixed()
                && self.realtime_graph.speed_bias_block(id)?.borrow().fixed();
            if both_fixed {
                break;
            }
            sync_imu(&self.full_graph, &self.realtime_graph, id)?;
        }
        self.reconcile_map_and_sensor_backlogs(&change)?;
        let updated = self.copy_full_graph_estimates(full_ids)?;
        self.reconcile_touched_factors()?;
        self.touched_states.clear();
        self.touched_landmarks.clear();
        if self.full_graph.landmarks.len() != self.realtime_graph.landmarks.len()
            || self.full_graph.states.len() != self.realtime_graph.states.len()
        {
            return Err(graph_error("synchronized graph counts differ"));
        }
        self.current_loop_closure_frames.clear();
        self.is_loop_closure_available
            .store(false, Ordering::Release);
        Ok(updated)
    }
    fn reconcile_touched_factors(&mut self) -> Result<()> {
        for &id in &self.touched_landmarks {
            if !self.full_graph.landmark_exists(id) {
                continue;
            }
            let keys = self.full_graph.landmark_info[&id].observations.clone();
            for key in keys {
                self.full_graph.remove_observation(key)?;
            }
        }
        for &id in &self.touched_landmarks {
            if !self.full_graph.landmark_exists(id) {
                continue;
            }
            for &key in &self.realtime_graph.landmark_info[&id].observations {
                let obs = &self.realtime_graph.observations[&key];
                self.full_graph.add_external_observation(
                    obs.core.error_term.borrow().as_ref(),
                    id,
                    key,
                    true,
                )?;
            }
        }
        for &id in &self.touched_states {
            if !self.full_graph.states.contains_key(&id) {
                continue;
            }
            let relative = self.full_graph.state_info[&id].relative_pose_links.clone();
            for &other in relative.keys() {
                self.full_graph.remove_relative_pose_constraint(id, other)?;
            }
            let visual = self.full_graph.state_info[&id].two_pose_const_links.clone();
            for &other in visual.keys() {
                self.full_graph.remove_two_pose_const_link(id, other)?;
            }
        }
        let (mut relative_added, mut visual_added) = (BTreeSet::new(), BTreeSet::new());
        for &id in &self.touched_states {
            if !self.full_graph.states.contains_key(&id) {
                continue;
            }
            for (&other, &rid) in &self.realtime_graph.state_info[&id].relative_pose_links {
                if relative_added.insert(rid) {
                    let GraphFactor::RelativePose(f) =
                        self.realtime_graph.problem.residuals[&rid].factor.as_ref()
                    else {
                        return Err(graph_error("relative factor invalid"));
                    };
                    let endpoints = &self.realtime_graph.problem.residuals[&rid].parameters;
                    let a = parameter_state_id(&endpoints[0])?;
                    let b = parameter_state_id(&endpoints[1])?;
                    let _ = other;
                    self.full_graph.add_relative_pose_constraint(
                        a,
                        b,
                        &from_isometry_coefficients(f.t_ab()),
                        *f.information(),
                    )?;
                }
            }
            for (&other, &rid) in &self.realtime_graph.state_info[&id].two_pose_links {
                if visual_added.insert(rid) {
                    let GraphFactor::Visual(f) =
                        self.realtime_graph.problem.residuals[&rid].factor.as_ref()
                    else {
                        return Err(graph_error("visual factor invalid"));
                    };
                    self.full_graph.add_external_two_pose_link(
                        f.borrow().clone_const()?,
                        id.min(other),
                        id.max(other),
                    )?;
                }
            }
        }
        Ok(())
    }

    fn copy_full_graph_estimates(&mut self, full_ids: Vec<StateId>) -> Result<Vec<StateId>> {
        let mut updated = Vec::new();
        for id in full_ids {
            if self.full_graph.pose_block(id)?.borrow().fixed()
                && self.full_graph.speed_bias_block(id)?.borrow().fixed()
                && self.realtime_graph.pose_block(id)?.borrow().fixed()
                && self.realtime_graph.speed_bias_block(id)?.borrow().fixed()
            {
                if self.full_graph.pose(id)?.matrix() != self.realtime_graph.pose(id)?.matrix()
                    || self.full_graph.speed_and_bias(id)?
                        != self.realtime_graph.speed_and_bias(id)?
                {
                    return Err(graph_error("fixed graph states disagree"));
                }
                break;
            }
            updated.push(id);
            self.realtime_graph
                .set_pose(id, &self.full_graph.pose(id)?)?;
            self.realtime_graph
                .set_speed_and_bias(id, self.full_graph.speed_and_bias(id)?)?;
            for camera in 0..self.full_graph.states[&id].extrinsics.len() {
                if !self
                    .full_graph
                    .extrinsics_block(id, camera)?
                    .borrow()
                    .fixed()
                {
                    self.realtime_graph.set_extrinsics(
                        id,
                        camera,
                        &self.full_graph.extrinsics(id, camera)?,
                    )?;
                }
            }
        }
        for (&id, lm) in &self.full_graph.landmarks {
            self.realtime_graph.set_landmark(
                id,
                *lm.h_point
                    .borrow()
                    .estimate()
                    .ok_or_else(|| graph_error("full point absent"))?,
                Some(lm.h_point.borrow().initialized()),
            )?;
            self.realtime_graph.set_landmark_quality(id, lm.quality)?;
        }
        if self.full_graph.landmarks.len() != self.realtime_graph.landmarks.len() {
            return Err(graph_error("inconsistent landmark counts"));
        }
        Ok(updated)
    }

    fn reconcile_map_and_sensor_backlogs(&mut self, change: &Transformation) -> Result<()> {
        let new_landmarks: Vec<_> = self
            .realtime_graph
            .landmarks
            .keys()
            .filter(|id| !self.full_graph.landmarks.contains_key(id))
            .copied()
            .collect();
        for id in new_landmarks {
            let record = self.realtime_graph.get_landmark(id)?;
            let point = change.transform_homogeneous(&record.point);
            self.realtime_graph.set_landmark(id, point, None)?;
            self.full_graph
                .add_landmark(id, point, record.is_initialised)?;
            self.full_graph.set_landmark_quality(id, record.quality)?;
        }
        for mut add in std::mem::take(&mut self.add_gps_backlog) {
            if self.full_graph.states.contains_key(&add.id) {
                if add.reinit {
                    self.full_graph.reinit_gps_extrinsics();
                }
                self.full_graph.add_gps_measurement(
                    add.id,
                    &mut add.measurement,
                    &add.imu_measurements,
                )?;
            }
        }
        if !self.full_graph.gps_flags.gps_fixed {
            let global = self.full_graph.t_gw(None)?;
            self.realtime_graph.set_gps_extrinsics(&global)?;
            if self.gps_observability.load(Ordering::Acquire) {
                self.full_graph.freeze_gps_extrinsics()?;
                self.realtime_graph.freeze_gps_extrinsics()?;
            }
        }
        for add in std::mem::take(&mut self.add_submap_alignment_backlog) {
            if self.full_graph.states.contains_key(&add.b) {
                self.full_graph.add_submap_alignment_constraints(
                    &add.map,
                    (add.a, add.b),
                    &add.points,
                    &add.sensor_error,
                    add.is_lidar,
                    &add.robust,
                )?;
            }
        }
        Ok(())
    }

    fn replay_state_backlog(&mut self, old_id: StateId, change: &Transformation) -> Result<()> {
        for add in std::mem::take(&mut self.add_states_backlog) {
            let exists = self.realtime_graph.states.contains_key(&add.id);
            let keyframe = exists && self.realtime_graph.is_keyframe(add.id);
            let assigned = self.full_graph.add_states_propagate(
                add.timestamp,
                &add.imu_measurements,
                keyframe,
            )?;
            if assigned != add.id {
                return Err(graph_error("state backlog ID differs"));
            }
            let (mut sb, pose) = (
                self.full_graph.speed_and_bias(add.id)?,
                if exists {
                    let relative = self
                        .realtime_graph
                        .pose(old_id)?
                        .inverse()
                        .compose(&self.realtime_graph.pose(add.id)?);
                    self.full_graph.pose(old_id)?.compose(&relative)
                } else {
                    let any = self
                        .realtime_graph
                        .any_state
                        .get(&add.id)
                        .ok_or_else(|| graph_error("historical backlog state absent"))?;
                    let reference = any
                        .keyframe_id
                        .ok_or_else(|| graph_error("historical keyframe absent"))?;
                    let world_keyframe = change.compose(&self.realtime_graph.pose(reference)?);
                    world_keyframe.compose(&any.t_sk_s)
                },
            );
            let velocity = if exists {
                let old_velocity = self
                    .realtime_graph
                    .speed_and_bias(add.id)?
                    .fixed_rows::<3>(0)
                    .into_owned();
                let local = self
                    .realtime_graph
                    .pose(old_id)?
                    .inverse()
                    .rotation_matrix()
                    * old_velocity;
                pose.rotation_matrix() * local
            } else {
                let any = &self.realtime_graph.any_state[&add.id];
                let reference = any
                    .keyframe_id
                    .ok_or_else(|| graph_error("historical keyframe absent"))?;
                change
                    .compose(&self.realtime_graph.pose(reference)?)
                    .rotation_matrix()
                    * any.v_sk
            };
            sb.fixed_rows_mut::<3>(0).copy_from(&velocity);
            self.full_graph.set_pose(add.id, &pose)?;
            self.full_graph.set_speed_and_bias(add.id, sb)?;
        }
        Ok(())
    }

    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn attempt_loop_closure(
        &mut self,
        a: StateId,
        b: StateId,
        measurement: &Transformation,
        information: SMatrix<f64, 6, 6>,
        drift_percentage: f64,
    ) -> Result<(bool, bool)> {
        if self.full_graph_busy() {
            return Err(graph_error("loop closure still running"));
        }
        if !self.realtime_graph.states.contains_key(&a)
            || !self.realtime_graph.states.contains_key(&b)
        {
            return Ok((false, false));
        }
        let ids: Vec<_> = self
            .realtime_graph
            .states
            .range(a..)
            .map(|(&id, _)| id)
            .collect();
        let LoopPath {
            first_loop,
            steps,
            distances,
            travelled,
            travelled2,
            displacement,
        } = self.loop_path(a, &ids)?;
        let mut last_loop;
        let old_b = self.pose(b)?;
        let new_b = self.pose(a)?.compose(measurement);
        let final_change = new_b.compose(&old_b.inverse());
        let increment = Transformation::new(
            Vector3::zeros(),
            delta_q(&(quaternion_angle_axis(&final_change.quaternion()) / f64::from(steps))),
        );
        let mut previous = self.pose(a)?;
        let mut adjusted = self.pose(a)?;
        last_loop = first_loop;
        for &id in ids.iter().skip(1) {
            let old = self.pose(id)?;
            let relative = previous.inverse().compose(&old);
            previous = old;
            let loop_id = self.auxiliary_states[&id].loop_id;
            if loop_id == last_loop {
                adjusted = adjusted.compose(&relative);
            } else {
                adjusted = increment.compose(&adjusted).compose(&relative);
                last_loop = loop_id;
            }
        }
        let translation_error = new_b.translation() - adjusted.translation();
        let relative_position = translation_error.norm() / travelled2;
        let relative_orientation =
            quaternion_angular_distance(&(new_b.quaternion() * old_b.quaternion().conjugate()))
                / f64::from(steps);
        let position_budget = drift_percentage / 100.0
            + 0.02 * displacement.norm() / travelled2
            + 0.08 / f64::from(steps).sqrt();
        let orientation_budget = 0.0004 + 0.004 / f64::from(steps).sqrt();
        if relative_position > position_budget
            || relative_orientation > orientation_budget
            || steps < 1
        {
            return Ok((false, false));
        }
        let mut covariance = DMatrix::zeros(6, 6);
        PseudoInverse::symm(
            &DMatrix::from_column_slice(6, 6, information.as_slice()),
            &mut covariance,
            f64::EPSILON,
            None,
        );
        let eigen = SymmetricEigen::new(covariance.fixed_view::<3, 3>(0, 0).into_owned());
        let sigma = eigen.eigenvalues.sum().sqrt();
        if sigma > 0.1 && 3.0 * sigma > position_budget * travelled2 {
            return Ok((false, false));
        }
        self.distribute_loop_correction(
            (a, first_loop),
            &ids,
            &increment,
            (&distances, travelled),
            translation_error,
        )?;
        self.transform_landmarks(&final_change)?;
        self.auxiliary_states
            .get_mut(&b)
            .expect("state exists")
            .recognition
            .closed_loop = true;
        self.full_graph_relative_pose_constraints
            .push(RelativePoseInfo {
                t_si_sj: measurement.clone(),
                information,
                pose_i: a,
                pose_j: b,
            });
        Ok((true, false))
    }
    fn loop_path(&self, a: StateId, ids: &[StateId]) -> Result<LoopPath> {
        let first_loop = self.auxiliary_states[&a].loop_id;
        let mut last_loop = first_loop;
        let mut steps = 0;
        for &id in ids {
            let loop_id = self.auxiliary_states[&id].loop_id;
            if loop_id != last_loop {
                last_loop = loop_id;
                steps += 1;
            }
        }
        let mut previous = self.pose(a)?;
        let mut distances = Vec::new();
        let mut travelled = 0.0;
        let mut travelled2 = 0.0;
        let mut displacement = Vector3::zeros();
        last_loop = first_loop;
        if a != first_loop {
            displacement += self.pose(first_loop)?.translation() - previous.translation();
            travelled2 += displacement.norm();
        }
        for &id in ids.iter().skip(1) {
            let pose = self.pose(id)?;
            let loop_id = self.auxiliary_states[&id].loop_id;
            if loop_id != last_loop {
                let delta = pose.translation() - previous.translation();
                let distance = delta.norm();
                distances.push(distance);
                displacement += delta;
                travelled += distance;
                travelled2 += distance;
                last_loop = loop_id;
                previous = pose;
            }
        }
        Ok(LoopPath {
            first_loop,
            steps,
            distances,
            travelled,
            travelled2,
            displacement,
        })
    }

    fn distribute_loop_correction(
        &mut self,
        (a, first_loop): (StateId, StateId),
        ids: &[StateId],
        increment: &Transformation,
        (distances, travelled): (&[f64], f64),
        translation_error: Vector3<f64>,
    ) -> Result<()> {
        let mut last_loop;
        let mut previous = self.pose(a)?;
        let mut adjusted = self.pose(a)?;
        last_loop = first_loop;
        let (mut counter, mut ratio) = (0, 0.0);
        for &id in ids.iter().skip(1) {
            let old = self.pose(id)?;
            let relative = previous.inverse().compose(&old);
            previous = old.clone();
            let loop_id = self.auxiliary_states[&id].loop_id;
            if loop_id == last_loop {
                adjusted = adjusted.compose(&relative);
            } else {
                ratio += distances[counter] / travelled;
                adjusted = increment.compose(&adjusted).compose(&relative);
                last_loop = loop_id;
                counter += 1;
            }
            let pose = Transformation::new(
                adjusted.translation() + ratio * translation_error,
                adjusted.quaternion(),
            );
            let change = pose.compose(&old.inverse());
            let mut sb = self.speed_and_bias(id)?;
            let velocity = change.rotation_matrix() * sb.fixed_rows::<3>(0);
            sb.fixed_rows_mut::<3>(0).copy_from(&velocity);
            self.realtime_graph.set_pose(id, &pose)?;
            self.full_graph.set_pose(id, &pose)?;
            self.realtime_graph.set_speed_and_bias(id, sb)?;
            self.full_graph.set_speed_and_bias(id, sb)?;
            self.updated_states_loop_closure_attempt.insert(id);
        }
        Ok(())
    }

    fn transform_landmarks(&mut self, change: &Transformation) -> Result<()> {
        let ids: Vec<_> = self.realtime_graph.landmarks.keys().copied().collect();
        for id in ids {
            let record = self.realtime_graph.get_landmark(id)?;
            let point = change.transform_homogeneous(&record.point);
            self.realtime_graph
                .set_landmark(id, point, Some(record.is_initialised))?;
            self.full_graph
                .set_landmark(id, point, Some(record.is_initialised))?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn attempt_full_gps_alignment(
        &mut self,
        a: StateId,
        b: StateId,
        new_global: &Transformation,
    ) -> Result<bool> {
        if self.full_graph_busy() {
            return Err(graph_error("loop closure still running"));
        }
        if !self.realtime_graph.states.contains_key(&a)
            || !self.realtime_graph.states.contains_key(&b)
        {
            return Ok(false);
        }
        let final_change = self
            .realtime_graph
            .t_gw(Some(a))?
            .inverse()
            .compose(new_global);
        let target = final_change.compose(&self.pose(b)?);
        let ids: Vec<_> = self
            .realtime_graph
            .states
            .range(a..)
            .map(|(&id, _)| id)
            .collect();
        let mut distances = Vec::new();
        let mut travelled = 0.0;
        let mut previous = self.pose(a)?;
        for &id in ids.iter().skip(1) {
            let pose = self.pose(id)?;
            let distance = (pose.translation() - previous.translation()).norm();
            distances.push(distance);
            travelled += distance;
            previous = pose;
        }
        let increment = Transformation::new(
            Vector3::zeros(),
            delta_q(&(quaternion_angle_axis(&final_change.quaternion()) / ids.len() as f64)),
        );
        let mut previous = self.pose(a)?;
        let mut adjusted = self.pose(a)?;
        for &id in ids.iter().skip(1) {
            let old = self.pose(id)?;
            let relative = previous.inverse().compose(&old);
            previous = old;
            adjusted = increment.compose(&adjusted).compose(&relative);
        }
        let translation_error = target.translation() - adjusted.translation();
        let mut previous = self.pose(a)?;
        let mut adjusted = self.pose(a)?;
        let mut ratio = 0.0;
        for (index, &id) in ids.iter().skip(1).enumerate() {
            let old = self.pose(id)?;
            let relative = previous.inverse().compose(&old);
            previous = old.clone();
            ratio += distances[index] / travelled;
            adjusted = increment.compose(&adjusted).compose(&relative);
            let pose = Transformation::new(
                adjusted.translation() + ratio * translation_error,
                adjusted.quaternion(),
            );
            let change = pose.compose(&old.inverse());
            let mut sb = self.speed_and_bias(id)?;
            let velocity = change.rotation_matrix() * sb.fixed_rows::<3>(0);
            sb.fixed_rows_mut::<3>(0).copy_from(&velocity);
            self.realtime_graph.set_pose(id, &pose)?;
            self.full_graph.set_pose(id, &pose)?;
            self.realtime_graph.set_speed_and_bias(id, sb)?;
            self.full_graph.set_speed_and_bias(id, sb)?;
        }
        self.transform_landmarks(&final_change)?;
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn attempt_pos_gps_alignment(
        &mut self,
        a: StateId,
        b: StateId,
        error: Vector3<f64>,
    ) -> Result<bool> {
        if self.full_graph_busy() {
            return Err(graph_error("loop closure still running"));
        }
        if !self.realtime_graph.states.contains_key(&a)
            || !self.realtime_graph.states.contains_key(&b)
        {
            return Ok(false);
        }
        self.full_graph.unfreeze_poses_from(a)?;
        self.full_graph.freeze_poses_until(a, false)?;
        self.full_graph.unfreeze_speed_and_biases_from(a)?;
        self.full_graph.freeze_speed_and_biases_until(a, false)?;
        let ids: Vec<_> = self
            .realtime_graph
            .states
            .range((std::ops::Bound::Excluded(a), std::ops::Bound::Unbounded))
            .map(|(&id, _)| id)
            .collect();
        let mut distances = Vec::new();
        let mut full_distance = 0.0;
        let mut previous = a;
        for &id in ids.iter().take_while(|&&id| id <= b) {
            let distance =
                (self.pose(id)?.translation() - self.pose(previous)?.translation()).norm();
            distances.push(distance);
            full_distance += distance;
            previous = id;
        }
        let mut travelled = 0.0;
        let mut index = 0;
        for id in ids {
            let delta = if id <= b {
                travelled += distances[index];
                index += 1;
                travelled / full_distance * error
            } else {
                error
            };
            let old = self.pose(id)?;
            let pose = Transformation::new(old.translation() + delta, old.quaternion());
            self.realtime_graph.set_pose(id, &pose)?;
            self.full_graph.set_pose(id, &pose)?;
        }
        let landmarks: Vec<_> = self.realtime_graph.landmarks.keys().copied().collect();
        for id in landmarks {
            let record = self.realtime_graph.get_landmark(id)?;
            let point = record.point + Vector4::new(error[0], error[1], error[2], 0.0);
            self.realtime_graph
                .set_landmark(id, point, Some(record.is_initialised))?;
            self.full_graph
                .set_landmark(id, point, Some(record.is_initialised))?;
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn add_submap_alignment_constraints(
        &mut self,
        map: &Rc<dyn OccupancySubmap>,
        (a, b): (StateId, StateId),
        points: &[Vector3<f32>],
        sensor_error: &[f32],
        is_lidar: bool,
        robust: &str,
    ) -> Result<bool> {
        if self.full_graph_busy() {
            self.add_submap_alignment_backlog
                .push(AddSubmapAlignmentBacklog {
                    map: map.clone(),
                    a,
                    b,
                    points: points.to_vec(),
                    sensor_error: sensor_error.to_vec(),
                    is_lidar,
                    robust: robust.to_owned(),
                });
        } else {
            self.full_graph.add_submap_alignment_constraints(
                &map.clone(),
                (a, b),
                points,
                sensor_error,
                is_lidar,
                robust,
            )?;
        }
        self.realtime_graph.add_submap_alignment_constraints(
            map,
            (a, b),
            points,
            sensor_error,
            is_lidar,
            robust,
        )
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn do_final_ba(
        &mut self,
        iterations: usize,
        position_uncertainty: f64,
        orientation_uncertainty: f64,
        threads: usize,
        verbose: bool,
    ) -> Result<FinalBaStatistics> {
        let ids: Vec<_> = self.full_graph.states.keys().copied().collect();
        for id in ids {
            if !self.full_graph.state_info[&id]
                .two_pose_const_links
                .is_empty()
            {
                if !self.keyframes.contains(&id) && !self.loop_closure_frames.contains(&id) {
                    self.keyframes.insert(id);
                }
                self.expand_keyframe(id)?;
            }
        }
        self.full_graph.clean_unobserved_landmarks()?;
        self.full_graph.unfreeze_poses_from(StateId::new(1))?;
        self.full_graph
            .unfreeze_speed_and_biases_from(StateId::new(1))?;
        if !self.full_graph.gps_parameters.is_empty() {
            self.full_graph.unfreeze_gps_extrinsics()?;
        }
        crate::ceres::imu_error::REDO_PROPAGATION_ALWAYS.store(true, Ordering::Release);
        self.optimise_full_graph(iterations, threads, verbose)?;
        self.full_graph
            .remove_speed_and_bias_prior(StateId::new(1))?;
        if position_uncertainty > 0.0 && orientation_uncertainty > 0.0 {
            self.full_graph.set_extrinsics_variable()?;
            self.full_graph
                .soft_constrain_extrinsics(position_uncertainty, orientation_uncertainty)?;
        }
        let summary = self.optimise_full_graph(iterations, threads, verbose)?;
        self.draw_overhead_image(2000, 2000, 2)?;
        self.raster
            .as_mut()
            .ok_or_else(|| graph_error("raster capability absent"))?
            .write_image(Path::new("fullBa.png"))?;
        let mut bias_mean = SVector::<f64, 6>::zeros();
        for &id in self.full_graph.states.keys() {
            bias_mean += self.full_graph.speed_and_bias(id)?.fixed_rows::<6>(3);
        }
        bias_mean /= self.full_graph.states.len() as f64;
        let mut bias_std = SVector::<f64, 6>::zeros();
        for &id in self.full_graph.states.keys() {
            let delta = self.full_graph.speed_and_bias(id)?.fixed_rows::<6>(3) - bias_mean;
            bias_std += delta.component_mul(&delta);
        }
        bias_std = bias_std.map(f64::sqrt);
        let cameras = self
            .multi_frames
            .values()
            .next_back()
            .ok_or_else(|| graph_error("multiframes empty"))?
            .borrow()
            .num_frames();
        let mut histograms = vec![[0usize; 12]; cameras];
        let (mut inliers, mut outliers) = (vec![0; cameras], vec![0; cameras]);
        for &key in self.full_graph.observations.keys() {
            let error = self
                .full_graph
                .observation_evaluation(key, false)?
                .residuals
                .norm();
            if error / 3.0 > 1.0 {
                outliers[key.camera_index] += 1;
            } else {
                inliers[key.camera_index] += 1;
            }
            let index = ((error.floor() as i32) + 6).clamp(0, 11) as usize;
            histograms[key.camera_index][index] += 1;
        }
        self.synchronise_realtime_and_full_graph()?;
        let updated_states = self.full_graph.states.keys().copied().collect();
        self.clean_unobserved_landmarks()?;
        Ok(FinalBaStatistics {
            summary,
            updated_states,
            bias_mean,
            bias_std,
            error_histograms: histograms,
            inlier_counts: inliers,
            outlier_counts: outliers,
        })
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn print_states(&self, id: StateId, output: &mut dyn Write) -> Result<()> {
        if self.is_keyframe(id) {
            write!(output, "KF ").map_err(|error| io_error(&error))?;
        }
        write!(output, "pose: ").map_err(|error| io_error(&error))?;
        let p = self.realtime_graph.pose_block(id)?;
        let p = p.borrow();
        write!(
            output,
            "{}id={}:{}",
            if p.fixed() { "(" } else { "" },
            id.value(),
            if p.fixed() { ")" } else { "" }
        )
        .map_err(|error| io_error(&error))?;
        write!(output, ", speedAndBias: ").map_err(|error| io_error(&error))?;
        let sb = self.realtime_graph.speed_bias_block(id)?;
        let sb = sb.borrow();
        write!(
            output,
            "{}id={}:{}",
            if sb.fixed() { "(" } else { "" },
            sb.id(),
            if sb.fixed() { ")" } else { "" }
        )
        .map_err(|error| io_error(&error))?;
        write!(output, ", extrinsics: ").map_err(|error| io_error(&error))?;
        for i in 0..self.realtime_graph.states[&id].extrinsics.len() {
            let ext = self.realtime_graph.extrinsics_block(id, i)?;
            let ext = ext.borrow();
            write!(
                output,
                "{}id={}:{}",
                if ext.fixed() { "(" } else { "" },
                ext.id(),
                if ext.fixed() { ")" } else { "" }
            )
            .map_err(|error| io_error(&error))?;
            if i + 1 < self.realtime_graph.states[&id].extrinsics.len() {
                write!(output, ", ").map_err(|error| io_error(&error))?;
            }
        }
        writeln!(output).map_err(|error| io_error(&error))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn save_map(&mut self, path: impl AsRef<Path>) -> Result<bool> {
        let path = path.as_ref();
        let text = path.to_string_lossy();
        if text.len() < 4 {
            return Err(graph_error("map path must have four-character extension"));
        }
        let g2o = format!("{}.g2o", &text[..text.len() - 4]);
        let rig = self
            .multi_frame(StateId::new(1))?
            .borrow()
            .camera_system()
            .clone();
        let imu = self
            .full_graph
            .imu_parameters
            .first()
            .ok_or_else(|| graph_error("IMU parameters absent"))?
            .clone();
        let component =
            Component::with_graph(imu, rig, &mut self.full_graph, self.multi_frames.clone());
        component.save(g2o)?;
        let mut file = BufWriter::new(File::create(path).map_err(|error| io_error(&error))?);
        writeln!(file, "landmarks:").map_err(|error| io_error(&error))?;
        for (&id, lm) in &self.full_graph.landmarks {
            if lm.quality > 0.001 {
                let p = self.full_graph.landmark(id)?;
                writeln!(
                    file,
                    "{},{},{},{}",
                    id.value(),
                    p[0] / p[3],
                    p[1] / p[3],
                    p[2] / p[3]
                )
                .map_err(|error| io_error(&error))?;
            }
        }
        self.full_graph.compute_covisibilities();
        for (&id, state) in &self.full_graph.states {
            write!(file, "frame: {}, covisibilities: ", id.value())
                .map_err(|error| io_error(&error))?;
            let mut covisible = Vec::new();
            for &other in self.full_graph.states.keys() {
                let count = self.full_graph.covisibilities(id, other)?;
                if count > 0 {
                    covisible.push((other, count));
                }
            }
            covisible.sort_by_key(|a| std::cmp::Reverse(a.1));
            for (id, _) in covisible {
                write!(file, "{} ", id.value()).map_err(|error| io_error(&error))?;
            }
            writeln!(file).map_err(|error| io_error(&error))?;
            for (&key, obs) in &state.observations {
                let lm = &self.full_graph.landmarks[&obs.landmark_id];
                if lm.quality <= 0.001 {
                    continue;
                }
                let p = self.full_graph.landmark(obs.landmark_id)?;
                write!(
                    file,
                    "{},{},{},{},{},",
                    key.keypoint_index,
                    obs.landmark_id.value(),
                    p[0] / p[3],
                    p[1] / p[3],
                    p[2] / p[3]
                )
                .map_err(|error| io_error(&error))?;
                let frame = self.multi_frame(id)?;
                let frame = frame.borrow();
                let descriptor = frame.keypoint_descriptor(key.camera_index, key.keypoint_index);
                if descriptor.len() < 48 {
                    return Err(graph_error("BRISK2 descriptor shorter than 48 bytes"));
                }
                for byte in &descriptor[..48] {
                    write!(file, "{byte:02x}").map_err(|error| io_error(&error))?;
                }
                writeln!(file).map_err(|error| io_error(&error))?;
            }
        }
        file.flush().map_err(|error| io_error(&error))?;
        Ok(true)
    }
    fn historical_state(&self, id: StateId) -> Result<(Transformation, SpeedAndBias, StateId)> {
        let any = self
            .realtime_graph
            .any_state
            .get(&id)
            .ok_or_else(|| graph_error("historical state absent"))?;
        if let Some(reference) = any.keyframe_id {
            let pose = self.pose(reference)?;
            let sb = self.speed_and_bias(reference)?;
            let mut reconstructed = sb;
            reconstructed
                .fixed_rows_mut::<3>(0)
                .copy_from(&(pose.rotation_matrix() * any.v_sk));
            Ok((pose.compose(&any.t_sk_s), reconstructed, reference))
        } else {
            Ok((self.pose(id)?, self.speed_and_bias(id)?, id))
        }
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn write_final_csv_trajectory(&self, path: impl AsRef<Path>, rpg: bool) -> Result<bool> {
        let mut file = BufWriter::new(File::create(path).map_err(|error| io_error(&error))?);
        if rpg {
            writeln!(file, "# timestamp tx ty tz qx qy qz qw").map_err(|error| io_error(&error))?;
        } else {
            writeln!(file,"timestamp, p_WS_W_x, p_WS_W_y, p_WS_W_z, q_WS_x, q_WS_y, q_WS_z, q_WS_w, v_WS_W_x, v_WS_W_y, v_WS_W_z, b_g_x, b_g_y, b_g_z, b_a_x, b_a_y, b_a_z, NrGps, SID, gpsMode").map_err(|error| io_error(&error))?;
        }
        for (&id, any) in &self.realtime_graph.any_state {
            let (pose, sb, reference) = self.historical_state(id)?;
            let p = pose.translation();
            let q = pose.quaternion();
            if rpg {
                writeln!(
                    file,
                    "{:.19} {} {} {} {} {} {} {}",
                    any.timestamp.as_nanosecond() as f64 * 1.0e-9,
                    p[0],
                    p[1],
                    p[2],
                    q.i,
                    q.j,
                    q.k,
                    q.w
                )
                .map_err(|error| io_error(&error))?;
            } else {
                let info = &self.realtime_graph.state_info[&reference];
                let mode = info.gps_mode.ok_or_else(|| {
                    graph_error("source GPS mode is uninitialized for this state")
                })?;
                write!(
                    file,
                    "{}, {:.18e}, {:.18e}, {:.18e}, {:.18e}, {:.18e}, {:.18e}, {:.18e}",
                    any.timestamp.as_nanosecond(),
                    p[0],
                    p[1],
                    p[2],
                    q.i,
                    q.j,
                    q.k,
                    q.w
                )
                .map_err(|error| io_error(&error))?;
                for value in sb.iter() {
                    write!(file, ", {value:.18e}").map_err(|error| io_error(&error))?;
                }
                writeln!(
                    file,
                    ", {}, {}, {}, {}",
                    info.gps_factors.len(),
                    id.value(),
                    mode as usize,
                    any.keyframe_id.unwrap_or_default().value()
                )
                .map_err(|error| io_error(&error))?;
            }
        }
        file.flush().map_err(|error| io_error(&error))?;
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn write_global_csv_trajectory(&self, path: impl AsRef<Path>) -> Result<bool> {
        let mut file = BufWriter::new(File::create(path).map_err(|error| io_error(&error))?);
        writeln!(file, "timestamp, p_GA_G_x, p_GA_G_y, p_GA_G_z")
            .map_err(|error| io_error(&error))?;
        let global = self.realtime_graph.t_gw(None)?;
        let antenna = self
            .realtime_graph
            .gps_parameters
            .last()
            .ok_or_else(|| graph_error("GPS parameters absent"))?
            .r_sa;
        for (&id, any) in &self.realtime_graph.any_state {
            let (pose, _, _) = self.historical_state(id)?;
            let p = global.rotation_matrix()
                * (pose.translation() + pose.rotation_matrix() * antenna)
                + global.translation();
            writeln!(
                file,
                "{}, {:.18e}, {:.18e}, {:.18e}",
                any.timestamp.as_nanosecond(),
                p[0],
                p[1],
                p[2]
            )
            .map_err(|error| io_error(&error))?;
        }
        file.flush().map_err(|error| io_error(&error))?;
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn draw_overhead_image(&mut self, rows: usize, cols: usize, index: i32) -> Result<()> {
        let primitives = self.overhead_primitives(rows, cols, index)?;
        self.raster
            .as_mut()
            .ok_or_else(|| graph_error("drawing raster capability absent"))?
            .draw(rows, cols, &primitives)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn overhead_primitives(
        &self,
        rows: usize,
        cols: usize,
        index: i32,
    ) -> Result<Vec<DrawPrimitive>> {
        let graph = if index == 0 {
            &self.realtime_graph
        } else {
            &self.full_graph
        };
        let mut minimum = Vector3::<f64>::repeat(-0.05);
        let mut maximum = Vector3::<f64>::repeat(0.05);
        for &id in graph.states.keys() {
            let p = graph.pose(id)?.translation();
            for i in 0..3 {
                minimum[i] = minimum[i].min(p[i]);
                maximum[i] = maximum[i].max(p[i]);
            }
        }
        let center = (minimum + maximum) * 0.5;
        let scale = ((cols as f64 - 100.0) / (maximum[0] - minimum[0]))
            .min((rows as f64 - 100.0) / (maximum[1] - minimum[1]));
        let pixel = |p: Vector3<f64>| {
            let p = (p - center) * scale;
            Vector2::new(p[0] + cols as f64 * 0.5, -p[1] + rows as f64 * 0.5)
        };
        let mut output = Vec::new();
        Self::overhead_landmarks(graph, (rows, cols), &pixel, &mut output)?;
        Self::overhead_links(graph, &pixel, &mut output)?;
        self.overhead_states(graph, index, &pixel, &mut output)?;
        let newest = graph.current_state_id()?;
        if index <= 2 {
            let mut counts: BTreeMap<_, usize> =
                self.loop_closure_frames.iter().map(|&id| (id, 0)).collect();
            let current = self.current_state_id()?;
            for landmark in graph.landmark_info.values() {
                if landmark
                    .observations
                    .iter()
                    .any(|k| k.frame_id == current.value())
                {
                    let visible: BTreeSet<_> = landmark
                        .observations
                        .iter()
                        .map(|k| StateId::new(k.frame_id))
                        .filter(|id| counts.contains_key(id))
                        .collect();
                    for id in visible {
                        *counts.get_mut(&id).expect("ID filtered") += 1;
                    }
                }
            }
            for (&id, &count) in &counts {
                let p = pixel(graph.pose(id)?.translation());
                output.push(circle(p, 5, [255.0; 3], -1));
                output.push(circle(
                    p,
                    (6.0 + count as f32 / 10.0) as i32,
                    [255.0, 0.0, 0.0],
                    ((1 + count) / 5) as i32,
                ));
            }
            let p = pixel(graph.pose(newest)?.translation());
            output.push(circle(p, 5, [255.0; 3], 2));
            output.push(circle(
                p,
                3,
                if graph.is_keyframe(newest) {
                    [255.0, 255.0, 0.0]
                } else {
                    [127.0; 3]
                },
                -1,
            ));
            if let Some(id) = self.current_keyframe_state_id(true)? {
                let p = pixel(graph.pose(id)?.translation());
                output.push(circle(p, 5, [255.0; 3], 2));
                output.push(circle(p, 3, [255.0, 255.0, 127.0], -1));
            }
        }
        let pose = graph.pose(newest)?;
        let sb = graph.speed_and_bias(newest)?;
        let p = pose.translation();
        for (index, (label, v)) in [
            ("position", p),
            ("velocity", sb.fixed_rows::<3>(0).into_owned()),
            ("gyro bias", sb.fixed_rows::<3>(3).into_owned()),
            ("acc bias", sb.fixed_rows::<3>(6).into_owned()),
        ]
        .into_iter()
        .enumerate()
        {
            output.push(DrawPrimitive::Text {
                at: Vector2::new(15.0, 15.0 + 20.0 * index as f64),
                text: format!("{label} = [{}, {}, {}]", v[0], v[1], v[2]),
                scale: 0.5,
                bgr: [255.0; 3],
            });
        }
        Ok(output)
    }
    fn overhead_landmarks(
        graph: &ViGraphEstimator,
        (rows, cols): (usize, usize),
        pixel: &impl Fn(Vector3<f64>) -> Vector2<f64>,
        output: &mut Vec<DrawPrimitive>,
    ) -> Result<()> {
        for (&id, lm) in &graph.landmarks {
            let point = graph.landmark(id)?;
            let p = pixel(point.fixed_rows::<3>(0).into_owned() / point[3]);
            if p[0] > 0.0 && p[1] > 0.0 && p[0] < (cols as f64 - 1.0) && p[1] < (rows as f64 - 1.0)
            {
                output.push(circle(
                    p,
                    1,
                    [
                        0.0,
                        f64::from((20 + (lm.quality / 0.03 * 225.0) as i32).min(255)),
                        0.0,
                    ],
                    -1,
                ));
            }
        }
        for (&id, info) in &graph.state_info {
            let _ = id;
            for (_, factor) in &info.gps_factors {
                let global = graph.t_gw(None)?;
                let world = global
                    .inverse()
                    .transform_homogeneous(&factor.borrow().measurement.push(1.0));
                output.push(DrawPrimitive::Diamond {
                    center: pixel(world.fixed_rows::<3>(0).into_owned()),
                    size: 12,
                    bgr: [255.0, 0.0, 255.0],
                    thickness: 3,
                });
            }
        }
        Ok(())
    }

    fn overhead_states(
        &self,
        graph: &ViGraphEstimator,
        index: i32,
        pixel: &impl Fn(Vector3<f64>) -> Vector2<f64>,
        output: &mut Vec<DrawPrimitive>,
    ) -> Result<()> {
        let ids: Vec<_> = graph.states.keys().copied().collect();
        for (i, &id) in ids.iter().enumerate() {
            let p = pixel(graph.pose(id)?.translation());
            let mut color = if graph.is_keyframe(id) {
                [255.0, 0.0, 0.0]
            } else {
                [127.0; 3]
            };
            let mut border = [255.0; 3];
            if index <= 2 {
                if self.keyframes.contains(&id) {
                    color = [255.0, 255.0, 0.0];
                }
                let auxiliary = self
                    .auxiliary_states
                    .get(&id)
                    .ok_or_else(|| graph_error("draw auxiliary state absent"))?;
                if auxiliary.is_pose_graph_frame {
                    border = if graph.pose_block(id)?.borrow().fixed() {
                        [25.0, 25.0, 200.0]
                    } else {
                        [127.0; 3]
                    };
                }
            }
            if graph.state_info[&id].next_imu.is_some() {
                let next = ids
                    .get(i + 1)
                    .ok_or_else(|| graph_error("IMU link has no next state"))?;
                output.push(line(
                    p,
                    pixel(graph.pose(*next)?.translation()),
                    [0.0, 0.0, 255.0],
                    1,
                ));
            }
            output.push(circle(p, 3, color, -1));
            output.push(circle(p, 3, border, 1));
            output.push(DrawPrimitive::Text {
                at: p + Vector2::new(6.0, 3.0),
                text: id.value().to_string(),
                scale: 0.3,
                bgr: [255.0; 3],
            });
        }
        Ok(())
    }

    fn overhead_links(
        graph: &ViGraphEstimator,
        pixel: &impl Fn(Vector3<f64>) -> Vector2<f64>,
        output: &mut Vec<DrawPrimitive>,
    ) -> Result<()> {
        let mut covisible: BTreeMap<(u64, u64), usize> = BTreeMap::new();
        for lm in graph.landmark_info.values() {
            let ids: BTreeSet<_> = lm.observations.iter().map(|k| k.frame_id).collect();
            for &a in &ids {
                for &b in ids.range(..a) {
                    *covisible.entry((a, b)).or_default() += 1;
                }
            }
        }
        for ((a, b), count) in covisible {
            let (a, b) = (StateId::new(a), StateId::new(b));
            if graph.states.contains_key(&a) && graph.states.contains_key(&b) {
                let brightness = 50.0 + (count * 2).min(205) as f64;
                output.push(line(
                    pixel(graph.pose(a)?.translation()),
                    pixel(graph.pose(b)?.translation()),
                    [0.0, brightness, brightness],
                    1,
                ));
            }
        }
        for (&id, info) in &graph.state_info {
            for (&other, &rid) in &info.two_pose_links {
                let factor = &graph.problem.residuals[&rid].factor;
                if let GraphFactor::Visual(factor) = factor.as_ref() {
                    let f = factor.borrow();
                    let reference = f.base().reference_pose_id;
                    let other = f.base().other_pose_id;
                    if reference < other {
                        let strength = match &*f {
                            crate::ceres::vi_graph::VisualFactor::Standard(f) => f.strength(),
                            crate::ceres::vi_graph::VisualFactor::Extrinsics(f) => f.strength(),
                        };
                        let brightness = 50.0 + strength.min(205.0);
                        output.push(line(
                            pixel(graph.pose(reference)?.translation()),
                            pixel(graph.pose(other)?.translation()),
                            [brightness / 2.0, brightness, brightness],
                            3,
                        ));
                    }
                }
                let _ = other;
            }
            for &other in info.two_pose_const_links.keys() {
                output.push(line(
                    pixel(graph.pose(id)?.translation()),
                    pixel(graph.pose(other)?.translation()),
                    [100.0, 200.0, 100.0],
                    3,
                ));
            }
        }
        Ok(())
    }

    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn write_lidar_debug_statistics_csv(&self, prefix: &str) -> Result<()> {
        self.realtime_graph.write_lidar_debug_statistics_csv(prefix)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn dump_gps_residuals(&mut self, path: impl AsRef<Path>) -> Result<()> {
        self.realtime_graph.dump_gps_residuals(path)
    }
    pub fn clear(&mut self) {
        self.realtime_graph.clear();
        self.full_graph.clear();
        self.multi_frames.clear();
        self.auxiliary_states.clear();
        self.current_component_index = 0;
        self.loop_closure_frames.clear();
        self.imu_frames.clear();
        self.keyframes.clear();
        self.needs_full_graph_optimisation
            .store(false, Ordering::Release);
        self.is_loop_closing.store(false, Ordering::Release);
        self.is_loop_closure_available
            .store(false, Ordering::Release);
        self.components.resize_with(1, Component0::default);
        self.add_states_backlog.clear();
        self.eliminate_states.clear();
        self.touched_states.clear();
        self.touched_landmarks.clear();
        self.full_graph_relative_pose_constraints.clear();
        self.last_freeze = None;
    }
}
fn circle(center: Vector2<f64>, radius: i32, bgr: [f64; 3], thickness: i32) -> DrawPrimitive {
    DrawPrimitive::Circle {
        center,
        radius,
        bgr,
        thickness,
    }
}
fn line(a: Vector2<f64>, b: Vector2<f64>, bgr: [f64; 3], thickness: i32) -> DrawPrimitive {
    DrawPrimitive::Line {
        a,
        b,
        bgr,
        thickness,
    }
}
fn parameter_state_id(parameter: &Parameter) -> Result<StateId> {
    if let Parameter::Pose(p) = parameter {
        Ok(StateId::new(p.borrow().id()))
    } else {
        Err(graph_error("expected pose parameter"))
    }
}
fn sync_imu(source: &ViGraphEstimator, destination: &ViGraphEstimator, id: StateId) -> Result<()> {
    let Some(left) = source.state_info.get(&id).and_then(|s| s.previous_imu) else {
        return Ok(());
    };
    let right = destination
        .state_info
        .get(&id)
        .and_then(|s| s.previous_imu)
        .ok_or_else(|| graph_error("destination IMU link absent"))?;
    match (
        source.problem.residuals[&left].factor.as_ref(),
        destination.problem.residuals[&right].factor.as_ref(),
    ) {
        (GraphFactor::Imu(a), GraphFactor::Imu(b)) => b.borrow_mut().sync_from(&a.borrow()),
        (GraphFactor::PseudoImu(a), GraphFactor::PseudoImu(b)) => {
            b.borrow_mut().sync_from(&a.borrow());
        }
        _ => return Err(graph_error("IMU synchronization factor mismatch")),
    }
    Ok(())
}

impl ViSlamBackend {
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn prepare_full_graph_task(
        &mut self,
        iterations: usize,
        threads: usize,
    ) -> Result<Box<dyn FnOnce() -> Result<SolverUpdate> + Send>> {
        if self.is_loop_closing.swap(true, Ordering::AcqRel) {
            return Err(graph_error("full graph already optimizing"));
        }
        self.needs_full_graph_optimisation
            .store(false, Ordering::Release);
        let result = (|| {
            let mut temporary = BTreeSet::new();
            for item in std::mem::take(&mut self.full_graph_relative_pose_constraints) {
                self.full_graph.add_relative_pose_constraint(
                    item.pose_i,
                    item.pose_j,
                    &item.t_si_sj,
                    item.information * 100.0,
                )?;
                temporary.insert(
                    self.full_graph.state_info[&item.pose_i].relative_pose_links[&item.pose_j],
                );
                self.pending_full_graph_residuals
                    .push((item.pose_i, item.pose_j));
            }
            let mut stages = Vec::new();
            let mut options = self.full_graph.options.clone();
            options.num_threads = threads;
            if !temporary.is_empty() {
                options.max_iterations = iterations / 3;
                options.function_tolerance = 0.001;
                stages.push(SolverStage {
                    options: options.clone(),
                    disabled_residuals: BTreeSet::new(),
                });
            }
            options.max_iterations = iterations;
            options.function_tolerance = 1.0e-6;
            stages.push(SolverStage {
                options,
                disabled_residuals: temporary,
            });
            let graph = &mut self.full_graph.graph;
            graph
                .solver
                .as_mut()
                .ok_or_else(|| graph_error("Ceres solver snapshot capability absent"))?
                .prepare_solve(&graph.problem, stages)
        })();
        if result.is_err() {
            for (a, b) in std::mem::take(&mut self.pending_full_graph_residuals) {
                self.full_graph.remove_relative_pose_constraint(a, b)?;
            }
            self.is_loop_closing.store(false, Ordering::Release);
        }
        result
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn finish_full_graph_task(&mut self, result: SolverUpdate) -> Result<()> {
        let update = (|| {
            result.restore_factor_caches(&self.full_graph.problem)?;
            for (key, values) in result.parameters {
                let (parameter, _) = self
                    .full_graph
                    .problem
                    .parameters
                    .get(&key)
                    .ok_or_else(|| graph_error("solver update parameter absent"))?;
                let dimension = parameter.values()?.len();
                if values.len() != dimension {
                    return Err(graph_error("solver update parameter dimension differs"));
                }
                match parameter {
                    Parameter::Pose(p) => p.borrow_mut().set_parameters(&values),
                    Parameter::SpeedBias(p) => p.borrow_mut().set_parameters(&values),
                    Parameter::Point(p) => p.borrow_mut().set_parameters(&values),
                }
            }
            for (a, b) in std::mem::take(&mut self.pending_full_graph_residuals) {
                self.full_graph.remove_relative_pose_constraint(a, b)?;
            }
            self.full_graph.summary = result.summary;
            self.full_graph.update_landmarks()?;
            self.is_loop_closure_available
                .store(true, Ordering::Release);
            Ok(())
        })();
        self.is_loop_closing.store(false, Ordering::Release);
        update
    }
}
impl crate::frontend::pipeline::FrontendGraph for ViSlamBackend {
    fn frame(&self, id: StateId) -> MultiFramePtr {
        self.multi_frame(id).expect("frontend frame ID exists")
    }
    fn frame_ids(&self) -> Vec<StateId> {
        self.multi_frames.keys().copied().collect()
    }
    fn pose(&self, id: StateId) -> Transformation {
        ViSlamBackend::pose(self, id).expect("frontend pose exists")
    }
    fn extrinsics(&self, id: StateId, camera: usize) -> Result<Transformation> {
        ViSlamBackend::extrinsics(self, id, camera)
    }
    fn set_pose(&mut self, id: StateId, pose: Transformation) -> Result<()> {
        ViSlamBackend::set_pose(self, id, &pose)?;
        Ok(())
    }
    fn landmarks(&self) -> MapPoints {
        self.get_landmarks()
            .expect("graph landmark estimates exist")
    }
    fn get_landmark(&self, id: LandmarkId) -> Result<MapPoint2> {
        ViSlamBackend::get_landmark(self, id)
    }
    fn landmark(&self, id: LandmarkId) -> Result<Vector4<f64>> {
        self.realtime_graph.landmark(id)
    }
    fn is_landmark_added(&self, id: LandmarkId) -> bool {
        ViSlamBackend::is_landmark_added(self, id)
    }
    fn is_landmark_initialised(&self, id: LandmarkId) -> Result<bool> {
        ViSlamBackend::is_landmark_initialised(self, id)
    }
    fn is_observed(&self, key: KeypointIdentifier) -> bool {
        ViSlamBackend::is_observed(self, key)
    }
    fn add_landmark(&mut self, point: Vector4<f64>, initialized: bool) -> Result<LandmarkId> {
        self.add_landmark_auto(point, initialized)
    }
    fn set_landmark(
        &mut self,
        id: LandmarkId,
        point: Vector4<f64>,
        initialized: bool,
    ) -> Result<()> {
        ViSlamBackend::set_landmark(self, id, point, initialized)?;
        Ok(())
    }
    fn add_observation(&mut self, id: LandmarkId, key: KeypointIdentifier) -> Result<()> {
        ViSlamBackend::add_observation(
            self,
            id,
            StateId::new(key.frame_id),
            key.camera_index,
            key.keypoint_index,
            true,
        )?;
        Ok(())
    }
    fn remove_observation(&mut self, key: KeypointIdentifier) -> Result<()> {
        ViSlamBackend::remove_observation(
            self,
            StateId::new(key.frame_id),
            key.camera_index,
            key.keypoint_index,
        )?;
        Ok(())
    }
    fn merge_landmark(&mut self, from: LandmarkId, into: LandmarkId) -> Result<()> {
        ViSlamBackend::merge_landmark(self, from, into)?;
        Ok(())
    }
    fn merge_landmarks(&mut self, from: &[LandmarkId], into: &[LandmarkId]) -> Result<usize> {
        ViSlamBackend::merge_landmarks(self, from, into)
    }
    fn set_observation_information(
        &mut self,
        key: KeypointIdentifier,
        information: Matrix2<f64>,
    ) -> Result<()> {
        ViSlamBackend::set_observation_information(
            self,
            StateId::new(key.frame_id),
            key.camera_index,
            key.keypoint_index,
            information,
        )?;
        Ok(())
    }
    fn set_landmark_classification(&mut self, id: LandmarkId, class: i32) -> Result<()> {
        ViSlamBackend::set_landmark_classification(self, id, class)?;
        Ok(())
    }
    fn is_keyframe(&self, id: StateId) -> bool {
        ViSlamBackend::is_keyframe(self, id)
    }
    fn keyframes(&self) -> BTreeSet<StateId> {
        self.keyframes.clone()
    }
    fn imu_frames(&self) -> BTreeSet<StateId> {
        self.imu_frames.clone()
    }
    fn loop_closure_frames(&self) -> BTreeSet<StateId> {
        self.loop_closure_frames.clone()
    }
    fn tracking_quality(&self, id: StateId) -> f64 {
        ViSlamBackend::tracking_quality(self, id).expect("tracking quality raster is configured")
    }
    fn optimise_realtime(
        &mut self,
        iterations: i32,
        threads: i32,
        initialized: bool,
    ) -> Result<()> {
        self.optimise_realtime_graph(
            iterations.max(0) as usize,
            threads.max(0) as usize,
            false,
            true,
            initialized,
        )?;
        Ok(())
    }
    fn is_loop_closing(&self) -> bool {
        self.is_loop_closing.load(Ordering::Acquire)
    }
    fn is_loop_closure_available(&self) -> bool {
        ViSlamBackend::is_loop_closure_available(self)
    }
    fn needs_full_graph_optimisation(&self) -> bool {
        self.needs_full_graph_optimisation.load(Ordering::Acquire)
    }
    fn is_pose_graph_frame(&self, id: StateId) -> bool {
        self.auxiliary_states[&id].is_pose_graph_frame
    }
    fn is_loop_closure_frame(&self, id: StateId) -> bool {
        ViSlamBackend::is_loop_closure_frame(self, id)
    }
    fn is_recent_loop_closure_frame(&self, id: StateId) -> bool {
        ViSlamBackend::is_recent_loop_closure_frame(self, id)
    }
    fn is_place_recognition_frame(&self, id: StateId) -> bool {
        self.auxiliary_states[&id]
            .recognition
            .is_place_recognition_frame
    }
    fn attempt_loop_closure(
        &mut self,
        a: StateId,
        b: StateId,
        relative: Transformation,
        information: SMatrix<f64, 6, 6>,
        drift: f64,
    ) -> Result<Option<bool>> {
        let (accepted, skip) =
            ViSlamBackend::attempt_loop_closure(self, a, b, &relative, information, drift)?;
        Ok(accepted.then_some(skip))
    }
    fn add_loop_closure_frame(&mut self, id: StateId, skip: bool) -> Result<BTreeSet<LandmarkId>> {
        ViSlamBackend::add_loop_closure_frame(self, id, skip)
    }
    fn set_component_alignment(
        &mut self,
        id: StateId,
        component: usize,
        pose: Transformation,
    ) -> Result<()> {
        self.t_ai_s
            .entry(id)
            .or_default()
            .insert(component as u64, pose);
        Ok(())
    }
    fn clean_unobserved_landmarks(&mut self) -> Result<()> {
        ViSlamBackend::clean_unobserved_landmarks(self)?;
        Ok(())
    }
}
impl crate::runtime::threaded_slam::ThreadedGraph for ViSlamBackend {
    type FullGraphResult = SolverUpdate;
    fn add_imu(&mut self, p: &ImuParameters) -> Result<()> {
        if ViSlamBackend::add_imu(self, p.clone()) < 0 {
            return Err(graph_error("only one IMU supported"));
        }
        Ok(())
    }
    fn add_camera(&mut self, p: &CameraParameters) -> Result<()> {
        ViSlamBackend::add_camera(self, p.clone());
        Ok(())
    }
    fn add_gps(&mut self, p: &GpsParameters) -> Result<()> {
        if ViSlamBackend::add_gps(self, p.clone()) < 0 {
            return Err(graph_error("only one GPS supported"));
        }
        Ok(())
    }
    fn set_detector_uniformity_radius(&mut self, r: f64) {
        ViSlamBackend::set_detector_uniformity_radius(self, r);
    }
    fn set_optimisation_time_limit(&mut self, s: f64, m: i32) -> Result<()> {
        ViSlamBackend::set_optimisation_time_limit(self, s, m.max(0) as usize);
        Ok(())
    }
    fn add_states(&mut self, f: MultiFramePtr, m: &ImuMeasurementDeque, k: bool) -> Result<bool> {
        ViSlamBackend::add_states(self, f, m, k)
    }
    fn clear(&mut self) -> Result<()> {
        ViSlamBackend::clear(self);
        Ok(())
    }
    fn set_keyframe(&mut self, id: StateId, k: bool) -> Result<()> {
        ViSlamBackend::set_keyframe(self, id, k)?;
        Ok(())
    }
    fn speed_and_bias(&self, id: StateId) -> Result<SpeedAndBias> {
        ViSlamBackend::speed_and_bias(self, id)
    }
    fn timestamp(&self, id: StateId) -> Result<Timestamp> {
        ViSlamBackend::timestamp(self, id)
    }
    fn optimise_realtime_graph(
        &mut self,
        iterations: i32,
        threads: i32,
        initialized: bool,
    ) -> Result<Vec<StateId>> {
        ViSlamBackend::optimise_realtime_graph(
            self,
            iterations.max(0) as usize,
            threads.max(0) as usize,
            false,
            false,
            initialized,
        )
    }
    fn prepare_full_graph_task(
        &mut self,
        iterations: i32,
        threads: i32,
    ) -> Result<Box<dyn FnOnce() -> Result<SolverUpdate> + Send>> {
        ViSlamBackend::prepare_full_graph_task(
            self,
            iterations.max(0) as usize,
            threads.max(0) as usize,
        )
    }
    fn finish_full_graph_task(&mut self, result: SolverUpdate) -> Result<()> {
        ViSlamBackend::finish_full_graph_task(self, result)
    }
    fn synchronise_realtime_and_full_graph(&mut self) -> Result<Vec<StateId>> {
        ViSlamBackend::synchronise_realtime_and_full_graph(self)
    }
    fn add_gps_measurements(
        &mut self,
        gps: &GpsMeasurementDeque,
        imu: &ImuMeasurementDeque,
    ) -> Result<()> {
        self.add_gps_measurements_on_all_graphs(&mut gps.clone(), imu)?;
        Ok(())
    }
    fn observed_ids(&self, id: StateId) -> BTreeSet<StateId> {
        self.get_observed_ids(id).expect("observed state exists")
    }
    fn most_overlapped_state_id(&self, id: StateId) -> StateId {
        ViSlamBackend::most_overlapped_state_id(self, id, false)
            .expect("overlap raster configured")
            .unwrap_or_default()
    }
    fn closed_loop(&self, id: StateId) -> bool {
        self.auxiliary_states[&id].recognition.closed_loop
    }
    fn component_alignments(&self, id: StateId) -> BTreeMap<u64, Transformation> {
        self.t_ai_s.get(&id).cloned().unwrap_or_default()
    }
    fn gps_points(&self, id: StateId) -> Vec<Vector3<f64>> {
        self.realtime_graph
            .gps_measurements(id)
            .expect("GPS state exists")
    }
    fn t_gw(&self) -> Transformation {
        self.realtime_graph
            .t_gw(None)
            .expect("GPS transform exists")
    }
    fn apply_strategy(&mut self, k: usize, l: usize, i: usize) -> Result<StrategyResult> {
        ViSlamBackend::apply_strategy(self, k, l, i, true)
    }
    fn do_final_ba(
        &mut self,
        iterations: i32,
        r: f64,
        a: f64,
        threads: i32,
    ) -> Result<Vec<StateId>> {
        Ok(ViSlamBackend::do_final_ba(
            self,
            iterations.max(0) as usize,
            r,
            a,
            threads.max(0) as usize,
            false,
        )?
        .updated_states
        .into_iter()
        .collect())
    }
    fn write_final_csv_trajectory(&self, path: &Path, rpg: bool) -> Result<()> {
        ViSlamBackend::write_final_csv_trajectory(self, path, rpg)?;
        Ok(())
    }
    fn write_global_csv_trajectory(&self, path: &Path) -> Result<()> {
        ViSlamBackend::write_global_csv_trajectory(self, path)?;
        Ok(())
    }
    fn save_map(&mut self, path: &Path) -> Result<bool> {
        ViSlamBackend::save_map(self, path)
    }
    fn dump_gps_residuals(&mut self, path: &Path) -> Result<()> {
        ViSlamBackend::dump_gps_residuals(self, path)
    }
    fn write_debug_statistics_csv(&self, prefix: &Path) -> Result<()> {
        ViSlamBackend::write_lidar_debug_statistics_csv(self, &prefix.to_string_lossy())
    }
}

struct LoopPath {
    first_loop: StateId,
    steps: i32,
    distances: Vec<f64>,
    travelled: f64,
    travelled2: f64,
    displacement: Vector3<f64>,
}
