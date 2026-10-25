/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Visual inertial graph storage, factor registration, GPS initialization and optimisation.
use crate::{
    cameras::n_camera_system::{DistortionType, NCameraSystem},
    ceres::{
        component::{ComponentLandmark, ComponentObservation, ComponentState},
        depth_error::OneSidedDepthError,
        error_interface::ErrorEvaluation,
        gps_error_asynchronous::GpsErrorAsynchronous,
        homogeneous_point_parameter_block::HomogeneousPointParameterBlock,
        imu_error::{ImuError, PseudoImuError},
        parameter_block::ParameterBlock,
        pose_error::PoseError,
        pose_parameter_block::PoseParameterBlock,
        relative_pose_error::RelativePoseError,
        reprojection_error::ReprojectionError,
        reprojection_error_base::ReprojectionError2dBase,
        speed_and_bias_error::SpeedAndBiasError,
        speed_and_bias_parameter_block::SpeedAndBiasParameterBlock,
        submap_icp_error::OccupancySubmap,
        two_pose_extrinsics_graph_error::TwoPoseExtrinsicsGraphError,
        two_pose_graph_error::{
            LossFunction, SharedPoint, SharedPose, TwoPoseGraphErrorConst,
            TwoPoseStandardGraphError,
        },
    },
    config::parameters::{CameraParameters, GpsParameters, ImuParameters},
    frames::{
        frame_typedefs::{
            KeypointIdentifier, LandmarkId, MapPoint2, MapPoints, SpeedAndBias, StateId,
        },
        multi_frame::MultiFrame,
    },
    kinematics::{
        operators::cross_mx,
        transformation::{
            Transformation, TransformationCacheless, quaternion_from_rotation_matrix,
            quaternion_inverse,
        },
    },
    sensors::measurements::{GpsMeasurement, GpsMeasurementDeque, ImuMeasurementDeque},
};
use firefly_error::{Error, ErrorKind, Result};
use jiff::Timestamp;
use nalgebra::{
    Matrix2, Matrix3, Matrix4, Quaternion, SMatrix, SVector, Vector2, Vector3, Vector4,
};
use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet, VecDeque},
    io::Write,
    path::Path,
    rc::Rc,
};

pub type ResidualId = u64;
pub type SharedSpeedBias = Rc<RefCell<SpeedAndBiasParameterBlock>>;
pub type SharedReprojection = Rc<RefCell<Box<dyn ReprojectionError2dBase>>>;
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum GpsStatus {
    #[default]
    Off,
    Idle,
    Initialising,
    Initialised,
    ReInitialising,
}
#[derive(Clone, Copy, Debug)]
pub enum ManifoldKind {
    Pose,
    Pose4d,
    HomogeneousPoint,
    Euclidean,
}
#[derive(Clone)]
pub enum Parameter {
    Pose(SharedPose),
    SpeedBias(SharedSpeedBias),
    Point(SharedPoint),
}
impl Parameter {
    #[must_use]
    pub fn key(&self) -> usize {
        match self {
            Self::Pose(p) => Rc::as_ptr(p) as usize,
            Self::SpeedBias(p) => Rc::as_ptr(p) as usize,
            Self::Point(p) => Rc::as_ptr(p) as usize,
        }
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn values(&self) -> Result<Vec<f64>> {
        match self {
            Self::Pose(p) => p.borrow().parameters().map(<[f64]>::to_vec),
            Self::SpeedBias(p) => p.borrow().parameters().map(<[f64]>::to_vec),
            Self::Point(p) => p.borrow().parameters().map(<[f64]>::to_vec),
        }
        .ok_or_else(|| graph_error("parameter estimate absent"))
    }
    #[must_use]
    pub fn fixed(&self) -> bool {
        match self {
            Self::Pose(p) => p.borrow().fixed(),
            Self::SpeedBias(p) => p.borrow().fixed(),
            Self::Point(p) => p.borrow().fixed(),
        }
    }
    pub fn set_fixed(&self, fixed: bool) {
        match self {
            Self::Pose(p) => p.borrow_mut().set_fixed(fixed),
            Self::SpeedBias(p) => p.borrow_mut().set_fixed(fixed),
            Self::Point(p) => p.borrow_mut().set_fixed(fixed),
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub enum RobustLoss {
    Cauchy(f64),
    Tukey(f64),
    Huber(f64),
}
impl LossFunction for RobustLoss {
    fn evaluate(&self, squared_residual_norm: f64) -> [f64; 3] {
        match *self {
            Self::Cauchy(scale) => {
                let scale_squared = scale * scale;
                let inverse_scale_squared = 1.0 / scale_squared;
                let normalized_error = 1.0 + squared_residual_norm * inverse_scale_squared;
                let inverse_error = 1.0 / normalized_error;
                [
                    scale_squared * normalized_error.ln(),
                    inverse_error.max(f64::MIN_POSITIVE),
                    -inverse_scale_squared * (inverse_error * inverse_error),
                ]
            }
            Self::Huber(scale) => {
                if squared_residual_norm > scale * scale {
                    let residual_norm = squared_residual_norm.sqrt();
                    let derivative = (scale / residual_norm).max(f64::MIN_POSITIVE);
                    [
                        2.0 * scale * residual_norm - scale * scale,
                        derivative,
                        -derivative / (2.0 * squared_residual_norm),
                    ]
                } else {
                    [squared_residual_norm, 1.0, 0.0]
                }
            }
            Self::Tukey(scale) => {
                let scale_squared = scale * scale;
                if squared_residual_norm <= scale_squared {
                    let normalized_error = 1.0 - squared_residual_norm / scale_squared;
                    [
                        scale_squared / 3.0
                            * (1.0 - normalized_error * normalized_error * normalized_error),
                        normalized_error * normalized_error,
                        -2.0 * normalized_error / scale_squared,
                    ]
                } else {
                    [scale_squared / 3.0, 0.0, 0.0]
                }
            }
        }
    }
}
pub enum VisualFactor {
    Standard(TwoPoseStandardGraphError),
    Extrinsics(TwoPoseExtrinsicsGraphError),
}
impl VisualFactor {
    #[must_use]
    pub fn base(&self) -> &crate::ceres::two_pose_graph_error::TwoPoseGraphError {
        match self {
            Self::Standard(f) => &f.base,
            Self::Extrinsics(f) => &f.base,
        }
    }
    pub fn base_mut(&mut self) -> &mut crate::ceres::two_pose_graph_error::TwoPoseGraphError {
        match self {
            Self::Standard(f) => &mut f.base,
            Self::Extrinsics(f) => &mut f.base,
        }
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn compute(&mut self) -> Result<bool> {
        match self {
            Self::Standard(f) => f.compute(),
            Self::Extrinsics(f) => f.compute(),
        }
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn convert(
        &mut self,
        obs: &mut Vec<crate::ceres::two_pose_graph_error::Observation>,
        dup: &mut Vec<KeypointIdentifier>,
    ) -> Result<bool> {
        match self {
            Self::Standard(f) => f.convert_to_reprojection_errors(obs, dup),
            Self::Extrinsics(f) => f.convert_to_reprojection_errors(obs, dup),
        }
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn clone_const(&self) -> Result<Rc<dyn TwoPoseGraphErrorConst>> {
        Ok(match self {
            Self::Standard(f) => Rc::new(f.clone_two_pose_graph_error_const()),
            Self::Extrinsics(f) => Rc::new(
                f.clone_two_pose_graph_error_const()
                    .ok_or_else(|| graph_error("extrinsics factor not computed"))?,
            ),
        })
    }
}
/// Concrete factors retain their shared estimate blocks in `Residual::parameters`.
pub enum GraphFactor {
    Reprojection(SharedReprojection),
    Depth(OneSidedDepthError),
    Imu(Rc<RefCell<ImuError>>),
    PseudoImu(Rc<RefCell<PseudoImuError>>),
    PosePrior(Box<PoseError>),
    SpeedBiasPrior(Box<SpeedAndBiasError>),
    RelativePose(Box<RelativePoseError>),
    Gps(Rc<RefCell<GpsErrorAsynchronous>>),
    Visual(Rc<RefCell<VisualFactor>>),
    VisualConst(Rc<dyn TwoPoseGraphErrorConst>),
    Submap {
        map: Rc<dyn OccupancySubmap>,
        point: Vector3<f64>,
        sigma: f64,
    },
}
pub struct Residual {
    pub factor: Rc<GraphFactor>,
    pub parameters: Vec<Parameter>,
    pub loss: Option<RobustLoss>,
}
#[derive(Default)]
pub struct GraphProblem {
    pub parameters: BTreeMap<usize, (Parameter, ManifoldKind)>,
    pub residuals: BTreeMap<ResidualId, Residual>,
    next_residual: ResidualId,
}
impl GraphProblem {
    pub fn register(&mut self, p: Parameter, m: ManifoldKind) {
        self.parameters.entry(p.key()).or_insert((p, m));
    }
    pub fn add(
        &mut self,
        f: GraphFactor,
        p: Vec<Parameter>,
        loss: Option<RobustLoss>,
    ) -> ResidualId {
        for parameter in &p {
            self.parameters
                .entry(parameter.key())
                .or_insert_with(|| (parameter.clone(), ManifoldKind::Euclidean));
        }
        self.next_residual += 1;
        let id = self.next_residual;
        self.residuals.insert(
            id,
            Residual {
                factor: Rc::new(f),
                parameters: p,
                loss,
            },
        );
        id
    }
    pub fn remove_parameter(&mut self, p: &Parameter) {
        let key = p.key();
        self.parameters.remove(&key);
        self.residuals
            .retain(|_, r| !r.parameters.iter().any(|p| p.key() == key));
    }
}
#[derive(Clone, Debug)]
pub struct SolverOptions {
    pub max_iterations: usize,
    pub function_tolerance: f64,
    pub num_threads: usize,
    pub verbose: bool,
    pub time_limit: Option<f64>,
    pub minimum_iterations: usize,
    pub linear_solver: &'static str,
    pub trust_region: &'static str,
}
impl Default for SolverOptions {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            function_tolerance: 1.0e-6,
            num_threads: 1,
            verbose: false,
            time_limit: None,
            minimum_iterations: 0,
            linear_solver: "SPARSE_NORMAL_CHOLESKY",
            trust_region: "DOGLEG",
        }
    }
}
#[derive(Clone, Debug, Default)]
pub struct SolverSummary {
    pub iterations: usize,
    pub initial_cost: Option<f64>,
    pub final_cost: Option<f64>,
    pub successful: bool,
    pub report: String,
}
/// The Ceres engine consumes the complete registered graph and updates shared blocks.
/// The adapter must honor each manifold, fixed block, loss and stopping condition.
pub struct SolverStage {
    pub options: SolverOptions,
    pub disabled_residuals: BTreeSet<ResidualId>,
}
pub enum SolverFactorCache {
    Imu(Box<ImuError>),
    Gps(Box<GpsErrorAsynchronous>),
}
pub struct SolverUpdate {
    pub caches: BTreeMap<ResidualId, SolverFactorCache>,
    pub parameters: BTreeMap<usize, Vec<f64>>,
    pub summary: SolverSummary,
}
pub trait GraphSolver {
    /// Detaches numerical solver data; the returned job must own all values and
    /// create thread-confined solver resources without capturing graph Rc blocks.
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn prepare_solve(
        &mut self,
        problem: &GraphProblem,
        stages: Vec<SolverStage>,
    ) -> Result<Box<dyn FnOnce() -> Result<SolverUpdate> + Send>>;

    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn solve(
        &mut self,
        problem: &mut GraphProblem,
        options: &SolverOptions,
    ) -> Result<SolverSummary>;
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn refine_alignment_4d(
        &mut self,
        gps: &[Vector3<f64>],
        world: &[Vector3<f64>],
        initial: &Transformation,
        iterations: usize,
        cauchy_scale: f64,
    ) -> Result<Transformation>;
}
/// `GeographicLib` `LocalCartesian` operations with WGS84 geocentric coordinates.
pub trait LocalCartesian {
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn reset(&mut self, latitude: f64, longitude: f64, height: f64) -> Result<()>;
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn forward(&self, latitude: f64, longitude: f64, height: f64) -> Result<Vector3<f64>>;
}
#[derive(Clone, Default)]
pub struct GraphState {
    pub is_keyframe: bool,
    pub t_gw: Option<SharedPose>,
    pub gps_mode: Option<GpsStatus>,
    pub previous_imu: Option<ResidualId>,
    pub next_imu: Option<ResidualId>,
    pub previous_extrinsics: Vec<ResidualId>,
    pub next_extrinsics: Vec<ResidualId>,
    pub pose_prior: Option<ResidualId>,
    pub speed_bias_prior: Option<ResidualId>,
    pub extrinsics_priors: Vec<ResidualId>,
    pub relative_pose_links: BTreeMap<StateId, ResidualId>,
    pub two_pose_links: BTreeMap<StateId, ResidualId>,
    pub two_pose_const_links: BTreeMap<StateId, ResidualId>,
    pub gps_factors: Vec<(Option<ResidualId>, Rc<RefCell<GpsErrorAsynchronous>>)>,
    pub map_res_ids: Vec<ResidualId>,
    pub submap_reference_links: Vec<ResidualId>,
    pub submap_links: Vec<ResidualId>,
}
#[derive(Clone, Default)]
pub struct GraphLandmark {
    pub observations: BTreeSet<KeypointIdentifier>,
    pub classification: i32,
}
#[derive(Clone)]
pub struct GraphObservation {
    pub core: ComponentObservation,
    pub residual: ResidualId,
    pub depth_error: Option<ResidualId>,
}
#[derive(Clone)]
pub struct AnyState {
    pub keyframe_id: Option<StateId>,
    pub timestamp: Timestamp,
    pub t_sk_s: Transformation,
    pub v_sk: Vector3<f64>,
}
#[derive(Clone, Debug)]
pub struct LidarDebugInfo {
    pub frame_id: u64,
    pub reference_id: u64,
    pub n_residuals: usize,
    pub n_nonzero_occ: usize,
    pub n_nonzero_grad: usize,
    pub mean_res: f64,
    pub max_residual: f64,
    pub n_big_residuals: usize,
    pub mean_field: f64,
    pub min_field: f64,
    pub max_field: f64,
    pub mean_grad: f64,
    pub max_grad: f64,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct GpsFlags {
    pub gps_observability: bool,
    pub gps_fixed: bool,
    pub gps_reinitialised: bool,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct GpsAlignmentRequests {
    pub needs_initial_alignment: bool,
    pub needs_position_alignment: bool,
    pub needs_full_alignment: bool,
}
#[derive(Default)]
pub struct ViGraph {
    pub camera_parameters: Vec<CameraParameters>,
    pub imu_parameters: Vec<ImuParameters>,
    pub gps_parameters: Vec<GpsParameters>,
    pub states: BTreeMap<StateId, ComponentState>,
    pub state_info: BTreeMap<StateId, GraphState>,
    pub landmarks: BTreeMap<LandmarkId, ComponentLandmark>,
    pub landmark_info: BTreeMap<LandmarkId, GraphLandmark>,
    pub observations: BTreeMap<KeypointIdentifier, GraphObservation>,
    pub problem: GraphProblem,
    pub solver: Option<Box<dyn GraphSolver>>,
    pub options: SolverOptions,
    pub summary: SolverSummary,
    pub gps_status: GpsStatus,
    pub gps_flags: GpsFlags,

    pub t_gw_init: Option<Transformation>,
    pub gps_states: BTreeSet<StateId>,
    pub gps_init_map: BTreeMap<StateId, GpsMeasurement>,
    pub gps_init_imu_queue: ImuMeasurementDeque,
    pub alignment_requests: GpsAlignmentRequests,

    pub gps_dropout_id: Option<StateId>,
    pub position_aligned_id: Option<StateId>,
    pub gps_reinit_states: BTreeSet<StateId>,

    pub co_observation_counts: BTreeMap<u64, BTreeMap<u64, i32>>,
    pub covisibilities_computed: bool,
    pub visible_frames: BTreeSet<StateId>,
    pub any_state: BTreeMap<StateId, AnyState>,
    pub local_cartesian: Option<Rc<RefCell<dyn LocalCartesian>>>,
    pub lidar_debug_infos: Vec<LidarDebugInfo>,
    pub debug_lidar_residuals: bool,
}
impl ViGraph {
    #[must_use]
    pub fn new() -> Self {
        Self {
            covisibilities_computed: true,
            solver: Some(Box::new(RustGraphSolver)),
            ..Self::default()
        }
    }
    pub fn add_camera(&mut self, p: CameraParameters) -> usize {
        self.camera_parameters.push(p);
        self.camera_parameters.len() - 1
    }
    pub fn add_imu(&mut self, p: ImuParameters) -> i32 {
        if self.imu_parameters.len() > 1 {
            return -1;
        }
        self.imu_parameters.push(p);
        self.imu_parameters.len() as i32 - 1
    }
    pub fn add_gps(&mut self, p: GpsParameters) -> i32 {
        if self.gps_parameters.len() > 1 {
            return -1;
        }
        self.gps_parameters.push(p);
        self.gps_parameters.len() as i32 - 1
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn pose_block(&self, id: StateId) -> Result<SharedPose> {
        self.states
            .get(&id)
            .and_then(|s| s.pose.clone())
            .ok_or_else(|| graph_error("state pose absent"))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn speed_bias_block(&self, id: StateId) -> Result<SharedSpeedBias> {
        self.states
            .get(&id)
            .and_then(|s| s.speed_and_bias.clone())
            .ok_or_else(|| graph_error("state speed/bias absent"))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn extrinsics_block(&self, id: StateId, camera: usize) -> Result<SharedPose> {
        self.states
            .get(&id)
            .and_then(|s| s.extrinsics.get(camera))
            .and_then(Clone::clone)
            .ok_or_else(|| graph_error("state extrinsics absent"))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn pose(&self, id: StateId) -> Result<Transformation> {
        Ok(Transformation::from_other(
            self.pose_block(id)?
                .borrow()
                .estimate()
                .ok_or_else(|| graph_error("pose estimate absent"))?,
        ))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn speed_and_bias(&self, id: StateId) -> Result<SpeedAndBias> {
        self.speed_bias_block(id)?
            .borrow()
            .estimate()
            .copied()
            .ok_or_else(|| graph_error("speed/bias estimate absent"))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn extrinsics(&self, id: StateId, camera: usize) -> Result<Transformation> {
        Ok(Transformation::from_other(
            self.extrinsics_block(id, camera)?
                .borrow()
                .estimate()
                .ok_or_else(|| graph_error("extrinsics estimate absent"))?,
        ))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn t_gw(&self, id: Option<StateId>) -> Result<Transformation> {
        let id = id
            .or_else(|| self.states.keys().next().copied())
            .ok_or_else(|| graph_error("graph empty"))?;
        let p = self
            .state_info
            .get(&id)
            .and_then(|s| s.t_gw.as_ref())
            .ok_or_else(|| graph_error("GPS transform absent"))?;
        Ok(Transformation::from_other(
            p.borrow()
                .estimate()
                .ok_or_else(|| graph_error("GPS estimate absent"))?,
        ))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn timestamp(&self, id: StateId) -> Result<Timestamp> {
        self.states
            .get(&id)
            .and_then(|s| s.timestamp)
            .ok_or_else(|| graph_error("timestamp absent"))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn current_state_id(&self) -> Result<StateId> {
        self.states
            .keys()
            .next_back()
            .copied()
            .ok_or_else(|| graph_error("no states"))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn state_id_by_age(&self, age: usize) -> Result<StateId> {
        self.states
            .keys()
            .rev()
            .nth(age)
            .copied()
            .ok_or_else(|| graph_error("too few states"))
    }
    #[must_use]
    pub fn is_keyframe(&self, id: StateId) -> bool {
        self.state_info.get(&id).is_some_and(|s| s.is_keyframe)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_keyframe(&mut self, id: StateId, value: bool) -> Result<bool> {
        self.state_info
            .get_mut(&id)
            .ok_or_else(|| graph_error("state absent"))?
            .is_keyframe = value;
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_pose(&self, id: StateId, p: &Transformation) -> Result<bool> {
        self.pose_block(id)?
            .borrow_mut()
            .set_estimate(TransformationCacheless::from_other(p));
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_speed_and_bias(&self, id: StateId, sb: SpeedAndBias) -> Result<bool> {
        self.speed_bias_block(id)?.borrow_mut().set_estimate(sb);
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_extrinsics(&self, id: StateId, camera: usize, p: &Transformation) -> Result<bool> {
        self.extrinsics_block(id, camera)?
            .borrow_mut()
            .set_estimate(TransformationCacheless::from_other(p));
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn add_states_initialise(
        &mut self,
        timestamp: Timestamp,
        imu: &ImuMeasurementDeque,
        rig: &NCameraSystem,
    ) -> Result<StateId> {
        if !self.states.is_empty() {
            return Err(graph_error("states already added"));
        }
        let params = self
            .imu_parameters
            .first()
            .ok_or_else(|| graph_error("IMU parameters absent"))?
            .clone();
        let mut pose = Transformation::identity();
        if params.r#use {
            if !ImuError::init_pose(imu, &mut pose) {
                return Err(graph_error("no IMU measurements"));
            }
        } else {
            let t_wc = Transformation::new(
                Vector3::zeros(),
                Quaternion::new(-2.0_f64.sqrt(), 2.0_f64.sqrt(), 0.0, 0.0),
            );
            pose = t_wc.compose(&rig.t_sc(0).inverse());
        }
        let mut sb = SpeedAndBias::zeros();
        sb.fixed_rows_mut::<3>(3).copy_from(&params.g0);
        sb.fixed_rows_mut::<3>(6).copy_from(&params.a0);
        let id = StateId::new(1);
        let p = Rc::new(RefCell::new(PoseParameterBlock::new(&pose, 1, timestamp)));
        let s = Rc::new(RefCell::new(SpeedAndBiasParameterBlock::new(
            sb, 1, timestamp,
        )));
        self.problem
            .register(Parameter::Pose(p.clone()), ManifoldKind::Pose);
        self.problem
            .register(Parameter::SpeedBias(s.clone()), ManifoldKind::Euclidean);
        let mut state = ComponentState {
            timestamp: Some(timestamp),
            pose: Some(p.clone()),
            speed_and_bias: Some(s.clone()),
            ..ComponentState::default()
        };
        let mut meta = GraphState {
            is_keyframe: true,
            ..GraphState::default()
        };
        for i in 0..self.camera_parameters.len() {
            let ext = Rc::new(RefCell::new(PoseParameterBlock::new(
                rig.t_sc(i).as_ref(),
                1,
                timestamp,
            )));
            self.problem
                .register(Parameter::Pose(ext.clone()), ManifoldKind::Pose);
            state.extrinsics.push(Some(ext));
        }
        let global = Rc::new(RefCell::new(PoseParameterBlock::new(
            &Transformation::identity(),
            1,
            timestamp,
        )));
        self.problem
            .register(Parameter::Pose(global.clone()), ManifoldKind::Pose4d);
        meta.t_gw = Some(global);
        let diag = SVector::<f64, 6>::from_row_slice(&[
            1.0e8,
            1.0e8,
            1.0e8,
            if params.r#use { 0.0 } else { 1.0e2 },
            if params.r#use { 0.0 } else { 1.0e2 },
            1.0e2,
        ]);
        meta.pose_prior = Some(self.problem.add(
            GraphFactor::PosePrior(Box::new(PoseError::from_information_diagonal(
                pose_isometry(&pose),
                diag,
            ))),
            vec![Parameter::Pose(p)],
            None,
        ));
        meta.speed_bias_prior = Some(self.problem.add(
            GraphFactor::SpeedBiasPrior(Box::new(SpeedAndBiasError::from_variances(
                sb,
                0.1,
                params.sigma_bg.powi(2),
                params.sigma_ba.powi(2),
            ))),
            vec![Parameter::SpeedBias(s)],
            None,
        ));
        self.initialise_extrinsics_priors(&state, &mut meta);
        self.states.insert(id, state);
        self.state_info.insert(id, meta);
        self.any_state.insert(
            id,
            AnyState {
                timestamp,
                keyframe_id: None,
                t_sk_s: Transformation::identity(),
                v_sk: Vector3::zeros(),
            },
        );
        Ok(id)
    }
    fn initialise_extrinsics_priors(&mut self, state: &ComponentState, meta: &mut GraphState) {
        for (i, camera) in self.camera_parameters.iter().enumerate() {
            let ext = state.extrinsics[i]
                .as_ref()
                .expect("extrinsics assigned")
                .clone();
            if camera.online_calibration.do_extrinsics {
                let c = &camera.online_calibration;
                let estimate =
                    Transformation::from_other(ext.borrow().estimate().expect("estimate assigned"));
                meta.extrinsics_priors.push(self.problem.add(
                    GraphFactor::PosePrior(Box::new(PoseError::from_variances(
                        pose_isometry(&estimate),
                        c.sigma_r.powi(2),
                        c.sigma_alpha.powi(2),
                    ))),
                    vec![Parameter::Pose(ext)],
                    None,
                ));
            } else {
                ext.borrow_mut().set_fixed(true);
            }
        }
    }

    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn add_states_propagate(
        &mut self,
        timestamp: Timestamp,
        imu: &ImuMeasurementDeque,
        is_keyframe: bool,
    ) -> Result<StateId> {
        let previous = self.current_state_id()?;
        let id = previous + 1;
        let t0 = self.timestamp(previous)?;
        let mut pose = self.pose(previous)?;
        let mut sb = self.speed_and_bias(previous)?;
        let params = self
            .imu_parameters
            .first()
            .ok_or_else(|| graph_error("IMU absent"))?
            .clone();
        if params.r#use {
            ImuError::propagation(
                imu,
                &params,
                &mut pose,
                &mut sb,
                (t0, timestamp),
                (None, None),
            )
            .map_err(|e| graph_error(&format!("IMU propagation: {e:?}")))?;
        } else if let Some(&older) = self.states.keys().rev().nth(1) {
            let r = seconds(timestamp, t0) / seconds(t0, self.timestamp(older)?);
            let old = self.pose(older)?;
            let relative = pose.quaternion() * quaternion_inverse(&old.quaternion());
            let angle = quaternion_angle_axis(&relative) * r;
            let dr = (pose.translation() - old.translation()) * r;
            pose.set(
                pose.translation() + dr,
                pose.quaternion() * crate::kinematics::transformation::delta_q(&angle),
            );
        }
        let p = Rc::new(RefCell::new(PoseParameterBlock::new(
            &pose,
            id.value(),
            timestamp,
        )));
        let s = Rc::new(RefCell::new(SpeedAndBiasParameterBlock::new(
            sb,
            id.value(),
            timestamp,
        )));
        self.problem
            .register(Parameter::Pose(p.clone()), ManifoldKind::Pose);
        self.problem
            .register(Parameter::SpeedBias(s.clone()), ManifoldKind::Euclidean);
        let core = ComponentState {
            timestamp: Some(timestamp),
            pose: Some(p.clone()),
            speed_and_bias: Some(s.clone()),
            extrinsics: self.states[&previous].extrinsics.clone(),
            ..ComponentState::default()
        };
        let meta = GraphState {
            is_keyframe,
            t_gw: self.state_info[&previous].t_gw.clone(),
            ..GraphState::default()
        };
        self.link_propagated_state(
            (previous, id),
            (t0, timestamp),
            params,
            imu,
            (core, meta),
            (p, s),
        )
    }
    fn link_propagated_state(
        &mut self,
        (previous, id): (StateId, StateId),
        (t0, timestamp): (Timestamp, Timestamp),
        params: ImuParameters,
        imu: &ImuMeasurementDeque,
        (mut core, mut meta): (ComponentState, GraphState),
        (p, s): (SharedPose, SharedSpeedBias),
    ) -> Result<StateId> {
        let factor = if params.r#use {
            if imu.front().is_none_or(|m| t0 <= m.time_stamp)
                || imu.back().is_none_or(|m| timestamp >= m.time_stamp)
            {
                return Err(graph_error("strict IMU time coverage required"));
            }
            let f = Rc::new(RefCell::new(
                ImuError::new(imu.clone(), params, t0, timestamp)
                    .map_err(|e| graph_error(&format!("IMU: {e:?}")))?,
            ));
            core.previous_imu_link = Some(Rc::new(f.borrow().clone()));
            self.states
                .get_mut(&previous)
                .expect("previous exists")
                .next_imu_link
                .clone_from(&core.previous_imu_link);
            GraphFactor::Imu(f)
        } else {
            GraphFactor::PseudoImu(Rc::new(RefCell::new(PseudoImuError::new(t0, timestamp))))
        };
        let rid = self.problem.add(
            factor,
            vec![
                Parameter::Pose(self.pose_block(previous)?),
                Parameter::SpeedBias(self.speed_bias_block(previous)?),
                Parameter::Pose(p),
                Parameter::SpeedBias(s),
            ],
            None,
        );
        self.state_info
            .get_mut(&previous)
            .expect("previous exists")
            .next_imu = Some(rid);
        meta.previous_imu = Some(rid);
        self.states.insert(id, core);
        self.state_info.insert(id, meta);
        self.any_state.insert(
            id,
            AnyState {
                timestamp,
                keyframe_id: None,
                t_sk_s: Transformation::identity(),
                v_sk: Vector3::zeros(),
            },
        );
        Ok(id)
    }

    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn add_states_from_other(&mut self, id: StateId, other: &ViGraph) -> Result<bool> {
        let source = other
            .states
            .get(&id)
            .ok_or_else(|| graph_error("source state absent"))?;
        let timestamp = other.timestamp(id)?;
        if !other.state_info[&id].extrinsics_priors.is_empty() {
            return Err(graph_error(
                "source extrinsics-prior copy indexes the empty destination vector",
            ));
        }
        let p = Rc::new(RefCell::new(PoseParameterBlock::new(
            &other.pose(id)?,
            id.value(),
            timestamp,
        )));
        let sb = Rc::new(RefCell::new(SpeedAndBiasParameterBlock::new(
            other.speed_and_bias(id)?,
            id.value(),
            timestamp,
        )));
        let mut core = ComponentState {
            timestamp: Some(timestamp),
            pose: Some(p.clone()),
            speed_and_bias: Some(sb.clone()),
            ..ComponentState::default()
        };
        for i in 0..source.extrinsics.len() {
            let ext = Rc::new(RefCell::new(PoseParameterBlock::new(
                &other.extrinsics(id, i)?,
                id.value(),
                timestamp,
            )));
            core.extrinsics.push(Some(ext));
        }
        let info = &other.state_info[&id];
        let mut meta = GraphState {
            is_keyframe: info.is_keyframe,
            t_gw: info.t_gw.clone(),
            ..GraphState::default()
        };
        if let Some(rid) = info.pose_prior
            && let GraphFactor::PosePrior(f) = other.problem.residuals[&rid].factor.as_ref()
        {
            let information = *f
                .information()
                .ok_or_else(|| graph_error("prior information absent"))?;
            meta.pose_prior = Some(self.problem.add(
                GraphFactor::PosePrior(Box::new(PoseError::new(*f.measurement(), information))),
                vec![Parameter::Pose(p.clone())],
                None,
            ));
        }
        if let Some(rid) = info.speed_bias_prior
            && let GraphFactor::SpeedBiasPrior(f) = other.problem.residuals[&rid].factor.as_ref()
        {
            meta.speed_bias_prior = Some(self.problem.add(
                GraphFactor::SpeedBiasPrior(Box::new(SpeedAndBiasError::new(
                    *f.measurement(),
                    *f.information(),
                ))),
                vec![Parameter::SpeedBias(sb.clone())],
                None,
            ));
        }
        self.copy_state_links(id, other, &mut core, &mut meta, (p, sb))?;
        self.states.insert(id, core);
        self.state_info.insert(id, meta);
        self.any_state.insert(
            id,
            AnyState {
                timestamp,
                keyframe_id: None,
                t_sk_s: Transformation::identity(),
                v_sk: Vector3::zeros(),
            },
        );
        Ok(true)
    }
    fn copy_state_links(
        &mut self,
        id: StateId,
        other: &ViGraph,
        core: &mut ComponentState,
        meta: &mut GraphState,
        (p, sb): (SharedPose, SharedSpeedBias),
    ) -> Result<()> {
        let info = &other.state_info[&id];
        if let Some(rid) = info.previous_imu {
            let previous = self.current_state_id()?;
            let other_previous = other
                .states
                .range(..id)
                .next_back()
                .map(|(&id, _)| id)
                .ok_or_else(|| graph_error("source previous absent"))?;
            if previous != other_previous
                || self.timestamp(previous)? != other.timestamp(previous)?
            {
                return Err(graph_error("inconsistent previous state"));
            }
            let factor = match other.problem.residuals[&rid].factor.as_ref() {
                GraphFactor::Imu(f) => {
                    let copy = f.borrow().clone();
                    core.previous_imu_link = Some(Rc::new(copy.clone()));
                    GraphFactor::Imu(Rc::new(RefCell::new(copy)))
                }
                GraphFactor::PseudoImu(f) => {
                    GraphFactor::PseudoImu(Rc::new(RefCell::new(f.borrow().clone())))
                }
                _ => return Err(graph_error("invalid IMU factor")),
            };
            let new_id = self.problem.add(
                factor,
                vec![
                    Parameter::Pose(self.pose_block(previous)?),
                    Parameter::SpeedBias(self.speed_bias_block(previous)?),
                    Parameter::Pose(p),
                    Parameter::SpeedBias(sb),
                ],
                None,
            );
            meta.previous_imu = Some(new_id);
            self.state_info
                .get_mut(&previous)
                .expect("previous exists")
                .next_imu = Some(new_id);
            self.states
                .get_mut(&previous)
                .expect("previous exists")
                .next_imu_link
                .clone_from(&core.previous_imu_link);
        }
        if !info.previous_extrinsics.is_empty() {
            let previous = self.current_state_id()?;
            for (camera, &rid) in info.previous_extrinsics.iter().enumerate() {
                let GraphFactor::RelativePose(factor) =
                    other.problem.residuals[&rid].factor.as_ref()
                else {
                    return Err(graph_error("extrinsics link factor invalid"));
                };
                let parameters = vec![
                    Parameter::Pose(self.extrinsics_block(previous, camera)?),
                    Parameter::Pose(
                        core.extrinsics
                            .get(camera)
                            .and_then(Clone::clone)
                            .ok_or_else(|| graph_error("copied extrinsics absent"))?,
                    ),
                ];
                let copied = self.problem.add(
                    GraphFactor::RelativePose(Box::new(RelativePoseError::new(
                        *factor.information(),
                        nalgebra::Isometry3::identity(),
                    ))),
                    parameters,
                    None,
                );
                meta.previous_extrinsics.push(copied);
                self.state_info
                    .get_mut(&previous)
                    .expect("previous state exists")
                    .next_extrinsics
                    .push(copied);
            }
        }
        Ok(())
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
        if !id.is_initialised() || self.landmarks.contains_key(&id) {
            return Err(graph_error("invalid or duplicate landmark"));
        }
        let p = Rc::new(RefCell::new(HomogeneousPointParameterBlock::new(
            point,
            id.value(),
            initialised,
        )));
        self.problem
            .register(Parameter::Point(p.clone()), ManifoldKind::HomogeneousPoint);
        self.landmarks.insert(
            id,
            ComponentLandmark {
                h_point: p,
                quality: 0.0,
            },
        );
        self.landmark_info.insert(
            id,
            GraphLandmark {
                classification: -1,
                ..GraphLandmark::default()
            },
        );
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn add_landmark_auto(&mut self, p: Vector4<f64>, initialised: bool) -> Result<LandmarkId> {
        let id = self
            .landmarks
            .keys()
            .next_back()
            .map_or(LandmarkId::new(1), |id| *id + 1);
        self.add_landmark(id, p, initialised)?;
        Ok(id)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn remove_landmark(&mut self, id: LandmarkId) -> Result<bool> {
        let keys = self
            .landmark_info
            .get(&id)
            .ok_or_else(|| graph_error("landmark absent"))?
            .observations
            .clone();
        let covisibilities_computed = self.covisibilities_computed;
        for key in keys {
            self.remove_observation(key)?;
        }
        self.covisibilities_computed = covisibilities_computed;
        let point = self
            .landmarks
            .remove(&id)
            .ok_or_else(|| graph_error("landmark absent"))?
            .h_point;
        self.problem.remove_parameter(&Parameter::Point(point));
        self.landmark_info.remove(&id);
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_landmark(
        &mut self,
        id: LandmarkId,
        p: Vector4<f64>,
        initialised: Option<bool>,
    ) -> Result<bool> {
        let lm = self
            .landmarks
            .get_mut(&id)
            .ok_or_else(|| graph_error("landmark absent"))?;
        let mut point = lm.h_point.borrow_mut();
        point.set_estimate(p);
        if let Some(value) = initialised {
            point.set_initialized(value);
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_landmark_initialised(&mut self, id: LandmarkId, value: bool) -> Result<bool> {
        self.landmarks
            .get(&id)
            .ok_or_else(|| graph_error("landmark absent"))?
            .h_point
            .borrow_mut()
            .set_initialized(value);
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_landmark_quality(&mut self, id: LandmarkId, value: f64) -> Result<bool> {
        self.landmarks
            .get_mut(&id)
            .ok_or_else(|| graph_error("landmark absent"))?
            .quality = value;
        Ok(true)
    }
    #[must_use]
    pub fn is_landmark_added(&self, id: LandmarkId) -> bool {
        self.landmarks.contains_key(&id)
    }
    #[must_use]
    pub fn landmark_exists(&self, id: LandmarkId) -> bool {
        self.is_landmark_added(id)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn is_landmark_initialised(&self, id: LandmarkId) -> Result<bool> {
        Ok(self
            .landmarks
            .get(&id)
            .ok_or_else(|| graph_error("landmark absent"))?
            .h_point
            .borrow()
            .initialized())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn landmark(&self, id: LandmarkId) -> Result<Vector4<f64>> {
        self.landmarks
            .get(&id)
            .ok_or_else(|| graph_error("landmark absent"))?
            .h_point
            .borrow()
            .estimate()
            .copied()
            .ok_or_else(|| graph_error("point estimate absent"))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn get_landmark(&self, id: LandmarkId) -> Result<MapPoint2> {
        let lm = self
            .landmarks
            .get(&id)
            .ok_or_else(|| graph_error("landmark absent"))?;
        let meta = self
            .landmark_info
            .get(&id)
            .ok_or_else(|| graph_error("landmark metadata absent"))?;
        Ok(MapPoint2 {
            id,
            point: self.landmark(id)?,
            observations: meta.observations.clone(),
            is_initialised: lm.h_point.borrow().initialized(),
            quality: lm.quality,
            classification: meta.classification,
        })
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn get_landmarks(&self) -> Result<MapPoints> {
        self.landmarks
            .keys()
            .map(|&id| Ok((id, self.get_landmark(id)?)))
            .collect()
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn add_observation(
        &mut self,
        frame: &MultiFrame,
        id: LandmarkId,
        key: KeypointIdentifier,
        _distortion: DistortionType,
        cauchy: bool,
    ) -> Result<bool> {
        if frame.landmark_id(key.camera_index, key.keypoint_index) != id.value() {
            return Err(graph_error("frame landmark differs"));
        }
        let mut measurement = Vector2::zeros();
        if !frame.get_keypoint(key.camera_index, key.keypoint_index, &mut measurement) {
            return Err(graph_error("keypoint absent"));
        }
        let mut size = 1.0;
        frame.get_keypoint_size(key.camera_index, key.keypoint_index, &mut size);
        let factor = ReprojectionError::new(
            frame.geometry(key.camera_index).clone(),
            key.camera_index as u64,
            measurement,
            Matrix2::identity() * (64.0 / (size * size)),
        );
        self.add_external_observation(&factor, id, key, cauchy)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn add_external_observation(
        &mut self,
        factor: &dyn ReprojectionError2dBase,
        id: LandmarkId,
        key: KeypointIdentifier,
        cauchy: bool,
    ) -> Result<bool> {
        if self.observations.contains_key(&key) {
            return Err(graph_error("duplicate observation"));
        }
        let point = self
            .landmarks
            .get(&id)
            .ok_or_else(|| graph_error("landmark absent"))?
            .h_point
            .clone();
        let pose = self.pose_block(StateId::new(key.frame_id))?;
        let ext = self.extrinsics_block(StateId::new(key.frame_id), key.camera_index)?;
        let error = Rc::new(RefCell::new(factor.clone_box()));
        let residual = self.problem.add(
            GraphFactor::Reprojection(error.clone()),
            vec![
                Parameter::Pose(pose),
                Parameter::Point(point),
                Parameter::Pose(ext),
            ],
            cauchy.then_some(RobustLoss::Cauchy(1.0)),
        );
        let core = ComponentObservation {
            landmark_id: id,
            error_term: error,
        };
        self.states
            .get_mut(&StateId::new(key.frame_id))
            .expect("pose checked")
            .observations
            .insert(key, core.clone());
        self.landmark_info
            .get_mut(&id)
            .expect("landmark checked")
            .observations
            .insert(key);
        self.observations.insert(
            key,
            GraphObservation {
                core,
                residual,
                depth_error: None,
            },
        );
        self.covisibilities_computed = false;
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn remove_observation(&mut self, key: KeypointIdentifier) -> Result<bool> {
        let obs = self
            .observations
            .remove(&key)
            .ok_or_else(|| graph_error("observation absent"))?;
        self.problem.residuals.remove(&obs.residual);
        if let Some(id) = obs.depth_error {
            self.problem.residuals.remove(&id);
        }
        self.landmark_info
            .get_mut(&obs.core.landmark_id)
            .ok_or_else(|| graph_error("landmark absent"))?
            .observations
            .remove(&key);
        self.states
            .get_mut(&StateId::new(key.frame_id))
            .ok_or_else(|| graph_error("state absent"))?
            .observations
            .remove(&key);
        self.covisibilities_computed = false;
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn add_one_sided_depth_error(&mut self, key: KeypointIdentifier) -> Result<bool> {
        let obs = self
            .observations
            .get(&key)
            .ok_or_else(|| graph_error("observation absent"))?;
        if obs.depth_error.is_some() {
            return Ok(false);
        }
        let id = obs.core.landmark_id;
        let parameters = vec![
            Parameter::Pose(self.pose_block(StateId::new(key.frame_id))?),
            Parameter::Point(self.landmarks[&id].h_point.clone()),
            Parameter::Pose(self.extrinsics_block(StateId::new(key.frame_id), key.camera_index)?),
        ];
        let rid = self.problem.add(
            GraphFactor::Depth(OneSidedDepthError::new(0.1, 0.001)),
            parameters,
            None,
        );
        self.observations
            .get_mut(&key)
            .expect("observation exists")
            .depth_error = Some(rid);
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn remove_one_sided_depth_error(&mut self, key: KeypointIdentifier) -> Result<bool> {
        let rid = self
            .observations
            .get_mut(&key)
            .ok_or_else(|| graph_error("observation absent"))?
            .depth_error
            .take();
        if let Some(id) = rid {
            self.problem.residuals.remove(&id);
            Ok(true)
        } else {
            Ok(false)
        }
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn check_observations(&self) -> Result<()> {
        for (key, obs) in &self.observations {
            if !self
                .landmark_info
                .get(&obs.core.landmark_id)
                .is_some_and(|m| m.observations.contains(key))
                || !self
                    .states
                    .get(&StateId::new(key.frame_id))
                    .is_some_and(|s| s.observations.contains_key(key))
            {
                return Err(graph_error("observation bookkeeping inconsistent"));
            }
        }
        for state in self.states.values() {
            for key in state.observations.keys() {
                if !self.observations.contains_key(key) {
                    return Err(graph_error("state observation absent"));
                }
            }
        }
        for lm in self.landmark_info.values() {
            for key in &lm.observations {
                if !self.observations.contains_key(key) {
                    return Err(graph_error("landmark observation absent"));
                }
            }
        }
        Ok(())
    }
    pub fn compute_covisibilities(&mut self) -> bool {
        if self.covisibilities_computed {
            return true;
        }
        self.co_observation_counts.clear();
        self.visible_frames.clear();
        for lm in self.landmark_info.values() {
            if lm.classification == 10 || lm.classification == 11 {
                continue;
            }
            let frames: BTreeSet<_> = lm.observations.iter().map(|k| k.frame_id).collect();
            for &a in &frames {
                self.visible_frames.insert(StateId::new(a));
                for &b in frames.range(..a) {
                    *self
                        .co_observation_counts
                        .entry(a)
                        .or_default()
                        .entry(b)
                        .or_default() += 1;
                }
            }
        }
        self.covisibilities_computed = true;
        !self.co_observation_counts.is_empty()
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn covisibilities(&self, a: StateId, b: StateId) -> Result<i32> {
        if !self.covisibilities_computed {
            return Err(graph_error("covisibilities not computed"));
        }
        if a == b {
            return Ok(0);
        }
        let (a, b) = (a.max(b).value(), a.min(b).value());
        Ok(self
            .co_observation_counts
            .get(&a)
            .and_then(|m| m.get(&b))
            .copied()
            .unwrap_or(0))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn add_relative_pose_constraint(
        &mut self,
        a: StateId,
        b: StateId,
        transform: &Transformation,
        information: SMatrix<f64, 6, 6>,
    ) -> Result<bool> {
        if self
            .state_info
            .get(&a)
            .is_some_and(|s| s.relative_pose_links.contains_key(&b))
        {
            return Err(graph_error("duplicate relative pose link"));
        }
        let parameters = vec![
            Parameter::Pose(self.pose_block(a)?),
            Parameter::Pose(self.pose_block(b)?),
        ];
        let factor = RelativePoseError::new(information, pose_isometry(transform));
        let rid = self.problem.add(
            GraphFactor::RelativePose(Box::new(factor)),
            parameters,
            None,
        );
        self.state_info
            .get_mut(&a)
            .ok_or_else(|| graph_error("state absent"))?
            .relative_pose_links
            .insert(b, rid);
        self.state_info
            .get_mut(&b)
            .ok_or_else(|| graph_error("state absent"))?
            .relative_pose_links
            .insert(a, rid);
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn remove_relative_pose_constraint(&mut self, a: StateId, b: StateId) -> Result<bool> {
        let rid = self
            .state_info
            .get_mut(&a)
            .ok_or_else(|| graph_error("state absent"))?
            .relative_pose_links
            .remove(&b)
            .ok_or_else(|| graph_error("relative link absent"))?;
        self.problem.residuals.remove(&rid);
        self.state_info
            .get_mut(&b)
            .ok_or_else(|| graph_error("state absent"))?
            .relative_pose_links
            .remove(&a);
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn observation_evaluation(
        &self,
        key: KeypointIdentifier,
        minimal: bool,
    ) -> Result<ErrorEvaluation> {
        let obs = self
            .observations
            .get(&key)
            .ok_or_else(|| graph_error("observation absent"))?;
        let r = self
            .problem
            .residuals
            .get(&obs.residual)
            .ok_or_else(|| graph_error("observation residual absent"))?;
        let values: Vec<_> = r
            .parameters
            .iter()
            .map(Parameter::values)
            .collect::<Result<_>>()?;
        let slices: Vec<_> = values.iter().map(Vec::as_slice).collect();
        obs.core
            .error_term
            .borrow()
            .evaluate_with_minimal_jacobians(
                &slices,
                minimal.then_some(&[true, true, true]),
                minimal.then_some(&[true, true, true]),
            )
            .ok_or_else(|| graph_error("reprojection evaluation failed"))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn get_landmark_hessian(&self, id: LandmarkId) -> Result<Matrix3<f64>> {
        let mut h = Matrix3::zeros();
        for &key in &self
            .landmark_info
            .get(&id)
            .ok_or_else(|| graph_error("landmark absent"))?
            .observations
        {
            let e = self.observation_evaluation(key, true)?;
            let j = e.minimal_jacobians[1]
                .as_ref()
                .ok_or_else(|| graph_error("landmark Jacobian absent"))?;
            h.gemm_tr(1.0, j, j, 1.0);
        }
        Ok(h)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn are_landmarks_in_front_of_cameras(&self) -> Result<bool> {
        for (key, obs) in &self.observations {
            let pose = self
                .pose(StateId::new(key.frame_id))?
                .compose(&self.extrinsics(StateId::new(key.frame_id), key.camera_index)?);
            let point = pose
                .inverse()
                .transform_homogeneous(&self.landmark(obs.core.landmark_id)?);
            if point[2] * point[3] < 0.0 || (point[3].abs() > 1.0e-16 && point[2] / point[3] < 0.05)
            {
                return Ok(false);
            }
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn update_landmarks(&mut self) -> Result<()> {
        let ids: Vec<_> = self.landmarks.keys().copied().collect();
        for id in ids {
            let point = self.landmark(id)?;
            let keys = self.landmark_info[&id].observations.clone();
            let mut directions = Vec::new();
            let mut behind = false;
            let mut best_error = 1.0e12;
            let mut best_position = None;
            for key in keys {
                let pose = self
                    .pose(StateId::new(key.frame_id))?
                    .compose(&self.extrinsics(StateId::new(key.frame_id), key.camera_index)?);
                let mut camera = pose.inverse().transform_homogeneous(&point);
                if camera[3].abs() > 1.0e-12 {
                    camera /= camera[3];
                }
                let mut dir = pose.rotation_matrix() * camera.fixed_rows::<3>(0);
                dir = normalize(dir);
                if camera[2] < 0.1 {
                    behind = true;
                }
                if camera[2] < 0.0 {
                    dir = -dir;
                }
                let err = self.observation_evaluation(key, false)?.residuals.norm();
                if err > 2.5 {
                    continue;
                }
                if err < best_error && camera.norm() > 0.0001 {
                    let p = pose.translation() + camera.norm().max(0.1) * dir;
                    best_position = Some(Vector4::new(p[0], p[1], p[2], 1.0));
                    best_error = err;
                }
                directions.push(dir);
            }
            let mean =
                directions.iter().fold(Vector3::zeros(), |s, d| s + d) / directions.len() as f64;
            let quality = directions
                .iter()
                .fold(0.0, |s, d| s + (d - mean).norm_squared())
                .sqrt();
            let initialised = quality > 0.04;
            if !initialised
                && behind
                && let Some(p) = best_position
            {
                self.set_landmark(id, p, None)?;
            }
            self.set_landmark_initialised(id, initialised)?;
            self.set_landmark_quality(id, quality)?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn optimise(
        &mut self,
        max_iterations: usize,
        num_threads: usize,
        verbose: bool,
    ) -> Result<()> {
        self.options.max_iterations = max_iterations;
        self.options.num_threads = num_threads;
        self.options.verbose = verbose;
        let solver = self
            .solver
            .as_mut()
            .ok_or_else(|| graph_error("Ceres GraphSolver capability absent"))?;
        self.summary = solver.solve(&mut self.problem, &self.options)?;
        Ok(())
    }
    pub fn set_optimisation_time_limit(&mut self, limit: f64, min_iterations: usize) -> bool {
        if self.options.time_limit.is_some() && limit < 0.0 {
            self.options.minimum_iterations = self.options.max_iterations;
        } else if limit >= 0.0 {
            self.options.time_limit = Some(limit);
            self.options.minimum_iterations = min_iterations;
        }
        true
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn clean_unobserved_landmarks(
        &mut self,
    ) -> Result<BTreeMap<LandmarkId, BTreeSet<KeypointIdentifier>>> {
        let ids: Vec<_> = self
            .landmark_info
            .iter()
            .filter(|(_, m)| m.observations.len() <= 1)
            .map(|(&id, m)| (id, m.observations.clone()))
            .collect();
        let mut removed = BTreeMap::new();
        for (id, keys) in ids {
            self.remove_landmark(id)?;
            removed.insert(id, keys);
        }
        Ok(removed)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_extrinsics_variable(&mut self) -> Result<bool> {
        let id = *self
            .states
            .keys()
            .next()
            .ok_or_else(|| graph_error("graph empty"))?;
        for i in 0..self.camera_parameters.len() {
            self.extrinsics_block(id, i)?.borrow_mut().set_fixed(false);
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn soft_constrain_extrinsics(&mut self, pos_std: f64, rot_std: f64) -> Result<bool> {
        let id = *self
            .states
            .keys()
            .next()
            .ok_or_else(|| graph_error("graph empty"))?;
        let old = self.state_info[&id].extrinsics_priors.clone();
        if old.len() == self.camera_parameters.len() {
            for rid in old {
                self.problem.residuals.remove(&rid);
            }
        }
        let mut priors = Vec::new();
        for i in 0..self.camera_parameters.len() {
            let ext = self.extrinsics_block(id, i)?;
            let pose = self.extrinsics(id, i)?;
            priors.push(self.problem.add(
                GraphFactor::PosePrior(Box::new(PoseError::from_variances(
                    pose_isometry(&pose),
                    pos_std * pos_std,
                    rot_std * rot_std,
                ))),
                vec![Parameter::Pose(ext)],
                None,
            ));
        }
        self.state_info
            .get_mut(&id)
            .expect("state exists")
            .extrinsics_priors = priors;
        Ok(true)
    }
    #[must_use]
    pub fn get_gps_status(&self) -> GpsStatus {
        self.gps_status
    }
    pub fn set_gps_status(&mut self, status: GpsStatus) {
        self.gps_status = status;
    }
    #[must_use]
    pub fn find_state_id(&self, id: StateId) -> bool {
        self.states.contains_key(&id)
    }
    #[must_use]
    pub fn is_gps_observable(&self) -> bool {
        self.gps_flags.gps_observability
    }
    #[must_use]
    pub fn is_gps_fixed(&self) -> bool {
        self.gps_flags.gps_fixed
    }
    #[must_use]
    pub fn needs_initial_gps_alignment(&self) -> bool {
        self.alignment_requests.needs_initial_alignment
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn freeze_gps_extrinsics(&mut self) -> Result<()> {
        let p = self
            .state_info
            .values()
            .next()
            .and_then(|s| s.t_gw.clone())
            .ok_or_else(|| graph_error("GPS transform absent"))?;
        p.borrow_mut().set_fixed(true);
        self.gps_flags.gps_fixed = true;
        Ok(())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn unfreeze_gps_extrinsics(&mut self) -> Result<()> {
        let p = self
            .state_info
            .values()
            .next()
            .and_then(|s| s.t_gw.clone())
            .ok_or_else(|| graph_error("GPS transform absent"))?;
        p.borrow_mut().set_fixed(false);
        self.gps_flags.gps_fixed = false;
        Ok(())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn set_gps_extrinsics(&mut self, pose: &Transformation) -> Result<bool> {
        let p = self
            .state_info
            .values()
            .next()
            .and_then(|s| s.t_gw.clone())
            .ok_or_else(|| graph_error("GPS transform absent"))?;
        p.borrow_mut()
            .set_estimate(TransformationCacheless::from_other(pose));
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn needs_gps_reinit(&mut self) -> Result<bool> {
        let Some(&last) = self.gps_states.last() else {
            return Ok(false);
        };
        if self.gps_status == GpsStatus::Initialised && self.pose_block(last)?.borrow().fixed() {
            self.alignment_requests.needs_position_alignment = true;
            self.gps_dropout_id = Some(last);
            return Ok(true);
        }
        if self.gps_status == GpsStatus::ReInitialising {
            let id = self
                .position_aligned_id
                .ok_or_else(|| graph_error("position alignment ID absent"))?;
            let next = self
                .states
                .range(id..)
                .next()
                .map(|(&i, _)| i)
                .ok_or_else(|| graph_error("position alignment state absent"))?;
            if self.pose_block(next)?.borrow().fixed() {
                self.alignment_requests.needs_position_alignment = true;
                return Ok(true);
            }
        }
        Ok(false)
    }
    pub fn reinit_gps_extrinsics(&mut self) {
        self.gps_status = GpsStatus::ReInitialising;
        self.alignment_requests.needs_position_alignment = true;
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn needs_full_gps_alignment(&self) -> Result<Option<(StateId, StateId, Transformation)>> {
        if !self.alignment_requests.needs_full_alignment {
            return Ok(None);
        }
        Ok(Some((
            self.gps_dropout_id
                .ok_or_else(|| graph_error("GPS dropout ID absent"))?,
            *self
                .gps_reinit_states
                .last()
                .ok_or_else(|| graph_error("GPS reinit states absent"))?,
            self.t_gw_init
                .clone()
                .ok_or_else(|| graph_error("GPS initialization absent"))?,
        )))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn needs_pos_gps_alignment(&mut self) -> Result<Option<(StateId, StateId, Vector3<f64>)>> {
        if self
            .gps_parameters
            .last()
            .ok_or_else(|| graph_error("GPS parameters absent"))?
            .robust_gps_init
            || !self.alignment_requests.needs_position_alignment
        {
            return Ok(None);
        }
        let mut measurement = self
            .gps_init_map
            .values()
            .next_back()
            .cloned()
            .ok_or_else(|| graph_error("GPS initialization measurement absent"))?;
        self.convert_gps(&mut measurement)?;
        let Some(id) = self.state_before(measurement.time_stamp) else {
            return Ok(None);
        };
        let mut factor = self.make_gps_factor(id, &measurement, &self.gps_init_imu_queue)?;
        let pose = self.pose(id)?;
        let sb = self.speed_and_bias(id)?;
        let global = self.t_gw(None)?;
        factor
            .evaluate(pose.coeffs(), &sb, global.coeffs())
            .map_err(|e| graph_error(&format!("GPS evaluation: {e:?}")))?;
        let error = global.inverse().rotation_matrix()
            * factor
                .error
                .ok_or_else(|| graph_error("GPS error absent"))?;
        self.position_aligned_id = Some(id);
        Ok(Some((
            self.gps_dropout_id
                .ok_or_else(|| graph_error("GPS dropout ID absent"))?,
            id,
            error,
        )))
    }
    pub fn reset_full_gps_alignment(&mut self) {
        self.gps_flags.gps_reinitialised = false;
        self.alignment_requests.needs_full_alignment = false;
        self.alignment_requests.needs_position_alignment = false;
        self.gps_status = GpsStatus::Initialised;
        self.gps_init_map.clear();
        self.gps_init_imu_queue.clear();
        self.gps_reinit_states.clear();
    }
    pub fn reset_pos_gps_alignment(&mut self) {
        self.alignment_requests.needs_position_alignment = false;
    }
    pub fn reset_initial_gps_alignment(&mut self) {
        self.gps_init_map.clear();
        self.gps_init_imu_queue.clear();
        self.alignment_requests.needs_initial_alignment = false;
    }
    #[must_use]
    pub fn state_before(&self, time: Timestamp) -> Option<StateId> {
        self.states
            .iter()
            .rev()
            .find(|(_, s)| s.timestamp.is_some_and(|t| t <= time))
            .map(|(&id, _)| id)
    }
    fn convert_gps(&self, measurement: &mut GpsMeasurement) -> Result<()> {
        let params = self
            .gps_parameters
            .last()
            .ok_or_else(|| graph_error("GPS parameters absent"))?;
        if params.r#type == "geodetic" || params.r#type == "geodetic-leica" {
            let local = self
                .local_cartesian
                .as_ref()
                .ok_or_else(|| graph_error("GeographicLib LocalCartesian capability absent"))?;
            let m = &measurement.measurement;
            measurement.measurement.position =
                local.borrow().forward(m.latitude, m.longitude, m.height)?;
        }
        Ok(())
    }
    fn make_gps_factor(
        &self,
        id: StateId,
        measurement: &GpsMeasurement,
        imu: &ImuMeasurementDeque,
    ) -> Result<GpsErrorAsynchronous> {
        let m = &measurement.measurement;
        Ok(GpsErrorAsynchronous::new(
            m.position,
            inverse3(&m.covariances),
            imu.clone(),
            self.imu_parameters
                .last()
                .ok_or_else(|| graph_error("IMU parameters absent"))?
                .clone(),
            self.timestamp(id)?,
            measurement.time_stamp,
            self.gps_parameters
                .last()
                .ok_or_else(|| graph_error("GPS parameters absent"))?
                .clone(),
        ))
    }
    fn gps_parameters_for_state(&self, id: StateId) -> Result<Vec<Parameter>> {
        Ok(vec![
            Parameter::Pose(self.pose_block(id)?),
            Parameter::SpeedBias(self.speed_bias_block(id)?),
            Parameter::Pose(
                self.state_info
                    .get(&id)
                    .and_then(|s| s.t_gw.clone())
                    .ok_or_else(|| graph_error("GPS transform absent"))?,
            ),
        ])
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn add_gps_measurement(
        &mut self,
        id: StateId,
        measurement: &mut GpsMeasurement,
        imu: &ImuMeasurementDeque,
    ) -> Result<bool> {
        let time = self.timestamp(id)?;
        if measurement.time_stamp < time {
            return Err(graph_error("GPS measurement too old"));
        }
        if imu.front().is_none_or(|m| m.time_stamp > time) {
            return Ok(false);
        }
        if imu
            .back()
            .is_none_or(|m| m.time_stamp < measurement.time_stamp)
        {
            return Err(graph_error("GPS measurement not covered by IMU"));
        }
        self.convert_gps(measurement)?;
        let factor = Rc::new(RefCell::new(self.make_gps_factor(id, measurement, imu)?));
        let residual = if matches!(
            self.gps_status,
            GpsStatus::Initialising | GpsStatus::Initialised | GpsStatus::ReInitialising
        ) {
            Some(self.problem.add(
                GraphFactor::Gps(factor.clone()),
                self.gps_parameters_for_state(id)?,
                Some(RobustLoss::Cauchy(3.0)),
            ))
        } else {
            None
        };
        self.gps_states.insert(id);
        self.state_info
            .get_mut(&id)
            .expect("timestamp checked")
            .gps_factors
            .push((residual, factor));
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn gps_measurements(&self, id: StateId) -> Result<Vec<Vector3<f64>>> {
        Ok(self
            .state_info
            .get(&id)
            .ok_or_else(|| graph_error("state absent"))?
            .gps_factors
            .iter()
            .map(|(_, f)| f.borrow().measurement)
            .collect())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn check_valid_gps_measurements(
        &self,
        input: &mut GpsMeasurementDeque,
    ) -> Result<GpsMeasurementDeque> {
        let mut result = VecDeque::new();
        let needs_reinit = if self.gps_status == GpsStatus::Initialised {
            let id = *self
                .gps_states
                .last()
                .ok_or_else(|| graph_error("initialized GPS states empty"))?;
            self.pose_block(id)?.borrow().fixed()
        } else {
            false
        };
        for measurement in input.iter_mut().rev() {
            let Some(id) = self.state_before(measurement.time_stamp) else {
                break;
            };
            if matches!(
                self.gps_status,
                GpsStatus::Off | GpsStatus::Idle | GpsStatus::Initialising
            ) {
                if measurement.measurement.covariances[(0, 0)] > 36.0
                    || measurement.measurement.covariances[(2, 2)] > 100.0
                {
                    continue;
                }
                result.push_back(measurement.clone());
            }
            if self.gps_status == GpsStatus::Initialised && !needs_reinit {
                self.convert_gps(measurement)?;
                let pose = self.pose(id)?;
                let global = self.t_gw(Some(id))?;
                let world = global
                    .inverse()
                    .transform_homogeneous(&measurement.measurement.position.push(1.0));
                let antenna = self
                    .gps_parameters
                    .last()
                    .ok_or_else(|| graph_error("GPS parameters absent"))?
                    .r_sa;
                let error = pose.translation() + pose.rotation_matrix() * antenna
                    - world.fixed_rows::<3>(0);
                if (0..3).any(|i| {
                    error[i].abs() > 3.0 * measurement.measurement.covariances[(i, i)].sqrt()
                }) {
                    continue;
                }
                result.push_back(measurement.clone());
            } else if self.gps_status == GpsStatus::ReInitialising || needs_reinit {
                result.push_back(measurement.clone());
            }
        }
        Ok(result)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn add_gps_measurements(
        &mut self,
        gps: &mut GpsMeasurementDeque,
        imu: &ImuMeasurementDeque,
    ) -> Result<VecDeque<StateId>> {
        let mut ids = VecDeque::new();
        if gps.is_empty() || imu.is_empty() {
            return Ok(ids);
        }
        if imu.front().expect("nonempty").time_stamp > gps.front().expect("nonempty").time_stamp {
            return Ok(ids);
        }
        if imu.back().expect("nonempty").time_stamp < gps.back().expect("nonempty").time_stamp {
            return Err(graph_error("GPS measurements not covered"));
        }
        if self.gps_status != GpsStatus::Initialised {
            for m in imu {
                if self
                    .gps_init_imu_queue
                    .back()
                    .is_some_and(|last| m.time_stamp < last.time_stamp)
                {
                    continue;
                }
                self.gps_init_imu_queue.push_back(m.clone());
            }
        }
        for m in gps.iter_mut().rev() {
            let Some(id) = self.state_before(m.time_stamp) else {
                break;
            };
            ids.push_front(id);
            self.state_info.get_mut(&id).expect("state found").gps_mode = Some(self.gps_status);
            match self.gps_status {
                GpsStatus::Off => {
                    let local = self.local_cartesian.as_ref().ok_or_else(|| {
                        graph_error("GeographicLib LocalCartesian capability absent")
                    })?;
                    local.borrow_mut().reset(
                        m.measurement.latitude,
                        m.measurement.longitude,
                        m.measurement.height,
                    )?;
                    self.gps_init_map.entry(id).or_insert_with(|| m.clone());
                    self.gps_status = GpsStatus::Idle;
                }
                GpsStatus::Idle | GpsStatus::Initialising => {
                    self.gps_init_map.entry(id).or_insert_with(|| m.clone());
                    self.add_gps_measurement(id, m, imu)?;
                }
                GpsStatus::Initialised => {
                    self.add_gps_measurement(id, m, imu)?;
                }
                GpsStatus::ReInitialising => {
                    self.gps_init_map.entry(id).or_insert_with(|| m.clone());
                    self.gps_reinit_states.insert(id);
                    self.add_gps_measurement(id, m, imu)?;
                }
            }
        }
        Ok(ids)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn check_for_gps_init(
        &mut self,
        considered: &BTreeSet<StateId>,
    ) -> Result<Option<(Transformation, f64, bool)>> {
        if considered.len() < 2 {
            return Ok(None);
        }
        let robust = self
            .gps_parameters
            .last()
            .ok_or_else(|| graph_error("GPS parameters absent"))?
            .robust_gps_init;
        let antenna = self.gps_parameters.last().expect("GPS checked").r_sa;
        let skip = if robust && considered.len() > 100 {
            considered.len() - 100
        } else {
            0
        };
        let mut gps = Vec::new();
        let mut world = Vec::new();
        let mut covariances = Vec::new();
        for &id in considered.iter().skip(skip) {
            let pose = self.pose(id)?;
            let sb = self.speed_and_bias(id)?;
            for (_, factor) in &self.state_info[&id].gps_factors {
                let mut f = factor.borrow_mut();
                gps.push(f.measurement);
                let propagated = f.apply_pre_int(&pose, &sb);
                world.push(propagated.translation() + propagated.rotation_matrix() * antenna);
                covariances.push(f.gps_covariance);
            }
        }
        let mut transform = if robust {
            let Some(best) = estimate_rigid_ransac(&gps, &world, 20, 20, 4.0, 0.7)? else {
                return Ok(None);
            };
            if best.inlier_ratio < 0.25 {
                return Ok(None);
            }
            Transformation::new(best.t, quaternion_from_rotation_matrix(&best.r))
        } else {
            umeyama_transform(&gps, &world)
        };
        let mut h = Matrix4::zeros();
        for (p, covariance) in world.iter().zip(&covariances) {
            let mut e = SMatrix::<f64, 3, 4>::zeros();
            e.fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&-Matrix3::identity());
            e.set_column(3, &cross_mx(&(transform.rotation_matrix() * p)).column(2));
            h += e.transpose() * inverse3(covariance) * e;
        }
        let covariance = inverse4(&h);
        let yaw = covariance[(3, 3)].sqrt() / std::f64::consts::PI * 180.0;
        let acceptable = yaw
            < self
                .gps_parameters
                .last()
                .expect("GPS checked")
                .yaw_error_threshold;
        if acceptable && robust {
            let solver = self
                .solver
                .as_mut()
                .ok_or_else(|| graph_error("Ceres alignment solver absent"))?;
            transform = solver.refine_alignment_4d(&gps, &world, &transform, 100, 3.0)?;
        }
        Ok(Some((transform, yaw, acceptable)))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn initialization_strategy(&mut self) -> Result<Option<Transformation>> {
        match self.gps_status {
            GpsStatus::Off | GpsStatus::Initialised => Ok(None),
            GpsStatus::Idle => {
                let states = self.gps_states.clone();
                if let Some((transform, yaw, _)) = self.check_for_gps_init(&states)?
                    && yaw < 5.0
                {
                    self.gps_status = GpsStatus::Initialising;
                    return Ok(Some(transform));
                }
                Ok(None)
            }
            GpsStatus::Initialising => {
                let states = self.gps_states.clone();
                if let Some((transform, _, acceptable)) = self.check_for_gps_init(&states)? {
                    self.gps_flags.gps_observability = acceptable;
                    if acceptable {
                        self.set_gps_extrinsics(&transform)?;
                        self.gps_status = GpsStatus::Initialised;
                        self.t_gw_init = Some(transform);
                        self.alignment_requests.needs_initial_alignment = true;
                    }
                }
                Ok(None)
            }
            GpsStatus::ReInitialising => {
                let states = self.gps_reinit_states.clone();
                if let Some((transform, _, acceptable)) = self.check_for_gps_init(&states)? {
                    self.gps_flags.gps_reinitialised = acceptable;
                    if acceptable {
                        let dropout = self
                            .gps_dropout_id
                            .ok_or_else(|| graph_error("GPS dropout ID absent"))?;
                        let steps = self
                            .states
                            .range((
                                std::ops::Bound::Excluded(dropout),
                                std::ops::Bound::Unbounded,
                            ))
                            .count() as f64;
                        let current = self.t_gw(None)?;
                        let q = current.quaternion() * transform.quaternion().conjugate();
                        let angle = quaternion_angular_distance(&q);
                        let error = angle / steps;
                        let budget = 0.03 + 0.004 / steps.sqrt();
                        if error > budget {
                            self.gps_flags.gps_reinitialised = false;
                        }
                    }
                    if self.gps_flags.gps_reinitialised {
                        self.alignment_requests.needs_full_alignment = true;
                        self.t_gw_init = Some(transform);
                    }
                }
                Ok(None)
            }
        }
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn add_gps_init_factors(&mut self) -> Result<()> {
        let ids: Vec<_> = self.gps_init_map.keys().copied().collect();
        for id in ids {
            let params = self.gps_parameters_for_state(id)?;
            let factors = self.state_info[&id].gps_factors.clone();
            for (index, (_, factor)) in factors.into_iter().enumerate() {
                let rid = self.problem.add(
                    GraphFactor::Gps(factor),
                    params.clone(),
                    Some(RobustLoss::Cauchy(3.0)),
                );
                self.state_info
                    .get_mut(&id)
                    .expect("state exists")
                    .gps_factors[index]
                    .0 = Some(rid);
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn dump_gps_residuals(&mut self, path: impl AsRef<Path>) -> Result<()> {
        let mut file = std::fs::File::create(path).map_err(|error| io_error(&error))?;
        writeln!(file, "# tk , tg , res_x , res_y , res_z").map_err(|error| io_error(&error))?;
        let ids: Vec<_> = self.state_info.keys().copied().collect();
        for id in ids {
            let pose = self.pose(id)?;
            let sb = self.speed_and_bias(id)?;
            let global = self.t_gw(Some(id))?;
            for (residual, factor) in &self.state_info[&id].gps_factors {
                if !residual.is_some_and(|rid| self.problem.residuals.contains_key(&rid)) {
                    continue;
                }
                let mut f = factor.borrow_mut();
                f.set_information(Matrix3::identity());
                let e = f
                    .evaluate(pose.coeffs(), &sb, global.coeffs())
                    .map_err(|e| graph_error(&format!("GPS residual: {e:?}")))?;
                let tk = f.tk.as_nanosecond();
                let tg = f.tg.as_nanosecond();
                writeln!(
                    file,
                    "{}.{:09} , {}.{:09} , {} , {} , {}",
                    tk.div_euclid(1_000_000_000),
                    tk.rem_euclid(1_000_000_000),
                    tg.div_euclid(1_000_000_000),
                    tg.rem_euclid(1_000_000_000),
                    e.residual[0],
                    e.residual[1],
                    e.residual[2]
                )
                .map_err(|error| io_error(&error))?;
            }
        }
        file.flush().map_err(|error| io_error(&error))
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn add_submap_alignment_constraints(
        &mut self,
        map: &Rc<dyn OccupancySubmap>,
        (a, b): (StateId, StateId),
        points: &[Vector3<f32>],
        sensor_error: &[f32],
        is_lidar: bool,
        robust: &str,
    ) -> Result<bool> {
        if !self.states.contains_key(&a) || !self.states.contains_key(&b) {
            return Ok(false);
        }
        if points.len() != sensor_error.len() {
            return Err(graph_error("sensor error count differs"));
        }
        let loss = match robust {
            "Cauchy" => RobustLoss::Cauchy(1.0),
            "Tukey" => RobustLoss::Tukey(if is_lidar { 2.0 } else { 0.1 }),
            _ => return Err(graph_error("only Cauchy or Tukey submap losses supported")),
        };
        let pa = self.pose_block(a)?;
        let pb = self.pose_block(b)?;
        let mut debug = LidarDebugInfo {
            frame_id: b.value(),
            reference_id: a.value(),
            n_residuals: points.len(),
            n_nonzero_occ: 0,
            n_nonzero_grad: 0,
            mean_res: 0.0,
            max_residual: 0.0,
            n_big_residuals: 0,
            mean_field: 0.0,
            min_field: 0.0,
            max_field: 0.0,
            mean_grad: 0.0,
            max_grad: 0.0,
        };
        for (point, &sigma) in points.iter().zip(sensor_error) {
            let rid = self.problem.add(
                GraphFactor::Submap {
                    map: map.clone(),
                    point: point.cast(),
                    sigma: f64::from(sigma),
                },
                vec![Parameter::Pose(pa.clone()), Parameter::Pose(pb.clone())],
                Some(loss),
            );
            let ma = self.state_info.get_mut(&a).expect("state checked");
            ma.map_res_ids.push(rid);
            ma.submap_reference_links.push(rid);
            let mb = self.state_info.get_mut(&b).expect("state checked");
            mb.map_res_ids.push(rid);
            mb.submap_links.push(rid);
            if self.debug_lidar_residuals {
                let factor = crate::ceres::submap_icp_error::SubmapIcpError::new(
                    map.as_ref(),
                    point.cast(),
                    f64::from(sigma),
                );
                let p0 = self.pose(a)?;
                let p1 = self.pose(b)?;
                let mut residual = 0.0;
                let (mut j0, mut j1) =
                    (SMatrix::<f64, 1, 7>::zeros(), SMatrix::<f64, 1, 7>::zeros());
                factor.evaluate(
                    [p0.coeffs(), p1.coeffs()],
                    &mut residual,
                    Some([Some(&mut j0), Some(&mut j1)]),
                );
                let residual = residual.abs();
                debug.mean_res += residual;
                debug.max_residual = debug.max_residual.max(residual);
                if residual > 3.0 {
                    debug.n_big_residuals += 1;
                }
                let (mut field, mut norm) = (0.0, 0.0);
                let (occ, grad) = factor.non_zero_field_and_gradient(
                    [p0.coeffs(), p1.coeffs()],
                    &mut field,
                    &mut norm,
                );
                debug.n_nonzero_occ += usize::from(occ);
                debug.n_nonzero_grad += usize::from(grad);
                debug.mean_field = f64::from(debug.mean_field as f32 + field);
                debug.min_field = debug.min_field.min(f64::from(field));
                debug.max_field = debug.max_field.max(f64::from(field));
                debug.mean_grad = f64::from(debug.mean_grad as f32 + norm);
                debug.max_grad = debug.max_grad.max(f64::from(norm));
            }
        }
        if self.debug_lidar_residuals {
            if !points.is_empty() {
                debug.mean_res /= points.len() as f64;
            }
            debug.mean_field = f64::from(debug.mean_field as f32 / points.len() as f32);
            debug.mean_grad = f64::from(debug.mean_grad as f32 / points.len() as f32);
            self.lidar_debug_infos.push(debug);
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn write_lidar_debug_statistics_csv(&self, prefix: &str) -> Result<()> {
        let mut file = std::fs::File::create(format!("{prefix}-lidar-info.csv"))
            .map_err(|error| io_error(&error))?;
        writeln!(file,"# frame id, reference frame id, n_residual, n_nonzro_occ, n_non_zero_grad, mean res., max res., n_big_residuals, mean occ, min occ, max occ, mean grad norm, max grad norm").map_err(|error| io_error(&error))?;
        for d in &self.lidar_debug_infos {
            writeln!(
                file,
                "{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}",
                d.frame_id,
                d.reference_id,
                d.n_residuals,
                d.n_nonzero_occ,
                d.n_nonzero_grad,
                d.mean_res,
                d.max_residual,
                d.n_big_residuals,
                d.mean_field,
                d.min_field,
                d.max_field,
                d.mean_grad,
                d.max_grad
            )
            .map_err(|error| io_error(&error))?;
        }
        file.flush().map_err(|error| io_error(&error))
    }
}
pub struct RigidResult {
    pub r: Matrix3<f64>,
    pub t: Vector3<f64>,
    pub inlier_ratio: f64,
    pub inliers: Vec<usize>,
}
#[must_use]
pub fn umeyama_transform(gps: &[Vector3<f64>], world: &[Vector3<f64>]) -> Transformation {
    if gps.len() < 3 || gps.len() != world.len() {
        return Transformation::identity();
    }
    let cg = gps.iter().fold(Vector3::zeros(), |s, p| s + p) / gps.len() as f64;
    let cw = world.iter().fold(Vector3::zeros(), |s, p| s + p) / world.len() as f64;
    let h = world.iter().zip(gps).fold(Matrix3::zeros(), |h, (w, g)| {
        h + (w - cw) * (g - cg).transpose()
    });
    let a = h[(0, 1)] - h[(1, 0)];
    let b = h[(0, 0)] + h[(1, 1)];
    let theta = std::f64::consts::FRAC_PI_2 - b.atan2(a);
    let r = Matrix3::new(
        theta.cos(),
        -theta.sin(),
        0.0,
        theta.sin(),
        theta.cos(),
        0.0,
        0.0,
        0.0,
        1.0,
    );
    Transformation::new(cg - r * cw, quaternion_from_rotation_matrix(&r))
}
/// Unique uniform subsets use the reference seed 42.
/// # Errors
/// Returns an error if required graph states, parameter blocks or measurements are absent,
/// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
pub fn estimate_rigid_ransac(
    gps: &[Vector3<f64>],
    world: &[Vector3<f64>],
    iterations: usize,
    n_points: usize,
    threshold: f64,
    required_ratio: f64,
) -> Result<Option<RigidResult>> {
    if gps.len() < 2 * n_points {
        return Ok(None);
    }
    if gps.len() != world.len() {
        return Err(graph_error("point cloud lengths differ"));
    }
    let mut rng = <rand::rngs::StdRng as rand::SeedableRng>::seed_from_u64(42);
    let mut best = None;
    let mut best_ratio = 0.0;
    for _ in 0..iterations {
        let mut indices = Vec::new();
        while indices.len() < n_points {
            let index = rand::RngExt::random_range(&mut rng, 0..gps.len());
            if !indices.contains(&index) {
                indices.push(index);
            }
        }
        let g: Vec<_> = indices.iter().map(|&i| gps[i]).collect();
        let w: Vec<_> = indices.iter().map(|&i| world[i]).collect();
        let transform = umeyama_transform(&g, &w);
        let r = transform.rotation_matrix();
        let t = transform.translation();
        let inliers: Vec<_> = gps
            .iter()
            .zip(world)
            .enumerate()
            .filter_map(|(i, (g, w))| ((g - (r * w + t)).norm() < threshold).then_some(i))
            .collect();
        let ratio = inliers.len() as f64 / gps.len() as f64;
        if ratio > best_ratio {
            best_ratio = ratio;
            best = Some(RigidResult {
                r,
                t,
                inlier_ratio: ratio,
                inliers,
            });
        }
        if ratio > required_ratio {
            break;
        }
    }
    Ok(best)
}
pub(crate) fn inverse3(m: &Matrix3<f64>) -> Matrix3<f64> {
    let c0 = m.column(1).cross(&m.column(2));
    let c1 = m.column(2).cross(&m.column(0));
    let c2 = m.column(0).cross(&m.column(1));
    Matrix3::from_rows(&[c0.transpose(), c1.transpose(), c2.transpose()]) / m.column(0).dot(&c0)
}
fn inverse4(m: &Matrix4<f64>) -> Matrix4<f64> {
    let det = m.determinant();
    Matrix4::from_fn(|i, j| {
        let rows: Vec<_> = (0..4).filter(|&r| r != j).collect();
        let cols: Vec<_> = (0..4).filter(|&c| c != i).collect();
        let minor = Matrix3::from_fn(|r, c| m[(rows[r], cols[c])]).determinant();
        if (i + j) % 2 == 0 {
            minor / det
        } else {
            -minor / det
        }
    })
}
pub(crate) fn seconds(a: Timestamp, b: Timestamp) -> f64 {
    (a.as_nanosecond() - b.as_nanosecond()) as f64 * 1.0e-9
}
pub(crate) fn normalize(v: Vector3<f64>) -> Vector3<f64> {
    let squared = v.norm_squared();
    if squared > 0.0 { v / squared.sqrt() } else { v }
}
pub(crate) fn pose_isometry(p: &Transformation) -> nalgebra::Isometry3<f64> {
    nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::from(p.translation()),
        nalgebra::UnitQuaternion::new_unchecked(p.quaternion()),
    )
}
#[track_caller]
pub(crate) fn graph_error(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("ViGraph")
}
pub(crate) fn io_error(error: &std::io::Error) -> Error {
    Error::new(ErrorKind::Internal, error.to_string()).with_operation("ViGraph::io")
}

pub(crate) fn quaternion_angle_axis(q: &Quaternion<f64>) -> Vector3<f64> {
    let mut n = q.vector().norm();
    if n < f64::EPSILON {
        let scale = q.vector().iter().map(|x| x.abs()).fold(0.0, f64::max);
        n = if scale > 0.0 {
            scale * (q.vector() / scale).norm()
        } else {
            0.0
        };
    }
    if n > 0.0 {
        let angle = 2.0 * n.atan2(q.w.abs());
        let divisor = if q.w < 0.0 { -n } else { n };
        (q.vector() / divisor) * angle
    } else {
        Vector3::zeros()
    }
}
pub(crate) fn quaternion_angular_distance(relative: &Quaternion<f64>) -> f64 {
    2.0 * relative.vector().norm().atan2(relative.w.abs())
}

mod rust_solver;
pub use rust_solver::RustGraphSolver;

impl SolverUpdate {
    /// Commits mutable preintegration caches along with a completed numerical job.
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn restore_factor_caches(&self, problem: &GraphProblem) -> Result<()> {
        for (id, cache) in &self.caches {
            let r = problem
                .residuals
                .get(id)
                .ok_or_else(|| graph_error("solver cache residual absent"))?;
            match (cache, r.factor.as_ref()) {
                (SolverFactorCache::Imu(source), GraphFactor::Imu(target)) => {
                    *target.borrow_mut() = source.as_ref().clone();
                }
                (SolverFactorCache::Gps(source), GraphFactor::Gps(target)) => {
                    *target.borrow_mut() = source.as_ref().clone();
                }
                _ => return Err(graph_error("solver cache residual type differs")),
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn landmark_hessian_matches_analytic_stereo_perspective_information() {
        use crate::cameras::{no_distortion::NoDistortion, pinhole_camera::PinholeCamera};
        use std::sync::Arc;

        let mut graph = ViGraph::new();
        let landmark = LandmarkId::new(1);
        graph
            .add_landmark(landmark, Vector4::new(2., 1., 4., 1.), true)
            .unwrap();
        assert_eq!(
            graph.get_landmark_hessian(landmark).unwrap(),
            Matrix3::zeros()
        );
        assert!(graph.get_landmark_hessian(LandmarkId::new(99)).is_err());
        let camera = Arc::new(PinholeCamera::new(
            (320, 240),
            (8., 12., 160., 120.),
            NoDistortion,
            0,
        ));
        for id in 1..=2 {
            let stamp = Timestamp::UNIX_EPOCH;
            let pose = Transformation::new(
                Vector3::new((id - 1) as f64, 0., 0.),
                Quaternion::identity(),
            );
            graph.states.insert(
                StateId::new(id),
                ComponentState {
                    pose: Some(Rc::new(RefCell::new(PoseParameterBlock::new(
                        &pose, id, stamp,
                    )))),
                    extrinsics: vec![Some(Rc::new(RefCell::new(PoseParameterBlock::new(
                        &Transformation::identity(),
                        id + 10,
                        stamp,
                    ))))],
                    ..Default::default()
                },
            );
            let factor = ReprojectionError::new(
                camera.clone(),
                0,
                Vector2::new(160., 120.),
                Matrix2::identity(),
            );
            graph
                .add_external_observation(
                    &factor,
                    landmark,
                    KeypointIdentifier::new(id, 0, 0),
                    false,
                )
                .unwrap();
        }
        let expected = Matrix3::new(8., 0., -3., 0., 18., -4.5, -3., -4.5, 2.375);
        assert!((graph.get_landmark_hessian(landmark).unwrap() - expected).amax() < 1e-12);
    }

    #[test]
    fn copied_states_register_only_residual_parameters_in_ambient_space() {
        let id = StateId::new(1);
        let stamp = Timestamp::UNIX_EPOCH;
        let block = || {
            Rc::new(RefCell::new(PoseParameterBlock::new(
                &Transformation::identity(),
                id.value(),
                stamp,
            )))
        };
        let pose = block();
        let mut source = ViGraph::new();
        source.states.insert(
            id,
            ComponentState {
                timestamp: Some(stamp),
                pose: Some(pose.clone()),
                speed_and_bias: Some(Rc::new(RefCell::new(SpeedAndBiasParameterBlock::new(
                    SpeedAndBias::zeros(),
                    id.value(),
                    stamp,
                )))),
                extrinsics: vec![Some(block())],
                ..Default::default()
            },
        );
        let prior = source.problem.add(
            GraphFactor::PosePrior(Box::new(PoseError::new(
                nalgebra::Isometry3::identity(),
                SMatrix::identity(),
            ))),
            vec![Parameter::Pose(pose)],
            None,
        );
        source.state_info.insert(
            id,
            GraphState {
                pose_prior: Some(prior),
                ..Default::default()
            },
        );
        let mut copy = ViGraph::new();
        assert!(copy.add_states_from_other(id, &source).unwrap());
        let key = Parameter::Pose(copy.pose_block(id).unwrap()).key();
        assert!(matches!(
            copy.problem.parameters[&key].1,
            ManifoldKind::Euclidean
        ));
        assert_eq!(copy.problem.parameters.len(), 1);
        let ext = Parameter::Pose(copy.extrinsics_block(id, 0).unwrap());
        assert!(!copy.problem.parameters.contains_key(&ext.key()));
    }
}
