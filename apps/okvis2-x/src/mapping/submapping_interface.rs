/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Pose-synchronised depth/LiDAR submaps, overlap selection and alignment constraints.
use crate::{
    config::parameters::ViParameters,
    frames::frame_typedefs::StateId,
    kinematics::transformation::Transformation,
    mapping::{
        config_mapping::SubMapConfig,
        depth_utils::register_depth,
        map_typedefs::SupereightMapType,
        object_mapping::{Descriptor, MeshFace, ObjectMap, ObjectMapBackend},
        submapping_utils::{
            MeanOccupancyMap, Safe, downsample_point_cloud, downsample_points_uncertainty,
        },
        voxel_grid_filter::{VoxelHashMap, VoxelScalar},
    },
    output::trajectory_output::TopViewPainter,
    runtime::vi_interface::{State, TrackingState, Trajectory},
    sensors::{
        measurements::{
            CameraMeasurement, ImuMeasurement, ImuSensorReadings, LidarMeasurement,
            LidarSensorReadings,
        },
        vi_sensor_base::BoundedQueue,
    },
};
use jiff::Timestamp;
use nalgebra::{DVector, Matrix3, Matrix4, Quaternion, Vector2, Vector3};
use purecv::core::{DynamicData, DynamicMatrix, Matrix};
use std::{
    collections::{BTreeMap, BTreeSet, HashSet, VecDeque},
    fs,
    io::{self, Write},
    path::{Path, PathBuf},
    sync::{
        Arc, Condvar, Mutex,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    thread::{self, JoinHandle},
    time::Duration,
};
pub type DepthFrames = BTreeMap<usize, Vec<CameraMeasurement>>;
pub type RayVector = Vec<(Matrix4<f32>, Vector3<f32>)>;
pub type PosedDepthFrames = BTreeMap<usize, Vec<(Matrix4<f32>, CameraMeasurement)>>;
#[derive(Clone)]
pub struct OkvisUpdate {
    pub latest_state: State,
    pub tracking_state: TrackingState,
    pub affected_states: BTreeSet<StateId>,
    pub updated_states: Arc<BTreeMap<StateId, State>>,
}
#[derive(Clone)]
pub struct SiFrameData {
    pub id: u64,
    pub t_ws: Transformation,
    pub covisibles: BTreeSet<StateId>,
}
#[derive(Clone)]
pub struct SupereightFrames {
    pub keyframe_id: u64,
    pub frame_data: BTreeMap<u64, SiFrameData>,
    pub loop_closure: bool,
    pub state_counter: u32,
    pub rays: RayVector,
    pub depth_frames: Vec<PosedDepthFrames>,
}
#[derive(Clone, Debug)]
pub struct PinholeSensor {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub width: usize,
    pub height: usize,
    pub near: f32,
    pub far: f32,
    pub t_sc: Transformation,
}
impl PinholeSensor {
    #[must_use]
    pub fn back_project(&self, x: usize, y: usize) -> Vector3<f32> {
        Vector3::new(
            (x as f32 - self.cx) / self.fx,
            (y as f32 - self.cy) / self.fy,
            1.0,
        )
    }
    #[must_use]
    pub fn intrinsics(&self) -> Matrix3<f32> {
        Matrix3::new(self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0)
    }
}
pub struct DepthIntegration<'a> {
    pub depth: &'a Matrix<f32>,
    pub sigma: Option<&'a Matrix<f32>>,
    pub sensor: &'a PinholeSensor,
    pub t_md: Matrix4<f32>,
    pub colour: Option<(&'a Matrix<u8>, &'a PinholeSensor, Matrix4<f32>)>,
    pub segments: Option<&'a Matrix<u16>>,
}
/// Supereight allocation, interpolation, integration and meshing operations.
/// The backend also supplies the mesh/voxel storage consumed by `ObjectMap`.
pub trait SupereightBackend: ObjectMapBackend + Send + 'static {
    type Map: SupereightMapType + MeanOccupancyMap;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn create_map(&mut self, id: u64) -> io::Result<()>;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn map(&self, id: u64) -> io::Result<&Self::Map>;
    fn map_resolution(&self) -> f32;
    fn log_odd_max(&self) -> f32;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn aabb(&self, id: u64) -> io::Result<(Vector3<f32>, Vector3<f32>)>;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn integrate_rays(&mut self, id: u64, counter: u64, rays: &RayVector) -> io::Result<()>;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn integrate_depth(
        &mut self,
        id: u64,
        counter: u64,
        data: DepthIntegration<'_>,
    ) -> io::Result<()>;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn lookup_ids(
        &self,
        id: u64,
        depth: &Matrix<f32>,
        sensor: &PinholeSensor,
        t_md: &Matrix4<f32>,
    ) -> io::Result<Matrix<u16>>;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn cache_mesh(&mut self, id: u64) -> io::Result<()>;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn downsample_depth(&self, original: &Matrix<f32>, factor: usize) -> io::Result<Matrix<f32>>;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn save_depth_png(&self, depth: &Matrix<f32>, path: &Path) -> io::Result<()>;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn save_mesh(&self, mesh: &[MeshFace], path: &Path, t_wm: &Matrix4<f32>) -> io::Result<()>;
    fn colour_mesh_by_id(&self, mesh: &mut [MeshFace]);
}
pub type AlignCallback = Arc<
    dyn Fn(Option<u64>, u64, u64, u64, &[Vector3<f32>], &[f32]) -> io::Result<bool> + Send + Sync,
>;
pub type AlignmentPublishCallback =
    Arc<dyn Fn(Timestamp, &Transformation, &[Vector3<f32>], bool) + Send + Sync>;
pub type AnchoredTrajectoryUpdateCallback =
    Arc<dyn Fn(&State, Arc<BTreeMap<StateId, State>>) + Send + Sync>;
pub type RealTimePublishingCallback =
    Arc<dyn Fn(Timestamp, Vector3<f64>, Quaternion<f64>, Vector3<f64>, Vector3<f64>) + Send + Sync>;
pub type IntegrationPublishCallback = Arc<dyn Fn(&RayVector) + Send + Sync>;
pub type SubmapCallback<B> =
    Arc<dyn Fn(&BTreeMap<u64, Transformation>, &[u64], &ObjectMap<B>) + Send + Sync>;
/// Field-slice visualization state: only `State::t_ws` is meaningful.
/// Other state fields are safe placeholders and must not be published as estimator output.
pub type FieldCallback<B> = Arc<dyn Fn(&State, &BTreeMap<u64, Transformation>, &B) + Send + Sync>;
pub type TrajectoryAnchoringCallback =
    Arc<dyn Fn(&State, &[Matrix4<f32>], &[u64], usize) + Send + Sync>;
struct InputQueues {
    depth: VecDeque<DepthFrames>,
    lidar: VecDeque<LidarMeasurement>,
    states: VecDeque<OkvisUpdate>,
    shutdown: bool,
    downsample_counter: usize,
}
#[derive(Clone)]
pub struct SubmapAlignmentTerm {
    pub frame_a_id: Option<u64>,
    pub frame_b_id: Option<u64>,
    pub points: Vec<Vector3<f32>>,
    pub sigma: Vec<f32>,
    pub voxel_hash: VoxelHashMap,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct SubmapTrigger {
    pub lidar_keyframe_detected: bool,
    pub force_submap: bool,
}
pub struct SubmappingState<B: SupereightBackend> {
    pub object_map: ObjectMap<B>,
    pub poses: BTreeMap<u64, Transformation>,
    pub covisible_frames: BTreeMap<StateId, BTreeSet<StateId>>,
    pub associations: BTreeMap<u64, u64>,
    pub config: SubMapConfig,
    pub image_delay: f64,
    pub lidar_t_sl: Option<Transformation>,
    pub camera_types: BTreeMap<usize, crate::cameras::n_camera_system::CameraType>,
    pub camera_extrinsics: BTreeMap<usize, Transformation>,
    pub cameras: BTreeMap<usize, PinholeSensor>,
    pub align_callback: Option<AlignCallback>,
    pub submap_callback: Option<SubmapCallback<B>>,
    pub field_callback: Option<FieldCallback<B>>,
    pub anchored_update_callback: Option<AnchoredTrajectoryUpdateCallback>,
    pub trajectory_anchoring_callback: Option<TrajectoryAnchoringCallback>,
    pub alignment_publish_callback: Option<AlignmentPublishCallback>,
    pub realtime_publish_callback: Option<RealTimePublishingCallback>,
    pub integration_publish_callback: Option<IntegrationPublishCallback>,
    pub propagated_states: Trajectory,
    pub alignment: SubmapAlignmentTerm,
    pub gt_poses: BTreeMap<i128, Transformation>,
    pub use_gt_poses: bool,
    pub language_descriptor: Option<Descriptor>,
    pub odometry_publishing_rate: f64,
    pub visualize_submaps: bool,
    pub topview: Option<DynamicMatrix>,
    pub top_view_painter: Option<Arc<dyn TopViewPainter + Send + Sync>>,
    latest_state: Option<State>,
    latest_tracking: Option<TrackingState>,
    latest_keyframe: Option<u64>,
    frame: Option<SupereightFrames>,
    previous_keyframe: Option<u64>,
    previous_submap: Option<u64>,
    current_keyframe: Option<u64>,
    keyframe_counter: usize,
    submap_counter: usize,
    num_integrated_lidar: usize,
    num_integrated_depth: usize,
    integration_counter: u64,
    pub submap_trigger: SubmapTrigger,

    ratio_streak: usize,
    updated_maps: BTreeSet<u64>,
    last_publish_time: Timestamp,
    t_bs: Transformation,
    imu: VecDeque<ImuMeasurement>,
    last_integrated_state: Option<u64>,
}
struct MappingCameraCalibration {
    camera_types: BTreeMap<usize, crate::cameras::n_camera_system::CameraType>,
    camera_extrinsics: BTreeMap<usize, Transformation>,
    cameras: BTreeMap<usize, PinholeSensor>,
}
pub struct SubmappingInterface<B: SupereightBackend> {
    pub state: Arc<Mutex<SubmappingState<B>>>,
    inputs: Arc<(Mutex<InputQueues>, Condvar)>,
    frames: Arc<BoundedQueue<SupereightFrames>>,
    pending_frames: Arc<AtomicUsize>,
    finished: Arc<AtomicBool>,
    processing: Arc<AtomicBool>,
    blocking: bool,
    workers: Vec<JoinHandle<io::Result<()>>>,
    worker_error: Arc<Mutex<Option<String>>>,
}
fn invalid(s: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, s.into())
}
fn err(e: impl std::fmt::Debug) -> io::Error {
    io::Error::other(format!("{e:?}"))
}
fn transform(t: &Matrix4<f32>, p: &Vector3<f32>) -> Vector3<f32> {
    (t * p.push(1.0)).fixed_rows::<3>(0).into_owned()
}
fn inverse(t: &Matrix4<f32>) -> Matrix4<f32> {
    let r = t.fixed_view::<3, 3>(0, 0).transpose();
    let v = -r * t.fixed_view::<3, 1>(0, 3);
    let mut out = Matrix4::identity();
    out.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    out.fixed_view_mut::<3, 1>(0, 3).copy_from(&v);
    out
}
/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn depth_mat_to_image(m: &DynamicMatrix) -> io::Result<Matrix<f32>> {
    match &m.data {
        DynamicData::F32(d) if d.channels == 1 => Ok(d.clone()),
        _ => Err(invalid("Only implemented for CV_32FC1")),
    }
}
/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn rgb_mat_to_image(m: &DynamicMatrix) -> io::Result<Matrix<u8>> {
    match &m.data {
        DynamicData::U8(d) if d.channels == 3 => Ok(d.clone()),
        _ => Err(invalid("Only implemented for CV_8UC3")),
    }
}
fn scaled_nearest(m: &DynamicMatrix, ratio: usize, scale: f32) -> io::Result<DynamicMatrix> {
    if ratio == 0 {
        return Err(invalid("zero image downsampling ratio"));
    }
    macro_rules! c {
        ($d:expr,$v:ident,$integer:expr,$convert:expr) => {{
            let d = $d;
            let rows = (d.rows as f64 / ratio as f64).round_ties_even() as usize;
            let cols = (d.cols as f64 / ratio as f64).round_ties_even() as usize;
            let mut v = Vec::with_capacity(rows * cols * d.channels);
            for y in 0..rows {
                for x in 0..cols {
                    for k in 0..d.channels {
                        let value =
                            ($convert)(d.data[(y * ratio * d.cols + x * ratio) * d.channels + k])
                                * f64::from(scale);
                        v.push(if $integer {
                            value.round_ties_even()
                        } else {
                            value
                        } as _);
                    }
                }
            }
            DynamicMatrix {
                data: DynamicData::$v(Matrix::from_vec(rows, cols, d.channels, v)),
            }
        }};
    }
    Ok(match &m.data {
        DynamicData::U8(d) => c!(d, U8, true, f64::from),
        DynamicData::I8(d) => c!(d, I8, true, f64::from),
        DynamicData::U16(d) => c!(d, U16, true, f64::from),
        DynamicData::I16(d) => c!(d, I16, true, f64::from),
        DynamicData::I32(d) => c!(d, I32, true, f64::from),
        DynamicData::F32(d) => c!(d, F32, false, f64::from),
        DynamicData::F64(d) => c!(d, F64, false, f64::from),
    })
}
impl<B: SupereightBackend> SubmappingInterface<B> {
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn new(config: SubMapConfig, parameters: &ViParameters, backend: B) -> io::Result<Self> {
        if config.sensor_measurement_downsampling == 0 || config.depth_image_res_downsampling == 0 {
            return Err(invalid("zero measurement/image downsampling"));
        }
        let MappingCameraCalibration {
            camera_types,
            camera_extrinsics,
            cameras,
        } = Self::mapping_camera_calibration(parameters, &config)?;
        if !cameras.is_empty() && parameters.lidar.is_some() {
            return Err(invalid(
                "LiDAR and mapping cameras cannot be enabled together",
            ));
        }
        let alignment = SubmapAlignmentTerm {
            frame_a_id: None,
            frame_b_id: None,
            points: Vec::new(),
            sigma: Vec::new(),
            voxel_hash: VoxelHashMap::new(f64::from(config.voxel_grid_resolution), 0.0, 1),
        };
        let state = SubmappingState {
            object_map: ObjectMap::new(backend),
            poses: BTreeMap::new(),
            covisible_frames: BTreeMap::new(),
            associations: BTreeMap::new(),
            config,
            image_delay: parameters.camera.image_delay,
            lidar_t_sl: parameters.lidar.as_ref().map(|l| l.t_sl.clone()),
            camera_types,
            camera_extrinsics,
            cameras,
            align_callback: None,
            submap_callback: None,
            field_callback: None,
            anchored_update_callback: None,
            trajectory_anchoring_callback: None,
            alignment_publish_callback: None,
            realtime_publish_callback: None,
            integration_publish_callback: None,
            propagated_states: Trajectory::default(),
            alignment,
            gt_poses: BTreeMap::new(),
            use_gt_poses: false,
            language_descriptor: None,
            odometry_publishing_rate: 40.0,
            visualize_submaps: false,
            topview: None,
            top_view_painter: None,
            latest_state: None,
            latest_tracking: None,
            latest_keyframe: None,
            frame: None,
            previous_keyframe: None,
            previous_submap: None,
            current_keyframe: None,
            keyframe_counter: 0,
            submap_counter: 0,
            num_integrated_lidar: 0,
            num_integrated_depth: 0,
            integration_counter: 0,
            submap_trigger: crate::mapping::submapping_interface::SubmapTrigger {
                lidar_keyframe_detected: false,
                force_submap: false,
            },

            ratio_streak: 0,
            updated_maps: BTreeSet::new(),
            last_publish_time: Timestamp::UNIX_EPOCH,
            t_bs: Transformation::identity(),
            imu: VecDeque::new(),
            last_integrated_state: None,
        };
        Ok(Self {
            state: Arc::new(Mutex::new(state)),
            inputs: Arc::new((
                Mutex::new(InputQueues {
                    depth: VecDeque::new(),
                    lidar: VecDeque::new(),
                    states: VecDeque::new(),
                    shutdown: false,
                    downsample_counter: 0,
                }),
                Condvar::new(),
            )),
            frames: Arc::new(BoundedQueue::new(5000)),
            pending_frames: Arc::new(AtomicUsize::new(0)),
            finished: Arc::new(AtomicBool::new(false)),
            processing: Arc::new(AtomicBool::new(false)),
            blocking: false,
            workers: Vec::new(),
            worker_error: Arc::new(Mutex::new(None)),
        })
    }
    fn mapping_camera_calibration(
        parameters: &ViParameters,
        config: &SubMapConfig,
    ) -> io::Result<MappingCameraCalibration> {
        let mut camera_types = BTreeMap::new();
        let mut camera_extrinsics = BTreeMap::new();
        for i in 0..parameters.n_camera_system.num_cameras() {
            let t = parameters.n_camera_system.camera_type(i);
            camera_types.insert(i, t.clone());
            camera_extrinsics.insert(
                i,
                if t.depth_type.need_rectify {
                    parameters.n_camera_system.rectify_t_sc(i)
                } else {
                    parameters.n_camera_system.t_sc(i)
                }
                .as_ref()
                .clone(),
            );
        }
        let mut cameras = BTreeMap::new();
        for i in 0..parameters.n_camera_system.num_cameras() {
            let ct = parameters.n_camera_system.camera_type(i);
            if !ct.is_used_mapping {
                continue;
            }
            let c = if ct.depth_type.need_rectify {
                parameters.n_camera_system.rectify_camera_geometry(i)
            } else {
                parameters.n_camera_system.camera_geometry(i)
            };
            let t_sc = parameters.n_camera_system.t_sc(i);
            let mut intr = DVector::zeros(0);
            c.get_intrinsics(&mut intr);
            if intr.len() < 4 {
                return Err(invalid("mapping camera has fewer than four intrinsics"));
            }
            let r = config.depth_image_res_downsampling as f32;
            cameras.insert(
                i,
                PinholeSensor {
                    fx: intr[0] as f32 / r,
                    fy: intr[1] as f32 / r,
                    cx: (intr[2] as f32 + 0.5) / r - 0.5,
                    cy: (intr[3] as f32 + 0.5) / r - 0.5,
                    width: c.image_width() as usize / config.depth_image_res_downsampling,
                    height: c.image_height() as usize / config.depth_image_res_downsampling,
                    near: config.near_plane,
                    far: config.far_plane,
                    t_sc: t_sc.as_ref().clone(),
                },
            );
        }
        Ok(MappingCameraCalibration {
            camera_types,
            camera_extrinsics,
            cameras,
        })
    }

    pub fn set_blocking(&mut self, b: bool) {
        if !self.workers.is_empty() {
            log::warn!("blocking mode requires idle workers");
            return;
        }
        self.blocking = b;
        self.frames = Arc::new(BoundedQueue::new(if b { 1 } else { 5000 }));
    }
    pub fn set_t_bs(&self, t: Transformation) {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .t_bs = t;
    }
    pub fn set_language_embedding(&self, d: Descriptor) {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .language_descriptor = Some(d);
    }
    pub fn set_use_gt_poses(&self, b: bool) {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .use_gt_poses = b;
    }
    pub fn add_gt_pose(&self, t: Timestamp, p: Transformation) {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .gt_poses
            .insert(t.as_nanosecond(), p);
    }
    #[must_use]
    pub fn get_gt_pose(&self, t: Timestamp) -> Option<Transformation> {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .gt_poses
            .get(&t.as_nanosecond())
            .cloned()
    }
    #[must_use]
    pub fn input_handle(&self) -> SubmappingInputs<B> {
        SubmappingInputs {
            state: self.state.clone(),
            inputs: self.inputs.clone(),
            blocking: self.blocking,
        }
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn add_lidar_measurement(&self, t: Timestamp, ray: Vector3<f64>) -> io::Result<bool> {
        self.input_handle().add_lidar_measurement(t, ray)
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn add_depth_measurement(&self, f: &mut DepthFrames) -> io::Result<bool> {
        self.input_handle().add_depth_measurement(f)
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn state_update_callback(
        &self,
        s: State,
        t: TrackingState,
        u: Arc<BTreeMap<StateId, State>>,
    ) -> io::Result<bool> {
        self.input_handle().state_update_callback(s, t, u)
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn realtime_predict(
        &self,
        t: Timestamp,
        alpha: Vector3<f64>,
        omega: Vector3<f64>,
    ) -> io::Result<bool> {
        let mut s = self
            .state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        s.imu.push_back(ImuMeasurement::new(
            t,
            ImuSensorReadings::new(omega, alpha),
            -1,
        ));
        let mut state = None;
        while let Some(m) = s.imu.pop_front() {
            if let Some(p) = s
                .propagated_states
                .add_imu_measurement(
                    m.time_stamp,
                    m.measurement.accelerometers,
                    m.measurement.gyroscopes,
                )
                .map_err(err)?
            {
                state = Some(p);
            }
        }
        let Some(mut state) = state else {
            return Ok(false);
        };
        let ready = (state.timestamp.as_nanosecond() - s.last_publish_time.as_nanosecond()) as f64
            * 1e-9
            >= 1.0 / s.odometry_publishing_rate;
        if let Some(cb) = &s.anchored_update_callback
            && ready
        {
            state.id = StateId::default();
            cb(&state, Arc::new(BTreeMap::new()));
        }
        if let Some(cb) = s.realtime_publish_callback.clone() {
            if !ready {
                return Ok(false);
            }
            s.last_publish_time = state.timestamp;
            let t_sb = s.t_bs.inverse();
            let world_from_body = state.t_ws.compose(&t_sb);
            let v = world_from_body.inverse().rotate_vector(&state.v_w)
                + s.t_bs
                    .rotate_vector(&t_sb.translation().cross(&state.omega_s));
            let omega = s.t_bs.rotate_vector(&state.omega_s);
            cb(
                state.timestamp,
                world_from_body.translation(),
                world_from_body.quaternion(),
                v,
                omega,
            );
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn check_for_available_data(&self) -> io::Result<bool> {
        let mut s = self
            .state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let (lock, cv) = &*self.inputs;
        let mut q = lock
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let frame = s.assemble_frame(&mut q)?;
        cv.notify_all();
        drop(q);
        drop(s);
        if let Some(f) = frame {
            self.pending_frames.fetch_add(1, Ordering::AcqRel);
            if self.blocking {
                if !self.frames.push_blocking(f) {
                    self.pending_frames.fetch_sub(1, Ordering::AcqRel);
                    return Ok(false);
                }
            } else if self.frames.push_dropping(f) {
                self.pending_frames.fetch_sub(1, Ordering::AcqRel);
            }
            return Ok(true);
        }
        Ok(false)
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn process_supereight_frame(&self) -> io::Result<bool> {
        let Some(f) = self.frames.try_pop() else {
            return Ok(false);
        };
        self.processing.store(true, Ordering::Release);
        let backlog = self
            .pending_frames
            .load(Ordering::Acquire)
            .saturating_sub(1);
        let r = self
            .state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .integrate_frame(f, backlog);
        self.pending_frames.fetch_sub(1, Ordering::AcqRel);
        self.processing.store(false, Ordering::Release);
        r?;
        Ok(true)
    }
    fn check_worker_error(&self) -> io::Result<()> {
        if let Some(e) = &*self
            .worker_error
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
        {
            return Err(io::Error::other(e.clone()));
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn start(&mut self) -> io::Result<bool> {
        if !self.workers.is_empty() {
            return Err(invalid("submapping workers already exist"));
        }
        self.start_assembly_worker();
        let (state, frames, pending, finished, processing) = (
            self.state.clone(),
            self.frames.clone(),
            self.pending_frames.clone(),
            self.finished.clone(),
            self.processing.clone(),
        );
        let failures = self.worker_error.clone();
        let input_failure = self.inputs.clone();
        let frame_failure = frames.clone();
        let finish_failure = finished.clone();
        self.workers.push(thread::spawn(move || {
            let result = (|| {
                while !finished.load(Ordering::Acquire) {
                    let Some(f) = frames.pop() else { break };
                    processing.store(true, Ordering::Release);
                    let r = state
                        .lock()
                        .unwrap_or_else(std::sync::PoisonError::into_inner)
                        .integrate_frame(f, pending.load(Ordering::Acquire).saturating_sub(1));
                    pending.fetch_sub(1, Ordering::AcqRel);
                    processing.store(false, Ordering::Release);
                    r?;
                }
                Ok(())
            })();
            if let Err(e) = &result {
                *failures
                    .lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner) = Some(format!("{e}"));
                finish_failure.store(true, Ordering::Release);
                frame_failure.shutdown();
                let (lock, cv) = &*input_failure;
                lock.lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner)
                    .shutdown = true;
                cv.notify_all();
            }
            result
        }));
        Ok(true)
    }
    fn start_assembly_worker(&mut self) {
        let (state, inputs, frames, pending, finished) = (
            self.state.clone(),
            self.inputs.clone(),
            self.frames.clone(),
            self.pending_frames.clone(),
            self.finished.clone(),
        );
        let blocking = self.blocking;
        let failures = self.worker_error.clone();
        let input_failure = inputs.clone();
        let frame_failure = frames.clone();
        let finish_failure = finished.clone();
        self.workers.push(thread::spawn(move || {
            let result = (|| {
                while !finished.load(Ordering::Acquire) {
                    let frame = {
                        let mut s = state
                            .lock()
                            .unwrap_or_else(std::sync::PoisonError::into_inner);
                        let (lock, cv) = &*inputs;
                        let mut q = lock
                            .lock()
                            .unwrap_or_else(std::sync::PoisonError::into_inner);
                        let frame = s.assemble_frame(&mut q)?;
                        cv.notify_all();
                        frame
                    };
                    if let Some(f) = frame {
                        pending.fetch_add(1, Ordering::AcqRel);
                        if blocking {
                            if !frames.push_blocking(f) {
                                pending.fetch_sub(1, Ordering::AcqRel);
                                break;
                            }
                        } else if frames.push_dropping(f) {
                            pending.fetch_sub(1, Ordering::AcqRel);
                        }
                    } else {
                        let (lock, cv) = &*inputs;
                        let q = lock
                            .lock()
                            .unwrap_or_else(std::sync::PoisonError::into_inner);
                        if q.shutdown {
                            break;
                        }
                        drop(
                            cv.wait_timeout(q, Duration::from_millis(1))
                                .unwrap_or_else(std::sync::PoisonError::into_inner),
                        );
                    }
                }
                Ok(())
            })();
            if let Err(e) = &result {
                *failures
                    .lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner) = Some(format!("{e}"));
                finish_failure.store(true, Ordering::Release);
                frame_failure.shutdown();
                let (lock, cv) = &*input_failure;
                lock.lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner)
                    .shutdown = true;
                cv.notify_all();
            }
            result
        }));
    }

    /// Checks worker failures without flushing pending depth or changing integration state.
    /// # Errors
    /// Returns a processing or integration worker error.
    pub fn check_health(&self) -> io::Result<()> {
        self.check_worker_error()
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn finished_integrating(&self) -> io::Result<bool> {
        self.check_worker_error()?;
        let mut s = self
            .state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let (lock, cv) = &*self.inputs;
        let mut q = lock
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if !q.states.is_empty()
            || self.pending_frames.load(Ordering::Acquire) > 0
            || s.poses.is_empty()
            || self.processing.load(Ordering::Acquire)
        {
            return Ok(false);
        }
        if s.lidar_t_sl.is_none()
            && let Some(f) = s.frame.take()
        {
            self.pending_frames.fetch_add(1, Ordering::AcqRel);
            drop(q);
            drop(s);
            if self.frames.push_dropping(f) {
                self.pending_frames.fetch_sub(1, Ordering::AcqRel);
            }
            return Ok(false);
        }
        s.map_to_map_factors()?;
        q.depth.clear();
        q.lidar.clear();
        cv.notify_all();
        Ok(true)
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn set_finished(&mut self) -> io::Result<()> {
        if self.finished.load(Ordering::Acquire) && self.workers.is_empty() {
            return self.check_worker_error();
        }
        self.finished.store(true, Ordering::Release);
        self.frames.shutdown();
        {
            let (lock, cv) = &*self.inputs;
            lock.lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .shutdown = true;
            cv.notify_all();
        }
        let mut failure = None;
        for w in self.workers.drain(..) {
            match w.join() {
                Ok(Ok(())) => {}
                Ok(Err(e)) => {
                    failure.get_or_insert(e);
                }
                Err(_) => {
                    failure.get_or_insert_with(|| io::Error::other("submapping worker panicked"));
                }
            }
        }
        let mut s = self
            .state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Some(id) = s.previous_keyframe {
            s.object_map.backend.cache_mesh(id)?;
            s.object_map.finish_submap_objects(id).map_err(err)?;
        }
        if let Some(e) = failure {
            return Err(e);
        }
        Ok(())
    }
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.finished.load(Ordering::Acquire)
    }
    #[must_use]
    pub fn get_supereight_queue_size(&self) -> usize {
        self.pending_frames.load(Ordering::Acquire)
    }
    #[must_use]
    pub fn publish_submap_top_view(&self) -> Option<DynamicMatrix> {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .topview
            .take()
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn save_all_submap_meshes(&self) -> io::Result<()> {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .save_all_submap_meshes()
    }
}
/// Shared sensor ingress; blocking producers do not own the processing host lock.
pub struct SubmappingInputs<B: SupereightBackend> {
    state: Arc<Mutex<SubmappingState<B>>>,
    inputs: Arc<(Mutex<InputQueues>, Condvar)>,
    blocking: bool,
}
impl<B: SupereightBackend> Clone for SubmappingInputs<B> {
    fn clone(&self) -> Self {
        Self {
            state: self.state.clone(),
            inputs: self.inputs.clone(),
            blocking: self.blocking,
        }
    }
}
impl<B: SupereightBackend> SubmappingInputs<B> {
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn add_lidar_measurement(&self, t: Timestamp, ray: Vector3<f64>) -> io::Result<bool> {
        let ds = self
            .state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .config
            .sensor_measurement_downsampling;
        let (lock, cv) = &*self.inputs;
        let mut q = lock
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if ds == 0 {
            return Err(invalid("zero sensor downsampling"));
        }
        q.downsample_counter += 1;
        if q.downsample_counter % ds != 0 {
            return Ok(true);
        }
        q.downsample_counter = 0;
        while self.blocking && q.lidar.len() >= 100_000_000 && !q.shutdown {
            q = cv
                .wait(q)
                .unwrap_or_else(std::sync::PoisonError::into_inner);
        }
        if q.shutdown {
            return Ok(false);
        }
        if q.lidar.len() >= 100_000_000 {
            q.lidar.pop_front();
        }
        q.lidar.push_back(LidarMeasurement::new(
            t,
            LidarSensorReadings::from_ray(ray),
            -1,
        ));
        cv.notify_all();
        Ok(true)
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn add_depth_measurement(&self, frames: &mut DepthFrames) -> io::Result<bool> {
        let (ds, delay) = {
            let s = self
                .state
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            (s.config.sensor_measurement_downsampling, s.image_delay)
        };
        let (lock, cv) = &*self.inputs;
        let mut q = lock
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if ds == 0 {
            return Err(invalid("zero sensor downsampling"));
        }
        q.downsample_counter += 1;
        if q.downsample_counter % ds != 0 {
            return Ok(true);
        }
        q.downsample_counter = 0;
        for c in frames.values_mut() {
            for m in c {
                m.time_stamp = Timestamp::from_nanosecond(
                    m.time_stamp.as_nanosecond() - (delay * 1e9) as i128,
                )
                .map_err(err)?;
            }
        }
        while self.blocking && q.depth.len() >= 4 && !q.shutdown {
            q = cv
                .wait(q)
                .unwrap_or_else(std::sync::PoisonError::into_inner);
        }
        if q.shutdown {
            return Ok(false);
        }
        if q.depth.len() >= 4 {
            q.depth.pop_front();
        }
        q.depth.push_back(frames.clone());
        cv.notify_all();
        Ok(true)
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn state_update_callback(
        &self,
        latest: State,
        tracking: TrackingState,
        updates: Arc<BTreeMap<StateId, State>>,
    ) -> io::Result<bool> {
        {
            let mut s = self
                .state
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if tracking.is_lidar_keyframe && s.num_integrated_lidar > s.config.submap_min_frames {
                s.submap_trigger.lidar_keyframe_detected = true;
                s.submap_trigger.force_submap = true;
                s.ratio_streak = 0;
            }
            s.latest_state = Some(latest.clone());
            s.latest_tracking = Some(tracking.clone());
        }
        let update = OkvisUpdate {
            latest_state: latest,
            tracking_state: tracking,
            affected_states: updates.keys().copied().collect(),
            updated_states: updates,
        };
        let (lock, cv) = &*self.inputs;
        let mut q = lock
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let capacity = if self.blocking { 1 } else { 100_000 };
        while self.blocking && q.states.len() >= capacity && !q.shutdown {
            q = cv
                .wait(q)
                .unwrap_or_else(std::sync::PoisonError::into_inner);
        }
        if q.shutdown {
            return Ok(false);
        }
        let dropped = q.states.len() >= capacity;
        if dropped {
            q.states.pop_front();
        }
        q.states.push_back(update);
        cv.notify_all();
        Ok(self.blocking || dropped)
    }
}
impl<B: SupereightBackend> Drop for SubmappingInterface<B> {
    fn drop(&mut self) {
        if let Err(e) = self.set_finished() {
            log::error!("submapping shutdown: {e}");
        }
    }
}
impl<B: SupereightBackend> SubmappingState<B> {
    fn frame_updater(&mut self, u: &OkvisUpdate) {
        if u.tracking_state.is_keyframe || self.latest_keyframe.is_none() {
            self.latest_keyframe = Some(u.latest_state.id.value());
        }
        let new: BTreeMap<_, _> = u
            .updated_states
            .iter()
            .map(|(id, s)| {
                (
                    id.value(),
                    SiFrameData {
                        id: id.value(),
                        t_ws: s.t_ws.clone(),
                        covisibles: s.covisible_frame_ids.clone(),
                    },
                )
            })
            .collect();
        if let Some(frame) = &mut self.frame {
            frame.loop_closure = u.tracking_state.backend.recognised_place;
            frame.frame_data.extend(new);
        } else {
            self.frame = Some(SupereightFrames {
                keyframe_id: self.latest_keyframe.unwrap_or(u.latest_state.id.value()),
                frame_data: new,
                loop_closure: u.tracking_state.backend.recognised_place,
                state_counter: 0,
                rays: Vec::new(),
                depth_frames: Vec::new(),
            });
        }
    }
    fn update_trajectory(&mut self, u: &mut OkvisUpdate) -> io::Result<()> {
        self.propagated_states
            .update(&u.tracking_state, &u.updated_states, &mut u.affected_states)
            .map_err(err)?;
        if let Some(cb) = &self.anchored_update_callback {
            cb(&u.latest_state, u.updated_states.clone());
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn predict(
        &mut self,
        t: Timestamp,
        t_sc: &Transformation,
    ) -> io::Result<Option<Transformation>> {
        if self.use_gt_poses
            && let Some(p) = self.gt_poses.get(&t.as_nanosecond())
        {
            return Ok(Some(p.compose(t_sc)));
        }
        Ok(self
            .propagated_states
            .get_state(t)
            .map_err(err)?
            .map(|s| s.t_ws.compose(t_sc)))
    }
    fn assemble_frame(&mut self, queue: &mut InputQueues) -> io::Result<Option<SupereightFrames>> {
        let Some(newest) = queue
            .states
            .back()
            .map(|state| state.latest_state.timestamp)
        else {
            return Ok(None);
        };
        let lidar = self.lidar_t_sl.is_some()
            && queue
                .lidar
                .front()
                .is_some_and(|measurement| measurement.time_stamp <= newest);
        let newest_data = if self.cameras.is_empty() {
            queue
                .lidar
                .front()
                .map(|measurement| measurement.time_stamp)
        } else {
            queue.depth.front().and_then(|depth_value| {
                depth_value
                    .values()
                    .flat_map(|velocity| velocity.iter())
                    .map(|measurement| measurement.time_stamp)
                    .max()
            })
        };
        let depth = !self.cameras.is_empty()
            && newest_data.is_some_and(|timestamp| timestamp <= newest)
            && !queue.depth.is_empty();
        if !lidar && !depth {
            while let Some(mut u) = queue.states.pop_front() {
                self.update_trajectory(&mut u)?;
                self.frame_updater(&u);
                if u.latest_state.timestamp >= newest {
                    break;
                }
            }
            return Ok(None);
        }
        let target = newest_data.ok_or_else(|| invalid("no sensor timestamp"))?;
        let current = loop {
            let Some(mut horizontal) = queue.states.pop_front() else {
                return Ok(None);
            };
            self.update_trajectory(&mut horizontal)?;
            if self.frame.is_none() {
                self.frame_updater(&horizontal);
            }
            if horizontal.latest_state.timestamp >= target || queue.states.is_empty() {
                break horizontal;
            }
        };
        let until = current.latest_state.timestamp;
        let mut measurements = Vec::new();
        while lidar
            && queue
                .lidar
                .front()
                .is_some_and(|measurement| measurement.time_stamp <= until)
        {
            measurements.push(
                queue
                    .lidar
                    .pop_front()
                    .ok_or_else(|| invalid("missing LiDAR front"))?,
            );
            if queue
                .lidar
                .front()
                .is_none_or(|measurement| measurement.time_stamp >= until)
            {
                break;
            }
        }
        if !self.assemble_depth_frames(queue, &current, depth)? {
            return Ok(None);
        }
        self.pose_lidar_measurements(measurements)?;
        self.frame_updater(&current);
        Ok(self.frame.take())
    }
    fn pose_lidar_measurements(&mut self, measurements: Vec<LidarMeasurement>) -> io::Result<()> {
        if !measurements.is_empty() {
            let times: Vec<_> = measurements
                .iter()
                .map(|measurement| measurement.time_stamp)
                .collect();
            let mut poses = Vec::new();
            if self
                .propagated_states
                .get_state_poses(&times, &mut poses)
                .map_err(err)?
            {
                let t_sl = self
                    .lidar_t_sl
                    .as_ref()
                    .ok_or_else(|| invalid("LiDAR config missing"))?
                    .matrix()
                    .cast::<f32>();
                let frame = self
                    .frame
                    .as_mut()
                    .ok_or_else(|| invalid("LiDAR frame not initialized"))?;
                for (point, measurement) in poses.iter().zip(measurements) {
                    frame.rays.push((
                        point.cast::<f32>() * t_sl,
                        measurement.measurement.ray_measurement.cast::<f32>(),
                    ));
                }
            }
        }
        Ok(())
    }

    fn assemble_depth_frames(
        &mut self,
        queue: &mut InputQueues,
        current: &OkvisUpdate,
        depth: bool,
    ) -> io::Result<bool> {
        let until = current.latest_state.timestamp;
        if depth {
            while let Some(depth_value) = queue.depth.front() {
                let mapping = *self
                    .cameras
                    .keys()
                    .next()
                    .ok_or_else(|| invalid("missing mapping camera"))?;
                let timestamp = depth_value
                    .get(&mapping)
                    .and_then(|measurement| measurement.first())
                    .map(|measurement| measurement.time_stamp)
                    .ok_or_else(|| invalid("mapping camera data missing"))?;
                if timestamp > until {
                    break;
                }
                let data = queue
                    .depth
                    .pop_front()
                    .ok_or_else(|| invalid("missing depth front"))?;
                let Some(posed) = self.pose_depth_batch(data, current)? else {
                    return Ok(false);
                };
                self.frame
                    .as_mut()
                    .ok_or_else(|| invalid("depth frame not initialized"))?
                    .depth_frames
                    .push(posed);
                if queue
                    .depth
                    .front()
                    .and_then(|depth_value| depth_value.get(&mapping))
                    .and_then(|velocity| velocity.first())
                    .is_none_or(|measurement| measurement.time_stamp >= until)
                {
                    break;
                }
            }
        }
        Ok(true)
    }
    fn pose_depth_batch(
        &mut self,
        data: DepthFrames,
        current: &OkvisUpdate,
    ) -> io::Result<Option<PosedDepthFrames>> {
        let mut posed = PosedDepthFrames::new();
        let mut all = true;
        for (id, measurements) in data {
            let mut t_sc = self
                .camera_extrinsics
                .get(&id)
                .ok_or_else(|| invalid("camera extrinsic missing"))?
                .clone();
            if current.latest_state.is_online_extrinsics == Some(true) {
                t_sc = current
                    .latest_state
                    .extrinsics
                    .get(&(id as u64))
                    .ok_or_else(|| invalid("online extrinsic missing"))?
                    .clone();
            }
            let mut output = Vec::new();
            for mut measurement in measurements {
                let Some(point) = self.predict(measurement.time_stamp, &t_sc)? else {
                    all = false;
                    log::error!(
                        "Depth image dropped at {} camera {id}",
                        measurement.time_stamp
                    );
                    break;
                };
                let ratio = self.config.depth_image_res_downsampling;
                if self
                    .camera_types
                    .get(&id)
                    .is_some_and(|timestamp| timestamp.depth_type.is_depth_camera)
                {
                    if let Some(depth_value) = &measurement.measurement.depth_image {
                        measurement.measurement.depth_image = Some(Arc::new(scaled_nearest(
                            depth_value,
                            ratio,
                            self.config.depth_scaling_factor,
                        )?));
                    }
                    if ratio > 1
                        && let Some(s) = &measurement.measurement.sigma_image
                    {
                        measurement.measurement.sigma_image =
                            Some(Arc::new(scaled_nearest(s, ratio, 1.0)?));
                    }
                }
                if ratio > 1
                    && let Some(im) = &measurement.measurement.image
                {
                    measurement.measurement.image = Some(Arc::new(scaled_nearest(im, ratio, 1.0)?));
                }
                output.push((point.matrix().cast::<f32>(), measurement));
            }
            posed.insert(id, output);
        }
        if !all {
            return Ok(None);
        }
        Ok(Some(posed))
    }

    pub fn obtain_covisibles(&self, id: StateId, depth: usize) -> BTreeSet<u64> {
        let mut maps = BTreeSet::new();
        if self.poses.contains_key(&id.value()) {
            maps.insert(id.value());
        }
        let mut queue = VecDeque::new();
        if let Some(c) = self.covisible_frames.get(&id) {
            for c in c {
                queue.push_back((0, *c));
                if self.poses.contains_key(&c.value()) {
                    maps.insert(c.value());
                }
            }
        }
        let mut checked = BTreeSet::from([id]);
        while let Some((d, c)) = queue.pop_front() {
            if checked.contains(&c) || d > depth {
                continue;
            }
            if let Some(cs) = self.covisible_frames.get(&c) {
                checked.insert(c);
                for n in cs {
                    queue.push_back((d + 1, *n));
                    if self.poses.contains_key(&n.value()) {
                        maps.insert(n.value());
                    }
                }
            }
        }
        maps
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn evaluate_depth_overlap(&self, id: u64, f: &SupereightFrames) -> io::Result<f32> {
        if f.depth_frames.is_empty() || self.previous_keyframe.is_none() {
            return Ok(1.0);
        }
        let t_kw = self
            .poses
            .get(&id)
            .ok_or_else(|| invalid("submap pose missing"))?
            .inverse()
            .matrix()
            .cast::<f32>();
        let map = self.object_map.backend.map(id)?;
        let (mut observed, mut all) = (0usize, 0usize);
        for batch in &f.depth_frames {
            for (&camera, measurements) in batch {
                if !self
                    .camera_types
                    .get(&camera)
                    .is_some_and(|t| t.depth_type.is_depth_camera)
                {
                    continue;
                }
                let sensor = self
                    .cameras
                    .get(&camera)
                    .ok_or_else(|| invalid("depth sensor missing"))?;
                let step = 4.0 * self.object_map.backend.map_resolution();
                let count = ((sensor.far - sensor.near) / step).floor() as usize;
                for (p, m) in measurements {
                    if let Some(im) = &m.measurement.depth_image {
                        let im = depth_mat_to_image(im)?;
                        let t_md = t_kw * p;
                        for x in (0..im.cols).step_by(8) {
                            for y in (0..im.rows).step_by(8) {
                                let ray = sensor.back_project(x, y);
                                let measured = im.data[y * im.cols + x];
                                for z in 0..count {
                                    let d = sensor.near + step * z as f32;
                                    if d > measured {
                                        break;
                                    }
                                    all += 1;
                                    if map
                                        .interp_field_safe(&transform(&t_md, &(ray * d)))
                                        .is_some()
                                    {
                                        observed += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        Ok(if all == 0 {
            1.0
        } else {
            observed as f32 / all as f32
        })
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn evaluate_lidar_overlap(
        &mut self,
        id: u64,
        f: &SupereightFrames,
        threshold: f64,
    ) -> io::Result<f64> {
        if f.rays.is_empty() {
            return Ok(1.0);
        }
        let t_mw = self
            .poses
            .get(&id)
            .ok_or_else(|| invalid("submap pose missing"))?
            .inverse()
            .matrix()
            .cast::<f32>();
        let map = self.object_map.backend.map(id)?;
        let (mut observed, mut actual) = (0usize, f.rays.len());
        for (t, p) in &f.rays {
            let norm = p.norm();
            if norm > self.config.far_plane || norm < self.config.near_plane {
                actual -= 1;
                continue;
            }
            if map.interp_field_safe(&transform(&(t_mw * t), p)).is_some() {
                observed += 1;
            }
        }
        let ratio = observed as f64 / actual as f64;
        if ratio <= threshold && self.ratio_streak > 0 {
            self.ratio_streak += 1;
            self.submap_trigger.lidar_keyframe_detected = true;
            self.ratio_streak = 0;
        } else if ratio <= threshold {
            self.ratio_streak += 1;
        } else {
            self.ratio_streak = 0;
        }
        Ok(ratio)
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn decide_new_submap(&mut self, f: &mut SupereightFrames) -> io::Result<bool> {
        let Some(previous) = self.previous_keyframe else {
            return Ok(true);
        };
        let mut create = false;
        if !self.cameras.is_empty() {
            if f.keyframe_id != previous {
                if self.current_keyframe != Some(f.keyframe_id) {
                    self.keyframe_counter += 1;
                    self.current_keyframe = Some(f.keyframe_id);
                }
                let overlap = if self.submap_counter > 1 {
                    self.evaluate_depth_overlap(
                        self.previous_submap
                            .ok_or_else(|| invalid("previous submap missing"))?,
                        f,
                    )?
                } else {
                    1.0
                };
                if (overlap < self.config.submap_overlap_ratio
                    || self.keyframe_counter > self.config.submap_kf_threshold)
                    && self.num_integrated_depth > self.config.submap_min_frames
                {
                    self.keyframe_counter = 0;
                    create = true;
                } else {
                    f.keyframe_id = previous;
                }
            }
        } else if self.lidar_t_sl.is_some() {
            if f.keyframe_id != previous {
                if self.current_keyframe != Some(f.keyframe_id) {
                    self.keyframe_counter += 1;
                    self.current_keyframe = Some(f.keyframe_id);
                }
                if self.submap_trigger.lidar_keyframe_detected
                    || (self.poses.len() == 1
                        && self.num_integrated_lidar > self.config.submap_min_frames)
                {
                    if self.keyframe_counter >= self.config.submap_kf_threshold
                        || self.submap_trigger.force_submap
                    {
                        self.keyframe_counter = 0;
                        self.submap_trigger.force_submap = false;
                        create = true;
                    } else {
                        f.keyframe_id = previous;
                    }
                } else {
                    if self.num_integrated_lidar > self.config.submap_min_frames {
                        self.evaluate_lidar_overlap(
                            previous,
                            f,
                            f64::from(self.config.submap_overlap_ratio),
                        )?;
                    }
                    if self.submap_trigger.lidar_keyframe_detected
                        && self.keyframe_counter >= self.config.submap_kf_threshold
                    {
                        self.keyframe_counter = 0;
                        create = true;
                    } else {
                        f.keyframe_id = previous;
                    }
                }
            } else if !self.submap_trigger.lidar_keyframe_detected
                && self.num_integrated_lidar > self.config.submap_min_frames
            {
                self.evaluate_lidar_overlap(
                    previous,
                    f,
                    f64::from(self.config.submap_overlap_ratio),
                )?;
            }
        }
        Ok(create)
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn observed_points(
        &self,
        reference_map: u64,
        query_map: u64,
        points: &[Vector3<f32>],
        sigma: Option<&[f32]>,
    ) -> io::Result<(Vec<Vector3<f32>>, Vec<f32>)> {
        let reference_from_query = self
            .poses
            .get(&reference_map)
            .ok_or_else(|| invalid("map A pose missing"))?
            .inverse()
            .compose(
                self.poses
                    .get(&query_map)
                    .ok_or_else(|| invalid("map B pose missing"))?,
            )
            .matrix()
            .cast::<f32>();
        let map = self.object_map.backend.map(reference_map)?;
        let margin = 0.8 * self.object_map.backend.log_odd_max();
        let (mut observed_points, mut observed_sigmas) = (Vec::new(), Vec::new());
        for (index, point) in points.iter().enumerate() {
            let query = transform(&reference_from_query, point);
            let valid = if sigma.is_some() {
                match (
                    map.interp_mean_occupancy(&query, 0, Safe::On),
                    map.grad_mean_occupancy(&query, 0, Safe::On),
                ) {
                    (Some((occupancy, _)), Some((gradient, _))) => {
                        occupancy > -margin && occupancy < margin && gradient.norm() >= 1e-3
                    }
                    _ => false,
                }
            } else {
                map.interp_field_safe(&query).is_some()
            };
            if valid {
                observed_points.push(*point);
                if let Some(sigmas) = sigma {
                    observed_sigmas.push(
                        *sigmas
                            .get(index)
                            .ok_or_else(|| invalid("alignment sigma missing"))?,
                    );
                }
            }
        }
        Ok((observed_points, observed_sigmas))
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn find_most_overlapping_submap(
        &self,
        points: &[Vector3<f32>],
        active: u64,
        previous: u64,
    ) -> io::Result<Option<u64>> {
        let bounds = self.object_map.backend.aabb(active)?;
        let t_aw = self
            .poses
            .get(&active)
            .ok_or_else(|| invalid("active pose missing"))?
            .inverse();
        let covis = self.obtain_covisibles(StateId::new(active), 15);
        let mut candidates = Vec::<(f32, u64)>::new();
        for (&id, pose) in &self.poses {
            if id >= previous || (!self.cameras.is_empty() && !covis.contains(&id)) {
                continue;
            }
            let b = self.object_map.backend.aabb(id)?;
            let b = transform_aabb(b.0, b.1, &t_aw.compose(pose).matrix().cast::<f32>());
            let iou = calculate_iou_3d(bounds.0, bounds.1, b.0, b.1);
            if let Some(c) = candidates.iter_mut().find(|(v, _)| *v == iou) {
                c.1 = id;
            } else {
                candidates.push((iou, id));
            }
        }
        candidates.sort_by(|a, b| b.0.total_cmp(&a.0));
        let top: BTreeSet<_> = candidates
            .iter()
            .take(5)
            .filter(|(iou, _)| *iou > 0.2)
            .map(|(_, id)| *id)
            .collect();
        for id in top {
            if self.observed_points(id, active, points, None)?.0.len() as f64
                > 0.5 * self.config.num_submap_factors as f64
            {
                return Ok(Some(id));
            }
        }
        Ok(None)
    }
    fn add_submap_alignment_factors(&mut self, f: &SupereightFrames) -> io::Result<()> {
        if self.config.factors.use_map2map_factors {
            if self.submap_counter > 2 {
                self.map_to_map_factors()?;
            } else if let (Some(cb), Some(previous)) = (&self.align_callback, self.previous_submap)
            {
                cb(
                    None,
                    previous,
                    0,
                    previous,
                    &self.alignment.points,
                    &vec![0.0; self.alignment.points.len()],
                )?;
            }
            self.alignment.frame_a_id = self.previous_keyframe;
            self.alignment.frame_b_id = Some(f.keyframe_id);
            self.alignment.points.clear();
            self.alignment.sigma.clear();
            self.alignment.voxel_hash.clear();
        } else if self.config.factors.use_map2live_factors
            && let (Some(cb), Some(previous)) = (&self.align_callback, self.previous_submap)
        {
            cb(None, previous, 0, previous, &self.alignment.points, &[])?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn map_to_map_factors(&mut self) -> io::Result<()> {
        let (Some(a), Some(b)) = (self.alignment.frame_a_id, self.alignment.frame_b_id) else {
            return Ok(());
        };
        if self.lidar_t_sl.is_some() {
            let cloud: Vec<_> = self
                .alignment
                .voxel_hash
                .pointcloud()
                .into_iter()
                .map(nalgebra::Matrix::cast::<f32>)
                .collect();
            if cloud.is_empty() {
                return Ok(());
            }
            let most = self
                .find_most_overlapping_submap(&cloud, b, a)?
                .unwrap_or(a);
            let ids = if most == a { vec![a] } else { vec![most, a] };
            for &id in &ids {
                let points = self.observed_points(id, b, &cloud, None)?.0;
                let mut sampled = Vec::new();
                downsample_point_cloud(
                    &points,
                    &mut sampled,
                    (if most == a { 10 } else { 5 }) * self.config.num_submap_factors,
                );
                if id == most
                    && let Some(cb) = &self.alignment_publish_callback
                {
                    cb(
                        Timestamp::from_nanosecond(i128::from(b)).map_err(err)?,
                        &self.poses[&b],
                        &sampled,
                        true,
                    );
                }
                if let Some(cb) = &self.align_callback {
                    cb(
                        Some(id),
                        b,
                        id,
                        b,
                        &sampled,
                        &vec![self.config.sensor_error; sampled.len()],
                    )?;
                }
            }
            self.associations.insert(b, most);
        } else if !self.alignment.points.is_empty() {
            let most = self
                .find_most_overlapping_submap(&self.alignment.points, b, a)?
                .unwrap_or(a);
            let (points, sigma) =
                self.observed_points(most, b, &self.alignment.points, Some(&self.alignment.sigma))?;
            let (mut sampled, mut sigmas) = (Vec::new(), Vec::new());
            if points.len() > 5 * self.config.num_submap_factors {
                if !downsample_points_uncertainty(
                    &points,
                    &sigma,
                    &mut sampled,
                    &mut sigmas,
                    5 * self.config.num_submap_factors,
                ) {
                    return Err(invalid("invalid depth uncertainty sample"));
                }
            } else {
                sampled = points;
                sigmas = sigma;
            }
            if !sampled.is_empty() {
                if let Some(cb) = &self.align_callback {
                    cb(Some(most), b, most, b, &sampled, &sigmas)?;
                }
                if let Some(cb) = &self.alignment_publish_callback {
                    cb(
                        Timestamp::from_nanosecond(i128::from(b)).map_err(err)?,
                        &self.poses[&b],
                        &sampled,
                        true,
                    );
                }
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn update_lidar_align_block(
        &mut self,
        rays: &mut RayVector,
        t_wk: &Transformation,
    ) -> io::Result<()> {
        let t_kw = t_wk.inverse().matrix().cast::<f32>();
        let mut allocated = HashSet::new();
        let mut output = Vec::new();
        let mut points = Vec::new();
        for (t, p) in rays.iter() {
            let keyframe_from_lidar = t_kw * t;
            let pk = transform(&keyframe_from_lidar, p);
            let size = f64::from(self.object_map.backend.map_resolution()) / 2.0;
            let voxel = [
                pk.x.voxel_index(size)
                    .ok_or_else(|| invalid("invalid voxel x"))?,
                pk.y.voxel_index(size)
                    .ok_or_else(|| invalid("invalid voxel y"))?,
                pk.z.voxel_index(size)
                    .ok_or_else(|| invalid("invalid voxel z"))?,
            ];
            if allocated.insert(voxel) {
                output.push((keyframe_from_lidar, *p));
                if self.config.factors.use_map2map_factors {
                    points.push(pk.cast::<f64>());
                }
            }
        }
        *rays = output;
        if !points.is_empty() && !self.alignment.voxel_hash.add_points(&points) {
            return Err(invalid("invalid alignment voxel point"));
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn update_depth_align_block(
        &mut self,
        data: &(Matrix4<f32>, CameraMeasurement),
        t_wk: &Transformation,
        camera: usize,
    ) -> io::Result<()> {
        let depth = depth_mat_to_image(
            data.1
                .measurement
                .depth_image
                .as_ref()
                .ok_or_else(|| invalid("depth image missing"))?,
        )?;
        let sigma = data
            .1
            .measurement
            .sigma_image
            .as_ref()
            .map(|m| depth_mat_to_image(m))
            .transpose()?;
        let sensor = self
            .cameras
            .get(&camera)
            .ok_or_else(|| invalid("sensor missing"))?;
        let t_md = t_wk.inverse().matrix().cast::<f32>() * data.0;
        for x in (0..depth.cols).step_by(20) {
            for y in (0..depth.rows).step_by(20) {
                let z = depth.data[y * depth.cols + x];
                let mut s = self.config.sensor_error;
                if self.config.use_uncertainty
                    && let Some(sigma) = &sigma
                {
                    s = *sigma
                        .data
                        .get(y * depth.cols + x)
                        .ok_or_else(|| invalid("sigma dimensions mismatch"))?;
                    if z < self.config.near_plane || z > self.config.far_plane || s > 3.0 {
                        continue;
                    }
                }
                self.alignment
                    .points
                    .push(transform(&t_md, &(sensor.back_project(x, y) * z)));
                self.alignment.sigma.push(s);
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn warp_depth_image_to_colour(
        &self,
        depth: &(Matrix4<f32>, CameraMeasurement),
        colour: &(Matrix4<f32>, CameraMeasurement),
        di: usize,
        ci: usize,
    ) -> io::Result<Matrix<f32>> {
        let ds = self
            .cameras
            .get(&di)
            .ok_or_else(|| invalid("depth camera missing"))?;
        let cs = self
            .cameras
            .get(&ci)
            .ok_or_else(|| invalid("colour camera missing"))?;
        let im = depth_mat_to_image(
            depth
                .1
                .measurement
                .depth_image
                .as_ref()
                .ok_or_else(|| invalid("depth image missing"))?,
        )?;
        let rgb = colour
            .1
            .measurement
            .image
            .as_ref()
            .ok_or_else(|| invalid("colour image missing"))?;
        register_depth(
            &ds.intrinsics(),
            &cs.intrinsics(),
            &[],
            &(inverse(&colour.0) * depth.0),
            &im,
            (rgb.cols(), rgb.rows()),
            true,
        )
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn integrate_frame(&mut self, mut f: SupereightFrames, backlog: usize) -> io::Result<()> {
        if self.visualize_submaps {
            self.draw_submaps()?;
        }
        let create = self.decide_new_submap(&mut f)?;
        self.updated_maps.insert(f.keyframe_id);
        for (&id, data) in &f.frame_data {
            self.covisible_frames
                .insert(StateId::new(id), data.covisibles.clone());
            if self.poses.contains_key(&id) || id == f.keyframe_id {
                self.poses.insert(id, data.t_ws.clone());
                self.updated_maps.insert(id);
            }
        }
        if create {
            self.submap_counter += 1;
            if self.submap_counter > 1 {
                self.previous_submap = self.previous_keyframe;
            }
            self.add_submap_alignment_factors(&f)?;
            self.object_map.backend.create_map(f.keyframe_id)?;
            self.previous_keyframe = Some(f.keyframe_id);
            self.submap_trigger.lidar_keyframe_detected = false;
            self.num_integrated_lidar = 0;
            self.num_integrated_depth = 0;
        }
        let id = f.keyframe_id;
        let t_wk = self
            .poses
            .get(&id)
            .ok_or_else(|| invalid("active submap pose missing"))?
            .clone();
        if self.lidar_t_sl.is_some() && !f.rays.is_empty() && backlog <= 1 {
            self.update_lidar_align_block(&mut f.rays, &t_wk)?;
            if let Some(cb) = &self.integration_publish_callback {
                cb(&f.rays);
            }
            self.object_map
                .backend
                .integrate_rays(id, self.integration_counter, &f.rays)?;
            self.integration_counter += 1;
            self.num_integrated_lidar += 1;
        }
        self.integrate_depth_frames(&f, &t_wk, backlog)?;
        if let Some(cb) = &self.field_callback {
            let pose = self
                .latest_state
                .as_ref()
                .map_or_else(Transformation::identity, |state| state.t_ws.clone());
            let current = State::new(
                Transformation::from_matrix(&pose.matrix().cast::<f32>().cast::<f64>()),
                Vector3::zeros(),
                (Vector3::zeros(), Vector3::zeros()),
                Vector3::zeros(),
                Timestamp::UNIX_EPOCH,
                StateId::default(),
                false,
            );
            cb(&current, &self.poses, &self.object_map.backend);
        }
        if create && let Some(previous) = self.previous_submap {
            self.object_map.backend.cache_mesh(previous)?;
            self.object_map
                .finish_submap_objects(previous)
                .map_err(err)?;
            if let Some(cb) = &self.submap_callback {
                let poses = self
                    .updated_maps
                    .iter()
                    .filter(|&&i| Some(i) != self.previous_keyframe)
                    .filter_map(|i| self.poses.get(i).map(|p| (*i, p.clone())))
                    .collect::<BTreeMap<_, _>>();
                let ids = poses.keys().copied().collect::<Vec<_>>();
                cb(&poses, &ids, &self.object_map);
                self.updated_maps.clear();
            }
        }
        Ok(())
    }
    fn integrate_depth_frames(
        &mut self,
        f: &SupereightFrames,
        t_wk: &Transformation,
        backlog: usize,
    ) -> io::Result<()> {
        let id = f.keyframe_id;
        if !self.cameras.is_empty() && backlog <= 1 {
            for batch in &f.depth_frames {
                let (depth_id, colour_id) = self.depth_batch_cameras(batch);
                let Some(di) = depth_id else {
                    log::warn!("No depth mapping image in batch");
                    continue;
                };
                let depth = batch[&di]
                    .iter()
                    .find(|(_, m)| m.measurement.depth_image.is_some())
                    .ok_or_else(|| invalid("depth data missing"))?;
                let depth_image = depth_mat_to_image(
                    depth
                        .1
                        .measurement
                        .depth_image
                        .as_ref()
                        .ok_or_else(|| invalid("depth image missing"))?,
                )?;
                let sigma = if self.config.use_uncertainty && colour_id.is_none() {
                    depth
                        .1
                        .measurement
                        .sigma_image
                        .as_ref()
                        .map(|m| depth_mat_to_image(m))
                        .transpose()?
                } else {
                    None
                };
                let colour = colour_id.and_then(|ci| {
                    batch[&ci]
                        .iter()
                        .find(|(_, m)| m.measurement.image.is_some())
                        .map(|c| (ci, c))
                });
                let rgb = colour
                    .map(|(_, c)| {
                        rgb_mat_to_image(
                            c.1.measurement
                                .image
                                .as_ref()
                                .ok_or_else(|| invalid("RGB image missing"))?,
                        )
                    })
                    .transpose()?;
                let segments =
                    self.depth_batch_segments(colour, id, (depth, di), &depth_image, t_wk)?;
                let sensor = &self.cameras[&di];
                let t_md = t_wk.inverse().matrix().cast::<f32>() * depth.0;
                let colour_data = colour.zip(rgb.as_ref()).map(|((ci, c), rgb)| {
                    (
                        rgb,
                        &self.cameras[&ci],
                        t_wk.inverse().matrix().cast::<f32>() * c.0,
                    )
                });
                self.object_map.backend.integrate_depth(
                    id,
                    self.integration_counter,
                    DepthIntegration {
                        depth: &depth_image,
                        sigma: sigma.as_ref(),
                        sensor,
                        t_md,
                        colour: colour_data,
                        segments: segments.as_ref(),
                    },
                )?;
                self.integration_counter += 1;
                self.num_integrated_depth += 1;
                self.last_integrated_state = f.frame_data.keys().next_back().copied();
                if self.config.factors.use_map2map_factors {
                    self.update_depth_align_block(depth, t_wk, di)?;
                }
            }
        }
        Ok(())
    }
    fn depth_batch_segments(
        &mut self,
        colour: Option<(usize, &(Matrix4<f32>, CameraMeasurement))>,
        id: u64,
        (depth, di): (&(Matrix4<f32>, CameraMeasurement), usize),
        depth_image: &Matrix<f32>,
        t_wk: &Transformation,
    ) -> io::Result<Option<Matrix<u16>>> {
        let mut segments = None;
        if let Some((ci, c)) = colour {
            let w = self
                .warp_depth_image_to_colour(depth, c, di, ci)
                .unwrap_or_else(|e| {
                    log::warn!("Warping Depth failed: {e}; using original depth");
                    depth_image.clone()
                });
            if let (Some(features), Some(masks)) = (
                c.1.measurement.deep_learning_data.get("language_features"),
                c.1.measurement.deep_learning_data.get("sam_masks"),
            ) {
                if self.object_map.backend.supports_colour_and_ids() {
                    let sensor = &self.cameras[&di];
                    let t_md = t_wk.inverse().matrix().cast::<f32>() * c.0;
                    let surface = self.object_map.backend.lookup_ids(id, &w, sensor, &t_md)?;
                    let invalid_mask = Matrix::from_vec(
                        w.rows,
                        w.cols,
                        1,
                        w.data
                            .iter()
                            .map(|&v| u8::from(v < sensor.near || v > sensor.far))
                            .collect(),
                    );
                    segments = Some(
                        self.object_map
                            .process_data(id, &invalid_mask, masks, features, &surface, 0.25)
                            .map_err(err)?,
                    );
                } else {
                    log::error!("SAM and language features require a colour/ID map");
                }
            }
        }
        Ok(segments)
    }

    fn depth_batch_cameras(&self, batch: &PosedDepthFrames) -> (Option<usize>, Option<usize>) {
        let (mut depth_id, mut colour_id) = (None, None);
        for (&camera, measurements) in batch {
            if !self.cameras.contains_key(&camera) {
                continue;
            }
            for (_, m) in measurements {
                if self
                    .camera_types
                    .get(&camera)
                    .is_some_and(|t| t.depth_type.is_depth_camera)
                    && m.measurement.depth_image.is_some()
                {
                    depth_id = Some(camera);
                }
                if self.camera_types.get(&camera).is_some_and(|t| t.is_colour)
                    && m.measurement.image.is_some()
                {
                    colour_id = Some(camera);
                }
            }
        }
        (depth_id, colour_id)
    }

    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn save_submap(&mut self, id: u64) -> io::Result<()> {
        let directory = PathBuf::from(&self.config.results_directory);
        if !directory.exists() {
            fs::create_dir(&directory)?;
        }
        let numbering = if id < 1_000_000 {
            format!("{id:06}")
        } else {
            String::new()
        };
        let filename = directory.join(format!("mesh_kf{numbering}.ply"));
        let t = self
            .poses
            .get(&id)
            .ok_or_else(|| invalid("submap pose missing"))?
            .matrix()
            .cast::<f32>();
        if self.object_map.backend.mesh(id).is_err() {
            self.object_map.backend.cache_mesh(id)?;
        }
        let mesh = self.object_map.backend.mesh(id).map_err(err)?.to_vec();
        if self.object_map.backend.supports_colour_and_ids() {
            let mut filtered: Vec<_> = mesh
                .iter()
                .filter(|f| f.colour.is_some())
                .cloned()
                .collect();
            self.object_map.backend.colour_mesh_by_id(&mut filtered);
            self.object_map.backend.save_mesh(
                if filtered.is_empty() {
                    &mesh
                } else {
                    &filtered
                },
                &filename,
                &t,
            )?;
            if self.object_map.number_of_objects() > 0
                && self
                    .object_map
                    .submap_objects
                    .get(&id)
                    .is_some_and(|m| !m.is_empty())
                && let Some(mut d) = self.language_descriptor
            {
                let norm_squared = d.iter().map(|v| v * v).sum::<f32>();
                if norm_squared > 0.0 {
                    let norm = norm_squared.sqrt();
                    for v in &mut d {
                        *v /= norm;
                    }
                }
                let objects = self
                    .object_map
                    .submap_objects
                    .get(&id)
                    .ok_or_else(|| invalid("object submap missing"))?;
                for face in &mut filtered {
                    let value = if face.id == 0 {
                        0.0
                    } else {
                        let f = &objects
                            .get(&face.id)
                            .ok_or_else(|| invalid("object descriptor missing"))?
                            .vl_feature;
                        let norm_squared = f.iter().map(|v| v * v).sum::<f32>();
                        let norm = norm_squared.sqrt();
                        let similarity = f
                            .iter()
                            .zip(d)
                            .map(|(a, b)| {
                                if norm_squared > 0.0 {
                                    a / norm * b
                                } else {
                                    a * b
                                }
                            })
                            .sum::<f32>();
                        f64::from((similarity.clamp(0.1, 0.5) - 0.1) / 0.4)
                    };
                    let c = self
                        .object_map
                        .backend
                        .colour(value, crate::deep_learning::utils::ColourMap::Jet);
                    face.colour = Some(c.map(|v| (255.0 * v) as u8));
                }
                self.object_map.backend.save_mesh(
                    &filtered,
                    &directory.join(format!("activationMesh_kf{numbering}.ply")),
                    &t,
                )?;
            }
        } else {
            self.object_map.backend.save_mesh(&mesh, &filename, &t)?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn save_all_submap_meshes(&mut self) -> io::Result<()> {
        let ids: Vec<_> = self.poses.keys().copied().collect();
        for id in ids {
            self.save_submap(id)?;
            if !self
                .object_map
                .save_objects(
                    id,
                    Path::new(&self.config.results_directory),
                    &self.poses[&id].matrix().cast::<f32>(),
                )
                .map_err(err)?
            {
                log::warn!("Could not save objects for submap {id}");
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn generate_point_cloud(
        &self,
        t: &Matrix4<f32>,
        depth: &Matrix<f32>,
        sensor: &PinholeSensor,
        counter: usize,
    ) -> io::Result<()> {
        let points = (0..depth.rows)
            .flat_map(|y| {
                (0..depth.cols)
                    .map(move |x| sensor.back_project(x, y) * depth.data[y * depth.cols + x])
            })
            .collect::<Vec<_>>();
        self.object_map.backend.save_depth_png(
            depth,
            &PathBuf::from(&self.config.results_directory)
                .join(format!("../depth_images/dI{counter}.png")),
        )?;
        save_point_cloud_vtk(
            &points,
            t,
            &PathBuf::from(&self.config.results_directory).join(format!("../pc{counter}.vtk")),
        )
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn generate_point_cloud_from_lidar(
        &self,
        rays: &RayVector,
        counter: usize,
    ) -> io::Result<()> {
        let points = rays
            .iter()
            .map(|(t, p)| transform(t, p))
            .collect::<Vec<_>>();
        save_point_cloud_vtk(
            &points,
            &Matrix4::identity(),
            &PathBuf::from(&self.config.results_directory)
                .join(format!("../lidarPc/{counter}.vtk")),
        )
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn draw_submaps(&mut self) -> io::Result<()> {
        if self.poses.is_empty() {
            return Ok(());
        }
        let painter = self
            .top_view_painter
            .as_ref()
            .ok_or_else(|| invalid("top view drawing capability missing"))?;
        let mut bounds = BTreeMap::new();
        let (mut min, mut max) = (
            Vector3::repeat(f32::INFINITY),
            Vector3::repeat(f32::NEG_INFINITY),
        );
        for (&id, pose) in &self.poses {
            let b = self.object_map.backend.aabb(id)?;
            let b = transform_aabb(b.0, b.1, &pose.matrix().cast::<f32>());
            min = min.inf(&b.0);
            max = max.sup(&b.1);
            bounds.insert(id, b);
        }
        let size = 1000usize;
        let scale = (size as f64 / f64::from(max.x - min.x + 1.0))
            .min(size as f64 / f64::from(max.y - min.y + 1.0));
        let point = |p: Vector3<f32>| {
            Vector2::new(
                scale * f64::from(p.x - min.x + 0.5),
                size as f64 - scale * f64::from(p.y - min.y + 0.5),
            )
        };
        let mut image = Matrix::from_vec(size, size, 3, vec![0u8; size * size * 3]);
        let latest = self.associations.last_key_value();
        let mut positions = BTreeMap::new();
        for (&id, p) in &self.poses {
            let b = bounds[&id];
            let a = point(b.0);
            let c = point(b.1);
            let emph = latest.is_some_and(|(a, b)| id == *a || id == *b);
            let colour = if emph {
                [0.0, 0.0, 255.0]
            } else {
                [0.0, 255.0, 0.0]
            };
            let thickness = if emph { 3 } else { 1 };
            for (start, end) in [
                (a, Vector2::new(c.x, a.y)),
                (Vector2::new(c.x, a.y), c),
                (c, Vector2::new(a.x, c.y)),
                (Vector2::new(a.x, c.y), a),
            ] {
                painter.line(&mut image, start, end, colour, thickness)?;
            }
            let xy = point(p.translation().cast::<f32>());
            positions.insert(id, xy);
            painter.circle(&mut image, xy, 2.0, [0.0, 0.0, 255.0])?;
            painter.text(
                &mut image,
                &id.to_string(),
                xy + Vector2::new(3.0, 3.0),
                0.5,
                [255.0; 3],
                1,
            )?;
        }
        let path = positions.values().copied().collect::<Vec<_>>();
        for p in path.windows(2) {
            painter.line(&mut image, p[0], p[1], [255.0, 0.0, 0.0], 1)?;
        }
        for (a, b) in &self.associations {
            let a = positions
                .get(a)
                .ok_or_else(|| invalid("association submap missing"))?;
            let b = positions
                .get(b)
                .ok_or_else(|| invalid("association submap missing"))?;
            painter.line(&mut image, *a, *b, [0.0, 0.0, 255.0], 3)?;
        }
        painter.text(
            &mut image,
            &format!("Number of submaps: {}", self.poses.len()),
            Vector2::new(15.0, 15.0),
            0.5,
            [255.0; 3],
            1,
        )?;
        self.topview = Some(DynamicMatrix {
            data: DynamicData::U8(image),
        });
        Ok(())
    }
}

#[must_use]
pub fn transform_aabb(
    min: Vector3<f32>,
    max: Vector3<f32>,
    t: &Matrix4<f32>,
) -> (Vector3<f32>, Vector3<f32>) {
    let mut lo = Vector3::repeat(f32::INFINITY);
    let mut hi = Vector3::repeat(f32::NEG_INFINITY);
    for x in [min.x, max.x] {
        for y in [min.y, max.y] {
            for z in [min.z, max.z] {
                let p = transform(t, &Vector3::new(x, y, z));
                lo = lo.inf(&p);
                hi = hi.sup(&p);
            }
        }
    }
    (lo, hi)
}
#[must_use]
pub fn calculate_volume(min: Vector3<f32>, max: Vector3<f32>) -> f32 {
    (max.x - min.x).max(0.0) * (max.y - min.y).max(0.0) * (max.z - min.z).max(0.0)
}
#[must_use]
pub fn calculate_intersection_volume(
    a: Vector3<f32>,
    b: Vector3<f32>,
    c: Vector3<f32>,
    d: Vector3<f32>,
) -> f32 {
    calculate_volume(a.sup(&c), b.inf(&d))
}
#[must_use]
pub fn calculate_iou_3d(
    first_minimum: Vector3<f32>,
    first_maximum: Vector3<f32>,
    second_minimum: Vector3<f32>,
    second_maximum: Vector3<f32>,
) -> f32 {
    let intersection =
        calculate_intersection_volume(first_minimum, first_maximum, second_minimum, second_maximum);
    let union_volume = calculate_volume(first_minimum, first_maximum)
        + calculate_volume(second_minimum, second_maximum)
        - intersection;
    if union_volume < f32::EPSILON {
        0.0
    } else {
        intersection / union_volume
    }
}
/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn save_aligned_points(
    points: &[Vector3<f64>],
    t: &Transformation,
    path: &Path,
) -> io::Result<()> {
    let mut f = fs::File::create(path)?;
    write!(
        f,
        "ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nend_header\n",
        points.len()
    )?;
    for p in points {
        let p = t.transform_homogeneous(&p.push(1.0));
        writeln!(f, "{} {} {}", p.x as f32, p.y as f32, p.z as f32)?;
    }
    Ok(())
}
/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn save_point_cloud_vtk(
    points: &[Vector3<f32>],
    t: &Matrix4<f32>,
    path: &Path,
) -> io::Result<()> {
    let mut f = fs::File::create(path)?;
    write!(
        f,
        "# vtk DataFile Version 1.0\nPoint cloud\nASCII\nDATASET POLYDATA\nPOINTS {} float\n",
        points.len()
    )?;
    for p in points {
        let p = transform(t, p);
        writeln!(f, "{} {} {}", p.x, p.y, p.z)?;
    }
    Ok(())
}

/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn depth_img_res_downsampling<B: SupereightBackend>(
    backend: &B,
    original: &Matrix<f32>,
    factor: usize,
) -> io::Result<(bool, Matrix<f32>)> {
    if factor == 0 {
        return Err(invalid("zero depth downsampling factor"));
    }
    if original.cols.is_multiple_of(factor) && original.rows.is_multiple_of(factor) {
        Ok((true, backend.downsample_depth(original, factor)?))
    } else {
        log::warn!("Invalid downsampling factor, providing original image");
        Ok((false, original.clone()))
    }
}

impl SupereightBackend for crate::mapping::object_mapping::OccupancyMapBackend {
    type Map = crate::mapping::map_typedefs::OccupancyMap;
    fn create_map(&mut self, id: u64) -> io::Result<()> {
        if self.maps.contains_key(&id) {
            return Err(invalid("submap ID already exists"));
        }
        let map = firefly_supereight2::map::Map::from_config(self.map_config, self.data_config, 8)
            .map_err(err)?;
        let map = crate::mapping::map_typedefs::OccupancyMap::new(map);
        self.shared_maps
            .write()
            .map_err(|_| invalid("map registry lock poisoned"))?
            .insert(id, map.clone());
        self.maps.insert(id, map);
        Ok(())
    }
    fn map(&self, id: u64) -> io::Result<&Self::Map> {
        self.map_handle(id).map_err(err)
    }
    fn map_resolution(&self) -> f32 {
        self.map_config.res
    }
    fn log_odd_max(&self) -> f32 {
        self.data_config.field.log_odd_max
    }
    fn aabb(&self, id: u64) -> io::Result<(Vector3<f32>, Vector3<f32>)> {
        let aabb = self
            .map_handle(id)
            .map_err(err)?
            .read()
            .map_err(err)?
            .aabb();
        Ok((aabb.min, aabb.max))
    }
    fn integrate_rays(&mut self, _id: u64, _counter: u64, _rays: &RayVector) -> io::Result<()> {
        Err(invalid(
            "occupancy depth backend does not accept LiDAR rays",
        ))
    }
    fn integrate_depth(
        &mut self,
        id: u64,
        counter: u64,
        data: DepthIntegration<'_>,
    ) -> io::Result<()> {
        // Colour/ID-off maps update occupancy from depth independently of optional appearance data.
        let timestamp =
            i32::try_from(counter).map_err(|_| invalid("integration counter exceeds i32"))?;
        let sensor = se_pinhole_sensor(data.sensor)?;
        let measurements = firefly_supereight2::measurement::Measurements {
            depth: firefly_supereight2::measurement::Measurement {
                image: data.depth,
                sensor: &sensor,
                t_wc: se_rigid_transform(&data.t_md)?,
            },
            depth_sigma: data.sigma,
        };
        let mut map = self.map_handle(id).map_err(err)?.write().map_err(err)?;
        firefly_supereight2::map_integrator::MapIntegrator::new(&mut map)
            .integrate_depth(timestamp, &measurements, None)
            .map_err(err)
    }
    fn lookup_ids(
        &self,
        _id: u64,
        _depth: &Matrix<f32>,
        _sensor: &PinholeSensor,
        _t_md: &Matrix4<f32>,
    ) -> io::Result<Matrix<u16>> {
        Err(invalid("occupancy-only maps do not store instance IDs"))
    }
    fn cache_mesh(&mut self, id: u64) -> io::Result<()> {
        let triangles = self
            .map_handle(id)
            .map_err(err)?
            .read()
            .map_err(err)?
            .mesh(&Matrix4::identity(), 0)
            .map_err(err)?;
        let mesh = triangles
            .into_iter()
            .map(|face| MeshFace {
                id: 0,
                scale: face.scale,
                vertices: face.vertices,
                colour: None,
            })
            .collect();
        self.meshes.insert(id, mesh);
        Ok(())
    }
    fn downsample_depth(&self, original: &Matrix<f32>, factor: usize) -> io::Result<Matrix<f32>> {
        if factor == 0
            || !original.rows.is_multiple_of(factor)
            || !original.cols.is_multiple_of(factor)
        {
            return Err(invalid(
                "depth dimensions must be divisible by a positive factor",
            ));
        }
        let rows = original.rows / factor;
        let cols = original.cols / factor;
        let mut output = Matrix::from_vec(rows, cols, 1, vec![0.0; rows * cols]);
        firefly_supereight2::preprocessor::downsample_depth(original, &mut output).map_err(err)?;
        Ok(output)
    }
    fn save_depth_png(&self, depth: &Matrix<f32>, path: &Path) -> io::Result<()> {
        firefly_supereight2::image::validate_image(depth).map_err(err)?;
        let mut bytes = Vec::with_capacity(depth.data.len() * 2);
        for &value in &depth.data {
            let millimetres = (1000.0f32 * value).round();
            if !millimetres.is_finite() || !(0.0..=f32::from(u16::MAX)).contains(&millimetres) {
                return Err(invalid(
                    "depth PNG sample cannot be represented as u16 millimetres",
                ));
            }
            bytes.extend_from_slice(&(millimetres as u16).to_be_bytes());
        }
        let width = u32::try_from(depth.cols).map_err(|_| invalid("PNG width exceeds u32"))?;
        let height = u32::try_from(depth.rows).map_err(|_| invalid("PNG height exceeds u32"))?;
        let mut encoder = png::Encoder::new(
            std::io::BufWriter::new(fs::File::create(path)?),
            width,
            height,
        );
        encoder.set_color(png::ColorType::Grayscale);
        encoder.set_depth(png::BitDepth::Sixteen);
        encoder
            .write_header()
            .map_err(err)?
            .write_image_data(&bytes)
            .map_err(err)
    }
    fn save_mesh(&self, mesh: &[MeshFace], path: &Path, t_wm: &Matrix4<f32>) -> io::Result<()> {
        save_occupancy_mesh(mesh, path, t_wm)
    }
    fn colour_mesh_by_id(&self, _mesh: &mut [MeshFace]) {}
}
fn se_rigid_transform(matrix: &Matrix4<f32>) -> io::Result<nalgebra::IsometryMatrix3<f32>> {
    if matrix.iter().any(|value| !value.is_finite()) {
        return Err(invalid("non-finite depth pose"));
    }
    let rotation =
        nalgebra::Rotation3::from_matrix_unchecked(matrix.fixed_view::<3, 3>(0, 0).into_owned());
    Ok(nalgebra::IsometryMatrix3::from_parts(
        nalgebra::Translation3::from(matrix.fixed_view::<3, 1>(0, 3).into_owned()),
        rotation,
    ))
}
fn se_pinhole_sensor(
    sensor: &PinholeSensor,
) -> io::Result<firefly_supereight2::pinhole_camera::PinholeCamera> {
    let config = firefly_supereight2::pinhole_camera::PinholeConfig {
        sensor: firefly_supereight2::sensor::SensorConfig {
            width: i32::try_from(sensor.width).map_err(|_| invalid("sensor width exceeds i32"))?,
            height: i32::try_from(sensor.height)
                .map_err(|_| invalid("sensor height exceeds i32"))?,
            near_plane: sensor.near,
            far_plane: sensor.far,
            t_bs: se_rigid_transform(&sensor.t_sc.matrix().cast())?,
            ..Default::default()
        },
        fx: sensor.fx,
        fy: sensor.fy,
        cx: sensor.cx,
        cy: sensor.cy,
    };
    firefly_supereight2::pinhole_camera::PinholeCamera::new(&config).map_err(err)
}
fn save_occupancy_mesh(mesh: &[MeshFace], path: &Path, t_wm: &Matrix4<f32>) -> io::Result<()> {
    const SCALE_COLOURS: [[u8; 3]; 8] = [
        [102, 194, 165],
        [252, 141, 98],
        [141, 160, 203],
        [231, 138, 195],
        [166, 216, 84],
        [255, 217, 47],
        [229, 196, 148],
        [179, 179, 179],
    ];
    let extension = path
        .extension()
        .and_then(|s| s.to_str())
        .ok_or_else(|| invalid("mesh file extension absent"))?;
    if !matches!(extension, "ply" | "vtk" | "obj") {
        return Err(invalid("unknown mesh file extension"));
    }
    if mesh.iter().any(|face| face.scale < 0) {
        return Err(invalid("negative mesh scale has no source colour"));
    }
    let colour = |face: &MeshFace| SCALE_COLOURS[(face.scale as usize).min(7)];
    let mut file = std::io::BufWriter::new(fs::File::create(path)?);
    let vertices = mesh
        .len()
        .checked_mul(3)
        .ok_or_else(|| invalid("mesh vertex count overflow"))?;
    match extension {
        "ply" => write!(
            file,
            "ply\nformat ascii 1.0\ncomment Mesh generated by supereight 2\nelement vertex {vertices}\nproperty float x\nproperty float y\nproperty float z\nelement face {}\nproperty list uchar int vertex_index\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n",
            mesh.len()
        )?,
        "vtk" => write!(
            file,
            "# vtk DataFile Version 3.0\nMesh generated by supereight 2\nASCII\nDATASET POLYDATA\n\nPOINTS {vertices} float\n"
        )?,
        "obj" => write!(
            file,
            "# Mesh generated by supereight 2\n# {vertices} vertices\n# {} faces\n",
            mesh.len()
        )?,
        _ => unreachable!(),
    }
    for face in mesh {
        for vertex in &face.vertices {
            let p = transform(t_wm, vertex);
            if extension == "obj" {
                write!(file, "v ")?;
            }
            writeln!(file, "{} {} {}", p.x, p.y, p.z)?;
        }
    }
    if extension == "vtk" {
        write!(file, "\nPOLYGONS {} {}\n", mesh.len(), mesh.len() * 4)?;
    }
    for (i, face) in mesh.iter().enumerate() {
        match extension {
            "ply" => {
                let c = colour(face);
                writeln!(
                    file,
                    "3 {} {} {} {} {} {}",
                    i * 3,
                    i * 3 + 1,
                    i * 3 + 2,
                    c[0],
                    c[1],
                    c[2]
                )?;
            }
            "vtk" => writeln!(file, "3 {} {} {}", i * 3, i * 3 + 1, i * 3 + 2)?,
            "obj" => writeln!(file, "f {} {} {}", i * 3 + 1, i * 3 + 2, i * 3 + 3)?,
            _ => unreachable!(),
        }
    }
    if extension == "vtk" {
        write!(
            file,
            "\nCELL_DATA {}\n\nSCALARS scale unsigned_char\nLOOKUP_TABLE default\n",
            mesh.len()
        )?;
        for face in mesh {
            writeln!(file, "{}", face.scale)?;
        }
        writeln!(file, "COLOR_SCALARS scale_colour 3")?;
        for face in mesh {
            let c = colour(face);
            writeln!(
                file,
                "{} {} {}",
                f32::from(c[0]) / 255.0,
                f32::from(c[1]) / 255.0,
                f32::from(c[2]) / 255.0
            )?;
        }
    }
    file.flush()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn depth_pose_adapter_preserves_the_input_matrix() {
        let mut matrix = Matrix4::identity();
        matrix[(0, 0)] = 0.992_808_64;
        matrix[(2, 2)] = matrix[(0, 0)];
        matrix[(0, 2)] = 0.119_712_204;
        matrix[(2, 0)] = -matrix[(0, 2)];
        matrix[(0, 3)] = 0.03;
        assert_eq!(
            se_rigid_transform(&matrix).unwrap().to_homogeneous(),
            matrix
        );
    }

    #[test]
    fn integer_depth_scaling_uses_opencv_rounding() {
        let input = DynamicMatrix {
            data: DynamicData::U16(Matrix::from_vec(1, 4, 1, vec![1, 3, 5, u16::MAX])),
        };
        let output = scaled_nearest(&input, 1, 0.5).unwrap();
        let DynamicData::U16(output) = output.data else {
            panic!("depth type changed")
        };
        assert_eq!(output.data, vec![0, 2, 2, 32768]);
    }

    #[test]
    fn occupancy_only_backend_accepts_optional_colour() {
        let mut backend = crate::mapping::object_mapping::OccupancyMapBackend::new(
            firefly_supereight2::map::MapConfig::default(),
            firefly_supereight2::data::DataConfig::default(),
        );
        backend.create_map(1).unwrap();
        let depth = Matrix::from_vec(4, 4, 1, vec![2.0; 16]);
        let colour = Matrix::from_vec(4, 4, 3, vec![128; 48]);
        let sensor = PinholeSensor {
            fx: 100.0,
            fy: 100.0,
            cx: 1.5,
            cy: 1.5,
            width: 4,
            height: 4,
            near: 0.1,
            far: 3.0,
            t_sc: Transformation::identity(),
        };
        backend
            .integrate_depth(
                1,
                0,
                DepthIntegration {
                    depth: &depth,
                    sigma: None,
                    sensor: &sensor,
                    t_md: Matrix4::identity(),
                    colour: Some((&colour, &sensor, Matrix4::identity())),
                    segments: None,
                },
            )
            .unwrap();
    }
}
