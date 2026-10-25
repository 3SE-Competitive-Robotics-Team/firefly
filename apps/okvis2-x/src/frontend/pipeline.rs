/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! BRISK matching, landmark triangulation, initialization and place recognition.
use super::{
    f_brisk::{Descriptor, FBrisk, hamming},
    frame_absolute_pose_sac_problem::{
        AbsolutePoseSolver, Algorithm as AbsoluteAlgorithm, FrameAbsolutePoseSacProblem,
    },
    frame_noncentral_absolute_adapter::{AbsoluteAdapter, FrameNoncentralAbsoluteAdapter},
    frame_relative_adapter::{EstimatorView, FrameRelativeAdapter},
    frame_relative_pose_sac_problem::{
        Algorithm as RelativeAlgorithm, CentralRelativePoseSolver, FrameRelativePoseSacProblem,
    },
    frame_rotation_only_sac_problem::{FrameRotationOnlySacProblem, RotationOnlySolver},
    loopclosure_noncentral_absolute_adapter::LoopclosureNoncentralAbsoluteAdapter,
    stereo_triangulation::triangulate_fast,
};
use crate::{
    cameras::camera_base::{CameraBase, ProjectionStatus},
    ceres::{component::MultiFramePtr, imu_error::ImuError},
    config::parameters::{ImuParameters, ViParameters},
    frames::{
        frame::{DescriptorExtractor, FeatureDetector, Frame, KeyPoint, Network},
        frame_typedefs::{
            KeypointIdentifier, LandmarkId, MapPoint2, MapPoints, SpeedAndBias, StateId,
        },
        multi_frame::MultiFrame,
    },
    kinematics::transformation::{Transformation, quaternion_from_rotation_matrix},
    sensors::measurements::ImuMeasurementDeque,
};
use firefly_dbow2::{
    templated_database::TemplatedDatabase, templated_vocabulary::TemplatedVocabulary,
};
use firefly_error::{Error, ErrorKind, Result};
use jiff::Timestamp;
use nalgebra::{DVector, Matrix2, SMatrix, Vector2, Vector3, Vector4};
use std::{
    collections::{BTreeMap, BTreeSet},
    marker::PhantomData,
    path::Path,
    sync::{Arc, Mutex},
};

/// Creates a classification network inside its owning worker thread.
pub type NetworkInitializer = Box<dyn FnOnce() -> Result<Arc<dyn Network>> + Send>;
/// Graph primitives used by the matching algorithms; no matching stage is delegated.
pub trait FrontendGraph {
    fn frame(&self, id: StateId) -> MultiFramePtr;
    fn frame_ids(&self) -> Vec<StateId>;
    fn pose(&self, id: StateId) -> Transformation;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn extrinsics(&self, id: StateId, camera: usize) -> Result<Transformation>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn set_pose(&mut self, id: StateId, pose: Transformation) -> Result<()>;
    fn landmarks(&self) -> MapPoints;
    /// # Errors
    /// Returns an error if the landmark is absent.
    fn landmark(&self, id: LandmarkId) -> Result<Vector4<f64>>;
    /// # Errors
    /// Returns an error if the landmark or its graph metadata is absent.
    fn get_landmark(&self, id: LandmarkId) -> Result<MapPoint2>;
    fn is_landmark_added(&self, id: LandmarkId) -> bool;
    /// # Errors
    /// Returns an error if the landmark is absent.
    fn is_landmark_initialised(&self, id: LandmarkId) -> Result<bool>;
    fn is_observed(&self, key: KeypointIdentifier) -> bool;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn add_landmark(&mut self, point: Vector4<f64>, initialized: bool) -> Result<LandmarkId>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn set_landmark(
        &mut self,
        id: LandmarkId,
        point: Vector4<f64>,
        initialized: bool,
    ) -> Result<()>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn add_observation(&mut self, id: LandmarkId, kid: KeypointIdentifier) -> Result<()>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn remove_observation(&mut self, kid: KeypointIdentifier) -> Result<()>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn merge_landmark(&mut self, from: LandmarkId, into: LandmarkId) -> Result<()>;
    /// Resolve repeated and chained IDs across one loop-matching batch, as in upstream.
    /// # Errors
    /// Returns an error for mismatched vectors, absent landmarks or a busy full graph.
    fn merge_landmarks(&mut self, from: &[LandmarkId], into: &[LandmarkId]) -> Result<usize>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn set_observation_information(
        &mut self,
        kid: KeypointIdentifier,
        information: Matrix2<f64>,
    ) -> Result<()>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn set_landmark_classification(&mut self, id: LandmarkId, class: i32) -> Result<()>;
    fn keyframes(&self) -> BTreeSet<StateId>;
    fn is_keyframe(&self, id: StateId) -> bool;
    fn imu_frames(&self) -> BTreeSet<StateId>;
    fn loop_closure_frames(&self) -> BTreeSet<StateId>;
    fn tracking_quality(&self, id: StateId) -> f64;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn optimise_realtime(&mut self, iterations: i32, threads: i32, initialized: bool)
    -> Result<()>;
    fn is_loop_closing(&self) -> bool;
    fn is_loop_closure_available(&self) -> bool;
    fn needs_full_graph_optimisation(&self) -> bool;
    fn is_pose_graph_frame(&self, id: StateId) -> bool;
    fn is_loop_closure_frame(&self, id: StateId) -> bool;
    fn is_recent_loop_closure_frame(&self, id: StateId) -> bool;
    fn is_place_recognition_frame(&self, id: StateId) -> bool;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn attempt_loop_closure(
        &mut self,
        old: StateId,
        current: StateId,
        relative: Transformation,
        hessian: SMatrix<f64, 6, 6>,
        drift_percentage: f64,
    ) -> Result<Option<bool>>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn add_loop_closure_frame(
        &mut self,
        id: StateId,
        skip_full_graph: bool,
    ) -> Result<BTreeSet<LandmarkId>>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn set_component_alignment(
        &mut self,
        id: StateId,
        component: usize,
        pose: Transformation,
    ) -> Result<()>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn clean_unobserved_landmarks(&mut self) -> Result<()>;
}
#[derive(Clone, Debug)]
pub struct BriskSettings {
    pub octaves: usize,
    pub threshold: f64,
    pub absolute_threshold: f64,
    pub maximum_keypoints: usize,
    pub rotation_invariance: bool,
    pub scale_invariance: bool,
}
impl Default for BriskSettings {
    fn default() -> Self {
        Self {
            octaves: 0,
            threshold: 40.0,
            absolute_threshold: 200.0,
            maximum_keypoints: 450,
            rotation_invariance: true,
            scale_invariance: false,
        }
    }
}
/// Camera-aware BRISK constructor and gravity extraction direction. Implementations
/// configure the supplied camera's ray/Jacobian maps and focal length before extraction.
pub trait BriskFeatureFactory {
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn create(
        &self,
        camera: &dyn CameraBase,
        settings: &BriskSettings,
        direction: Vector3<f32>,
    ) -> Result<(Arc<dyn FeatureDetector>, Arc<dyn DescriptorExtractor>)>;
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn classification_network_factory(&self, _camera: usize) -> Result<NetworkInitializer> {
        Err(frontend_error(
            "CNN network constructor capability is required",
        ))
    }
}
/// A fixed landmark/extrinsic reprojection residual for loop-pose Ceres refinement.
pub struct LoopResidual {
    pub camera: usize,
    pub keypoint: usize,
    pub point: Vector4<f64>,
    pub measurement: Vector2<f64>,
    pub information: Matrix2<f64>,
    pub extrinsics: Transformation,
}
pub struct LoopRefinement {
    pub pose: Transformation,
    pub residuals: Vec<(Vector2<f64>, SMatrix<f64, 2, 6>)>,
}
/// Ceres pose-only solve, Cauchy(3), constant homogeneous landmarks/extrinsics.
pub trait LoopPoseSolver {
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn refine_loop_pose(
        &self,
        frame: &MultiFrame,
        initial: &Transformation,
        residuals: &[LoopResidual],
        iterations: i32,
        threads: i32,
    ) -> Result<LoopRefinement>;
}
struct BowDatabase {
    database: TemplatedDatabase<FBrisk>,
    pose_ids: Vec<StateId>,
}
impl BowDatabase {
    fn new(vocabulary: &TemplatedVocabulary<FBrisk>) -> Self {
        Self {
            database: TemplatedDatabase::new(vocabulary, false, 0),
            pose_ids: Vec::new(),
        }
    }
}
pub struct RecognitionComponent {
    pub frames: BTreeMap<StateId, MultiFramePtr>,
    pub poses: BTreeMap<StateId, Transformation>,
    pub fixed: bool,
}
/// A component loader fills actual graph poses and frame/landmark records.
pub trait ComponentLoader {
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    fn load(
        &self,
        path: &Path,
        imu: &ImuParameters,
        cameras: &crate::cameras::n_camera_system::NCameraSystem,
        fixed: bool,
    ) -> Result<RecognitionComponent>;
}
struct ComponentDatabase {
    component: RecognitionComponent,
    bow: BowDatabase,
}
type MotionStereoMatch = (usize, usize, Vector4<f64>, bool, f64);
type MapCandidates = BTreeMap<LandmarkId, LandmarkToMatch>;
type MapAssignment = (usize, LandmarkId, Option<Vector4<f64>>);
struct InitializedMatches {
    prepared: Vec<MapCandidates>,
    count: usize,
    reprojection_error: f64,
}

/// 单相机复用的一组特征检测器/描述子。
type FeaturePair = (Arc<dyn FeatureDetector>, Arc<dyn DescriptorExtractor>);

pub struct Frontend<G, S, B> {
    pub is_initialized: bool,
    pub tracking_lost: bool,
    pub num_cameras: usize,
    pub brisk: BriskSettings,
    pub brisk_matching_threshold: f64,
    pub keyframe_insertion_overlap_threshold: f32,
    solver: S,
    features: B,
    detection_mutexes: Vec<Mutex<()>>,
    /// 每相机只构建一次的 BRISK 检测器/描述子：旋转模式表与相机属性与帧无关，
    /// 每帧重建要重算约 390 万个旋转项。
    features_cache: Vec<Mutex<Option<FeaturePair>>>,
    bow: BowDatabase,
    components: Vec<ComponentDatabase>,
    graph: PhantomData<G>,
    cnn_threads: Vec<CnnJob>,
}
impl<
    G: FrontendGraph,
    S: AbsolutePoseSolver + CentralRelativePoseSolver + RotationOnlySolver + LoopPoseSolver,
    B: BriskFeatureFactory,
> Frontend<G, S, B>
{
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn new(
        num_cameras: usize,
        directory: impl AsRef<Path>,
        solver: S,
        features: B,
    ) -> Result<Self> {
        let vocabulary = TemplatedVocabulary::load(directory.as_ref().join("small_voc.yml.gz"))
            .map_err(|e| frontend_error(e.to_string()))?;
        Ok(Self::with_vocabulary(
            num_cameras,
            &vocabulary,
            solver,
            features,
        ))
    }
    pub fn with_vocabulary(
        num_cameras: usize,
        vocabulary: &TemplatedVocabulary<FBrisk>,
        solver: S,
        features: B,
    ) -> Self {
        Self {
            is_initialized: false,
            tracking_lost: false,
            num_cameras,
            brisk: BriskSettings::default(),
            brisk_matching_threshold: 60.0,
            keyframe_insertion_overlap_threshold: 0.55,
            solver,
            features,
            detection_mutexes: (0..num_cameras).map(|_| Mutex::new(())).collect(),
            features_cache: (0..num_cameras).map(|_| Mutex::new(None)).collect(),
            bow: BowDatabase::new(vocabulary),
            components: Vec::new(),
            graph: PhantomData,
            cnn_threads: Vec::new(),
        }
    }
    /// # Errors
    /// Returns an error if the component, graph or vocabulary files cannot be read or decoded.
    ///
    /// # Panics
    /// Panics if a required resource/invariant is absent: `frontend owns a vocabulary`.
    pub fn load_component(
        &mut self,
        path: impl AsRef<Path>,
        imu: &ImuParameters,
        cameras: &crate::cameras::n_camera_system::NCameraSystem,
        fixed: bool,
        loader: &impl ComponentLoader,
    ) -> Result<bool> {
        let component = loader.load(path.as_ref(), imu, cameras, fixed)?;
        let mut bow = BowDatabase::new(
            &self
                .bow
                .database
                .get_vocabulary()
                .expect("frontend owns a vocabulary")
                .clone(),
        );
        for (&id, frame) in &component.frames {
            let descriptors = descriptors(&frame.borrow())?;
            bow.database.add(&descriptors);
            bow.pose_ids.push(id);
        }
        self.components.push(ComponentDatabase { component, bow });
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn detect_and_describe(
        &self,
        camera: usize,
        frame: &MultiFramePtr,
        t_wc: &Transformation,
        keypoints: Option<&[KeyPoint]>,
    ) -> Result<bool> {
        if keypoints.is_some() {
            return Err(frontend_error("external keypoints currently not supported"));
        }
        let _guard = self
            .detection_mutexes
            .get(camera)
            .ok_or_else(|| frontend_error("camera index out of range"))?
            .lock()
            .map_err(|_| frontend_error("detector mutex poisoned"))?;
        let direction = t_wc
            .inverse()
            .rotate_vector(&Vector3::new(0.0, 0.0, -1.0))
            .cast::<f32>();
        let mut frame = frame.borrow_mut();
        let (detector, extractor) = {
            let mut cached = self
                .features_cache
                .get(camera)
                .ok_or_else(|| frontend_error("camera index out of range"))?
                .lock()
                .map_err(|_| frontend_error("feature cache mutex poisoned"))?;
            if let Some(pair) = cached.as_ref() {
                pair.clone()
            } else {
                let pair = self.features.create(
                    frame.geometry(camera).as_ref(),
                    &self.brisk,
                    direction,
                )?;
                *cached = Some(pair.clone());
                pair
            }
        };
        extractor.set_extraction_direction(direction);
        frame.set_detector(camera, detector);
        frame.set_extractor(camera, extractor);
        frame.detect(camera)?;
        frame.describe(camera)?;
        frame.compute_back_projections(camera);
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn propagation(
        &self,
        measurements: &ImuMeasurementDeque,
        parameters: &ImuParameters,
        pose: &mut Transformation,
        sb: &mut SpeedAndBias,
        (start, end): (Timestamp, Timestamp),
        (covariance, jacobian): (
            Option<&mut SMatrix<f64, 15, 15>>,
            Option<&mut SMatrix<f64, 15, 15>>,
        ),
    ) -> Result<bool> {
        if measurements.len() < 2 {
            return Ok(false);
        }
        ImuError::propagation(
            measurements,
            parameters,
            pose,
            sb,
            (start, end),
            (covariance, jacobian),
        )
        .map(|n| n > 0)
        .map_err(|e| frontend_error(format!("{e:?}")))
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn clear(&mut self) -> Result<()> {
        self.end_cnn_threads()?;
        self.is_initialized = false;
        self.bow.database.clear();
        self.bow.pose_ids.clear();
        self.tracking_lost = false;
        Ok(())
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn data_association_and_initialization(
        &mut self,
        graph: &mut G,
        parameters: &ViParameters,
        frame: &MultiFramePtr,
        kf_prior: bool,
        as_keyframe: &mut bool,
    ) -> Result<bool> {
        self.finish_ready_cnn_threads()?;
        let id = StateId::new(frame.borrow().id());
        let distortion = parameters.n_camera_system.distortion_type(0);
        for c in 1..self.num_cameras {
            if parameters.n_camera_system.distortion_type(c) != distortion {
                return Err(frontend_error("mixed frame types are not supported"));
            }
        }
        let mut tracking_quality = 1.0;
        if graph.frame_ids().len() > 1 {
            let match_mask = self.match_to_map(graph, parameters, id, None)?;
            tracking_quality = graph.tracking_quality(id);
            if !self.tracking_lost || !self.is_initialized {
                let (_, rotation_only) = self.match_motion_stereo(graph, parameters, id)?;
                if !rotation_only || match_mask > 5 {
                    self.is_initialized = true;
                }
                tracking_quality = graph.tracking_quality(id);
            }
            *as_keyframe = kf_prior || self.do_we_need_a_new_keyframe(graph, &frame.borrow());
        } else {
            *as_keyframe = true;
        }
        let features = descriptors(&frame.borrow())?;
        let can_recognize = !graph.is_loop_closing()
            && !graph.is_loop_closure_available()
            && !graph.needs_full_graph_optimisation()
            && self.is_initialized;
        self.recognise_components(graph, parameters, (id, frame), &features, can_recognize)?;
        if parameters.estimator.do_loop_closures && can_recognize {
            let candidates = filtered_bow(&self.bow, &features);
            let mut attempts = 0;
            for (old, p) in candidates {
                if attempts > 10.max(self.bow.pose_ids.len() / 20) {
                    break;
                }
                if p <= parameters.estimator.p_dbow {
                    continue;
                }
                if !graph.is_pose_graph_frame(old)
                    || graph.is_loop_closure_frame(old)
                    || graph.is_recent_loop_closure_frame(old)
                    || !graph.is_place_recognition_frame(old)
                {
                    continue;
                }
                attempts += 1;
                let old_frame = graph.frame(old);
                let Some((relative, hessian)) = self.verify_recognised_place(
                    graph,
                    parameters,
                    &frame.borrow(),
                    &old_frame.borrow(),
                    10,
                )?
                else {
                    continue;
                };
                let Some(skip) = graph.attempt_loop_closure(
                    old,
                    id,
                    relative,
                    hessian,
                    parameters.estimator.drift_percentage_heuristic,
                )?
                else {
                    continue;
                };
                let landmarks = graph.add_loop_closure_frame(old, skip)?;
                self.match_to_map(graph, parameters, id, Some(&landmarks))?;
                *as_keyframe = kf_prior || self.do_we_need_a_new_keyframe(graph, &frame.borrow());
                break;
            }
            if *as_keyframe && !kf_prior {
                self.bow.database.add(&features);
                self.bow.pose_ids.push(id);
            }
        }
        self.classify_keyframe(parameters, frame, *as_keyframe)?;
        if *as_keyframe {
            self.match_stereo(graph, frame, *as_keyframe)?;
        }
        // 无畸变相机同样走这两遍剪枝：remove_outliers 经 valid_reprojection_inclusive
        // 走相机模型投影，本身不区分畸变类型，这里不再拦截。
        // The source calls this same pruning stage twice.
        self.remove_outliers(graph, frame)?;
        self.remove_outliers(graph, frame)?;
        if parameters.frontend.use_cnn {
            Self::prune_dynamic_observations(graph)?;
        }
        graph.clean_unobserved_landmarks()?;
        Ok(tracking_quality >= 0.01)
    }
    fn classify_keyframe(
        &mut self,
        parameters: &ViParameters,
        frame: &MultiFramePtr,
        as_keyframe: bool,
    ) -> Result<()> {
        if parameters.frontend.use_cnn && self.is_initialized && as_keyframe {
            self.end_cnn_threads()?;
            for c in 0..self.num_cameras {
                let (image, keypoints) = {
                    let mf = frame.borrow();
                    let image = mf
                        .image(c)
                        .ok_or_else(|| frontend_error("missing classification image"))?
                        .clone();
                    let mut keypoints = Vec::with_capacity(mf.num_keypoints(c));
                    for k in 0..mf.num_keypoints(c) {
                        let mut keypoint = KeyPoint::default();
                        mf.get_cv_keypoint(c, k, &mut keypoint);
                        keypoints.push(keypoint);
                    }
                    (image, keypoints)
                };
                let construct = self.features.classification_network_factory(c)?;
                let handle = std::thread::spawn(move || {
                    let (cols, rows) = (image.cols(), image.rows());
                    let mut local = Frame::default();
                    local.set_image(image);
                    local.reset_keypoints(&keypoints);
                    local.set_network(construct()?);
                    local.compute_classifications(64 * (cols / 64), 64 * (rows / 64), 1)?;
                    let mut values = Vec::with_capacity(keypoints.len());
                    for k in 0..keypoints.len() {
                        let mut value = [0.0; 19];
                        local.get_classification(k, &mut value);
                        values.push(value);
                    }
                    Ok(values)
                });
                self.cnn_threads.push(CnnJob {
                    frame: frame.clone(),
                    camera: c,
                    handle,
                });
            }
        }
        Ok(())
    }

    fn recognise_components(
        &self,
        graph: &mut G,
        parameters: &ViParameters,
        (id, frame): (StateId, &MultiFramePtr),
        features: &[Descriptor],
        can_recognize: bool,
    ) -> Result<()> {
        if can_recognize {
            for (c, component) in self.components.iter().enumerate() {
                let candidates = filtered_bow(&component.bow, features);
                let mut attempts = 0;
                for (old, p) in candidates {
                    if attempts > 10.max(component.bow.pose_ids.len() / 20) {
                        break;
                    }
                    if p > 0.4 {
                        let old_frame = component.component.frames[&old].borrow();
                        attempts += 1;
                        if let Some((relative, _)) = self.verify_recognised_place(
                            graph,
                            parameters,
                            &frame.borrow(),
                            &old_frame,
                            40,
                        )? {
                            graph.set_component_alignment(
                                id,
                                c,
                                component.component.poses[&old].compose(&relative),
                            )?;
                            break;
                        }
                    }
                }
            }
        }
        Ok(())
    }

    pub fn do_we_need_a_new_keyframe(&self, graph: &G, current: &MultiFrame) -> bool {
        if graph.frame_ids().len() < 4 {
            return true;
        }
        if !self.is_initialized {
            return false;
        }
        let ids = (0..current.num_frames())
            .flat_map(|c| (0..current.num_keypoints(c)).map(move |k| current.landmark_id(c, k)))
            .filter(|&id| id != 0)
            .collect::<BTreeSet<_>>();
        let overlap = coverage_overlap(current, |id| id != 0);
        let mut all = graph.keyframes();
        all.extend(graph.loop_closure_frames());
        let imu = graph.imu_frames();
        for id in graph.frame_ids().into_iter().rev() {
            if !imu.contains(&id) {
                break;
            }
            if graph.is_keyframe(id) {
                all.insert(id);
            }
        }
        let mut others = 0.0f64;
        for id in all {
            let frame = graph.frame(id);
            others = others.max(coverage_overlap(&frame.borrow(), |id| {
                id != 0 && ids.contains(&id)
            }));
        }
        let overlap = overlap.min(others);
        current.total_keypoints() >= 7 * current.num_frames()
            && (overlap as f32).partial_cmp(&self.keyframe_insertion_overlap_threshold)
                != Some(std::cmp::Ordering::Greater)
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn remove_outliers(&self, graph: &mut G, frame: &MultiFramePtr) -> Result<usize> {
        let mut count = 0;
        let mut removals = Vec::new();
        {
            let mf = frame.borrow();
            let pose = graph.pose(StateId::new(mf.id()));
            for c in 0..mf.num_frames() {
                let t_cw = pose.compose(mf.t_sc(c)).inverse();
                for k in 0..mf.num_keypoints(c) {
                    let id = LandmarkId::new(mf.landmark_id(c, k));
                    if graph.is_landmark_added(id) {
                        let point = graph.landmark(id)?;
                        if valid_reprojection_inclusive(&mf, c, k, &t_cw, &point, 4.0) {
                            count += 1;
                        } else {
                            removals.push(KeypointIdentifier::new(mf.id(), c, k));
                        }
                    }
                }
            }
        }
        for kid in removals {
            graph.remove_observation(kid)?;
        }
        Ok(count)
    }
    fn prune_dynamic_observations(graph: &mut G) -> Result<()> {
        for (id, point) in graph.landmarks() {
            let mut class = if point.classification == 10 || point.classification == 11 {
                Some(point.classification)
            } else {
                None
            };
            if class.is_none() {
                for obs in &point.observations {
                    if !graph.keyframes().contains(&StateId::new(obs.frame_id)) {
                        continue;
                    }
                    let frame = graph.frame(StateId::new(obs.frame_id));
                    let frame = frame.borrow();
                    let mut logits = [0.0; 19];
                    if frame.is_classified(obs.camera_index)
                        && frame.get_classification(
                            obs.camera_index,
                            obs.keypoint_index,
                            &mut logits,
                        )
                    {
                        if logits[10] > 3.5 {
                            class = Some(10);
                            break;
                        }
                        if logits[11] > 53.5 {
                            class = Some(11);
                            break;
                        }
                    }
                }
            }
            if let Some(class) = class {
                graph.set_landmark_classification(id, class)?;
                for obs in point.observations {
                    graph.set_observation_information(obs, Matrix2::identity() * 0.0001)?;
                }
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn match_stereo(
        &self,
        graph: &mut G,
        frame: &MultiFramePtr,
        as_keyframe: bool,
    ) -> Result<()> {
        let nc = frame.borrow().num_frames();
        let id = StateId::new(frame.borrow().id());
        let pose = graph.pose(id);
        for c0 in 0..nc {
            for c1 in c0 + 1..nc {
                if !frame.borrow().has_overlap(c0, c1) {
                    continue;
                }
                let t0 = pose.compose(frame.borrow().t_sc(c0));
                let t1 = pose.compose(frame.borrow().t_sc(c1));
                let n0 = frame.borrow().num_keypoints(c0);
                for k0 in 0..n0 {
                    let best = {
                        let mf = frame.borrow();
                        let mut best = None;
                        let mut distance = self.brisk_matching_threshold;
                        for k1 in 0..mf.num_keypoints(c1) {
                            let d = hamming(
                                mf.keypoint_descriptor(c0, k0),
                                mf.keypoint_descriptor(c1, k1),
                            );
                            if d >= distance {
                                continue;
                            }
                            let sigma = (keypoint_size(&mf, c0, k0) / focal(&mf, c0))
                                .max(keypoint_size(&mf, c1, k1) / focal(&mf, c1))
                                * 0.125;
                            if let Some((point, initialized, _)) =
                                triangulate_pair((&mf, c0, k0, &t0), (&mf, c1, k1, &t1), sigma, 0.1)
                            {
                                distance = d;
                                best = Some((k1, point, initialized));
                            }
                        }
                        best
                    };
                    let Some((k1, point, initialized)) = best else {
                        continue;
                    };
                    let (id0, id1) = {
                        let mf = frame.borrow();
                        (
                            LandmarkId::new(mf.landmark_id(c0, k0)),
                            LandmarkId::new(mf.landmark_id(c1, k1)),
                        )
                    };
                    let mut add0 = false;
                    let mut add1 = false;
                    let lm;
                    if id0.is_initialised() && id1.is_initialised() {
                        if id0 != id1 {
                            graph.merge_landmark(id1, id0)?;
                        }
                        if !graph.is_landmark_initialised(id0)? && initialized {
                            graph.set_landmark(id0, point, true)?;
                        }
                        continue;
                    } else if id1.is_initialised() {
                        lm = id1;
                        add0 = true;
                    } else if id0.is_initialised() {
                        lm = id0;
                        add1 = true;
                    } else {
                        if !as_keyframe {
                            continue;
                        }
                        lm = graph.add_landmark(point, initialized)?;
                        add0 = true;
                        add1 = true;
                    }
                    let effective = graph.landmark(lm)?;
                    for (c, k, t, add) in [(c0, k0, &t0, add0), (c1, k1, &t1, add1)] {
                        if add
                            && valid_reprojection(
                                &frame.borrow(),
                                c,
                                k,
                                &t.inverse(),
                                &effective,
                                4.0,
                            )
                        {
                            frame.borrow_mut().set_landmark_id(c, k, lm.value());
                            graph.add_observation(lm, KeypointIdentifier::new(id.value(), c, k))?;
                        }
                    }
                }
            }
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn match_motion_stereo(
        &self,
        graph: &mut G,
        parameters: &ViParameters,
        current: StateId,
    ) -> Result<(usize, bool)> {
        let ids = graph.frame_ids();
        if ids.len() < 2 {
            return Ok((0, true));
        }
        let previous = ids[ids.len() - 2];
        let mut all = graph.keyframes();
        for id in graph.imu_frames() {
            if graph.is_keyframe(id) {
                all.insert(id);
            }
        }
        let prev = graph.frame(previous);
        let mut overlaps = all
            .into_iter()
            .map(|id| {
                let overlap = if id == previous {
                    1.0
                } else {
                    landmark_overlap(
                        &prev.borrow(),
                        &graph.frame(id).borrow(),
                        0.09 * parameters.frontend.detection_threshold / 36.0,
                    )
                };
                (overlap, id)
            })
            .collect::<Vec<_>>();
        overlaps.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let frame1 = graph.frame(current);
        let t1 = graph.pose(current);
        let mut count = 0;
        let mut rotation_only = true;
        let mut first = true;
        for (overlap, older) in overlaps.into_iter().rev() {
            if overlap <= 1.0e-8 {
                break;
            }
            let frame0 = graph.frame(older);
            let t0 = graph.pose(older);
            for c in 0..self.num_cameras {
                let twc0 = t0.compose(&graph.extrinsics(older, c)?);
                let twc1 = t1.compose(&graph.extrinsics(current, c)?);
                let match_mask = self.motion_stereo_candidates(
                    graph,
                    (&frame0, &frame1),
                    (older, c),
                    (&twc0, &twc1),
                )?;
                for (k0, k1, point, initialized, quality) in match_mask {
                    if frame1.borrow().landmark_id(c, k1) != 0 {
                        continue;
                    }
                    let kid0 = KeypointIdentifier::new(older.value(), c, k0);
                    let old = LandmarkId::new(frame0.borrow().landmark_id(c, k0));
                    if graph.is_observed(kid0) {
                        continue;
                    }
                    let lm = if old.is_initialised() {
                        if !graph.is_landmark_added(old) {
                            continue;
                        }
                        let p = graph.get_landmark(old)?;
                        if p.is_initialised {
                            continue;
                        }
                        if p.quality < quality {
                            graph.set_landmark(old, point, initialized)?;
                        }
                        old
                    } else {
                        let id = graph.add_landmark(point, initialized)?;
                        frame0.borrow_mut().set_landmark_id(c, k0, id.value());
                        graph.add_observation(id, kid0)?;
                        id
                    };
                    frame1.borrow_mut().set_landmark_id(c, k1, lm.value());
                    graph.add_observation(lm, KeypointIdentifier::new(current.value(), c, k1))?;
                    count += 1;
                }
            }
            let ro = if self.is_initialized {
                false
            } else {
                self.run_ransac_2d2d(graph, parameters, current, older, true, true)?
                    .1
            };
            if first {
                rotation_only = ro;
                first = false;
            }
        }
        Ok((count, rotation_only))
    }
    fn motion_stereo_candidates(
        &self,
        graph: &G,
        (frame0, frame1): (&MultiFramePtr, &MultiFramePtr),
        (older, c): (StateId, usize),
        (twc0, twc1): (&Transformation, &Transformation),
    ) -> Result<Vec<MotionStereoMatch>> {
        let mut match_mask = Vec::new();
        {
            let f0 = frame0.borrow();
            let f1 = frame1.borrow();
            for k0 in 0..f0.num_keypoints(c) {
                let id0 = LandmarkId::new(f0.landmark_id(c, k0));
                if id0.is_initialised()
                    && (!graph.is_landmark_added(id0) || graph.is_landmark_initialised(id0)?)
                {
                    continue;
                }
                let kid0 = KeypointIdentifier::new(older.value(), c, k0);
                if graph.is_observed(kid0) {
                    continue;
                }
                let mut best = None;
                let mut distance = self.brisk_matching_threshold;
                for k1 in 0..f1.num_keypoints(c) {
                    if f1.landmark_id(c, k1) != 0 {
                        continue;
                    }
                    let d = hamming(f0.keypoint_descriptor(c, k0), f1.keypoint_descriptor(c, k1));
                    if d >= distance {
                        continue;
                    }
                    let sigma = keypoint_size(&f0, c, k0) / focal(&f0, c) * 0.125;
                    if let Some((point, initialized, quality)) =
                        triangulate_pair((&f0, c, k0, twc0), (&f1, c, k1, twc1), sigma, 0.2)
                    {
                        distance = d;
                        best = Some((k0, k1, point, initialized, quality));
                    }
                }
                if let Some(m) = best {
                    let mut projection = Vector2::zeros();
                    let status = f0.geometry(c).project_homogeneous(
                        &twc1.inverse().transform_homogeneous(&m.2),
                        &mut projection,
                    );
                    if status == ProjectionStatus::Successful
                        && (keypoint(&f1, c, m.1) - projection).norm() < 4.0
                    {
                        match_mask.push(m);
                    }
                }
            }
        }
        Ok(match_mask)
    }

    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    ///
    /// # Panics
    /// Panics if a keypoint-index lookup is absent after its correspondence validation.
    pub fn match_to_map(
        &self,
        graph: &mut G,
        parameters: &ViParameters,
        current: StateId,
        exclusive: Option<&BTreeSet<LandmarkId>>,
    ) -> Result<usize> {
        if graph.frame_ids().len() < 2 {
            return Ok(0);
        }
        let frame = graph.frame(current);
        let mut pose = graph.pose(current);
        let points = graph.landmarks();
        let mut count;
        let mut reprojection_error;
        let threads = parameters.frontend.num_matching_threads as usize;
        if threads == 0 {
            return Err(frontend_error("num_matching_threads must be positive"));
        }
        let mut merges = Vec::new();
        let InitializedMatches {
            prepared,
            count: matched,
            reprojection_error: error,
        } = self.match_initialized_landmarks(
            graph,
            parameters,
            (current, &frame),
            &pose,
            (&points, exclusive),
            &mut merges,
        )?;
        count = matched;
        reprojection_error = error;
        reprojection_error /= (threads * self.num_cameras) as f64;
        let mut iterations = 2;
        let mut run_ransac = !parameters.imu.r#use;
        let strict = 3.0 + focal(&frame.borrow(), 0) * 0.006;
        if reprojection_error > strict {
            run_ransac = true;
            iterations += 2;
        }
        let mut second_ransac = false;
        if run_ransac {
            if !self.run_ransac_3d2d(graph, &frame, true, true)? {
                iterations += 4;
                second_ransac = true;
            }
            pose = graph.pose(current);
        }
        if exclusive.is_none() && count > 3 {
            graph.optimise_realtime(
                iterations,
                parameters.estimator.realtime_num_threads,
                self.is_initialized,
            )?;
            self.remove_outliers(graph, &frame)?;
            graph.optimise_realtime(
                2,
                parameters.estimator.realtime_num_threads,
                self.is_initialized,
            )?;
            pose = graph.pose(current);
        }
        if count <= 3 && self.is_initialized {
            second_ransac = true;
        }
        self.match_bearing_landmarks(
            graph,
            (current, &frame),
            &pose,
            &prepared,
            exclusive,
            (&mut count, &mut merges),
        )?;
        if exclusive.is_some() {
            let (from, into): (Vec<_>, Vec<_>) = merges.into_iter().unzip();
            graph.merge_landmarks(&from, &into).map_err(|error| {
                error
                    .with_context("stage", "Frontend::match_to_map")
                    .with_context("frame_id", current.value())
            })?;
        }
        if second_ransac {
            if !self.run_ransac_3d2d(graph, &frame, true, true)? {
                iterations += 4;
            }
            graph.optimise_realtime(
                iterations,
                parameters.estimator.realtime_num_threads,
                self.is_initialized,
            )?;
        }
        Ok(count)
    }
    fn match_bearing_landmarks(
        &self,
        graph: &mut G,
        (current, frame): (StateId, &MultiFramePtr),
        pose: &Transformation,
        prepared: &[MapCandidates],
        exclusive: Option<&BTreeSet<LandmarkId>>,
        (count, merges): (&mut usize, &mut Vec<(LandmarkId, LandmarkId)>),
    ) -> Result<()> {
        for (c, candidates) in prepared.iter().enumerate().take(self.num_cameras) {
            let twc = pose.compose(frame.borrow().t_sc(c));
            let sigma = 1.0 / focal(&frame.borrow(), c);
            let cos6 = (6.0 * sigma).cos();
            let assignments = self.triangulate_map_candidates(
                (frame, c),
                candidates,
                &twc,
                (sigma, cos6),
                exclusive,
            );
            for (k, lm, hp) in assignments {
                let previous = LandmarkId::new(frame.borrow().landmark_id(c, k));
                let kid = KeypointIdentifier::new(current.value(), c, k);
                if previous.is_initialised() && exclusive.is_some() {
                    graph.remove_observation(kid)?;
                    merges.push((previous, lm));
                }
                let point = graph.get_landmark(lm)?;
                if let Some(hp) = hp {
                    let mut valid = true;
                    for obs in &point.observations {
                        let old = graph.frame(StateId::new(obs.frame_id));
                        let t = graph
                            .pose(StateId::new(obs.frame_id))
                            .compose(
                                &graph.extrinsics(StateId::new(obs.frame_id), obs.camera_index)?,
                            )
                            .inverse();
                        if !valid_reprojection(
                            &old.borrow(),
                            obs.camera_index,
                            obs.keypoint_index,
                            &t,
                            &hp,
                            4.0,
                        ) {
                            valid = false;
                            break;
                        }
                    }
                    if !valid {
                        continue;
                    }
                }
                let effective = if let Some(hp) = hp.filter(|_| !point.is_initialised) {
                    hp
                } else {
                    point.point
                };
                if !valid_reprojection(&frame.borrow(), c, k, &twc.inverse(), &effective, 4.0) {
                    continue;
                }
                if hp.is_some() && !point.is_initialised {
                    graph.set_landmark(lm, effective, true)?;
                }
                frame.borrow_mut().set_landmark_id(c, k, lm.value());
                graph.add_observation(lm, kid)?;
                *count += 1;
            }
        }
        Ok(())
    }

    fn triangulate_map_candidates(
        &self,
        (frame, c): (&MultiFramePtr, usize),
        candidates: &MapCandidates,
        twc: &Transformation,
        (sigma, cos6): (f64, f64),
        exclusive: Option<&BTreeSet<LandmarkId>>,
    ) -> Vec<MapAssignment> {
        let mut assignments = Vec::new();
        {
            let mf = frame.borrow();
            for k in 0..mf.num_keypoints(c) {
                let previous = LandmarkId::new(mf.landmark_id(c, k));
                if previous.is_initialised() && exclusive.is_none() {
                    continue;
                }
                let Some(e1) = bearing(&mf, c, k).map(|e| twc.rotate_vector(&e)) else {
                    continue;
                };
                let mut distance = self.brisk_matching_threshold;
                let mut best = None;
                let mut triangulated = None;
                for (&lm, candidate) in candidates {
                    if candidate.is_3d {
                        continue;
                    }
                    for descriptor in &candidate.descriptors {
                        let d = hamming(mf.keypoint_descriptor(c, k), &descriptor.descriptor);
                        if d >= distance {
                            continue;
                        }
                        let e0 = descriptor.bearing;
                        let center = descriptor.center;
                        if e0.dot(&e1) < cos6 {
                            let et = normalized(twc.translation() - center);
                            let n0 = normalized(e0.cross(&et));
                            let n1 = normalized(e1.cross(&et));
                            if n0.dot(&n1) < cos6 {
                                continue;
                            }
                            if e0.cross(&e1).dot(&normalized(n0 + n0)) > 0.0 {
                                continue;
                            }
                        }
                        let (mut valid, mut parallel) = (false, false);
                        let hp = triangulate_fast(
                            &center,
                            &e0,
                            &twc.translation(),
                            &e1,
                            sigma,
                            &mut valid,
                            &mut parallel,
                        );
                        let point = hp.fixed_rows::<3>(0).into_owned() / hp[3];
                        if !valid
                            || (point - center).norm() < 0.2
                            || (point - twc.translation()).norm() < 0.2
                        {
                            continue;
                        }
                        if lm == previous {
                            break;
                        }
                        distance = d;
                        if !parallel {
                            triangulated = Some(hp);
                        }
                        best = Some((lm, triangulated));
                    }
                }
                if let Some((lm, hp)) = best {
                    assignments.push((k, lm, hp));
                }
            }
        }
        assignments
    }

    fn match_initialized_landmarks(
        &self,
        graph: &mut G,
        parameters: &ViParameters,
        (current, frame): (StateId, &MultiFramePtr),
        pose: &Transformation,
        (points, exclusive): (&MapPoints, Option<&BTreeSet<LandmarkId>>),
        merges: &mut Vec<(LandmarkId, LandmarkId)>,
    ) -> Result<InitializedMatches> {
        let mut prepared = Vec::new();
        let mut count = 0;
        let mut reprojection_error = 0.0;
        let threads = parameters.frontend.num_matching_threads as usize;
        for c in 0..self.num_cameras {
            if frame.borrow().num_keypoints(c) == 0 {
                prepared.push(BTreeMap::new());
                continue;
            }
            let candidates = Self::prepare_landmarks(
                graph,
                &frame.borrow(),
                pose,
                c,
                points,
                parameters,
                exclusive,
            );
            let mut assignment = vec![None; frame.borrow().num_keypoints(c)];
            let mut distances = vec![self.brisk_matching_threshold; assignment.len()];
            let mut errors = vec![0.0; threads];
            let mut ctrs = vec![0usize; threads];
            if candidates.values().any(|candidate| candidate.is_3d) {
                let mf = frame.borrow();
                let threshold =
                    3.0 + focal(&mf, c) * if parameters.imu.r#use { 0.06 } else { 0.34 };
                let threshold_squared = threshold * threshold;
                let keypoints = ProjectedKeypointIndex::new(
                    (0..mf.num_keypoints(c))
                        .map(|k| keypoint(&mf, c, k))
                        .collect(),
                );
                let mut nearby = Vec::new();
                for (&lm, candidate) in &candidates {
                    if !candidate.is_3d {
                        continue;
                    }
                    keypoints.query(&candidate.projection, threshold_squared, &mut nearby);
                    for &k in &nearby {
                        if exclusive.is_none() && mf.landmark_id(c, k) != 0 {
                            continue;
                        }
                        let pixel = keypoints.pixels[k];
                        let error = (candidate.projection - pixel).norm_squared();
                        if error > threshold_squared {
                            continue;
                        }
                        for descriptor in &candidate.descriptors {
                            let distance =
                                hamming(mf.keypoint_descriptor(c, k), &descriptor.descriptor);
                            if distance < distances[k] {
                                distances[k] = distance;
                                assignment[k] = Some(lm);
                                let segment = mf.num_keypoints(c) / threads;
                                let thread = k
                                    .checked_div(segment)
                                    .map_or(threads - 1, |index| index.min(threads - 1));
                                ctrs[thread] += 1;
                                errors[thread] += error.sqrt();
                            }
                        }
                    }
                }
            }
            for (t, &n) in ctrs.iter().enumerate() {
                reprojection_error += errors[t] / n as f64;
            }
            for (k, lm) in assignment.into_iter().enumerate() {
                if let Some(lm) = lm {
                    let previous = LandmarkId::new(frame.borrow().landmark_id(c, k));
                    let kid = KeypointIdentifier::new(current.value(), c, k);
                    if previous.is_initialised() && exclusive.is_some() {
                        graph.remove_observation(kid)?;
                        merges.push((previous, lm));
                    }
                    frame.borrow_mut().set_landmark_id(c, k, lm.value());
                    graph.add_observation(lm, kid)?;
                    if candidates[&lm].ignore {
                        graph.set_observation_information(kid, Matrix2::identity() * 0.00001)?;
                    }
                    count += 1;
                }
            }
            prepared.push(candidates);
        }
        Ok(InitializedMatches {
            prepared,
            count,
            reprojection_error,
        })
    }

    fn prepare_landmarks(
        graph: &G,
        frame: &MultiFrame,
        pose: &Transformation,
        camera_index: usize,
        points: &MapPoints,
        parameters: &ViParameters,
        exclusive: Option<&BTreeSet<LandmarkId>>,
    ) -> BTreeMap<LandmarkId, LandmarkToMatch> {
        let twc = pose.compose(frame.t_sc(camera_index));
        let tcw = twc.inverse();
        let focal_scale = 2.0 * focal(frame, camera_index);
        let threshold = 3.0 + focal_scale * 0.5 * if parameters.imu.r#use { 0.06 } else { 0.34 };
        let mut candidates = BTreeMap::new();
        for (&id, point) in points {
            if exclusive.is_some_and(|ids| !ids.contains(&id)) {
                continue;
            }
            let point_value = point.point.fixed_rows::<3>(0).into_owned() / point.point[3];
            let ray = point_value - twc.translation();
            let residual = normalized(ray);
            let range = 0.01f64.max(ray.norm());
            let mut projection = Vector2::zeros();
            let status = frame
                .geometry(camera_index)
                .project_homogeneous(&tcw.transform_homogeneous(&point.point), &mut projection);
            if matches!(status, ProjectionStatus::Invalid | ProjectionStatus::Behind)
                || projection.x < -threshold
                || projection.y < -threshold
                || projection.x > f64::from(frame.geometry(camera_index).image_width()) + threshold
                || projection.y > f64::from(frame.geometry(camera_index).image_height()) + threshold
            {
                continue;
            }
            let mut descriptor_slots: [Option<(f64, MapDescriptor)>; 3] = [None, None, None];
            let mut is_3d = false;
            for kid in point.observations.iter().rev() {
                let old = graph.frame(StateId::new(kid.frame_id));
                let old = old.borrow();
                let old_twc = graph
                    .pose(StateId::new(kid.frame_id))
                    .compose(frame.t_sc(kid.camera_index));
                let old_ray = point_value - old_twc.translation();
                if !is_3d {
                    let close = ray - (0.2 / focal_scale / point.quality) * old_ray;
                    if normalized(ray).dot(&normalized(close)) > (10.0 / focal_scale).cos() {
                        is_3d = true;
                    }
                }
                let cosine = residual.dot(&normalized(old_ray));
                let scale = (range - old_ray.norm()).abs() / range;
                if exclusive.is_none() && (cosine < 0.6f64.cos() || scale > 0.5) {
                    continue;
                }
                let score_sum = cosine.acos() / 0.6 + scale / 0.5;
                let score = 0.5 * score_sum;
                let mut worst = 0.0;
                let mut slot = 0;
                for (index, item) in descriptor_slots.iter().enumerate() {
                    let observation = item.as_ref().map_or(1.0, |value| value.0);
                    if observation > worst {
                        worst = observation;
                        slot = index;
                    }
                }
                if score < worst
                    && let (Some(bearing), Ok(descriptor)) = (
                        bearing(&old, kid.camera_index, kid.keypoint_index),
                        descriptor(&old, kid.camera_index, kid.keypoint_index),
                    )
                {
                    descriptor_slots[slot] = Some((
                        score,
                        MapDescriptor {
                            descriptor,
                            bearing: old_twc.rotate_vector(&bearing),
                            center: old_twc.translation(),
                        },
                    ));
                }
            }
            let descriptors = descriptor_slots
                .into_iter()
                .flatten()
                .map(|value| value.1)
                .collect::<Vec<_>>();
            if descriptors.is_empty() {
                continue;
            }
            candidates.insert(
                id,
                LandmarkToMatch {
                    is_3d,
                    projection,
                    descriptors,
                    ignore: point.classification == 10 || point.classification == 11,
                },
            );
        }
        candidates
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn run_ransac_3d2d(
        &self,
        graph: &mut G,
        frame: &MultiFramePtr,
        initialize_pose: bool,
        remove_outliers: bool,
    ) -> Result<bool> {
        if graph.frame_ids().len() < 2 {
            return Ok(false);
        }
        let landmarks = graph.landmarks();
        let mf = frame.borrow();
        let view = BorrowedView {
            a: &mf,
            b: None,
            landmarks: &landmarks,
        };
        let mut adapter = FrameNoncentralAbsoluteAdapter::new(&view, mf.camera_system(), &mf);
        let n = adapter.number_correspondences();
        if n < 10 {
            return Ok(n != 0);
        }
        let mut problem =
            FrameAbsolutePoseSacProblem::new(&mut adapter, AbsoluteAlgorithm::Gp3p, &self.solver);
        let result = compute_ransac(&mut problem, n, 16.0, 50)?;
        drop(problem);
        let Some((model, inliers)) = result else {
            return Ok(false);
        };
        if inliers.len() < 10 || inliers.len() as f64 / (n as f64) <= 0.7 {
            return Ok(false);
        }
        let id = StateId::new(mf.id());
        let removals = (0..n)
            .filter(|k| !inliers.contains(k))
            .map(|k| {
                KeypointIdentifier::new(mf.id(), adapter.cam_index(k), adapter.keypoint_index(k))
            })
            .collect::<Vec<_>>();
        drop(mf);
        if remove_outliers {
            for kid in removals {
                graph.remove_observation(kid)?;
            }
        }
        if initialize_pose {
            graph.set_pose(id, transformation(&model))?;
        }
        Ok(true)
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn run_ransac_2d2d(
        &self,
        graph: &mut G,
        _parameters: &ViParameters,
        current: StateId,
        older: StateId,
        _initialize_pose: bool,
        remove_outliers: bool,
    ) -> Result<(i32, bool)> {
        let fa = graph.frame(older);
        let fb = graph.frame(current);
        let landmarks = graph.landmarks();
        let mut total = 0;
        let mut rotation_success = false;
        let mut relative_success = false;
        let mut rotation_only = false;
        let mut removals = Vec::new();
        {
            let a = fa.borrow();
            let b = fb.borrow();
            let view = BorrowedView {
                a: &a,
                b: Some(&b),
                landmarks: &landmarks,
            };
            for c in 0..self.num_cameras {
                let mut adapter = FrameRelativeAdapter::new(
                    &view,
                    a.camera_system(),
                    older.value(),
                    c,
                    current.value(),
                    c,
                );
                let n = adapter.number_correspondences();
                if n < 10 {
                    continue;
                }
                let rotation = {
                    let mut problem = FrameRotationOnlySacProblem::new(&adapter, &self.solver);
                    compute_ransac(&mut problem, n, 9.0, 50)?
                };
                let relative = {
                    let mut problem = FrameRelativePoseSacProblem::new(
                        &mut adapter,
                        RelativeAlgorithm::Stewenius,
                        &self.solver,
                    );
                    compute_ransac(&mut problem, n, 9.0, 50)?
                };
                let nr = rotation.as_ref().map_or(0, |x| x.1.len());
                let nt = relative.as_ref().map_or(0, |x| x.1.len());
                let rr = nr as f32 / n as f32;
                let tr = nt as f32 / n as f32;

                let selected = if rr > tr || rr > 0.8 {
                    rotation_success |= nr > 10;
                    rotation_only = true;
                    total += nr as i32;
                    rotation.as_ref().map(|x| &x.1)
                } else {
                    relative_success |= nt > 10 && tr > 0.8;
                    total += nt as i32;
                    relative.as_ref().map(|x| &x.1)
                };
                if !rotation_success && !relative_success {
                    continue;
                }
                if remove_outliers {
                    for k in 0..n {
                        if !selected.is_some_and(|s| s.contains(&k)) {
                            let key = adapter.match_keypoint_idx_b(k);
                            if b.landmark_id(c, key) != 0 {
                                removals.push(KeypointIdentifier::new(current.value(), c, key));
                            }
                        }
                    }
                }
            }
        }
        for kid in removals {
            graph.remove_observation(kid)?;
        }
        if relative_success || rotation_success {
            Ok((total, rotation_only))
        } else {
            Ok((-1, true))
        }
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn verify_recognised_place(
        &self,
        graph: &G,
        parameters: &ViParameters,
        current: &MultiFrame,
        old: &MultiFrame,
        min_inliers: usize,
    ) -> Result<Option<(Transformation, SMatrix<f64, 6, 6>)>> {
        let LoopLandmarkData {
            descriptors,
            landmarks,
        } = Self::loop_landmark_data(parameters, old)?;
        let mut points = BTreeMap::new();
        let mut match_mask = BTreeMap::new();
        let mut count = 0;
        for (&id, &point) in &landmarks {
            for c in 0..self.num_cameras {
                let mut best = self.brisk_matching_threshold;
                let mut selected = None;
                for old_descriptor in &descriptors[&id] {
                    for k in 0..current.num_keypoints(c) {
                        let distance = hamming(current.keypoint_descriptor(c, k), old_descriptor);
                        if distance < best {
                            best = distance;
                            selected = Some(k);
                        }
                    }
                }
                if let Some(k) = selected {
                    count += 1;
                    points.insert(id.value(), point);
                    match_mask.insert(KeypointIdentifier::new(current.id(), c, k), id.value());
                }
            }
        }
        if count < min_inliers || points.len() < 8 {
            return Ok(None);
        }
        let mut adapter = LoopclosureNoncentralAbsoluteAdapter::new(
            &points,
            &match_mask,
            current.camera_system(),
            current,
        );
        let n = adapter.number_correspondences();
        if n < 7 {
            return Ok(None);
        }
        let mut problem =
            FrameAbsolutePoseSacProblem::new(&mut adapter, AbsoluteAlgorithm::Gp3p, &self.solver);
        let Some((model, inliers)) = compute_ransac(&mut problem, n, 16.0, 50)? else {
            return Ok(None);
        };
        drop(problem);
        if inliers.len() < min_inliers || inliers.len() as f64 / (n as f64) < 0.7 {
            return Ok(None);
        }
        let average = self.descriptor_spread(current, &adapter, &inliers);
        if average < 182.0 && inliers.len() < 20 {
            return Ok(None);
        }
        // 无畸变相机同样走回环精化：后续残差计算经相机模型投影，不区分畸变类型，
        // 上游也没有这个拒绝分支。
        let mut residuals = Vec::new();
        for &i in &inliers {
            let c = adapter.cam_index(i);
            let k = adapter.keypoint_index(i);
            let kid = KeypointIdentifier::new(current.id(), c, k);
            let size = keypoint_size(current, c, k);
            residuals.push(LoopResidual {
                camera: c,
                keypoint: k,
                point: points[&match_mask[&kid]],
                measurement: keypoint(current, c, k),
                information: Matrix2::identity() * (64.0 / (size * size)),
                extrinsics: graph.extrinsics(StateId::new(current.id()), c)?,
            });
        }
        let refined = self.solver.refine_loop_pose(
            current,
            &transformation(&model),
            &residuals,
            parameters.estimator.realtime_max_iterations,
            parameters.estimator.realtime_num_threads,
        )?;
        if refined.residuals.len() != residuals.len() {
            return Err(frontend_error("loop refinement residual count mismatch"));
        }
        let mut hessian = SMatrix::<f64, 6, 6>::zeros();
        let mut outliers = 0;
        for (error, jacobian) in refined.residuals {
            if error.norm() > 3.0 {
                outliers += 1;
            } else {
                hessian += jacobian.transpose() * jacobian;
            }
        }
        let final_inliers = inliers.len() - outliers;
        if final_inliers < min_inliers || final_inliers as f64 / (n as f64) < 0.7 {
            return Ok(None);
        }
        Ok(Some((refined.pose, hessian)))
    }
    fn loop_landmark_data(parameters: &ViParameters, old: &MultiFrame) -> Result<LoopLandmarkData> {
        let mut descriptors = BTreeMap::<LandmarkId, Vec<Descriptor>>::new();
        let mut landmarks = BTreeMap::new();
        for c in 0..old.num_frames() {
            for k in 0..old.num_keypoints(c) {
                let id = LandmarkId::new(old.landmark_id(c, k));
                if !id.is_initialised() {
                    continue;
                }
                let mut point = Vector4::zeros();
                let mut initialized = false;
                old.get_landmark(c, k, &mut point, &mut initialized);
                if !initialized || point.norm() < 1.0e-12 {
                    continue;
                }
                if parameters.frontend.use_cnn && old.is_classified(c) {
                    let mut classification = [0.0; 19];
                    old.get_classification(c, k, &mut classification);
                    if classification[10] > 3.5 || classification[11] > 53.5 {
                        continue;
                    }
                }
                descriptors
                    .entry(id)
                    .or_default()
                    .push(descriptor(old, c, k)?);
                landmarks.entry(id).or_insert(point);
            }
        }
        Ok(LoopLandmarkData {
            descriptors,
            landmarks,
        })
    }

    fn descriptor_spread(
        &self,
        current: &MultiFrame,
        adapter: &LoopclosureNoncentralAbsoluteAdapter,
        inliers: &[usize],
    ) -> f32 {
        let mut stdev_sum = 0.0f32;
        for c in 0..self.num_cameras {
            let rows = inliers
                .iter()
                .filter(|&&i| adapter.cam_index(i) == c)
                .map(|&i| current.keypoint_descriptor(c, adapter.keypoint_index(i)))
                .collect::<Vec<_>>();
            if rows.is_empty() {
                continue;
            }
            let mut sum = 0.0f32;
            for bit in 0..384 {
                let mean = rows
                    .iter()
                    .map(|row| f32::from(row[bit / 8] & (1 << (bit % 8)) != 0))
                    .sum::<f32>()
                    / rows.len() as f32;
                let variance = rows
                    .iter()
                    .map(|row| {
                        let x = f32::from(row[bit / 8] & (1 << (bit % 8)) != 0) - mean;
                        x * x
                    })
                    .sum::<f32>()
                    / (rows.len() as f32 - 1.0);
                sum += variance.sqrt();
            }
            stdev_sum += rows.len() as f32 * sum;
        }

        stdev_sum / inliers.len() as f32
    }

    fn finish_ready_cnn_threads(&mut self) -> Result<()> {
        let mut pending = Vec::new();
        let mut ready = Vec::new();
        for job in self.cnn_threads.drain(..) {
            if job.handle.is_finished() {
                ready.push(job);
            } else {
                pending.push(job);
            }
        }
        self.cnn_threads = pending;
        finish_cnn_jobs(&mut ready)
    }
    /// # Errors
    /// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
    /// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
    pub fn end_cnn_threads(&mut self) -> Result<()> {
        finish_cnn_jobs(&mut self.cnn_threads)
    }
    pub fn get_brisk_detection_octaves(&self) -> usize {
        self.brisk.octaves
    }
    pub fn set_brisk_detection_octaves(&mut self, v: usize) {
        self.brisk.octaves = v;
    }
    pub fn get_brisk_detection_threshold(&self) -> f64 {
        self.brisk.threshold
    }
    pub fn set_brisk_detection_threshold(&mut self, v: f64) {
        self.brisk.threshold = v;
    }
    pub fn get_brisk_detection_absolute_threshold(&self) -> f64 {
        self.brisk.absolute_threshold
    }
    pub fn set_brisk_detection_absolute_threshold(&mut self, v: f64) {
        self.brisk.absolute_threshold = v;
    }
    pub fn get_brisk_detection_maximum_keypoints(&self) -> usize {
        self.brisk.maximum_keypoints
    }
    pub fn set_brisk_detection_maximum_keypoints(&mut self, v: usize) {
        self.brisk.maximum_keypoints = v;
    }
    pub fn get_brisk_description_rotation_invariance(&self) -> bool {
        self.brisk.rotation_invariance
    }
    pub fn set_brisk_description_rotation_invariance(&mut self, v: bool) {
        self.brisk.rotation_invariance = v;
    }
    pub fn get_brisk_description_scale_invariance(&self) -> bool {
        self.brisk.scale_invariance
    }
    pub fn set_brisk_description_scale_invariance(&mut self, v: bool) {
        self.brisk.scale_invariance = v;
    }
    pub fn get_brisk_matching_threshold(&self) -> f64 {
        self.brisk_matching_threshold
    }
    pub fn set_brisk_matching_threshold(&mut self, v: f64) {
        self.brisk_matching_threshold = v;
    }
    pub fn get_keyframe_insertion_overlap_threshold(&self) -> f32 {
        self.keyframe_insertion_overlap_threshold
    }
    pub fn set_keyframe_insertion_overlap_threshold(&mut self, v: f32) {
        self.keyframe_insertion_overlap_threshold = v;
    }
    pub fn is_initialized(&self) -> bool {
        self.is_initialized
    }
}
struct CnnJob {
    frame: MultiFramePtr,
    camera: usize,
    handle: std::thread::JoinHandle<Result<Vec<[f32; 19]>>>,
}
fn finish_cnn_jobs(jobs: &mut Vec<CnnJob>) -> Result<()> {
    let mut failure = None;
    for job in jobs.drain(..) {
        let result = job
            .handle
            .join()
            .map_err(|_| frontend_error("CNN worker panicked"))
            .and_then(|r| r);
        match result {
            Ok(values) => {
                if let Err(e) = job
                    .frame
                    .borrow_mut()
                    .frame_mut(job.camera)
                    .set_classifications(values)
                {
                    failure.get_or_insert(e);
                }
            }
            Err(e) => {
                failure.get_or_insert(e);
            }
        }
    }
    failure.map_or(Ok(()), Err)
}
impl<G, S, B> Drop for Frontend<G, S, B> {
    fn drop(&mut self) {
        if let Err(error) = finish_cnn_jobs(&mut self.cnn_threads) {
            log::error!("{error}");
        }
    }
}
struct MapDescriptor {
    descriptor: Descriptor,
    bearing: Vector3<f64>,
    center: Vector3<f64>,
}
struct LandmarkToMatch {
    is_3d: bool,
    projection: Vector2<f64>,
    descriptors: Vec<MapDescriptor>,
    ignore: bool,
}
/// Conservative horizontal search; the caller applies the circular distance gate.
struct ProjectedKeypointIndex {
    pixels: Vec<Vector2<f64>>,
    by_x: Vec<usize>,
    nonfinite: Vec<usize>,
}
impl ProjectedKeypointIndex {
    fn new(pixels: Vec<Vector2<f64>>) -> Self {
        let (mut by_x, nonfinite): (Vec<_>, Vec<_>) =
            (0..pixels.len()).partition(|&k| pixels[k].iter().all(|v| v.is_finite()));
        by_x.sort_unstable_by(|&a, &b| pixels[a].x.total_cmp(&pixels[b].x));
        Self {
            pixels,
            by_x,
            nonfinite,
        }
    }
    fn query(&self, projection: &Vector2<f64>, radius_squared: f64, indices: &mut Vec<usize>) {
        indices.clear();
        let range = if projection.iter().all(|v| v.is_finite())
            && radius_squared.is_finite()
            && radius_squared >= 0.0
        {
            // Squared differences use the same arithmetic as the circular gate;
            // rounded projection +/- radius bounds can exclude boundary points.
            let first = self.by_x.partition_point(|&k| {
                let dx = projection.x - self.pixels[k].x;
                self.pixels[k].x < projection.x && dx * dx > radius_squared
            });
            let last = self.by_x.partition_point(|&k| {
                let dx = projection.x - self.pixels[k].x;
                self.pixels[k].x <= projection.x || dx * dx <= radius_squared
            });
            first..last
        } else {
            0..self.by_x.len()
        };
        if range.len() + self.nonfinite.len() == self.pixels.len() {
            indices.extend(0..self.pixels.len());
            return;
        }
        indices.extend_from_slice(&self.by_x[range]);
        // Nonfinite coordinates must reach the caller's IEEE distance comparison.
        indices.extend_from_slice(&self.nonfinite);
        // Keep descriptor ties and reprojection-error accumulation in keypoint order.
        indices.sort_unstable();
    }
}
struct BorrowedView<'a> {
    a: &'a MultiFrame,
    b: Option<&'a MultiFrame>,
    landmarks: &'a MapPoints,
}
impl EstimatorView for BorrowedView<'_> {
    fn multi_frame(&self, id: StateId) -> &MultiFrame {
        if self.a.id() == id.value() {
            self.a
        } else {
            self.b
                .filter(|b| b.id() == id.value())
                .expect("requested frame must belong to adapter")
        }
    }
    fn is_landmark_added(&self, id: LandmarkId) -> bool {
        self.landmarks.contains_key(&id)
    }
    fn landmark(&self, id: LandmarkId) -> &MapPoint2 {
        &self.landmarks[&id]
    }
}
fn frontend_error(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message.into()).with_operation("Frontend")
}
fn descriptor(frame: &MultiFrame, camera_index: usize, keypoint: usize) -> Result<Descriptor> {
    frame
        .keypoint_descriptor(camera_index, keypoint)
        .try_into()
        .map_err(|_| frontend_error("BRISK descriptor must contain 48 bytes"))
}
fn descriptors(frame: &MultiFrame) -> Result<Vec<Descriptor>> {
    (0..frame.num_frames())
        .flat_map(|c| (0..frame.num_keypoints(c)).map(move |k| descriptor(frame, c, k)))
        .collect()
}
fn keypoint(frame: &MultiFrame, camera_index: usize, keypoint_value: usize) -> Vector2<f64> {
    let mut point = Vector2::zeros();
    frame.get_keypoint(camera_index, keypoint_value, &mut point);
    point
}
fn keypoint_size(frame: &MultiFrame, camera_index: usize, keypoint: usize) -> f64 {
    let mut size = 1.0;
    frame.get_keypoint_size(camera_index, keypoint, &mut size);
    size
}
fn focal(frame: &MultiFrame, c: usize) -> f64 {
    let mut p = DVector::zeros(frame.geometry(c).num_intrinsics());
    frame.geometry(c).get_intrinsics(&mut p);
    let focal_sum = p[0] + p[1];
    0.5 * focal_sum
}
fn normalized(v: Vector3<f64>) -> Vector3<f64> {
    let n = v.norm_squared();
    if n > 0.0 { v / n.sqrt() } else { v }
}
fn bearing(frame: &MultiFrame, camera_index: usize, keypoint: usize) -> Option<Vector3<f64>> {
    let mut residual = Vector3::zeros();
    frame
        .get_back_projection(camera_index, keypoint, &mut residual)
        .then(|| normalized(residual))
}
fn valid_reprojection(
    frame: &MultiFrame,
    camera_index: usize,
    keypoint_value: usize,
    tcw: &Transformation,
    point: &Vector4<f64>,
    threshold: f64,
) -> bool {
    let mut point_value = Vector2::zeros();
    frame
        .geometry(camera_index)
        .project_homogeneous(&tcw.transform_homogeneous(point), &mut point_value)
        == ProjectionStatus::Successful
        && (keypoint(frame, camera_index, keypoint_value) - point_value).norm() < threshold
}
fn valid_reprojection_inclusive(
    frame: &MultiFrame,
    camera_index: usize,
    keypoint_value: usize,
    tcw: &Transformation,
    point: &Vector4<f64>,
    threshold: f64,
) -> bool {
    let mut point_value = Vector2::zeros();
    frame
        .geometry(camera_index)
        .project_homogeneous(&tcw.transform_homogeneous(point), &mut point_value)
        == ProjectionStatus::Successful
        && (keypoint(frame, camera_index, keypoint_value) - point_value).norm() <= threshold
}
fn triangulate_pair(
    (a, ca, ka, ta): (&MultiFrame, usize, usize, &Transformation),
    (b, cb, kb, tb): (&MultiFrame, usize, usize, &Transformation),
    sigma: f64,
    min_depth: f64,
) -> Option<(Vector4<f64>, bool, f64)> {
    let ea = normalized(ta.rotate_vector(&bearing(a, ca, ka)?));
    let eb = normalized(tb.rotate_vector(&bearing(b, cb, kb)?));
    if ea.dot(&eb) < 0.5 {
        return None;
    }
    let (mut valid, mut parallel) = (false, false);
    let hp = triangulate_fast(
        &ta.translation(),
        &ea,
        &tb.translation(),
        &eb,
        sigma,
        &mut valid,
        &mut parallel,
    );
    let pa = ta.inverse().transform_homogeneous(&hp);
    let pb = tb.inverse().transform_homogeneous(&hp);
    if !valid || ea.dot(&eb) < 0.8 || pa.z / pa.w < min_depth || pb.z / pb.w < min_depth {
        return None;
    }
    let point = hp / hp.w;
    let quality = normalized(point.fixed_rows::<3>(0).into_owned() - ta.translation())
        .dot(&normalized(
            point.fixed_rows::<3>(0).into_owned() - tb.translation(),
        ))
        .acos();
    Some((point, !parallel, quality))
}
fn landmark_overlap(a: &MultiFrame, b: &MultiFrame, radius: f64) -> f64 {
    let ids = |frame: &MultiFrame| {
        (0..frame.num_frames())
            .flat_map(|c| (0..frame.num_keypoints(c)).map(move |k| frame.landmark_id(c, k)))
            .filter(|&id| id != 0)
            .collect::<BTreeSet<_>>()
    };
    let common = ids(a)
        .intersection(&ids(b))
        .copied()
        .collect::<BTreeSet<_>>();
    if common.is_empty() {
        return 0.0;
    }
    coverage_overlap_with_radius(a, |id| common.contains(&id), radius).min(
        coverage_overlap_with_radius(b, |id| common.contains(&id), radius),
    )
}
/// `OpenCV`'s integer filled circle rasterization on the 1/10-resolution coverage grid.
fn coverage_overlap(frame: &MultiFrame, matched: impl Fn(u64) -> bool) -> f64 {
    coverage_overlap_with_radius(frame, matched, 0.09)
}
fn coverage_overlap_with_radius(
    frame: &MultiFrame,
    matched: impl Fn(u64) -> bool,
    radius: f64,
) -> f64 {
    let mut intersection = 0;
    let mut union = 0;
    for c in 0..frame.num_frames() {
        let Some(image) = frame.image(c) else {
            continue;
        };
        let rows = image.rows() / 10;
        let cols = image.cols() / 10;
        let radius = ((rows.min(cols) as f64) * radius) as i32;
        let mut circles = Vec::new();
        let mut matched_circles = Vec::new();
        for k in 0..frame.num_keypoints(c) {
            let point = keypoint(frame, c, k) * 0.1;
            circles.push((point, radius));
            if matched(frame.landmark_id(c, k)) {
                matched_circles.push((point, radius));
            }
        }
        let detections = filled_circle_mask(rows, cols, &circles)
            .expect("coverage raster requires a nonnegative radius");
        let match_mask = filled_circle_mask(rows, cols, &matched_circles)
            .expect("coverage raster requires a nonnegative radius");
        for (i, m) in detections.into_iter().zip(match_mask) {
            intersection += usize::from(i && m);
            union += usize::from(i || m);
        }
    }
    intersection as f64 / union as f64
}
/// Rasterizes `OpenCV` `LINE_8/FILLED` integer disks. Centers undergo Point2f
/// conversion followed by cvRound (round-to-nearest, ties-to-even).
/// # Errors
/// Returns an error if required frames, cameras, landmarks or vocabulary data are absent,
/// measurement dimensions are invalid, or matching, geometry, graph or file operations fail.
pub fn filled_circle_mask(
    rows: usize,
    cols: usize,
    circles: &[(Vector2<f64>, i32)],
) -> Result<Vec<bool>> {
    let size = rows
        .checked_mul(cols)
        .ok_or_else(|| frontend_error("raster dimensions overflow"))?;
    let mut mask = vec![false; size];
    for &(center, radius) in circles {
        if radius < 0 {
            return Err(frontend_error("circle radius must be nonnegative"));
        }
        let center = center.cast::<f32>().cast::<f64>();
        let u = center.x.round_ties_even() as i32;
        let v = center.y.round_ties_even() as i32;
        let radius = i64::from(radius);
        for dy in -radius..=radius {
            let y = i64::from(v) + dy;
            if y < 0 || y >= rows as i64 {
                continue;
            }
            let dx = ((radius * radius - dy * dy) as f64).sqrt() as i64;
            for x in (i64::from(u) - dx).max(0)..=(i64::from(u) + dx).min(cols as i64 - 1) {
                mask[y as usize * cols + x as usize] = true;
            }
        }
    }
    Ok(mask)
}
fn filtered_bow(bow: &BowDatabase, features: &[Descriptor]) -> Vec<(StateId, f64)> {
    let original = bow.database.query(features, -1, -1);
    let mut by_id = original.clone();
    by_id.sort_by_key(|r| r.id);
    let n = original.len();
    let mut ids = BTreeSet::new();
    let mut suppressed = BTreeSet::new();
    for (f, r) in original.iter().enumerate() {
        let id = r.id as usize;
        if id >= n {
            continue;
        }
        if r.score < 0.375 {
            break;
        }
        if suppressed.contains(&f) {
            continue;
        }
        let start = id.saturating_sub(5);
        let end = (id + 5).min(n - 1);
        if (start..=end).any(|a| by_id[a].score > r.score) {
            continue;
        }
        suppressed.extend(start..=end);
        ids.insert(id);
    }
    ids.into_iter()
        .filter_map(|id| bow.pose_ids.get(id).map(|&pose| (pose, by_id[id].score)))
        .collect()
}
fn transformation(model: &SMatrix<f64, 3, 4>) -> Transformation {
    Transformation::new(
        model.column(3).into_owned(),
        quaternion_from_rotation_matrix(&model.fixed_view::<3, 3>(0, 0).into_owned()),
    )
}
trait ConsensusProblem {
    type Model: Clone;
    fn sample_size(&self) -> usize;
    fn model(&mut self, sample: &[usize]) -> Result<Option<Self::Model>>;
    fn scores(&mut self, model: &Self::Model, indices: &[usize], scores: &mut Vec<f64>);
}
impl<A: AbsoluteAdapter, S: AbsolutePoseSolver> ConsensusProblem
    for FrameAbsolutePoseSacProblem<'_, A, S>
{
    type Model = SMatrix<f64, 3, 4>;
    fn sample_size(&self) -> usize {
        self.sample_size()
    }
    fn model(&mut self, s: &[usize]) -> Result<Option<Self::Model>> {
        self.compute_model_coefficients(s)
    }
    fn scores(&mut self, m: &Self::Model, i: &[usize], scores: &mut Vec<f64>) {
        self.get_selected_distances_to_model(m, i, scores);
    }
}
impl<S: CentralRelativePoseSolver> ConsensusProblem for FrameRelativePoseSacProblem<'_, S> {
    type Model = SMatrix<f64, 3, 4>;
    fn sample_size(&self) -> usize {
        self.sample_size()
    }
    fn model(&mut self, s: &[usize]) -> Result<Option<Self::Model>> {
        self.compute_model_coefficients(s)
    }
    fn scores(&mut self, m: &Self::Model, i: &[usize], scores: &mut Vec<f64>) {
        self.get_selected_distances_to_model(m, i, scores);
    }
}
impl<S: RotationOnlySolver> ConsensusProblem for FrameRotationOnlySacProblem<'_, S> {
    type Model = nalgebra::Matrix3<f64>;
    fn sample_size(&self) -> usize {
        self.sample_size()
    }
    fn model(&mut self, s: &[usize]) -> Result<Option<Self::Model>> {
        self.compute_model_coefficients(s)
    }
    fn scores(&mut self, m: &Self::Model, i: &[usize], scores: &mut Vec<f64>) {
        self.get_selected_distances_to_model(m, i, scores);
    }
}
fn compute_ransac<P: ConsensusProblem>(
    problem: &mut P,
    n: usize,
    threshold: f64,
    max_iterations: usize,
) -> Result<Option<(P::Model, Vec<usize>)>> {
    let sample_size = problem.sample_size();
    if n < sample_size {
        return Ok(None);
    }
    let indices = (0..n).collect::<Vec<_>>();
    let mut sampling = firefly_opengv::sac::sample_consensus_problem::SamplingState::new(true);
    sampling.set_uniform_indices(n);
    let mut best = None;
    let mut best_count = -i32::MAX;
    let mut bound = 1.0;
    let mut iterations = 0;
    let mut skipped = 0;
    let mut sample = vec![0; sample_size];
    let mut scores = Vec::with_capacity(n);
    while (iterations as f64) < bound && skipped < max_iterations.saturating_mul(10) {
        sampling.draw_index_sample(&mut sample);
        let Some(model) = problem.model(&sample)? else {
            skipped += 1;
            continue;
        };
        scores.clear();
        problem.scores(&model, &indices, &mut scores);
        let inliers = scores.iter().filter(|score| **score < threshold).count();
        if inliers as i32 > best_count {
            best_count = inliers as i32;
            let w = f64::from(best_count) / n as f64;
            let p_no_outliers =
                (1.0 - w.powf(sample_size as f64)).clamp(f64::EPSILON, 1.0 - f64::EPSILON);
            bound = (1.0 - 0.99f64).ln() / p_no_outliers.ln();
            best = Some(model);
        }
        iterations += 1;
        if iterations > max_iterations {
            break;
        }
    }
    Ok(best.map(|model| {
        scores.clear();
        problem.scores(&model, &indices, &mut scores);
        let inliers = scores
            .iter()
            .enumerate()
            .filter(|(_, score)| **score < threshold)
            .map(|(i, _)| i)
            .collect();
        (model, inliers)
    }))
}

#[cfg(test)]
mod matching_tests {
    use super::*;

    use crate::{
        application::live::config::Config,
        cameras::{
            n_camera_system::{CameraType, DistortionType, NCameraSystem},
            no_distortion::NoDistortion,
            pinhole_camera::PinholeCamera,
        },
        ceres::vi_slam_backend::ViSlamBackend,
        frames::frame::KeyPoint,
        frontend::frame_absolute_pose_sac_problem::OpenGvSolver,
        sensors::measurements::{ImuMeasurement, ImuSensorReadings},
    };
    use purecv::core::{Matrix, types::Point2f};
    use std::{cell::RefCell, rc::Rc};

    fn stereo_merge_graph() -> (ViSlamBackend, ViParameters) {
        let mut parameters = Config::default().parameters().unwrap();
        parameters.frontend.num_matching_threads = 1;
        let mut rig = NCameraSystem::new();
        for camera in 0..2 {
            rig.add_camera(
                &Arc::new(Transformation::identity()),
                Arc::new(PinholeCamera::new(
                    (64, 48),
                    (60., 60., 31.5, 23.5),
                    NoDistortion,
                    camera,
                )),
                DistortionType::NoDistortion,
                true,
                CameraType::default(),
            );
        }
        parameters.n_camera_system = rig;
        let mut graph = ViSlamBackend::default();
        graph.add_imu(parameters.imu.clone());
        for _ in 0..2 {
            graph.add_camera(parameters.camera.clone());
        }
        let imu = (0..=30)
            .map(|i| {
                ImuMeasurement::new(
                    Timestamp::from_millisecond(i * 10).unwrap(),
                    ImuSensorReadings::new(
                        Vector3::zeros(),
                        Vector3::new(0., 0., parameters.imu.g),
                    ),
                    0,
                )
            })
            .collect();
        for state in 1..=2 {
            let mut frame = MultiFrame::new(
                &parameters.n_camera_system,
                Timestamp::from_millisecond(state * 100).unwrap(),
                state as u64,
            );
            for camera in 0..2 {
                let keypoints: Vec<_> = (0..3)
                    .map(|k| {
                        KeyPoint::new(
                            Point2f::new(11.5 + k as f32 * 20., 23.5),
                            12.,
                            -1.,
                            1.,
                            0,
                            -1,
                        )
                    })
                    .collect();
                frame.reset_keypoints(camera, &keypoints);
                frame.reset_descriptors(
                    camera,
                    Arc::new(Matrix::from_vec(3, 48, 1, vec![0; 3 * 48])),
                );
                frame.compute_back_projections(camera);
            }
            graph
                .add_states(Rc::new(RefCell::new(frame)), &imu, true)
                .unwrap();
            for k in 0..3 {
                let id = LandmarkId::new((state as u64 - 1) * 3 + k as u64 + 1);
                graph
                    .add_landmark(id, Vector4::new(k as f64 - 1., 0., 3., 1.), true)
                    .unwrap();
                for g in [&mut graph.realtime_graph, &mut graph.full_graph] {
                    g.set_landmark_quality(id, 1.).unwrap();
                }
                for camera in 0..2 {
                    graph
                        .multi_frame(StateId::new(state as u64))
                        .unwrap()
                        .borrow_mut()
                        .set_landmark_id(camera, k, id.value());
                    graph
                        .add_observation(id, StateId::new(state as u64), camera, k, true)
                        .unwrap();
                }
            }
        }
        (graph, parameters)
    }

    #[test]
    fn loop_matching_merges_shared_stereo_landmarks_as_one_batch() {
        let (mut graph, parameters) = stereo_merge_graph();
        let frontend = Frontend::<ViSlamBackend, OpenGvSolver, _>::with_vocabulary(
            2,
            &TemplatedVocabulary::default(),
            OpenGvSolver::default(),
            RustBriskFeatureFactory,
        );
        let exclusive = (1..=3).map(LandmarkId::new).collect();
        assert_eq!(
            frontend
                .match_to_map(&mut graph, &parameters, StateId::new(2), Some(&exclusive))
                .unwrap(),
            6
        );
        for g in [&graph.realtime_graph, &graph.full_graph] {
            assert_eq!(
                g.landmarks.keys().copied().collect::<Vec<_>>(),
                (1..=3).map(LandmarkId::new).collect::<Vec<_>>()
            );
            for id in 1..=3 {
                assert_eq!(g.landmark_info[&LandmarkId::new(id)].observations.len(), 4);
            }
        }
        for frame in graph.multi_frames.values() {
            for camera in 0..2 {
                for k in 0..3 {
                    assert_eq!(frame.borrow().landmark_id(camera, k), k as u64 + 1);
                }
            }
        }
    }

    fn accepted(
        index: &ProjectedKeypointIndex,
        projection: &Vector2<f64>,
        radius_squared: f64,
        candidates: impl IntoIterator<Item = usize>,
    ) -> Vec<usize> {
        candidates
            .into_iter()
            .filter(|&k| {
                let error = (projection - index.pixels[k]).norm_squared();
                error.partial_cmp(&radius_squared) != Some(std::cmp::Ordering::Greater)
            })
            .collect()
    }

    #[test]
    fn indexed_projection_gate_matches_exhaustive_search_and_keypoint_order() {
        let mut pixels: Vec<_> = (0..400)
            .map(|i| {
                let i = f64::from(i);
                Vector2::new((i * 0.7).sin() * 400.0, (i * 1.3).cos() * 240.0)
            })
            .collect();
        pixels.extend([
            Vector2::new(0.0, 0.0),
            Vector2::new(-0.0, 0.0),
            Vector2::new(3.0, 4.0),
            Vector2::new(-3.0, -4.0),
            Vector2::new(5.0, 0.0),
            Vector2::new(5.0_f64.next_up(), 0.0),
            Vector2::new(5.0_f64.next_down(), 0.0),
            Vector2::new(f64::NAN, 2.0),
            Vector2::new(1000.0, f64::NAN),
            Vector2::new(f64::INFINITY, 0.0),
            Vector2::new(0.0, f64::NEG_INFINITY),
            Vector2::new(f64::MAX, 0.0),
            Vector2::new(-f64::MAX, 0.0),
        ]);
        let index = ProjectedKeypointIndex::new(pixels);
        let mut nearby = Vec::new();
        for projection in [
            Vector2::zeros(),
            Vector2::new(0.1, -0.2),
            Vector2::new(-400.0, 240.0),
            Vector2::new(f64::MAX, 0.0),
            Vector2::new(f64::NAN, 1.0),
            Vector2::new(f64::INFINITY, 1.0),
        ] {
            for radius_squared in [0.0, 25.0, 200.0, f64::MAX, f64::INFINITY, f64::NAN] {
                index.query(&projection, radius_squared, &mut nearby);
                assert!(nearby.windows(2).all(|pair| pair[0] < pair[1]));
                assert_eq!(
                    accepted(&index, &projection, radius_squared, nearby.iter().copied()),
                    accepted(&index, &projection, radius_squared, 0..index.pixels.len()),
                    "projection={projection:?}, radius_squared={radius_squared}"
                );
            }
        }
        let empty = ProjectedKeypointIndex::new(Vec::new());
        empty.query(&Vector2::zeros(), 25.0, &mut nearby);
        assert_eq!(nearby, Vec::<usize>::new());
    }

    #[test]
    fn projection_search_limits_candidate_checks_to_the_horizontal_neighborhood() {
        let pixels = (0..10)
            .flat_map(|y| {
                (0..100)
                    .rev()
                    .map(move |x| Vector2::new(f64::from(x), f64::from(y)))
            })
            .collect();
        let index = ProjectedKeypointIndex::new(pixels);
        let projection = Vector2::new(50.0, 5.0);
        let mut nearby = Vec::new();
        index.query(&projection, 4.0, &mut nearby);
        assert_eq!(index.pixels.len(), 1000);
        assert_eq!(nearby.len(), 50);
        let matches = accepted(&index, &projection, 4.0, nearby.iter().copied());
        assert_eq!(matches.len(), 13);
        assert_eq!(matches, accepted(&index, &projection, 4.0, 0..1000));
        index.query(&projection, f64::INFINITY, &mut nearby);
        assert_eq!(nearby, (0..1000).collect::<Vec<_>>());
    }
}

#[cfg(test)]
mod consensus_tests {
    use super::*;

    struct FixedScores {
        calls: usize,
        inliers: usize,
        rejected: usize,
    }

    impl ConsensusProblem for FixedScores {
        type Model = usize;
        fn sample_size(&self) -> usize {
            2
        }
        fn model(&mut self, _: &[usize]) -> Result<Option<usize>> {
            self.calls += 1;
            Ok((self.calls > self.rejected).then_some(self.calls))
        }
        fn scores(&mut self, _: &usize, indices: &[usize], scores: &mut Vec<f64>) {
            scores.extend(
                indices
                    .iter()
                    .map(|&i| if i < self.inliers { 0.0 } else { 2.0 }),
            );
        }
    }

    #[test]
    fn ransac_retains_zero_inlier_model_and_source_iteration_budget() {
        for (inliers, rejected, expected_calls, expected_inliers) in [
            (0, 0, 51, Some(0)),
            (10, 0, 1, Some(10)),
            (5, 0, 17, Some(5)),
            (10, 3, 4, Some(10)),
            (10, 500, 500, None),
        ] {
            let mut problem = FixedScores {
                calls: 0,
                inliers,
                rejected,
            };
            let result = compute_ransac(&mut problem, 10, 1.0, 50).unwrap();
            assert_eq!(problem.calls, expected_calls);
            assert_eq!(result.map(|(_, indices)| indices.len()), expected_inliers);
        }
    }

    #[test]
    fn ransac_scores_models_independently_and_keeps_the_first_tied_model() {
        #[derive(Default)]
        struct ChangingScores {
            models: usize,
            scored_models: Vec<usize>,
        }
        impl ConsensusProblem for ChangingScores {
            type Model = usize;
            fn sample_size(&self) -> usize {
                2
            }
            fn model(&mut self, _: &[usize]) -> Result<Option<usize>> {
                self.models += 1;
                Ok(Some(self.models))
            }
            fn scores(&mut self, model: &usize, indices: &[usize], scores: &mut Vec<f64>) {
                assert_eq!(*scores, Vec::<f64>::new());
                self.scored_models.push(*model);
                scores.extend(indices.iter().map(|&i| match (*model, i) {
                    (1, 0..2) | (2 | 3, 2..) => 0.0,
                    (2 | 3, 0) => f64::NAN,
                    _ => 1.0,
                }));
            }
        }
        let mut problem = ChangingScores::default();
        let (model, inliers) = compute_ransac(&mut problem, 10, 1.0, 50).unwrap().unwrap();
        assert_eq!(model, 2);
        assert_eq!(inliers, (2..10).collect::<Vec<_>>());
        assert_eq!(problem.models, 5);
        assert_eq!(problem.scored_models, vec![1, 2, 3, 4, 5, 2]);
    }
}

impl<G, S, B> crate::frontend::vi_frontend_interface::ViFrontendInterface for Frontend<G, S, B>
where
    G: FrontendGraph,
    S: AbsolutePoseSolver + CentralRelativePoseSolver + RotationOnlySolver + LoopPoseSolver,
    B: BriskFeatureFactory,
{
    type Estimator = G;
    fn detect_and_describe(
        &mut self,
        camera_index: usize,
        frame_out: MultiFramePtr,
        t_wc: &Transformation,
        keypoints: Option<&[KeyPoint]>,
    ) -> bool {
        frontend_bool(Frontend::detect_and_describe(
            self,
            camera_index,
            &frame_out,
            t_wc,
            keypoints,
        ))
    }
    fn data_association_and_initialization(
        &mut self,
        estimator: &mut G,
        parameters: &ViParameters,
        frames_in_out: MultiFramePtr,
        kf_prior: bool,
        as_keyframe: &mut bool,
    ) -> bool {
        frontend_bool(Frontend::data_association_and_initialization(
            self,
            estimator,
            parameters,
            &frames_in_out,
            kf_prior,
            as_keyframe,
        ))
    }
    fn propagation(
        &self,
        imu_measurements: &ImuMeasurementDeque,
        imu_params: &ImuParameters,
        t_ws_propagated: &mut Transformation,
        speed_and_biases: &mut SpeedAndBias,
        (t_start, t_end): (Timestamp, Timestamp),
        (covariance, jacobian): (
            Option<&mut SMatrix<f64, 15, 15>>,
            Option<&mut SMatrix<f64, 15, 15>>,
        ),
    ) -> bool {
        frontend_bool(Frontend::propagation(
            self,
            imu_measurements,
            imu_params,
            t_ws_propagated,
            speed_and_biases,
            (t_start, t_end),
            (covariance, jacobian),
        ))
    }
}
fn frontend_bool(result: Result<bool>) -> bool {
    match result {
        Ok(success) => success,
        Err(error) => {
            log::error!("{error}");
            false
        }
    }
}

/// BRISK2 with camera ray/Jacobian maps and a gravity-aligned extraction direction.
#[derive(Clone, Copy, Debug, Default)]
pub struct RustBriskFeatureFactory;
impl BriskFeatureFactory for RustBriskFeatureFactory {
    fn create(
        &self,
        camera: &dyn CameraBase,
        settings: &BriskSettings,
        direction: Vector3<f32>,
    ) -> Result<(Arc<dyn FeatureDetector>, Arc<dyn DescriptorExtractor>)> {
        use crate::frames::frame::{BriskDetector, BriskExtractor};
        use firefly_brisk::brisk_descriptor_extractor::{BriskDescriptorExtractor, BriskVersion};
        let maps = camera
            .camera_awareness_maps()
            .ok_or_else(|| frontend_error("camera awareness maps not initialised"))?;
        let mut intrinsics = DVector::zeros(camera.num_intrinsics());
        camera.get_intrinsics(&mut intrinsics);
        let focal = *intrinsics
            .get(0)
            .ok_or_else(|| frontend_error("camera focal length absent"))?
            as f32;
        let detector = BriskDetector::new(
            settings.threshold,
            settings.octaves,
            settings.absolute_threshold,
            settings.maximum_keypoints,
        )?;
        let mut extractor = BriskDescriptorExtractor::new(
            settings.rotation_invariance,
            settings.scale_invariance,
            BriskVersion::V2,
            1.0,
        )?;
        extractor.set_camera_properties(maps.rays.clone(), maps.image_jacobians.clone(), focal)?;
        extractor.set_extraction_direction(direction);
        Ok((
            Arc::new(detector),
            Arc::new(BriskExtractor {
                extractor: std::sync::Mutex::new(extractor),
            }),
        ))
    }
}

impl<B> LoopPoseSolver for super::frame_absolute_pose_sac_problem::OpenGvSolver<B> {
    fn refine_loop_pose(
        &self,
        frame: &MultiFrame,
        initial: &Transformation,
        residuals: &[LoopResidual],
        iterations: i32,
        threads: i32,
    ) -> Result<LoopRefinement> {
        use crate::ceres::{
            homogeneous_point_parameter_block::HomogeneousPointParameterBlock,
            parameter_block::ParameterBlock,
            pose_parameter_block::PoseParameterBlock,
            reprojection_error::ReprojectionError,
            vi_graph::{
                GraphFactor, GraphProblem, GraphSolver, ManifoldKind, Parameter, RobustLoss,
                RustGraphSolver, SolverOptions,
            },
        };
        use std::{cell::RefCell, rc::Rc};
        if iterations < 0 || threads <= 0 {
            return Err(frontend_error(
                "invalid loop-pose solver iteration/thread count",
            ));
        }
        let pose = Rc::new(RefCell::new(PoseParameterBlock::new(
            initial,
            1,
            frame.timestamp(),
        )));
        let mut problem = GraphProblem::default();
        problem.register(Parameter::Pose(pose.clone()), ManifoldKind::Pose);
        let mut terms = Vec::with_capacity(residuals.len());
        let mut extrinsics = BTreeMap::new();
        for (index, residual) in residuals.iter().enumerate() {
            let extrinsic = extrinsics
                .entry(residual.camera)
                .or_insert_with(|| {
                    let mut p = PoseParameterBlock::new(
                        &residual.extrinsics,
                        residual.camera as u64 + 2,
                        frame.timestamp(),
                    );
                    p.set_fixed(true);
                    Rc::new(RefCell::new(p))
                })
                .clone();
            let mut p = HomogeneousPointParameterBlock::new(residual.point, index as u64, true);
            p.set_fixed(true);
            let point = Rc::new(RefCell::new(p));
            let factor = Rc::new(RefCell::new(Box::new(ReprojectionError::new(
                frame.geometry(residual.camera).clone(),
                residual.camera as u64,
                residual.measurement,
                residual.information,
            ))
                as Box<dyn crate::ceres::reprojection_error_base::ReprojectionError2dBase>));
            let parameters = vec![
                Parameter::Pose(pose.clone()),
                Parameter::Point(point),
                Parameter::Pose(extrinsic),
            ];
            problem.register(parameters[1].clone(), ManifoldKind::HomogeneousPoint);
            problem.register(parameters[2].clone(), ManifoldKind::Pose);
            problem.add(
                GraphFactor::Reprojection(factor.clone()),
                parameters.clone(),
                Some(RobustLoss::Cauchy(3.0)),
            );
            terms.push((factor, parameters));
        }
        let options = SolverOptions {
            max_iterations: iterations as usize,
            num_threads: threads as usize,
            trust_region: "LEVENBERG_MARQUARDT",
            ..SolverOptions::default()
        };
        RustGraphSolver.solve(&mut problem, &options)?;
        let pose = pose
            .borrow()
            .estimate()
            .cloned()
            .ok_or_else(|| frontend_error("loop-pose estimate absent"))?;
        let mut evaluated = Vec::with_capacity(terms.len());
        for (factor, parameters) in terms {
            let values = parameters
                .iter()
                .map(Parameter::values)
                .collect::<Result<Vec<_>>>()?;
            let slices = values.iter().map(Vec::as_slice).collect::<Vec<_>>();
            let requested = [true, false, false];
            let result = factor
                .borrow()
                .evaluate_with_minimal_jacobians(&slices, Some(&requested), Some(&requested))
                .ok_or_else(|| frontend_error("loop-pose reprojection evaluation failed"))?;
            let jacobian = result.minimal_jacobians[0]
                .as_ref()
                .ok_or_else(|| frontend_error("loop-pose minimal Jacobian absent"))?;
            evaluated.push((
                Vector2::from_column_slice(result.residuals.as_slice()),
                SMatrix::<f64, 2, 6>::from_column_slice(jacobian.as_slice()),
            ));
        }
        Ok(LoopRefinement {
            pose: Transformation::from_other(&pose),
            residuals: evaluated,
        })
    }
}

struct LoopLandmarkData {
    descriptors: BTreeMap<LandmarkId, Vec<Descriptor>>,
    landmarks: BTreeMap<LandmarkId, Vector4<f64>>,
}
