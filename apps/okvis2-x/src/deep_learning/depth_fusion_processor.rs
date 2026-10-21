/* OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich. SPDX-License-Identifier: BSD-3-Clause */
//! Delayed optimized-pose MVS, sparse landmark projection and inverse-variance depth fusion.
use super::{
    deep_learning_processor::{
        CameraMeasurements, CameraWorker, DeepLearningProcessor, DisplayImages, ImageCallback,
        Images, LiveDepthCallback, ProcessingControl, UpdatedStates, camera_measurements,
        send_dropping_oldest,
    },
    stereo2_depth_processor::{
        StereoCalibration, StereoModel, StereoPrediction, disparity_to_depth,
    },
    utils::{
        Tensor, VisualizationRaster, gray_to_bgr, image_f32, image_u8, invalid,
        tensor_to_i_depth_vis, tensor_to_i_sigma_vis,
    },
};
use crate::{
    frames::frame_typedefs::MapPointVector,
    kinematics::transformation::Transformation,
    runtime::vi_interface::{State, TrackingState, Trajectory},
    sensors::measurements::{CameraData, Measurement},
};
use firefly_error::Result;
use jiff::SignedDuration;
use nalgebra::Matrix4;
use purecv::core::Matrix;
use std::{
    collections::{BTreeMap, VecDeque},
    sync::{
        Arc, Condvar, Mutex, RwLock,
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver},
    },
    thread::{self, JoinHandle},
    time::Duration,
};

pub struct MvsInput<'a> {
    pub current: &'a Matrix<u8>,
    pub sources: &'a [Matrix<u8>],
    pub current_k0: Matrix4<f32>,
    pub current_inv_k1: Matrix4<f32>,
    pub source_k1: [Matrix4<f32>; 7],
    pub current_t_wc: Matrix4<f32>,
    pub source_t_wc: Vec<Matrix4<f32>>,
    pub sparse_depth: &'a Matrix<f32>,
}
/// Multi-view inference receives current/source byte images, the three intrinsics
/// tensors, world poses and sparse half-resolution depth in the source order.
pub trait MvsModel: Send {
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn forward(&mut self, input: MvsInput<'_>) -> Result<Tensor>;
}
#[derive(Clone)]
struct SourceFrame {
    stamp: jiff::Timestamp,
    image: Matrix<u8>,
    t_wc: Transformation,
}
struct ViStates {
    tracking: TrackingState,
    updated: UpdatedStates,
    landmarks: MapPointVector,
}
struct FusionQueues {
    states: VecDeque<ViStates>,
    predictions: VecDeque<StereoPrediction>,
}
struct FusionShared {
    queues: Mutex<FusionQueues>,
    available: Condvar,
    live: RwLock<Option<LiveDepthCallback>>,
    motion_processing: AtomicBool,
}
pub struct DepthFusionProcessor {
    worker: CameraWorker,
    shared: Arc<FusionShared>,
    motion: Option<JoinHandle<()>>,
    visualizations: Arc<Mutex<Receiver<DisplayImages>>>,
}
impl DepthFusionProcessor {
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn from_parameters(
        parameters: &crate::config::parameters::ViParameters,
        stereo: Box<dyn StereoModel>,
        mvs: Box<dyn MvsModel>,
        raster: Arc<dyn VisualizationRaster>,
    ) -> Result<Self> {
        let system = &parameters.n_camera_system;
        let id_color = (0..system.num_cameras()).rfind(|&i| system.camera_type(i).is_colour);
        Self::new(
            StereoCalibration::from_parameters(parameters)?,
            id_color,
            parameters.camera.image_delay,
            stereo,
            mvs,
            raster,
        )
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    ///
    /// # Panics
    /// Panics if a shared model/queue/configuration mutex has been poisoned.
    pub fn new(
        mut calibration: StereoCalibration,
        id_color: Option<usize>,
        image_delay: f64,
        mut stereo: Box<dyn StereoModel>,
        mut mvs: Box<dyn MvsModel>,
        raster: Arc<dyn VisualizationRaster>,
    ) -> Result<Self> {
        // The fusion constructor explicitly casts the focal length through float.
        calibration.focal_length = f64::from(calibration.intrinsics[0] as f32);
        Self::warm_models(&calibration, stereo.as_mut(), mvs.as_mut())?;
        let control = ProcessingControl::new();
        let shared = Arc::new(FusionShared {
            queues: Mutex::new(FusionQueues {
                states: VecDeque::new(),
                predictions: VecDeque::new(),
            }),
            available: Condvar::new(),
            live: RwLock::new(None),
            motion_processing: AtomicBool::new(false),
        });
        let c = control.clone();
        let s = shared.clone();
        let cal = calibration.clone();
        let worker = CameraWorker::start(control.clone(), move |mut frames| {
            let (left, right) = cal.rectify(&mut frames)?;
            let output = stereo.forward(&left, &right)?;
            if output.shape.len() != 3 || output.shape[0] != 2 {
                return Err(invalid(
                    "fusion stereo model requires disparity and sigma planes",
                    "DepthFusionProcessor::stereo",
                ));
            }
            let fb = (cal.focal_length * cal.baseline) as f32;
            let (depth, sigma, _, _) = disparity_to_depth(&output, f64::from(fb), Some(0.01))?;
            let stamp = frames[&cal.id_left][0].time_stamp;
            let prediction = StereoPrediction {
                time_stamp: stamp,
                left,
                right,
                rgb: id_color.and_then(|id| {
                    frames
                        .get(&id)
                        .and_then(|v| v.first())
                        .and_then(|m| m.measurement.image.clone())
                }),
                depth: depth.clone(),
                sigma: sigma.clone(),
            };
            let mut queues = s.queues.lock().unwrap();
            if c.blocking()? {
                while queues.predictions.len() >= 10 && !c.shutdown.load(Ordering::Acquire) {
                    queues = s
                        .available
                        .wait_timeout(queues, Duration::from_millis(10))
                        .unwrap()
                        .0;
                }
            } else if queues.predictions.len() >= 10 {
                queues.predictions.pop_front();
            }
            if c.shutdown.load(Ordering::Acquire) {
                return Ok(());
            }
            queues.predictions.push_back(prediction);
            drop(queues);
            s.available.notify_all();
            if let Some(callback) = s.live.read().unwrap().as_ref() {
                callback(
                    stamp,
                    &image_f32(depth.matrix()?),
                    Some(&*image_f32(sigma.matrix()?)),
                )?;
            }
            Ok(())
        });
        let (vis_sender, vis_receiver) = mpsc::sync_channel(1);
        let vis_receiver = Arc::new(Mutex::new(vis_receiver));
        let vis = vis_receiver.clone();
        let s = shared.clone();
        let c = control;
        let motion = thread::spawn(move || {
            Self::run_motion_worker(
                (calibration, id_color, image_delay),
                (mvs, raster),
                (s, c),
                (vis_sender, vis),
            );
        });
        Ok(Self {
            worker,
            shared,
            motion: Some(motion),
            visualizations: vis_receiver,
        })
    }
    fn warm_models(
        calibration: &StereoCalibration,
        stereo: &mut dyn StereoModel,
        mvs: &mut dyn MvsModel,
    ) -> Result<()> {
        let warm = Matrix::new(calibration.height, calibration.width, 1);
        for _ in 0..10 {
            stereo.forward(&warm, &warm)?;
        }
        let warm_sources = vec![warm.clone(); 7];
        let sparse = Matrix::from_vec(
            calibration.height / 2,
            calibration.width / 2,
            1,
            vec![1.; calibration.height / 2 * (calibration.width / 2)],
        );
        for _ in 0..10 {
            mvs.forward(MvsInput {
                current: &warm,
                sources: &warm_sources,
                current_k0: Matrix4::identity(),
                current_inv_k1: Matrix4::identity(),
                source_k1: [Matrix4::identity(); 7],
                current_t_wc: Matrix4::identity(),
                source_t_wc: vec![Matrix4::identity(); 7],
                sparse_depth: &sparse,
            })?;
        }
        Ok(())
    }

    fn run_motion_worker(
        (calibration, id_color, image_delay): (StereoCalibration, Option<usize>, f64),
        (mvs, raster): (Box<dyn MvsModel>, Arc<dyn VisualizationRaster>),
        (s, c): (Arc<FusionShared>, Arc<ProcessingControl>),
        (vis_sender, vis): (
            mpsc::SyncSender<DisplayImages>,
            Arc<Mutex<mpsc::Receiver<DisplayImages>>>,
        ),
    ) {
        let mut processor = MotionProcessor {
            calibration,
            id_color,
            image_delay: SignedDuration::from_secs_f64(image_delay),
            sources: VecDeque::new(),
            source_select_count: 0,
            trajectory: Trajectory::default(),
            landmarks: BTreeMap::new(),
            mvs,
            raster,
        };
        while !c.shutdown.load(Ordering::Acquire) {
            let work = (|| -> Result<Option<StereoPrediction>> {
                let mut queues = s.queues.lock().unwrap();
                while let Some(update) = queues.states.pop_front() {
                    processor.update_trajectory(update)?;
                }
                s.available.notify_all();
                let latest = processor.trajectory.state_ids().iter().next_back().copied();
                let ready =
                    if let (Some(prediction), Some(id)) = (queues.predictions.front(), latest) {
                        processor
                            .trajectory
                            .state_by_id(id)
                            .is_some_and(|state| prediction.time_stamp <= state.timestamp)
                    } else {
                        false
                    };
                if ready {
                    s.motion_processing.store(true, Ordering::Release);
                    let result = queues.predictions.pop_front();
                    s.available.notify_all();
                    Ok(result)
                } else {
                    Ok(None)
                }
            })();
            let result = match work {
                Ok(Some(prediction)) => processor
                    .process(prediction, &c)
                    .and_then(|images| send_dropping_oldest(&vis_sender, &vis, images).map(|_| ())),
                Ok(None) => {
                    thread::sleep(Duration::from_millis(1));
                    Ok(())
                }
                Err(error) => Err(error),
            };
            s.motion_processing.store(false, Ordering::Release);
            if let Err(error) = result {
                *c.error.lock().unwrap() = Some(error);
                c.shutdown.store(true, Ordering::Release);
                s.available.notify_all();
            }
        }
    }

    pub fn set_finished(&self) {
        self.worker.control.shutdown.store(true, Ordering::Release);
        self.shared.available.notify_all();
    }
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.worker.control.shutdown.load(Ordering::Acquire)
    }
}
impl Drop for DepthFusionProcessor {
    fn drop(&mut self) {
        self.set_finished();
        if let Some(thread) = self.motion.take()
            && thread.join().is_err()
        {
            *self.worker.control.error.lock().unwrap() = Some(invalid(
                "motion processing thread panicked",
                "DepthFusionProcessor::drop",
            ));
        }
    }
}
impl DeepLearningProcessor for DepthFusionProcessor {
    fn set_blocking(&self, value: bool) {
        *self.worker.control.blocking.write().unwrap() = Some(value);
    }
    fn set_image_callback(&self, callback: ImageCallback) {
        *self.worker.control.callback.write().unwrap() = Some(callback);
    }
    fn set_live_depth_image_callback(&self, callback: LiveDepthCallback) {
        *self.shared.live.write().unwrap() = Some(callback);
    }
    fn is_depth_fusion(&self) -> bool {
        true
    }
    fn add_images(&self, images: &Images, _depths: &Images) -> Result<bool> {
        self.worker.submit(camera_measurements(images, None)?)
    }
    fn state_update_callback(
        &self,
        _latest: &State,
        tracking: &TrackingState,
        updated: UpdatedStates,
        landmarks: Arc<MapPointVector>,
    ) -> Result<bool> {
        self.worker.control.check_error()?;
        let blocking = self.worker.control.blocking()?;
        let mut queues = self.shared.queues.lock().unwrap();
        let max = if blocking { 4 } else { 10 };
        while blocking && queues.states.len() >= max && !self.is_finished() {
            queues = self
                .shared
                .available
                .wait_timeout(queues, Duration::from_millis(10))
                .unwrap()
                .0;
        }
        if self.is_finished() {
            return Ok(false);
        }
        let dropped = queues.states.len() >= max;
        if dropped {
            queues.states.pop_front();
        }
        queues.states.push_back(ViStates {
            tracking: tracking.clone(),
            updated,
            landmarks: landmarks
                .iter()
                .filter(|lm| lm.state_id == tracking.id.value())
                .cloned()
                .collect(),
        });
        self.shared.available.notify_all();
        Ok(!dropped)
    }
    fn display(&self, images: &mut DisplayImages) -> Result<()> {
        self.worker.control.check_error()?;
        if let Ok(vis) = self.visualizations.lock().unwrap().try_recv() {
            images.extend(vis);
        }
        Ok(())
    }
    fn finished_processing(&self) -> Result<bool> {
        self.worker.control.check_error()?;
        let mut queues = self.shared.queues.lock().unwrap();
        let finished = self.worker.control.pending.load(Ordering::Acquire) + queues.states.len()
            == 0
            && !self.worker.control.is_processing.load(Ordering::Acquire)
            && !self.shared.motion_processing.load(Ordering::Acquire);
        if finished {
            queues.predictions.clear();
            self.shared.available.notify_all();
        }
        Ok(finished)
    }
}
type FusionResult = (Tensor, Tensor, Tensor, Tensor);
struct MotionProcessor {
    calibration: StereoCalibration,
    id_color: Option<usize>,
    image_delay: SignedDuration,
    sources: VecDeque<SourceFrame>,
    source_select_count: u32,
    trajectory: Trajectory,
    landmarks: BTreeMap<jiff::Timestamp, MapPointVector>,
    mvs: Box<dyn MvsModel>,
    raster: Arc<dyn VisualizationRaster>,
}
impl MotionProcessor {
    fn update_trajectory(&mut self, states: ViStates) -> Result<()> {
        self.trajectory
            .update(
                &states.tracking,
                &states.updated,
                &mut std::collections::BTreeSet::new(),
            )
            .map_err(integration_error)?;
        let last = states.updated.last_key_value().ok_or_else(|| {
            invalid(
                "updated state map is empty",
                "DepthFusionProcessor::update_trajectory",
            )
        })?;
        if states.tracking.id == *last.0 {
            self.landmarks.insert(last.1.timestamp, states.landmarks);
        }
        Ok(())
    }
    fn is_source_frame(&mut self, transform: &Transformation) -> bool {
        let baseline = transform.translation().norm() as f32;
        let angle = (0.5 * (transform.rotation_matrix().trace() - 1.))
            .acos()
            .abs() as f32;
        let selected = if self.source_select_count.is_multiple_of(2) {
            f64::from(baseline) > 0.03 || f64::from(angle) > 0.05
        } else {
            f64::from(baseline) > 0.10 || f64::from(angle) > 0.35
        };
        if selected {
            self.source_select_count = self.source_select_count.wrapping_add(1);
        }
        selected
    }
    fn intrinsics(&self) -> (Matrix4<f32>, Matrix4<f32>, [Matrix4<f32>; 7]) {
        let [fu, fv, cu, cv] = self.calibration.intrinsics;
        let mut k0 = Matrix4::identity();
        let mut inv = Matrix4::identity();
        let mut k1 = Matrix4::identity();
        k0[(0, 0)] = (0.5 * fu) as f32;
        k0[(1, 1)] = (0.5 * fv) as f32;
        k0[(0, 2)] = (0.5 * cu) as f32;
        k0[(1, 2)] = (0.5 * cv) as f32;
        inv[(0, 0)] = (1. / (0.25 * fu)) as f32;
        inv[(1, 1)] = (1. / (0.25 * fv)) as f32;
        inv[(0, 2)] = (-cu / fu) as f32;
        inv[(1, 2)] = (-cv / fv) as f32;
        k1[(0, 0)] = (0.25 * fu) as f32;
        k1[(1, 1)] = (0.25 * fv) as f32;
        k1[(0, 2)] = (0.25 * cu) as f32;
        k1[(1, 2)] = (0.25 * cv) as f32;
        (k0, inv, [k1; 7])
    }
    fn process(
        &mut self,
        mut prediction: StereoPrediction,
        control: &ProcessingControl,
    ) -> Result<DisplayImages> {
        prediction.time_stamp = prediction
            .time_stamp
            .checked_sub(self.image_delay)
            .map_err(|e| invalid(e.to_string(), "DepthFusionProcessor::process"))?;
        let mut sparse_vis = gray_to_bgr(&prediction.left)?;
        let mut mvs_result = None;
        if let Some(state) = self
            .trajectory
            .get_state(prediction.time_stamp)
            .map_err(integration_error)?
        {
            let live = SourceFrame {
                stamp: prediction.time_stamp,
                image: prediction.left.clone(),
                t_wc: state.t_ws.compose(&self.calibration.t_sc),
            };
            if self.sources.len() < 7 {
                let select = self
                    .sources
                    .back()
                    .map(|last| last.t_wc.inverse().compose(&live.t_wc));
                if select.as_ref().is_none_or(|t| self.is_source_frame(t)) {
                    self.sources.push_back(live);
                }
            } else {
                for source in &mut self.sources {
                    if let Some(state) = self
                        .trajectory
                        .get_state(source.stamp)
                        .map_err(integration_error)?
                    {
                        source.t_wc = state.t_ws.compose(&self.calibration.t_sc);
                    }
                }
                let sparse = self.project_sparse_depth(&live, &mut sparse_vis)?;
                mvs_result = Some(self.fuse_prediction(&prediction, &live, &sparse)?);
                let transform = self
                    .sources
                    .back()
                    .unwrap()
                    .t_wc
                    .inverse()
                    .compose(&live.t_wc);
                if self.is_source_frame(&transform) {
                    self.sources.push_back(live);
                    self.sources.pop_front();
                }
            }
        }
        let (depth, sigma) = mvs_result
            .as_ref()
            .map_or((&prediction.depth, &prediction.sigma), |(_, _, d, s)| {
                (d, s)
            });
        let stamp = prediction
            .time_stamp
            .checked_add(self.image_delay)
            .map_err(|e| invalid(e.to_string(), "DepthFusionProcessor::process"))?;
        let mut frames = CameraMeasurements::new();
        frames.insert(
            self.calibration.id_left,
            vec![Measurement::new(
                stamp,
                CameraData {
                    depth_image: Some(image_f32(depth.matrix()?)),
                    sigma_image: Some(image_f32(sigma.matrix()?)),
                    ..CameraData::default()
                },
                self.calibration.id_left as i32,
            )],
        );
        if let (Some(id), Some(rgb)) = (
            self.id_color,
            prediction
                .rgb
                .take()
                .filter(|image| image.rows() > 0 && image.cols() > 0),
        ) {
            frames.insert(
                id,
                vec![Measurement::new(
                    stamp,
                    CameraData {
                        image: Some(rgb),
                        ..CameraData::default()
                    },
                    id as i32,
                )],
            );
        }
        control.invoke(&mut frames)?;
        self.prediction_images(prediction, sparse_vis, mvs_result)
    }
    fn fuse_prediction(
        &mut self,
        prediction: &StereoPrediction,
        live: &SourceFrame,
        sparse: &Matrix<f32>,
    ) -> Result<FusionResult> {
        let sources = self
            .sources
            .iter()
            .map(|s| s.image.clone())
            .collect::<Vec<_>>();
        let poses = self
            .sources
            .iter()
            .map(|s| s.t_wc.matrix().cast::<f32>())
            .collect();
        let (k0, inv, k1) = self.intrinsics();
        let output = self.mvs.forward(MvsInput {
            current: &prediction.left,
            sources: &sources,
            current_k0: k0,
            current_inv_k1: inv,
            source_k1: k1,
            current_t_wc: live.t_wc.matrix().cast(),
            source_t_wc: poses,
            sparse_depth: sparse,
        })?;
        let depth = output.plane(0)?;
        let sigma = output.plane(1)?.map(|x| {
            let s = 4. * x;
            if s.is_finite() { s } else { 100. }
        });
        let (fused_depth, fused_sigma) =
            fuse_depth(&prediction.depth, &prediction.sigma, &depth, &sigma)?;
        Ok((depth, sigma, fused_depth, fused_sigma))
    }

    fn prediction_images(
        &self,
        prediction: StereoPrediction,
        sparse_vis: Matrix<u8>,
        mvs_result: Option<FusionResult>,
    ) -> Result<DisplayImages> {
        let stereo_depth = image_u8(tensor_to_i_depth_vis(
            &prediction.depth,
            self.raster.as_ref(),
        )?);
        let stereo_sigma = image_u8(tensor_to_i_sigma_vis(
            &prediction.depth,
            &prediction.sigma,
            self.raster.as_ref(),
        )?);
        let mut images = BTreeMap::from([
            ("leftImage".into(), image_u8(prediction.left)),
            ("rightImage".into(), image_u8(prediction.right)),
            ("stereoDepth".into(), stereo_depth.clone()),
            ("stereoSigma".into(), stereo_sigma.clone()),
            ("sparseDepth".into(), image_u8(sparse_vis)),
        ]);
        if self.sources.len() == 7 {
            for (i, source) in self.sources.iter().enumerate() {
                images.insert(format!("srcImage{i}"), image_u8(source.image.clone()));
            }
        }
        if let Some((d, s, fd, fs)) = mvs_result {
            images.insert(
                "mvsDepth".into(),
                image_u8(tensor_to_i_depth_vis(&d, self.raster.as_ref())?),
            );
            images.insert(
                "mvsSigma".into(),
                image_u8(tensor_to_i_sigma_vis(&d, &s, self.raster.as_ref())?),
            );
            images.insert(
                "fuseDepth".into(),
                image_u8(tensor_to_i_depth_vis(&fd, self.raster.as_ref())?),
            );
            images.insert(
                "fuseSigma".into(),
                image_u8(tensor_to_i_sigma_vis(&fd, &fs, self.raster.as_ref())?),
            );
        } else {
            images.insert(
                "mvsDepth".into(),
                image_u8(Matrix::new(
                    self.calibration.height,
                    self.calibration.width,
                    3,
                )),
            );
            images.insert(
                "mvsSigma".into(),
                image_u8(Matrix::new(
                    self.calibration.height,
                    self.calibration.width,
                    3,
                )),
            );
            images.insert("fuseDepth".into(), stereo_depth);
            images.insert("fuseSigma".into(), stereo_sigma);
        }
        Ok(images)
    }

    fn project_sparse_depth(
        &mut self,
        live: &SourceFrame,
        sparse_vis: &mut Matrix<u8>,
    ) -> Result<Matrix<f32>> {
        let (half_h, half_w) = (self.calibration.height / 2, self.calibration.width / 2);
        let mut sparse = Matrix::new(half_h, half_w, 1);
        let t_cw = live.t_wc.inverse();
        let half_f = 0.5 * (self.calibration.focal_length as f32);
        let horizontal_center_sum = f64::from(self.calibration.intrinsics[2] as f32) + 0.5;
        let half_cu = (0.5_f64 * horizontal_center_sum - 0.5) as f32;
        let vertical_center_sum = f64::from(self.calibration.intrinsics[3] as f32) + 0.5;
        let vertical_center = (0.5_f64 * vertical_center_sum - 0.5) as f32;
        for lm in self.landmarks.remove(&live.stamp).unwrap_or_default() {
            let p = t_cw.transform_homogeneous(&lm.point);
            let z = p[2] as f32;
            if f64::from(z) <= 0.4 || z > 10. {
                continue;
            }
            let u = (f64::from(half_f) * (p[0] / f64::from(z)) + f64::from(half_cu)) as i32;
            let v = (f64::from(half_f) * (p[1] / f64::from(z)) + f64::from(vertical_center)) as i32;
            if u > 0 && u < half_w as i32 - 1 && v > 0 && v < half_h as i32 - 1 {
                sparse.data[v as usize * half_w + u as usize] = z;
                self.raster.circle(
                    sparse_vis,
                    [(2 * u) as f32, (2 * v) as f32],
                    3,
                    [0, (f64::from(z / 10.) * 255.) as u8, 0],
                    -1,
                    false,
                )?;
            }
        }
        Ok(sparse)
    }
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn fuse_depth(
    a: &Tensor,
    sigma_a: &Tensor,
    b: &Tensor,
    sigma_b: &Tensor,
) -> Result<(Tensor, Tensor)> {
    if a.shape != b.shape || a.shape != sigma_a.shape || a.shape != sigma_b.shape {
        return Err(invalid("fusion depth/sigma shape mismatch", "fuse_depth"));
    }
    let mut depth = Vec::with_capacity(a.data.len());
    let mut sigma = Vec::with_capacity(a.data.len());
    for i in 0..a.data.len() {
        let ia = 1. / sigma_a.data[i].powi(2);
        let ib = 1. / sigma_b.data[i].powi(2);
        let variance = 1. / (ia + ib);
        depth.push(variance * (ia * a.data[i] + ib * b.data[i]));
        sigma.push(variance.sqrt());
    }
    Ok((
        Tensor::new(a.shape.clone(), depth)?,
        Tensor::new(a.shape.clone(), sigma)?,
    ))
}

fn integration_error(error: crate::ceres::imu_error::IntegrationError) -> firefly_error::Error {
    invalid(format!("{error:?}"), "DepthFusionProcessor::trajectory")
}
