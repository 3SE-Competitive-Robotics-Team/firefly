//! Online IPC adapter for the simulated camera and IMU topics.
mod imu;
use firefly_basalt::{
    io::dataset_io::ImageData,
    optical_flow::{OpticalFlowBase, OpticalFlowFactory, OpticalFlowInput},
    utils::vio_config::VioConfig,
    vi_estimator::{
        local_map::LocalMapSnapshot,
        vio_estimator::{VioEstimatorBase, VioEstimatorFactory, VioFrameDiagnostics},
    },
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::image::image::ManagedImage;
use firefly_error::{Error, ErrorKind, Result};
use firefly_imu_initialization::GravityEstimate;
use firefly_pubsub::{
    camera::{CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, GrayImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH},
    node::IpcNode,
    node::create_node,
    subscriber::{Subscriber, SubscriberOptions},
    viz::{VizMessage, VizPublisher, kind},
};
use firefly_rootslam_mapping::{loop_closure::LoopOptions, map::MapPolicy};
use firefly_vio_runtime::{ImuWatermark, publish_pose_diagnostic, publish_scalar_diagnostic};
use nalgebra::{Isometry3, Translation3, Vector3};
use std::{
    collections::BTreeMap,
    path::PathBuf,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    time::Duration,
};

#[derive(serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
#[expect(
    clippy::struct_excessive_bools,
    reason = "Independent precision, mapping, loop and output-ownership options"
)]
pub struct Options {
    pub vio_config: Option<PathBuf>,
    /// Detection cell width in pixels; overrides the Basalt archive when present.
    pub optical_flow_detection_grid_size: Option<i32>,
    pub stats_directory: Option<PathBuf>,
    pub use_double: bool,
    pub map: bool,
    /// 回环检测开关；关闭后只保留局部建图与关键帧共视图。
    pub loops: bool,
    /// Let localization exclusively publish corrected odometry and navigation.
    pub external_localization: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            vio_config: None,
            optical_flow_detection_grid_size: None,
            stats_directory: None,
            use_double: false,
            map: true,
            loops: true,
            external_localization: false,
        }
    }
}

impl Options {
    /// # Errors
    /// Rejects detection cells that cannot fit the calibrated image.
    pub fn validate(&self) -> Result<()> {
        if self.optical_flow_detection_grid_size.is_some_and(|size| {
            size <= 0
                || usize::try_from(size).map_or(true, |size| size > IMAGE_WIDTH.min(IMAGE_HEIGHT))
        }) {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "optical_flow_detection_grid_size must fit the calibrated image",
            ));
        }
        Ok(())
    }
}

fn error(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::Internal, message).with_operation("rootslam.online")
}

struct StopOnDrop<'a>(&'a AtomicBool);
impl Drop for StopOnDrop<'_> {
    fn drop(&mut self) {
        self.0.store(true, Ordering::Release);
    }
}

#[derive(Default)]
struct Pairing {
    left: BTreeMap<i64, Arc<ManagedImage<u16>>>,
    right: BTreeMap<i64, Arc<ManagedImage<u16>>>,
}

impl Pairing {
    fn insert(&mut self, packet: &GrayImageMessage, camera: usize) -> Result<()> {
        if packet.width as usize != IMAGE_WIDTH
            || packet.height as usize != IMAGE_HEIGHT
            || packet.sensor_id != camera as i32
            || !packet.timestamp.is_finite()
            || packet.timestamp < 0.0
        {
            return Err(error(
                "Camera message does not match the calibrated stereo stream",
            ));
        }
        let stamp = (packet.timestamp * 1e9).round() as i64;
        let mut image = ManagedImage::new(IMAGE_WIDTH, IMAGE_HEIGHT);
        for (destination, source) in image.data_mut().iter_mut().zip(packet.data) {
            *destination = u16::from(source) << 8;
        }
        let queue = if camera == 0 {
            &mut self.left
        } else {
            &mut self.right
        };
        queue.insert(stamp, Arc::new(image));
        while queue.len() > 8 {
            queue.pop_first();
        }
        Ok(())
    }

    fn next(&mut self, watermark: i64) -> Option<(i64, Vec<ImageData>)> {
        loop {
            let (&left_time, _) = self.left.first_key_value()?;
            let (&right_time, _) = self.right.first_key_value()?;
            if (left_time - right_time).unsigned_abs() > 1_000_000 {
                if left_time < right_time {
                    self.left.pop_first();
                } else {
                    self.right.pop_first();
                }
                continue;
            }
            let time = left_time.max(right_time);
            if watermark < time {
                return None;
            }
            let left = self.left.pop_first()?.1;
            let right = self.right.pop_first()?.1;
            return Some((
                time,
                vec![
                    ImageData {
                        img: Some(left),
                        exposure: 0.,
                    },
                    ImageData {
                        img: Some(right),
                        exposure: 0.,
                    },
                ],
            ));
        }
    }

    fn discard_before(&mut self, timestamp: i64) {
        while self
            .left
            .first_key_value()
            .is_some_and(|(&time, _)| time < timestamp)
        {
            self.left.pop_first();
        }
        while self
            .right
            .first_key_value()
            .is_some_and(|(&time, _)| time < timestamp)
        {
            self.right.pop_first();
        }
    }
}

/// Reads Render and Sim IPC, runs Basalt VIO and publishes FC odometry topics.
/// # Errors
/// Returns input, estimator, mapping or IPC failures.
pub fn run(options: &Options, stop: &AtomicBool) -> Result<()> {
    LiveSession::new(options)?.run(stop)
}

struct LiveSession {
    node: IpcNode,
    left: Subscriber<GrayImageMessage>,
    right: Subscriber<GrayImageMessage>,
    viz: VizPublisher,
    estimator: VioEstimatorBase,
    flow: OpticalFlowBase,
    map_worker: Option<crate::system::MappingWorker>,
    map_input: Option<std::sync::mpsc::Sender<Option<Arc<LocalMapSnapshot>>>>,
    imu_inputs: Option<imu::Inputs>,
    gravity_rx: mpsc::Receiver<GravityEstimate>,
    flow_tx: std::sync::mpsc::SyncSender<Option<Arc<OpticalFlowInput>>>,
    diagnostics_rx: mpsc::Receiver<VioFrameDiagnostics>,
    stats_directory: Option<PathBuf>,
    pairing: Pairing,
    imu_watermark: ImuWatermark,
    gravity_estimate: Option<GravityEstimate>,
    estimator_initialized: bool,
    frame_started: imu::FrameStarts,
}

impl LiveSession {
    fn new(options: &Options) -> Result<Self> {
        let calibration = render_calibration();
        let vio = initial_vio_config(options)?;
        let node = create_node()?;
        let image_options = SubscriberOptions {
            buffer_size: 2,
            ..Default::default()
        };
        let left =
            Subscriber::<GrayImageMessage>::with_options(&node, CAMERA_LEFT_TOPIC, image_options)?;
        let right =
            Subscriber::<GrayImageMessage>::with_options(&node, CAMERA_RIGHT_TOPIC, image_options)?;
        let viz = VizPublisher::new(&node)?;
        let mut estimator = VioEstimatorFactory::get_vio_estimator(
            &vio,
            &calibration,
            &Vector3::new(0., 0., -9.81),
            true,
            options.use_double,
        )?;
        let (states_tx, states_rx) = mpsc::sync_channel(8);
        estimator.outputs.state_bounded = Some(states_tx);
        let (diagnostics_tx, diagnostics_rx) = mpsc::channel();
        estimator.outputs.diagnostics = Some(diagnostics_tx);
        let (map_worker, map_input) = if options.map {
            let worker = crate::system::MappingWorker::spawn(
                calibration.clone(),
                crate::system::SystemOptions {
                    map: MapPolicy::paper_thresholds(),
                    loops: LoopOptions::default(),
                    global_optimization: false,
                    loop_closures: options.loops,
                    ..Default::default()
                },
            )?;
            let input = worker.input();
            estimator.outputs.local_map = Some(input.clone());
            (Some(worker), Some(input))
        } else {
            (None, None)
        };
        let flow = OpticalFlowFactory::get_optical_flow_with_output(
            &vio,
            &calibration,
            estimator.vision_data_queue().clone(),
        )?;
        let imu_tx = estimator.imu_data_queue().clone();
        let flow_tx = flow.input_queue().clone();
        let frame_started = Arc::new(Mutex::new(BTreeMap::new()));
        let imu_watermark = ImuWatermark::default();
        let (gravity_tx, gravity_rx) = mpsc::sync_channel(1);
        let imu_inputs = Some(imu::Inputs {
            external_localization: options.external_localization,
            imu: imu_tx,
            posteriors: states_rx,
            gravity: gravity_tx,
            frame_started: frame_started.clone(),
            watermark: imu_watermark.clone(),
        });
        Ok(Self {
            node,
            left,
            right,
            viz,
            estimator,
            flow,
            map_worker,
            map_input,
            imu_inputs,
            gravity_rx,
            flow_tx,
            diagnostics_rx,
            stats_directory: options.stats_directory.clone(),
            pairing: Pairing::default(),
            imu_watermark,
            gravity_estimate: None,
            estimator_initialized: false,
            frame_started,
        })
    }

    fn run(mut self, stop: &AtomicBool) -> Result<()> {
        let inputs = self
            .imu_inputs
            .take()
            .ok_or_else(|| error("IMU worker already started"))?;
        let process_result = std::thread::scope(|scope| {
            let worker = std::thread::Builder::new()
                .name("rootslam-imu".into())
                .spawn_scoped(scope, move || imu::run(inputs, stop))
                .map_err(|e| error("Cannot start rootslam-imu").with_source(e))?;
            let process = {
                let _stopping = StopOnDrop(stop);
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.process(stop)))
                    .map_err(|_| error("RootSLAM image ingress panicked"))
                    .and_then(|result| result)
            };
            let transport = worker
                .join()
                .map_err(|_| error("rootslam-imu panicked"))
                .and_then(|r| r);
            process.and(transport)
        });
        let shutdown_result = self.shutdown();
        process_result.and(shutdown_result)
    }

    fn process(&mut self, stop: &AtomicBool) -> Result<()> {
        while !stop.load(Ordering::Acquire) && self.node.wait(Duration::from_millis(1)).is_ok() {
            if self.estimator.status.finished.load(Ordering::Acquire) {
                return Err(error("VIO estimator stopped"));
            }
            if let Ok(estimate) = self.gravity_rx.try_recv() {
                self.gravity_estimate = Some(estimate);
            }
            self.receive_images()?;
            self.enqueue_stereo()?;
            self.publish_diagnostics();
        }
        Ok(())
    }

    fn receive_images(&mut self) -> Result<()> {
        for _ in 0..4 {
            let Some(packet) = self.left.receive()? else {
                break;
            };
            self.pairing.insert(&packet, 0)?;
        }
        for _ in 0..4 {
            let Some(packet) = self.right.receive()? else {
                break;
            };
            self.pairing.insert(&packet, 1)?;
        }
        Ok(())
    }

    fn enqueue_stereo(&mut self) -> Result<()> {
        while let Some((t_ns, img_data)) = self.gravity_estimate.and_then(|estimate| {
            self.pairing
                .discard_before(i64::try_from(estimate.end_ns).ok()?);
            self.pairing
                .next(i64::try_from(self.imu_watermark.load()?.as_nanosecond()).ok()?)
        }) {
            if !self.estimator_initialized {
                let estimate = self
                    .gravity_estimate
                    .ok_or_else(|| error("stationary gravity estimate is absent"))?;
                let rotation = estimate
                    .rotation_body_to_world()
                    .ok_or_else(|| error("stationary gravity estimate is invalid"))?;
                let pose = Isometry3::from_parts(Translation3::identity(), rotation);
                self.estimator.initialize_state(
                    t_ns,
                    pose,
                    Vector3::zeros(),
                    Vector3::zeros(),
                    Vector3::zeros(),
                )?;
                self.estimator_initialized = true;
            }
            let mut frame_started = self
                .frame_started
                .lock()
                .map_err(|_| error("frame timing mutex poisoned"))?;
            let enqueue_begin = firefly_basalt::utils::time_utils::profile_clock_ns();
            match self
                .flow_tx
                .try_send(Some(Arc::new(OpticalFlowInput { t_ns, img_data })))
            {
                Ok(()) => {
                    frame_started.insert(t_ns, std::time::Instant::now());
                    firefly_basalt::utils::time_utils::profile_stage(
                        t_ns,
                        "enqueue",
                        enqueue_begin,
                    );
                }
                Err(mpsc::TrySendError::Full(_)) => {
                    log::warn!("RootSLAM live dropped stereo frame: optical flow queue full");
                }
                Err(mpsc::TrySendError::Disconnected(_)) => {
                    return Err(error("Optical flow input closed"));
                }
            }
        }
        Ok(())
    }

    fn publish_diagnostics(&mut self) {
        while let Ok(data) = self.diagnostics_rx.try_recv() {
            let stamp = data.t_ns as f64 * 1e-9;
            for (name, values) in [
                ("vio/diagnostics/tracks", data.tracks),
                ("vio/diagnostics/graph", data.graph),
                ("vio/diagnostics/keyframes", data.keyframes),
                ("vio/diagnostics/residual_before_px", data.residual_before),
                ("vio/diagnostics/residual_after_px", data.residual_after),
            ] {
                publish_scalar_diagnostic(&self.viz, stamp, name, values);
            }
            publish_scalar_diagnostic(
                &self.viz,
                stamp,
                "vio/diagnostics/stereo_ratio",
                [if data.tracks[0] > 0. {
                    data.tracks[2] / data.tracks[0]
                } else {
                    0.
                }],
            );
            publish_scalar_diagnostic(
                &self.viz,
                stamp,
                "vio/diagnostics/residual_rms_px",
                [data.residual_before[1], data.residual_after[1]],
            );
            for (name, values) in [
                ("vio/diagnostics/position_delta_m", data.position_delta),
                ("vio/diagnostics/velocity_delta_m_s", data.velocity_delta),
                ("vio/diagnostics/rotation_delta_rad", data.rotation_delta),
            ] {
                publish_scalar_diagnostic(&self.viz, stamp, name, values);
            }
            if let Some(flow) = data.flow {
                for (camera, points) in flow.observations.iter().enumerate().take(2) {
                    let name = firefly_pubsub::viz::VIO_KEYPOINT_ENTITIES[camera];
                    let mut message = VizMessage::base(kind::KEYPOINTS_2D, stamp, name);
                    message.color = [64, 255, 128];
                    message.scalars[..3].copy_from_slice(&[
                        IMAGE_WIDTH as f64,
                        IMAGE_HEIGHT as f64,
                        points.len() as f64,
                    ]);
                    message.scalar_count = 3;
                    for (index, (&id, point)) in
                        points.iter().take(message.points.len()).enumerate()
                    {
                        message.points[index] = [
                            f64::from(point.translation.x),
                            f64::from(point.translation.y),
                            id as f64,
                        ];
                        message.point_count += 1;
                    }
                    let _ = self.viz.publish(message);
                    if camera == 0
                        && log::log_enabled!(target: "vio-diagnostics", log::Level::Debug)
                    {
                        let mut active = VizMessage::base(
                            kind::KEYPOINTS_2D,
                            stamp,
                            "vio/diagnostics/keypoints/active_left",
                        );
                        active.color = [255, 128, 64];
                        active.scalars[..3].copy_from_slice(&[
                            IMAGE_WIDTH as f64,
                            IMAGE_HEIGHT as f64,
                            data.active_ids.len() as f64,
                        ]);
                        active.scalar_count = 3;
                        for (index, id) in
                            data.active_ids.iter().take(active.points.len()).enumerate()
                        {
                            let point = &points[id];
                            active.points[index] = [
                                f64::from(point.translation.x),
                                f64::from(point.translation.y),
                                *id as f64,
                            ];
                            active.point_count += 1;
                        }
                        let _ = self.viz.publish(active);
                    }
                }
            }
        }
    }

    fn shutdown(self) -> Result<()> {
        let Self {
            map_worker,
            map_input,
            imu_inputs,
            flow_tx,
            flow,
            mut estimator,
            stats_directory,
            ..
        } = self;
        drop(map_input);
        drop(imu_inputs);
        drop(flow_tx);
        estimator.close_inputs();
        let flow_result = flow.shutdown();
        let estimator_result = estimator.maybe_join();
        if let Some(directory) = &stats_directory {
            std::fs::create_dir_all(directory).map_err(|failure| {
                error(format!("Cannot create VIO statistics directory: {failure}"))
            })?;
            estimator.debug_finalize(directory)?;
        }
        let shutdown_result = estimator.shutdown();
        flow_result.map_err(|failure| {
            log::error!("RootSLAM optical-flow worker stopped with an error: {failure:?}");
            failure
        })?;
        estimator_result.map_err(|failure| {
            log::error!("RootSLAM VIO worker stopped with an error: {failure:?}");
            failure
        })?;
        shutdown_result?;
        if let Some(worker) = map_worker {
            worker.finish().map_err(|failure| {
                log::error!("RootSLAM map worker stopped with an error: {failure:?}");
                failure
            })?;
        }
        Ok(())
    }
}

/// Builds the Basalt VIO configuration with bias and yaw initialization priors.
///
/// The upstream archive replaces every field on load and skips the yaw prior, so the yaw
/// information is applied after the archive is read.
#[expect(
    clippy::field_reassign_with_default,
    reason = "The yaw prior must be applied after the archive load replaces every field"
)]
fn initial_vio_config(options: &Options) -> Result<VioConfig> {
    options.validate()?;
    let mut vio = VioConfig::default();
    // Basalt stores precision: invert the squared standard deviations.
    vio.vio_init_ba_weight = 1.0 / 0.1_f64.powi(2);
    vio.vio_init_bg_weight = 1.0 / 0.01_f64.powi(2);
    if let Some(path) = &options.vio_config {
        vio.load(path)?;
    }
    if let Some(size) = options.optical_flow_detection_grid_size {
        vio.optical_flow_detection_grid_size = size;
    }
    // Keep yaw information independent of the translation prior.
    vio.vio_init_yaw_weight = Some(1e2);
    Ok(vio)
}

fn render_calibration() -> Calibration<f64> {
    use firefly_base::{FrameId, rig};
    use firefly_calibration::camera::generic_camera::GenericCamera;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector2, Vector3, Vector4};
    let focal = IMAGE_HEIGHT as f64 * 0.5 / (rig::FOV_Y_DEG.to_radians() * 0.5).tan();
    let center = rig::principal_point(IMAGE_WIDTH, IMAGE_HEIGHT);
    let mut camera = GenericCamera::from_name("pinhole");
    camera.set_from_init(Vector4::new(focal, focal, center[0], center[1]));
    let pose = |frame| {
        let rotation =
            UnitQuaternion::from_matrix(&(rig::cam_axes_in_body() * rig::CV_FROM_RENDER));
        Isometry3::from_parts(Translation3::from(rig::position_in_body(frame)), rotation)
    };
    Calibration {
        t_i_c: vec![pose(FrameId::LEFT_CAMERA), pose(FrameId::RIGHT_CAMERA)],
        intrinsics: vec![camera.clone(), camera],
        resolution: vec![Vector2::new(IMAGE_WIDTH as i32, IMAGE_HEIGHT as i32); 2],
        imu_update_rate: 1_000.0,
        gyro_noise_std: Vector3::repeat(0.0002),
        accel_noise_std: Vector3::repeat(0.002),
        gyro_bias_std: Vector3::repeat(0.000_019),
        accel_bias_std: Vector3::repeat(0.003),
        ..Calibration::default()
    }
}

#[cfg(test)]
mod config_tests {
    use super::*;

    #[test]
    fn detection_override_preserves_all_other_online_parameters() {
        let options: Options = toml::from_str("optical_flow_detection_grid_size = 20").unwrap();
        let mut expected = initial_vio_config(&Options::default()).unwrap();
        assert_eq!(expected.optical_flow_detection_grid_size, 50);
        expected.optical_flow_detection_grid_size = 20;
        assert_eq!(initial_vio_config(&options).unwrap(), expected);
    }

    #[test]
    fn invalid_detection_cells_fail_before_runtime_creation() {
        for size in [-1, 0, 241] {
            let options = Options {
                optical_flow_detection_grid_size: Some(size),
                ..Options::default()
            };
            assert!(options.validate().is_err());
            assert!(initial_vio_config(&options).is_err());
        }
    }
}
