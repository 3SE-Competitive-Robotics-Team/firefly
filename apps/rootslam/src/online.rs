//! Online IPC adapter for the simulated camera and IMU topics.
use crate::config::{Options, initial_vio_config};
mod imu;
use crate::input::SensorModel;
use firefly_basalt::{
    io::dataset_io::ImageData,
    optical_flow::{OpticalFlowBase, OpticalFlowFactory, OpticalFlowInput},
    vi_estimator::{
        local_map::LocalMapSnapshot,
        vio_estimator::{VioEstimatorBase, VioEstimatorFactory, VioFrameDiagnostics},
    },
};
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
use firefly_rootslam_mapping::{
    global_optimization::Precision, loop_closure::LoopOptions, map::MapPolicy,
};
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
    emitted: Option<i64>,
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
        if self.emitted.is_some_and(|last| stamp <= last) {
            return Ok(());
        }
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
            if self.emitted.is_some_and(|last| time <= last) {
                self.left.pop_first();
                self.right.pop_first();
                continue;
            }
            if watermark < time {
                return None;
            }
            let left = self.left.pop_first()?.1;
            let right = self.right.pop_first()?.1;
            self.emitted = Some(time);
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
pub fn run(
    options: &Options,
    model: &SensorModel,
    precision: Precision,
    stop: &AtomicBool,
) -> Result<()> {
    LiveSession::new(options, model, precision)?.run(stop)
}

struct LiveSession {
    model: SensorModel,
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
    startup: Startup,
    frame_started: imu::FrameStarts,
}

#[derive(Clone, Copy)]
enum Startup {
    WaitingGravity,
    WaitingStereo(GravityEstimate),
    Running(GravityEstimate),
}

impl Startup {
    fn gravity(self) -> Option<GravityEstimate> {
        match self {
            Self::WaitingGravity => None,
            Self::WaitingStereo(estimate) | Self::Running(estimate) => Some(estimate),
        }
    }
}

impl LiveSession {
    fn new(options: &Options, model: &SensorModel, precision: Precision) -> Result<Self> {
        let calibration = &model.calibration;
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
            calibration,
            &model.gravity,
            true,
            precision == Precision::Float64,
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
            calibration,
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
            model: SensorModel {
                calibration: calibration.clone(),
                gravity: model.gravity,
            },
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
            startup: Startup::WaitingGravity,
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
                self.startup = match self.startup {
                    Startup::Running(_) => Startup::Running(estimate),
                    Startup::WaitingGravity | Startup::WaitingStereo(_) => {
                        Startup::WaitingStereo(estimate)
                    }
                };
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
        while let Some((t_ns, img_data)) = self.startup.gravity().and_then(|estimate| {
            self.pairing
                .discard_before(i64::try_from(estimate.end_ns).ok()?);
            self.pairing
                .next(i64::try_from(self.imu_watermark.load()?.as_nanosecond()).ok()?)
        }) {
            if let Startup::WaitingStereo(estimate) = self.startup {
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
                self.startup = Startup::Running(estimate);
            }
            let mut frame_started = self
                .frame_started
                .lock()
                .map_err(|_| error("frame timing mutex poisoned"))?;
            let enqueue_begin = firefly_basalt::utils::time_utils::profile_clock_ns();
            match self
                .flow_tx
                .try_send(Some(Arc::new(crate::input::image_frame(
                    &self.model.calibration,
                    t_ns,
                    img_data,
                )?))) {
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
            if let Some(solver) = data.solver {
                self.publish_solver_diagnostics(stamp, &solver);
            }
            if let Some(updates) = data.yaw_updates {
                publish_scalar_diagnostic(
                    &self.viz,
                    stamp,
                    "vio/diagnostics/yaw_updates_rad",
                    updates,
                );
            }
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

    fn publish_solver_diagnostics(
        &self,
        stamp: f64,
        solver: &firefly_basalt::vi_estimator::vio_estimator::VioSolverDiagnostics,
    ) {
        for (name, values) in [
            ("vio/diagnostics/costs_before", solver.costs_before),
            ("vio/diagnostics/costs_after", solver.costs_after),
            ("vio/diagnostics/solver_termination", solver.termination),
        ] {
            publish_scalar_diagnostic(&self.viz, stamp, name, values);
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

#[cfg(test)]
mod config_tests {
    use super::*;
    use firefly_basalt::utils::vio_config::VioConfig;

    #[test]
    fn stereo_waits_for_imu_and_never_replays_consumed_frames() {
        let mut pairing = Pairing::default();
        let mut packet = GrayImageMessage {
            width: IMAGE_WIDTH as _,
            height: IMAGE_HEIGHT as _,
            ..Default::default()
        };
        for stamp in [1., 1., 0.9, 2.] {
            packet.timestamp = stamp;
            for camera in 0..2 {
                packet.sensor_id = camera as _;
                pairing.insert(&packet, camera).unwrap();
            }
            if stamp == 1. && pairing.emitted.is_none() {
                assert!(pairing.next(999_999_999).is_none());
                assert_eq!(pairing.next(1_000_000_000).unwrap().0, 1_000_000_000);
            } else if stamp <= 1. {
                assert!(pairing.next(i64::MAX).is_none());
                assert!(pairing.left.is_empty() && pairing.right.is_empty());
            } else {
                assert_eq!(pairing.next(i64::MAX).unwrap().0, 2_000_000_000);
            }
        }
    }

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
        for size in [-1, 0] {
            let options = Options {
                optical_flow_detection_grid_size: Some(size),
                ..Options::default()
            };
            assert!(options.validate().is_err());
            assert!(initial_vio_config(&options).is_err());
        }
    }

    #[test]
    fn stereo_baseline_gate_is_independent_of_body_attitude() {
        use firefly_basalt::utils::ba_utils::compute_rel_pose;
        use nalgebra::UnitQuaternion;
        let calibration = crate::input::simulator_calibration();
        let minimum = VioConfig::default().vio_min_triangulation_dist.powi(2);
        for step in 0..360 {
            let t = f64::from(step) * 0.01;
            let body = Isometry3::from_parts(
                Translation3::new(t, -t, t),
                UnitQuaternion::from_euler_angles(0.01 * t, -0.02 * t, 0.03 * t),
            );
            let relative = compute_rel_pose(
                &body,
                &calibration.t_i_c[1],
                &body,
                &calibration.t_i_c[0],
                None,
                None,
            );
            assert!(
                relative.translation.vector.norm_squared() >= minimum,
                "step={step}, baseline2={:.18}, minimum2={minimum:.18}",
                relative.translation.vector.norm_squared()
            );
        }
    }

    #[test]
    fn the_board_hil_configuration_selects_the_rig_preset() {
        #[derive(serde::Deserialize)]
        struct Document {
            online: Options,
        }
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../configs/rootslam-hil.toml");
        let document = std::fs::read_to_string(&path).expect("板端 HIL 配置随仓库发布");
        let parsed: Document = toml::from_str(&document).expect("板端 HIL 配置可解析");
        assert_eq!(parsed.online.vio_preset, VioPreset::Rig);
        assert!(
            parsed.online.use_double,
            "单精度路径没有板端验证，保持双精度"
        );
    }
}
