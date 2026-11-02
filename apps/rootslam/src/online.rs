//! Online IPC adapter for the simulated camera and IMU topics.
use firefly_basalt::imu::imu_types::{ImuData, PoseVelBiasState};
use firefly_basalt::{
    io::dataset_io::ImageData,
    optical_flow::{OpticalFlowBase, OpticalFlowFactory, OpticalFlowInput},
    utils::vio_config::VioConfig,
    vi_estimator::{
        local_map::LocalMapSnapshot,
        vio_estimator::{VioEstimatorBase, VioEstimatorFactory},
    },
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::image::image::ManagedImage;
use firefly_error::{Error, ErrorKind, Result};
use firefly_pubsub::{
    attitude::{ATTITUDE_TOPIC, AttitudeMessage},
    camera::{CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, GrayImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH},
    imu::{IMU_SUBSCRIPTION, IMU_TOPIC, ImuMessage},
    navigation::{NAVIGATION_TOPIC, NavigationMessage},
    node::IpcNode,
    node::create_node,
    odom::{CORRECTED_ODOM_TOPIC, ODOM_TOPIC, OdomMessage},
    publish::Publisher,
    subscriber::{Subscriber, SubscriberOptions},
    viz::{ScalarMessage, VizPublisher},
};
use firefly_rootslam_mapping::{loop_closure::LoopOptions, map::MapPolicy};
use nalgebra::Vector3;
use std::{
    collections::{BTreeMap, VecDeque},
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    time::Duration,
};

#[derive(serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Options {
    pub vio_config: Option<PathBuf>,
    /// VIO 参数预设；`vio_config` 指定的文件在预设之后整体覆盖。
    pub vio_preset: VioPreset,
    pub stats_directory: Option<PathBuf>,
    pub use_double: bool,
    pub map: bool,
    /// 回环检测开关；关闭后只保留局部建图与关键帧共视图。
    pub loops: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            vio_config: None,
            vio_preset: VioPreset::default(),
            stats_directory: None,
            use_double: false,
            map: true,
            loops: true,
        }
    }
}

/// Basalt VIO 的参数预设。
///
/// Basalt 自带的默认参数是按 512×512 的 TUM-VI 调的。本仓库的 rig 是 320×240,
/// 同样的像素网格在 rig 上只剩约四分之一的特征点,光流跟踪密度不足以支撑
/// 悬停。`Rig` 预设按 rig 的分辨率重选网格边长,特征点数与 OKVIS2-X 在板端
/// 实测可用的 200 点对齐。
#[derive(Clone, Copy, Debug, Default, serde::Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum VioPreset {
    /// Basalt 默认参数,按 512×512 TUM-VI 调。
    Basalt,
    /// 按 320×240 双目 rig 选定的参数。
    #[default]
    Rig,
}

impl VioPreset {
    /// 构造该预设的 Basalt 配置。
    #[must_use]
    pub fn vio_config(self) -> VioConfig {
        match self {
            Self::Basalt => VioConfig::default(),
            Self::Rig => VioConfig {
                // 网格边长 50 px 在 320×240 上只有 6×4 个单元（约 24 个特征点）,
                // 20 px 得到 16×12 个单元（约 192 个特征点）。
                optical_flow_detection_grid_size: 20,
                ..VioConfig::default()
            },
        }
    }
}

fn error(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::Internal, message).with_operation("rootslam.online")
}

fn flush_imu(
    sender: &mpsc::SyncSender<Option<Arc<ImuData<f64>>>>,
    pending: &mut VecDeque<Arc<ImuData<f64>>>,
    sent: &mut u64,
) -> Result<()> {
    while let Some(data) = pending.front().cloned() {
        match sender.try_send(Some(data)) {
            Ok(()) => {
                pending.pop_front();
                *sent += 1;
            }
            Err(mpsc::TrySendError::Full(_)) => break,
            Err(mpsc::TrySendError::Disconnected(_)) => {
                return Err(error("VIO IMU input closed"));
            }
        }
    }
    Ok(())
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
            *destination = u16::from(source) * 257;
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
    imu: Subscriber<ImuMessage>,
    left: Subscriber<GrayImageMessage>,
    right: Subscriber<GrayImageMessage>,
    odom_pub: Publisher<OdomMessage>,
    corrected_pub: Publisher<OdomMessage>,
    nav_pub: Publisher<NavigationMessage>,
    attitude_pub: Publisher<AttitudeMessage>,
    viz: VizPublisher,
    estimator: VioEstimatorBase,
    flow: OpticalFlowBase,
    map_worker: Option<crate::system::MappingWorker>,
    map_input: Option<std::sync::mpsc::Sender<Option<Arc<LocalMapSnapshot>>>>,
    imu_tx: std::sync::mpsc::SyncSender<Option<Arc<ImuData<f64>>>>,
    flow_tx: std::sync::mpsc::SyncSender<Option<Arc<OpticalFlowInput>>>,
    states_rx: mpsc::Receiver<Option<Arc<PoseVelBiasState<f64>>>>,
    stats_directory: Option<PathBuf>,
    pairing: Pairing,
    imu_watermark: i64,
    stationary_since: Option<i64>,
    initialization_ready_at: Option<i64>,
    pending_imu: VecDeque<Arc<ImuData<f64>>>,
    frame_started: BTreeMap<i64, std::time::Instant>,
    session: u64,
    last_gyro: Vector3<f64>,
    gyro_bias: Vector3<f64>,
    predicted_attitude: Option<(nalgebra::UnitQuaternion<f64>, f64, f64)>,
    imu_count: u64,
    image_count: u64,
    pair_count: u64,
    state_count: u64,
    last_report: std::time::Instant,
}

impl LiveSession {
    fn new(options: &Options) -> Result<Self> {
        let calibration = render_calibration();
        let mut vio = options.vio_preset.vio_config();
        if let Some(path) = &options.vio_config {
            vio.load(path)?;
        }
        let node = create_node()?;
        let imu = Subscriber::<ImuMessage>::with_options(&node, IMU_TOPIC, IMU_SUBSCRIPTION)?;
        let image_options = SubscriberOptions {
            buffer_size: 2,
            ..Default::default()
        };
        let left =
            Subscriber::<GrayImageMessage>::with_options(&node, CAMERA_LEFT_TOPIC, image_options)?;
        let right =
            Subscriber::<GrayImageMessage>::with_options(&node, CAMERA_RIGHT_TOPIC, image_options)?;
        let odom_pub = Publisher::<OdomMessage>::with_topic(&node, ODOM_TOPIC)?;
        let corrected_pub = Publisher::<OdomMessage>::with_topic(&node, CORRECTED_ODOM_TOPIC)?;
        let nav_pub = Publisher::<NavigationMessage>::with_topic(&node, NAVIGATION_TOPIC)?;
        let attitude_pub = Publisher::<AttitudeMessage>::with_topic(&node, ATTITUDE_TOPIC)?;
        let viz = VizPublisher::new(&node)?;
        let mut estimator = VioEstimatorFactory::get_vio_estimator(
            &vio,
            &calibration,
            &Vector3::new(0., 0., -9.81),
            true,
            options.use_double,
        )?;
        let (states_tx, states_rx) = mpsc::channel();
        estimator.outputs.state = Some(states_tx);
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
        estimator.initialize(Vector3::zeros(), Vector3::zeros())?;
        let flow = OpticalFlowFactory::get_optical_flow_with_output(
            &vio,
            &calibration,
            estimator.vision_data_queue().clone(),
        )?;
        let imu_tx = estimator.imu_data_queue().clone();
        let flow_tx = flow.input_queue().clone();
        let session = (std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64)
            .max(1);
        Ok(Self {
            node,
            imu,
            left,
            right,
            odom_pub,
            corrected_pub,
            nav_pub,
            attitude_pub,
            viz,
            estimator,
            flow,
            map_worker,
            map_input,
            imu_tx,
            flow_tx,
            states_rx,
            stats_directory: options.stats_directory.clone(),
            pairing: Pairing::default(),
            imu_watermark: i64::MIN,
            stationary_since: None,
            initialization_ready_at: None,
            pending_imu: VecDeque::new(),
            frame_started: BTreeMap::new(),
            session,
            last_gyro: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            predicted_attitude: None,
            imu_count: 0,
            image_count: 0,
            pair_count: 0,
            state_count: 0,
            last_report: std::time::Instant::now(),
        })
    }

    fn run(mut self, stop: &AtomicBool) -> Result<()> {
        let process_result = self.process(stop);
        let shutdown_result = self.shutdown();
        process_result?;
        shutdown_result
    }

    fn process(&mut self, stop: &AtomicBool) -> Result<()> {
        while !stop.load(Ordering::Acquire) {
            if self.node.wait(Duration::from_millis(2)).is_err() {
                break;
            }
            self.receive_imu()?;
            self.receive_images()?;
            self.enqueue_stereo()?;
            self.publish_states()?;
            flush_imu(&self.imu_tx, &mut self.pending_imu, &mut self.imu_count)?;
            self.report_counts();
        }
        Ok(())
    }

    fn receive_imu(&mut self) -> Result<()> {
        for _ in 0..128 {
            let Some(packet) = self.imu.receive()? else {
                break;
            };
            let packet = &*packet;
            if !packet.timestamp.is_finite() || packet.timestamp < 0.0 {
                continue;
            }
            let stamp = (packet.timestamp * 1e9).round() as i64;
            if stamp <= self.imu_watermark {
                continue;
            }
            let gyro = Vector3::new(
                packet.angular_velocity_x,
                packet.angular_velocity_y,
                packet.angular_velocity_z,
            );
            let accel = Vector3::new(
                packet.linear_acceleration_x,
                packet.linear_acceleration_y,
                packet.linear_acceleration_z,
            );
            if gyro
                .iter()
                .chain(accel.iter())
                .any(|value| !value.is_finite())
            {
                continue;
            }
            self.update_initialization_window(stamp, &accel, &gyro);
            self.last_gyro = gyro;
            self.imu_watermark = stamp;
            if self.pending_imu.len() >= 10_000 {
                return Err(error("VIO IMU backlog exceeded 10 seconds"));
            }
            self.pending_imu.push_back(Arc::new(ImuData {
                t_ns: stamp,
                accel,
                gyro,
            }));
            self.publish_prediction(packet.timestamp, gyro)?;
        }
        flush_imu(&self.imu_tx, &mut self.pending_imu, &mut self.imu_count)
    }

    fn update_initialization_window(
        &mut self,
        stamp: i64,
        accel: &Vector3<f64>,
        gyro: &Vector3<f64>,
    ) {
        let quiet_on_ground = (8.0..=11.5).contains(&accel.norm()) && gyro.norm() < 0.2;
        if quiet_on_ground {
            self.stationary_since.get_or_insert(stamp);
        } else {
            self.stationary_since = None;
        }
        if self.initialization_ready_at.is_none()
            && self
                .stationary_since
                .is_some_and(|since| stamp - since >= 100_000_000)
        {
            self.initialization_ready_at = Some(stamp);
            self.pairing.discard_before(stamp);
            log::info!(
                "RootSLAM VIO initialization enabled after stable IMU window at {:.3}s",
                stamp as f64 * 1e-9
            );
        }
    }

    fn publish_prediction(&mut self, timestamp: f64, gyro: Vector3<f64>) -> Result<()> {
        let Some((rotation, posterior_time, previous_time)) = self.predicted_attitude.as_mut()
        else {
            return Ok(());
        };
        if timestamp <= *previous_time {
            return Ok(());
        }
        let corrected_gyro = gyro - self.gyro_bias;
        *rotation *= nalgebra::UnitQuaternion::from_scaled_axis(
            corrected_gyro * (timestamp - *previous_time),
        );
        *previous_time = timestamp;
        let q = rotation.quaternion();
        self.attitude_pub.publish(AttitudeMessage {
            timestamp,
            posterior_time: *posterior_time,
            session: self.session,
            quat_xyzw: [q.i, q.j, q.k, q.w],
            angular_velocity: corrected_gyro.into(),
            gyro_bias: self.gyro_bias.into(),
        })?;
        Ok(())
    }

    fn receive_images(&mut self) -> Result<()> {
        for _ in 0..4 {
            let Some(packet) = self.left.receive()? else {
                break;
            };
            self.pairing.insert(&packet, 0)?;
            self.image_count += 1;
        }
        for _ in 0..4 {
            let Some(packet) = self.right.receive()? else {
                break;
            };
            self.pairing.insert(&packet, 1)?;
            self.image_count += 1;
        }
        Ok(())
    }

    fn enqueue_stereo(&mut self) -> Result<()> {
        while let Some((t_ns, img_data)) = self.initialization_ready_at.and_then(|ready_at| {
            self.pairing.discard_before(ready_at);
            self.pairing.next(self.imu_watermark)
        }) {
            match self
                .flow_tx
                .try_send(Some(Arc::new(OpticalFlowInput { t_ns, img_data })))
            {
                Ok(()) => {
                    self.frame_started.insert(t_ns, std::time::Instant::now());
                    self.pair_count += 1;
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

    fn publish_states(&mut self) -> Result<()> {
        for _ in 0..64 {
            let Ok(state) = self.states_rx.try_recv() else {
                break;
            };
            let state = state.ok_or_else(|| error("VIO state stream ended"))?;
            self.publish_state(&state)?;
        }
        Ok(())
    }

    fn publish_state(&mut self, state: &Arc<PoseVelBiasState<f64>>) -> Result<()> {
        let state = state.cast::<f64>();
        let pose = &state.state.pose.t_w_i;
        let p = pose.translation.vector;
        let q = pose.rotation.quaternion();
        let vel = state.state.vel_w_i;
        let stamp = state.state.pose.t_ns as f64 * 1e-9;
        if let Some(started) = self.frame_started.remove(&state.state.pose.t_ns) {
            let mut timing = ScalarMessage::base(stamp, "rootslam/frame_processing_ms");
            timing.scalars[0] = started.elapsed().as_secs_f64() * 1000.0;
            timing.scalar_count = 1;
            let _ = self.viz.publish(timing);
        }
        self.gyro_bias = state.bias_gyro;
        let corrected_gyro = self.last_gyro - self.gyro_bias;
        let odom = OdomMessage {
            timestamp: stamp,
            position_x: p.x,
            position_y: p.y,
            position_z: p.z,
            velocity_x: vel.x,
            velocity_y: vel.y,
            velocity_z: vel.z,
            quat_x: q.i,
            quat_y: q.j,
            quat_z: q.k,
            quat_w: q.w,
            is_initialized: true,
        };
        self.odom_pub.publish(odom)?;
        self.corrected_pub.publish(odom)?;
        let alignment = firefly_base::RigidTransform::from_parts(
            firefly_base::FrameId::MAP,
            firefly_base::FrameId::ODOM,
            [0.; 3],
            [0., 0., 0., 1.],
        )?;
        self.nav_pub
            .publish(NavigationMessage::new(odom, &alignment, 0))?;
        self.predicted_attitude = Some((pose.rotation, stamp, stamp));
        self.attitude_pub.publish(AttitudeMessage {
            timestamp: stamp,
            posterior_time: stamp,
            session: self.session,
            quat_xyzw: [q.i, q.j, q.k, q.w],
            angular_velocity: corrected_gyro.into(),
            gyro_bias: state.bias_gyro.into(),
        })?;
        self.state_count += 1;
        Ok(())
    }

    fn report_counts(&mut self) {
        if self.last_report.elapsed() >= Duration::from_secs(2) {
            log::info!(
                "RootSLAM live input/output counts: IMU={}, images={}, stereo={}, states={}",
                self.imu_count,
                self.image_count,
                self.pair_count,
                self.state_count
            );
            self.last_report = std::time::Instant::now();
        }
    }

    fn shutdown(self) -> Result<()> {
        let Self {
            map_worker,
            map_input,
            imu_tx,
            flow_tx,
            flow,
            mut estimator,
            stats_directory,
            ..
        } = self;
        drop(map_input);
        drop(imu_tx);
        drop(flow_tx);
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

fn render_calibration() -> Calibration<f64> {
    use firefly_base::{FrameId, rig};
    use firefly_calibration::camera::generic_camera::GenericCamera;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector2, Vector4};
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
        ..Calibration::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 光流检测的网格单元数；每单元最多取一个角点。
    fn detection_cells(grid_size: i32) -> usize {
        let grid = grid_size as usize;
        let columns = (IMAGE_WIDTH - IMAGE_WIDTH % grid) / grid;
        let rows = (IMAGE_HEIGHT - IMAGE_HEIGHT % grid) / grid;
        columns * rows
    }

    #[test]
    fn the_basalt_preset_matches_the_upstream_defaults() {
        let preset = VioPreset::Basalt.vio_config();
        let upstream = VioConfig::default();
        assert_eq!(
            preset.optical_flow_detection_grid_size,
            upstream.optical_flow_detection_grid_size
        );
        assert_eq!(preset.optical_flow_levels, upstream.optical_flow_levels);
    }

    #[test]
    fn the_rig_preset_keeps_the_feature_count_of_the_board_tested_estimator() {
        let preset = VioPreset::Rig.vio_config();
        // 320×240 上 50 px 网格只剩 24 个单元；20 px 网格给到 192 个。
        assert_eq!(detection_cells(50), 24);
        assert_eq!(detection_cells(20), 192);
        assert_eq!(
            preset.optical_flow_detection_grid_size, 20,
            "rig 预设的光流网格边长"
        );
        assert_eq!(
            detection_cells(preset.optical_flow_detection_grid_size),
            192
        );
    }

    #[test]
    fn the_rig_preset_is_the_default_of_the_live_session() {
        assert_eq!(Options::default().vio_preset, VioPreset::Rig);
    }
}
