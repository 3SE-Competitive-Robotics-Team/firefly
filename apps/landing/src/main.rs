//! 下视识别、图像时刻几何与己方平台观测；生产推理使用 VSINPU。
mod model;
mod tracking;

use firefly_pubsub::{
    activity::{ACTIVITY_TOPIC, ActivityReceiver, LANDING},
    attitude::{ATTITUDE_TOPIC, AttitudeMessage},
    camera::{CAMERA_DOWN_TOPIC, DOWN_HEIGHT, DOWN_SENSOR_ID, DOWN_WIDTH, DownImageMessage},
    landing::{LANDING_TARGET_TOPIC, LandingTargetMessage},
    node::create_node,
    publish::Publisher,
    subscriber::Subscriber,
    viz::{ScalarMessage, VizPublisher},
};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalGuard, SignalHandler};
use serde::Deserialize;
use std::{
    path::PathBuf,
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
static RUNNING: AtomicBool = AtomicBool::new(true);

#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct Config {
    manifest: PathBuf,
    backend: model::Backend,
    runtime: Option<PathBuf>,
    rate_hz: f64,
    confidence: f32,
    diagnostics: bool,
    target_radius_m: f64,
    /// 标记平面到机体触地原点的高度差（米）；仿真为 21mm 减 1mm 涂层。
    contact_offset_m: f64,
    geometry: firefly_landing::geometry::Geometry,
    /// 光心在机体中的米制位置。
    camera_position: [f64; 3],
    /// 机体 ← CV 光学坐标旋转，单位四元数 xyzw。
    camera_rotation_xyzw: [f64; 4],
}
impl Default for Config {
    fn default() -> Self {
        let rotation = nalgebra::UnitQuaternion::from_matrix(
            &(firefly_base::rig::down_axes_in_body() * firefly_base::rig::CV_FROM_RENDER),
        );
        let q = rotation.quaternion();
        Self {
            manifest: firefly_config::paths::models("h-marker-yolo26n-cif-v1/manifest.json"),
            backend: model::Backend::Vsinpu,
            runtime: if cfg!(target_os = "linux") {
                Some(firefly_config::paths::models("npu/libonnxruntime.so"))
            } else {
                None
            },
            rate_hz: 10.,
            confidence: 0.5,
            diagnostics: false,
            target_radius_m: 0.8,
            contact_offset_m: 0.020,
            geometry: firefly_landing::geometry::Geometry::default(),
            camera_position: firefly_base::rig::DOWN_IN_BODY,
            camera_rotation_xyzw: [q.i, q.j, q.k, q.w],
        }
    }
}
fn validate(config: &Config) -> firefly_error::Result<()> {
    config.geometry.validate()?;
    if !config.rate_hz.is_finite()
        || !(5.0..=30.).contains(&config.rate_hz)
        || !config.confidence.is_finite()
        || !(0.5..=1.).contains(&config.confidence)
        || !config.target_radius_m.is_finite()
        || !(0.01..=1.5).contains(&config.target_radius_m)
        || !config.contact_offset_m.is_finite()
        || config.contact_offset_m.abs() > 0.2
        || !config
            .camera_position
            .iter()
            .chain(config.camera_rotation_xyzw.iter())
            .all(|v| v.is_finite())
        || (config
            .camera_rotation_xyzw
            .iter()
            .map(|v| v * v)
            .sum::<f64>()
            - 1.)
            .abs()
            > 1e-6
    {
        return Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            "invalid landing rate, confidence, acquisition radius or mounting",
        ));
    }
    Ok(())
}

fn main() -> Result<()> {
    let mut path = firefly_config::paths::config("landing.toml");
    let (mut image, mut simulation) = (None, false);
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--config" => path = args.next().ok_or("--config requires a path")?.into(),
            "--image" => image = Some(PathBuf::from(args.next().ok_or("--image requires a path")?)),
            "--simulation" => simulation = true,
            "--help" | "-h" => {
                println!(
                    "landing [--config <path>] [--image <352x288 image>] [--simulation]\nProduction requires INT8 and VSINPU. --simulation explicitly selects CPU."
                );
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    let mut config: Config = firefly_config::startup(path, firefly_observability::init, validate)?;
    let game = firefly_config::game::load()?;
    if simulation {
        config.backend = model::Backend::Cpu;
    }
    if config.backend != model::Backend::Vsinpu && !simulation && image.is_none() {
        return Err("live CPU inference requires explicit --simulation".into());
    }
    if let Some(path) = image {
        let environment = model::environment(config.runtime.as_deref())?;
        let image = image::open(path)?.to_luma8();
        if image.dimensions() != (DOWN_WIDTH as u32, DOWN_HEIGHT as u32) {
            return Err("offline input must be 352x288".into());
        }
        let mut model = model::Model::load(&config.manifest, config.backend, &environment)?;
        let mut results = Vec::new();
        let q = config.camera_rotation_xyzw;
        let camera = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                q[3], q[0], q[1], q[2],
            )),
        );
        for detection in model.detect(image.as_raw(), config.confidence)? {
            let best = firefly_landing::geometry::corner_candidates(
                image.as_raw(),
                DOWN_WIDTH,
                DOWN_HEIGHT,
                detection,
            )?
            .into_iter()
            .filter_map(|corners| {
                config
                    .geometry
                    .horizontal_target(corners, &camera)
                    .map(|pose| (corners, pose))
            })
            .min_by(|a, b| a.1.1.total_cmp(&b.1.1));
            let corners = best.as_ref().map(|(corners, _)| *corners);
            let pose = best.map(|(_, pose)| pose);
            let translation = pose.as_ref().map(|(point, _)| {
                camera
                    .inverse_transform_point(&nalgebra::Point3::from(*point))
                    .coords
            });
            results.push(
                serde_json::json!({"bounds":detection.bounds,"confidence":detection.confidence,
                "corners":corners,"camera_translation":translation.as_ref().map(nalgebra::Matrix::as_slice),
                "reprojection_px":pose.map(|(_,error)|error)}),
            );
        }
        println!("{}", serde_json::to_string(&results)?);
        return Ok(());
    }
    run(&config, game.team.code())
}

/// 工作资源只在 Active 中存在；Dormant 保留控制与姿态订阅。
enum Pipeline {
    Dormant,
    Active(Box<Worker>),
}

struct Worker {
    model: model::Model,
    camera: Subscriber<DownImageMessage>,
    odometry: firefly_planner_runtime::odometry::OdometryReceiver,
    tracker: tracking::Tracker,
    last_frame: f64,
    next_inference: Instant,
}

impl Worker {
    fn start(
        config: &Config,
        node: &firefly_pubsub::node::IpcNode,
        not_before: f64,
        environment: &ort::environment::Environment,
    ) -> Result<Self> {
        Ok(Self {
            model: model::Model::load(&config.manifest, config.backend, environment)?,
            camera: Subscriber::with_topic(node, CAMERA_DOWN_TOPIC)?,
            odometry: firefly_planner_runtime::odometry::OdometryReceiver::start()?,
            tracker: tracking::Tracker::default(),
            last_frame: not_before,
            next_inference: Instant::now(),
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn tick(
        &mut self,
        config: &Config,
        rate: u32,
        team: u32,
        session: u64,
        session_start: f64,
        mounting: &nalgebra::Isometry3<f64>,
        targets: &Publisher<LandingTargetMessage>,
        viz: Option<&VizPublisher>,
    ) -> Result<bool> {
        if Instant::now() < self.next_inference {
            return Ok(false);
        }
        let mut latest = None;
        while let Some(sample) = self.camera.receive()? {
            latest = Some(sample);
        }
        let Some(sample) = latest else {
            return Ok(false);
        };
        if sample.sensor_id != DOWN_SENSOR_ID
            || sample.width as usize != DOWN_WIDTH
            || sample.height as usize != DOWN_HEIGHT
            || !sample.timestamp.is_finite()
        {
            return Err("invalid down camera message".into());
        }
        if sample.timestamp <= self.last_frame {
            return Ok(false);
        }
        self.last_frame = sample.timestamp;
        let started = Instant::now();
        let detections = self.model.detect(&sample.data, config.confidence)?;
        let deadline = Instant::now() + Duration::from_millis(150);
        let pose = loop {
            if sample.timestamp < session_start || !RUNNING.load(Ordering::Relaxed) {
                break None;
            }
            let history = self.odometry.history().snapshot()?;
            if let Some(pose) = tracking::pose_at(&history, sample.timestamp) {
                break Some(pose);
            }
            if Instant::now() >= deadline {
                break None;
            }
            std::thread::sleep(Duration::from_millis(2));
        };
        let target = self.tracker.process(
            session,
            team,
            sample.timestamp,
            &sample.data,
            &detections,
            pose,
            &config.geometry,
            mounting,
            config.target_radius_m,
            config.contact_offset_m,
        );
        targets.publish(target)?;
        if let Some(viz) = viz {
            let mut row = ScalarMessage::base(sample.timestamp, "landing/vision");
            row.scalar_count = 4;
            row.scalars = [
                detections.len() as f64,
                f64::from(target.valid),
                target.reprojection_px,
                started.elapsed().as_secs_f64() * 1000.,
            ];
            viz.publish(row)?;
        }
        // Schedule start-to-start, without a catch-up burst after a slow inference.
        self.next_inference = (started
            + Duration::from_secs_f64(1. / config.rate_hz.min(f64::from(rate))))
        .max(Instant::now());
        Ok(true)
    }
}

#[allow(clippy::too_many_lines)]
fn run(config: &Config, team: u32) -> Result<()> {
    let _guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_| {
            RUNNING.store(false, Ordering::Relaxed);
        },
    )?;
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "landing");
    let mut activity = ActivityReceiver::open(&node, ACTIVITY_TOPIC)?;
    let targets = Publisher::<LandingTargetMessage>::with_topic(&node, LANDING_TARGET_TOPIC)?;
    let attitude = Subscriber::<AttitudeMessage>::with_topic(&node, ATTITUDE_TOPIC)?;
    let status = VizPublisher::new(&node)?;
    let q = config.camera_rotation_xyzw;
    let mounting = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::from(nalgebra::Vector3::from(config.camera_position)),
        nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(q[3], q[0], q[1], q[2])),
    );
    let mut pipeline = Pipeline::Dormant;
    let mut environment = None;
    let (mut session, mut session_start, mut stamp) = (0, f64::INFINITY, 0.);
    let (mut inferences, mut next_status) = (0_u64, Instant::now());
    while RUNNING.load(Ordering::Relaxed) {
        while let Some(sample) = attitude.receive()? {
            stamp = sample.timestamp;
            if sample.session != session && sample.session != 0 {
                session = sample.session;
                session_start = sample.timestamp;
                // A new estimator session always requires a new detector/tracker acquisition.
                pipeline = Pipeline::Dormant;
            }
        }
        let rate = activity.poll(LANDING)?;
        match (&pipeline, rate) {
            (Pipeline::Dormant, Some(_)) => {
                log::info!("landing pipeline starting: backend={:?}", config.backend);
                if environment.is_none() {
                    environment = Some(model::environment(config.runtime.as_deref()).inspect_err(
                        |error| {
                            log::error!("landing runtime activation failed: {error}");
                            firefly_observability::pump_log_ipc(&log_ipc);
                        },
                    )?);
                }
                let worker = Worker::start(
                    config,
                    &node,
                    stamp,
                    environment.as_ref().expect("runtime initialized"),
                )
                .inspect_err(|error| {
                    log::error!("landing pipeline activation failed: {error}");
                    firefly_observability::pump_log_ipc(&log_ipc);
                })?;
                pipeline = Pipeline::Active(Box::new(worker));
                log::info!("landing pipeline active");
                next_status = Instant::now();
                // Native model compilation may outlive a lease; recheck demand before the first inference.
                continue;
            }
            (Pipeline::Active(_), None) => {
                pipeline = Pipeline::Dormant;
                targets.publish(LandingTargetMessage {
                    timestamp: stamp,
                    session,
                    team,
                    ..Default::default()
                })?;
                log::info!(
                    "landing pipeline dormant: camera subscription, odometry worker and inference session released"
                );
                next_status = Instant::now();
            }
            _ => {}
        }
        if let (Pipeline::Active(worker), Some(rate)) = (&mut pipeline, rate) {
            inferences += u64::from(worker.tick(
                config,
                rate,
                team,
                session,
                session_start,
                &mounting,
                &targets,
                config.diagnostics.then_some(&status),
            )?);
        }
        if stamp.is_finite() && Instant::now() >= next_status {
            let mut row = ScalarMessage::base(stamp, "landing/runtime");
            row.scalar_count = 4;
            row.scalars = [
                f64::from(matches!(pipeline, Pipeline::Active(_))),
                inferences as f64,
                f64::from(rate.unwrap_or(0)),
                f64::from(config.backend == model::Backend::Vsinpu),
            ];
            status.publish(row)?;
            next_status = Instant::now() + Duration::from_secs(1);
        }
        firefly_observability::pump_log_ipc(&log_ipc);
        std::thread::sleep(Duration::from_millis(10));
    }
    drop(pipeline);
    firefly_observability::pump_log_ipc(&log_ipc);
    Ok(())
}

#[cfg(test)]
mod config_tests {
    use super::*;
    #[test]
    fn shipped_defaults_use_quantized_npu_and_validate_measured_mounting() {
        let mut config: Config =
            toml::from_str(include_str!("../../../configs/landing.toml")).unwrap();
        validate(&config).unwrap();
        assert_eq!(config.backend, model::Backend::Vsinpu);
        assert_eq!(config.camera_position, [-0.03, 0., 0.]);
        config.camera_rotation_xyzw = [0.; 4];
        assert!(validate(&config).is_err());
        assert!(toml::from_str::<Config>("unknown=1").is_err());
    }
}
