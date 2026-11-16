//! 下视识别、图像时刻几何与己方平台观测；生产推理使用 VSINPU。
mod model;
mod tracking;

use firefly_pubsub::{
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
    if let Some(path) = image {
        let image = image::open(path)?.to_luma8();
        if image.dimensions() != (DOWN_WIDTH as u32, DOWN_HEIGHT as u32) {
            return Err("offline input must be 352x288".into());
        }
        let mut model =
            model::Model::load(&config.manifest, config.backend, config.runtime.as_deref())?;
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
    if config.backend != model::Backend::Vsinpu && !simulation {
        return Err("live CPU inference requires explicit --simulation".into());
    }
    run(&config, game.team.code())
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
    let mut model =
        model::Model::load(&config.manifest, config.backend, config.runtime.as_deref())?;
    let camera = Subscriber::<DownImageMessage>::with_topic(&node, CAMERA_DOWN_TOPIC)?;
    let targets = Publisher::<LandingTargetMessage>::with_topic(&node, LANDING_TARGET_TOPIC)?;
    let attitude = Subscriber::<AttitudeMessage>::with_topic(&node, ATTITUDE_TOPIC)?;
    let odometry = firefly_planner_runtime::odometry::OdometryReceiver::start()?;
    let mut tracker = tracking::Tracker::default();
    let (mut session, mut session_start) = (0, f64::INFINITY);
    let mounting = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::from(nalgebra::Vector3::from(config.camera_position)),
        nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            config.camera_rotation_xyzw[3],
            config.camera_rotation_xyzw[0],
            config.camera_rotation_xyzw[1],
            config.camera_rotation_xyzw[2],
        )),
    );
    let viz = config
        .diagnostics
        .then(|| VizPublisher::new(&node))
        .transpose()?;
    let period = Duration::from_secs_f64(1. / config.rate_hz);
    let (mut last_frame, mut next_inference) = (f64::NEG_INFINITY, Instant::now());
    while RUNNING.load(Ordering::Relaxed) {
        while let Some(sample) = attitude.receive()? {
            if sample.session != session && sample.session != 0 {
                session = sample.session;
                session_start = sample.timestamp;
            }
        }
        if Instant::now() >= next_inference {
            let mut latest = None;
            while let Some(sample) = camera.receive()? {
                latest = Some(sample);
            }
            if let Some(sample) = latest {
                if sample.sensor_id != DOWN_SENSOR_ID
                    || sample.width as usize != DOWN_WIDTH
                    || sample.height as usize != DOWN_HEIGHT
                    || !sample.timestamp.is_finite()
                {
                    return Err("invalid down camera message".into());
                }
                if sample.timestamp > last_frame {
                    last_frame = sample.timestamp;
                    let started = Instant::now();
                    let detections = model.detect(&sample.data, config.confidence)?;
                    // A camera frame can arrive before its VIO posterior. Wait
                    // within the FC freshness budget for a bracketing pose.
                    let deadline = Instant::now() + Duration::from_millis(150);
                    let pose = loop {
                        if sample.timestamp < session_start || !RUNNING.load(Ordering::Relaxed) {
                            break None;
                        }
                        let history = odometry.history().snapshot()?;
                        if let Some(pose) = tracking::pose_at(&history, sample.timestamp) {
                            break Some(pose);
                        }
                        if Instant::now() >= deadline {
                            break None;
                        }
                        std::thread::sleep(Duration::from_millis(2));
                    };
                    let target = tracker.process(
                        session,
                        team,
                        sample.timestamp,
                        &sample.data,
                        &detections,
                        pose,
                        &config.geometry,
                        &mounting,
                        config.target_radius_m,
                        config.contact_offset_m,
                    );
                    targets.publish(target)?;
                    if let Some(viz) = &viz {
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
                    next_inference = Instant::now() + period;
                }
            }
        }
        firefly_observability::pump_log_ipc(&log_ipc);
        std::thread::sleep(Duration::from_millis(1));
    }
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
