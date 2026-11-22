use crate::{RUNNING, Result, config::Config, model, tracking};
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
use std::{
    sync::atomic::Ordering,
    time::{Duration, Instant},
};

/// 工作资源只在 Active 中存在；Dormant 保留控制与姿态订阅。
enum Pipeline {
    Dormant,
    Active(Box<Worker>),
}

struct Worker {
    model: model::Model,
    camera: Subscriber<DownImageMessage>,
    odometry: firefly_odometry::OdometryReceiver,
    tracker: tracking::Tracker,
    last_frame: f64,
    next_inference: Instant,
}

impl Worker {
    fn start(
        config: &Config,
        node: &firefly_pubsub::node::IpcNode,
        not_before: f64,
        environment: &model::Environment,
    ) -> Result<Self> {
        Ok(Self {
            model: model::Model::load(&config.manifest(), environment)?,
            camera: Subscriber::with_topic(node, CAMERA_DOWN_TOPIC)?,
            odometry: firefly_odometry::OdometryReceiver::start()?,
            tracker: tracking::Tracker::new(config.tracking()),
            last_frame: not_before,
            next_inference: Instant::now(),
        })
    }

    fn tick(
        &mut self,
        config: &Config,
        team: u32,
        session: u64,
        session_start: f64,
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
            return Err(crate::invalid("invalid down camera message"));
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
        let target = self.tracker.process(tracking::Observation {
            session,
            team,
            timestamp: sample.timestamp,
            image: &sample.data,
            detections: &detections,
            pose,
        });
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
        self.next_inference =
            (started + Duration::from_secs_f64(1. / config.rate_hz)).max(Instant::now());
        Ok(true)
    }
}

#[allow(clippy::too_many_lines)]
pub(crate) fn run(config: &Config, team: u32) -> Result<()> {
    let _guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_| {
            RUNNING.store(false, Ordering::Relaxed);
        },
    )
    .map_err(|e| crate::external("landing.signal.register", e))?;
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "landing");
    let mut activity = ActivityReceiver::open(&node, ACTIVITY_TOPIC)?;
    let targets = Publisher::<LandingTargetMessage>::with_topic(&node, LANDING_TARGET_TOPIC)?;
    let attitude = Subscriber::<AttitudeMessage>::with_topic(&node, ATTITUDE_TOPIC)?;
    let status = VizPublisher::new(&node)?;
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
        let requested = activity.poll(LANDING)?;
        match (&pipeline, requested) {
            (Pipeline::Dormant, true) => {
                log::info!("landing pipeline starting: backend={:?}", config.backend);
                if environment.is_none() {
                    environment = Some(
                        model::environment(config.backend, config.runtime.as_deref()).inspect_err(
                            |error| {
                                log::error!("landing runtime activation failed: {error}");
                                firefly_observability::pump_log_ipc(&log_ipc);
                            },
                        )?,
                    );
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
            (Pipeline::Active(_), false) => {
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
        if let (Pipeline::Active(worker), true) = (&mut pipeline, requested) {
            inferences += u64::from(worker.tick(
                config,
                team,
                session,
                session_start,
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
                f64::from(requested),
                f64::from(config.backend == model::Backend::Viplite),
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
