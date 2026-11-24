//! Depth and pose are paired by measurement-time interpolation;
//! occupancy and fading use independent timers.
use std::{
    collections::VecDeque,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc::SyncSender,
    },
    thread::JoinHandle,
    time::{Duration, Instant},
};

use firefly_error::{Error, ErrorKind, Result};
use firefly_map::depth::{DepthImage, DepthMapper, FrameStats, FrameUpdate};
use firefly_odometry::{OdometryHistory, PoseBracket};
use firefly_pubsub::{
    camera::{DEPTH_TOPIC, DepthImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH},
    node::create_node,
    odom::OdomMessage,
    subscriber::Subscriber,
    trace::TraceContext,
    viz::{ScalarMessage, ScalarPublisher},
};
use nalgebra::Isometry3;

use crate::{config::DepthOptions, map_input::LocalSnapshot};

const OCCUPANCY_PERIOD: Duration = Duration::from_millis(32);
const FADING_PERIOD: Duration = Duration::from_millis(500);
const MAX_PENDING_DEPTHS: usize = 8;

#[derive(Default)]
struct Shared {
    latest: Option<LocalSnapshot>,
    error: Option<Error>,
}

pub struct DepthReceiver {
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl DepthReceiver {
    pub fn start(fusion: DepthFusion, history: OdometryHistory) -> Result<Self> {
        let shared = Arc::new(Mutex::new(Shared::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let (state, stopping) = (Arc::clone(&shared), Arc::clone(&stop));
        let (ready, startup) = std::sync::mpsc::sync_channel(1);
        let thread = std::thread::Builder::new()
            .name("planner-depth-map".into())
            .spawn(move || {
                if let Err(error) = run_mapping(fusion, &history, &stopping, &state, &ready) {
                    if let Ok(mut state) = state.lock() {
                        state.error = Some(error.clone());
                        state.latest = None;
                    }
                    log::error!("depth mapping receiver: {error}");
                    let _ = ready.send(Err(error));
                }
                stopping.store(true, Ordering::Release);
            })
            .map_err(|e| Error::from(e).with_operation("spawn depth mapping"))?;
        let receiver = Self {
            shared,
            stop,
            thread: Some(thread),
        };
        startup
            .recv()
            .map_err(|e| Error::caused_by(ErrorKind::Unavailable, "start depth mapping", e))??;
        Ok(receiver)
    }

    pub fn take_snapshot(&self) -> Result<Option<LocalSnapshot>> {
        let mut shared = self
            .shared
            .lock()
            .map_err(|_| internal("depth state poisoned"))?;
        if let Some(error) = &shared.error {
            return Err(error.clone());
        }
        if !self.stop.load(Ordering::Acquire)
            && self.thread.as_ref().is_some_and(JoinHandle::is_finished)
        {
            return Err(internal("depth mapping worker panicked"));
        }
        Ok(shared.latest.take())
    }

    pub fn terminated(&self) -> bool {
        self.stop.load(Ordering::Acquire)
            || self.thread.as_ref().is_some_and(JoinHandle::is_finished)
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            log::error!("depth mapping receiver panicked");
        }
    }
}

impl Drop for DepthReceiver {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn run_mapping(
    mut fusion: DepthFusion,
    history: &OdometryHistory,
    stop: &AtomicBool,
    shared: &Mutex<Shared>,
    ready: &SyncSender<Result<()>>,
) -> Result<()> {
    let node = create_node()?;
    let subscriber = Subscriber::<DepthImageMessage>::with_topic(&node, DEPTH_TOPIC)?;
    let diagnostics = ScalarPublisher::new(&node)
        .inspect_err(|e| {
            log::warn!("map timing publisher unavailable: {e}");
        })
        .ok();
    let _ = ready.send(Ok(()));
    let epoch = Instant::now();
    let mut next_occupancy = epoch + OCCUPANCY_PERIOD;
    let mut next_fading = epoch + FADING_PERIOD;
    let mut last_stamp = None;
    let mut pending = VecDeque::new();

    while !stop.load(Ordering::Relaxed) && node.wait(Duration::from_millis(2)).is_ok() {
        // The subscriber keeps one latest frame; bound ingress work before servicing timers.
        if let Some(message) = subscriber.receive()? {
            if !message.timestamp.is_finite() || message.timestamp < 0. {
                return Err(invalid("invalid depth timestamp"));
            }
            if last_stamp.is_none_or(|stamp| message.timestamp > stamp) {
                last_stamp = Some(message.timestamp);
                pending.push_back(DepthFrame::from_message(
                    &message,
                    &fusion.map,
                    fusion.options.k_depth_scaling_factor,
                    *message.user_header(),
                )?);
                if pending.len() > MAX_PENDING_DEPTHS {
                    pending.pop_front();
                    log::warn!("planner depth queue overrun; dropped oldest unpaired frame");
                }
            }
        }

        let now = Instant::now();
        let mut timing = UpdateTiming::default();
        if timer_due(&mut next_occupancy, OCCUPANCY_PERIOD, now)
            && let Some(pair) = history
                .read(|poses| take_pair(&mut pending, poses, now, fusion.options.max_pose_gap))?
        {
            let timer_time = now.duration_since(epoch).as_secs_f64();
            if let Some((frame, pose)) =
                pair.and_then(|(frame, bracket)| Some((frame, bracket.pose()?)))
            {
                let _span = frame.trace.continue_span("planner-depth-fusion");
                let start = Instant::now();
                let wait = start - frame.received;
                if fusion.integrate(frame, pose, timer_time)? {
                    timing.frame = Some((start, wait));
                    timing.integrate = start.elapsed();
                }
            }
            if fusion.map.timed_out_at(timer_time)? {
                return Err(invalid("depth/pose fusion timed out; restart planner"));
            }
        }
        let maintenance_due = (fusion.options.fading_time > 0.
            || fusion.options.occupied_max_age.is_some())
            && timer_due(&mut next_fading, FADING_PERIOD, now);
        if maintenance_due {
            let start = Instant::now();
            fusion.map.fade();
            timing.maintenance = start.elapsed();
        }
        if timing.frame.is_none() && !maintenance_due {
            continue;
        }
        let start = Instant::now();
        let snapshot = fusion.snapshot()?;
        let built = Instant::now();
        timing.snapshot = built - start;
        if let Some(snapshot) = snapshot {
            // Drop a superseded large snapshot after releasing the planner's result lock.
            let previous = shared
                .lock()
                .map_err(|_| internal("depth state poisoned"))?
                .latest
                .replace(snapshot);
            drop(previous);
            timing.handoff = built.elapsed();
        }
        if let (Some(port), Some(stats)) = (&diagnostics, fusion.map.last_frame()) {
            timing.publish(port, stats);
        }
    }
    Ok(())
}

#[derive(Default)]
struct UpdateTiming {
    frame: Option<(Instant, Duration)>,
    integrate: Duration,
    maintenance: Duration,
    snapshot: Duration,
    handoff: Duration,
}

impl UpdateTiming {
    fn publish(self, port: &ScalarPublisher, stats: FrameStats) {
        let finished = Instant::now();
        let ms = |d: Duration| d.as_secs_f64() * 1000.;
        let send = |path, values: &[f64]| {
            let mut msg = ScalarMessage::base(stats.measured_at, path);
            msg.scalar_count = values.len() as u32;
            msg.scalars[..values.len()].copy_from_slice(values);
            if let Err(e) = port.publish(msg) {
                log::warn!("publishing {path}: {e}");
            }
        };
        send(
            "map/update_ms",
            &[
                ms(self.integrate),
                ms(self.maintenance),
                ms(self.snapshot),
                ms(self.handoff),
            ],
        );
        if let Some((started, wait)) = self.frame {
            send("map/frame_ms", &[ms(wait), ms(finished - started)]);
            send(
                "map/fusion_ms",
                &[
                    ms(stats.projection),
                    ms(stats.window),
                    ms(stats.raycast),
                    ms(stats.inflation),
                ],
            );
            send(
                "map/work",
                &[
                    stats.projected_rays as f64,
                    stats.voted_voxels as f64,
                    stats.cleared_voxels as f64,
                ],
            );
        }
    }
}

/// Keep the original timer phase, coalescing missed ticks instead of replaying a backlog.
fn timer_due(deadline: &mut Instant, period: Duration, now: Instant) -> bool {
    if now < *deadline {
        return false;
    }
    let remainder = now.duration_since(*deadline).as_nanos() % period.as_nanos();
    *deadline = now
        + period
            .checked_sub(Duration::from_nanos(remainder as u64))
            .expect("timer remainder is smaller than its period");
    true
}

/// Consume the newest bracketed frame and discard older frames, retaining future images.
fn take_pair(
    pending: &mut VecDeque<DepthFrame>,
    poses: &VecDeque<OdomMessage>,
    now: Instant,
    max_pose_gap: f64,
) -> Option<(DepthFrame, PoseBracket)> {
    let first_stamp = poses.front()?.timestamp;
    let timeout = Duration::from_secs_f64(crate::MAP_TIMEOUT);
    while pending.front().is_some_and(|frame| {
        frame.timestamp < first_stamp || now.saturating_duration_since(frame.received) > timeout
    }) {
        pending.pop_front();
    }
    let (index, pose) = pending
        .iter()
        .enumerate()
        .rev()
        .find_map(|(index, frame)| {
            PoseBracket::find(poses, frame.timestamp, max_pose_gap).map(|pose| (index, pose))
        })?;
    let frame = pending.remove(index)?;
    pending.drain(..index);
    Some((frame, pose))
}

struct DepthFrame {
    timestamp: f64,
    received: Instant,
    image: DepthImage,
    trace: TraceContext,
}

impl DepthFrame {
    fn from_message(
        message: &DepthImageMessage,
        map: &DepthMapper,
        scale: f64,
        trace: TraceContext,
    ) -> Result<Self> {
        if message.sensor_id != 0
            || message.width as usize != IMAGE_WIDTH
            || message.height as usize != IMAGE_HEIGHT
        {
            return Err(invalid("depth image does not match the calibrated rig"));
        }
        let image = map.sample_depth(IMAGE_WIDTH, IMAGE_HEIGHT, &message.data, |metres| {
            depth_units(metres, scale)
        })?;
        Ok(Self {
            timestamp: message.timestamp,
            received: Instant::now(),
            image,
            trace,
        })
    }
}

fn depth_units(metres: f32, scale: f64) -> u16 {
    if !metres.is_finite() || metres <= 0. {
        return 0;
    }
    (metres * scale as f32)
        .round_ties_even()
        .clamp(0., f32::from(u16::MAX)) as u16
}

/// Numerical state is private to the mapping worker. Construct before opening IPC.
pub struct DepthFusion {
    map: DepthMapper,
    options: DepthOptions,
    last_fused: Option<(f64, Instant)>,
    sequence: u64,
}

impl DepthFusion {
    pub fn new(grid: &firefly_map::GridMap, options: DepthOptions, clearance: f64) -> Result<Self> {
        let parameters = options.parameters(grid, clearance)?;
        Ok(Self {
            map: DepthMapper::new(parameters, options.extrinsic()?, grid)?,
            options,
            last_fused: None,
            sequence: 0,
        })
    }

    fn integrate(
        &mut self,
        frame: DepthFrame,
        pose: Isometry3<f64>,
        timer_time: f64,
    ) -> Result<bool> {
        let update = self
            .map
            .integrate(frame.image, pose, frame.timestamp, timer_time)?;
        if update == FrameUpdate::Observed {
            self.last_fused = Some((frame.timestamp, frame.received));
        }
        Ok(update != FrameUpdate::Stale)
    }

    #[fastrace::trace]
    fn snapshot(&mut self) -> Result<Option<LocalSnapshot>> {
        let Some((timestamp, fused_at)) = self.last_fused else {
            return Ok(None);
        };
        let Some(snapshot) = self.map.snapshot()? else {
            return Ok(None);
        };
        self.sequence += 1;
        Ok(Some(LocalSnapshot {
            map: snapshot,
            sequence: self.sequence,
            last_observed_at: timestamp,
            fused_at,
        }))
    }
}

fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message)
}

fn internal(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::Internal, message.into())
}

#[cfg(test)]
mod tests;
