//! Planner-owned EGO depth map. Fusion runs outside the FSM/solver threads.
//! Measurement-time interpolation replaces ROS `ApproximateTime` transport pairing;
//! the numerical callbacks, 32 ms occupancy and 500 ms fading periods stay separate.
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

use firefly_base::FrameId;
use firefly_ego_map::{DepthImage, GridMap};
use firefly_error::{Error, ErrorKind, Result};
use firefly_pubsub::{
    camera::{DEPTH_TOPIC, DepthImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH},
    node::create_node,
    odom::OdomMessage,
    subscriber::Subscriber,
    trace::TraceContext,
};
use nalgebra::{Isometry3, Translation3};

use crate::{
    config::DepthOptions,
    map_input::{Bounds, LocalSnapshot},
    odometry::OdometryHistory,
};

const OCCUPANCY_PERIOD: Duration = Duration::from_millis(32);
const FADING_PERIOD: Duration = Duration::from_millis(500);
const MAX_PENDING_DEPTHS: usize = 8;
const MAX_COLLISION_RUNS: usize = 1_000_000;

#[derive(Default)]
struct Shared {
    latest: Option<LocalSnapshot>,
    error: Option<String>,
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
                        state.error = Some(error.to_string());
                        state.latest = None;
                    }
                    let _ = ready.send(Err(error.to_string()));
                    log::error!("depth mapping receiver: {error}");
                }
                stopping.store(true, Ordering::Release);
            })
            .map_err(|e| internal(e.to_string()))?;
        let mut receiver = Self {
            shared,
            stop,
            thread: Some(thread),
        };
        if let Err(error) = startup.recv().map_err(|e| e.to_string()).and_then(|v| v) {
            receiver.shutdown();
            return Err(internal(error).with_operation("start depth mapping"));
        }
        Ok(receiver)
    }

    pub fn take_snapshot(&self) -> Result<Option<LocalSnapshot>> {
        let mut shared = self
            .shared
            .lock()
            .map_err(|_| internal("depth state poisoned"))?;
        if let Some(error) = &shared.error {
            return Err(internal(error.clone()));
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
    ready: &SyncSender<std::result::Result<(), String>>,
) -> Result<()> {
    let node = create_node()?;
    let subscriber = Subscriber::<DepthImageMessage>::with_topic(&node, DEPTH_TOPIC)?;
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
        let mut changed = false;
        if timer_due(&mut next_occupancy, OCCUPANCY_PERIOD, now) {
            let poses = history.snapshot()?;
            if !poses.is_empty() {
                // The upstream timer treats times below one second as uninitialized.
                let timer_time = 1. + now.duration_since(epoch).as_secs_f64();
                if let Some((frame, pose)) =
                    take_pair(&mut pending, &poses, now, fusion.options.max_pose_gap)
                {
                    let _span = frame.trace.continue_span("planner-depth-fusion");
                    changed = fusion.integrate(frame, pose, timer_time)?;
                } else {
                    fusion.map.update_occupancy_callback(timer_time)?;
                }
                if fusion.map.odom_depth_timeout() {
                    return Err(invalid("depth/pose fusion timed out; restart planner"));
                }
            }
        }
        if fusion.options.fading_time > 0. && timer_due(&mut next_fading, FADING_PERIOD, now) {
            fusion.map.fading_callback();
            changed = true;
        }
        if changed && let Some(snapshot) = fusion.snapshot()? {
            // Drop a superseded large snapshot after releasing the planner's result lock.
            let previous = shared
                .lock()
                .map_err(|_| internal("depth state poisoned"))?
                .latest
                .replace(snapshot);
            drop(previous);
        }
    }
    Ok(())
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
) -> Option<(DepthFrame, Isometry3<f64>)> {
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
            interpolate_pose(poses, frame.timestamp, max_pose_gap).map(|pose| (index, pose))
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
    fn from_message(message: &DepthImageMessage, scale: f64, trace: TraceContext) -> Result<Self> {
        if message.sensor_id != 0
            || message.width as usize != IMAGE_WIDTH
            || message.height as usize != IMAGE_HEIGHT
        {
            return Err(invalid("depth image does not match the calibrated rig"));
        }
        let pixels = message
            .data
            .iter()
            .map(|&metres| depth_units(metres, scale))
            .collect();
        Ok(Self {
            timestamp: message.timestamp,
            received: Instant::now(),
            image: DepthImage::new(IMAGE_WIDTH, IMAGE_HEIGHT, pixels)?,
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

fn interpolate_pose(
    history: &VecDeque<OdomMessage>,
    timestamp: f64,
    max_gap: f64,
) -> Option<Isometry3<f64>> {
    let mut previous: Option<&OdomMessage> = None;
    for message in history {
        let stamp = message.timestamp;
        if stamp == timestamp {
            return Some(*message.body_pose(FrameId::ODOM).ok()?.isometry());
        }
        if stamp > timestamp {
            let before = previous?;
            let gap = stamp - before.timestamp;
            if gap > max_gap || gap <= 0. {
                return None;
            }
            let alpha = (timestamp - before.timestamp) / gap;
            let a = before.body_pose(FrameId::ODOM).ok()?;
            let b = message.body_pose(FrameId::ODOM).ok()?;
            let position = a
                .isometry()
                .translation
                .vector
                .lerp(&b.isometry().translation.vector, alpha);
            let rotation = a.isometry().rotation.slerp(&b.isometry().rotation, alpha);
            return Some(Isometry3::from_parts(
                Translation3::from(position),
                rotation,
            ));
        }
        previous = Some(message);
    }
    None
}

/// Numerical state is private to the mapping worker. Construct before opening IPC.
pub struct DepthFusion {
    map: GridMap,
    options: DepthOptions,
    last_fused: Option<(f64, Instant)>,
    sequence: u64,
}

impl DepthFusion {
    pub fn new(grid: &firefly_map::GridMap, options: DepthOptions, clearance: f64) -> Result<Self> {
        let parameters = options.parameters(grid, clearance)?;
        let mut map = GridMap::new(parameters)?;
        map.extrinsic_callback(options.extrinsic()?)?;
        Ok(Self {
            map,
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
        self.map.depth_odom_callback(frame.image, pose)?;
        let changed = self.map.update_occupancy_callback(timer_time)?;
        if !self.map.projected_points().is_empty() {
            self.last_fused = Some((frame.timestamp, frame.received));
        }
        Ok(changed)
    }

    #[fastrace::trace]
    fn snapshot(&mut self) -> Result<Option<LocalSnapshot>> {
        let Some((timestamp, fused_at)) = self.last_fused else {
            return Ok(None);
        };
        let Some((low, high)) = self.map.bounds() else {
            return Ok(None);
        };
        let resolution = self.map.resolution();
        let bounds = Bounds {
            min: low.map(|v| f64::from(v) * resolution),
            max: high.map(|v| (f64::from(v) + 1.) * resolution),
        };
        self.sequence += 1;
        let mut cells = Vec::new();
        let mut overflow = false;
        self.map.visit_inflated_runs(|low, high| {
            if cells.len() >= MAX_COLLISION_RUNS {
                overflow = true;
                return;
            }
            cells.push(Bounds {
                min: low.map(|v| f64::from(v) * resolution),
                max: high.map(|v| f64::from(v) * resolution),
            });
        });
        if overflow {
            return Err(invalid("local collision snapshot capacity exceeded"));
        }
        Ok(Some(LocalSnapshot {
            cells,
            bounds,
            sequence: self.sequence,
            timestamp,
            fused_at,
        }))
    }
}

/// Local snapshots already contain upstream inflation; never inflate them twice.
pub fn collision_grid(source: &firefly_map::GridMap) -> Result<firefly_map::GridMap> {
    firefly_map::GridMapBuilder::new(source.resolution(), source.dims())
        .with_origin(source.origin())
        .with_obstacles_inflation(0.)
        .build()
}

fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message)
}

fn internal(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::Internal, message.into())
}

#[cfg(test)]
mod tests;
