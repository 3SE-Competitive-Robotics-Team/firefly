//! One IPC owner copies loans once into a bounded writer channel.
use std::collections::BTreeMap;
use std::mem::size_of;
use std::path::Path;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, AtomicUsize, Ordering},
    mpsc,
};
use std::time::{Duration, Instant};

use firefly_error::{Error, ErrorKind, Result};
use firefly_pubsub::{
    attitude::{ATTITUDE_TOPIC, AttitudeMessage},
    camera::{CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, GrayImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH},
    log::{LOG_BUFFER_SIZE, LOG_MAX_PUBLISHERS, LOG_TOPIC, LogMessage},
    node::{IpcNode, create_node},
    odom::{CORRECTED_ODOM_TOPIC, GROUND_TRUTH_TOPIC, OdomMessage},
    subscriber::{Subscriber, SubscriberOptions},
    viz::{
        SCALAR_BUFFER_SIZE, SCALAR_TOPIC, ScalarMessage, VIZ_BUFFER_SIZE, VIZ_MAX_PUBLISHERS,
        VIZ_TOPIC, VOXEL_BUFFER_SIZE, VOXEL_TOPIC, VizMessage, VoxelMessage,
    },
};
use re_sdk_types::archetypes as rr;

use crate::{
    archive::{Archive, wall_time_ns},
    config::Options,
    failure, invalid, schema,
};

enum Event {
    Scalar(ScalarMessage),
    Geometry(Box<VizMessage>),
    Voxel(Box<VoxelMessage>),
    Log(Box<LogMessage>),
    Odom(&'static str, OdomMessage),
    Attitude(AttitudeMessage),
    Image(Box<GrayImageMessage>),
}

impl Event {
    fn bytes(&self) -> usize {
        size_of::<Self>()
            + match self {
                Self::Geometry(_) => size_of::<VizMessage>(),
                Self::Voxel(_) => size_of::<VoxelMessage>(),
                Self::Log(_) => size_of::<LogMessage>(),
                Self::Image(_) => size_of::<GrayImageMessage>(),
                _ => 0,
            }
    }

    fn write(self, archive: &mut Archive) -> Result<()> {
        match self {
            Self::Scalar(msg) => archive.scalar(
                schema::entity(&msg.entity, msg.entity_len)?,
                msg.timestamp,
                msg.scalars
                    .get(..msg.scalar_count as usize)
                    .ok_or_else(|| invalid("scalar count"))?,
            ),
            Self::Geometry(msg) => archive.geometry(&msg),
            Self::Voxel(msg) => archive.voxel(&msg),
            Self::Log(msg) => {
                if !msg.sim_time.is_finite() || msg.wall_nanos >= 1_000_000_000 {
                    return Err(invalid("log timestamp"));
                }
                let tag = schema::entity(&msg.tag, u32::from(msg.tag_len))?;
                let text = msg
                    .text
                    .get(..msg.text_len as usize)
                    .ok_or_else(|| invalid("log text length"))?;
                let text = std::str::from_utf8(text).map_err(|e| invalid(e.to_string()))?;
                let level = match msg.log_level {
                    1 => "ERROR",
                    2 => "WARN",
                    3 => "INFO",
                    4 => "DEBUG",
                    5 => "TRACE",
                    _ => return Err(invalid("log level")),
                };
                let wall_ns = msg
                    .wall_secs
                    .checked_mul(1_000_000_000)
                    .and_then(|v| v.checked_add(i64::from(msg.wall_nanos)))
                    .ok_or_else(|| invalid("log wall time"))?;
                archive.log(tag, msg.sim_time, wall_ns, text, level)
            }
            Self::Odom(name, msg) => {
                if !msg.is_initialized {
                    return Ok(());
                }
                archive.pose(
                    name,
                    msg.timestamp,
                    [msg.position_x, msg.position_y, msg.position_z],
                    [msg.quat_x, msg.quat_y, msg.quat_z, msg.quat_w],
                )
            }
            Self::Attitude(msg) => archive.scalar("okvis/gyro_bias", msg.timestamp, &msg.gyro_bias),
            Self::Image(msg) => {
                if msg.width as usize != IMAGE_WIDTH || msg.height as usize != IMAGE_HEIGHT {
                    return Err(invalid("camera dimensions"));
                }
                let camera = usize::try_from(msg.sensor_id).map_err(|_| invalid("camera id"))?;
                archive.image(
                    camera,
                    msg.timestamp,
                    &rr::Image::from_l8(msg.data.to_vec(), [msg.width, msg.height]),
                )
            }
        }
    }
}

struct Sender {
    tx: mpsc::SyncSender<Event>,
    bytes: Arc<AtomicUsize>,
    high_water: Arc<AtomicUsize>,
    limit: usize,
}

impl Sender {
    fn send(&self, event: Event) -> Result<()> {
        let size = event.bytes();
        let previous = self
            .bytes
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |used| {
                used.checked_add(size).filter(|&total| total <= self.limit)
            })
            .map_err(|_| {
                Error::new(
                    ErrorKind::ResourceExhausted,
                    "recorder byte budget exceeded; recording incomplete",
                )
            })?;
        self.high_water
            .fetch_max(previous + size, Ordering::Relaxed);
        if let Err(error) = self.tx.try_send(event) {
            self.bytes.fetch_sub(size, Ordering::AcqRel);
            return Err(failure("enqueue recording", error));
        }
        Ok(())
    }
}

#[derive(Default)]
struct Continuity(BTreeMap<u128, u64>);

impl Continuity {
    fn observe(&mut self, publisher: u128, sequence: u64, maximum: usize) -> Result<()> {
        if let Some(previous) = self.0.get(&publisher) {
            if sequence != previous.wrapping_add(1) {
                return Err(invalid(format!(
                    "diagnostic sequence gap: {previous} -> {sequence}"
                )));
            }
        } else if self.0.len() >= maximum {
            return Err(invalid("publisher budget exceeded"));
        }
        self.0.insert(publisher, sequence);
        Ok(())
    }
}

fn buffered<T: std::fmt::Debug + iceoryx2::prelude::ZeroCopySend>(
    node: &IpcNode,
    topic: &str,
    capacity: usize,
    publishers: usize,
) -> Result<Subscriber<T>> {
    Subscriber::with_options(
        node,
        topic,
        SubscriberOptions {
            buffer_size: capacity,
            service_max_buffer_size: Some(capacity),
            max_publishers: Some(publishers),
        },
    )
}

fn receive(options: &Options, sender: &Sender, stop: &AtomicBool) -> Result<()> {
    let node = create_node()?;
    let scalars =
        buffered::<ScalarMessage>(&node, SCALAR_TOPIC, SCALAR_BUFFER_SIZE, VIZ_MAX_PUBLISHERS)?;
    let geometry = buffered::<VizMessage>(&node, VIZ_TOPIC, VIZ_BUFFER_SIZE, VIZ_MAX_PUBLISHERS)?;
    let voxels = buffered::<VoxelMessage>(&node, VOXEL_TOPIC, VOXEL_BUFFER_SIZE, 1)?;
    let logs = buffered::<LogMessage>(&node, LOG_TOPIC, LOG_BUFFER_SIZE, LOG_MAX_PUBLISHERS)?;
    let gt = Subscriber::<OdomMessage>::with_topic(&node, GROUND_TRUTH_TOPIC)?;
    let map = Subscriber::<OdomMessage>::with_topic(&node, CORRECTED_ODOM_TOPIC)?;
    let attitude = Subscriber::<AttitudeMessage>::with_topic(&node, ATTITUDE_TOPIC)?;
    let cameras = if options.images {
        vec![
            Subscriber::<GrayImageMessage>::with_topic(&node, CAMERA_LEFT_TOPIC)?,
            Subscriber::<GrayImageMessage>::with_topic(&node, CAMERA_RIGHT_TOPIC)?,
        ]
    } else {
        Vec::new()
    };
    let mut continuity = Continuity::default();
    let mut last_attitude = f64::NEG_INFINITY;
    log::info!("mimir subscriptions ready");
    loop {
        let stopping = stop.load(Ordering::Acquire);
        // One final bounded sweep drains messages already delivered when shutdown began.
        for _ in 0..VIZ_BUFFER_SIZE {
            let Some(sample) = scalars.receive()? else {
                break;
            };
            continuity.observe(
                sample.header().publisher_id().value(),
                sample.sequence,
                options.max_publishers,
            )?;
            sender.send(Event::Scalar(*sample))?;
        }
        for _ in 0..VIZ_BUFFER_SIZE {
            let Some(sample) = geometry.receive()? else {
                break;
            };
            continuity.observe(
                sample.header().publisher_id().value(),
                sample.sequence,
                options.max_publishers,
            )?;
            sender.send(Event::Geometry(Box::new(*sample)))?;
        }
        for _ in 0..VOXEL_BUFFER_SIZE {
            let Some(sample) = voxels.receive()? else {
                break;
            };
            continuity.observe(
                sample.header().publisher_id().value(),
                sample.sequence,
                options.max_publishers,
            )?;
            sender.send(Event::Voxel(Box::new(*sample)))?;
        }
        for _ in 0..LOG_BUFFER_SIZE {
            let Some(sample) = logs.receive()? else {
                break;
            };
            continuity.observe(
                sample.header().publisher_id().value(),
                sample.sequence,
                options.max_publishers,
            )?;
            if sample.dropped_records != 0 {
                return Err(invalid(format!(
                    "publisher lost {} log records",
                    sample.dropped_records
                )));
            }
            sender.send(Event::Log(Box::new(*sample)))?;
        }
        for (name, subscriber) in [("gt/pose", &gt), ("okvis/map/pose", &map)] {
            if let Some(sample) = subscriber.receive()? {
                sender.send(Event::Odom(name, *sample))?;
            }
        }
        if let Some(sample) = attitude.receive()?
            && sample.timestamp - last_attitude >= 1.
        {
            last_attitude = sample.timestamp;
            sender.send(Event::Attitude(*sample))?;
        }
        for subscriber in &cameras {
            if let Some(sample) = subscriber.receive()? {
                sender.send(Event::Image(Box::new(*sample)))?;
            }
        }
        if stopping {
            break;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    Ok(())
}

/// Run the recorder until stopped, then drain the owned queue and close the RRD.
/// # Errors
/// IPC, data-contract, queue-budget, writer-thread, or disk failures. Failures are never converted to success.
pub fn run(path: &Path, options: &Options, stop: &Arc<AtomicBool>) -> Result<()> {
    let mut archive = Archive::create(path, options.clone())?;
    archive.log("mimir", -1., wall_time_ns(), "recording started", "INFO")?;
    let (tx, rx) = mpsc::sync_channel(options.queue_messages);
    let bytes = Arc::new(AtomicUsize::new(0));
    let high_water = Arc::new(AtomicUsize::new(0));
    let sender = Sender {
        tx,
        bytes: Arc::clone(&bytes),
        high_water: Arc::clone(&high_water),
        limit: options.queue_bytes,
    };
    let error = Arc::new(Mutex::new(None::<String>));
    let writer_error = Arc::clone(&error);
    let writer_stop = Arc::clone(stop);
    let flush_period = Duration::from_millis(options.flush_ms);
    let writer = std::thread::Builder::new()
        .name("mimir-write".into())
        .spawn(move || {
            let result = (|| -> Result<()> {
                let mut flushed = Instant::now();
                loop {
                    match rx.recv_timeout(flush_period.saturating_sub(flushed.elapsed())) {
                        Ok(event) => {
                            let size = event.bytes();
                            let result = event.write(&mut archive);
                            bytes.fetch_sub(size, Ordering::AcqRel);
                            result?;
                        }
                        Err(mpsc::RecvTimeoutError::Timeout) => {}
                        Err(mpsc::RecvTimeoutError::Disconnected) => break,
                    }
                    if flushed.elapsed() >= flush_period {
                        archive.flush()?;
                        flushed = Instant::now();
                    }
                }
                Ok(())
            })();
            writer_stop.store(true, Ordering::Release);
            let ingress_error = writer_error
                .lock()
                .map_err(|e| failure("read ingress status", e))?
                .clone();
            let own_error = result.as_ref().err().map(ToString::to_string);
            archive.log(
                "mimir",
                -1.,
                wall_time_ns(),
                &format!(
                    "scalar_samples={} chunks={} queue_high_water_bytes={}",
                    archive.samples,
                    archive.chunks,
                    high_water.load(Ordering::Relaxed)
                ),
                "INFO",
            )?;
            let closed = archive.finish(own_error.as_deref().or(ingress_error.as_deref()));
            result.and(closed)
        })
        .map_err(|e| failure("start writer", e))?;
    let received = receive(options, &sender, stop);
    if let Err(value) = &received {
        *error
            .lock()
            .map_err(|e| failure("store ingress status", e))? = Some(value.to_string());
    }
    drop(sender);
    let written = writer
        .join()
        .map_err(|_| failure("join writer", "writer panicked"))?;
    received.and(written)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sequence_gaps_and_publisher_exhaustion_are_errors() {
        let mut state = Continuity::default();
        state.observe(1, 100, 1).unwrap();
        state.observe(1, 101, 1).unwrap();
        assert!(state.observe(1, 103, 1).is_err());
        assert!(state.observe(2, 0, 1).is_err());
    }

    #[test]
    fn full_queue_does_not_hide_loss_or_leak_byte_budget() {
        let (tx, _rx) = mpsc::sync_channel(1);
        let sender = Sender {
            tx,
            bytes: Arc::new(AtomicUsize::new(0)),
            high_water: Arc::new(AtomicUsize::new(0)),
            limit: 1024,
        };
        let event = || Event::Scalar(ScalarMessage::base(1., "fc/debug/altitude"));
        sender.send(event()).unwrap();
        let bytes = sender.bytes.load(Ordering::Relaxed);
        assert!(sender.send(event()).is_err());
        assert_eq!(sender.bytes.load(Ordering::Relaxed), bytes);
    }
}
