use crate::{RUNNING, matrix};
use firefly_pubsub::sticks::{STICKS_BUFFER_SIZE, STICKS_TOPIC, SticksMessage};
use firefly_pubsub::subscriber::Subscriber;

use firefly_pubsub::event::TopicNotifier;
use firefly_pubsub::imu::{IMU_SUBSCRIPTION, IMU_TOPIC, ImuMessage};
use firefly_pubsub::node::create_node;
use firefly_pubsub::publish::Publisher;
use iceoryx2_bb_posix::clock::{ClockType, Time};
use rustix::termios::{self, OptionalActions, SpecialCodeIndex};

use std::fs::OpenOptions;
use std::io::{self, Read, Write};
use std::path::Path;
use std::sync::atomic::Ordering;
use std::thread;
use std::time::{Duration, Instant};

enum ImuStream {
    Waiting { since: Instant },
    Receiving { last_frame: Instant },
}

impl ImuStream {
    fn stalled(&self) -> bool {
        let since = match self {
            Self::Waiting { since } => since,
            Self::Receiving { last_frame } => last_frame,
        };
        since.elapsed() > Duration::from_secs(2)
    }
}

fn session(
    device: &Path,
    sticks: &Subscriber<SticksMessage>,
    mut publish: impl FnMut(ImuMessage) -> io::Result<()>,
) -> io::Result<()> {
    let mut port = OpenOptions::new().read(true).write(true).open(device)?;
    termios::ioctl_tiocexcl(&port)?;
    let mut mode = termios::tcgetattr(&port)?;
    mode.make_raw();
    mode.special_codes[SpecialCodeIndex::VMIN] = 0;
    mode.special_codes[SpecialCodeIndex::VTIME] = 1;
    termios::tcsetattr(&port, OptionalActions::Flush, &mode)?;
    port.write_all(matrix::ENTER)?;
    let start = Instant::now();
    let mut scratch = [0; 2048];
    let mut buffer = Vec::with_capacity(scratch.len() + 20);
    loop {
        let n = port.read(&mut scratch)?;
        buffer.extend_from_slice(&scratch[..n]);
        if let Some(end) = matrix::ack_end(&buffer)? {
            buffer.drain(..end);
            break;
        }
        let consumed = buffer.len().saturating_sub(matrix::ACK.len() - 1);
        buffer.drain(..consumed);
        if start.elapsed() > Duration::from_secs(2) || !RUNNING.load(Ordering::Relaxed) {
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "Matrix handshake timed out",
            ));
        }
    }
    let mut clock = matrix::SampleClock::default();
    let mut samples = Vec::with_capacity(scratch.len() / 20 + 1);
    let mut imu = ImuStream::Waiting {
        since: Instant::now(),
    };
    let mut heartbeat = Instant::now();
    let session_start = monotonic_seconds()?;
    let mut last_sent = session_start;
    let mut next_command = Instant::now();
    let mut commands = 0u64;
    log::info!("Matrix USB session ready");
    while RUNNING.load(Ordering::Relaxed) {
        if Instant::now() >= heartbeat {
            port.write_all(matrix::HEARTBEAT)?;
            heartbeat = Instant::now() + Duration::from_millis(250);
        }
        if Instant::now() >= next_command {
            let mut command = None;
            while let Some(sample) = sticks.receive().map_err(io::Error::other)? {
                if sample.valid() && sample.state_time > last_sent {
                    command = Some(*sample);
                }
            }
            if let Some(value) = command
                && matches!(imu, ImuStream::Receiving { .. })
                && (0.0..0.1).contains(&(monotonic_seconds()? - value.state_time))
            {
                port.write_all(&matrix::control(value.channels))?;
                last_sent = value.state_time;
                commands += 1;
            }
            next_command = Instant::now() + Duration::from_millis(20);
        }
        matrix::decode(&mut buffer, &mut samples);
        if let Some(newest) = samples.last() {
            let tick = newest.tick;
            let arrival = monotonic_seconds()?;
            for sample in samples.drain(..) {
                let age = f64::from(tick.wrapping_sub(sample.tick)) * 1e-6;
                publish(sample.message(clock.align(sample.tick, arrival - age)?))?;
            }
            imu = ImuStream::Receiving {
                last_frame: Instant::now(),
            };
        }
        if imu.stalled() {
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "Matrix IMU stream stalled",
            ));
        }
        match port.read(&mut scratch) {
            Ok(n) => buffer.extend_from_slice(&scratch[..n]),
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e),
        }
    }
    log::info!("Matrix session closed: {commands} stick frames sent");
    Ok(())
}

fn monotonic_seconds() -> io::Result<f64> {
    Time::now_with_clock(ClockType::Monotonic)
        .map(|t| t.as_duration().as_secs_f64())
        .map_err(io::Error::other)
}

pub fn run(device: &Path) -> firefly_error::Result<()> {
    let node = create_node()?;
    let sticks = Subscriber::with_topic_and_buffer(&node, STICKS_TOPIC, STICKS_BUFFER_SIZE)?;
    let publisher = Publisher::with_topic_and_buffer(
        &node,
        IMU_TOPIC,
        IMU_SUBSCRIPTION.service_max_buffer_size,
    )?;
    let notify = TopicNotifier::with_topic(&node, IMU_TOPIC)?;
    while RUNNING.load(Ordering::Relaxed) {
        if let Err(error) = session(device, &sticks, |msg| {
            publisher.publish(msg).map_err(io::Error::other)?;
            notify.notify_sent_sample();
            Ok(())
        }) {
            log::warn!("{}: {error}", device.display());
            thread::sleep(Duration::from_millis(1250));
        }
    }
    Ok(())
}
