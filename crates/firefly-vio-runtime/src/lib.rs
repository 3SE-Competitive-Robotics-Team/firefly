//! One IMU scheduling and publication contract for online SLAM adapters.
mod publish;
pub use publish::{Publishers, Sample, publish_pose_diagnostic, publish_scalar_diagnostic};

use firefly_error::{Error, ErrorKind, Result};
use firefly_imu_propagation::ImuSample;
use firefly_pubsub::{
    event::TopicListener,
    imu::{IMU_SUBSCRIPTION, IMU_TOPIC, ImuMessage},
    node::create_node,
    subscriber::Subscriber,
};
use iceoryx2::prelude::{CallbackProgression, ipc};
use iceoryx2::waitset::{WaitSetBuilder, WaitSetRunResult};
use jiff::Timestamp;
use nalgebra::Vector3;
use std::{
    cell::RefCell,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicI64, Ordering},
    },
    time::Duration,
};

#[derive(Clone)]
pub struct Topics {
    pub imu: String,
    pub odometry: String,
    pub corrected: String,
    pub navigation: String,
    pub attitude: String,
}
impl Default for Topics {
    fn default() -> Self {
        Self {
            imu: IMU_TOPIC.into(),
            odometry: firefly_pubsub::odom::ODOM_TOPIC.into(),
            corrected: firefly_pubsub::odom::CORRECTED_ODOM_TOPIC.into(),
            navigation: firefly_pubsub::navigation::NAVIGATION_TOPIC.into(),
            attitude: firefly_pubsub::attitude::ATTITUDE_TOPIC.into(),
        }
    }
}

pub struct Options {
    pub process: &'static str,
    pub topics: Topics,
    pub diagnostics: bool,
    pub external_localization: bool,
    pub imu_latency_entity: &'static str,
}

/// Latest measurement admitted by the estimator adapter. Images must not pass this watermark.
#[derive(Clone)]
pub struct ImuWatermark(Arc<AtomicI64>);
impl Default for ImuWatermark {
    fn default() -> Self {
        Self(Arc::new(AtomicI64::new(i64::MIN)))
    }
}
impl ImuWatermark {
    pub fn store(&self, time: Timestamp) {
        if let Ok(ns) = i64::try_from(time.as_nanosecond()) {
            self.0.store(ns, Ordering::Release);
        }
    }
    #[must_use]
    pub fn load(&self) -> Option<Timestamp> {
        let ns = self.0.load(Ordering::Acquire);
        (ns != i64::MIN)
            .then(|| Timestamp::from_nanosecond(i128::from(ns)).ok())
            .flatten()
    }
}

/// Algorithm-specific queue, posterior and coordinate conversion only.
pub trait ImuAdapter {
    /// Drain committed posteriors before consuming the next batch of IMU samples.
    /// # Errors
    /// Returns estimator/publication failures.
    fn posteriors(&mut self, publishers: &Publishers) -> Result<()>;
    /// Admit an IMU measurement to the estimator before advancing its watermark.
    /// # Errors
    /// Returns a closed/full estimator ingress error.
    fn enqueue(&mut self, sample: ImuSample) -> Result<()>;
    /// Replay/advance the shared predictor and adapt its coordinates.
    /// # Errors
    /// Returns prediction or coordinate conversion failures.
    fn predict(&mut self, sample: ImuSample) -> Result<Option<Sample>>;
    /// Release adapter-owned outputs/workers even after a processing error.
    /// # Errors
    /// Returns worker shutdown failures.
    fn finish(self) -> Result<()>
    where
        Self: Sized,
    {
        Ok(())
    }
}

struct StopOnDrop<'a>(&'a AtomicBool);
impl Drop for StopOnDrop<'_> {
    fn drop(&mut self) {
        self.0.store(true, Ordering::Release);
    }
}

/// Run on a dedicated thread; all IPC endpoints are created and dropped on that thread.
/// # Errors
/// Returns IPC, adapter, input validation or shutdown failures.
pub fn run_imu<A: ImuAdapter>(
    options: &Options,
    watermark: &ImuWatermark,
    stop: &AtomicBool,
    build: impl FnOnce() -> Result<A>,
) -> Result<()> {
    let _stopping = StopOnDrop(stop);
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, options.process);
    let imu = Subscriber::<ImuMessage>::with_options(&node, &options.topics.imu, IMU_SUBSCRIPTION)?;
    let events = TopicListener::with_topic(&node, &options.topics.imu)?;
    let mut publishers = Publishers::open(&node, options)?;
    let waitset = WaitSetBuilder::new()
        .create::<ipc::Service>()
        .map_err(|e| error(format!("create IMU WaitSet: {e:?}")))?;
    let _notification = waitset
        .attach_notification(&events)
        .map_err(|e| error(format!("attach IMU event: {e:?}")))?;
    let _timeout = waitset
        .attach_interval(Duration::from_millis(2))
        .map_err(|e| error(format!("attach IMU timeout: {e:?}")))?;
    let mut adapter = build()?;
    let failure = RefCell::new(None);
    let mut last_time = None;
    let mut published = 0u64;
    let result = (|| {
        let mut on_event = |_| {
            if stop.load(Ordering::Acquire) {
                return CallbackProgression::Stop;
            }
            let _ = events.drain();
            firefly_observability::pump_log_ipc(&log_ipc);
            let result = (|| {
                adapter.posteriors(&publishers)?;
                while let Some(packet) = imu.receive()? {
                    let trace = *packet.user_header();
                    let _span = trace.continue_span("vio-receive-imu");
                    let (time, sample) = measurement(&packet)?;
                    if last_time.is_some_and(|last| time <= last) {
                        continue;
                    }
                    firefly_observability::set_sim_time(packet.timestamp);
                    last_time = Some(time);
                    adapter.enqueue(sample)?;
                    watermark.store(time);
                    if let Some(output) = adapter.predict(sample)? {
                        publishers.publish(&output, packet.timestamp)?;
                        published += 1;
                        if published.is_multiple_of(10) && trace.send_ts_secs > 0 {
                            let ms = (Timestamp::now().as_nanosecond()
                                - trace.send_timestamp().as_nanosecond())
                                as f64
                                * 1e-6;
                            publishers.ipc(packet.timestamp, ms.max(0.0))?;
                        }
                    }
                }
                Ok(())
            })();
            match result {
                Ok(()) => CallbackProgression::Continue,
                Err(e) => {
                    *failure.borrow_mut() = Some(e);
                    CallbackProgression::Stop
                }
            }
        };
        while !stop.load(Ordering::Acquire) {
            match waitset.wait_and_process(&mut on_event) {
                Ok(WaitSetRunResult::AllEventsHandled) => {}
                Ok(
                    WaitSetRunResult::StopRequest
                    | WaitSetRunResult::Interrupt
                    | WaitSetRunResult::TerminationRequest,
                ) => break,
                Err(e) => return Err(error(format!("IMU WaitSet: {e:?}"))),
            }
        }
        Ok(())
    })();
    firefly_observability::pump_log_ipc(&log_ipc);
    stop.store(true, Ordering::Release);
    let finished = adapter.finish();
    result
        .and(failure.into_inner().map_or(Ok(()), Err))
        .and(finished)
}

fn measurement(packet: &ImuMessage) -> Result<(Timestamp, ImuSample)> {
    let time = sensor_time(packet.timestamp)?;
    let sample = ImuSample {
        timestamp_ns: time.as_nanosecond(),
        gyro: Vector3::new(
            packet.angular_velocity_x,
            packet.angular_velocity_y,
            packet.angular_velocity_z,
        ),
        accel: Vector3::new(
            packet.linear_acceleration_x,
            packet.linear_acceleration_y,
            packet.linear_acceleration_z,
        ),
    };
    if sample
        .gyro
        .iter()
        .chain(sample.accel.iter())
        .any(|v| !v.is_finite())
    {
        return Err(error("nonfinite IMU measurement"));
    }
    Ok((time, sample))
}

/// # Errors
/// Rejects negative, nonfinite and unrepresentable measurement times.
pub fn sensor_time(seconds: f64) -> Result<Timestamp> {
    if !seconds.is_finite() || seconds < 0.0 {
        return Err(error("invalid sensor timestamp"));
    }
    Timestamp::from_nanosecond((seconds * 1e9).round() as i128).map_err(|e| error(e.to_string()))
}

fn error(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::Internal, message).with_operation("vio.imu")
}
