//! FC 专用 IMU 线程：端口属于工作线程，控制线程只读取非阻塞内存快照。
use firefly_error::{Error, ErrorKind, Result};
use firefly_imu::{Estimator, Options, Sample};
use firefly_pubsub::{
    imu::{IMU_SUBSCRIPTION, IMU_TOPIC, ImuMessage},
    node::create_node,
    subscriber::Subscriber,
};
use nalgebra::{UnitQuaternion, Vector3};
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

#[derive(Clone, Copy)]
pub struct Snapshot {
    pub timestamp: f64,
    pub received: Instant,
    pub count: u64,
    pub rotation: UnitQuaternion<f32>,
    pub rate: Vector3<f32>,
    pub ready: bool,
    pub nis: f64,
    pub accel_accepted: bool,
    pub bias: [f64; 3],
    pub covariance_diagonal: [f64; 6],
}

pub struct Worker {
    latest: Arc<Mutex<Option<Snapshot>>>,
    pub grounded: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}
impl Worker {
    pub fn spawn(options: Options) -> Result<Self> {
        options.validate()?;
        let latest = Arc::new(Mutex::new(None));
        let grounded = Arc::new(AtomicBool::new(true));
        let stop = Arc::new(AtomicBool::new(false));
        let (output, on_ground, stopping) = (latest.clone(), grounded.clone(), stop.clone());
        let handle = thread::Builder::new()
            .name("fc-imu".into())
            .spawn(move || {
                if let Err(error) = run(options, &output, &on_ground, &stopping) {
                    log::error!("IMU 工作线程退出: {error}");
                    super::RUNNING.store(false, Ordering::Relaxed);
                }
                if let Ok(mut value) = output.lock()
                    && let Some(s) = value.as_mut()
                {
                    s.ready = false;
                }
            })
            .map_err(|e| {
                Error::new(ErrorKind::Internal, e.to_string()).with_operation("spawn FC IMU worker")
            })?;
        Ok(Self {
            latest,
            grounded,
            stop,
            handle: Some(handle),
        })
    }
    pub fn snapshot(&self) -> Option<Snapshot> {
        self.latest.try_lock().ok().and_then(|v| *v)
    }
}
impl Drop for Worker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take()
            && handle.join().is_err()
        {
            log::error!("IMU 工作线程 panic");
        }
    }
}
fn run(
    options: Options,
    output: &Mutex<Option<Snapshot>>,
    grounded: &AtomicBool,
    stop: &AtomicBool,
) -> Result<()> {
    let node = create_node().map_err(|e| e.with_operation("FC IMU node"))?;
    let imu = Subscriber::<ImuMessage>::with_options(&node, IMU_TOPIC, IMU_SUBSCRIPTION).map_err(
        |error| {
            log::error!("FC IMU 订阅失败: {error}");
            error
        },
    )?;
    log::info!("FC IMU 订阅已建立：{IMU_TOPIC}");
    let mut estimator = Estimator::new(options)?;
    let mut latest = None;
    let mut count = 0u64;
    let mut rejected = false;
    let mut last_log = Instant::now();
    while !stop.load(Ordering::Relaxed) && super::RUNNING.load(Ordering::Relaxed) {
        let root = fastrace::Span::root(
            "fc-imu",
            fastrace::prelude::SpanContext::random().sampled(false),
        );
        let _guard = root.set_local_parent();
        let mut changed = false;
        while let Some(msg) = imu.receive()? {
            let sample = Sample {
                timestamp: msg.timestamp,
                gyro: Vector3::new(
                    msg.angular_velocity_x,
                    msg.angular_velocity_y,
                    msg.angular_velocity_z,
                ),
                accel: Vector3::new(
                    msg.linear_acceleration_x,
                    msg.linear_acceleration_y,
                    msg.linear_acceleration_z,
                ),
            };
            match estimator.observe(sample, grounded.load(Ordering::Relaxed)) {
                Ok(true) => {
                    latest = Some((sample, Instant::now()));
                    count += 1;
                    changed = true;
                }
                Ok(false) => {}
                Err(error) => {
                    if !rejected {
                        log::warn!("IMU 样本拒绝: {error}");
                    }
                    rejected = true;
                    changed = true;
                }
            }
        }
        if last_log.elapsed() >= Duration::from_secs(1) {
            log::debug!("FC IMU worker 已收 {count} 条");
            last_log = Instant::now();
        }
        if changed && let Some((sample, received)) = latest {
            let state = estimator.state();
            let rotation = state.map_or(UnitQuaternion::identity(), |s| s.rotation.cast::<f32>());
            let bias = state.map_or([0.; 3], |s| [s.bias.x, s.bias.y, s.bias.z]);
            if let Ok(mut value) = output.lock() {
                *value = Some(Snapshot {
                    timestamp: sample.timestamp,
                    received,
                    count,
                    rotation,
                    rate: estimator.rate().cast::<f32>(),
                    ready: state.is_some(),
                    nis: estimator.gate().nis,
                    accel_accepted: estimator.gate().accepted,
                    bias,
                    covariance_diagonal: state
                        .map_or([0.; 6], |s| std::array::from_fn(|i| s.covariance[(i, i)])),
                });
            }
        }
        if node.wait(Duration::from_millis(1)).is_err() {
            break;
        }
    }
    fastrace::flush();
    Ok(())
}
