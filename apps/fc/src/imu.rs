//! FC 专用 IMU 线程：端口属于工作线程，控制线程只读取非阻塞内存快照。
use firefly_error::{Error, ErrorKind, Result};
use firefly_imu::{Aid, AttitudeState, Estimator, Matrix6, Options, Sample};
use firefly_pubsub::{
    attitude::{ATTITUDE_AID_TOPIC, AttitudeAidMessage},
    imu::ImuSubscriber,
    node::create_node,
    subscriber::Subscriber,
};
use glam::{Quat, Vec3};
use nalgebra::Vector3;
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
    pub rotation: Quat,
    pub rate: Vec3,
    pub ready: bool,
    pub aligned: bool,
    pub nis: f64,
    pub accel_accepted: bool,
    pub bias: [f64; 3],
    pub aid_time: Option<f64>,
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
                    s.aligned = false;
                }
            })
            .map_err(|e| {
                Error::new(ErrorKind::Internal, e.to_string())
                    .with_context("operation", "spawn FC IMU worker")
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
fn to_aid(m: &AttitudeAidMessage) -> Result<Aid> {
    let pose = firefly_base::RigidTransform::from_parts(
        firefly_base::FrameId::ODOM,
        firefly_base::FrameId::BODY,
        [0.; 3],
        m.quat_xyzw,
    )?;
    Ok(Aid {
        timestamp: m.timestamp,
        session: m.session,
        state: AttitudeState {
            rotation: pose.isometry().rotation,
            bias: Vector3::from(m.gyro_bias),
            covariance: Matrix6::from_row_slice(&m.covariance),
        },
    })
}
fn vector(v: Vector3<f64>) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}
#[allow(clippy::too_many_lines)]
fn run(
    options: Options,
    output: &Mutex<Option<Snapshot>>,
    grounded: &AtomicBool,
    stop: &AtomicBool,
) -> Result<()> {
    let node = create_node().map_err(|e| e.with_context("operation", "FC IMU node"))?;
    let imu = ImuSubscriber::new(&node)?;
    let aids = Subscriber::<AttitudeAidMessage>::with_topic(&node, ATTITUDE_AID_TOPIC)?;
    let mut estimator = Estimator::new(options)?;
    let mut latest = None;
    let mut pending = None;
    let mut count = 0u64;
    let mut rejected = false;
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
        while let Some(msg) = aids.receive()? {
            pending = Some(*msg);
        }
        if let Some(msg) = pending
            && latest.is_some_and(|(s, _)| msg.timestamp <= s.timestamp)
        {
            pending = None;
            match to_aid(&msg).and_then(|aid| estimator.aid(&aid)) {
                Ok(_) => {}
                Err(error) => log::warn!("VIO 姿态外援拒绝: {error}"),
            }
            changed = true;
        }
        if changed && let Some((sample, received)) = latest {
            let state = estimator.state();
            let rotation = state.map_or(Quat::IDENTITY, |s| {
                let q = s.rotation.quaternion();
                Quat::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32)
            });
            let bias = state.map_or([0.; 3], |s| [s.bias.x, s.bias.y, s.bias.z]);
            if let Ok(mut value) = output.lock() {
                *value = Some(Snapshot {
                    timestamp: sample.timestamp,
                    received,
                    count,
                    rotation,
                    rate: vector(estimator.rate()),
                    ready: state.is_some(),
                    aligned: estimator.aligned(),
                    nis: estimator.gate().nis,
                    accel_accepted: estimator.gate().accepted,
                    bias,
                    aid_time: estimator.last_aid(),
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
