//! FC 专用 IMU 线程：端口属于工作线程，控制线程只读取非阻塞内存快照。
use firefly_error::{Error, ErrorKind, Result};
use firefly_imu::{Aid, AttitudeState, Estimator, Matrix6, Options, Sample};
use firefly_pubsub::{
    attitude::{ATTITUDE_AID_TOPIC, AttitudeAidMessage},
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
    pub aligned: bool,
    pub nis: f64,
    pub accel_accepted: bool,
    pub bias: [f64; 3],
    pub aid_time: Option<f64>,
    pub aid_gate: firefly_imu::Intersection,
    pub aid_gate_time: Option<f64>,
    pub covariance_diagonal: [f64; 6],
}

impl Snapshot {
    pub fn source(&self, stale_limit: Duration) -> &'static str {
        if !self.ready {
            "等待 IMU"
        } else if !self.aligned {
            "本地 ESKF（未对齐 odom）"
        } else if self
            .aid_time
            .is_none_or(|t| self.timestamp - t > stale_limit.as_secs_f64())
        {
            "CI 后验 IMU 传播（无新鲜外援）"
        } else if !self.aid_gate.accepted {
            "CI 后验 IMU 传播（外援拒绝）"
        } else {
            "本地 ESKF + VIO CI"
        }
    }
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
#[allow(clippy::too_many_lines)]
fn run(
    options: Options,
    output: &Mutex<Option<Snapshot>>,
    grounded: &AtomicBool,
    stop: &AtomicBool,
) -> Result<()> {
    let node = create_node().map_err(|e| e.with_operation("FC IMU node"))?;
    let imu = Subscriber::<ImuMessage>::with_options(&node, IMU_TOPIC, IMU_SUBSCRIPTION)?;
    let aids = Subscriber::<AttitudeAidMessage>::with_topic(&node, ATTITUDE_AID_TOPIC)?;
    let mut estimator = Estimator::new(options)?;
    let mut latest = None;
    let mut pending = None;
    let mut count = 0u64;
    let mut rejected = false;
    let mut aid_rejected = false;
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
                Ok(true) => {
                    if aid_rejected {
                        log::info!("VIO 姿态外援恢复接受，测量时刻 {:.6}", msg.timestamp);
                    }
                    aid_rejected = false;
                }
                Ok(false) => {
                    if !aid_rejected {
                        let gate = (estimator.aid_gate_time() == Some(msg.timestamp))
                            .then(|| estimator.aid_gate());
                        let reason = match gate.and_then(|g| g.rejection) {
                            Some(firefly_imu::IntersectionRejection::RotationDistance) => {
                                "旋转差超过 0.5 rad"
                            }
                            Some(firefly_imu::IntersectionRejection::Innovation) => {
                                "六维创新门未通过"
                            }
                            None => "测量时间未覆盖、重复/乱序或 IMU 不连续",
                        };
                        log::warn!(
                            "VIO 姿态外援拒绝: {reason}，测量时刻 {:.6}，旋转差 {:?} rad，NIS {:?}",
                            msg.timestamp,
                            gate.and_then(|g| g.rotation_distance()),
                            gate.and_then(|g| g.nis)
                        );
                    }
                    aid_rejected = true;
                }
                Err(error) => {
                    if !aid_rejected {
                        log::warn!("VIO 姿态外援拒绝: {error}");
                    }
                    aid_rejected = true;
                }
            }
            changed = true;
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
                    aligned: estimator.aligned(),
                    nis: estimator.gate().nis,
                    accel_accepted: estimator.gate().accepted,
                    bias,
                    aid_time: estimator.last_aid(),
                    aid_gate: estimator.aid_gate(),
                    aid_gate_time: estimator.aid_gate_time(),
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
