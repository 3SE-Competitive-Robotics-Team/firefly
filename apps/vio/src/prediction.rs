//! 独立调度的 VIO 高频输出；只传播后验副本，不修改滤波器。
use std::collections::VecDeque;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use firefly_error::{Error, ErrorKind, Result};
use firefly_pubsub::{
    attitude::{ATTITUDE_TOPIC, AttitudeMessage},
    imu::{IMU_SUBSCRIPTION, IMU_TOPIC, ImuMessage},
    node::create_node,
    odom::{ODOM_TOPIC, OdomMessage},
    publish::Publisher,
    subscriber::Subscriber,
};
use firefly_vio::vio_manager::VioManager;
use firefly_vio_core::{
    imu_model::ImuCalibration,
    noise::ImuNoise,
    propagation::{FastInit, Propagator},
    sensor::ImuData,
};
use firefly_vio_types::quat_ops::quat_2_rot;
use nalgebra::Vector3;

const HISTORY_CAPACITY: usize = 4096;
const MAX_IMU_GAP: f64 = 0.05;
const ODOM_PERIOD: f64 = 0.01;
/// 无视觉校正时高频位置输出的最大预测跨度（秒）。
const ODOM_PREDICTION_HORIZON: f64 = 0.5;

struct Prediction {
    attitude: AttitudeMessage,
    odometry: Option<OdomMessage>,
}

struct Posterior {
    initial: FastInit,
    calibration: ImuCalibration,
}

struct Predictor {
    propagator: Propagator,
    times: VecDeque<f64>,
    posterior: Option<Posterior>,
    pending: Option<Posterior>,
    last_output: f64,
    session: u64,
}

impl Predictor {
    fn new(noise: ImuNoise, session: u64) -> Self {
        Self {
            propagator: Propagator::new(noise),
            times: VecDeque::with_capacity(HISTORY_CAPACITY),
            posterior: None,
            pending: None,
            last_output: f64::NEG_INFINITY,
            session,
        }
    }

    fn observe(&mut self, sample: ImuData) -> Result<()> {
        if !sample.timestamp.is_finite()
            || sample.timestamp < 0.
            || !sample
                .wm
                .iter()
                .chain(sample.am.iter())
                .all(|x| x.is_finite())
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "invalid prediction IMU",
            ));
        }
        if let Some(last) = self.times.back() {
            if sample.timestamp <= *last {
                return Ok(());
            }
            if sample.timestamp - last > MAX_IMU_GAP + 1e-9 {
                if self.posterior.is_some() {
                    return Err(Error::new(ErrorKind::Timeout, "VIO prediction IMU gap"));
                }
                self.times.clear();
                self.propagator.clean_old_imu_measurements(sample.timestamp);
            }
        }
        if self.times.len() == HISTORY_CAPACITY {
            self.times.pop_front();
        }
        self.times.push_back(sample.timestamp);
        self.propagator.feed_imu(sample, -1.);
        self.propagator.clean_old_imu_measurements(self.times[0]);
        Ok(())
    }

    fn sample(&mut self) -> Option<Prediction> {
        let time = *self.times.back()?;
        if self
            .pending
            .as_ref()
            .is_some_and(|p| p.initial.time <= time)
        {
            let next = self.pending.take()?;
            if next.initial.time >= self.times[0]
                && self
                    .posterior
                    .as_ref()
                    .is_none_or(|p| next.initial.time >= p.initial.time)
            {
                self.posterior = Some(next);
                self.propagator.invalidate_cache();
            }
        }
        if time <= self.last_output {
            return None;
        }
        let posterior = self.posterior.as_ref()?;
        let predicted = self.propagator.fast_state_propagate(
            Some(&posterior.initial),
            time,
            &posterior.calibration,
        )?;
        self.last_output = time;
        let velocity = quat_2_rot(&predicted.q).transpose() * predicted.v;
        Some(Prediction {
            odometry: (time - posterior.initial.time <= ODOM_PREDICTION_HORIZON).then_some(
                OdomMessage {
                    timestamp: time,
                    position_x: predicted.p.x,
                    position_y: predicted.p.y,
                    position_z: predicted.p.z,
                    velocity_x: velocity.x,
                    velocity_y: velocity.y,
                    velocity_z: velocity.z,
                    quat_x: predicted.q[0],
                    quat_y: predicted.q[1],
                    quat_z: predicted.q[2],
                    quat_w: predicted.q[3],
                    is_initialized: true,
                },
            ),
            attitude: AttitudeMessage {
                timestamp: time,
                posterior_time: posterior.initial.time,
                session: self.session,
                quat_xyzw: predicted.q.into(),
                angular_velocity: predicted.w.into(),
                gyro_bias: posterior.initial.est[10..13].try_into().ok()?,
                attitude_variance: std::array::from_fn(|i| predicted.covariance[(i, i)]),
            },
        })
    }
}

pub struct Worker {
    pending: Arc<Mutex<Option<Posterior>>>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<Result<()>>>,
}

impl Worker {
    pub fn spawn(vio: &VioManager, session: u64) -> Result<Self> {
        let pending = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let input = pending.clone();
        let stopping = stop.clone();
        let noise = vio.params.imu_noises.clone();
        let handle = thread::Builder::new()
            .name("vio-prediction".into())
            .spawn(move || {
                let result = run(&input, &stopping, noise, session);
                stopping.store(true, Ordering::Relaxed);
                result
            })
            .map_err(|e| Error::new(ErrorKind::Internal, e.to_string()))?;
        Ok(Self {
            pending,
            stop,
            handle: Some(handle),
        })
    }

    pub fn update(&self, vio: &VioManager) {
        if vio.initialized() {
            *self.pending.lock().expect("prediction mailbox poisoned") = Some(Posterior {
                initial: vio.fast_state_snapshot(),
                calibration: vio.state.imu_calibration(),
            });
        }
    }

    pub fn stopped(&self) -> bool {
        self.stop.load(Ordering::Relaxed)
            || self.handle.as_ref().is_some_and(JoinHandle::is_finished)
    }

    pub fn finish(&mut self) -> Result<()> {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            handle
                .join()
                .map_err(|_| Error::new(ErrorKind::Internal, "prediction worker panicked"))??;
        }
        Ok(())
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        if let Err(error) = self.finish() {
            log::error!("VIO prediction shutdown: {error}");
        }
    }
}

fn run(
    pending: &Mutex<Option<Posterior>>,
    stop: &AtomicBool,
    noise: ImuNoise,
    session: u64,
) -> Result<()> {
    let node = create_node()?;
    let imu = Subscriber::<ImuMessage>::with_options(&node, IMU_TOPIC, IMU_SUBSCRIPTION)?;
    let output = Publisher::<AttitudeMessage>::with_topic(&node, ATTITUDE_TOPIC)?;
    let odometry = Publisher::<OdomMessage>::with_topic_notify(&node, ODOM_TOPIC)?;
    let mut next_odom = f64::NEG_INFINITY;
    let mut predictor = Predictor::new(noise, session);
    while !stop.load(Ordering::Relaxed) {
        while let Some(m) = imu.receive()? {
            predictor.observe(ImuData {
                timestamp: m.timestamp,
                wm: Vector3::new(
                    m.angular_velocity_x,
                    m.angular_velocity_y,
                    m.angular_velocity_z,
                ),
                am: Vector3::new(
                    m.linear_acceleration_x,
                    m.linear_acceleration_y,
                    m.linear_acceleration_z,
                ),
            })?;
        }
        if let Some(posterior) = pending.lock().expect("prediction mailbox poisoned").take() {
            predictor.pending = Some(posterior);
        }
        if let Some(message) = predictor.sample() {
            output.publish(message.attitude)?;
            if let Some(message) = message.odometry
                && message.timestamp + 1e-9 >= next_odom
            {
                odometry.publish(message)?;
                next_odom = message.timestamp + ODOM_PERIOD;
            }
        }
        if node.wait(Duration::from_millis(1)).is_err() {
            break;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{DMatrix, Matrix3, Quaternion, UnitQuaternion};

    fn posterior(time: f64, yaw: f64, bias: f64) -> Posterior {
        let mut est = [0.; 16];
        est[..4].copy_from_slice(
            UnitQuaternion::from_euler_angles(0., 0., yaw)
                .coords
                .as_slice(),
        );
        est[12] = bias;
        Posterior {
            initial: FastInit {
                time,
                t_off: 0.,
                est,
                covariance: DMatrix::identity(15, 15) * 0.01,
            },
            calibration: ImuCalibration {
                bias_a: Vector3::zeros(),
                bias_g: Vector3::z() * bias,
                r_acc_to_imu: Matrix3::identity(),
                r_gyro_to_imu: Matrix3::identity(),
                da: Matrix3::identity(),
                dw: Matrix3::identity(),
                tg: Matrix3::zeros(),
            },
        }
    }
    fn measured(time: f64) -> ImuData {
        ImuData {
            timestamp: time,
            wm: Vector3::z() * 0.6,
            am: Vector3::z() * 9.81,
        }
    }
    fn yaw(message: AttitudeMessage) -> f64 {
        let [x, y, z, w] = message.quat_xyzw;
        UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))
            .euler_angles()
            .2
    }

    #[test]
    fn delayed_posterior_replays_then_replaces_prediction_without_fusion() {
        let mut prediction = Predictor::new(ImuNoise::default(), 7);
        for i in 0..=100 {
            prediction.observe(measured(f64::from(i) * 0.001)).unwrap();
        }
        assert!(prediction.sample().is_none());
        prediction.pending = Some(posterior(0.035, 0.4, 0.1));
        let first = prediction.sample().unwrap().attitude;
        assert!((yaw(first) - (0.4 + 0.5 * 0.065)).abs() < 1e-12);
        assert!((first.angular_velocity[2] - 0.5).abs() < 1e-12);
        assert!((first.posterior_time - 0.035).abs() < 1e-12);
        assert!(first.attitude_variance[2] > 0.01);
        assert!(prediction.sample().is_none());
        // 后验变化可以包含大角度修正；输出必须从新后验重新传播。
        prediction.pending = Some(posterior(0.06, -0.7, 0.2));
        prediction.observe(measured(0.101)).unwrap();
        let next = prediction.sample().unwrap().attitude;
        assert!((yaw(next) - (-0.7 + 0.4 * 0.041)).abs() < 1e-12);
        assert!((next.angular_velocity[2] - 0.4).abs() < 1e-12);
        prediction.observe(measured(0.102)).unwrap();
        let cached = prediction.sample().unwrap().attitude;
        assert!((yaw(cached) - (-0.7 + 0.4 * 0.042)).abs() < 1e-12);
        assert_eq!(cached.session, 7);
        assert!((cached.posterior_time - 0.06).abs() < 1e-12);
    }

    #[test]
    fn odometry_continues_from_measured_imu_while_visual_update_is_busy() {
        let mut prediction = Predictor::new(ImuNoise::default(), 7);
        let mut initial = posterior(0., std::f64::consts::FRAC_PI_2, 0.6);
        initial.initial.est[4] = 2.;
        initial.initial.est[7] = 1.;
        prediction.pending = Some(initial);
        for i in 0..=400 {
            let time = f64::from(i) * 0.001;
            prediction.observe(measured(time)).unwrap();
            if i == 0 {
                assert!(prediction.sample().is_none());
                continue;
            }
            let output = prediction.sample().unwrap();
            let odom = output.odometry.unwrap();
            assert_eq!(odom.timestamp, output.attitude.timestamp);
            assert_eq!(output.attitude.posterior_time, 0.);
            assert!((odom.position_x - (2. + time)).abs() < 1e-10);
            assert!(odom.position_y.abs() < 1e-10);
            assert!((odom.velocity_x - 1.).abs() < 1e-10);
            assert!(odom.velocity_y.abs() < 1e-10);
        }
        assert!(prediction.sample().is_none());
        let mut corrected = posterior(0.3, 0., 0.6);
        corrected.initial.est[4] = 3.;
        prediction.pending = Some(corrected);
        prediction.observe(measured(0.401)).unwrap();
        let output = prediction.sample().unwrap();
        assert!((output.odometry.unwrap().position_x - 3.).abs() < 1e-10);
        assert!(output.odometry.unwrap().velocity_x.abs() < 1e-10);
        assert_eq!(output.attitude.posterior_time, 0.3);
        for i in 402..=801 {
            prediction.observe(measured(f64::from(i) * 0.001)).unwrap();
            let output = prediction.sample().unwrap();
            assert_eq!(output.odometry.is_some(), i <= 800);
            assert_eq!(output.attitude.posterior_time, 0.3);
        }
    }

    #[test]
    fn coverage_order_and_gap_guard_prediction() {
        let mut prediction = Predictor::new(ImuNoise::default(), 7);
        prediction.observe(measured(0.1)).unwrap();
        prediction.pending = Some(posterior(0.11, 0., 0.));
        assert!(prediction.sample().is_none());
        prediction.observe(measured(0.12)).unwrap();
        assert!(prediction.sample().is_some());
        prediction.observe(measured(0.12)).unwrap();
        prediction.observe(measured(0.11)).unwrap();
        assert!(prediction.sample().is_none());
        prediction.pending = Some(posterior(0.05, 2., 0.));
        prediction.observe(measured(0.13)).unwrap();
        assert!((yaw(prediction.sample().unwrap().attitude) - 0.012).abs() < 1e-12);
        assert!(prediction.observe(measured(0.2)).is_err());
        assert!(prediction.observe(measured(f64::NAN)).is_err());
    }

    #[test]
    fn history_is_bounded_and_cannot_initialize_from_uncovered_posterior() {
        let mut prediction = Predictor::new(ImuNoise::default(), 7);
        for i in 0..=HISTORY_CAPACITY {
            prediction.observe(measured(i as f64 * 0.001)).unwrap();
        }
        assert_eq!(prediction.times.len(), HISTORY_CAPACITY);
        prediction.pending = Some(posterior(0., 0., 0.));
        assert!(prediction.sample().is_none());
    }
}
