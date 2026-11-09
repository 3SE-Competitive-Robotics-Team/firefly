//! Basalt queue and posterior adaptation for the event-driven IMU runtime.
use super::{error, publish_pose_diagnostic, publish_scalar_diagnostic};
use firefly_basalt::imu::imu_types::{ImuData, PoseVelBiasState};
use firefly_error::Result;
use firefly_imu_initialization::{
    GravityEstimate, ImuSample as GravitySample, StationaryGravitySampler,
};
use firefly_imu_propagation::{Bias, ImuSample, Predictor, State};
use firefly_pubsub::{attitude::AttitudeMessage, navigation::NavigationMessage, odom::OdomMessage};
use firefly_vio_runtime::{ImuAdapter, ImuWatermark, Publishers, Sample};
use nalgebra::Vector3;
use std::{
    collections::{BTreeMap, VecDeque},
    sync::{Arc, Mutex, atomic::AtomicBool, mpsc},
    time::Instant,
};

pub(super) type FrameStarts = Arc<Mutex<BTreeMap<i64, Instant>>>;

pub(super) struct Inputs {
    pub external_localization: bool,
    pub imu: mpsc::SyncSender<Option<Arc<ImuData<f64>>>>,
    pub posteriors: mpsc::Receiver<Option<Arc<PoseVelBiasState<f64>>>>,
    pub gravity: mpsc::SyncSender<GravityEstimate>,
    pub frame_started: FrameStarts,
    pub watermark: ImuWatermark,
}

pub(super) fn run(inputs: Inputs, stop: &AtomicBool) -> Result<()> {
    let watermark = inputs.watermark.clone();
    let options = firefly_vio_runtime::Options {
        process: "firefly-rootslam",
        topics: firefly_vio_runtime::Topics::default(),
        diagnostics: true,
        external_localization: inputs.external_localization,
        imu_latency_entity: "rootslam/ipc/imu_publish_ms",
    };
    firefly_vio_runtime::run_imu(&options, &watermark, stop, || {
        Ok(Adapter {
            inputs,
            pending: VecDeque::new(),
            sampler: StationaryGravitySampler::new(),
            initialized: false,
            predictor: Predictor::new(5., Vector3::new(0., 0., -9.81)),
            session: (std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64)
                .max(1),
            alignment: firefly_base::RigidTransform::from_parts(
                firefly_base::FrameId::MAP,
                firefly_base::FrameId::ODOM,
                [0.; 3],
                [0., 0., 0., 1.],
            )?,
        })
    })
}

struct Adapter {
    inputs: Inputs,
    // Basalt starts consuming IMU only after the first initialized image. Preserve
    // that startup backlog without blocking the event thread on its 300-entry queue.
    pending: VecDeque<Arc<ImuData<f64>>>,
    sampler: StationaryGravitySampler,
    initialized: bool,
    predictor: Predictor,
    session: u64,
    alignment: firefly_base::RigidTransform,
}

impl Adapter {
    fn flush(&mut self) -> Result<()> {
        while let Some(data) = self.pending.front() {
            match self.inputs.imu.try_send(Some(Arc::clone(data))) {
                Ok(()) => {
                    self.pending.pop_front();
                }
                Err(mpsc::TrySendError::Full(_)) => break,
                Err(mpsc::TrySendError::Disconnected(_)) => {
                    return Err(error("VIO IMU input closed"));
                }
            }
        }
        Ok(())
    }
}

impl ImuAdapter for Adapter {
    fn posteriors(&mut self, publishers: &Publishers) -> Result<()> {
        self.flush()?;
        loop {
            let state = match self.inputs.posteriors.try_recv() {
                Ok(Some(state)) => state,
                Ok(None) | Err(mpsc::TryRecvError::Disconnected) => {
                    return Err(error("VIO state stream ended"));
                }
                Err(mpsc::TryRecvError::Empty) => break,
            };
            let pose = &state.state.pose.t_w_i;
            let stamp_ns = state.state.pose.t_ns;
            let received = firefly_basalt::utils::time_utils::profile_clock_ns();
            firefly_basalt::utils::time_utils::profile_stage(
                stamp_ns,
                "posterior_receive",
                received,
            );
            let stamp = stamp_ns as f64 * 1e-9;
            let started = self
                .inputs
                .frame_started
                .lock()
                .map_err(|_| error("frame timing mutex poisoned"))?
                .remove(&stamp_ns);
            let accepted = self.predictor.posterior(
                State {
                    timestamp_ns: i128::from(stamp_ns),
                    rotation: pose.rotation.into_inner(),
                    position: pose.translation.vector,
                    velocity: state.state.vel_w_i,
                },
                Bias {
                    gyro: state.bias_gyro,
                    accel: state.bias_accel,
                },
            );
            if let Some(viz) = publishers.viz() {
                if let Some(started) = started {
                    publish_scalar_diagnostic(
                        viz,
                        stamp,
                        "rootslam/frame_processing_ms",
                        [started.elapsed().as_secs_f64() * 1000.],
                    );
                }
                if accepted {
                    let q = pose.rotation.quaternion();
                    publish_pose_diagnostic(
                        viz,
                        stamp,
                        "vio/posterior/pose_imu_world",
                        pose.translation.vector.into(),
                        [q.i, q.j, q.k, q.w],
                    );
                    publish_scalar_diagnostic(
                        viz,
                        stamp,
                        "vio/posterior/velocity_world",
                        state.state.vel_w_i.into(),
                    );
                    publish_scalar_diagnostic(
                        viz,
                        stamp,
                        "vio/posterior/gyro_bias_imu",
                        state.bias_gyro.into(),
                    );
                    publish_scalar_diagnostic(
                        viz,
                        stamp,
                        "vio/posterior/accel_bias_imu",
                        state.bias_accel.into(),
                    );
                }
            }
        }
        Ok(())
    }

    fn enqueue(&mut self, sample: ImuSample) -> Result<()> {
        if !self.initialized
            && let Some(estimate) = self.sampler.push(GravitySample {
                timestamp_ns: sample.timestamp_ns,
                gyro: sample.gyro,
                accel: sample.accel,
            })
        {
            self.inputs
                .gravity
                .send(estimate)
                .map_err(|_| error("gravity receiver stopped"))?;
            self.initialized = true;
            log::info!(
                "RootSLAM VIO gravity initialized from {} stationary IMU samples over {:.3}s",
                estimate.sample_count,
                (estimate.end_ns - estimate.start_ns) as f64 * 1e-9
            );
        }
        if self.pending.len() >= 10_000 {
            return Err(error("VIO IMU backlog exceeded 10 seconds"));
        }
        self.pending.push_back(Arc::new(ImuData {
            t_ns: i64::try_from(sample.timestamp_ns)
                .map_err(|_| error("IMU timestamp overflow"))?,
            gyro: sample.gyro,
            accel: sample.accel,
        }));
        self.flush()
    }

    fn predict(&mut self, sample: ImuSample) -> Result<Option<Sample>> {
        let Some(prediction) = self
            .predictor
            .imu(sample)
            .map_err(|e| error(format!("IMU prediction: {e:?}")))?
        else {
            return Ok(None);
        };
        let state = prediction.state;
        let q = state.rotation;
        let stamp = state.timestamp_ns as f64 * 1e-9;
        let odom = OdomMessage {
            timestamp: stamp,
            position_x: state.position.x,
            position_y: state.position.y,
            position_z: state.position.z,
            velocity_x: state.velocity.x,
            velocity_y: state.velocity.y,
            velocity_z: state.velocity.z,
            quat_x: q.i,
            quat_y: q.j,
            quat_z: q.k,
            quat_w: q.w,
            is_initialized: true,
        };
        Ok(Some(Sample {
            odom,
            corrected: odom,
            navigation: NavigationMessage::new(odom, &self.alignment, 0),
            attitude: Some(AttitudeMessage {
                timestamp: stamp,
                posterior_time: self
                    .predictor
                    .posterior_time_ns()
                    .ok_or_else(|| error("missing posterior time"))?
                    as f64
                    * 1e-9,
                session: self.session,
                quat_xyzw: [q.i, q.j, q.k, q.w],
                angular_velocity: prediction.angular_velocity.into(),
                gyro_bias: self.predictor.bias().gyro.into(),
            }),
        }))
    }
}
