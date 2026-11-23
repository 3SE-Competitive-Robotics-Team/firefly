//! Desktop Matrix substitute. The simulated plant supplies the IMU and rotor geometry.
use firefly_flight::{
    ControlParams, Controller, QuadState, ipc::airframe_to_params, sticks::AngleDemand,
};
use firefly_imu::{Estimator, Sample};
use firefly_pubsub::{
    control::{CONTROL_BUFFER_SIZE, CONTROL_TOPIC, ControlMessage},
    imu::{IMU_SUBSCRIPTION, IMU_TOPIC, ImuMessage},
    node::create_node,
    plant::{AIRFRAME_BUFFER_SIZE, AIRFRAME_TOPIC, AirframeMessage},
    publish::Publisher,
    sticks::{STICKS_BUFFER_SIZE, STICKS_TOPIC, SticksMessage},
    subscriber::Subscriber,
    viz::{ScalarMessage, VizPublisher},
};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalHandler};
use nalgebra::{UnitQuaternion, Vector3};
use serde::Deserialize;
use std::{
    path::PathBuf,
    sync::atomic::{AtomicBool, Ordering},
    thread,
    time::{Duration, Instant},
};

#[derive(usage_rs::Cli)]
#[usage(bin = "fc")]
struct Arguments {
    /// Simulation controller parameters
    #[usage(long)]
    config: Option<PathBuf>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    #[serde(default)]
    control: ControlParams,
}

const PERIOD: Duration = Duration::from_millis(1);
const STICK_TIMEOUT: Duration = Duration::from_millis(100);
static RUNNING: AtomicBool = AtomicBool::new(true);

fn demand(command: Option<(SticksMessage, Instant)>, stamp: f64) -> Option<AngleDemand> {
    command
        .filter(|(m, arrival)| {
            m.valid()
                && arrival.elapsed() < STICK_TIMEOUT
                && (0.0..STICK_TIMEOUT.as_secs_f64()).contains(&(stamp - m.state_time))
        })
        .map(|(m, _)| AngleDemand::from_channels(m.channels))
        .filter(|d| d.thrust > 0.)
}

fn scalars(viz: &VizPublisher, stamp: f64, path: &str, values: &[f64]) {
    let mut m = ScalarMessage::base(stamp, path);
    m.scalar_count = values.len() as u32;
    m.scalars[..values.len()].copy_from_slice(values);
    let _ = viz.publish(m);
}

fn main() -> std::process::ExitCode {
    firefly_error::report("fc", run_app())
}

#[allow(clippy::too_many_lines)]
fn run_app() -> firefly_error::Result<()> {
    let cfg_path = Arguments::parse()
        .config
        .unwrap_or_else(|| firefly_config::paths::config("fc.toml"));
    let cfg = firefly_config::startup::<Config>(&cfg_path, firefly_observability::init, |c| {
        c.control.validate()
    })?;
    let _signals = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_| RUNNING.store(false, Ordering::Relaxed),
    )
    .map_err(|e| {
        firefly_error::Error::caused_by(
            firefly_error::ErrorKind::Internal,
            "register shutdown signals",
            e,
        )
    })?;
    let node = create_node()?;
    let sticks = Subscriber::<SticksMessage>::with_topic_and_buffer(
        &node,
        STICKS_TOPIC,
        STICKS_BUFFER_SIZE,
    )?;
    let imu = Subscriber::<ImuMessage>::with_options(&node, IMU_TOPIC, IMU_SUBSCRIPTION)?;
    let airframe = Subscriber::<AirframeMessage>::with_topic_and_buffer(
        &node,
        AIRFRAME_TOPIC,
        AIRFRAME_BUFFER_SIZE,
    )?;
    let motors = Publisher::with_topic_and_buffer(&node, CONTROL_TOPIC, Some(CONTROL_BUFFER_SIZE))?;
    let viz = VizPublisher::new(&node)?;
    let mut filter = Estimator::new(firefly_imu::Options::default())?;
    let mut controller = Controller::new(cfg.control);
    let mut command = None;
    let mut model = None;
    let mut stamp = 0.;
    let mut imu_at = None;
    let mut gyro = Vector3::zeros();
    let mut tick = 0;
    let mut next = Instant::now();
    let mut last_tick = next;
    let mut saturated = 0u64;
    let mut max_late = Duration::ZERO;
    let mut summary_stamp = -1.;
    let mut window_ticks = 0u32;
    let mut window = next;
    while RUNNING.load(Ordering::Relaxed) {
        let now = Instant::now();
        let wall_dt = now.duration_since(last_tick);
        last_tick = now;
        while let Some(m) = airframe.receive()? {
            model = Some(airframe_to_params(&m));
        }
        while let Some(m) = sticks.receive()? {
            if m.valid()
                && command
                    .is_none_or(|(old, _): (SticksMessage, Instant)| old.state_time <= m.state_time)
            {
                command = Some((*m, now));
            }
        }
        let grounded = demand(command, stamp).is_none();
        while let Some(m) = imu.receive()? {
            let sample = Sample {
                timestamp: m.timestamp,
                gyro: Vector3::new(
                    m.angular_velocity_x,
                    m.angular_velocity_y,
                    m.angular_velocity_z,
                ),
                accel: Vector3::new(
                    m.linear_acceleration_x,
                    m.linear_acceleration_y,
                    m.linear_acceleration_z,
                ),
            };
            match filter.observe(sample, grounded) {
                Ok(true) => {
                    stamp = sample.timestamp;
                    gyro = sample.gyro;
                    imu_at = Some(now);
                }
                Err(_) if grounded => {
                    filter = Estimator::new(firefly_imu::Options::default())?;
                    imu_at = None;
                }
                Err(_) => imu_at = None,
                Ok(false) => {}
            }
        }
        let fresh = imu_at.is_some_and(|at| now.duration_since(at) < Duration::from_millis(50));
        let thrust = if let (true, Some(att), Some((quad, frame)), Some(d)) = (
            fresh,
            filter.state(),
            model.as_ref(),
            demand(command, stamp),
        ) {
            let state = QuadState {
                attitude: att.rotation.cast(),
                ang_vel: filter.rate().cast(),
                ..Default::default()
            };
            let goal = UnitQuaternion::from_euler_angles(d.roll, d.pitch, state.yaw());
            controller.attitude_mode(
                &state,
                goal,
                d.thrust,
                d.yaw_rate,
                quad,
                frame,
                PERIOD.as_secs_f32(),
            )
        } else {
            controller.reset();
            [0.; 4]
        };
        motors.publish(ControlMessage {
            state_time: stamp,
            tick,
            thrust: thrust.map(f64::from),
        })?;
        saturated += u64::from(controller.saturated);
        if log::log_enabled!(target:"fc::diagnostics", log::Level::Debug) {
            let age = imu_at.map_or(-1., |at| now.duration_since(at).as_secs_f64() * 1e3);
            scalars(
                &viz,
                stamp,
                "fc/loop/timing",
                &[
                    tick as f64,
                    command.map_or(-1., |(m, _)| (stamp - m.state_time) * 1e3),
                    age,
                    wall_dt.as_secs_f64() * 1e3,
                ],
            );
            let rate = filter.rate();
            for (name, sp, meas, raw, torque) in [
                (
                    "fc/loop/roll",
                    controller.rates_setpoint.roll,
                    rate.x,
                    gyro.x,
                    controller.torque_setpoint.xyz.x,
                ),
                (
                    "fc/loop/pitch",
                    controller.rates_setpoint.pitch,
                    -rate.y,
                    -gyro.y,
                    controller.torque_setpoint.xyz.y,
                ),
            ] {
                scalars(&viz, stamp, name, &[sp.into(), meas, raw, torque.into()]);
            }
            scalars(
                &viz,
                stamp,
                "fc/loop/control",
                &[
                    f64::from(thrust.iter().sum::<f32>()),
                    f64::from(controller.saturated),
                    controller.allocated_control[0].into(),
                    f64::from(fresh),
                ],
            );
            scalars(&viz, stamp, "fc/loop/motors", &thrust.map(f64::from));
        }
        if stamp - summary_stamp >= 0.1 {
            summary_stamp = stamp;
            scalars(&viz, stamp, "fc/debug/motors", &thrust.map(f64::from));
            scalars(
                &viz,
                stamp,
                "fc/debug/thrust_total",
                &[f64::from(thrust.iter().sum::<f32>())],
            );
            let rates = controller.rates_setpoint;
            scalars(
                &viz,
                stamp,
                "fc/debug/rate_sp_frd",
                &[rates.roll.into(), rates.pitch.into(), rates.yaw.into()],
            );
            scalars(
                &viz,
                stamp,
                "fc/debug/torque_sp_frd",
                controller.torque_setpoint.xyz.cast::<f64>().as_slice(),
            );
            scalars(
                &viz,
                stamp,
                "fc/debug/allocated_control",
                controller.allocated_control.cast::<f64>().as_slice(),
            );
            scalars(
                &viz,
                stamp,
                "fc/debug/saturated",
                &[f64::from(controller.saturated)],
            );
        }
        window_ticks += 1;
        tick += 1;
        max_late = max_late.max(now.saturating_duration_since(next));
        if window.elapsed() >= Duration::from_secs(1) {
            scalars(
                &viz,
                stamp,
                "fc/debug/tick_rate",
                &[f64::from(window_ticks) / window.elapsed().as_secs_f64()],
            );
            scalars(&viz, stamp, "fc/debug/saturated_ticks", &[saturated as f64]);
            scalars(
                &viz,
                stamp,
                "fc/debug/late_max_ms",
                &[max_late.as_secs_f64() * 1e3],
            );
            window = now;
            max_late = Duration::ZERO;
            saturated = 0;
            window_ticks = 0;
        }
        next += PERIOD;
        if now.saturating_duration_since(next) > 10 * PERIOD {
            next = now + PERIOD;
        }
        thread::sleep(next.saturating_duration_since(Instant::now()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn simulated_motor_output_requires_fresh_valid_sticks() {
        let at = Instant::now();
        let sticks = SticksMessage {
            state_time: 1.,
            channels: [1500; 4],
        };
        assert!(demand(Some((sticks, at)), 1.01).is_some());
        assert!(demand(Some((sticks, at)), 1.2).is_none());
        assert!(demand(Some((sticks, at.checked_sub(STICK_TIMEOUT).unwrap())), 1.).is_none());
        assert!(
            demand(
                Some((
                    SticksMessage {
                        channels: [0; 4],
                        ..sticks
                    },
                    at
                )),
                1.
            )
            .is_none()
        );
    }
}
