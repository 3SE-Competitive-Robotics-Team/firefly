//! VIO 进程：MSCKF 估计器（`firefly-vio`）+ iceoryx2 zero-copy 发布。
//!
//! 传感器输入通过 [`SensorInput`](firefly_vio_core::input::SensorInput)
//! 端口抽象（`--input <synthetic|iceoryx>`，默认 synthetic）：
//! - `synthetic`：合成 IMU（+x 漂移）闭环自测，并发布合成 IMU；
//! - `iceoryx`：订阅 `MuJoCo` 物理环境发布的 IMU + 双目灰度（跑完整
//!   MSCKF 视觉更新），imu 话题由 `MuJoCo` 直接发布。
//!
//! 输出：odom（估计位姿，10Hz）到 `Firefly/Odometry`。所有消息的 User
//! Header 自动携带 fastrace trace 上下文（跨进程 span 树可观测）。
//!
//! 运行：`cargo run -p vio -- --input iceoryx`（配合 `uv run python -m
//! firefly_sim` 的 `MuJoCo` 物理环境）。

use std::collections::BTreeMap;
use std::time::Duration;

use fastrace::prelude::*;
use firefly_pubsub::imu::{IMU_TOPIC, ImuMessage, ImuPublisher};
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::{ODOM_TOPIC, OdomPublisher};
use firefly_vio::options::VioManagerOptions;
use firefly_vio::vio_manager::VioManager;
use firefly_vio_core::input::SensorInput;
use firefly_vio_core::track::{HistogramMethod, TrackKlt};
use nalgebra::Vector3;

mod input;
use input::{IceoryxInput, SyntheticInput};

/// IMU 采样周期（秒，合成源；iceoryx 模式为轮询周期）。
const IMU_PERIOD: f64 = 0.01;
/// odom 发布周期（秒）。
const ODOM_PERIOD: f64 = 0.1;
/// 合成 +x 初速（m/s）。
const INITIAL_VELOCITY: f64 = 0.5;
/// `MuJoCo` 场景无人机起点（= demo 地图 start；iceoryx 模式 GT 先验）。
const SIM_START: [f64; 3] = [1.0, 4.0, 1.0];

/// 传感器输入模式。
enum InputMode {
    Synthetic,
    Iceoryx,
}

fn parse_args() -> (InputMode, bool) {
    let mut mode = InputMode::Synthetic;
    let mut camera = true;
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--input" => {
                mode = match it.next().as_deref() {
                    Some("iceoryx") => InputMode::Iceoryx,
                    Some("synthetic") => InputMode::Synthetic,
                    other => {
                        eprintln!("未知输入模式 `{other:?}`（可选 synthetic|iceoryx）");
                        std::process::exit(2);
                    }
                };
            }
            "--camera" => {
                camera = match it.next().as_deref() {
                    Some("on") => true,
                    Some("off") => false,
                    other => {
                        eprintln!("未知相机开关 `{other:?}`（可选 on|off）");
                        std::process::exit(2);
                    }
                };
            }
            other => {
                eprintln!("未知参数 `{other}`");
                std::process::exit(2);
            }
        }
    }
    (mode, camera)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    let (mode, camera) = parse_args();
    log::info!("VIO 进程启动：输入源 {:?}（iceoryx2 输出 + trace 上下文）", match mode {
        InputMode::Synthetic => "synthetic",
        InputMode::Iceoryx => "iceoryx",
    });
    if camera {
        log::info!("相机路径已启用（MSCKF 视觉更新）");
    }

    // 估计器（默认参数；相机驱动接入后在此配置 CamRadtan/CamEqui 与 tracker）
    let params = VioManagerOptions::default();
    let tracker = TrackKlt::new(
        std::collections::HashMap::new(),
        200,
        0,
        false,
        HistogramMethod::None,
        20,
        4,
        4,
        10,
    );
    let mut vio = VioManager::new(params, BTreeMap::new(), tracker);

    // 真值先验：synthetic 原点 +x 漂移；iceoryx 与 MuJoCo 场景一致（起点静止）
    let mut imustate = [0.0f64; 17];
    imustate[0] = 0.0; // t
    imustate[4] = 1.0; // qw
    match mode {
        InputMode::Synthetic => imustate[8] = INITIAL_VELOCITY, // vx
        InputMode::Iceoryx => {
            imustate[5] = SIM_START[0];
            imustate[6] = SIM_START[1];
            imustate[7] = SIM_START[2];
        }
    }
    vio.initialize_with_gt(&imustate);
    log::info!(
        "已初始化：t=0 {}（GT 先验）",
        match mode {
            InputMode::Synthetic => format!("原点，+x 匀速 {INITIAL_VELOCITY} m/s"),
            InputMode::Iceoryx => format!("({SIM_START:?})，静止"),
        }
    );

    // odom 发布器（Trace 上下文由中间件在 publish 时自动注入）
    let odom_pub = OdomPublisher::new()?;
    log::info!("已打开话题 {ODOM_TOPIC}");

    // 输入源 + synthetic 模式的 IMU 发布器
    let mut input: Box<dyn SensorInput>;
    let imu_pub: Option<ImuPublisher>;
    match mode {
        InputMode::Synthetic => {
            let pub_ = ImuPublisher::new()?;
            log::info!("已打开话题 {IMU_TOPIC}（synthetic 模式发布合成 IMU）");
            input = Box::new(SyntheticInput::new(
                IMU_PERIOD,
                Vector3::new(0.001, -0.002, 0.005),
                Vector3::new(0.0, 0.0, 9.81),
            ));
            imu_pub = Some(pub_);
        }
        InputMode::Iceoryx => {
            log::info!("输入源：订阅 MuJoCo 物理环境（IMU/双目灰度），imu 话题由 MuJoCo 发布");
            input = Box::new(IceoryxInput::new()?);
            imu_pub = None;
        }
    }

    run_loop(&mut vio, input.as_mut(), imu_pub.as_ref(), &odom_pub, camera)
}

/// 驱动循环：推进输入 → 喂 IMU/相机 → 传播 → 发布 odom（10Hz）。
fn run_loop(
    vio: &mut VioManager,
    input: &mut dyn SensorInput,
    imu_pub: Option<&ImuPublisher>,
    odom_pub: &OdomPublisher,
    camera: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut t_sim = 0.0f64;
    let mut next_odom = 0.0f64;
    loop {
        // 推进输入源并消费到当前时刻
        input.advance();
        let now = input.now();

        // 帧 trace：续接最近收到的传感器 trace（每周期一条，跨进程同 trace），
        // 无 trace 上下文时自建新 root
        let root = match input.last_trace() {
            Some((tid, sid, sampled)) => Span::root(
                "vio",
                SpanContext::new(TraceId(tid), SpanId(sid)).sampled(sampled),
            ),
            None => Span::root("vio", SpanContext::random()),
        };
        let _guard = root.set_local_parent();

        while let Some(imu) = input.next_imu() {
            vio.feed_measurement_imu(&imu);
            // synthetic 模式：把生成的 IMU 发布出去
            if let Some(pub_) = imu_pub {
                let _ = pub_.publish(ImuMessage {
                    timestamp: imu.timestamp,
                    angular_velocity_x: imu.wm.x,
                    angular_velocity_y: imu.wm.y,
                    angular_velocity_z: imu.wm.z,
                    linear_acceleration_x: imu.am.x,
                    linear_acceleration_y: imu.am.y,
                    linear_acceleration_z: imu.am.z,
                });
            }
        }
        // 相机（MSCKF 视觉更新；`--camera on`，标定后启用）
        if camera
            && let Some(cam) = input.next_camera()
        {
            vio.feed_measurement_camera(&cam);
            log::debug!(
                "camera t={:.3} sensors={:?} imgs={}x{}",
                cam.timestamp,
                cam.sensor_ids,
                cam.images.first().map_or(0, |g| g.width),
                cam.images.first().map_or(0, |g| g.height),
            );
        }
        vio.propagate_to(now);
        t_sim = t_sim.max(now);

        // 按发布周期输出 odom
        if t_sim + 1e-9 >= next_odom {
            let s = &vio.state;
            let msg = OdomMessage {
                timestamp: t_sim,
                position_x: s.imu.pos().x,
                position_y: s.imu.pos().y,
                position_z: s.imu.pos().z,
                velocity_x: s.imu.vel().x,
                velocity_y: s.imu.vel().y,
                velocity_z: s.imu.vel().z,
                quat_x: s.imu.quat()[0],
                quat_y: s.imu.quat()[1],
                quat_z: s.imu.quat()[2],
                quat_w: s.imu.quat()[3],
                is_initialized: vio.initialized(),
            };
            match odom_pub.publish(msg) {
                Ok(ctx) => {
                    log::info!(
                        "odom t={t_sim:.2} p=({:.2},{:.2},{:.2}) v=({:.3},{:.3},{:.3}) trace_id={:032x} sampled={}",
                        s.imu.pos().x,
                        s.imu.pos().y,
                        s.imu.pos().z,
                        s.imu.vel().x,
                        s.imu.vel().y,
                        s.imu.vel().z,
                        ctx.trace_id(),
                        ctx.sampled(),
                    );
                }
                Err(e) => log::warn!("odom 发布失败（temporary 可重试）: {e}"),
            }
            next_odom += ODOM_PERIOD;
        }

        std::thread::sleep(Duration::from_secs_f64(IMU_PERIOD));
    }
}
