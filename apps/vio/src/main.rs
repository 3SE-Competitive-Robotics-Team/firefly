//! VIO 进程：MSCKF 估计器（`firefly-vio`）+ iceoryx2 zero-copy 发布。
//!
//! 当前为**合成数据最小闭环**（无真实驱动）：GT 初始化 → 合成 IMU
//! （100Hz）→ 高频传播 → 发布：
//! - odom（10Hz，`Firefly/Odometry`）：位置/速度/姿态；
//! - imu（100Hz，`Firefly/Imu`）：原始角速度 + 比力。
//!
//! 真实驱动接入见 TODO：RealSense 相机（realsense-rust）+ 飞控 IMU
//! （串口/PX4），届时配置相机标定与 KLT 跟踪器即可启用完整 MSCKF 更新。
//!
//! 所有消息的 User Header 自动携带 fastrace trace 上下文（Trace ID 中间件），
//! 订阅端可续接为跨进程 span 树。

use std::collections::BTreeMap;
use std::time::Duration;

use fastrace::prelude::*;
use firefly_pubsub::imu::{IMU_TOPIC, ImuMessage, ImuPublisher};
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::{ODOM_TOPIC, OdomPublisher};
use firefly_vio::options::VioManagerOptions;
use firefly_vio::vio_manager::VioManager;
use firefly_vio_core::sensor::ImuData;
use firefly_vio_core::track::{HistogramMethod, TrackKlt};
use nalgebra::Vector3;

/// IMU 采样周期（秒）。
const IMU_PERIOD: f64 = 0.01;
/// odom 发布周期（秒）。
const ODOM_PERIOD: f64 = 0.1;
/// 合成初速（+x，m/s）：匀速漂移，odom 有真实运动可消费。
const INITIAL_VELOCITY: f64 = 0.5;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    log::info!("VIO 进程启动：iceoryx2 zero-copy odom/imu（Trace 上下文自动附加）");

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

    // 真值初始化：原点，+x 匀速漂移（q 单位四元数，v=[0.5,0,0]，bias=0）
    let mut imustate = [0.0f64; 17];
    imustate[0] = 0.0; // t
    imustate[4] = 1.0; // qw
    imustate[8] = INITIAL_VELOCITY; // vx
    vio.initialize_with_gt(&imustate);
    log::info!("已初始化：t=0 原点，+x 匀速 {INITIAL_VELOCITY} m/s");

    // 发布器（Trace 上下文由中间件在 publish 时自动注入）
    let odom_pub = OdomPublisher::new()?;
    log::info!("已打开话题 {ODOM_TOPIC}");
    let imu_pub = ImuPublisher::new()?;
    log::info!("已打开话题 {IMU_TOPIC}");

    // 合成 IMU：比力 = +g 抵消重力（匀速漂移）+ 轻微角速度扰动
    let wm = Vector3::new(0.001, -0.002, 0.005);
    let am = Vector3::new(0.0, 0.0, 9.81);

    let mut t_sim = 0.0f64;
    let mut next_odom = 0.0f64;
    loop {
        // 每帧建立 trace 上下文（fastrace：`#[trace]` 仅在 root span 下收集）
        let root = Span::root("vio", SpanContext::random());
        let _guard = root.set_local_parent();

        // 喂 IMU 并传播到当前时刻（高频位姿输出）
        vio.feed_measurement_imu(&ImuData {
            timestamp: t_sim,
            wm,
            am,
        });
        vio.propagate_to(t_sim);

        // 原始 IMU 发布（100Hz，debug 级别避免刷屏）
        match imu_pub.publish(ImuMessage {
            timestamp: t_sim,
            angular_velocity_x: wm.x,
            angular_velocity_y: wm.y,
            angular_velocity_z: wm.z,
            linear_acceleration_x: am.x,
            linear_acceleration_y: am.y,
            linear_acceleration_z: am.z,
        }) {
            Ok(_) => log::debug!("imu t={t_sim:.3} w={wm:?} a={am:?}"),
            Err(e) => log::warn!("imu 发布失败: {e}"),
        }

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
                        "odom t={t_sim:.2} p=({:.2},{:.2},{:.2}) v=({:.3},{:.3},{:.3}) trace_id={:032x} sampled={} send_ts={}",
                        s.imu.pos().x,
                        s.imu.pos().y,
                        s.imu.pos().z,
                        s.imu.vel().x,
                        s.imu.vel().y,
                        s.imu.vel().z,
                        ctx.trace_id(),
                        ctx.sampled(),
                        ctx.send_timestamp(),
                    );
                }
                Err(e) => log::warn!("odom 发布失败（temporary 可重试）: {e}"),
            }
            next_odom += ODOM_PERIOD;
        }

        t_sim += IMU_PERIOD;
        std::thread::sleep(Duration::from_secs_f64(IMU_PERIOD));
    }
}
