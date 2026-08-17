//! VIO 进程：MSCKF 估计器（`firefly-vio`）+ iceoryx2 zero-copy 发布。
//!
//! 固定接入 `MuJoCo` 物理环境（Python `firefly-sim`）经 iceoryx2 发布的
//! IMU + 双目灰度（跑完整 MSCKF 视觉更新），输出 odom（估计位姿，10Hz）
//! 到 `Firefly/Odometry`。所有消息的 User Header 自动携带 fastrace trace
//! 上下文（跨进程 span 树可观测）。
//!
//! 可视化：订阅的传感器（双目灰度 + 深度）与估计 odom 位姿同步写入 rerun
//! viewer（`sensor/stereo_left|right`、`sensor/depth`、`vio/odom`，统一
//! `sim_time` 时间轴）。已有 `rerun` viewer 则共享（多进程），否则自起。
//!
//! 运行：`cargo run -p vio`（配合 `uv run firefly-sim` 的 `MuJoCo` 物理环境）。

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use fastrace::prelude::*;
use firefly_pubsub::camera::{DEPTH_TOPIC, DepthImageMessage};
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::{ODOM_TOPIC, OdomPublisher};
use firefly_pubsub::subscriber::Subscriber;
use firefly_rerun::Stream;
use firefly_vio::options::VioManagerOptions;
use firefly_vio::vio_manager::VioManager;
use firefly_vio_core::cam::{CamRadtan, SharedCamera};
use firefly_vio_core::input::SensorInput;
use firefly_vio_core::track::{HistogramMethod, TrackKlt};
use nalgebra::Vector3;

mod input;
use input::IceoryxInput;

/// IMU 轮询周期（秒）。
const IMU_PERIOD: f64 = 0.01;
/// odom 发布周期（秒）。
const ODOM_PERIOD: f64 = 0.1;
/// `MuJoCo` 场景无人机起点（= demo 地图 start；GT 先验）。
const SIM_START: [f64; 3] = [1.0, 4.0, 1.0];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    log::info!("VIO 进程启动：订阅 MuJoCo 物理环境（iceoryx2 输入 + trace 上下文）");

    // 估计器：MuJoCo 双目相机标定（`scene.py`：320×240、fovy=60°、方形像素、
    // 基线 0.1m）。内参 focal=(H/2)/tan(fovy/2)=207.85，无畸变；外参：
    // 相机轴在机体 x=(0,-1,0)、y=(0,0,-1)、z=(1,0,0)，左 (-0.05,0,0)、
    // 右 (0.05,0,0)。
    let focal = 120.0 / (60.0_f64 / 2.0).to_radians().tan();
    let intrinsics = [focal, focal, 160.0, 120.0, 0.0, 0.0, 0.0, 0.0];
    let cam_left: SharedCamera = Arc::new(CamRadtan::new(320, 240, &intrinsics));
    let cam_right: SharedCamera = Arc::new(CamRadtan::new(320, 240, &intrinsics));

    let mut params = VioManagerOptions::default();
    params.state_options.num_cameras = 2;
    let mut tracker_calib = std::collections::HashMap::new();
    tracker_calib.insert(0usize, cam_left.clone());
    tracker_calib.insert(1usize, cam_right.clone());
    let tracker = TrackKlt::new(
        tracker_calib,
        200,
        0,
        true,
        HistogramMethod::None,
        20,
        4,
        4,
        10,
    );
    let mut cameras = BTreeMap::new();
    cameras.insert(0usize, cam_left);
    cameras.insert(1usize, cam_right);
    let mut vio = VioManager::new(params, cameras, tracker);

    // 相机外参（IMU→cam）：R_ItoC 列 = 相机轴在机体，p_IinC = IMU 原点在相机系
    // 左相机 p_IinC=(0,-0.05,0)、右 (0,0.05,0)；四元数 JPL xyzw（实测重建校验）
    let q_ito_c = nalgebra::Vector4::new(0.5, -0.5, 0.5, -0.5);
    let p_left_in_c = Vector3::new(0.0, -0.05, 0.0);
    let p_right_in_c = Vector3::new(0.0, 0.05, 0.0);
    for (cam_id, p_iin_c) in [(0usize, p_left_in_c), (1usize, p_right_in_c)] {
        let calib = vio
            .state
            .calib_imu_to_cam
            .get_mut(&cam_id)
            .expect("num_cameras=2 已建外参槽位");
        calib.set_value(q_ito_c, p_iin_c);
        calib.set_fej(q_ito_c, p_iin_c);
    }

    // 真值先验：与 MuJoCo 场景一致（起点静止）
    let mut imustate = [0.0f64; 17];
    imustate[0] = 0.0; // t
    imustate[4] = 1.0; // qw
    imustate[5] = SIM_START[0];
    imustate[6] = SIM_START[1];
    imustate[7] = SIM_START[2];
    vio.initialize_with_gt(&imustate);
    log::info!("已初始化：t=0 ({SIM_START:?})，静止（GT 先验）");

    // odom 发布器（Trace 上下文由中间件在 publish 时自动注入）
    let odom_pub = OdomPublisher::new()?;
    log::info!("已打开话题 {ODOM_TOPIC}");

    // 输入源：订阅 MuJoCo 物理环境（IMU/双目灰度），imu 话题由 MuJoCo 发布
    let mut input = IceoryxInput::new()?;
    log::info!("输入源：订阅 MuJoCo 物理环境（IMU/双目灰度），imu 话题由 MuJoCo 发布");

    // rerun 可视化：已有 viewer 则共享（多进程），否则自起；失败不影响估计
    let viewer = match Stream::connect_or_spawn("firefly-vio") {
        Ok(v) => {
            log::info!("rerun viewer 就绪（传感器/odom 可视化）");
            Some(v)
        }
        Err(e) => {
            log::warn!("rerun viewer 不可用（跳过可视化，继续运行）：{e}");
            None
        }
    };
    // 深度订阅（仅用于可视化；估计器只用 IMU + 双目）
    let depth_sub = match Subscriber::<DepthImageMessage>::with_topic(DEPTH_TOPIC) {
        Ok(s) => {
            log::info!("已订阅深度话题 {DEPTH_TOPIC}（rerun 可视化）");
            Some(s)
        }
        Err(e) => {
            log::warn!("深度订阅不可用（无深度可视化）：{e}");
            None
        }
    };

    run_loop(&mut vio, &mut input, &odom_pub, viewer.as_ref(), depth_sub.as_ref())
}

/// 驱动循环：推进输入 → 喂 IMU/相机 → 传播 → 发布 odom（10Hz）。
fn run_loop(
    vio: &mut VioManager,
    input: &mut dyn SensorInput,
    odom_pub: &OdomPublisher,
    viewer: Option<&Stream>,
    depth_sub: Option<&Subscriber<DepthImageMessage>>,
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
        }
        // 深度帧 → rerun（仅可视化；估计器不用深度）
        if let Some(sub) = depth_sub
            && let Some(viewer) = viewer
            && let Ok(Some(sample)) = sub.receive()
        {
            let m = *sample;
            viewer.set_time(m.timestamp);
            log_depth(viewer, &m);
        }
        // 相机（MSCKF 视觉更新 + rerun 双目可视化）
        if let Some(cam) = input.next_camera() {
            vio.feed_measurement_camera(&cam);
            log::debug!(
                "camera t={:.3} sensors={:?} imgs={}x{}",
                cam.timestamp,
                cam.sensor_ids,
                cam.images.first().map_or(0, |g| g.width),
                cam.images.first().map_or(0, |g| g.height),
            );
            if let Some(viewer) = viewer {
                viewer.set_time(cam.timestamp);
                log_cameras(viewer, &cam);
            }
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
                    // 估计位姿 → rerun（统一 sim_time 时间轴）
                    if let Some(viewer) = viewer {
                        viewer.set_time(t_sim);
                        let q = s.imu.quat();
                        if let Err(e) = viewer.log_pose("vio/odom", [
                            s.imu.pos().x,
                            s.imu.pos().y,
                            s.imu.pos().z,
                        ], [q[0], q[1], q[2], q[3]]) {
                            log::debug!("rerun 记录 odom 位姿失败：{e}");
                        }
                    }
                }
                Err(e) => log::warn!("odom 发布失败（temporary 可重试）: {e}"),
            }
            next_odom += ODOM_PERIOD;
        }

        std::thread::sleep(Duration::from_secs_f64(IMU_PERIOD));
    }
}

/// 双目灰度帧 → rerun（按 `sensor_id` 分左右目实体）。
fn log_cameras(viewer: &Stream, cam: &firefly_vio_core::sensor::CameraData) {
    for (id, img) in cam.sensor_ids.iter().zip(&cam.images) {
        let entity = match id {
            0 => Some("sensor/stereo_left"),
            1 => Some("sensor/stereo_right"),
            other => {
                log::debug!("未知 sensor_id {other}，跳过可视化");
                None
            }
        };
        let Some(entity) = entity else { continue };
        if let Err(e) = viewer.log_gray_image(entity, img.width as u32, img.height as u32, &img.data)
        {
            log::debug!("rerun 记录 {entity} 失败：{e}");
        }
    }
}

/// 深度帧 → rerun。
fn log_depth(viewer: &Stream, m: &DepthImageMessage) {
    if let Err(e) = viewer.log_depth_image("sensor/depth", m.width, m.height, &m.data) {
        log::debug!("rerun 记录 depth 失败：{e}");
    }
}