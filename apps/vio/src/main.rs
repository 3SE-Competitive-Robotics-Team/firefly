//! VIO 进程：MSCKF 估计器（`firefly-vio`）+ iceoryx2 zero-copy 发布。
//!
//! 固定接入 `MuJoCo` 物理环境（Python `firefly-sim`）经 iceoryx2 发布的
//! IMU + 双目灰度（跑完整 MSCKF 视觉更新），输出 odom（估计位姿，10Hz）
//! 到 `Firefly/Odometry`。所有消息的 User Header 自动携带 fastrace trace
//! 上下文（跨进程 span 树可观测）。
//!
//! 运行：`cargo run -p vio`（配合 `uv run firefly-sim` 的 `MuJoCo` 物理环境）。
//! - 无 rerun 可视化
//! - 无 depth/GT 订阅
//! - sleep(1ms) 尽快消费消息

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use fastrace::prelude::*;
use firefly_pubsub::publish::{ODOM_TOPIC, OdomPublisher};
use firefly_vio::options::VioManagerOptions;
use firefly_vio::vio_manager::VioManager;
use firefly_vio_core::cam::{CamRadtan, SharedCamera};
use firefly_vio_core::input::SensorInput;
use firefly_vio_core::track::{HistogramMethod, TrackKlt};

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

    // 估计器：MuJoCo 双目相机标定（`scene.py`：320×240、fovy=70.88°≈D430 87°HFOV、
    // 方形像素、基线 0.05m）。内参 focal=(H/2)/tan(fovy/2)=168.6，无畸变；外参见
    // [`mujoco_stereo_extrinsic`]。
    let focal = mujoco_focal();
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
        200,  // num_pts: 对齐 OpenVINS 默认值
        0,
        true,
        HistogramMethod::None,
        10,   // fast_threshold
        5,    // grid_x: 对齐 OpenVINS 默认值
        5,    // grid_y: 对齐 OpenVINS 默认值
        15,   // min_px_dist: 对齐 OpenVINS 默认值
    );
    let mut cameras = BTreeMap::new();
    cameras.insert(0usize, cam_left);
    cameras.insert(1usize, cam_right);
    let mut vio = VioManager::new(params, cameras, tracker);

    // 相机外参（IMU→cam）：R_ItoC + p_IinC（OpenVINS JPL 约定），见
    // [`mujoco_stereo_extrinsic`]。p_IinC = IMU 原点在相机系。
    let (q_ito_c, [p_left_in_c, p_right_in_c]) = mujoco_stereo_extrinsic();
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

    run_loop(&mut vio, &mut input, &odom_pub)
}

/// 驱动循环：推进输入 → 喂 IMU/相机 → 传播 → 发布 odom（10Hz）。
/// - 无 rerun/depth/GT 订阅
/// - sleep(1ms) 尽快消费消息
fn run_loop(
    vio: &mut VioManager,
    input: &mut dyn SensorInput,
    odom_pub: &OdomPublisher,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut t_sim = 0.0f64;
    let mut next_odom = 0.0f64;
    let mut loop_count = 0u64;
    let mut camera_count = 0u64;
    let mut imu_batch_count = 0u64;
    let t_wall_start = std::time::Instant::now();

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

        let mut imu_batch = 0u32;
        while let Some(imu) = input.next_imu() {
            vio.feed_measurement_imu(&imu);
            imu_batch += 1;
        }
        imu_batch_count += u64::from(imu_batch);

        // 相机（MSCKF 视觉更新）
        if let Some(cam) = input.next_camera() {
            camera_count += 1;
            vio.feed_measurement_camera(&cam);
            log::debug!(
                "camera t={:.3} sensors={:?} imgs={}x{}",
                cam.timestamp,
                cam.sensor_ids,
                cam.images.first().map_or(0, |g| g.width),
                cam.images.first().map_or(0, |g| g.height),
            );
        }
        t_sim = t_sim.max(now);

        // 每 2 秒打印一次诊断
        loop_count += 1;
        if loop_count % 2000 == 0 {
            let wall_s = t_wall_start.elapsed().as_secs_f64();
            log::info!(
                "[perf-diag] wall={wall_s:.1}s sim={t_sim:.2} loops={loop_count} cameras={camera_count} imu_batches={imu_batch_count} sim_rate={:.2}x",
                if wall_s > 0.0 { t_sim / wall_s } else { 0.0 }
            );
        }

        // 按发布周期输出 odom
        if t_sim + 1e-9 >= next_odom {
            let s = &vio.state;
            let msg = firefly_pubsub::odom::OdomMessage {
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

        // 性能模式：sleep(1ms) 避免空转，消息由 sim 按 sim_time 发布，vio 尽快消费
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// `MuJoCo` 相机焦距：`(H/2) / tan(fovy/2)`（320×240、fovy=70.88°≈D430 87°HFOV、方形像素）。
#[must_use]
fn mujoco_focal() -> f64 {
    120.0 / (70.88_f64 / 2.0).to_radians().tan()
}

/// `MuJoCo` 双目相机外参（OpenVINS JPL 约定），返回 `(q_ito_c, [p_left, p_right])`。
///
/// 物理相机（`scene.py` 的 `xyaxes="0 -1 0  0 0 1"`，与 `firefly-map::DepthCamera`
/// 投影一致实测校验）：**前向 = 机体 +x，上 = 机体 +z，右 = 机体 -y**。
/// VIO 三角化视线取 `(nx, ny, 1)`（标准 y-down / z-forward 相机系），故
/// body→camera 旋转 `R_ItoC = [[0,-1,0],[0,0,-1],[1,0,0]]`（列向量为相机系在 body 下的基）。
/// 基线 0.05m（左右各 ±0.025m），IMU 在两相机中点。
#[must_use]
fn mujoco_stereo_extrinsic() -> (nalgebra::Vector4<f64>, [nalgebra::Vector3<f64>; 2]) {
    use nalgebra::{Matrix3, Vector3, Vector4, UnitQuaternion};
    // R_ItoC: body -> camera
    let r_ito_c = Matrix3::new(
        0.0, -1.0, 0.0,
        0.0, 0.0, -1.0,
        1.0, 0.0, 0.0,
    );
    // 四元数 (x, y, z, w)
    let q = UnitQuaternion::from_matrix(&r_ito_c);
    let q_vec = Vector4::new(q.i, q.j, q.k, q.w);
    // 左右相机在 IMU 系下的位置：左右各 ±0.025m（基线 0.05m）
    let p_left_in_c = Vector3::new(0.0, -0.025, 0.0);
    let p_right_in_c = Vector3::new(0.0, 0.025, 0.0);
    (q_vec, [p_left_in_c, p_right_in_c])
}

#[cfg(test)]
mod tests {
    use super::{mujoco_focal, mujoco_stereo_extrinsic};
    use nalgebra::{Matrix3, Vector3};

    #[allow(clippy::many_single_char_names)]
    fn quat_to_rot(q: &nalgebra::Vector4<f64>) -> Matrix3<f64> {
        let (x, y, z, w) = (q[0], q[1], q[2], q[3]);
        Matrix3::new(
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        )
    }

    /// 双目外参：body→camera 旋转应使相机前向 = body +x，上 = body +z
    #[test]
    fn camera_forward_is_body_x() {
        let (q, _) = mujoco_stereo_extrinsic();
        let r = quat_to_rot(&q);
        // camera z 轴（前向）在 body 下 = body +x
        let cam_fwd = r * Vector3::new(0.0, 0.0, 1.0);
        assert!((cam_fwd - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-9);
        // camera y 轴（下）在 body 下 = body -y
        let cam_down = r * Vector3::new(0.0, 1.0, 0.0);
        assert!((cam_down - Vector3::new(0.0, -1.0, 0.0)).norm() < 1e-9);
    }

    /// 基线横向（沿 y），给出立体视差
    #[test]
    fn stereo_baseline_is_lateral() {
        let (_, [p_l, p_r]) = mujoco_stereo_extrinsic();
        let baseline = p_r - p_l;
        assert!((baseline - Vector3::new(0.0, 0.05, 0.0)).norm() < 1e-9);
    }

    /// 焦距与 scene.py 一致
    #[test]
    fn focal_is_mujoco_consistent() {
        assert!((mujoco_focal() - 168.606_993_943_65).abs() < 1e-6);
    }
}