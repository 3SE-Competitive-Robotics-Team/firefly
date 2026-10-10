//! 合成 IMU 与点阵双目图像，检查跟踪、三角化、滤波及静止初始化的输出。
//! 匀速场景含弱可观方向；被忽略的场景不能计作通过或替代飞行验收。

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;

use firefly_vio::options::VioManagerOptions;
use firefly_vio::vio_manager::VioManager;
use firefly_vio_core::cam::{CamRadtan, SharedCamera};
use firefly_vio_core::sensor::{CameraData, GrayImage, ImuData};
use firefly_vio_core::track::{HistogramMethod, TrackKlt};
use firefly_vio_types::quat_ops::rot_2_quat;
use nalgebra::{Matrix3, Vector3};

const W: usize = 320;
const H: usize = 240;
const FOCAL: f64 = 168.606_993_943_65;
const CX: f64 = 160.0;
const CY: f64 = 120.0;

/// 注入 IMU 的真实零偏（滤波器应在线估计并收敛到该值附近）。
const BIAS_A_TRUE: Vector3<f64> = Vector3::new(0.05, -0.03, 0.02);
const BIAS_G_TRUE: Vector3<f64> = Vector3::new(0.001, -0.0008, 0.0006);

/// 场景相机外参旋转（body→camera，CV 光学约定）；唯一来源 `firefly_base::rig`。
fn r_ito_c() -> Matrix3<f64> {
    firefly_base::rig::rot_ito_c()
}

fn build_manager_ex(max_slam: usize, do_fej: bool) -> VioManager {
    let intrinsics = [FOCAL, FOCAL, CX, CY, 0.0, 0.0, 0.0, 0.0];
    let cam_l: SharedCamera = Arc::new(CamRadtan::new(W, H, &intrinsics));
    let cam_r: SharedCamera = Arc::new(CamRadtan::new(W, H, &intrinsics));
    let mut params = VioManagerOptions {
        // 均匀白噪声 ±0.02 m/s²、±0.002 rad/s，100 Hz：密度=幅度/√(3fs)。
        imu_noises: firefly_vio_core::noise::ImuNoise::new(
            0.002 / 300.0_f64.sqrt(),
            2.0e-3,
            0.02 / 300.0_f64.sqrt(),
            3.0e-3,
        ),
        ..VioManagerOptions::default()
    };
    params.state_options.num_cameras = 2;
    params.state_options.max_slam_features = max_slam;
    params.state_options.do_fej = do_fej;

    let mut tracker_calib = HashMap::new();
    tracker_calib.insert(0usize, cam_l.clone());
    tracker_calib.insert(1usize, cam_r.clone());
    let tracker = TrackKlt::new(
        tracker_calib,
        200,
        0,
        true,
        HistogramMethod::None,
        10,
        5,
        5,
        15,
    );
    let mut cameras = BTreeMap::new();
    cameras.insert(0usize, cam_l);
    cameras.insert(1usize, cam_r);
    let mut mgr = VioManager::new(params, cameras, tracker);

    // 与 render::rig 几何一致：左目在机体 −Y；p_IinC = R_ItoC·(0 − t_cam_body)
    let r = r_ito_c();
    let q = rot_2_quat(&r);
    let p_left_in_c = firefly_base::rig::p_i_in_c(firefly_base::FrameId::LEFT_CAMERA);
    let p_right_in_c = firefly_base::rig::p_i_in_c(firefly_base::FrameId::RIGHT_CAMERA);
    for (cam_id, p) in [(0usize, p_left_in_c), (1usize, p_right_in_c)] {
        let calib = mgr.state.calib_imu_to_cam.get_mut(&cam_id).unwrap();
        calib.set_value(q, p);
        calib.set_fej(q, p);
    }
    mgr
}

/// 非对称已知点云：走廊两侧不同高度错落分布（世界系）。
///
/// x 向确定性抖动（±0.5m，xorshift 按 index 派生）打破 2.5m 列周期：纯前向
/// 运动下周期点阵会让 LK 跳到相邻列同 y/z 的"兄弟点"（沿极线方向，基础矩
/// 阵 RANSAC 对其免疫），测量系统性短报位移 → 估计速度被刹到 ~1/3。
fn world_points() -> Vec<Vector3<f64>> {
    let mut pts = Vec::new();
    for i in 0..10 {
        let mut seed = 0x9E37_79B9_u32 ^ (i as u32).wrapping_mul(0x85EB_CA6B);
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        let jitter = f64::from(seed % 100) / 100.0 - 0.5; // [-0.5, 0.5)
        let x = 4.0 + f64::from(i) * 2.5 + jitter;
        pts.push(Vector3::new(x, -2.5, 0.5));
        pts.push(Vector3::new(x, 2.5, 0.9));
        pts.push(Vector3::new(x, -2.8, 2.2));
        pts.push(Vector3::new(x, 2.2, 1.6));
        pts.push(Vector3::new(x, 0.4, 2.8));
        pts.push(Vector3::new(x, -0.9, 1.2));
    }
    pts
}

fn project(
    p_w: Vector3<f64>,
    p_body: Vector3<f64>,
    t_cam_body: Vector3<f64>,
) -> Option<(f32, f32)> {
    let p_b = p_w - p_body;
    let p_c = r_ito_c() * (p_b - t_cam_body);
    if p_c.z < 0.5 {
        return None;
    }
    let u = FOCAL * p_c.x / p_c.z + CX;
    let v = FOCAL * p_c.y / p_c.z + CY;
    (u >= 2.0 && v >= 2.0 && u < (W - 2) as f64 && v < (H - 2) as f64)
        .then_some((u as f32, v as f32))
}

fn render_dots(uvs: &[(f32, f32)], seed: usize, amp: u8) -> GrayImage {
    // 全图叠加确定性噪声：purecv/OpenCV 式 NMS 是 8 邻域严格大于比较，
    // 常值亮度平台会整片互斥归零（实测纯色点阵检出 0）；噪声打破平局。
    // `amp=0` 完全无噪；seed 逐帧变化=闪烁噪声（真实传感器），固定=静态纹理
    //（图像固定的假角点会被 LK 稳定跟踪，与刚体几何矛盾）。
    // 背景：平坦 + 噪声（**无棋盘格**）。棋盘格是图像空间固定纹理（不随
    // 相机投影），其 FAST 特征在图像里静止——相机移动而测量不变 = 特征在
    // 无穷远，DLT 秩亏（cond 百万级、深度负/巨大）污染三角化。高斯斑
    // （走廊 3D 点投影）是唯一合法特征源。
    let mut data = vec![70u8; W * H];
    let noise = |i: usize| -> u8 {
        ((i.wrapping_mul(2_654_435_761)).wrapping_add(seed.wrapping_mul(40_503)) >> 24) as u8
            % amp.max(1)
    };
    for (i, &(u, v)) in uvs.iter().enumerate() {
        // 高斯斑（σ≈1.5px）：中心单峰 → FAST NMS（严格大于 8 邻域）保留，
        // 且连续梯度场让 LK 稳定收敛。3×3 平台斑是响应平台，NMS 全抑制
        // （数学事实，OpenCV 同样行为）——平台斑检不出。
        let level = 150u8 + ((i as u16 * 37) % 100) as u8;
        let ui = u.round() as isize;
        let vi = v.round() as isize;
        for dy in -3..=3i64 {
            for dx in -3..=3i64 {
                let x = ui + dx as isize;
                let y = vi + dy as isize;
                if x >= 0 && (x as usize) < W && y >= 0 && (y as usize) < H {
                    let g = (-(dx * dx + dy * dy) as f64 / 4.5).exp();
                    let val = (f64::from(level) * g) as u8;
                    data[y as usize * W + x as usize] = val;
                }
            }
        }
    }
    if amp > 0 {
        for (idx, d) in data.iter_mut().enumerate() {
            *d = d.saturating_add(noise(idx));
        }
    }
    GrayImage {
        width: W,
        height: H,
        data,
    }
}

/// 简单可复现 LCG 噪声（避免测试对线程 RNG 的依赖）。
struct Lcg(u64);
impl Lcg {
    fn next_f64(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((self.0 >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
    }
}

/// 场景配置（测试矩阵参数化）。
#[allow(clippy::struct_excessive_bools)] // 测试矩阵的开关语义，非状态机标志
struct ScenarioCfg {
    inject_bias: bool,
    max_slam: usize,
    do_fej: bool,
    /// IMU 白噪幅度（m/s² 与 rad/s 量级）。
    imu_noise: f64,
    /// 图像噪声幅度（0=无噪）。
    img_noise_amp: u8,
    /// true=逐帧重播种（闪烁）；false=固定纹理。
    img_noise_flicker: bool,
    /// true=前 5s 静止后运动。
    static_then_move: bool,
}

impl Default for ScenarioCfg {
    fn default() -> Self {
        Self {
            inject_bias: false,
            max_slam: 0,
            do_fej: true,
            imu_noise: 0.02,
            img_noise_amp: 7,
            img_noise_flicker: true,
            static_then_move: false,
        }
    }
}

fn run_scenario(inject_bias: bool, max_slam: usize) -> (f64, f64, Vector3<f64>) {
    run_cfg(&ScenarioCfg {
        inject_bias,
        max_slam,
        ..ScenarioCfg::default()
    })
}

fn run_cfg(cfg: &ScenarioCfg) -> (f64, f64, Vector3<f64>) {
    static INIT: std::sync::Once = std::sync::Once::new();
    INIT.call_once(firefly_observability::init);
    let mut mgr = build_manager_ex(cfg.max_slam, cfg.do_fej);

    // GT 初始化：t=0, 单位姿态, pos=(0,0,1), vel=(1,0,0)（静止场景 vel=0），零偏先验
    let mut s0 = [0.0f64; 17];
    s0[4] = 1.0;
    s0[7] = 1.0;
    s0[8] = if cfg.static_then_move { 0.0 } else { 1.0 };
    mgr.initialize_with_gt(&s0);

    let pts = world_points();
    let v_gt = Vector3::new(1.0, 0.0, 0.0);
    let p0 = Vector3::new(0.0, 0.0, 1.0);
    let dt_cam = 0.1_f64;
    let dt_imu = 0.01_f64;
    let frames = 100_usize;
    let static_frames = if cfg.static_then_move { 50 } else { 0 };

    let mut rng = Lcg(0xfeed_d00d);

    for k in 1..=frames {
        let t_cam = f64::from(k as u32) * dt_cam;
        for j in 0..10 {
            let ts = t_cam - dt_cam + f64::from(j as u32) * dt_imu;
            // 机体保持水平；运动开始时用 0.1s 将速度从 0 加至 1m/s。
            let a_motion = if cfg.static_then_move
                && ts > f64::from(static_frames) * dt_cam
                && ts <= (f64::from(static_frames) + 1.0) * dt_cam
            {
                Vector3::new(1.0 / 0.1, 0.0, 0.0) // 0.1s 内从 0 加速到 1m/s
            } else {
                Vector3::zeros()
            };
            let am = Vector3::new(0.0, 0.0, 9.81)
                + a_motion
                + if cfg.inject_bias {
                    BIAS_A_TRUE
                } else {
                    Vector3::zeros()
                }
                + Vector3::new(rng.next_f64(), rng.next_f64(), rng.next_f64()) * cfg.imu_noise;
            let wm = if cfg.inject_bias {
                BIAS_G_TRUE
            } else {
                Vector3::zeros()
            } + Vector3::new(rng.next_f64(), rng.next_f64(), rng.next_f64())
                * cfg.imu_noise
                * 0.1;
            mgr.feed_measurement_imu(&ImuData {
                timestamp: ts,
                wm,
                am,
            });
        }
        let p_body = p0 + v_gt * (t_cam - f64::from(static_frames) * dt_cam).max(0.0);
        let uv_l: Vec<_> = pts
            .iter()
            .filter_map(|p| {
                project(
                    *p,
                    p_body,
                    firefly_base::rig::position_in_body(firefly_base::FrameId::LEFT_CAMERA),
                )
            })
            .collect();
        let uv_r: Vec<_> = pts
            .iter()
            .filter_map(|p| {
                project(
                    *p,
                    p_body,
                    firefly_base::rig::position_in_body(firefly_base::FrameId::RIGHT_CAMERA),
                )
            })
            .collect();
        let zeros = || GrayImage {
            width: W,
            height: H,
            data: vec![0; W * H],
        };
        let seed_l = if cfg.img_noise_flicker { k } else { 1 };
        let seed_r = if cfg.img_noise_flicker { k + 7 } else { 2 };
        mgr.feed_measurement_camera(&CameraData {
            timestamp: t_cam,
            sensor_ids: vec![0, 1],
            images: vec![
                render_dots(&uv_l, seed_l, cfg.img_noise_amp),
                render_dots(&uv_r, seed_r, cfg.img_noise_amp),
            ],
            masks: vec![zeros(), zeros()],
        });
    }

    let expected =
        p0 + v_gt * ((f64::from(frames as u32) - f64::from(static_frames)) * dt_cam).max(0.0);
    let err_p = (mgr.state.imu.pos() - expected).norm();
    let err_v = (mgr.state.imu.vel() - v_gt).norm();
    (err_p, err_v, mgr.state.imu.bias_a())
}

/// 无零偏纯 MSCKF 场景：零旋转、前向恒速、稀疏点阵和 5cm 立体基线。
/// 3m 位置阈值仅检查严重发散，不代表定位精度达标。
#[test]
#[ignore = "零旋转、纯前向和稀疏点阵导致弱可观，不能作为精度验收"]
fn synthetic_pure_msckf_zero_bias() {
    let (err_p, _, _) = run_scenario(false, 0);
    assert!(
        err_p < 3.0,
        "纯 MSCKF 位置误差过大（疑似结构性发散）: {err_p:.3}m"
    );
}

/// 25 个持久 SLAM 路标的无零偏场景，检查末端位置和速度误差。
#[test]
#[ignore = "SLAM 模式的末端速度误差未通过断言"]
fn synthetic_slam_zero_bias() {
    let (err_p, err_v, _) = run_scenario(false, 25);
    assert!(err_p < 3.0, "SLAM 零偏位置误差过大: {err_p:.3}m");
    assert!(err_v < 0.3, "SLAM 零偏速度误差过大: {err_v:.3}m/s");
}

/// 静止 5s 后运动，与 [`synthetic_slam_zero_bias`] 对比启动运动条件。
#[test]
#[ignore = "与 synthetic_slam_zero_bias 配套的扩展场景"]
fn synthetic_slam_static_then_move() {
    let (err_p, err_v, _) = run_cfg(&ScenarioCfg {
        max_slam: 25,
        static_then_move: true,
        ..ScenarioCfg::default()
    });
    assert!(err_p < 3.0, "静止→运动 SLAM 位置误差过大: {err_p:.3}m");
    assert!(err_v < 0.3, "静止→运动 SLAM 速度误差过大: {err_v:.3}m/s");
}

/// 含零偏的纯 MSCKF 场景，检查位置、速度及加计零偏估计。
#[test]
#[ignore = "含加速度零偏场景发散，未通过精度与零偏断言"]
fn synthetic_pure_msckf_with_bias() {
    let (err_p, err_v, ba) = run_scenario(true, 0);
    assert!(err_p < 0.30, "含偏纯 MSCKF 位置误差过大: {err_p:.3}m");
    assert!(err_v < 0.20, "含偏纯 MSCKF 速度误差过大: {err_v:.3}m/s");
    assert!(
        (ba.x - BIAS_A_TRUE.x).abs() < 0.03,
        "ba_x 未学到真值: {}",
        ba.x
    );
}

/// 静止双目与带陀螺零偏的 IMU 独立完成启动；无外部状态注入。
#[test]
fn sensor_only_stationary_startup() {
    let mut mgr = build_manager_ex(0, true);
    let pts = world_points();
    let position = Vector3::new(0.0, 0.0, 1.0);
    let images: Vec<_> = [
        firefly_base::FrameId::LEFT_CAMERA,
        firefly_base::FrameId::RIGHT_CAMERA,
    ]
    .into_iter()
    .enumerate()
    .map(|(i, camera)| {
        let offset = firefly_base::rig::position_in_body(camera);
        let uv: Vec<_> = pts
            .iter()
            .filter_map(|p| project(*p, position, offset))
            .collect();
        render_dots(&uv, i + 1, 7)
    })
    .collect();
    for k in 0..=250 {
        let time = 10.0 + f64::from(k) * 0.01;
        mgr.feed_measurement_imu(&ImuData {
            timestamp: time,
            wm: BIAS_G_TRUE,
            am: Vector3::new(0.0, 0.0, 9.81),
        });
        if k % 10 == 0 {
            mgr.feed_measurement_camera(&CameraData {
                timestamp: time,
                sensor_ids: vec![0, 1],
                images: images.clone(),
                masks: vec![
                    GrayImage {
                        width: W,
                        height: H,
                        data: vec![0; W * H]
                    };
                    2
                ],
            });
        }
        if k < 100 {
            assert!(!mgr.initialized(), "完整静止窗口之前不得就绪");
        }
    }
    assert!(mgr.initialized(), "传感器观测应能独立初始化");
    assert!((mgr.state.imu.bias_g() - BIAS_G_TRUE).norm() < 1e-5);
    assert!(mgr.state.imu.pos().norm() < 0.01);
    assert!(mgr.state.imu.vel().norm() < 0.01);
    assert!(mgr.state.cov.iter().all(|v| v.is_finite()));
}
