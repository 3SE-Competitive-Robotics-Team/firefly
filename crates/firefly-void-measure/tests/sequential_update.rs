//! P3 集成测试：两测量模型经 esikf 顺序更新在合成序列上收敛到真值。
//!
//! 构造：相机静止，深度平面 z=1（法向 +z）+ 棋盘格参考帧，真值位姿
//! 绕 y 转 2°、平移 2cm；先深度更新再视觉更新（论文 Algorithm 1 的
//! 顺序更新顺序），迭代数轮后状态收敛。

use firefly_void_esikf::update::{EskfUpdater, depth_convergence, visual_convergence};
use firefly_void_map::VoxelMap;
use firefly_void_map::options::VoxelMapOptions;
use firefly_void_measure::{DepthMeasurement, DepthOptions, VisualMeasurement, VisualOptions};
use firefly_void_types::state::State;
use firefly_void_types::visual::{GrayImage, Intrinsics};
use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};

fn intrinsics() -> Intrinsics {
    Intrinsics::new(300.0, 300.0, 160.0, 120.0)
}

fn opts() -> VoxelMapOptions {
    VoxelMapOptions::default()
}

/// 合成平滑灰度图（连续梯度，直接对齐所需）。
fn smooth_image(width: usize, height: usize) -> GrayImage {
    let mut data = Vec::with_capacity(width * height);
    for y in 0..height {
        for x in 0..width {
            let u = x as f64 * 0.06;
            let v = y as f64 * 0.08;
            let value =
                120.0 + 60.0 * (u.sin() * v.cos() + 0.5 * (2.0 * u).sin() * (3.0 * v).sin());
            data.push(value.round().clamp(0.0, 255.0) as u8);
        }
    }
    GrayImage::new(width, height, data)
}

/// 世界系平面 z=1 上的深度点（相机系 = 世界系，共面）。
fn depth_points_on_plane() -> Vec<Vector3<f64>> {
    (0..64)
        .map(|i| {
            let x = -0.2 + f64::from(i % 8) * 0.05;
            let y = -0.2 + f64::from(i / 8) * 0.05;
            Vector3::new(x, y, 1.0)
        })
        .collect()
}

/// 世界系平面 z=1 上的视觉点（法向 +z 面向相机）。
fn visual_points(
    img: &GrayImage,
    intrinsics: &Intrinsics,
    pose: &Isometry3<f64>,
) -> Vec<firefly_void_map::visual_point::VisualPointView> {
    use firefly_void_map::image_patch::PatchPyramid;
    use firefly_void_map::visual_point::VisualPointView;
    use firefly_void_map::voxel::transform_point;
    let ps = 11usize;
    let half = (ps as i64) / 2;
    let mut out = Vec::new();
    for i in 0..16 {
        let x = -0.1 + f64::from(i % 4) * 0.06;
        let y = -0.1 + f64::from(i / 4) * 0.06;
        let pos = Vector3::new(x, y, 1.0);
        let p_cam = transform_point(pose, &pos);
        let px = intrinsics.project(&p_cam).unwrap();
        let mut levels = Vec::with_capacity(3);
        for level in 0..3usize {
            let scale = 1usize << level;
            let mut data = Vec::with_capacity(ps * ps);
            for yy in 0..ps {
                for xx in 0..ps {
                    let u = px[0] + (xx as i64 - half) as f64 * scale as f64;
                    let v = px[1] + (yy as i64 - half) as f64 * scale as f64;
                    data.push(
                        firefly_void_measure::visual_update::bilinear_sample(img, u, v)
                            .unwrap_or(0.0),
                    );
                }
            }
            levels.push(data);
        }
        let patch = PatchPyramid {
            levels,
            scale: vec![1, 2, 4],
            patch_size: ps,
        };
        out.push(VisualPointView {
            pos,
            normal: Vector3::z_axis().into_inner(),
            ref_patch: patch,
            ref_pose: *pose,
            ref_inv_expo: 1.0,
            px,
        });
    }
    out
}

#[test]
fn sequential_depth_then_visual_converges() {
    // 真值位姿
    let truth = Isometry3::from_parts(
        Translation3::new(0.02, -0.01, 0.005),
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.02),
    );
    // 地图：平面 z=1
    let mut map = VoxelMap::new(opts());
    let cov = Matrix3::identity() * 1e-8;
    let mut pts = Vec::new();
    for i in 0..400 {
        let x = -0.2 + f64::from(i % 20) * 0.02;
        let y = -0.2 + f64::from(i / 20) * 0.02;
        pts.push(Vector3::new(x, y, 1.0));
    }
    map.register_points(&pts, &vec![cov; 400], &Vector3::zeros());

    // 参考帧（世界系 = 相机系，单位阵位姿）
    let ref_img = smooth_image(320, 240);
    let ref_pose = Isometry3::identity();
    let vis_pts = visual_points(&ref_img, &intrinsics(), &ref_pose);

    // 当前帧：真值位姿下的重渲染（曝光 1.15）
    let exposure = 1.15;
    let mut cur_data = Vec::with_capacity(320 * 240);
    let intrinsics = intrinsics();
    for v in 0..240 {
        for u in 0..320 {
            let ray = Vector3::new(
                (f64::from(u) - intrinsics.cx) / intrinsics.fx,
                (f64::from(v) - intrinsics.cy) / intrinsics.fy,
                1.0,
            );
            let t = 1.0 / ray[2]; // 平面 z=1
            let p_cam = ray * t;
            let p_w = truth.inverse() * p_cam;
            let value = firefly_void_measure::visual_update::bilinear_sample(
                &ref_img,
                intrinsics.cx + intrinsics.fx * p_w[0] / p_w[2],
                intrinsics.cy + intrinsics.fy * p_w[1] / p_w[2],
            )
            .unwrap_or(60.0);
            cur_data.push((value * exposure).round().clamp(0.0, 255.0) as u8);
        }
    }
    let cur_img = GrayImage::new(320, 240, cur_data);

    // 深度点：真值位姿下的相机系点（世界系平面点变换到真值相机系）
    let depth_pts: Vec<Vector3<f64>> = depth_points_on_plane()
        .iter()
        .map(|p| truth.inverse() * p)
        .collect();
    let _depth_covs = vec![Matrix3::identity() * 1e-6; depth_pts.len()];

    // 顺序更新（多轮）：深度 → 视觉
    let mut x = State::default();
    let mut iters_total = 0;
    for _round in 0..3 {
        let mut d_updater = EskfUpdater::new(
            DepthMeasurement::new(
                &map,
                depth_points_on_plane(),
                vec![Matrix3::identity() * 1e-6; 64],
                Isometry3::identity(),
                DepthOptions::default(),
            ),
            5,
            depth_convergence(),
        );
        let (iters, _) = d_updater.update(&mut x).unwrap();
        iters_total += iters;

        let warps = VisualMeasurement::compute_warps(&vis_pts, &x, &intrinsics);
        let warp_patches = VisualMeasurement::compute_warp_patches(&vis_pts, &warps, 11, 0);
        let vis_model = VisualMeasurement::new(
            &cur_img,
            vis_pts.clone(),
            warp_patches,
            intrinsics,
            VisualOptions {
                pyramid_level: 1,
                max_iterations: 10,
                convergence_eps: 1e-6,
                ..VisualOptions::default()
            },
            0,
        );
        let mut v_updater = EskfUpdater::new(vis_model, 10, visual_convergence());
        let (iters, _) = v_updater.update(&mut x).unwrap();
        iters_total += iters;
    }
    assert!(iters_total > 0);
    let rot_err = UnitQuaternion::from_rotation_matrix(&x.rot).angle_to(&truth.rotation);
    let pos_err = (x.pos - truth.translation.vector).norm();
    assert!(
        rot_err.to_degrees() < 2.0,
        "旋转误差 {:.3}° 应 < 2°",
        rot_err.to_degrees()
    );
    assert!(pos_err < 0.03, "位置误差 {pos_err} m 应 < 3cm");
}
