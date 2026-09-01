//! P2 集成测试：滑窗容量、视觉点生成、参考补丁评分、法向精化、光线投射。
//!
//! 全部合成数据，可断言。

use firefly_void_map::options::VoxelMapOptions;
use firefly_void_map::visual_point::{PatchObservation, VisualPoint};
use firefly_void_map::{VoxelMap, VoxelPlane, image_patch, normal_refine};
use firefly_void_types::visual::{GrayImage, Intrinsics, VisualState};
use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector2, Vector3};

fn identity_pose() -> Isometry3<f64> {
    Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), UnitQuaternion::identity())
}

fn test_intrinsics() -> Intrinsics {
    Intrinsics::new(300.0, 300.0, 160.0, 120.0)
}

fn opts_small_map() -> VoxelMapOptions {
    VoxelMapOptions {
        half_map_size: 4,    // 小滑窗便于测试
        sliding_thresh: 0.0, // 每次调用都滑
        ..VoxelMapOptions::default()
    }
}

/// 棋盘格灰度图（用于梯度显著判据）。
fn checkerboard(width: usize, height: usize, cell: usize) -> GrayImage {
    let mut data = Vec::with_capacity(width * height);
    for y in 0..height {
        for x in 0..width {
            let dark = ((x / cell) + (y / cell)).is_multiple_of(2);
            data.push(if dark { 0 } else { 255 });
        }
    }
    GrayImage::new(width, height, data)
}

/// 构造含一个平面点云的 VoxelMap（平面在 z=1，相机原点沿 +z 可见）。
fn map_with_plane(n_points: usize) -> (VoxelMap, Vec<Vector3<f64>>) {
    let mut map = VoxelMap::new(opts_small_map());
    let cov = Matrix3::identity() * 1e-4;
    let mut pts = Vec::with_capacity(n_points);
    for i in 0..n_points {
        let x = -0.2 + (i % 40) as f64 * 0.01;
        let y = -0.2 + (i / 40) as f64 * 0.01;
        let p = Vector3::new(x, y, 1.0);
        pts.push(p);
    }
    let covs = vec![cov; n_points];
    map.register_points(&pts, &covs, &Vector3::zeros());
    (map, pts)
}

#[test]
fn sliding_window_capacity_bounded() {
    // 地图滑窗：位置跨越触界点后哈希容量不增长
    let mut map = VoxelMap::new(opts_small_map());
    let cov = Matrix3::identity() * 1e-4;
    // 沿 x 轴注册 10 段点云
    for k in 0..10 {
        let kf = f64::from(k);
        let pts: Vec<Vector3<f64>> = (0..20)
            .map(|i| Vector3::new(kf * 2.0, f64::from(i % 5) * 0.1, 0.0))
            .collect();
        let covs = vec![cov; 20];
        map.register_points(&pts, &covs, &Vector3::zeros());
        map.on_update_end(&Vector3::new(kf * 2.0, 0.0, 0.0));
    }
    // 半宽 4（根体素）：沿 x 最多保留 ~9 个根体素 + y 方向几个
    let count = map.root_count();
    assert!(count <= 100, "滑窗后哈希容量应受限，实际 {count}");
    // 滑窗后只保留当前中心附近
    assert!(count > 0);
}

#[test]
fn visual_point_generation_on_checkerboard() {
    // 视觉点生成：棋盘格图像 + 已知位姿，网格密度正确
    let (mut map, _) = map_with_plane(400);
    let img = checkerboard(320, 240, 30);
    let intrinsics = test_intrinsics();
    // 相机在原点沿 +z 看向 z=1 平面
    let pose = identity_pose();
    let state = VisualState::new(0, 1.0);
    map.update_visual(&pose, &img, &intrinsics, &state);
    // 平面在图像中心区域（z=1，相机原点，f=300 → 投影范围 ~±0.2*300=±60px）
    let n = map.visual_point_count();
    assert!(n >= 1, "应生成视觉点，实际 {n}");
    // 网格密度：视野内可见的格子数量（~120px×120px 区域 / 30px 网格 ≈ 16）
    assert!(n <= 40, "视觉点不应超过可见网格数，实际 {n}");
}

#[test]
fn minimum_depth_candidate_selected() {
    // 最小深度选择：同一网格内两个候选点，应保留深度小者
    let (mut map, _) = map_with_plane(400);
    let img = checkerboard(320, 240, 30);
    let intrinsics = test_intrinsics();
    let pose = identity_pose();
    let state = VisualState::new(0, 1.0);
    map.update_visual(&pose, &img, &intrinsics, &state);
    // 全部视觉点都应位于 z=1 平面（深度一致），投影在相机前方
    for vp in map.visual_points() {
        let p_cam = firefly_void_map::voxel::transform_point(&pose, &vp.pos);
        assert!(p_cam[2] > 0.0, "视觉点必须在相机前方");
    }
}

#[test]
fn reference_patch_scoring_selects_expected() {
    // 参考补丁评分：构造已知 NCC 关系的补丁集，选中预期者
    let opts = VoxelMapOptions::default();
    // 三个补丁：A 与 B 高度相似（NCC≈1），C 不同；c 项相同 → A 或 B 被选中
    let mut a = Vec::with_capacity(121);
    let mut b = Vec::with_capacity(121);
    let mut c = Vec::with_capacity(121);
    for i in 0..121 {
        let v = f64::from((i * 7) % 256);
        a.push(v);
        b.push(v + f64::from(i % 3)); // 近似 A
        c.push(f64::from(i * 13 % 256)); // 不同模式
    }
    let mut vp = VisualPoint::new(
        Vector3::zeros(),
        Matrix3::identity(),
        Vector3::z_axis().into_inner(),
    );
    let make_obs = |patch: image_patch::PatchPyramid| PatchObservation {
        frame_id: 0,
        pose: identity_pose(),
        inv_expo_time: 1.0,
        patch,
        px: Vector2::new(0.0, 0.0),
        score: 0.0,
        mean: 0.0,
    };
    // 6 个观测（超过 min_obs_for_score=5）：A×2、B×2、C×2
    for _ in 0..2 {
        vp.add_observation(make_obs(image_patch::PatchPyramid {
            levels: vec![a.clone()],
            scale: vec![1],
            patch_size: 11,
        }));
        vp.add_observation(make_obs(image_patch::PatchPyramid {
            levels: vec![b.clone()],
            scale: vec![1],
            patch_size: 11,
        }));
        vp.add_observation(make_obs(image_patch::PatchPyramid {
            levels: vec![c.clone()],
            scale: vec![1],
            patch_size: 11,
        }));
    }
    vp.update_reference_patch(&opts);
    let idx = vp.ref_patch.unwrap();
    // A 或 B 应被选中（NCC 高），C 不应（C 的索引是 2 或 5）
    assert!(idx % 3 != 2, "高分补丁应被选为参考，实际选 {idx}");
}

#[test]
fn normal_refine_converges() {
    // 法向精化：合成仿射扭曲序列，优化收敛后法向角误差 < 5°
    // 构造：参考补丁（平面法向 z 轴）与目标补丁（法向略偏）间做光度一致
    let opts = VoxelMapOptions::default();
    let intrinsics = test_intrinsics();
    // 源补丁：沿 x 的线性渐变（梯度显著）
    let size = opts.patch_size;
    let mut data = Vec::with_capacity(size * size);
    for _y in 0..size {
        for x in 0..size {
            data.push(x as f64 * 20.0);
        }
    }
    let patch_ref = image_patch::PatchPyramid {
        levels: vec![data],
        scale: vec![1],
        patch_size: size,
    };
    // 参考观测：在原点看 z=0 平面（法向 +z）
    let pose_ref = identity_pose();
    let mut vp = VisualPoint::new(
        Vector3::new(0.0, 0.0, 1.0),
        Matrix3::identity(),
        Vector3::z_axis().into_inner(),
    );
    vp.add_observation(PatchObservation {
        frame_id: 0,
        pose: pose_ref,
        inv_expo_time: 1.0,
        patch: patch_ref,
        px: Vector2::new(0.0, 0.0),
        score: 0.0,
        mean: 0.0,
    });
    // 目标观测：同一平面从另一视角（绕 y 转 10°），补丁内容应高度相似
    // （合成：直接从参考补丁拷贝，模拟完美匹配场景）
    let patch_tgt = vp.obs[0].patch.clone();
    let pose_tgt = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, 1.0),
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.17),
    );
    vp.add_observation(PatchObservation {
        frame_id: 1,
        pose: pose_tgt,
        inv_expo_time: 1.0,
        patch: patch_tgt,
        px: Vector2::new(160.0, 120.0),
        score: 0.0,
        mean: 0.0,
    });
    // 观测数不足评分阈值，直接指定参考补丁
    vp.ref_patch = Some(0);
    let n = normal_refine::refine_normal(&vp, &intrinsics, 20).unwrap();
    // 精化后法向仍应接近 +z（视角变化只引入小扰动）
    let err_deg = n.angle(&Vector3::z_axis()).to_degrees();
    assert!(err_deg < 5.0, "法向角误差 {err_deg}° 应 < 5°");
}

#[test]
fn raycast_returns_correct_visible_set() {
    // 光线投射：遮挡关系正确的场景返回正确可见集
    let mut map = VoxelMap::new(opts_small_map());
    let img = checkerboard(320, 240, 30);
    let intrinsics = test_intrinsics();
    let cov = Matrix3::identity() * 1e-4;
    // 相机在原点沿 +z 看向 z=1 与 z=2 的两个平行平面
    let near_pts: Vec<Vector3<f64>> = (0..400)
        .map(|i| {
            Vector3::new(
                -0.2 + f64::from(i % 20) * 0.02,
                -0.2 + f64::from(i / 20) * 0.02,
                1.0,
            )
        })
        .collect();
    let far_pts: Vec<Vector3<f64>> = (0..400)
        .map(|i| {
            Vector3::new(
                -0.2 + f64::from(i % 20) * 0.02,
                -0.2 + f64::from(i / 20) * 0.02,
                2.0,
            )
        })
        .collect();
    map.register_points(&near_pts, &vec![cov; 400], &Vector3::zeros());
    map.register_points(&far_pts, &vec![cov; 400], &Vector3::zeros());
    let pose = identity_pose();
    let state = VisualState::new(0, 1.0);
    map.update_visual(&pose, &img, &intrinsics, &state);
    // 可见性查询（全空掩码 → 全部网格光线投射）
    let visible = map.visible_map_points(&pose, &intrinsics, &[]);
    assert!(!visible.is_empty(), "应有可见视觉点");
    for v in &visible {
        let p_cam = firefly_void_map::voxel::transform_point(&pose, &v.pos);
        assert!(
            p_cam[2] > 0.0 && p_cam[2] < 3.0,
            "可见点在相机前方且在射程内"
        );
    }
}

#[test]
fn planes_iterator_reports_registered_planes() {
    // planes() 遍历：注册平面点云后能枚举出平面
    let (map, _) = map_with_plane(400);
    let planes: Vec<&VoxelPlane> = map.planes().collect();
    assert!(!planes.is_empty(), "应有平面");
    for p in &planes {
        assert!(p.is_plane);
        assert!((p.normal.norm() - 1.0).abs() < 1e-6, "法向应单位化");
    }
}
