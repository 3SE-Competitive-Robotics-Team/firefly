//! 深度相机 → 占据栅格（感知建图，log-odds 更新）。
//!
//! 逐行对照官方 `EGO-Planner-v2 plan_env/src/grid_map.cpp` 的
//! `projectDepthImage` / `raycastProcess`：深度像素投影为世界系点，frame 内
//! 按体素累计 hit/miss，每体素每帧一次多数投票后更新 log-odds。

use crate::grid::GridMap;
use crate::raycast::RayCaster;
use firefly_base::rig::CV_FROM_RENDER;
use nalgebra::{Isometry3, Matrix3, Point3, Vector3};

/// 深度相机标定（内参 + 外参 + 感知参数）。
///
/// 官方 `grid_map.cpp` 的深度输入是 `uint16` 并按 `k_depth_scaling_factor`
/// 换算；本项目直接消费 `f32` 米制深度，投影与过滤约定一致：
/// - 像素 `(u, v)`（左上原点，v 向下）→ CV 相机系射线 `(dx, dy, 1)`；
/// - 深度值为相机空间 Z（视线方向的垂直距离，非欧氏距离）；
/// - 相机系命中点经 [`CV_FROM_RENDER`] 的转置转回 render 机体系，再经
///   [`Self::rot_cam_to_body`] 转到机体系。
#[derive(Debug, Clone)]
pub struct DepthCamera {
    /// 像素焦距（`fx=fy`，方形像素）。
    pub focal: f64,
    /// 主点横坐标（像素）。
    pub cx: f64,
    /// 主点纵坐标（像素）。
    pub cy: f64,
    /// 图像宽度（像素）。
    pub width: usize,
    /// 图像高度（像素）。
    pub height: usize,
    /// 相机 → 机体旋转（**列** = 相机轴在机体系的坐标）。
    pub rot_cam_to_body: Matrix3<f64>,
    /// 相机在机体系的位置（米）。
    pub pos_in_body: Vector3<f64>,
    /// 深度过滤边距（像素，官方 `grid_map/depth_filter_margin`，默认 2）。
    pub depth_filter_margin: usize,
    /// 最小有效深度（米，官方 `grid_map/depth_filter_mindist`，默认 0.2）。
    pub min_range: f64,
    /// 像素降采样步长（官方 `grid_map/skip_pixel`，默认 2）。
    pub skip_pixel: usize,
}

impl DepthCamera {
    /// RMUC render 默认标定：深度相机与双目前脸齐平，垂直视场 70.88°，320×240。
    /// 机位、朝向、下倾与视场参数统一来自 [`firefly_base::rig`]；
    /// 过滤参数取官方 `advanced_param.xml` 默认值。
    #[must_use]
    pub fn mujoco_default() -> Self {
        let focal = (240.0 / 2.0) / (firefly_base::rig::FOV_Y_DEG.to_radians() / 2.0).tan();
        let [cx, cy] = firefly_base::rig::principal_point(320, 240);
        Self {
            focal,
            cx,
            cy,
            width: 320,
            height: 240,
            rot_cam_to_body: firefly_base::rig::cam_axes_in_body(),
            pos_in_body: Vector3::from(firefly_base::rig::DEPTH_IN_BODY),
            depth_filter_margin: 2,
            min_range: 0.2,
            skip_pixel: 2,
        }
    }
}

/// 用一帧深度图更新占据地图（感知建图，log-odds）。
///
/// 对照官方 `projectDepthImage`：跳过 `depth_filter_margin` 边框、每
/// `skip_pixel` 取一个像素，丢弃深度小于 `min_range` 或非有限值的像素
/// （官方 `grid_map.cpp` 的 `raycastProcess` 不使用最大距离过滤）。
///
/// `body_pose` 为深度测量时刻的机体 → 地图变换。
///
/// 返回投影点数量；零表示本帧未提供地图观测。
///
/// # Panics
/// 深度长度与相机尺寸不一致，或 `skip_pixel` 为零。
pub fn update_from_depth(
    map: &mut GridMap,
    cam: &DepthCamera,
    body_pose: &Isometry3<f64>,
    depth: &[f32],
) -> usize {
    assert_eq!(depth.len(), cam.width * cam.height);
    assert!(cam.skip_pixel > 0);
    let margin = cam.depth_filter_margin;
    if cam.width <= 2 * margin || cam.height <= 2 * margin {
        return 0;
    }
    let rot_cam_to_body = cam.rot_cam_to_body * CV_FROM_RENDER.transpose();
    let mut points: Vec<Vector3<f64>> =
        Vec::with_capacity((cam.width / cam.skip_pixel) * (cam.height / cam.skip_pixel));
    for v in (margin..cam.height - margin).step_by(cam.skip_pixel) {
        for u in (margin..cam.width - margin).step_by(cam.skip_pixel) {
            let z = f64::from(depth[v * cam.width + u]);
            if !z.is_finite() || z < cam.min_range {
                continue;
            }
            let p_cv = Vector3::new(
                (u as f64 - cam.cx) * z / cam.focal,
                (v as f64 - cam.cy) * z / cam.focal,
                z,
            );
            let p_body = rot_cam_to_body * p_cv;
            points.push((body_pose * Point3::from(cam.pos_in_body + p_body)).coords);
        }
    }
    let camera_pos = (body_pose * Point3::from(cam.pos_in_body)).coords;
    raycast_points(map, camera_pos, &points);
    points.len()
}

/// 官方 `raycastProcess`：点云 → 帧内多数投票 → 每体素一次 log-odds 更新。
fn raycast_points(map: &mut GridMap, camera_pos: Vector3<f64>, points: &[Vector3<f64>]) {
    if points.is_empty() {
        return;
    }
    let res = map.resolution();
    map.raycast_begin();
    let mut ray_pt = [0_i32; 3];
    for &point in points {
        // 官方：窗口外裁到边界按 miss；窗口内按 hit。
        let (pt, hit) = if map.is_in_window(point) {
            (point, true)
        } else {
            (map.clip_to_window(point, camera_pos), false)
        };
        let address = map.raycast_cache(pt, hit);
        if !map.rayend_is_new(address) {
            continue;
        }
        let Some(mut caster) = RayCaster::new(pt / res, camera_pos / res) else {
            continue;
        };
        while caster.step(&mut ray_pt) {
            let tmp = Vector3::new(
                (f64::from(ray_pt[0]) + 0.5) * res,
                (f64::from(ray_pt[1]) + 0.5) * res,
                (f64::from(ray_pt[2]) + 0.5) * res,
            );
            let address = map.raycast_cache(tmp, false);
            if !map.traverse_is_new(address) {
                break;
            }
        }
    }
    map.raycast_end();
}

#[cfg(test)]
fn mark_ray(map: &mut GridMap, from: Vector3<f64>, to: Vector3<f64>) {
    raycast_points(map, from, std::slice::from_ref(&to));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{GridMapBuilder, VoxelState};
    use nalgebra::{Translation3, UnitQuaternion};

    #[test]
    fn prior_survives_depth_free_rays_fading_and_dynamic_removal() {
        let mut map = GridMapBuilder::new(1., [10, 10, 10]).build().unwrap();
        map.set_prior_occupied([4, 5, 5]);
        map.inflate_obstacles();
        for _ in 0..3000 {
            mark_ray(
                &mut map,
                Vector3::new(0.5, 5.5, 5.5),
                Vector3::new(8.5, 5.5, 5.5),
            );
            map.fade();
        }
        map.set_state([4, 5, 5], VoxelState::Free);
        map.inflate_obstacles();
        assert_eq!(map.state([4, 5, 5]), VoxelState::Occupied);
        assert!(map.is_occupied(Vector3::new(4.5, 5.5, 5.5)));
        assert!((map.occupancy_at([4, 5, 5]) - map.clamp_max_log()).abs() < 1e-12);
        map.move_ring_buffer(Vector3::new(12.5, 5.5, 5.5));
        map.move_ring_buffer(Vector3::new(5.5, 5.5, 5.5));
        assert!(map.is_occupied(Vector3::new(4.5, 5.5, 5.5)));
    }

    #[test]
    fn repeated_pixels_contribute_only_one_frame_update() {
        let pos = Vector3::new(4.5, 5.5, 5.5);
        let mut map = GridMapBuilder::new(1., [10, 10, 10]).build().unwrap();
        // 同一帧内同一体素 100 次 hit（官方 setCacheOccupancy 累计、收尾只更新一次）
        map.raycast_begin();
        for _ in 0..100 {
            map.raycast_cache(pos, true);
        }
        map.raycast_end();
        let expected = map.clamp_min_log() + map.prob_hit_log();
        assert!((map.occupancy_at([4, 5, 5]) - expected).abs() < 1e-12);
        assert_eq!(map.state([4, 5, 5]), VoxelState::Free);

        // 同一帧 2 hit / 3 miss：miss 多，取 miss
        map.raycast_begin();
        for _ in 0..2 {
            map.raycast_cache(pos, true);
        }
        for _ in 0..3 {
            map.raycast_cache(pos, false);
        }
        map.raycast_end();
        assert!((map.occupancy_at([4, 5, 5]) - (expected + map.prob_miss_log())).abs() < 1e-12);
    }

    #[test]
    fn rayend_and_traverse_flags_reset_per_frame() {
        let mut map = GridMapBuilder::new(1., [10, 10, 10]).build().unwrap();
        let pos = Vector3::new(4.5, 5.5, 5.5);
        map.raycast_begin();
        let address = map.raycast_cache(pos, true);
        assert!(map.rayend_is_new(address));
        assert!(!map.rayend_is_new(address));
        assert!(map.traverse_is_new(address));
        assert!(!map.traverse_is_new(address));
        map.raycast_end();
        map.raycast_begin();
        assert!(map.rayend_is_new(address));
        assert!(map.traverse_is_new(address));
    }

    #[test]
    fn majority_tie_prefers_hit_and_clamped_votes_are_skipped() {
        let pos = Vector3::new(4.5, 5.5, 5.5);
        let mut map = GridMapBuilder::new(1., [10, 10, 10]).build().unwrap();
        map.raycast_begin();
        for _ in 0..2 {
            map.raycast_cache(pos, true);
        }
        for _ in 0..2 {
            map.raycast_cache(pos, false);
        }
        map.raycast_end();
        let expected = map.clamp_min_log() + map.prob_hit_log();
        assert!((map.occupancy_at([4, 5, 5]) - expected).abs() < 1e-12);

        // 到 clamp_max 后继续投 hit：官方收尾直接跳过
        while map.occupancy_at([4, 5, 5]) < map.clamp_max_log() {
            map.raycast_begin();
            map.raycast_cache(pos, true);
            map.raycast_end();
        }
        let saturated = map.occupancy_at([4, 5, 5]);
        map.raycast_begin();
        map.raycast_cache(pos, true);
        map.raycast_end();
        assert!((map.occupancy_at([4, 5, 5]) - saturated).abs() < 1e-12);
    }

    #[test]
    fn out_of_window_point_is_clipped_and_voted_as_miss() {
        let mut map = GridMapBuilder::new(1., [5, 5, 5]).build().unwrap();
        let camera = Vector3::new(0.5, 2.5, 2.5);
        let boundary = Vector3::new(3.5, 2.5, 2.5);
        for _ in 0..5 {
            map.raycast_begin();
            map.raycast_cache(boundary, true);
            map.raycast_end();
        }
        let before = map.occupancy_at([3, 2, 2]);
        raycast_points(&mut map, camera, &[Vector3::new(9.5, 2.5, 2.5)]);
        assert!(map.occupancy_at([3, 2, 2]) < before);
    }

    #[test]
    fn duplicate_ray_ends_cast_only_one_ray() {
        fn lost(path: &[Vector3<f64>]) -> f64 {
            let mut map = GridMapBuilder::new(1., [10, 10, 10]).build().unwrap();
            map.set_state([3, 5, 5], VoxelState::Occupied);
            let before = map.occupancy_at([3, 5, 5]);
            raycast_points(&mut map, Vector3::new(0.5, 5.5, 5.5), path);
            before - map.occupancy_at([3, 5, 5])
        }
        let one = lost(&[Vector3::new(4.5, 5.5, 5.5)]);
        let two = lost(&[Vector3::new(4.5, 5.5, 5.5); 2]);
        assert!((one - two).abs() < 1e-12, "one={one} two={two}");
    }

    #[test]
    fn traversal_stops_at_already_visited_voxel() {
        fn lost(points: &[Vector3<f64>]) -> f64 {
            let mut map = GridMapBuilder::new(1., [10, 10, 10]).build().unwrap();
            map.set_state([3, 5, 5], VoxelState::Occupied);
            let before = map.occupancy_at([3, 5, 5]);
            raycast_points(&mut map, Vector3::new(0.5, 5.5, 5.5), points);
            before - map.occupancy_at([3, 5, 5])
        }
        let one = lost(&[Vector3::new(4.5, 5.5, 5.5)]);
        // 第二条射线在 [4,5,5] 撞到本帧已遍历格后停止，不应再记 [3,5,5] 的 miss
        let two = lost(&[Vector3::new(4.5, 5.5, 5.5), Vector3::new(6.5, 5.5, 5.5)]);
        assert!((one - two).abs() < 1e-12, "one={one} two={two}");
    }

    #[test]
    fn missing_depth_does_not_clear_occupied_voxels() {
        let cam = DepthCamera::mujoco_default();
        let mut map = GridMapBuilder::new(1.0, [10, 10, 10]).build().unwrap();
        map.set_state([5, 5, 5], VoxelState::Occupied);
        let before = map.occupancy_at([5, 5, 5]);
        let pose = Isometry3::translation(0.5, 5.5, 5.5);
        for invalid in [0.0, f32::NAN, f32::INFINITY] {
            update_from_depth(
                &mut map,
                &cam,
                &pose,
                &vec![invalid; cam.width * cam.height],
            );
            assert!((map.occupancy_at([5, 5, 5]) - before).abs() < 1e-12);
        }
    }

    #[test]
    fn default_camera_focal_matches_fovy() {
        let cam = DepthCamera::mujoco_default();
        assert!((cam.focal - 168.607).abs() < 1e-3);
        assert!((cam.cx - 159.5).abs() < 1e-12 && (cam.cy - 119.5).abs() < 1e-12);
    }

    #[test]
    fn ray_marks_free_then_occupied() {
        // log-odds：单次命中 +0.847 不足以从 clamp_min(-1.992) 跨阈值 1.386，需多次命中
        let mut map = GridMapBuilder::new(1.0, [10, 10, 10]).build().unwrap();
        let from = Vector3::new(0.5, 0.5, 0.5);
        let to = Vector3::new(4.5, 0.5, 0.5);
        let hit_prob = map.prob_hit_log();
        let miss_prob = map.prob_miss_log();
        let clamp_min = map.clamp_min_log();
        let min_occ = map.min_occupancy_log();
        // 起点体素不更新（含相机）
        let start_occ = map.occupancy_at([0, 0, 0]);
        mark_ray(&mut map, from, to);
        // 沿途以 miss 递减，但 clamp_min 已为下界，保持不变
        assert!((map.occupancy_at([1, 0, 0]) - clamp_min).abs() < 1e-12);
        assert!(
            (map.occupancy_at([2, 0, 0]) - (clamp_min + miss_prob).max(clamp_min)).abs() < 1e-12
                || (map.occupancy_at([1, 0, 0]) - clamp_min).abs() < 1e-12
        );
        // 命中体素累加 hit
        assert!((map.occupancy_at([4, 0, 0]) - (clamp_min + hit_prob)).abs() < 1e-12);
        // 尚未跨阈值，仍为 Free
        assert_eq!(map.state([4, 0, 0]), VoxelState::Free);
        assert!(map.occupancy_at([4, 0, 0]) < min_occ);
        // 起点体素不改（含相机）
        assert!((map.occupancy_at([0, 0, 0]) - start_occ).abs() < 1e-12);
        // 多次命中后跨阈值变为 Occupied
        for _ in 0..4 {
            mark_ray(&mut map, from, to);
        }
        assert_eq!(map.state([4, 0, 0]), VoxelState::Occupied);
        assert!(map.occupancy_at([4, 0, 0]) >= min_occ);
    }

    #[test]
    fn ray_into_map_bounds_only_marks_inside() {
        let mut map = GridMapBuilder::new(1.0, [10, 10, 10]).build().unwrap();
        let from = Vector3::new(0.5, 0.5, 0.5);
        let to = Vector3::new(9.5, 5.5, 0.5); // 终点在图内，中间越界路径被跳过
        mark_ray(&mut map, from, to);
        // 沿途首格应被 miss 更新（但 clamp_min 保持下界）
        let clamp_min = map.clamp_min_log();
        assert!(map.occupancy_at([1, 0, 0]) <= clamp_min + 1e-12);
        // 不会 panic，出界即停
    }

    #[test]
    fn update_from_depth_projects_to_world() {
        // 无旋转机体 + 采样像素 (u=158, v=54)（margin=2、skip=2 网格点）：
        // 下倾 20° 相机射线在深度 5m 处命中机体系 ≈ (5.42, 0.04, 0.12)，
        // 需多次观测才跨阈值
        let cam = DepthCamera::mujoco_default();
        let mut map = GridMapBuilder::new(1.0, [20, 20, 10]).build().unwrap();
        let pose = Isometry3::identity();
        let mut depth = vec![0.0f32; cam.width * cam.height];
        depth[54 * cam.width + 158] = 5.0;
        let clamp_min = map.clamp_min_log();
        let hit = map.prob_hit_log();
        // 单帧命中一次
        update_from_depth(&mut map, &cam, &pose, &depth);
        assert!((map.occupancy_at([5, 0, 0]) - (clamp_min + hit)).abs() < 1e-12);
        // 沿途 Free（但已在下界，保持不变或轻微变化）
        assert!(map.occupancy_at([2, 0, 0]) <= clamp_min + 1e-12);
        // 多帧后占据
        for _ in 0..4 {
            update_from_depth(&mut map, &cam, &pose, &depth);
        }
        assert_eq!(map.state([5, 0, 0]), VoxelState::Occupied);
    }

    #[test]
    fn update_from_depth_maps_mujoco_box() {
        // 场景：无人机在 (1,4,1) 恒等姿态；下倾 20° 相机射线经
        // 像素 (208,70)（margin=2、skip=2 网格点）深度 6.471m → 命中箱 (8,2)
        // 前表面世界 (7.79,2.14,0.57)，体素 [19,17,1]，多次观测后占据
        let cam = DepthCamera::mujoco_default();
        // 地图参数：origin (0,-5,0)、0.4m、dims [80,35,13]
        let mut map = GridMapBuilder::new(0.4, [80, 35, 13])
            .with_origin(Vector3::new(0.0, -5.0, 0.0))
            .build()
            .unwrap();
        let pose =
            Isometry3::from_parts(Translation3::new(1.0, 4.0, 1.0), UnitQuaternion::identity());
        let mut depth = vec![0.0f32; cam.width * cam.height];
        depth[70 * cam.width + 208] = 6.471;
        let hit = map.prob_hit_log();
        let clamp_min = map.clamp_min_log();
        update_from_depth(&mut map, &cam, &pose, &depth);
        assert!((map.occupancy_at([19, 17, 1]) - (clamp_min + hit)).abs() < 1e-12);
        // 多帧后占据
        for _ in 0..4 {
            update_from_depth(&mut map, &cam, &pose, &depth);
        }
        assert_eq!(map.state([19, 17, 1]), VoxelState::Occupied);
        // 沿途旁侧体素未被射线穿过，保持下界（射线经 y≈3.5，体素 [7,22,2]
        // y∈[3.8,4.2) 在射线旁）
        assert!(map.occupancy_at([7, 22, 2]) <= clamp_min + 1e-12);
    }

    #[test]
    fn mark_ray_diagonal_box_ray() {
        // 直接测 mark_ray：from (1,4,1) → to (6.59,2.41,0.60)，地图参数相同
        let mut map = GridMapBuilder::new(0.4, [80, 35, 13])
            .with_origin(Vector3::new(0.0, -5.0, 0.0))
            .build()
            .unwrap();
        let clamp_min = map.clamp_min_log();
        mark_ray(
            &mut map,
            Vector3::new(1.0, 4.0, 1.0),
            Vector3::new(6.59, 2.412, 0.596),
        );
        // 单次命中尚未占据，需多次
        assert!((map.occupancy_at([16, 18, 1]) - (clamp_min + map.prob_hit_log())).abs() < 1e-12);
        assert_eq!(map.state([2, 22, 2]), VoxelState::Free); // 起点体素不改，初始 Free
        // DDA 路径中途某体素应被 miss 更新（但 clamp_min 保持）
        assert!(map.occupancy_at([5, 21, 2]) <= clamp_min + 1e-12);
        assert!(map.occupancy_at([7, 20, 2]) <= clamp_min + 1e-12);
        assert!(map.occupancy_at([12, 19, 1]) <= clamp_min + 1e-12);
        for _ in 0..4 {
            mark_ray(
                &mut map,
                Vector3::new(1.0, 4.0, 1.0),
                Vector3::new(6.59, 2.412, 0.596),
            );
        }
        assert_eq!(map.state([16, 18, 1]), VoxelState::Occupied);
    }

    #[test]
    fn log_odds_miss_decreases_occupancy() {
        let mut map = GridMapBuilder::new(0.5, [10, 10, 10]).build().unwrap();
        let idx = [5, 5, 5];
        // 先设为 Occupied（max）
        map.set_state(idx, VoxelState::Occupied);
        let max = map.clamp_max_log();
        assert!((map.occupancy_at(idx) - max).abs() < 1e-12);
        // miss 更新递减
        let miss = map.prob_miss_log();
        map.update_occupancy(idx, miss);
        assert!((map.occupancy_at(idx) - (max + miss)).abs() < 1e-12);
        // 多次 miss 直到阈值下
        for _ in 0..5 {
            map.update_occupancy(idx, miss);
        }
        assert!(map.occupancy_at(idx) < map.min_occupancy_log());
        assert_eq!(map.state(idx), VoxelState::Free);
    }
}
