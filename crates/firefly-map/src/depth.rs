//! 深度相机 → 占据栅格（感知建图，log-odds 更新）。
//!
//! 把一帧深度图投影成世界系点云并做光线标记：相机原点 → 命中点射线沿途
//! 体素以 `prob_miss_log` 递减（趋向 Free），命中体素以 `prob_hit_log` 递增
//! （趋向 Occupied），对照官方 `grid_map.cpp:577-700` `raycastProcess` 的
//! log-odds 更新语义。对照差异见 `mark_ray` 注释。
//!
//! 相机模型为硬编码合成标定（`MuJoCo` 场景，`packages/firefly-mujoco/src/
//! firefly_mujoco/scene.py`），投影约定经实测验证（见 `DepthCamera::mujoco_default`
//! 文档）。

use nalgebra::{Isometry3, Matrix3, Point3, Vector3};

use crate::grid::GridMap;

/// 深度相机标定（内参 + 外参 + 感知参数）。
///
/// 投影模型（`MuJoCo` 深度相机实测，`verify_cam*.py` 实证）：
/// - 像素 (u, v)（左上原点，v 向下）→ 相机系射线 `(dx, dy, -1)`，
///   `dx=(u-cx)/focal`、`dy=-(v-cy)/focal`；
/// - 深度值为**相机空间 Z**（视线方向的垂直距离，非欧氏距离）；
/// - 相机系命中点 `(dx·z, dy·z, -z)`，经 [`Self::rot_cam_to_body`] 转到机体系。
#[derive(Debug, Clone)]
pub struct DepthCamera {
    /// 像素焦距（`fx=fy`，`MuJoCo` 方形像素）。
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
    /// 最大感知距离（米），超出视为无效。
    pub max_range: f64,
    /// 像素降采样步长（每 `step` 个像素取一个，控制射线数）。
    pub pixel_step: usize,
}

impl DepthCamera {
    /// `MuJoCo` 合成场景默认标定（`scene.py`：深度相机在机体原点、`fovy=70.88°`≈
    /// D430 87°HFOV、`320×240`）。实测投影约定：
    /// - 相机看 `+x_body`（无人机前进方向）并**下倾 20°**（`scene.py` 相机
    ///   `xyaxes` 第二向量 `(0.342, 0, 0.9397)`，`sin20°=0.342`），图右 →
    ///   `-y_body`，图下 → 机体下方；
    /// - `focal = (H/2)/tan(fovy/2) = 120/tan(35.44°) ≈ 168.6`；
    /// - `rot_cam_to_body` 列 = 相机轴在机体系：`x=(0,-1,0)`、
    ///   `y=(0.342,0,0.9397)`、`z=x×y=(-0.9397,0,0.342)`。
    #[must_use]
    pub fn mujoco_default() -> Self {
        let focal = 120.0 / (70.88_f64 / 2.0).to_radians().tan();
        // 相机系 x/y/z 轴在机体系：x=(0,-1,0)、y=(0.342,0,0.9397)（下倾 20°）、
        // z=x×y=(-0.9397,0,0.342)
        let rot_cam_to_body = Matrix3::new(
            0.0, 0.3420, -0.9397, //
            -1.0, 0.0, 0.0, //
            0.0, 0.9397, 0.3420,
        );
        Self {
            focal,
            cx: 160.0,
            cy: 120.0,
            width: 320,
            height: 240,
            rot_cam_to_body,
            pos_in_body: Vector3::zeros(),
            max_range: 8.0,
            pixel_step: 3,
        }
    }
}

/// 用一帧深度图更新占据地图（感知建图，log-odds）。
///
/// 对每个有效像素（`0.05 < z ≤ max_range` 且有限）：把命中点变换到世界系，
/// 相机原点 → 命中点射线沿途体素以 `prob_miss_log` 递减、命中体素以
/// `prob_hit_log` 递增（对照官方 `raycastProcess` 658-669 行的 log-odds 更新）。
///
/// `body_pose` 为机体 → 世界变换（仿真阶段用真值，VIO 修复后换 odom）。
///
/// 注意：`Isometry * Vector3` 只旋转不平移（Vector 视为方向量），平移须用
/// `Isometry * Point3`。
pub fn update_from_depth(
    map: &mut GridMap,
    cam: &DepthCamera,
    body_pose: &Isometry3<f64>,
    depth: &[f32],
) {
    let cam_world = body_pose * Point3::from(cam.pos_in_body);
    let mut v = 0usize;
    while v < cam.height {
        let mut u = 0usize;
        while u < cam.width {
            let z = depth[v * cam.width + u];
            let z = f64::from(z);
            if z > 0.05 && z <= cam.max_range && z.is_finite() {
                let dx = (u as f64 - cam.cx) / cam.focal;
                let dy = -(v as f64 - cam.cy) / cam.focal;
                let hit_cam = Vector3::new(dx * z, dy * z, -z);
                let hit_world =
                    body_pose * Point3::from(cam.pos_in_body + cam.rot_cam_to_body * hit_cam);
                mark_ray(map, cam_world.coords, hit_world.coords);
            }
            u += cam.pixel_step;
        }
        v += cam.pixel_step;
    }
}

/// 从 `from` 到 `to` 的体素遍历（3D DDA）：沿途以 `prob_miss_log` 递减，末端以 `prob_hit_log` 递增。
///
/// 对照官方 `grid_map.cpp:577-700` `raycastProcess`：官方对同一体素在帧内做多数投票
/// （`count_hit/count_hit_and_miss` 统计后统一 `log_odds_update` 一次），firefly 逐像素 DDA 每条射线
/// 独立累积更新（`update_occupancy(idx, delta)` clamp 到 `[clamp_min_log_, clamp_max_log_]`，
/// 方向一致，差异仅在于帧内多次命中/穿过的合并时机，注释中写明对照关系，不引入帧缓冲）。
fn mark_ray(map: &mut GridMap, from: Vector3<f64>, to: Vector3<f64>) {
    let Some(mut idx) = map.index_of(from) else {
        return;
    };
    let Some(idx_to) = map.index_of(to) else {
        return;
    };
    // 取 log-odds 增量（避免在可变借用期间再借 map）
    let prob_hit = map.prob_hit_log();
    let prob_miss = map.prob_miss_log(); // 负值，对照官方 `logit(p_miss)`； miss 递减即 `+prob_miss`
    // 命中体素即起点（相机贴障碍）：直接累加命中
    if idx == idx_to {
        map.update_occupancy(idx, prob_hit);
        return;
    }

    let res = map.resolution();
    let origin = map.origin();
    // DDA（Amanatides & Woo）：沿射线逐格推进
    let dir = to - from;
    let mut step = [0i32; 3];
    let mut t_delta = [0.0f64; 3];
    let mut t_max = [0.0f64; 3];
    for i in 0..3 {
        if dir[i].abs() < 1e-12 {
            step[i] = 0;
            t_delta[i] = f64::INFINITY;
            t_max[i] = f64::INFINITY;
        } else if dir[i] > 0.0 {
            step[i] = 1;
            let bound = origin[i] + (idx[i] as f64 + 1.0) * res;
            t_delta[i] = res / dir[i].abs();
            t_max[i] = (bound - from[i]) / dir[i];
        } else {
            step[i] = -1;
            let bound = origin[i] + idx[i] as f64 * res;
            t_delta[i] = res / dir[i].abs();
            t_max[i] = (bound - from[i]) / dir[i];
        }
    }
    loop {
        let axis = (0..3)
            .min_by(|&a, &b| t_max[a].total_cmp(&t_max[b]))
            .unwrap();
        let t = t_max[axis];
        t_max[axis] += t_delta[axis];
        idx[axis] = idx[axis].wrapping_add_signed(step[axis] as isize);
        let [x, y, z] = idx;
        if x >= map.dims()[0] || y >= map.dims()[1] || z >= map.dims()[2] {
            break;
        }
        if idx == idx_to {
            map.update_occupancy(idx, prob_hit);
            break;
        }
        map.update_occupancy(idx, prob_miss);
        // 防御：射线异常长时截断（应被命中或出界提前终止）
        if t > 100.0 * dir.norm() {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{GridMapBuilder, VoxelState};
    use nalgebra::{Translation3, UnitQuaternion};

    #[test]
    fn default_camera_focal_matches_fovy() {
        let cam = DepthCamera::mujoco_default();
        assert!((cam.focal - 168.607).abs() < 1e-3);
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
        // 无旋转机体 + 采样像素 (u=159, v=54)（pixel_step=3 网格点）：
        // 下倾 20° 相机射线在深度 5m 处命中机体系 ≈ (5.37, 0.03, 0.13)，
        // 需多次观测才跨阈值
        let cam = DepthCamera::mujoco_default();
        let mut map = GridMapBuilder::new(1.0, [20, 20, 10]).build().unwrap();
        let pose = Isometry3::identity();
        let mut depth = vec![0.0f32; cam.width * cam.height];
        depth[54 * cam.width + 159] = 5.0;
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
        // 复现 demo 场景：无人机在 (1,4,1) 恒等姿态；下倾 20° 相机射线经
        // 像素 (207,69)（pixel_step=3 网格点）深度 6.471m → 命中箱 (8,2)
        // 前表面世界 (7.75,2.20,0.63)，体素 [19,17,1]，多次观测后占据
        let cam = DepthCamera::mujoco_default();
        // 地图与 demo 一致：origin (0,-5,0)、0.4m、dims [80,35,13]
        let mut map = GridMapBuilder::new(0.4, [80, 35, 13])
            .with_origin(Vector3::new(0.0, -5.0, 0.0))
            .build()
            .unwrap();
        let pose =
            Isometry3::from_parts(Translation3::new(1.0, 4.0, 1.0), UnitQuaternion::identity());
        let mut depth = vec![0.0f32; cam.width * cam.height];
        depth[69 * cam.width + 207] = 6.471;
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
        // 直接测 mark_ray：from (1,4,1) → to (6.59,2.41,0.60)，地图与 demo 一致
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
