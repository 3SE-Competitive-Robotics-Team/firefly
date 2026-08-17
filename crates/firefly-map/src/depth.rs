//! 深度相机 → 占据栅格（感知建图）。
//!
//! 把一帧深度图投影成世界系点云并做光线标记：相机原点 → 命中点射线沿途
//! 体素标 [`VoxelState::Free`]，命中体素标 [`VoxelState::Occupied`]。
//!
//! 相机模型为硬编码合成标定（`MuJoCo` 场景，`packages/firefly-mujoco/src/
//! firefly_mujoco/scene.py`），投影约定经实测验证（见 `DepthCamera::mujoco_default`
//! 文档）。

use nalgebra::{Isometry3, Matrix3, Point3, Vector3};

use crate::grid::{GridMap, VoxelState};

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
    /// `MuJoCo` 合成场景默认标定（`scene.py`：深度相机在机体原点、`fovy=60°`、
    /// `320×240`）。实测投影约定：
    /// - 相机看 `+x_body`（无人机前进方向），图右 → `-y_body`，图下 → `-z_body`；
    /// - `focal = (H/2)/tan(fovy/2) = 120/tan(30°) ≈ 207.85`。
    #[must_use]
    pub fn mujoco_default() -> Self {
        let focal = 120.0 / (60.0_f64 / 2.0).to_radians().tan();
        // 相机系 x/y/z 轴在机体系：x=(0,-1,0)、y=(0,0,1)、z=(-1,0,0)
        let rot_cam_to_body = Matrix3::new(
            0.0, 0.0, -1.0, //
            -1.0, 0.0, 0.0, //
            0.0, 1.0, 0.0,
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

/// 用一帧深度图更新占据地图（感知建图）。
///
/// 对每个有效像素（`0.05 < z ≤ max_range` 且有限）：把命中点变换到世界系，
/// 相机原点 → 命中点射线沿途体素标 [`VoxelState::Free`]，命中体素标
/// [`VoxelState::Occupied`]（命中点在图外则只标到图边界，无命中体素）。
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
                let hit_world = body_pose
                    * Point3::from(cam.pos_in_body + cam.rot_cam_to_body * hit_cam);
                mark_ray(map, cam_world.coords, hit_world.coords);
            }
            u += cam.pixel_step;
        }
        v += cam.pixel_step;
    }
}

/// 从 `from` 到 `to` 的体素遍历（3D DDA）：沿途标 Free，末端标 Occupied。
fn mark_ray(map: &mut GridMap, from: Vector3<f64>, to: Vector3<f64>) {
    let Some(mut idx) = map.index_of(from) else {
        return;
    };
    let Some(idx_to) = map.index_of(to) else {
        return;
    };
    // 命中体素即起点（相机贴障碍）：直接标 Occupied
    if idx == idx_to {
        map.set_state(idx, VoxelState::Occupied);
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
        let axis = (0..3).min_by(|&a, &b| t_max[a].total_cmp(&t_max[b])).unwrap();
        let t = t_max[axis];
        t_max[axis] += t_delta[axis];
        idx[axis] = idx[axis].wrapping_add_signed(step[axis] as isize);
        let [x, y, z] = idx;
        if x >= map.dims()[0] || y >= map.dims()[1] || z >= map.dims()[2] {
            break;
        }
        if idx == idx_to {
            map.set_state(idx, VoxelState::Occupied);
            break;
        }
        map.set_state(idx, VoxelState::Free);
        // 防御：射线异常长时截断（应被命中或出界提前终止）
        if t > 100.0 * dir.norm() {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::GridMapBuilder;
    use nalgebra::{Translation3, UnitQuaternion};

    #[test]
    fn default_camera_focal_matches_fovy() {
        let cam = DepthCamera::mujoco_default();
        assert!((cam.focal - 207.846).abs() < 1e-3);
    }

    #[test]
    fn ray_marks_free_then_occupied() {
        let mut map = GridMapBuilder::new(1.0, [10, 10, 10]).build().unwrap();
        let from = Vector3::new(0.5, 0.5, 0.5);
        let to = Vector3::new(4.5, 0.5, 0.5);
        mark_ray(&mut map, from, to);
        // 沿途 Free，命中 Occupied
        assert_eq!(map.state([1, 0, 0]), VoxelState::Free);
        assert_eq!(map.state([2, 0, 0]), VoxelState::Free);
        assert_eq!(map.state([4, 0, 0]), VoxelState::Occupied);
        // 起点体素不改（含相机）
        assert_eq!(map.state([0, 0, 0]), VoxelState::Unknown);
    }

    #[test]
    fn ray_into_map_bounds_only_marks_inside() {
        let mut map = GridMapBuilder::new(1.0, [10, 10, 10]).build().unwrap();
        let from = Vector3::new(0.5, 0.5, 0.5);
        let to = Vector3::new(9.5, 5.5, 0.5); // 终点在图内，中间越界路径被跳过
        mark_ray(&mut map, from, to);
        assert_eq!(map.state([1, 0, 0]), VoxelState::Free);
        // 不会 panic，出界即停
    }

    #[test]
    fn update_from_depth_projects_to_world() {
        // 无旋转机体 + 中心像素：命中点应在 +x_body
        let cam = DepthCamera::mujoco_default();
        let mut map = GridMapBuilder::new(1.0, [20, 20, 10]).build().unwrap();
        let pose = Isometry3::identity();
        let mut depth = vec![0.0f32; cam.width * cam.height];
        // 采样像素（u=159 是 pixel_step=3 的采样点）深度 5m → 命中 ≈ (5,0,0)
        depth[120 * cam.width + 159] = 5.0;
        update_from_depth(&mut map, &cam, &pose, &depth);
        // 命中点 ≈ (5, 0, 0)，体素 [5,0,0] Occupied
        assert_eq!(map.state([5, 0, 0]), VoxelState::Occupied);
        // 沿途 Free
        assert_eq!(map.state([2, 0, 0]), VoxelState::Free);
    }

    #[test]
    fn update_from_depth_maps_mujoco_box() {
        // 复现 demo 场景：无人机在 (1,4,1) 恒等姿态，盒子 (8,2,0.5) 前表面
        // 在像素 (219,135) 深度 5.59m（实测值）→ 命中世界 (6.59,2.41,0.60)
        let cam = DepthCamera::mujoco_default();
        // 地图与 demo 一致：origin (0,-5,0)、0.4m、dims [80,35,13]
        let mut map = GridMapBuilder::new(0.4, [80, 35, 13])
            .with_origin(Vector3::new(0.0, -5.0, 0.0))
            .build()
            .unwrap();
        let pose = Isometry3::from_parts(
            Translation3::new(1.0, 4.0, 1.0),
            UnitQuaternion::identity(),
        );
        let mut depth = vec![0.0f32; cam.width * cam.height];
        depth[135 * cam.width + 219] = 5.59;
        update_from_depth(&mut map, &cam, &pose, &depth);
        // 命中点 (6.59,2.41,0.60) → 体素 [16,18,1]
        assert_eq!(map.state([16, 18, 1]), VoxelState::Occupied);
        // 沿途（含相机所在体素后）Free
        assert_eq!(map.state([4, 22, 2]), VoxelState::Free);
    }

    #[test]
    fn mark_ray_diagonal_box_ray() {
        // 直接测 mark_ray：from (1,4,1) → to (6.59,2.41,0.60)，地图与 demo 一致
        let mut map = GridMapBuilder::new(0.4, [80, 35, 13])
            .with_origin(Vector3::new(0.0, -5.0, 0.0))
            .build()
            .unwrap();
        mark_ray(
            &mut map,
            Vector3::new(1.0, 4.0, 1.0),
            Vector3::new(6.59, 2.412, 0.596),
        );
        assert_eq!(map.state([16, 18, 1]), VoxelState::Occupied);
        assert_eq!(map.state([2, 22, 2]), VoxelState::Unknown); // 起点体素不改
        // DDA 路径中途某体素应 Free（手工跟踪：5,21,2 → 7,20,2 → 12,19,1 → 16,18,1）
        assert_eq!(map.state([5, 21, 2]), VoxelState::Free);
        assert_eq!(map.state([7, 20, 2]), VoxelState::Free);
        assert_eq!(map.state([12, 19, 1]), VoxelState::Free);
    }
}
