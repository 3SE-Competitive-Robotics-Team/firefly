//! 可见体素查询与按需光线投射（论文 VII-A，对照 Fig.7）。
//!
//! - 可见体素查询：相机视锥（FoV 球）内根体素哈希查询，收集其中视觉点；
//! - 光线投射：图像 30×30 网格中未被视觉点占据的格子，沿中心像素反向
//!   投射，采样点从 `d_min` 到 `d_max` 均匀分布，命中含视觉点的体素即止。

use nalgebra::{Isometry3, Vector3};

use crate::options::VoxelMapOptions;
use crate::visual_point::VisualPointView;
use crate::voxel::{VoxelKey, VoxelMap};

/// 光线采样点（相机系，论文 VII-A 预计算）。
#[derive(Debug, Clone)]
pub struct RaySample {
    /// 相机系下的单位方向。
    pub dir_cam: Vector3<f64>,
    /// 深度采样点（相机系，`d_min..d_max` 均匀）。
    pub samples: Vec<Vector3<f64>>,
}

/// 为图像网格中心像素生成光线采样点（相机系）。
///
/// 网格 `(col, row)` 中心像素 → 归一化方向 → `d_min..d_max` 均匀 `n_samples` 点。
#[must_use]
pub fn build_ray_samples(
    col: usize,
    row: usize,
    intrinsics: &firefly_void_types::visual::Intrinsics,
    opts: &VoxelMapOptions,
) -> RaySample {
    let cell = opts.grid_size as f64;
    let px = Vector3::new(
        (col as f64 + 0.5) * cell - intrinsics.cx,
        (row as f64 + 0.5) * cell - intrinsics.cy,
        1.0,
    );
    let dir = px.normalize();
    let n = 32;
    let mut samples = Vec::with_capacity(n);
    for i in 0..n {
        let t = opts.ray_depth_min
            + (opts.ray_depth_max - opts.ray_depth_min) * i as f64 / (n as f64 - 1.0);
        samples.push(dir * t);
    }
    RaySample {
        dir_cam: dir,
        samples,
    }
}

impl VoxelMap {
    /// 查询相机视锥内的可见视觉地图点（论文 VII-A）。
    ///
    /// 遍历以相机为中心的 `ray_depth_max` 球体内根体素（FoV 余弦粗剔除），
    /// 收集其中视觉点。`prev_visible` 为上一帧可见点（FoV 重叠假设），
    /// 并入返回集。
    #[fastrace::trace]
    #[must_use]
    pub fn visible_map_points(
        &self,
        cam_pose: &Isometry3<f64>,
        intrinsics: &firefly_void_types::visual::Intrinsics,
        prev_visible: &[VisualPointView],
    ) -> Vec<VisualPointView> {
        let opts = self.options();
        let mut out = Vec::new();
        let fov_cos = opts.fov.cos();
        let query_radius = opts.ray_depth_max;
        let r0 = opts.root_size;
        let cam_pos = cam_pose.translation.vector;

        let r_span = (query_radius / r0).ceil() as i64;
        for dz in -r_span..=r_span {
            for dy in -r_span..=r_span {
                for dx in -r_span..=r_span {
                    let center =
                        cam_pos + Vector3::new(dx as f64 * r0, dy as f64 * r0, dz as f64 * r0);
                    if (center - cam_pos).norm() > query_radius + r0 {
                        continue;
                    }
                    // FoV 余弦检查（体素中心方向 vs 相机光轴）。
                    // 光轴为世界系下相机看向的方向 = Rᵀ·z_cam（相机系 +z 前向）。
                    let dir_to_voxel = center - cam_pos;
                    let dir_norm = dir_to_voxel.norm();
                    if dir_norm < 1e-12 {
                        continue;
                    }
                    let dir_unit = dir_to_voxel / dir_norm;
                    let cam_axis = cam_pose.rotation.inverse() * Vector3::z_axis().into_inner();
                    if dir_unit.dot(&cam_axis) < fov_cos {
                        continue;
                    }
                    let key = VoxelKey::from_point(&center, r0);
                    self.collect_visual_points(&key, cam_pose, intrinsics, &mut out);
                }
            }
        }
        out.extend(prev_visible.iter().cloned());
        out
    }

    /// 光线投射补漏（论文 VII-A 第 2 小节）。
    ///
    /// `occupied_grid` 为已投影像素点占据的网格掩码（`cols×rows`，行主序）；
    /// 未占据网格沿中心像素投射，命中含视觉点的体素即止。
    #[fastrace::trace]
    pub fn raycast_visual_points(
        &self,
        cam_pose: &Isometry3<f64>,
        intrinsics: &firefly_void_types::visual::Intrinsics,
        occupied_grid: &[bool],
        out: &mut Vec<VisualPointView>,
    ) {
        let opts = self.options();
        let n = occupied_grid.len();
        if n == 0 {
            return;
        }
        // 网格宽高由调用方经 grid_dims 填掩码；此处按 4:3 恢复（宽 ≥ 高）
        let cols_est = (((n as f64) * 4.0 / 3.0).sqrt().round() as usize).max(1);
        let rows_est = n.div_ceil(cols_est);
        for (idx, occupied) in occupied_grid.iter().enumerate() {
            if *occupied {
                continue;
            }
            let col = idx % cols_est;
            let row = idx / cols_est;
            if row >= rows_est {
                continue;
            }
            let ray = build_ray_samples(col, row, intrinsics, opts);
            for sample in &ray.samples {
                let p_world = crate::voxel::transform_point(cam_pose, sample);
                let key = VoxelKey::from_point(&p_world, opts.root_size);
                let before = out.len();
                self.collect_visual_points(&key, cam_pose, intrinsics, out);
                if out.len() > before {
                    break; // 命中即止（论文 VII-A：cease for this ray）
                }
            }
        }
    }
}
