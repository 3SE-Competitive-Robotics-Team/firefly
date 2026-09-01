//! 可见体素查询与按需光线投射（论文 VII-A，对照 Fig.7）。
//!
//! - 可见体素查询：相机视锥（FoV 球）内根体素哈希查询，收集其中视觉点；
//! - 光线投射：图像 30×30 网格中未被视觉点占据的格子，沿中心像素反向
//!   投射，采样点从 `d_min` 到 `d_max` 均匀分布，命中含视觉点的体素即止。
//!
//! `FoV` 球心取相机世界位置（`cam_pose` 为世界→相机约定时
//! `cam_pos = −Rᵀ·t`，对照 `vio.cpp:1542-1543` 的
//! `Rcw = Rci·Rwiᵀ`、`Pcw = −Rci·Rwiᵀ·Pwi`）；最终可见性判定与官方一致
//! 按投影入图像（`vio.cpp:462` `dir[2] < 0` + `isInFrame`）。

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
        // 相机世界位置：cam_pose 为世界→相机变换 `(Rcw, t_cw)`，逆变换的
        // 平移 `p_cw = −Rcwᵀ·t_cw`（对照 `vio.cpp:1694-1696` `updateFrameState`）。
        let r_cw = cam_pose.rotation.to_rotation_matrix().into_inner();
        let cam_pos = -r_cw.transpose() * cam_pose.translation.vector;
        // 相机光轴（世界系）= 相机系 +z 经 Rcwᵀ 转出（对照 `vio.cpp:462`
        // `dir[2] < 0` 的前向判据：点在相机系 +z 半球内）。
        let cam_axis = r_cw.transpose() * Vector3::z_axis().into_inner();

        let r_span = (query_radius / r0).ceil() as i64;
        for dz in -r_span..=r_span {
            for dy in -r_span..=r_span {
                for dx in -r_span..=r_span {
                    let center =
                        cam_pos + Vector3::new(dx as f64 * r0, dy as f64 * r0, dz as f64 * r0);
                    if (center - cam_pos).norm() > query_radius + r0 {
                        continue;
                    }
                    // FoV 余弦粗剔除（论文 VII-A：视锥内根体素）。
                    let dir_to_voxel = center - cam_pos;
                    let dir_norm = dir_to_voxel.norm();
                    if dir_norm < 1e-12 {
                        continue;
                    }
                    let dir_unit = dir_to_voxel / dir_norm;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image_patch::PatchPyramid;
    use crate::options::VoxelMapOptions;
    use crate::visual_point::VisualPoint;
    use nalgebra::{Matrix3, Translation3, UnitQuaternion, Vector2};

    fn intrinsics() -> firefly_void_types::visual::Intrinsics {
        firefly_void_types::visual::Intrinsics::new(300.0, 300.0, 160.0, 120.0)
    }

    /// 向地图插入一个带参考补丁的视觉点（世界系）。
    ///
    /// 走 [`VoxelMap::register_points`] 建根体素后经测试辅助方法挂视觉点。
    fn insert_visual_point(map: &mut VoxelMap, pos: Vector3<f64>) {
        // 先注册一个几何点以创建根体素
        map.register_points(&[pos], &[Matrix3::identity() * 1e-8], &Vector3::zeros());
        let mut vp = VisualPoint::new(
            pos,
            Matrix3::identity() * 1e-4,
            Vector3::z_axis().into_inner(),
        );
        // 空补丁金字塔占位（ref_patch 必须 Some 才会被收集）
        vp.ref_patch = Some(0);
        vp.obs.push(crate::visual_point::PatchObservation {
            frame_id: 0,
            pose: Isometry3::identity(),
            inv_expo_time: 1.0,
            px: Vector2::new(160.0, 120.0),
            patch: PatchPyramid {
                levels: vec![vec![0.0; 121]],
                scale: vec![1],
                patch_size: 11,
            },
            score: 0.0,
            mean: 0.0,
        });
        map.push_visual_point_for_test(vp);
    }

    #[test]
    fn fov_center_uses_camera_world_position() {
        // 相机在 (0, 0, 1) 朝 −z 看（cam_pose 世界→相机：R=I、t=(0,0,1)）。
        // 旧实现把 t 当相机位置 → 球心在 (0,0,1) 而非原点，FoV 球错位。
        let mut map = VoxelMap::new(VoxelMapOptions::default());
        // 点放在相机正前方（世界系 z=1 平面上、相机前 0.5m）
        insert_visual_point(&mut map, Vector3::new(0.0, 0.0, 0.5));
        // 相机在原点朝 +z 看（R=I、t=0），点 z=0.5 在正前方
        let cam_pose = Isometry3::identity();
        let vis = map.visible_map_points(&cam_pose, &intrinsics(), &[]);
        assert_eq!(vis.len(), 1, "正前方点应可见");
    }

    #[test]
    fn fov_center_with_nonidentity_rotation() {
        // 相机位姿带旋转：世界→相机 R = Ry(π)（相机朝 −z），t 非零。
        // 相机世界位置 = −Rᵀ·t 必须正确，否则 FoV 球心错位把点全剔。
        let rot_cw = nalgebra::Rotation3::from_axis_angle(&Vector3::y_axis(), std::f64::consts::PI);
        // 相机世界位置 (0,0,1) → Rcw = Ry(π)，t_cw = −Rcw·(0,0,1) = (0,0,1)
        let cam_pose = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 1.0),
            UnitQuaternion::from_rotation_matrix(&rot_cw),
        );
        let mut map = VoxelMap::new(VoxelMapOptions::default());
        // 相机在 (0,0,1) 朝 −z，点在其正前方 (0,0,0.4)
        insert_visual_point(&mut map, Vector3::new(0.0, 0.0, 0.4));
        let vis = map.visible_map_points(&cam_pose, &intrinsics(), &[]);
        assert_eq!(
            vis.len(),
            1,
            "旋转位姿下正前方点应可见（球心=相机世界位置）"
        );
    }
}
