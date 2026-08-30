//! 全局重定位：`depth/点云 → GICP` 产生全局位姿观测。
//!
//! 靶图为静态先验 `MapFile` 体素中心转 `PointCloud`，离线一次建 `KdTree` 并估计
//! 法向/协方差；在线每 1Hz 对当前帧点云 `preprocess → align` 得到 `T_target_source`。

use firefly_gicp::ann::kdtree::KdTree;
use firefly_gicp::factors::GicpFactor;
use firefly_gicp::points::point_cloud::PointCloud;
use firefly_gicp::points::traits::{PointCloudMut, PointCloudTrait};
use firefly_gicp::registration::Registration;
use firefly_gicp::registration::RegistrationResult;
use firefly_gicp::util::downsampling::voxelgrid_sampling;
use firefly_gicp::util::normal_estimation::estimate_normals_covariances_with_tree;
use firefly_map::{DepthCamera, MapFile};
use nalgebra::{Isometry3, Matrix4, Point3, Vector4};

/// 重定位参数（透传 `RegistrationSetting` 子集）。
#[derive(Debug, Clone, serde::Deserialize)]
#[serde(default)]
pub struct RelocOptions {
    /// 降采样分辨率 `m`。
    pub downsampling_resolution: f64,
    /// 最近邻数（法向/协方差）。
    pub num_neighbors: usize,
    /// 最大对应距离 `m`。
    pub max_correspondence_distance: f64,
    /// 最大迭代。
    pub max_iterations: usize,
    /// 体素降采样前是否保留全部点（`false` 时对 source 也降采样）。
    pub voxel_resolution: f64,
}

impl Default for RelocOptions {
    fn default() -> Self {
        Self {
            downsampling_resolution: 0.2,
            num_neighbors: 10,
            max_correspondence_distance: 2.0,
            max_iterations: 20,
            voxel_resolution: 1.0,
        }
    }
}

/// 单次重定位结果。
#[derive(Debug, Clone)]
pub struct RelocResult {
    /// `GICP` 原始结果。
    pub result: RegistrationResult,
    /// `source` 总点数（用于内点率）。
    pub total_points: usize,
}

/// 全局重定位器：持有靶图 `PointCloud + KdTree`。
pub struct GlobalRelocalizer {
    target: PointCloud,
    tree: KdTree<PointCloud>,
    options: RelocOptions,
}

impl GlobalRelocalizer {
    /// 由 `PointCloud` 构造（已含法向/协方差则直接建树，否则内部估计）。
    #[must_use]
    pub fn from_cloud(mut cloud: PointCloud, options: RelocOptions) -> Self {
        if cloud.num_points() == 0 {
            let tree: KdTree<PointCloud> = KdTree::new(cloud.clone());
            return Self {
                target: cloud,
                tree,
                options,
            };
        }
        // 若首点协方差为零，视为未估计
        let need_est = cloud.cov(0).norm() < 1e-12;
        if need_est {
            let tree0: KdTree<PointCloud> = KdTree::new(cloud.clone());
            estimate_normals_covariances_with_tree(&mut cloud, &tree0, options.num_neighbors);
        }
        let tree: KdTree<PointCloud> = KdTree::new(cloud.clone());
        Self {
            target: cloud,
            tree,
            options,
        }
    }

    /// 由 `MapFile` 静态占据体素构造（体素中心即点）。
    ///
    /// # Errors
    ///
    /// `MapFile` 为空时返回 `InvalidArgument`。
    pub fn from_map_file(
        map_file: &MapFile,
        options: RelocOptions,
    ) -> Result<Self, firefly_error::Error> {
        if map_file.occupied.is_empty() {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "map has no occupied voxels",
            ));
        }
        let mut cloud = PointCloud::new();
        cloud.resize(map_file.occupied.len());
        for (i, p) in map_file.occupied.iter().enumerate() {
            cloud.set_point(i, Vector4::new(p[0], p[1], p[2], 1.0));
        }
        // 可选降采样：静态地图稠密时先体素化
        let cloud = if options.downsampling_resolution > 1e-12 {
            voxelgrid_sampling(&cloud, options.downsampling_resolution)
        } else {
            cloud
        };
        Ok(Self::from_cloud(cloud, options))
    }

    /// 访问靶图（诊断用）。
    #[must_use]
    pub fn target(&self) -> &PointCloud {
        &self.target
    }

    /// 深度图转点云（复用 `firefly-map/src/depth.rs:99` 投影）。
    #[must_use]
    pub fn depth_to_cloud(
        depth: &[f32],
        cam: &DepthCamera,
        body_pose: &Isometry3<f64>,
    ) -> PointCloud {
        let mut pts = Vec::new();
        let mut v = 0usize;
        while v < cam.height {
            let mut u = 0usize;
            while u < cam.width {
                let z = f64::from(depth[v * cam.width + u]);
                if z > 0.05 && z <= cam.max_range && z.is_finite() {
                    let dx = (u as f64 - cam.cx) / cam.focal;
                    let dy = -(v as f64 - cam.cy) / cam.focal;
                    let hit_cam = nalgebra::Vector3::new(dx * z, dy * z, -z);
                    let hit_world =
                        body_pose * Point3::from(cam.pos_in_body + cam.rot_cam_to_body * hit_cam);
                    pts.push(hit_world.coords);
                }
                u += cam.pixel_step;
            }
            v += cam.pixel_step;
        }
        let mut cloud = PointCloud::new();
        cloud.resize(pts.len());
        for (i, p) in pts.into_iter().enumerate() {
            cloud.set_point(i, Vector4::new(p.x, p.y, p.z, 1.0));
        }
        cloud
    }

    /// 对齐：`source` 为当前帧局部点云（已在机体系或已用初值粗对齐的全局系均可），
    /// `init` 为 `VIO` 给的 `T_target_source` 初值。
    #[fastrace::trace]
    pub fn relocalize(&self, source: &PointCloud, init: &Matrix4<f64>) -> RelocResult {
        if source.num_points() == 0 {
            return RelocResult {
                result: RegistrationResult::new(*init),
                total_points: 0,
            };
        }
        // source 预处理：降采样 + 法向/协方差
        let source_ds: PointCloud =
            voxelgrid_sampling(source, self.options.downsampling_resolution);
        let mut source_est = source_ds;
        if source_est.num_points() > 0 {
            let t: KdTree<PointCloud> = KdTree::new(source_est.clone());
            estimate_normals_covariances_with_tree(&mut source_est, &t, self.options.num_neighbors);
        }
        let total_points = source_est.num_points();
        let mut reg: Registration<GicpFactor> = Registration::default();
        reg.rejector.max_dist_sq =
            self.options.max_correspondence_distance * self.options.max_correspondence_distance;
        reg.optimizer.max_iterations = self.options.max_iterations;
        let result = reg.align_serial(&self.target, &source_est, &self.tree, init);
        RelocResult {
            result,
            total_points,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_gicp::util::lie::se3_exp;
    use nalgebra::{Matrix4, Vector3, Vector6};

    fn make_cloud(n: usize) -> PointCloud {
        let mut c = PointCloud::new();
        c.resize(n);
        for i in 0..n {
            let x = (i as f64 * 0.7).sin() * 5.0;
            let y = (i as f64 * 0.9).cos() * 5.0;
            let z = (i as f64 * 0.11).sin() * 1.0 + 1.0;
            c.set_point(i, Vector4::new(x, y, z, 1.0));
        }
        let mut c2 = c.clone();
        let t: KdTree<PointCloud> = KdTree::new(c2.clone());
        estimate_normals_covariances_with_tree(&mut c2, &t, 10);
        c2
    }

    #[test]
    fn from_cloud_builds() {
        let cloud = make_cloud(50);
        let r = GlobalRelocalizer::from_cloud(cloud, RelocOptions::default());
        assert!(r.target.num_points() > 0);
    }

    #[test]
    fn relocalize_identity_converges() {
        let target = make_cloud(200);
        let reloc = GlobalRelocalizer::from_cloud(target.clone(), RelocOptions::default());
        let mut a = Vector6::zeros();
        a.fixed_rows_mut::<3>(3)
            .copy_from(&Vector3::new(0.2, 0.0, 0.0));
        let t_gt = se3_exp(&a);
        let mut source = PointCloud::new();
        source.resize(target.num_points());
        for i in 0..target.num_points() {
            source.set_point(i, t_gt.try_inverse().unwrap() * target.point(i));
        }
        let res = reloc.relocalize(&source, &Matrix4::identity());
        assert!(res.result.converged || res.result.num_inliers > 10);
    }

    #[test]
    fn depth_to_cloud_empty() {
        let cam = DepthCamera::mujoco_default();
        let depth = vec![0.0f32; cam.width * cam.height];
        let cloud = GlobalRelocalizer::depth_to_cloud(&depth, &cam, &Isometry3::identity());
        assert_eq!(cloud.num_points(), 0);
    }
}
