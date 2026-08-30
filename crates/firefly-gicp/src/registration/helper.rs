//! 配准辅助（对照 `registration/registration_helper.hpp`）。

use nalgebra::Matrix4;

use crate::ann::incremental_voxelmap::GaussianVoxelMap;
use crate::ann::kdtree::KdTree;
use crate::factors::{GicpFactor, IcpFactor, PlaneIcpFactor};
use crate::points::point_cloud::PointCloud;
use crate::points::traits::PointCloudTrait;
use crate::registration::registration::Registration;
use crate::registration::registration_result::RegistrationResult;
use crate::registration::termination_criteria::TerminationCriteria;
use crate::util::downsampling::voxelgrid_sampling;
use crate::util::normal_estimation::{
    estimate_covariances_with_tree, estimate_normals_covariances_with_tree,
    estimate_normals_with_tree,
};

/// 预处理点云：降采样 + 建树 + 法向/协方差估计（对照 `preprocess_points`）。
#[fastrace::trace]
pub fn preprocess_points<P>(
    points: &P,
    downsampling_resolution: f64,
    num_neighbors: usize,
) -> (PointCloud, KdTree<PointCloud>)
where
    P: PointCloudTrait + Sync,
{
    let downsampled: PointCloud = voxelgrid_sampling(points, downsampling_resolution);
    let kdtree: KdTree<PointCloud> = KdTree::new(downsampled.clone());
    let mut downsampled_mut = downsampled;
    estimate_normals_covariances_with_tree(&mut downsampled_mut, &kdtree, num_neighbors);
    let kdtree: KdTree<PointCloud> = KdTree::new(downsampled_mut.clone());
    (downsampled_mut, kdtree)
}

/// 创建高斯体素图（对照 `create_gaussian_voxelmap`）。
pub fn create_gaussian_voxelmap(points: &PointCloud, voxel_resolution: f64) -> GaussianVoxelMap {
    let mut map = GaussianVoxelMap::new(voxel_resolution);
    map.insert_identity(points);
    map
}

/// 配准类型（对照 `RegistrationSetting::RegistrationType`）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegistrationType {
    /// ICP.
    Icp,
    /// Plane ICP.
    PlaneIcp,
    /// GICP.
    Gicp,
    /// VGICP.
    Vgicp,
}

/// 配准设定（对照 `RegistrationSetting`）。
#[derive(Clone, Copy, Debug)]
pub struct RegistrationSetting {
    /// 配准类型。
    pub registration_type: RegistrationType,
    /// VGICP 体素分辨率。
    pub voxel_resolution: f64,
    /// 降采样分辨率（仅 `align` 的 Eigen 输入路径使用）。
    pub downsampling_resolution: f64,
    /// 最大对应距离。
    pub max_correspondence_distance: f64,
    /// 旋转容差 [rad]。
    pub rotation_eps: f64,
    /// 平移容差。
    pub translation_eps: f64,
    /// 最大迭代。
    pub max_iterations: usize,
    /// 打印调试。
    pub verbose: bool,
}

impl Default for RegistrationSetting {
    fn default() -> Self {
        Self {
            registration_type: RegistrationType::Gicp,
            voxel_resolution: 1.0,
            downsampling_resolution: 0.25,
            max_correspondence_distance: 1.0,
            rotation_eps: 0.1 * std::f64::consts::PI / 180.0,
            translation_eps: 1e-3,
            max_iterations: 20,
            verbose: false,
        }
    }
}

/// 对齐预处理点云（KdTree 路径，对照 `align(const PointCloud&, const PointCloud&, const KdTree&, ...)`）。
pub fn align(
    target: &PointCloud,
    source: &PointCloud,
    target_tree: &KdTree<PointCloud>,
    init_t: &Matrix4<f64>,
    setting: &RegistrationSetting,
) -> RegistrationResult {
    let criteria = TerminationCriteria {
        rotation_eps: setting.rotation_eps,
        translation_eps: setting.translation_eps,
    };

    match setting.registration_type {
        RegistrationType::Icp => {
            let mut reg: Registration<IcpFactor> = Registration::default();
            reg.criteria = criteria;
            reg.rejector.max_dist_sq =
                setting.max_correspondence_distance * setting.max_correspondence_distance;
            reg.optimizer.max_iterations = setting.max_iterations;
            reg.optimizer.verbose = setting.verbose;
            reg.align_serial(target, source, target_tree, init_t)
        }
        RegistrationType::PlaneIcp => {
            let mut reg: Registration<PlaneIcpFactor> = Registration::default();
            reg.criteria = criteria;
            reg.rejector.max_dist_sq =
                setting.max_correspondence_distance * setting.max_correspondence_distance;
            reg.optimizer.max_iterations = setting.max_iterations;
            reg.optimizer.verbose = setting.verbose;
            reg.align_serial(target, source, target_tree, init_t)
        }
        RegistrationType::Gicp => {
            let mut reg: Registration<GicpFactor> = Registration::default();
            reg.criteria = criteria;
            reg.rejector.max_dist_sq =
                setting.max_correspondence_distance * setting.max_correspondence_distance;
            reg.optimizer.max_iterations = setting.max_iterations;
            reg.optimizer.verbose = setting.verbose;
            reg.align_serial(target, source, target_tree, init_t)
        }
        RegistrationType::Vgicp => {
            eprintln!("warning: use align_vgicp for VGICP with GaussianVoxelMap");
            let mut reg: Registration<GicpFactor> = Registration::default();
            reg.criteria = criteria;
            reg.rejector.max_dist_sq =
                setting.max_correspondence_distance * setting.max_correspondence_distance;
            reg.optimizer.max_iterations = setting.max_iterations;
            reg.optimizer.verbose = setting.verbose;
            reg.align_serial(target, source, target_tree, init_t)
        }
    }
}

/// VGICP 对齐（对照 `align(const GaussianVoxelMap&, const PointCloud&, ...)`）。
pub fn align_vgicp(
    target: &GaussianVoxelMap,
    source: &PointCloud,
    init_t: &Matrix4<f64>,
    setting: &RegistrationSetting,
) -> RegistrationResult {
    let criteria = TerminationCriteria {
        rotation_eps: setting.rotation_eps,
        translation_eps: setting.translation_eps,
    };
    let mut r: Registration<GicpFactor> = Registration::default();
    r.criteria = criteria;
    r.rejector.max_dist_sq =
        setting.max_correspondence_distance * setting.max_correspondence_distance;
    r.optimizer.max_iterations = setting.max_iterations;
    r.optimizer.verbose = setting.verbose;
    // VGICP 的 target_tree 即 target 自身（voxelmap 同时是点云与搜索结构）
    r.align_serial(target, source, target, init_t)
}

/// 便捷：由未预处理点云直接对齐（会内部预处理）。
pub fn align_from_points<Pt, Ps>(
    target_points: &Pt,
    source_points: &Ps,
    init_t: &Matrix4<f64>,
    setting: &RegistrationSetting,
) -> RegistrationResult
where
    Pt: PointCloudTrait + Sync,
    Ps: PointCloudTrait + Sync,
{
    let (target, target_tree) =
        preprocess_points(target_points, setting.downsampling_resolution, 10);
    let (source, _) = preprocess_points(source_points, setting.downsampling_resolution, 10);

    if setting.registration_type == RegistrationType::Vgicp {
        let voxelmap = create_gaussian_voxelmap(&target, setting.voxel_resolution);
        return align_vgicp(&voxelmap, &source, init_t, setting);
    }
    align(&target, &source, &target_tree, init_t, setting)
}

// 防止未使用警告
#[allow(dead_code)]
fn _use_estimators() {
    let _ = estimate_normals_with_tree::<PointCloud, KdTree<PointCloud>>;
    let _ = estimate_covariances_with_tree::<PointCloud, KdTree<PointCloud>>;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ann::kdtree::KdTree;
    use crate::factors::GicpFactor;
    use crate::points::point_cloud::PointCloud;
    use crate::points::traits::PointCloudMut;
    use crate::registration::{ParallelReduction, SerialReduction};
    use crate::util::lie::se3_exp;
    use nalgebra::{Matrix4, Vector3, Vector4, Vector6};

    fn make_grid_cloud() -> PointCloud {
        // 5×5×5 =125 点，间距 1.0，带随机抖动避免退化
        let mut pts = Vec::new();
        for x in 0..5 {
            for y in 0..5 {
                for z in 0..5 {
                    let jitter = |v: f64| v + 0.05 * ((x * 7 + y * 11 + z * 13) as f64).sin();
                    pts.push(Vector3::new(
                        jitter(x as f64 * 0.8),
                        jitter(y as f64 * 0.8),
                        jitter(z as f64 * 0.8),
                    ));
                }
            }
        }
        let mut cloud = PointCloud::from_points3(&pts);
        // 预估计法向/协方差（供 GICP/Plane 使用）
        let tree: KdTree<PointCloud> = KdTree::new(cloud.clone());
        estimate_normals_covariances_with_tree(&mut cloud, &tree, 10);
        cloud
    }

    fn transform_cloud(cloud: &PointCloud, t: &Matrix4<f64>) -> PointCloud {
        let mut out = PointCloud::new();
        out.resize(cloud.num_points());
        for i in 0..cloud.num_points() {
            let pt = t * cloud.point(i);
            out.set_point(i, pt);
            // 法向旋转（不含平移）
            let n = cloud.normal(i);
            let r = t.fixed_view::<3, 3>(0, 0).into_owned();
            let mut nr = Vector4::zeros();
            nr.fixed_rows_mut::<3>(0)
                .copy_from(&(r * n.fixed_rows::<3>(0).into_owned()));
            out.set_normal(i, nr);
            // 协方差旋转
            let cov = cloud.cov(i);
            let cov3 = cov.fixed_view::<3, 3>(0, 0).into_owned();
            let cov_rot = r * cov3 * r.transpose();
            let mut cov4 = Matrix4::zeros();
            cov4.fixed_view_mut::<3, 3>(0, 0).copy_from(&cov_rot);
            out.set_cov(i, cov4);
        }
        out
    }

    fn random_transform(trans: f64, rot_deg: f64) -> Matrix4<f64> {
        let rot = Vector3::new(
            rot_deg.to_radians() * 0.3,
            rot_deg.to_radians() * 0.5,
            rot_deg.to_radians() * 0.7,
        );
        let mut a = Vector6::zeros();
        a.fixed_rows_mut::<3>(0).copy_from(&rot);
        a.fixed_rows_mut::<3>(3)
            .copy_from(&Vector3::new(trans * 0.5, trans * 0.3, trans * 0.2));
        se3_exp(&a)
    }

    fn error_between(a: &Matrix4<f64>, b: &Matrix4<f64>) -> (f64, f64) {
        let diff = b.try_inverse().unwrap() * a;
        let r = diff.fixed_view::<3, 3>(0, 0).into_owned();
        let t = diff.fixed_view::<3, 1>(0, 3).into_owned();
        let angle = nalgebra::Rotation3::from_matrix(&r).angle();
        (angle, t.norm())
    }

    #[test]
    fn icp_converges_small_transform_serial() {
        let target = make_grid_cloud();
        let t_gt = random_transform(0.3, 5.0);
        let source = transform_cloud(&target, &t_gt.try_inverse().unwrap());
        let tree: KdTree<PointCloud> = KdTree::new(target.clone());

        let init = Matrix4::identity();
        let setting = RegistrationSetting {
            registration_type: RegistrationType::Icp,
            max_correspondence_distance: 2.0,
            max_iterations: 20,
            ..Default::default()
        };
        let result = align(&target, &source, &tree, &init, &setting);
        let (rot_err, trans_err) = error_between(&result.t_target_source, &t_gt);
        assert!(rot_err < 2.5_f64.to_radians(), "rot {rot_err}");
        assert!(trans_err < 0.2, "trans {trans_err}");
    }

    #[test]
    fn gicp_converges_serial_and_parallel_consistent() {
        let target = make_grid_cloud();
        let t_gt = random_transform(0.4, 8.0);
        let source = transform_cloud(&target, &t_gt.try_inverse().unwrap());
        let tree: KdTree<PointCloud> = KdTree::new(target.clone());

        let setting = RegistrationSetting {
            registration_type: RegistrationType::Gicp,
            max_correspondence_distance: 2.0,
            max_iterations: 20,
            ..Default::default()
        };
        let res_serial = align(&target, &source, &tree, &Matrix4::identity(), &setting);

        // 并行路径
        let reg_par: crate::registration::Registration<GicpFactor, ParallelReduction> =
            crate::registration::Registration {
                criteria: crate::registration::TerminationCriteria {
                    rotation_eps: setting.rotation_eps,
                    translation_eps: setting.translation_eps,
                },
                rejector: crate::registration::DistanceRejector { max_dist_sq: 4.0 },
                point_factor_setting: GicpFactor::default(),
                general_factor: crate::factors::NullFactor,
                reduction: ParallelReduction { num_threads: 0 },
                optimizer: crate::registration::LevenbergMarquardtOptimizer {
                    max_iterations: 20,
                    ..Default::default()
                },
                _phantom: std::marker::PhantomData,
            };
        let res_par = reg_par.align_parallel(&target, &source, &tree, &Matrix4::identity());

        let (rot_err, trans_err) =
            error_between(&res_serial.t_target_source, &res_par.t_target_source);
        assert!(
            rot_err < 0.5_f64.to_radians(),
            "serial vs parallel rot diff {rot_err}"
        );
        assert!(
            trans_err < 0.05,
            "serial vs parallel trans diff {trans_err}"
        );
        let (rot_err, trans_err) = error_between(&res_par.t_target_source, &t_gt);
        assert!(rot_err < 2.5_f64.to_radians());
        assert!(trans_err < 0.2);
    }

    #[test]
    fn plane_icp_converges() {
        let target = make_grid_cloud();
        let t_gt = random_transform(0.2, 4.0);
        let source = transform_cloud(&target, &t_gt.try_inverse().unwrap());
        let tree: KdTree<PointCloud> = KdTree::new(target.clone());
        let setting = RegistrationSetting {
            registration_type: RegistrationType::PlaneIcp,
            max_correspondence_distance: 2.0,
            ..Default::default()
        };
        let result = align(&target, &source, &tree, &Matrix4::identity(), &setting);
        let (rot_err, trans_err) = error_between(&result.t_target_source, &t_gt);
        assert!(rot_err < 2.5_f64.to_radians());
        assert!(trans_err < 0.3);
    }

    #[test]
    fn vgicp_converges() {
        let target_raw = make_grid_cloud();
        let voxelmap = create_gaussian_voxelmap(&target_raw, 1.0);
        // 确保体素内协方差已归一化
        assert!(!voxelmap.flat_voxels.is_empty());
        let t_gt = random_transform(0.3, 5.0);
        let source = transform_cloud(&target_raw, &t_gt.try_inverse().unwrap());
        // 重新估计 source 协方差
        let mut source_est = source.clone();
        let tree_s: KdTree<PointCloud> = KdTree::new(source_est.clone());
        estimate_normals_covariances_with_tree(&mut source_est, &tree_s, 10);

        let setting = RegistrationSetting {
            registration_type: RegistrationType::Vgicp,
            voxel_resolution: 1.0,
            max_correspondence_distance: 2.0,
            ..Default::default()
        };
        let result = align_vgicp(&voxelmap, &source_est, &Matrix4::identity(), &setting);
        let (rot_err, trans_err) = error_between(&result.t_target_source, &t_gt);
        assert!(rot_err < 3.0_f64.to_radians(), "vgicp rot {rot_err}");
        assert!(trans_err < 0.3, "vgicp trans {trans_err}");
    }

    #[test]
    fn robust_gicp_converges() {
        use crate::factors::{Huber, HuberSetting, RobustFactor};
        let target = make_grid_cloud();
        let t_gt = random_transform(0.3, 6.0);
        let mut source = transform_cloud(&target, &t_gt.try_inverse().unwrap());
        // 注入离群点
        source.set_point(0, Vector4::new(100.0, 100.0, 100.0, 1.0));
        let tree: KdTree<PointCloud> = KdTree::new(target.clone());

        let reg: crate::registration::Registration<RobustFactor<Huber, GicpFactor>> =
            crate::registration::Registration {
                criteria: crate::registration::TerminationCriteria::default(),
                rejector: crate::registration::DistanceRejector { max_dist_sq: 4.0 },
                point_factor_setting: RobustFactor::new(
                    Huber::new(HuberSetting { c: 1.0 }),
                    GicpFactor::default(),
                ),
                general_factor: crate::factors::NullFactor,
                reduction: SerialReduction,
                optimizer: crate::registration::LevenbergMarquardtOptimizer::default(),
                _phantom: std::marker::PhantomData,
            };
        let result = reg.align_serial(&target, &source, &tree, &Matrix4::identity());
        let (rot_err, trans_err) = error_between(&result.t_target_source, &t_gt);
        assert!(rot_err < 2.5_f64.to_radians());
        assert!(trans_err < 0.25);
    }

    #[test]
    fn official_data_gicp_accuracy() {
        use crate::util::ply::{load_small_gicp_ply, load_transform_txt};
        use std::path::PathBuf;
        let target_path = PathBuf::from(env!("HOME")).join("Projects/small-gicp/data/target.ply");
        let source_path = PathBuf::from(env!("HOME")).join("Projects/small-gicp/data/source.ply");
        let t_path =
            PathBuf::from(env!("HOME")).join("Projects/small-gicp/data/T_target_source.txt");
        if !target_path.exists() || !source_path.exists() || !t_path.exists() {
            eprintln!("official data not found, skip");
            return;
        }
        let target_raw = load_small_gicp_ply(&target_path).expect("load target");
        let source_raw = load_small_gicp_ply(&source_path).expect("load source");
        let t_gt = load_transform_txt(&t_path).expect("load T");

        // 对照 registration_test.cpp：downsample 0.3 + estimate covariances（20 近邻）
        let target_ds: PointCloud = crate::util::downsampling::voxelgrid_sampling(&target_raw, 0.3);
        let source_ds: PointCloud = crate::util::downsampling::voxelgrid_sampling(&source_raw, 0.3);
        let mut target = target_ds;
        let mut source = source_ds;
        let target_tree_pre: KdTree<PointCloud> = KdTree::new(target.clone());
        let source_tree_pre: KdTree<PointCloud> = KdTree::new(source.clone());
        crate::util::normal_estimation::estimate_normals_covariances_with_tree(
            &mut target,
            &target_tree_pre,
            20,
        );
        crate::util::normal_estimation::estimate_normals_covariances_with_tree(
            &mut source,
            &source_tree_pre,
            20,
        );
        let target_tree: KdTree<PointCloud> = KdTree::new(target.clone());

        // 初值 identity，阈值与 C++ 一致：max_dist 1.0，收敛容差默认
        let setting = RegistrationSetting {
            registration_type: RegistrationType::Gicp,
            max_correspondence_distance: 1.0,
            max_iterations: 20,
            ..Default::default()
        };
        let result = align(
            &target,
            &source,
            &target_tree,
            &Matrix4::identity(),
            &setting,
        );
        let (rot_err, trans_err) = error_between(&result.t_target_source, &t_gt);
        // registration_test.cpp 容差：rot 2.5° trans 0.2m
        eprintln!(
            "official GICP: rot_err={:.4}° trans_err={:.4}m inliers={}/{}",
            rot_err.to_degrees(),
            trans_err,
            result.num_inliers,
            source.num_points()
        );
        assert!(
            rot_err < 2.5_f64.to_radians(),
            "official rot err {rot_err:.4} rad >2.5°"
        );
        assert!(trans_err < 0.2, "official trans err {trans_err:.4} >0.2");

        // VGICP 对照
        let voxelmap = create_gaussian_voxelmap(&target, 1.0);
        let vgicp_res = align_vgicp(&voxelmap, &source, &Matrix4::identity(), &setting);
        let (rot_err_v, trans_err_v) = error_between(&vgicp_res.t_target_source, &t_gt);
        eprintln!(
            "official VGICP: rot_err={:.4}° trans_err={:.4}m",
            rot_err_v.to_degrees(),
            trans_err_v
        );
        assert!(rot_err_v < 2.5_f64.to_radians());
        assert!(trans_err_v < 0.2);
    }
}
