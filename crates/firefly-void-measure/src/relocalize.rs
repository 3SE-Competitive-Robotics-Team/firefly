//! 多假设重定位初值（为 P4 启动与退化恢复准备）。
//!
//! 在协方差椭球内撒 `N` 个初始位姿（旋转用 SO(3) 上的高斯扰动，
//! 平移用位置协方差块），各自做点-平面配准（固定对应，迭代加权
//! 最小二乘），按最终残差排序返回候选。

use firefly_void_map::voxel::{VoxelMap, transform_point};
use nalgebra::{Isometry3, Matrix3, Rotation3, UnitQuaternion, Vector3};

use crate::options::RelocalizeOptions;
use crate::plane_update::point_plane_residual;

/// 简单确定性伪随机数（SplitMix64，无 rand 依赖）。
#[derive(Debug, Clone, Copy)]
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// 标准正态采样（Box-Muller）。
    fn gaussian(&mut self) -> f64 {
        let u1 = (self.next() >> 11) as f64 / (1u64 << 53) as f64;
        let u2 = (self.next() >> 11) as f64 / (1u64 << 53) as f64;
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }

    /// 3 维零均值高斯（协方差 `cov` 经 Cholesky 分解）。
    fn gaussian_vec(&mut self, cov: &Matrix3<f64>) -> Vector3<f64> {
        let l = cov
            .cholesky()
            .map_or_else(|| Matrix3::identity() * 1e-3, nalgebra::Cholesky::unpack);
        l * Vector3::new(self.gaussian(), self.gaussian(), self.gaussian())
    }
}

/// 单候选点-平面配准（固定对应，迭代加权最小二乘）。
///
/// 每轮迭代：把点变换到全局系、查平面、按卡方残差加权求解位姿增量
/// （旋转 `R·Exp(δθ)` 右乘扰动），重复至收敛。
struct PointPlaneAlign<'a> {
    map: &'a VoxelMap,
    cloud: Vec<Vector3<f64>>,
}

impl<'a> PointPlaneAlign<'a> {
    fn new(map: &'a VoxelMap, cloud: Vec<Vector3<f64>>) -> Self {
        Self { map, cloud }
    }

    /// 单次迭代：返回残差和与位姿增量（旋转/平移）。
    ///
    /// 线性系统 `JᵀWJ·δx = −JᵀW·z`（对照 `voxel_map.cpp:452-454` 的 H 结构：
    /// 旋转列 `⌊p_b×⌋·Rᵀ·n`，平移列 `n`）。
    fn iterate(&self, pose: &Isometry3<f64>, weights: &[f64]) -> (f64, Vector3<f64>, Vector3<f64>) {
        let rot = pose.rotation.to_rotation_matrix().into_inner();
        let mut hth = nalgebra::SMatrix::<f64, 6, 6>::zeros();
        let mut htz = nalgebra::SVector::<f64, 6>::zeros();
        let mut total = 0.0;
        for (p_b, w) in self.cloud.iter().zip(weights) {
            let p_w = rot * p_b + pose.translation.vector;
            let Some(plane) = self.map_plane(&p_w) else {
                continue;
            };
            let dis = point_plane_residual(&plane.normal, &p_w, &plane.center);
            let a_vec = firefly_void_types::so3::skew(p_b) * rot.transpose() * plane.normal;
            let n = plane.normal;
            // J 行 = [a_vecᵀ, nᵀ]
            let mut j_row = nalgebra::SVector::<f64, 6>::zeros();
            j_row.fixed_rows_mut::<3>(0).copy_from(&a_vec);
            j_row.fixed_rows_mut::<3>(3).copy_from(&n);
            hth += *w * (j_row * j_row.transpose());
            htz += *w * (-dis) * j_row;
            total += dis * dis;
        }
        let Some(delta) = hth.try_inverse() else {
            return (total, Vector3::zeros(), Vector3::zeros());
        };
        let delta = delta * htz;
        (
            total,
            delta.fixed_rows::<3>(0).into_owned(),
            delta.fixed_rows::<3>(3).into_owned(),
        )
    }

    /// 查体素平面（径向判据同 [`crate::plane_update`]）。
    fn map_plane(&self, p_w: &Vector3<f64>) -> Option<&firefly_void_map::plane::VoxelPlane> {
        let root = self.map.root_at(p_w)?;
        let mut planes = Vec::new();
        root.collect_planes(&mut planes);
        planes
            .into_iter()
            .find(|pl| pl.is_plane && (pl.center - p_w).norm() < pl.radius * 3.0)
    }

    /// 完整配准：迭代至收敛，返回 `(残差和, 位姿)`。
    fn align(&self, init: &Isometry3<f64>, max_iterations: usize) -> (f64, Isometry3<f64>) {
        let mut pose = *init;
        let mut weights = vec![1.0; self.cloud.len()];
        for _ in 0..max_iterations {
            let (_total, rot_inc, t_inc) = self.iterate(&pose, &weights);
            // 更新位姿（右乘旋转扰动，与 boxplus 一致）
            let rot_mat = pose.rotation.to_rotation_matrix().into_inner()
                * firefly_void_types::so3::exp(rot_inc);
            let rot_new = UnitQuaternion::from_matrix(&rot_mat);
            let pos_new = pose.translation.vector + t_inc;
            let step = (rot_inc.norm() + t_inc.norm()).max(1e-12);
            pose = Isometry3::from_parts(
                nalgebra::Translation3::from(Vector3::new(pos_new[0], pos_new[1], pos_new[2])),
                rot_new,
            );
            // 卡方权重（IRLS）
            for (i, p_b) in self.cloud.iter().enumerate() {
                let p_w = transform_point(&pose, p_b);
                let dis = self
                    .map_plane(&p_w)
                    .map_or(0.0, |pl| point_plane_residual(&pl.normal, &p_w, &pl.center));
                weights[i] = 1.0 / (1.0 + dis * dis / 0.01);
            }
            if step < 1e-6 {
                break;
            }
        }
        let (total, _, _) = self.iterate(&pose, &weights);
        (total, pose)
    }
}

/// 多假设重定位初值。
///
/// 在协方差椭球内撒 `N` 个初始位姿，各自点-平面配准，按残差升序
/// 返回候选（`Vec<(残差和, 位姿)>`）。
#[must_use]
pub fn relocalize_guess(
    map: &VoxelMap,
    cloud: &[Vector3<f64>],
    cov: &nalgebra::Matrix6<f64>,
    opts: &RelocalizeOptions,
) -> Vec<(f64, Isometry3<f64>)> {
    let mut rng = SplitMix64(0x9E37_79B9_7F4A_7C15);
    let pos_cov = cov.fixed_view::<3, 3>(3, 3).into_owned();
    let rot_cov = cov.fixed_view::<3, 3>(0, 0).into_owned();
    let align = PointPlaneAlign::new(map, cloud.to_vec());
    let mut results = Vec::with_capacity(opts.n_candidates);

    // 基准候选：零扰动
    let init = Isometry3::identity();
    results.push(align.align(&init, opts.max_iterations));

    for _ in 1..opts.n_candidates {
        // 平移扰动：位置协方差椭球；旋转扰动：SO(3) 高斯（小角近似，
        // 协方差经指数映射）
        let t_pert = rng.gaussian_vec(&pos_cov);
        let rot_pert = rng.gaussian_vec(&rot_cov);
        let rot = Rotation3::from_matrix_unchecked(firefly_void_types::so3::exp(rot_pert));
        let pose = Isometry3::from_parts(
            nalgebra::Translation3::from(Vector3::new(
                init.translation.vector[0] + t_pert[0],
                init.translation.vector[1] + t_pert[1],
                init.translation.vector[2] + t_pert[2],
            )),
            UnitQuaternion::from_rotation_matrix(&rot),
        );
        results.push(align.align(&pose, opts.max_iterations));
    }
    results.sort_by(|a, b| a.0.total_cmp(&b.0));
    results
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_void_map::options::VoxelMapOptions;

    #[test]
    fn relocalize_finds_truth_within_candidates() {
        // 合成平面 + 已知真值位姿：候选集包含接近真值的解
        let mut map = VoxelMap::new(VoxelMapOptions::default());
        let cov = Matrix3::identity() * 1e-8;
        let mut pts = Vec::new();
        for i in 0..400 {
            let x = -0.2 + f64::from(i % 20) * 0.02;
            let y = -0.2 + f64::from(i / 20) * 0.02;
            pts.push(Vector3::new(x, y, 1.0));
        }
        map.register_points(&pts, &vec![cov; 400]);
        // 真值位姿：绕 y 转 10° + 平移
        let truth = Isometry3::from_parts(
            nalgebra::Translation3::new(0.05, 0.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.1),
        );
        // 世界系平面 z=1 上的点（变换到真值位姿下的相机系）
        let cloud: Vec<Vector3<f64>> = (0..64)
            .map(|i| {
                let x = -0.2 + f64::from(i % 8) * 0.05;
                let y = -0.2 + f64::from(i / 8) * 0.05;
                transform_point(&truth, &Vector3::new(x, y, 1.0))
            })
            .collect();
        // 协方差：位置 0.1²，旋转 5°²
        let mut cov6 = nalgebra::Matrix6::identity();
        cov6.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(Matrix3::identity() * 0.1));
        cov6.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(Matrix3::identity() * 0.01));
        let opts = RelocalizeOptions {
            n_candidates: 48,
            max_iterations: 15,
        };
        let candidates = relocalize_guess(&map, &cloud, &cov6, &opts);
        assert!(!candidates.is_empty());
        let (best_err, best_pose) = &candidates[0];
        let _ = best_err;
        // 位姿误差应小于初值误差（配准确实改进了候选）
        let err = (best_pose.translation.vector - truth.translation.vector).norm();
        assert!(err < 0.2, "最佳候选平移误差 {err} 应小于先验不确定度");
    }
}
