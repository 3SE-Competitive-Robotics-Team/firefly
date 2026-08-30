//! 深度点-平面测量模型（论文 VI 节）。
//!
//! 残差对照论文 (18)(19) 式：`0 = nᵀ(^G T_I · ^I T_L · ^L p_j − q)`，
//! 其中 `^I T_L` 为深度相机→IMU 外参（仿真为共面近似单位阵）。
//!
//! H 阵推导对照官方 `voxel_map.cpp:452-454`：
//! `A = ⌊p_b×⌋·Rᵀ·n`（`p_b` 为 IMU 系点，`R` 为当前位姿旋转）。
//! 残差 `h = nᵀ(R·p_b + p − q)`，对误差状态 `δx = [δθ, δp, …]`
//! （旋转右乘扰动，与 `State::boxplus` 一致）：
//! - `∂h/∂δθ = Aᵀ`（`A = ⌊p_b×⌋·Rᵀ·n`，推导：`∂(R·Exp(δθ)·p_b)/∂δθ
//!   = −R·⌊p_b×⌋`，代入 `h` 得 `nᵀ·(−R·⌊p_b×⌋) = (⌊p_b×⌋·Rᵀ·n)ᵀ`）；
//! - `∂h/∂δp = nᵀ`；
//! - 其余块为零。
//!
//! 测量噪声 `R` 对照 `voxel_map.cpp:447-449`：
//! `σ² = J_nq·Σ_nq·J_nqᵀ + nᵀ·Σ_pj·n + 0.001`（平面不确定度 +
//! 点不确定度，`J_nq = [p−q, −n]`，`Σ_pj` 由 [`crate::noise::DepthNoise`]
//! 预计算并旋转到世界系）。
//!
//! 退化保护：法向分布集中在少数方向（单平面近正对相机）时丢弃多余
//! 共面点，避免信息矩阵病态。

use firefly_void_esikf::update::MeasurementModel;
use firefly_void_map::voxel::{VoxelMap, transform_point};
use firefly_void_types::state::{DIM_STATE, State};
use firefly_void_types::visual::Intrinsics;
use nalgebra::{DMatrix, DVector, Isometry3, Matrix3, Vector3};

use crate::noise::DepthNoise;
use crate::options::DepthOptions;
use crate::outlier::{GateVerdict, chi2_gate};

/// 单点平面测量中间量 `(p_b, n, dis, σ_l²)`：IMU 系点、平面法向、
/// 残差距离与噪声方差。
type PlaneResidual = Option<(Vector3<f64>, Vector3<f64>, f64, f64)>;

/// 单点平面残差（论文 (18) 式：`dis_to_plane = nᵀ(p_w − q)`）。
#[must_use]
pub fn point_plane_residual(
    normal: &Vector3<f64>,
    p_world: &Vector3<f64>,
    center: &Vector3<f64>,
) -> f64 {
    normal.dot(&(p_world - center))
}

/// 深度点-平面测量模型。
///
/// 每次构造对应一帧深度点云（已反投影/下采样），`residual` 在当前
/// 估计位姿下把点变换到全局系、查体素平面并计算残差。
pub struct DepthMeasurement<'a> {
    map: &'a VoxelMap,
    /// 深度相机系点云。
    points_l: Vec<Vector3<f64>>,
    /// 各点相机系协方差 `Σ_pj`（由 [`DepthNoise`] 预计算）。
    covs: Vec<Matrix3<f64>>,
    /// 深度相机→IMU 外参。
    ext: Isometry3<f64>,
    opts: DepthOptions,
}

impl<'a> DepthMeasurement<'a> {
    /// 构造：`points_l` 为深度相机系点云，`covs` 为各点相机系协方差。
    ///
    /// # Panics
    /// `points_l.len() != covs.len()` 时 panic。
    #[must_use]
    pub fn new(
        map: &'a VoxelMap,
        points_l: Vec<Vector3<f64>>,
        covs: Vec<Matrix3<f64>>,
        ext: Isometry3<f64>,
        opts: DepthOptions,
    ) -> Self {
        assert_eq!(points_l.len(), covs.len(), "点与协方差数量必须一致");
        Self {
            map,
            points_l,
            covs,
            ext,
            opts,
        }
    }

    /// 由深度图反投影构造（逐像素反投影，`depth ≤ 0.05` 的空洞丢弃）。
    ///
    /// 反投影：`p_cam = ((u−cx)/fx, (v−cy)/fy, 1)·z`；协方差由
    /// [`DepthNoise::point_covariance`] 给出（含 `σ∝z²` 与空洞邻域项）。
    #[must_use]
    pub fn from_depth_frame(
        map: &'a VoxelMap,
        depth: &[f64],
        width: usize,
        height: usize,
        intrinsics: Intrinsics,
        ext: Isometry3<f64>,
        opts: DepthOptions,
    ) -> Self {
        let noise = DepthNoise::from_intrinsics(&opts, intrinsics.fx, intrinsics.fy);
        let mut points_l = Vec::new();
        let mut covs = Vec::new();
        for y in 0..height {
            for x in 0..width {
                let z = depth[y * width + x];
                if z <= 0.05 || !z.is_finite() {
                    continue;
                }
                let p = Vector3::new(
                    (x as f64 - intrinsics.cx) / intrinsics.fx * z,
                    (y as f64 - intrinsics.cy) / intrinsics.fy * z,
                    z,
                );
                covs.push(noise.point_covariance(&p));
                points_l.push(p);
            }
        }
        Self::new(map, points_l, covs, ext, opts)
    }

    /// 有效点数（上次 `residual` 计算后的有效平面点数）。
    #[must_use]
    pub fn effective_count(&self, x: &State) -> usize {
        let (z, _, r) = self.build(x);
        z.iter()
            .zip(r.diagonal().iter())
            .filter(|&(_, sig)| *sig < 1e6)
            .count()
    }

    /// 查体素平面并计算单点残差/噪声（对照 `build_single_residual`，
    /// `voxel_map.cpp:713-786`：径向判据 `radius_k` + 卡方门控）。
    ///
    /// `cov_w` 为点在世界系的协方差（`R·C·Rᵀ`）。
    ///
    /// 返回 `(n, dis, σ_l²)`。
    fn plane_for_point(
        &self,
        p_world: &Vector3<f64>,
        cov_w: &Matrix3<f64>,
    ) -> Option<(Vector3<f64>, f64, f64)> {
        let root = self.map.root_at(p_world)?;
        let mut planes = Vec::new();
        root.collect_planes(&mut planes);
        // 取概率最高（残差/噪声最小）的平面（对照官方对八叉子节点的
        // `this_prob` 择优）。
        let mut best: Option<(Vector3<f64>, f64, f64)> = None;
        let mut best_prob = f64::NEG_INFINITY;
        for plane in planes {
            if !plane.is_plane {
                continue;
            }
            let dis = point_plane_residual(&plane.normal, p_world, &plane.center);
            // 径向判据（对照 voxel_map.cpp:726-730）
            let dis_to_center = (plane.center - p_world).norm_squared();
            let range_dis = (dis_to_center - dis * dis).max(0.0).sqrt();
            if range_dis > self.opts.radius_k * plane.radius {
                continue;
            }
            // 噪声：J_nq·Σ_nq·J_nqᵀ + nᵀ·Σ_pj·n（对照 voxel_map.cpp:447-449）
            let j_nq = nalgebra::RowVector6::new(
                p_world[0] - plane.center[0],
                p_world[1] - plane.center[1],
                p_world[2] - plane.center[2],
                -plane.normal[0],
                -plane.normal[1],
                -plane.normal[2],
            );
            let sigma_l = j_nq * plane.plane_var * j_nq.transpose();
            let sigma_l = sigma_l[(0, 0)] + plane.normal.dot(&(cov_w * plane.normal));
            let sigma_l = sigma_l + 0.001;
            // 卡方门控（对照 voxel_map.cpp:737）
            if chi2_gate(dis, sigma_l, self.opts.sigma_num) == GateVerdict::Outlier {
                continue;
            }
            let prob = (-0.5 * dis * dis / sigma_l).exp() / sigma_l.sqrt();
            if prob > best_prob {
                best_prob = prob;
                best = Some((plane.normal, dis, sigma_l));
            }
        }
        best
    }

    /// 计算残差、H 与 R（对照 `voxel_map.cpp:414-458` 的 H 组装）。
    ///
    /// 固定维度契约：返回行数恒为 `points_l.len()`（与 [`dim`] 一致）。
    /// 无效点（无对应平面/外点/退化过滤）填零信息行（`z=0, H=0, R=1e12`），
    /// 对 KF 不贡献信息，保证与 esikf 的 `dim()`→`residual()` 调用序一致。
    #[allow(clippy::many_single_char_names)] // 单字符 `n`/`h`/`r`/`m` 为论文 (18)(19) 式记号
    fn build(&self, x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
        let rot = x.rot.matrix();
        // 相机光轴（世界系）＝当前姿态的 +z。
        let cam_axis = rot * Vector3::z_axis().into_inner();
        let mut residuals: Vec<PlaneResidual> = Vec::new();
        let mut normals: Vec<Vector3<f64>> = Vec::new();

        for (p_l, cov) in self.points_l.iter().zip(&self.covs) {
            let p_b = transform_point(&self.ext, p_l);
            let p_w = rot * p_b + x.pos;
            let cov_w = rot * cov * rot.transpose();
            match self.plane_for_point(&p_w, &cov_w) {
                Some((n, dis, sig)) => {
                    normals.push(n);
                    residuals.push(Some((p_b, n, dis, sig)));
                }
                None => residuals.push(None),
            }
        }

        // 共面退化保护：法向集中在少数方向时丢弃多余点（置零信息）。
        // normals 与 residuals 的 Some 项一一对应（下标经有效计数映射）。
        let keep_mask = self.degenerate_filter(&normals, &residuals, &cam_axis);

        let mut zs = Vec::with_capacity(self.points_l.len());
        let mut rs = Vec::with_capacity(self.points_l.len());
        let zero_info = 1e12;
        for (i, res) in residuals.iter().enumerate() {
            if let (Some((_p_b, _n, dis, sig)), Some(true)) = (res, keep_mask.get(i)) {
                zs.push(*dis);
                rs.push(*sig);
            } else {
                zs.push(0.0);
                rs.push(zero_info);
            }
        }

        let n_rows = self.points_l.len();
        let z_vec = DVector::from_iterator(n_rows, zs);
        let mut h_mat = DMatrix::zeros(n_rows, DIM_STATE);
        let mut r_mat = DMatrix::from_diagonal(&DVector::from_iterator(n_rows, rs.clone()));
        let zero_info_row = h_mat.row_mut(0).clone_owned(); // 零行
        for (i, res) in residuals.iter().enumerate() {
            let Some((p_b, n, _, _)) = res else { continue };
            if !keep_mask.get(i).copied().unwrap_or(false) {
                continue;
            }
            let p_cross = firefly_void_types::so3::skew(p_b);
            let a_vec = p_cross * rot.transpose() * n;
            let mut row = zero_info_row.clone_owned();
            for k in 0..3 {
                row[k] = a_vec[k];
                row[3 + k] = n[k];
            }
            h_mat.set_row(i, &row);
            r_mat[(i, i)] = rs[i];
        }
        (z_vec, h_mat, r_mat)
    }

    /// 共面退化保护。
    ///
    /// 主法向与相机光轴夹角余弦 < `min_cos_plane_normal`（近正对，
    /// 切向不可观）且单一法向占比超过 `max_single_normal_ratio` 时，
    /// 按法向-光轴夹角分桶丢弃多余点（每桶保留一个），避免信息矩阵病态。
    ///
    /// `normals` 与 `residuals` 中的 `Some` 项一一对应（下标同源）。
    /// 返回与 `residuals` 等长的布尔掩码（`true` 保留）。
    fn degenerate_filter(
        &self,
        normals: &[Vector3<f64>],
        residuals: &[PlaneResidual],
        cam_axis: &Vector3<f64>,
    ) -> Vec<bool> {
        let mut keep = vec![false; residuals.len()];
        // 先标记有有效平面的点
        let mut valid_idx: Vec<usize> = Vec::new();
        for (i, res) in residuals.iter().enumerate() {
            if res.is_some() {
                keep[i] = true;
                valid_idx.push(i);
            }
        }
        if valid_idx.is_empty() {
            return keep;
        }
        let mean_n: Vector3<f64> = normals.iter().sum::<Vector3<f64>>() / normals.len() as f64;
        let mean_n = mean_n.normalize();
        let cos_angle = mean_n.dot(cam_axis).abs();
        if cos_angle >= self.opts.min_cos_plane_normal {
            return keep;
        }
        let n_bins = 8;
        let mut bin_count = vec![0usize; n_bins];
        for n in normals {
            let c = n.dot(cam_axis).clamp(-1.0, 1.0).acos();
            let bin = ((c / std::f64::consts::PI) * n_bins as f64) as usize;
            bin_count[bin.min(n_bins - 1)] += 1;
        }
        let max_bin = *bin_count.iter().max().unwrap_or(&0);
        if max_bin as f64 <= self.opts.max_single_normal_ratio * valid_idx.len() as f64 {
            return keep;
        }
        let mut kept_bins = vec![false; n_bins];
        for &i in &valid_idx {
            let n = &residuals[i].as_ref().expect("valid_idx 仅含 Some 项").1;
            let c = n.dot(cam_axis).clamp(-1.0, 1.0).acos();
            let bin = ((c / std::f64::consts::PI) * n_bins as f64) as usize;
            let bin = bin.min(n_bins - 1);
            if kept_bins[bin] {
                keep[i] = false;
                continue;
            }
            kept_bins[bin] = true;
        }
        keep
    }
}

impl MeasurementModel for DepthMeasurement<'_> {
    fn residual(&self, x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
        self.build(x)
    }

    fn dim(&self) -> usize {
        self.points_l.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_void_map::VoxelMap;
    use firefly_void_map::options::VoxelMapOptions;
    use firefly_void_types::state::ErrorState;
    use nalgebra::{Rotation3, Translation3, UnitQuaternion};

    /// 构造含一个平面（z=1，法向 ±z）的地图：20×20 网格，跨度 ±0.2 m。
    fn map_with_plane_z1() -> VoxelMap {
        let mut map = VoxelMap::new(VoxelMapOptions::default());
        let cov = Matrix3::identity() * 1e-8;
        let mut pts = Vec::with_capacity(400);
        for i in 0..400 {
            let x = -0.2 + f64::from(i % 20) * 0.02;
            let y = -0.2 + f64::from(i / 20) * 0.02;
            pts.push(Vector3::new(x, y, 1.0));
        }
        let covs = vec![cov; 400];
        map.register_points(&pts, &covs);
        map
    }

    fn identity_pose() -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), UnitQuaternion::identity())
    }

    /// 平面 z=1 上的深度点（相机系 = 世界系，共面）。
    fn plane_points(n: usize) -> Vec<Vector3<f64>> {
        let side = (n as f64).sqrt().ceil() as usize;
        (0..n)
            .map(|i| {
                let x = -0.2 + (i % side) as f64 * 0.4 / side as f64;
                let y = -0.2 + (i / side) as f64 * 0.4 / side as f64;
                Vector3::new(x, y, 1.0)
            })
            .collect()
    }

    fn model(map: &VoxelMap, pts: Vec<Vector3<f64>>) -> DepthMeasurement<'_> {
        let covs = vec![Matrix3::identity() * 1e-8; pts.len()];
        DepthMeasurement::new(map, pts, covs, identity_pose(), DepthOptions::default())
    }

    /// 数值雅可比（中心差分）：对 19 维误差状态逐分量扰动。
    fn numeric_h<F: Fn(&State) -> DVector<f64>>(x: &State, f: F, eps: f64) -> DMatrix<f64> {
        let z0 = f(x);
        let m = z0.len();
        let mut h = DMatrix::zeros(m, DIM_STATE);
        for j in 0..DIM_STATE {
            let mut dp = ErrorState::zeros();
            dp[j] = eps;
            let zp = f(&x.boxplus(&dp));
            dp[j] = -eps;
            let zm = f(&x.boxplus(&dp));
            h.set_column(j, &((&zp - &zm) / (2.0 * eps)));
        }
        h
    }

    #[test]
    fn point_plane_residual_zero_on_plane() {
        let n = Vector3::z_axis().into_inner();
        let c = Vector3::new(0.0, 0.0, 1.0);
        let on_plane = Vector3::new(0.1, -0.2, 1.0);
        assert!(point_plane_residual(&n, &on_plane, &c).abs() < 1e-12);
        let off_plane = Vector3::new(0.1, -0.2, 1.05);
        assert!((point_plane_residual(&n, &off_plane, &c) - 0.05).abs() < 1e-12);
    }

    #[test]
    fn residual_zero_at_truth_pose() {
        // 已知真值位姿 + 完美平面：残差 ≈ 0（有效行）
        let map = map_with_plane_z1();
        let model = model(&map, plane_points(64));
        let state = State::default();
        let (z_vec, h_mat, r_mat) = model.residual(&state);
        assert_eq!(z_vec.len(), 64, "固定维度 = 点数");
        // 有效行（R 小）残差 ≈ 0
        let valid: Vec<f64> = z_vec
            .iter()
            .zip(r_mat.diagonal().iter())
            .filter(|&(_, sig)| *sig < 1e6)
            .map(|(v, _)| *v)
            .collect();
        assert!(!valid.is_empty(), "应有有效平面点");
        assert!(valid.iter().all(|v| v.abs() < 1e-6), "残差应≈0: {valid:?}");
        assert_eq!(h_mat.nrows(), z_vec.len());
        assert_eq!(h_mat.ncols(), DIM_STATE);
        assert_eq!(r_mat.nrows(), z_vec.len());
        assert_eq!(model.dim(), 64);
    }

    #[test]
    fn jacobian_matches_finite_difference() {
        // 位姿扰动 5cm：解析 H 与数值雅可比一致（相对误差 < 1e-6）
        let map = map_with_plane_z1();
        let model = model(&map, plane_points(64));
        let x = State {
            pos: Vector3::new(0.05, -0.02, 0.01),
            rot: Rotation3::from_axis_angle(&Vector3::y_axis(), 0.03),
            ..State::default()
        };
        let (_, h_mat, _) = model.residual(&x);

        let f = |s: &State| -> DVector<f64> {
            let (z, _, _) = model.residual(s);
            z
        };
        let h_num = numeric_h(&x, f, 1e-6);
        let err = (&h_mat - &h_num).norm() / (h_mat.norm() + 1e-12);
        assert!(err < 1e-6, "解析 H 与数值 H 相对误差 {err}");
    }

    #[test]
    fn noise_grows_with_depth() {
        // 深度噪声传播：Σ_pj 随 z 增长（远距点协方差更大）
        let opts = DepthOptions::default();
        let noise = DepthNoise::from_intrinsics(&opts, 300.0, 300.0);
        let c_near = noise.point_covariance(&Vector3::new(0.1, 0.1, 1.0));
        let c_far = noise.point_covariance(&Vector3::new(0.1, 0.1, 3.0));
        assert!(
            c_far.trace() > c_near.trace() * 5.0,
            "tr(Σ_3m)={} 应大于 tr(Σ_1m)={}",
            c_far.trace(),
            c_near.trace()
        );
    }

    #[test]
    fn degenerate_filter_keeps_single_point_per_bin() {
        // 退化保护：法向集中在单方向且与光轴夹角大时，每桶保留一个点
        let opts = DepthOptions::default();
        let map = map_with_plane_z1();
        let m = DepthMeasurement::new(&map, Vec::new(), Vec::new(), identity_pose(), opts);
        // 法向全部指向 +x（与光轴 +z 夹角 90° → cos≈0 < 0.9，触发退化）
        let normal = Vector3::x_axis().into_inner();
        let normals = vec![normal; 50];
        let residuals: Vec<PlaneResidual> = (0..50)
            .map(|_i| Some((Vector3::new(0.0, 0.0, 1.0), normal, 0.0, 0.001)))
            .collect();
        let cam_axis = Vector3::z_axis().into_inner();
        let keep = m.degenerate_filter(&normals, &residuals, &cam_axis);
        // 全部同一法向 → 只保留 1 个点（单桶）
        assert_eq!(
            keep.iter().filter(|&&k| k).count(),
            1,
            "退化时应丢弃多余共面点"
        );
    }
}
