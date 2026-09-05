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

use firefly_void_esikf::update::MeasurementModel;
use firefly_void_map::voxel::{VoxelMap, transform_point};
use firefly_void_types::state::{DIM_STATE, State};
use nalgebra::{DMatrix, DVector, Isometry3, Matrix3, Vector3};
use std::cell::Cell;

use crate::options::DepthOptions;
use crate::planar::{PlaneQuery, match_plane};

/// 单点平面测量中间量 `(p_b, n, dis, σ_l²)`：IMU 系点、平面法向、
/// 残差距离与噪声方差。
type PlaneResidual = Option<(Vector3<f64>, Vector3<f64>, f64, f64)>;

/// 深度测量逐帧诊断（探针，不参与算法决策）。
#[derive(Debug, Clone, Copy, Default)]
pub struct DepthDiag {
    /// 点云总点数。
    pub total: usize,
    /// 无对应平面（无体素 / 无平面 / 径向判据不过）。
    pub no_plane: usize,
    /// 卡方门控拒绝。
    pub chi2_rejected: usize,
    /// 最终有效点（卡方过滤后）。
    pub kept: usize,
}

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
    /// 最近一次 `residual`/`effective_count` 的逐点拒绝统计（探针）。
    last_diag: Cell<DepthDiag>,
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
            last_diag: Cell::new(DepthDiag::default()),
        }
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

    /// 最近一次 `residual`/`effective_count` 的逐点拒绝统计（探针）。
    #[must_use]
    pub fn last_diag(&self) -> DepthDiag {
        self.last_diag.get()
    }

    /// 查体素平面并计算单点残差/噪声（对照 `build_single_residual`，
    /// `voxel_map.cpp:713-786`：径向判据 `radius_k` + 卡方门控）。
    ///
    /// `cov_w` 为点在世界系的协方差（`R·C·Rᵀ`）。
    ///
    /// 返回匹配结果（含拒绝原因，探针）。候选平面取自在线体素图的
    /// 根体素八叉树，判据本体在 [`match_plane`]（与先验测量共用）。
    fn plane_for_point(&self, p_world: &Vector3<f64>, cov_w: &Matrix3<f64>) -> PlaneQuery {
        let Some(root) = self.map.root_at(p_world) else {
            return PlaneQuery::NoPlane;
        };
        let mut planes = Vec::new();
        root.collect_planes(&mut planes);
        match_plane(
            &planes,
            p_world,
            cov_w,
            self.opts.radius_k,
            self.opts.sigma_num,
            1.0, // 在线地图 Σ_nq 已含真实传播，不放大
        )
    }

    /// 计算残差、H 与 R（对照 `voxel_map.cpp:414-458` 的 H 组装）。
    ///
    /// 固定维度契约：返回行数恒为 `points_l.len()`（与 [`dim`] 一致）。
    /// 无效点（无对应平面/外点）填零信息行（`z=0, H=0, R=1e12`），
    /// 对 KF 不贡献信息，保证与 esikf 的 `dim()`→`residual()` 调用序一致。
    #[allow(clippy::many_single_char_names)] // 单字符 `n`/`h`/`r`/`m` 为论文 (18)(19) 式记号
    fn build(&self, x: &State) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
        let rot = x.rot.matrix();
        let mut residuals: Vec<PlaneResidual> = Vec::new();
        let mut diag = DepthDiag {
            total: self.points_l.len(),
            ..DepthDiag::default()
        };

        for (p_l, cov) in self.points_l.iter().zip(&self.covs) {
            let p_b = transform_point(&self.ext, p_l);
            let p_w = rot * p_b + x.pos;
            let cov_w = rot * cov * rot.transpose();
            match self.plane_for_point(&p_w, &cov_w) {
                PlaneQuery::Matched(n, dis, sig) => {
                    residuals.push(Some((p_b, n, dis, sig)));
                }
                PlaneQuery::NoPlane => {
                    diag.no_plane += 1;
                    residuals.push(None);
                }
                PlaneQuery::Chi2Rejected => {
                    diag.chi2_rejected += 1;
                    residuals.push(None);
                }
            }
        }

        diag.kept = residuals.iter().filter(|r| r.is_some()).count();
        self.last_diag.set(diag);

        let mut zs = Vec::with_capacity(self.points_l.len());
        let mut rs = Vec::with_capacity(self.points_l.len());
        let zero_info = 1e12;
        for res in &residuals {
            if let Some((_p_b, _n, dis, sig)) = res {
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
    use crate::noise::DepthNoise;
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
        map.register_points(&pts, &covs, &Vector3::zeros());
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
}
