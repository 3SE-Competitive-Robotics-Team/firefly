//! 叶体素局部平面（论文 V-A/V-B，对照 `voxel_map.h:69` `VoxelPlane`）。
//!
//! 平面参数 `(q, n)` 与不确定度 `Σ_nq` 由体素内点集 SVD 得到：
//! - `q`：平面中心（点均值）；
//! - `n`：最小特征值对应特征向量（法向）；
//! - `Σ_nq`：6×6 协方差（法向+中心联合，含点测量噪声传播，
//!   对照 `voxel_map.cpp:88-111` 的 J 矩阵线性化）。
//!
//! 特征值/特征向量索引记号（`min_idx`/`mid_idx`/`max_idx` 与
//! `eigen_min`/`eigen_mid`/`eigen_max`）为 SVD 惯例，予以模块级允许。
#![allow(clippy::similar_names)]

use nalgebra::{Matrix3, Matrix6, Vector3};

use crate::options::{MAX_POINTS_PER_PLANE, PlaneOptions};

/// 局部平面：`n·x + d = 0`（`n` 单位法向，`d` 由中心算出）。
#[derive(Debug, Clone)]
pub struct VoxelPlane {
    /// 平面中心 `q`（世界系，单位 m）。
    pub center: Vector3<f64>,
    /// 平面法向 `n`（世界系，单位向量）。
    pub normal: Vector3<f64>,
    /// `n·x + d = 0` 的常数项。
    pub d: f64,
    /// 6×6 协方差 `Σ_nq`（块序 `[n, q]`）。
    pub plane_var: Matrix6<f64>,
    /// 点集协方差（用于半径/特征值统计）。
    pub covariance: Matrix3<f64>,
    /// 点分布半径 `sqrt(λ_max)`（用于最近邻判据，对照 `voxel_map.cpp:119`）。
    pub radius: f64,
    /// 最小/中间/最大特征值。
    pub eigen_min: f64,
    pub eigen_mid: f64,
    pub eigen_max: f64,
    /// 参与拟合的点数。
    pub points_count: usize,
    /// 是否判为平面（`eigen_min < threshold`）。
    pub is_plane: bool,
    /// 是否成熟（参数收敛固定，新点丢弃）。
    pub is_mature: bool,
}

impl Default for VoxelPlane {
    fn default() -> Self {
        Self {
            center: Vector3::zeros(),
            normal: Vector3::zeros(),
            d: 0.0,
            plane_var: Matrix6::zeros(),
            covariance: Matrix3::zeros(),
            radius: 0.0,
            eigen_min: 1.0,
            eigen_mid: 1.0,
            eigen_max: 1.0,
            points_count: 0,
            is_plane: false,
            is_mature: false,
        }
    }
}

/// 由点集拟合平面（SVD），返回 `Some(plane)` 当且仅当判为平面。
///
/// 对照 `voxel_map.cpp:55` `init_plane`：
/// 1. 均值 = 中心 `q`，点集协方差 `C = Σ(p pᵀ)/N − qqᵀ`；
/// 2. `C` 特征分解（nalgebra `SymmetricEigen`），最小特征值向量 = 法向 `n`；
/// 3. `eigen_min < planer_threshold` 判为平面，否则返回 `None`；
/// 4. 成熟判定：点数达到 [`MAX_POINTS_PER_PLANE`] 后参数固定、新点丢弃。
///
/// # Panics
/// `points.len() < 3` 或 `points.len() != covs.len()` 时 panic。
#[must_use]
pub fn fit_plane(
    points: &[Vector3<f64>],
    covs: &[Matrix3<f64>],
    opts: &PlaneOptions,
) -> Option<VoxelPlane> {
    assert_eq!(points.len(), covs.len(), "点与协方差数量必须一致");
    assert!(points.len() >= 3, "平面拟合至少需要 3 个点");
    let n = points.len();

    let mut center = Vector3::zeros();
    let mut scatter = Matrix3::zeros();
    for p in points {
        center += p;
        scatter += p * p.transpose();
    }
    center /= n as f64;
    let covariance = scatter / n as f64 - center * center.transpose();

    // 对称 3×3 特征分解（nalgebra 不保证特征值顺序，显式找最小/最大）
    let eig = covariance.symmetric_eigen();
    let mut min_idx = 0;
    let mut max_idx = 0;
    for i in 1..3 {
        if eig.eigenvalues[i] < eig.eigenvalues[min_idx] {
            min_idx = i;
        }
        if eig.eigenvalues[i] > eig.eigenvalues[max_idx] {
            max_idx = i;
        }
    }
    let mid_idx = 3 - min_idx - max_idx;
    let eigen_min = eig.eigenvalues[min_idx];
    let eigen_mid = eig.eigenvalues[mid_idx];
    let eigen_max = eig.eigenvalues[max_idx];
    let normal = eig.eigenvectors.column(min_idx).into_owned();

    if eigen_min >= opts.planer_threshold {
        return None;
    }

    // Σ_nq：法向/中心对点位置的雅可比传播（对照 voxel_map.cpp:88-111）
    //
    // J 为 6×3：上 3 行 = evec·F（法向对点位置），下 3 行 = (1/N)·I（中心对点位置）。
    // Σ_nq += J · var(3×3) · Jᵀ，结果 6×6（块序 [n, q]）。
    let mut plane_var = Matrix6::zeros();
    let inv_n = 1.0 / n as f64;
    for (p, var) in points.iter().zip(covs) {
        let diff = p - center;
        // F 为 3×3，行 m（m ≠ min_idx）对特征向量 v_m 的导数，min_idx 行为零
        let mut f = Matrix3::zeros();
        for m in 0..3 {
            if m == min_idx {
                continue;
            }
            let v_m = eig.eigenvectors.column(m);
            let v_min = eig.eigenvectors.column(min_idx);
            let denom = n as f64 * (eigen_min - eig.eigenvalues[m]);
            let f_m =
                (diff.transpose() / denom) * (v_m * v_min.transpose() + v_min * v_m.transpose());
            f.set_row(m, &f_m);
        }
        let j_top = eig.eigenvectors * f; // 3×3
        // J 6×3
        let mut j = nalgebra::SMatrix::<f64, 6, 3>::zeros();
        j.fixed_view_mut::<3, 3>(0, 0).copy_from(&j_top);
        for i in 0..3 {
            j[(3 + i, i)] = inv_n;
        }
        let j_var = j * var * j.transpose(); // 6×6
        plane_var += j_var;
    }

    let d = -(normal.dot(&center));
    let is_mature = n >= MAX_POINTS_PER_PLANE;
    Some(VoxelPlane {
        center,
        normal,
        d,
        plane_var,
        covariance,
        radius: eigen_max.sqrt(),
        eigen_min,
        eigen_mid,
        eigen_max,
        points_count: n,
        is_plane: true,
        is_mature,
    })
}
