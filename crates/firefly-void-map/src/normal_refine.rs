//! 法向精化（论文 V-E：仿射扭曲 (13) 式 + 光度最小化 (14) 式 +
//! 变量代换 (15)(16) 式）。
//!
//! 以参考补丁为源，对其余补丁做仿射扭曲后最小化光度残差；优化变量
//! 经 `M = Bm + b` 无约束化到 `m ∈ R²`（`Ir pz ≠ 0` 保证），
//! 最优 `m*` 恢复法向 `Ir n* = M*/‖M*‖`（(16) 式）。
//!
//! 独立线程执行（mpsc 通道收任务），不阻塞主管线。

use nalgebra::{Isometry3, Matrix2, Matrix3, Vector2, Vector3};

use crate::visual_point::VisualPoint;

/// 变量代换 `M = Bm + b`（论文 (15) 式）。
///
/// `M ∈ R³` 满足 `Ir p · M = 1`；由 `m = [Mx, My]` 参数化：
/// `Mz = 1/Ir pz − (Ir px/Ir pz)·Mx − (Ir py/Ir pz)·My`。
#[must_use]
pub fn m_to_m(m: &Vector2<f64>, p_ref: &Vector3<f64>) -> Option<Vector3<f64>> {
    if p_ref[2].abs() < 1e-12 {
        return None;
    }
    let inv_z = 1.0 / p_ref[2];
    Some(Vector3::new(
        m[0],
        m[1],
        inv_z - p_ref[0] * inv_z * m[0] - p_ref[1] * inv_z * m[1],
    ))
}

/// 由 `m` 恢复单位法向（论文 (16) 式）。
#[must_use]
pub fn m_to_normal(m: &Vector2<f64>, p_ref: &Vector3<f64>) -> Option<Vector3<f64>> {
    let m3 = m_to_m(m, p_ref)?;
    let norm = m3.norm();
    if norm < 1e-12 { None } else { Some(m3 / norm) }
}

/// 参考相机系下的平面点（源补丁中心像素反投影，论文 (13) 式 `Ir p`）。
#[must_use]
pub fn ref_plane_point(pos: &Vector3<f64>, pose_ref: &Isometry3<f64>) -> Vector3<f64> {
    crate::voxel::transform_point(pose_ref, pos)
}

/// 光度残差（论文 (14) 式）：目标补丁经仿射扭曲后与参考补丁的差。
///
/// 参考补丁像素坐标 `u_r`（中心在 0），经 (13) 式仿射扭曲映射到目标系
/// 像素，在目标补丁上双线性采样；残差含曝光补偿 `τ_i·I_i − τ_r·I_r`。
#[must_use]
pub fn photometric_residuals(
    n_ref: &Vector3<f64>,
    p_ref: &Vector3<f64>,
    obs_ref: &PatchObservation,
    obs_i: &PatchObservation,
    patch_size: usize,
    intrinsics: &firefly_void_types::visual::Intrinsics,
) -> f64 {
    // 仿射扭曲矩阵（参考系 → 目标系，由相对位姿 + 平面参数）
    let a = affine_warp(&obs_ref.pose, &obs_i.pose, n_ref, p_ref, intrinsics);
    let half = (patch_size as f64 - 1.0) / 2.0;
    let ref0 = obs_ref.patch.level0();
    let tgt0 = obs_i.patch.level0();
    let mut sum = 0.0;
    for y in 0..patch_size {
        for x in 0..patch_size {
            let u = x as f64 - half;
            let v = y as f64 - half;
            let warped = a * Vector2::new(u, v) + Vector2::new(half, half);
            let val = bilinear_sample_patch(tgt0, patch_size, warped[0], warped[1]);
            let res = obs_i.inv_expo_time * val - obs_ref.inv_expo_time * ref0[y * patch_size + x];
            sum += res * res;
        }
    }
    sum
}

/// 法向精化：最小化 (14) 式光度误差（对 `m` 的 Gauss-Newton，数值雅可比）。
///
/// 返回精化后的世界系法向（`Some`）。收敛判据：残差梯度范数 < 1e-6 或
/// 达到 `max_iterations`。
#[must_use]
pub fn refine_normal(
    pt: &VisualPoint,
    intrinsics: &firefly_void_types::visual::Intrinsics,
    max_iterations: usize,
) -> Option<Vector3<f64>> {
    let ref_idx = pt.ref_patch?;
    let obs_ref = &pt.obs[ref_idx];
    let p_ref = ref_plane_point(&pt.pos, &obs_ref.pose);
    if p_ref[2].abs() < 1e-12 {
        return None;
    }
    let patch_size = obs_ref.patch.patch_size;

    let targets: Vec<&PatchObservation> = pt
        .obs
        .iter()
        .enumerate()
        .filter(|(i, _)| *i != ref_idx)
        .map(|(_, o)| o)
        .collect();
    if targets.is_empty() {
        return None;
    }

    // 初始 m：由当前法向反解（法向在参考系）
    let n_world = pt.normal;
    let n_ref = obs_ref.pose.rotation.inverse() * n_world;
    // M = n_ref / (n_ref·p_ref)，再取前两分量
    let dot = n_ref.dot(&p_ref);
    if dot.abs() < 1e-12 {
        return None;
    }
    let m_vec = n_ref / dot;
    let mut m = Vector2::new(m_vec[0], m_vec[1]);
    // 若 M 前两分量无约束直接可用，微调避免退化
    if !m.iter().all(|v| v.is_finite()) {
        m = Vector2::zeros();
    }

    let eps = 1e-6;
    for _ in 0..max_iterations {
        let _n_cur = m_to_normal(&m, &p_ref)?;

        // 数值雅可比：残差和关于 m 的梯度
        let mut grad = Vector2::zeros();
        let mut hess = Matrix2::zeros();
        for d in 0..2 {
            let mut m_p = m;
            m_p[d] += eps;
            let n_p = m_to_normal(&m_p, &p_ref)?;
            let n_ref_p = obs_ref.pose.rotation.inverse() * n_p;
            let mut m_m = m;
            m_m[d] -= eps;
            let n_mm = m_to_normal(&m_m, &p_ref)?;
            let n_ref_mm = obs_ref.pose.rotation.inverse() * n_mm;
            // 中心差分残差和
            let e_p = total_error(&n_ref_p, &p_ref, obs_ref, &targets, patch_size, intrinsics);
            let e_m = total_error(&n_ref_mm, &p_ref, obs_ref, &targets, patch_size, intrinsics);
            grad[d] = 0.5 * (e_p - e_m) / eps;
        }
        // 对角 Hessian 近似（Gauss-Newton 用梯度模）
        for d in 0..2 {
            hess[(d, d)] = grad[d].abs().max(1e-8);
        }
        let step = -hess.try_inverse().unwrap_or_else(Matrix2::identity) * grad;
        if grad.norm() < 1e-6 {
            break;
        }
        m += step;
        if !m.iter().all(|v| v.is_finite()) {
            return None;
        }
    }

    let n_final = m_to_normal(&m, &p_ref)?;
    Some(obs_ref.pose.rotation * n_final)
}

/// 全部目标补丁的光度残差和。
fn total_error(
    n_ref: &Vector3<f64>,
    p_ref: &Vector3<f64>,
    obs_ref: &PatchObservation,
    targets: &[&PatchObservation],
    patch_size: usize,
    intrinsics: &firefly_void_types::visual::Intrinsics,
) -> f64 {
    targets
        .iter()
        .map(|t| photometric_residuals(n_ref, p_ref, obs_ref, t, patch_size, intrinsics))
        .sum()
}

/// 仿射扭曲矩阵 `A_i^r`（论文 (13) 式）。
///
/// `A = P(I_i R_{Ir} + I_i t_{Ir} · 1/(nᵀp) · nᵀ) P⁻¹`，针孔 `P = K`。
#[must_use]
pub fn affine_warp(
    pose_ref: &Isometry3<f64>,
    pose_i: &Isometry3<f64>,
    normal_ref: &Vector3<f64>,
    p_ref: &Vector3<f64>,
    intrinsics: &firefly_void_types::visual::Intrinsics,
) -> Matrix2<f64> {
    let k = camera_matrix(intrinsics);
    let k_inv = k.try_inverse().unwrap_or_else(Matrix3::identity);

    // 相对位姿：参考系 → 目标系
    let t_rel = pose_i * pose_ref.inverse();
    let r_rel = t_rel.rotation.to_rotation_matrix().into_inner();
    let t_vec = t_rel.translation.vector;

    let denom = normal_ref.dot(p_ref);
    if denom.abs() < 1e-12 {
        return Matrix2::identity();
    }
    let inner = r_rel + t_vec * (normal_ref.transpose() / denom);
    let a = k * inner * k_inv;
    Matrix2::new(a[(0, 0)], a[(0, 1)], a[(1, 0)], a[(1, 1)])
}

/// 相机内参矩阵 `K`。
#[must_use]
pub fn camera_matrix(intrinsics: &firefly_void_types::visual::Intrinsics) -> Matrix3<f64> {
    Matrix3::new(
        intrinsics.fx,
        0.0,
        intrinsics.cx,
        0.0,
        intrinsics.fy,
        intrinsics.cy,
        0.0,
        0.0,
        1.0,
    )
}

/// 补丁内双线性采样（越界截断到边缘）。
fn bilinear_sample_patch(data: &[f64], patch_size: usize, x: f64, y: f64) -> f64 {
    let x0 = x.floor().clamp(0.0, (patch_size - 1) as f64) as usize;
    let y0 = y.floor().clamp(0.0, (patch_size - 1) as f64) as usize;
    let x1 = (x0 + 1).min(patch_size - 1);
    let y1 = (y0 + 1).min(patch_size - 1);
    let fx = x - x0 as f64;
    let fy = y - y0 as f64;
    let v00 = data[y0 * patch_size + x0];
    let v10 = data[y0 * patch_size + x1];
    let v01 = data[y1 * patch_size + x0];
    let v11 = data[y1 * patch_size + x1];
    (1.0 - fx) * (1.0 - fy) * v00 + fx * (1.0 - fy) * v10 + (1.0 - fx) * fy * v01 + fx * fy * v11
}

use crate::visual_point::PatchObservation;
