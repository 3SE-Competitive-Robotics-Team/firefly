//! 点-平面匹配公共逻辑（在线深度测量与先验测量共用）。
//!
//! [`match_plane`] 输入**候选平面集**（来自在线 `VoxelMap` 或静态先验
//! 容器），逐候选执行与 `DepthMeasurement::plane_for_point` 完全相同的
//! 判据链（对照官方 `voxel_map.cpp:713-786`）：
//! 1. 径向判据 `radius_k·radius`（:726-730）；
//! 2. 测量噪声 `σ² = J_nq·Σ_nq·J_nqᵀ + nᵀ·Σ_pj·n + 0.001`
//!    （:447-449，`J_nq = [p−q, −n]`）；
//! 3. 卡方门控 `|dis| ≤ sigma_num·√σ²`（:737）；
//! 4. 概率 `exp(−½dis²/σ²)/√σ²` 择优（对候选间相对优劣，与门控正交）。
//!
//! 先验语义对照 `map_location.cpp:1701-1766`：先验全局 kdtree 产出的
//! 平面候选与在线局部图候选走**同一残差路径**——本模块让两测量模型
//! 共享判据，仅候选来源不同。

use firefly_void_map::plane::VoxelPlane;
use nalgebra::{Matrix3, Vector3};

use crate::outlier::{GateVerdict, chi2_gate};

/// 单点平面匹配结果（探针：区分拒绝原因）。
#[derive(Debug, Clone, Copy)]
pub enum PlaneQuery {
    /// 匹配成功（通过径向判据 + 卡方门控）。
    Matched(Vector3<f64>, f64, f64),
    /// 无候选平面通过径向判据（无体素 / 无平面 / 径向判据不过）。
    NoPlane,
    /// 有候选但全部被卡方门控拒绝。
    Chi2Rejected,
}

/// 单点平面测量中间量 `(p_b, n, dis, σ²)`：机体系点、平面法向、
/// 残差距离与噪声方差。
pub type PlaneResidual = Option<(Vector3<f64>, Vector3<f64>, f64, f64)>;

/// 在候选平面集中匹配点 `p_world`（世界系）的最优平面。
///
/// `cov_w` 为点在世界系的协方差（`R·C·Rᵀ`），`radius_k`/`sigma_num` 为
/// 径向/卡方判据参数，`plane_var_scale` 为平面 `Σ_nq` 各向同性放大系数
/// （先验面噪声放大用，见 [`crate::options::PriorOptions`]；在线测量传
/// `1.0`）。返回匹配结果（含拒绝原因，探针）。
#[must_use]
pub fn match_plane(
    planes: &[&VoxelPlane],
    p_world: &Vector3<f64>,
    cov_w: &Matrix3<f64>,
    radius_k: f64,
    sigma_num: f64,
    plane_var_scale: f64,
) -> PlaneQuery {
    // 取概率最高（残差/噪声最小）的平面（对照官方对八叉子节点的
    // `this_prob` 择优）。
    let mut best: Option<(Vector3<f64>, f64, f64)> = None;
    let mut best_prob = f64::NEG_INFINITY;
    let mut any_candidate = false;
    for plane in planes {
        if !plane.is_plane {
            continue;
        }
        let dis = crate::plane_update::point_plane_residual(&plane.normal, p_world, &plane.center);
        // 径向判据（对照 voxel_map.cpp:726-730）
        let dis_to_center = (plane.center - p_world).norm_squared();
        let range_dis = (dis_to_center - dis * dis).max(0.0).sqrt();
        if range_dis > radius_k * plane.radius {
            continue;
        }
        any_candidate = true;
        // 噪声：J_nq·(Σ_nq·scale)·J_nqᵀ + nᵀ·Σ_pj·n（对照 voxel_map.cpp:447-449）
        let j_nq = nalgebra::RowVector6::new(
            p_world[0] - plane.center[0],
            p_world[1] - plane.center[1],
            p_world[2] - plane.center[2],
            -plane.normal[0],
            -plane.normal[1],
            -plane.normal[2],
        );
        let sigma_l = j_nq * (plane.plane_var * plane_var_scale) * j_nq.transpose();
        let sigma_l = sigma_l[(0, 0)] + plane.normal.dot(&(cov_w * plane.normal));
        let sigma_l = sigma_l + 0.001;
        // 卡方门控（对照 voxel_map.cpp:737）
        if chi2_gate(dis, sigma_l, sigma_num) == GateVerdict::Outlier {
            continue;
        }
        let prob = (-0.5 * dis * dis / sigma_l).exp() / sigma_l.sqrt();
        if prob > best_prob {
            best_prob = prob;
            best = Some((plane.normal, dis, sigma_l));
        }
    }
    match best {
        Some((n, dis, sig)) => PlaneQuery::Matched(n, dis, sig),
        None if any_candidate => PlaneQuery::Chi2Rejected,
        None => PlaneQuery::NoPlane,
    }
}
