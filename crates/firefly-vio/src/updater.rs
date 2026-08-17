//! MSCKF 特征更新（对照 `OpenVINS` `ov_msckf/update/UpdaterMSCKF.cpp/.h`）。
//!
//! [`UpdaterMsckf::update`] 流程（对照 `UpdaterMSCKF::update`）：
//! 1. 清理特征测量（只保留克隆时刻上的测量，不足 2 个删除）；
//! 2. 由 IMU 克隆 + 相机外参组装各相机的克隆位姿表；
//! 3. 三角化每个特征（失败删除）；
//! 4. 逐特征组装雅可比 → 零空间投影 → chi2 检验（拒绝外点）；
//! 5. 合并大矩阵 → 测量压缩 → [`crate::state_helper::ekf_update`]。
//!
//! SLAM 特征初始化的高斯牛顿精化（`refine_features`）未移植，标注 TODO。

use firefly_vio_core::feat::Feature;
use firefly_vio_core::triangulation::{
    CloneMap, ClonePose, TriangulationOptions, single_gaussnewton, single_triangulation,
};
use nalgebra::{DMatrix, DVector, Vector2};

use crate::options::UpdaterOptions;
use crate::state::State;
use crate::state_helper::{ekf_update, get_marginal_covariance};
use crate::updater_helper::{
    get_feature_jacobian_full, measurement_compress_inplace, nullspace_project_inplace,
};

/// chi2 95% 分位数（自由度 1..=30 的精确值，`>30` 用 Wilson–Hilferty 近似）。
const CHI2_95_TABLE: [f64; 30] = [
    3.8415, 5.9915, 7.8147, 9.4877, 11.0705, 12.5916, 14.0671, 15.5073, 16.9190, 18.3070, 19.6751,
    21.0261, 22.3620, 23.6848, 24.9958, 26.2962, 27.5871, 28.8693, 30.1435, 31.4104, 32.6706,
    33.9244, 35.1725, 36.4150, 37.6525, 38.8851, 40.1133, 41.3372, 42.5570, 43.7730,
];

/// chi2 95% 分位数（对照 `UpdaterMSCKF` 构造中的 `boost::math::chi_squared` 表）。
#[must_use]
pub fn chi2_95(dof: usize) -> f64 {
    if dof == 0 {
        return 0.0;
    }
    if dof <= CHI2_95_TABLE.len() {
        return CHI2_95_TABLE[dof - 1];
    }
    // Wilson–Hilferty 近似：χ²_α(ν) ≈ ν(1 − 2/(9ν) + z_α·√(2/(9ν)))³
    // z_0.95 = 1.6448536269514722
    let nu = dof as f64;
    let t = 1.0 - 2.0 / (9.0 * nu) + 1.644_853_626_951_472_2 * (2.0 / (9.0 * nu)).sqrt();
    nu * t * t * t
}

/// MSCKF 特征更新器（对照 `UpdaterMSCKF`）。
#[derive(Debug, Clone)]
pub struct UpdaterMsckf {
    /// 更新选项（像素噪声与 chi2 乘子）。
    pub options: UpdaterOptions,
    /// 三角化检查参数（对照 `FeatureInitializerOptions` 默认值）。
    pub triangulation_options: TriangulationOptions,
    /// MSCKF 特征表示（对照 `StateOptions::feat_rep_msckf`）。
    pub rep_msckf: crate::options::FeatRepresentation,
}

impl UpdaterMsckf {
    /// 构造（对照 `UpdaterMSCKF` 构造函数；`sigma_pix_sq` 由 sigma 刷新）。
    #[must_use]
    pub fn new(options: UpdaterOptions) -> Self {
        let mut options = options;
        options.sigma_pix_sq = options.sigma_pix * options.sigma_pix;
        Self {
            options,
            triangulation_options: TriangulationOptions::default(),
            rep_msckf: crate::options::FeatRepresentation::Global3D,
        }
    }

    /// 用特征向量更新状态（对照 `UpdaterMSCKF::update`）。
    ///
    /// 被 chi2 拒绝或三角化失败的特征标记 `to_delete`（由调用方在更新后
    /// 调用 `FeatureDatabase::cleanup` 清理）。
    // 与 C++ 1:1 移植的长流程函数，拆分会破坏对照可审计性。
    #[allow(clippy::too_many_lines)]
    pub fn update(&mut self, state: &mut State, feature_vec: &mut Vec<Feature>) {
        // (h_x, res, x_order, row0)
        type HxBlock = (DMatrix<f64>, DVector<f64>, Vec<(i32, usize)>, usize);
        if feature_vec.is_empty() {
            return;
        }
        // 可观测性：MSCKF 每帧输入特征数 + 克隆数（诊断视觉更新是否在执行）
        log::debug!(
            "MSCKF update: 输入特征 {}（克隆 {}）",
            feature_vec.len(),
            state.clones_imu.len()
        );

        // 1. 克隆时刻集合（对照 C++ 的 clonetimes）
        let clonetimes: Vec<f64> = state.clones_imu.iter().map(|(t, _)| *t).collect();

        // 2. 清理特征测量：只保留克隆时刻上的测量；不足 2 个删除
        feature_vec.retain_mut(|feat| {
            feat.clean_old_measurements(&clonetimes);
            let ct_meas: usize = feat.timestamps.values().map(Vec::len).sum();
            if ct_meas < 2 {
                feat.to_delete = true;
                false
            } else {
                true
            }
        });

        // 3. 组装各相机克隆位姿表（对照 C++ 的 clones_cam）
        let mut clones_cam: std::collections::HashMap<usize, CloneMap> =
            std::collections::HashMap::new();
        for (cam_id, calib) in &state.calib_imu_to_cam {
            let r_ito_c = calib.rot();
            let p_iin_c = calib.pos();
            let mut cam_clones: CloneMap = Vec::with_capacity(state.clones_imu.len());
            for (t, clone) in &state.clones_imu {
                // R_GtoCi = R_ItoC · R_GtoI；p_CioinG = p_IinG − R_GtoCiᵀ·p_IinC
                let r_gto_ci = r_ito_c * clone.rot();
                let p_ciin_g = clone.pos() - r_gto_ci.transpose() * p_iin_c;
                cam_clones.push((
                    *t,
                    ClonePose {
                        rot: r_gto_ci,
                        pos: p_ciin_g,
                    },
                ));
            }
            clones_cam.insert(*cam_id, cam_clones);
        }

        // 4. 三角化（失败删除）+ 高斯牛顿精化（对照 C++ 的
        // single_triangulation → single_gaussnewton 链；refine_features 默认 true）
        feature_vec.retain_mut(|feat| {
            let ok = single_triangulation(feat, &clones_cam, &self.triangulation_options);
            let ok = ok
                && (!self.triangulation_options.refine_features
                    || single_gaussnewton(feat, &clones_cam, &self.triangulation_options));
            if !ok {
                feat.to_delete = true;
            }
            ok
        });

        let n_triang = feature_vec.len();
        // 4b. 反投影一致性门控：p_FinA 在每视图的归一化反投影须与测量一致
        // （<0.3 归一化 ≈ 60px）。低视差下 DLT 深度病态会产出"贴近相机"的
        // 垃圾点，其他视图投影被视差放大 → 巨大残差 + 巨大增益修正（实测单次
        // 56m）。此门与协方差 P 无关，直接从源头拒掉不一致特征。
        let clones = &state.clones_imu;
        let n_before = feature_vec.len();
        feature_vec.retain_mut(|feat| {
            let mut worst = 0.0f64;
            for (cam_id, times) in &feat.timestamps {
                let Some(calib) = state.calib_imu_to_cam.get(cam_id) else {
                    continue;
                };
                let r_ito_c = calib.rot();
                let p_iin_c = calib.pos();
                let Some(uvs_norm) = feat.uvs_norm.get(cam_id) else {
                    continue;
                };
                for (m, t) in times.iter().enumerate() {
                    let Some((_, clone)) =
                        clones.iter().find(|(ct, _)| ct.total_cmp(t).is_eq())
                    else {
                        continue;
                    };
                    let p_in_im = clone.rot() * (feat.p_FinG - clone.pos());
                    let p_in_cw = r_ito_c * p_in_im + p_iin_c;
                    if p_in_cw.z <= 0.0 {
                        worst = f64::INFINITY;
                        break;
                    }
                    let uv_n =
                        Vector2::new(p_in_cw.x / p_in_cw.z, p_in_cw.y / p_in_cw.z);
                    let uv_m = uvs_norm[m];
                    let err =
                        (Vector2::new(f64::from(uv_m.x), f64::from(uv_m.y)) - uv_n).norm();
                    worst = worst.max(err);
                }
            }
            if worst > 0.3 || !worst.is_finite() {
                feat.to_delete = true;
                false
            } else {
                true
            }
        });
        log::debug!("reproj 门控: {}/{} 特征存活", feature_vec.len(), n_before);

        // 5. 逐特征：雅可比 → 零空间投影 → chi2 检验 → 收集块
        // （对照 C++ 的 Hx_mapping/Hx_big/res_big 组装）
        // 可观测性：首个存活特征的三角化结果与测量（诊断投影/深度病态）
        if let Some(f0) = feature_vec.first()
            && let Some((&c0, ts0)) = f0.timestamps.iter().next()
            && let Some(uv0) = f0.uvs_norm.get(&c0).and_then(|v| v.first())
        {
            log::debug!(
                "triang 首个特征 id={} p_FinA=({:.3},{:.3},{:.3}) cam{}t0={:.2} uv_n=({:.3},{:.3})",
                f0.featid,
                f0.p_FinA.x,
                f0.p_FinA.y,
                f0.p_FinA.z,
                c0,
                ts0.first().copied().unwrap_or(-1.0),
                uv0.x,
                uv0.y
            );
        }
        let mut hx_mapping: std::collections::HashMap<(i32, usize), usize> =
            std::collections::HashMap::new();
        let mut next_col = 0usize;
        let mut blocks: Vec<HxBlock> = Vec::new();
        let mut total_rows = 0usize;
        let mut hard_cap_rej = 0usize;

        for feat in feature_vec.iter_mut() {
            // 表示选择（对照 C++：ANCHORED_INVERSE_DEPTH_SINGLE → MSCKF 逆深度）
            let rep = match self.rep_msckf {
                crate::options::FeatRepresentation::AnchoredInverseDepthSingle => {
                    crate::options::FeatRepresentation::AnchoredMsckfInverseDepth
                }
                r => r,
            };
            let jac = get_feature_jacobian_full(state, feat, rep);
            let (mut h_f, mut h_x, mut res) = (jac.h_f, jac.h_x, jac.res);
            let x_order = jac.x_order;

            nullspace_project_inplace(&mut h_f, &mut h_x, &mut res);

            // 硬残差上限：协方差 P 膨胀时 chi2（H·P·Hᵀ+R）会退化性地接受
            // 巨大残差特征（实测 400+ px）而主动把估计推得更发散。无论 P 大小，
            // 平均像素残差 > `max_pix_res`（默认 40px）的特征一律拒收——这是
            // 与协方差无关的硬外点门，防止垃圾更新加剧漂移。
            if res.norm() > 40.0 * f64::sqrt(res.len() as f64) {
                feat.to_delete = true;
                hard_cap_rej += 1;
                continue;
            }

            // chi2 检验（对照 C++：S = H_x·P_marg·H_xᵀ + R，chi2 = resᵀS⁻¹res）。
            // 注意必须加测量噪声 R（sigma_pix²）：残差/雅可比在像素空间（fx 量级），
            // 缺 R 时 S 尺度错误 → 外点全部误纳/误拒。
            let p_marg = get_marginal_covariance(state, &x_order);
            let s = &h_x * p_marg * h_x.transpose()
                + self.options.sigma_pix_sq * DMatrix::identity(h_x.nrows(), h_x.nrows());
            let chi2 = match s.clone().cholesky() {
                Some(chol) => res.dot(&chol.solve(&res)),
                None => f64::INFINITY,
            };
            let chi2_check = chi2_95(res.len());
            if chi2 > self.options.chi2_multipler * chi2_check {
                feat.to_delete = true;
                continue;
            }

            // 注册列映射（对照 C++ 的 ct_hx 单调分配）
            for (var_id, var_size) in &x_order {
                hx_mapping.entry((*var_id, *var_size)).or_insert_with(|| {
                    let c = next_col;
                    next_col += var_size;
                    c
                });
            }

            let row0 = total_rows;
            total_rows += h_x.nrows();
            blocks.push((h_x, res, x_order, row0));
        }

        // 6. 统一组装大矩阵（行 = 各特征行和；列 = 映射覆盖范围）
        log::debug!("MSCKF 漏斗: 三角化存活 {n_triang} 硬残差拒 {hard_cap_rej} 组装行 {total_rows}");
        if total_rows == 0 {
            return;
        }
        let mut hx_big = DMatrix::zeros(total_rows, next_col);
        let mut res_big = DVector::zeros(total_rows);
        for (h_x, res, x_order, row0) in &blocks {
            let rows = h_x.nrows();
            let mut src_col = 0usize;
            for (var_id, var_size) in x_order {
                let dst_col = hx_mapping[&(*var_id, *var_size)];
                let block = h_x.view((0, src_col), (rows, *var_size)).into_owned();
                hx_big
                    .view_mut((*row0, dst_col), (rows, *var_size))
                    .copy_from(&block);
                src_col += var_size;
            }
            res_big.rows_range_mut(*row0..*row0 + rows).copy_from(res);
        }

        // 7. 测量压缩 + EKF 更新（对照 C++ 的末尾）
        measurement_compress_inplace(&mut hx_big, &mut res_big);
        if hx_big.nrows() == 0 {
            return;
        }
        // hx_order 按列偏移排序（对照 C++ 的 Hx_order 顺序）
        let mut hx_order: Vec<(i32, usize, usize)> = hx_mapping
            .iter()
            .map(|((id, size), col)| (*id, *size, *col))
            .collect();
        hx_order.sort_by_key(|(_, _, col)| *col);
        let hx_order: Vec<(i32, usize)> = hx_order
            .into_iter()
            .map(|(id, size, _)| (id, size))
            .collect();

        let r = self.options.sigma_pix_sq * DMatrix::identity(res_big.len(), res_big.len());
        // 可观测性：更新前残差量级与位置（诊断视觉矫正方向/量级）
        let pos_before = state.imu.pos();
        let res_norm = res_big.norm() / f64::sqrt(res_big.len() as f64);
        log::debug!(
            "MSCKF 应用更新: 行 {} 平均|res|={res_norm:.3}px 位置({:.2},{:.2},{:.2})",
            res_big.len(),
            pos_before.x,
            pos_before.y,
            pos_before.z
        );
        ekf_update(state, &hx_order, &hx_big, &res_big, &r);
        let pos_after = state.imu.pos();
        log::debug!(
            "MSCKF 更新后: 位置({:.2},{:.2},{:.2}) Δ=({:.3},{:.3},{:.3})",
            pos_after.x,
            pos_after.y,
            pos_after.z,
            pos_after.x - pos_before.x,
            pos_after.y - pos_before.y,
            pos_after.z - pos_before.z
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chi2_table_matches_known_values() {
        // 已知 χ² 95% 分位数：ν=1 → 3.8415，ν=2 → 5.9915，ν=10 → 18.3070
        assert!((chi2_95(1) - 3.8415).abs() < 1e-3);
        assert!((chi2_95(2) - 5.9915).abs() < 1e-3);
        assert!((chi2_95(10) - 18.3070).abs() < 1e-3);
    }

    #[test]
    fn chi2_approx_for_large_dof() {
        // ν=100 的 95% 分位数 ≈ 124.34（Wilson–Hilferty 精度 ~1e-2）
        let v = chi2_95(100);
        assert!((v - 124.34).abs() < 0.1, "chi2(100) = {v}");
        // 单调性
        assert!(chi2_95(100) > chi2_95(50));
    }

    #[test]
    fn chi2_monotonic() {
        for n in 1..60 {
            assert!(chi2_95(n + 1) > chi2_95(n), "dof={n}");
        }
    }
}
