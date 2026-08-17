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
use nalgebra::{DMatrix, DVector};

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

        // 5. 逐特征：雅可比 → 零空间投影 → chi2 检验 → 收集块
        // （对照 C++ 的 Hx_mapping/Hx_big/res_big 组装）
        let mut hx_mapping: std::collections::HashMap<(i32, usize), usize> =
            std::collections::HashMap::new();
        let mut next_col = 0usize;
        let mut blocks: Vec<HxBlock> = Vec::new();
        let mut total_rows = 0usize;

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

            // chi2 检验（对照 C++：S = H_x·P_marg·H_xᵀ，chi2 = resᵀS⁻¹res）
            let p_marg = get_marginal_covariance(state, &x_order);
            let s = &h_x * p_marg * h_x.transpose();
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
        ekf_update(state, &hx_order, &hx_big, &res_big, &r);
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
