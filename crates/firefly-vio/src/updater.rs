//! MSCKF 特征更新（对照 `OpenVINS` `ov_msckf/update/UpdaterMSCKF.cpp/.h`）。
//!
//! [`UpdaterMsckf::update`] 流程（对照 `UpdaterMSCKF::update`）：
//! 1. 清理特征测量（只保留克隆时刻上的测量，不足 2 个删除）；
//! 2. 由 IMU 克隆 + 相机外参组装各相机的克隆位姿表；
//! 3. 三角化每个特征（失败删除）；
//! 4. 逐特征组装雅可比 → 零空间投影 → chi2 检验（拒绝外点）；
//! 5. 合并大矩阵 → 测量压缩 → [`crate::state_helper::ekf_update`]。

use firefly_vio_core::feat::Feature;
use firefly_vio_core::triangulation::{
    TriangulationOptions, single_gaussnewton, single_triangulation,
};
use nalgebra::{DMatrix, DVector, Vector2};

use crate::options::UpdaterOptions;
use crate::state::State;
use crate::state_helper::{ekf_update, get_marginal_covariance};
use crate::updater_helper::{
    get_feature_jacobian_full, measurement_compress_inplace, nullspace_project_inplace,
};
use firefly_vio_types::var::PoseJpl;

pub use firefly_base::stats::chi2_95;

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
    pub fn new(options: UpdaterOptions, triangulation_options: TriangulationOptions) -> Self {
        let mut options = options;
        options.sigma_pix_sq = options.sigma_pix * options.sigma_pix;
        Self {
            options,
            triangulation_options,
            rep_msckf: crate::options::FeatRepresentation::Global3D,
        }
    }

    /// 用特征向量更新状态（对照 `UpdaterMSCKF::update`）。
    ///
    /// 被 chi2 拒绝或三角化失败的特征标记 `to_delete`（由调用方在更新后
    /// 调用 `FeatureDatabase::cleanup` 清理）。
    // 更新步骤与 `UpdaterMSCKF::update` 对应。
    #[allow(clippy::too_many_lines)]
    #[fastrace::trace]
    pub fn update(&mut self, state: &mut State, feature_vec: &mut Vec<Feature>) {
        // (h_x, res, x_order, row0, depth_m)
        type HxBlock = (DMatrix<f64>, DVector<f64>, Vec<(i32, usize)>, usize, f64);
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
        let clones_cam = crate::updater_slam::build_clones_cam(state);

        // 4. 三角化（失败删除）+ 高斯牛顿精化（对照 C++ 的
        // single_triangulation → single_gaussnewton 链；refine_features 默认 true）
        let n_in = feature_vec.len();
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
        // 4b. 反投影一致性门控：每个视图的归一化反投影须与测量相差小于 0.3。
        // 该门控直接验证几何一致性，不依赖协方差 `P`。
        // 预建时间→克隆查找表，避免每个测量线性扫描 clones（O(n²)→O(n)）
        let clone_map: std::collections::HashMap<u64, &PoseJpl> = state
            .clones_imu
            .iter()
            .map(|(t, c)| (t.to_bits(), c))
            .collect();
        let n_before = feature_vec.len();
        let mut rej_z = 0usize;
        let mut rej_err = 0usize;
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
                    let Some(&clone) = clone_map.get(&t.to_bits()) else {
                        continue;
                    };
                    let p_in_im = clone.rot() * (feat.p_FinG - clone.pos());
                    let p_in_cw = r_ito_c * p_in_im + p_iin_c;
                    if p_in_cw.z <= 0.0 {
                        worst = f64::INFINITY;
                        rej_z += 1;
                        break;
                    }
                    let uv_n = Vector2::new(p_in_cw.x / p_in_cw.z, p_in_cw.y / p_in_cw.z);
                    let uv_m = uvs_norm[m];
                    let err = (Vector2::new(f64::from(uv_m.x), f64::from(uv_m.y)) - uv_n).norm();
                    worst = worst.max(err);
                }
            }
            if worst > 0.3 || !worst.is_finite() {
                if worst.is_finite() && worst > 0.3 {
                    rej_err += 1;
                }
                feat.to_delete = true;
                false
            } else {
                true
            }
        });
        log::debug!(
            "reproj 门控: {}/{} 特征存活（拒因 z<=0:{rej_z} err>{rej_err}）",
            feature_vec.len(),
            n_before
        );

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
        let mut blocks: Vec<HxBlock> = Vec::with_capacity(feature_vec.len());
        let mut total_rows = 0usize;

        for feat in feature_vec.iter_mut() {
            // 表示选择（对照 C++：ANCHORED_INVERSE_DEPTH_SINGLE → MSCKF 逆深度）
            let rep = match self.rep_msckf {
                crate::options::FeatRepresentation::AnchoredInverseDepthSingle => {
                    crate::options::FeatRepresentation::AnchoredMsckfInverseDepth
                }
                r => r,
            };
            // 位置与 FEJ（对照 C++ 的「Save the position and its fej value」
            // 段，UpdaterMSCKF.cpp:186-193）：MSCKF 特征当帧边缘化、无首估计
            // 历史，fej = 当前三角化值
            if rep.is_relative() {
                feat.p_FinA_fej = feat.p_FinA;
            } else {
                feat.p_FinG_fej = feat.p_FinG;
            }
            let jac = get_feature_jacobian_full(state, feat, rep);
            let (mut h_f, mut h_x, mut res) = (jac.h_f, jac.h_x, jac.res);
            let x_order = jac.x_order;

            nullspace_project_inplace(&mut h_f, &mut h_x, &mut res);

            // chi2 检验（对照 C++：S = H_x·P_marg·H_xᵀ + R，
            // S.diagonal() += sigma_pix_sq，UpdaterMSCKF.cpp:211）。
            // 注意必须加测量噪声 R（sigma_pix²）：残差/雅可比在像素空间（fx 量级），
            // 缺 R 时 S 尺度错误 → 外点全部误纳/误拒。
            let p_marg = get_marginal_covariance(state, &x_order);
            let mut s = &h_x * p_marg * h_x.transpose();
            // 直接加对角线，跳过 DMatrix::identity 分配 + 矩阵加法（数学等价）
            for i in 0..s.nrows() {
                s[(i, i)] += self.options.sigma_pix_sq;
            }
            let chi2 = match s.cholesky() {
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
            blocks.push((h_x, res, x_order, row0, feat.p_FinA.z.max(0.1)));
        }

        // 6. 统一组装大矩阵（行 = 各特征行和；列 = 映射覆盖范围）
        log::debug!("MSCKF 漏斗: 候选 {n_in} 三角化存活 {n_triang} 组装行 {total_rows}");
        // 可观测性：进更新特征的深度分布（诊断近/远特征占比与姿态可观性）
        if !blocks.is_empty() {
            let depths: Vec<String> = blocks
                .iter()
                .map(|(_, _, _, _, d)| format!("{d:.1}"))
                .take(12)
                .collect();
            log::debug!(
                "MSCKF 更新特征深度: {} 个 [{}]",
                blocks.len(),
                depths.join(",")
            );
        }
        if total_rows == 0 {
            return;
        }
        let mut hx_big = DMatrix::zeros(total_rows, next_col);
        let mut res_big = DVector::zeros(total_rows);
        for (h_x, res, x_order, row0, _depth) in &blocks {
            let rows = h_x.nrows();
            let mut src_col = 0usize;
            for (var_id, var_size) in x_order {
                let dst_col = hx_mapping[&(*var_id, *var_size)];
                let block = h_x.view((0, src_col), (rows, *var_size));
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
        let dp = state.imu.pos() - pos_before;
        log::debug!(
            "MSCKF 更新后: 位置({:.2},{:.2},{:.2}) Δ=({:.3},{:.3},{:.3})",
            state.imu.pos().x,
            state.imu.pos().y,
            state.imu.pos().z,
            dp.x,
            dp.y,
            dp.z
        );
    }
}
