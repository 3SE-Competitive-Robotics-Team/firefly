//! SLAM 特征更新（对照 `OpenVINS` `ov_msckf/update/UpdaterSLAM.cpp/.h`）。
//!
//! - [`UpdaterSlam::delayed_init`]：新 SLAM 特征延迟初始化（三角化 → 高斯
//!   牛顿精化 → [`crate::state_helper::initialize_feature`] 增广进状态）；
//! - [`UpdaterSlam::update`]：已有 SLAM 特征的空闲更新（`H_f` 并入大雅可比，
//!   因为特征已在状态中 → chi2 → EKF 更新）；
//! - [`UpdaterSlam::change_anchors`]：锚点切换（锚定表示重锚 + 协方差传播）。
//!
//! ARUCO 分支（`feat_rep_aruco`）未移植：`max_aruco_features` 为 0 时所有
//! 特征走 SLAM 表示。单逆深度表示（`ANCHORED_INVERSE_DEPTH_SINGLE`）合并到
//! MSCKF 逆深度（对照 C++ 的映射）。

// 数学符号/新旧锚点块命名对照 C++ 源码（h_f_old/h_x_new 等），保留可审计性。
#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::too_many_lines
)]

use firefly_vio_core::feat::Feature;
use firefly_vio_core::triangulation::{
    CloneMap, ClonePose, TriangulationOptions, single_gaussnewton, single_triangulation,
};
use nalgebra::{DMatrix, DVector, Vector3};

use crate::landmark::Landmark;
use crate::options::{FeatRepresentation, UpdaterOptions};
use crate::state::State;
use crate::state_helper::{ekf_update, get_marginal_covariance, initialize_feature};
use crate::updater::chi2_95;
use crate::updater_helper::{get_feature_jacobian_full, nullspace_project_inplace};
use firefly_vio_types::var::Variable;

/// SLAM 特征更新器（对照 `UpdaterSLAM`）。
#[derive(Debug, Clone)]
pub struct UpdaterSlam {
    /// SLAM 特征更新选项（`_options_slam`）。
    pub options: UpdaterOptions,
    /// 三角化参数（对照 `initializer_feat->config()`）。
    pub triangulation_options: TriangulationOptions,
    /// SLAM 特征表示（对照 `StateOptions::feat_rep_slam`）。
    pub rep_slam: FeatRepresentation,
}

impl UpdaterSlam {
    /// 构造（对照 `UpdaterSLAM` 构造函数；`sigma_pix_sq` 由 sigma 刷新）。
    #[must_use]
    pub fn new(
        options: UpdaterOptions,
        rep_slam: FeatRepresentation,
        triangulation_options: TriangulationOptions,
    ) -> Self {
        let sigma_pix = options.sigma_pix;
        let mut options = options;
        options.sigma_pix_sq = sigma_pix * sigma_pix;
        Self {
            options,
            triangulation_options,
            rep_slam,
        }
    }

    /// SLAM 特征延迟初始化（对照 `UpdaterSLAM::delayed_init`）。
    ///
    /// 成功初始化的特征被加入 `state.features_slam` 并标记 `to_delete`；
    /// 失败的特征标记 `to_delete` 并从 `feature_vec` 移除。
    // 与 C++ 1:1 移植的长流程，拆分会破坏对照可审计性。
    #[allow(clippy::too_many_lines)]
    pub fn delayed_init(&mut self, state: &mut State, feature_vec: &mut Vec<Feature>) {
        if feature_vec.is_empty() {
            return;
        }

        // 0. 克隆时刻（对照 C++ 的 clonetimes）
        let clonetimes: Vec<f64> = state.clones_imu.iter().map(|(t, _)| *t).collect();

        // 1. 清理测量：只保留克隆时刻上的测量，不足 2 个删除（对照 C++）
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

        // 2. 组装各相机克隆位姿表（对照 C++ 的 clones_cam）
        let clones_cam = build_clones_cam(state);

        // 3. 三角化 + 高斯牛顿精化（失败删除；对照 C++ 的
        // single_triangulation → single_gaussnewton 链）
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

        // 4. 逐特征初始化（对照 C++ 的 step 4）
        feature_vec.retain_mut(|feat| {
            // 表示选择（对照 C++ 的 feat_rep：SINGLE 时用 MSCKF 逆深度表示
            // 算 3 列 H_f，再做深度分离 + bearing 零空间投影）
            let feat_rep = self.rep_slam;
            let jac_rep = if feat_rep == FeatRepresentation::AnchoredInverseDepthSingle {
                FeatRepresentation::AnchoredMsckfInverseDepth
            } else {
                feat_rep
            };

            // 雅可比 + 残差（对照 C++ 的 get_feature_jacobian_full）
            let jac = get_feature_jacobian_full(state, feat, jac_rep);
            let mut h_f = jac.h_f;
            let mut h_x = jac.h_x;
            let mut res = jac.res;
            let x_order = jac.x_order;
            let _ = &mut h_f;
            let _ = &mut h_x;

            // 单逆深度：深度列并入 H_x，bearing 零空间投影（对照 C++
            // delayed_init 的 SINGLE 段：H_xf=[H_x|H_f深度列]，
            // H_f 去掉深度列后 nullspace_project_inplace，再拆回）
            if feat_rep == FeatRepresentation::AnchoredInverseDepthSingle {
                let mut h_xf = DMatrix::zeros(h_x.nrows(), h_x.ncols() + 1);
                h_xf.view_mut((0, 0), (h_x.nrows(), h_x.ncols()))
                    .copy_from(&h_x);
                h_xf.column_mut(h_x.ncols())
                    .copy_from(&h_f.column(h_f.ncols() - 1));
                let mut h_f_bearing = h_f.columns(0, h_f.ncols() - 1).into_owned();
                nullspace_project_inplace(&mut h_f_bearing, &mut h_xf, &mut res);
                h_x = h_xf.columns(0, h_xf.ncols() - 1).into_owned();
                h_f = h_xf.columns(h_xf.ncols() - 1, 1).into_owned();
            }

            // 创建 landmark（对照 C++：先 size/UUID/anchor 再 set_from_xyz）
            let mut landmark = Landmark::new(feat_rep, feat.featid);
            landmark.unique_camera_id = feat.anchor_cam_id;
            if jac_rep.is_relative() {
                landmark.anchor_cam_id = feat.anchor_cam_id;
                landmark.anchor_clone_timestamp = feat.anchor_clone_timestamp;
                landmark.set_from_xyz(&feat.p_FinA, false);
                landmark.set_from_xyz(&feat.p_FinA, true);
            } else {
                landmark.set_from_xyz(&feat.p_FinG, false);
                landmark.set_from_xyz(&feat.p_FinG, true);
            }

            // 测量噪声矩阵（对照 C++：sigma_pix_sq * I）
            let r_noise = self.options.sigma_pix_sq * DMatrix::identity(res.len(), res.len());

            // 尝试初始化（对照 C++ 的 StateHelper::initialize）
            let ok = initialize_feature(
                state,
                landmark,
                &x_order,
                &mut h_x,
                &mut h_f,
                &r_noise,
                &mut res,
                self.options.chi2_multipler,
            );
            if ok {
                // 初始化成功：特征保留在向量中（标记删除，由调用方 cleanup）
                feat.to_delete = true;
                true
            } else {
                // 初始化失败：删除
                feat.to_delete = true;
                false
            }
        });
    }

    /// SLAM 特征更新（对照 `UpdaterSLAM::update`）。
    ///
    /// 特征已在状态中，故把 `H_f` 追加到 `H_x` 组成 `H_xf`，并将 landmark
    /// 追加到 `Hx_order`。返回后 `feature_vec` 中剩余特征全部标记 `to_delete`
    /// （对照 C++ 末尾循环）。
    // 与 C++ 1:1 移植的长流程，拆分会破坏对照可审计性。
    #[allow(clippy::too_many_lines)]
    // 与 C++ 的 `.at()` 断言语义一致：特征无对应 landmark 时崩溃。
    #[allow(clippy::missing_panics_doc)]
    pub fn update(&mut self, state: &mut State, feature_vec: &mut Vec<Feature>) {
        // (h_x_and_f, res, x_order, row0)
        type Block = (DMatrix<f64>, DVector<f64>, Vec<(i32, usize)>, usize);
        if feature_vec.is_empty() {
            return;
        }

        // 0. 克隆时刻（对照 C++ 的 clonetimes）
        let clonetimes: Vec<f64> = state.clones_imu.iter().map(|(t, _)| *t).collect();

        // 1. 清理测量（对照 C++：SINGLE 表示需 ≥2）
        feature_vec.retain_mut(|feat| {
            feat.clean_old_measurements(&clonetimes);
            let ct_meas: usize = feat.timestamps.values().map(Vec::len).sum();
            let landmark = state.features_slam.get(&feat.featid);
            let required = if landmark
                .is_some_and(|l| l.representation == FeatRepresentation::AnchoredInverseDepthSingle)
            {
                2
            } else {
                1
            };
            if ct_meas < 1 {
                feat.to_delete = true;
                false
            } else if ct_meas < required {
                // 不足但不删除（对照 C++ 的 else if：erase 不标记）
                false
            } else {
                true
            }
        });

        // 2. 逐特征组装（对照 C++ 的 Hx_mapping/Hx_big/res_big 段）
        let mut hx_mapping: std::collections::HashMap<(i32, usize), usize> =
            std::collections::HashMap::new();
        let mut hx_order_big: Vec<(i32, usize)> = Vec::new();
        let mut next_col = 0usize;
        let mut blocks: Vec<Block> = Vec::new();
        let mut total_rows = 0usize;

        feature_vec.retain_mut(|feat| {
            // 断言 landmark 存在（对照 C++ 的两个 assert）
            let Some(landmark) = state.features_slam.get(&feat.featid).cloned() else {
                panic!(
                    "UpdaterSLAM::update: 特征 {} 没有对应 landmark",
                    feat.featid
                );
            };

            // 表示选择（对照 C++ 的 landmark->_feat_representation）
            let jac_rep =
                if landmark.representation == FeatRepresentation::AnchoredInverseDepthSingle {
                    FeatRepresentation::AnchoredMsckfInverseDepth
                } else {
                    landmark.representation
                };

            // 从 landmark 取位置（含 FEJ）
            if jac_rep.is_relative() {
                feat.anchor_cam_id = landmark.anchor_cam_id;
                feat.anchor_clone_timestamp = landmark.anchor_clone_timestamp;
                feat.p_FinA = landmark.get_xyz(false);
                feat.p_FinG = landmark.get_xyz(true);
            } else {
                feat.p_FinG = landmark.get_xyz(false);
            }

            let jac = get_feature_jacobian_full(state, feat, jac_rep);
            let h_f = jac.h_f;
            let h_x = jac.h_x;
            let mut res = jac.res;
            let mut x_order = jac.x_order;

            // 把 H_f 并到 H_x 上，landmark 追加进 x_order（对照 C++ 的 H_xf）
            // SINGLE 表示：深度列并入（landmark 是 1 维状态）；bearing 两列
            // 零空间投影消除（对照 C++ update 的 SINGLE 段）
            let h_combined =
                if landmark.representation == FeatRepresentation::AnchoredInverseDepthSingle {
                    let mut h_xf = DMatrix::zeros(h_x.nrows(), h_x.ncols() + 1);
                    h_xf.view_mut((0, 0), (h_x.nrows(), h_x.ncols()))
                        .copy_from(&h_x);
                    h_xf.column_mut(h_x.ncols())
                        .copy_from(&h_f.column(h_f.ncols() - 1));
                    let mut h_f_bearing = h_f.columns(0, h_f.ncols() - 1).into_owned();
                    nullspace_project_inplace(&mut h_f_bearing, &mut h_xf, &mut res);
                    h_xf
                } else {
                    let mut h_xf = DMatrix::zeros(h_x.nrows(), h_x.ncols() + h_f.ncols());
                    h_xf.view_mut((0, 0), (h_x.nrows(), h_x.ncols()))
                        .copy_from(&h_x);
                    h_xf.view_mut((0, h_x.ncols()), (h_f.nrows(), h_f.ncols()))
                        .copy_from(&h_f);
                    h_xf
                };
            x_order.push((landmark.id(), landmark.size()));

            // chi2 检验（对照 C++：S.diagonal() += sigma_pix_sq）
            let p_marg = get_marginal_covariance(state, &x_order);
            let mut s = &h_combined * p_marg * h_combined.transpose();
            let n = s.nrows();
            for i in 0..n {
                s[(i, i)] += self.options.sigma_pix_sq;
            }
            let chi2 = match s.clone().cholesky() {
                Some(chol) => res.dot(&chol.solve(&res)),
                None => f64::INFINITY,
            };
            let chi2_check = chi2_95(res.len());
            if chi2 > self.options.chi2_multipler * chi2_check {
                // 拒绝（对照 C++：非 aruco → 失败计数 + 删除）
                if let Some(l) = state.features_slam.get_mut(&feat.featid) {
                    l.update_fail_count += 1;
                    if l.update_fail_count > 2 {
                        l.should_marg = true;
                    }
                }
                feat.to_delete = true;
                false
            } else {
                // 注册列映射 + 收集块
                for (var_id, var_size) in &x_order {
                    hx_mapping.entry((*var_id, *var_size)).or_insert_with(|| {
                        let c = next_col;
                        next_col += var_size;
                        hx_order_big.push((*var_id, *var_size));
                        c
                    });
                }
                let row0 = total_rows;
                total_rows += h_combined.nrows();
                blocks.push((h_combined, res, x_order, row0));
                true
            }
        });

        // 3. 统一组装大矩阵（对照 C++ 的 block 拷贝）
        if total_rows == 0 {
            return;
        }
        let mut hx_big = DMatrix::zeros(total_rows, next_col);
        let mut res_big = DVector::zeros(total_rows);
        for (h_xf_block, res, x_order, row0) in &blocks {
            let rows = h_xf_block.nrows();
            let mut src_col = 0usize;
            for (var_id, var_size) in x_order {
                let dst_col = hx_mapping[&(*var_id, *var_size)];
                let block = h_xf_block
                    .view((0, src_col), (rows, *var_size))
                    .into_owned();
                hx_big
                    .view_mut((*row0, dst_col), (rows, *var_size))
                    .copy_from(&block);
                src_col += var_size;
            }
            res_big.rows_range_mut(*row0..*row0 + rows).copy_from(res);
        }

        // hx_order_big 按列偏移排序（对照 C++ 的 Hx_order_big 顺序）
        hx_order_big.sort_by_key(|(id, size)| hx_mapping[&(*id, *size)]);

        // 4. EKF 更新（对照 C++ 末尾）
        let r = self.options.sigma_pix_sq * DMatrix::identity(total_rows, total_rows);
        ekf_update(state, &hx_order_big, &hx_big, &res_big, &r, f64::INFINITY);
    }

    /// 锚点切换（对照 `UpdaterSLAM::change_anchors`）。
    ///
    /// 当滑动窗口将边缘化最老克隆时，把锚在该克隆上的锚定 SLAM 特征重锚到
    /// 最新时刻（同一相机），并做协方差传播（对照
    /// `perform_anchor_change`）。`GLOBAL_3D`/`GLOBAL_FULL_INVERSE_DEPTH`
    /// 无锚，跳过。
    ///
    /// # Panics
    /// 锚点克隆时刻早于边缘化时刻（对照 C++ 的 assert）。
    pub fn change_anchors(&mut self, state: &mut State) {
        // 克隆数不足时不触发（对照 C++）
        if state.clones_imu.len() <= state.options.max_clone_size {
            return;
        }
        let marg_timestep = state.marg_timestep();
        let landmark_ids: Vec<usize> = state.features_slam.keys().copied().collect();
        for featid in landmark_ids {
            let Some(lm) = state.features_slam.get(&featid) else {
                continue;
            };
            // 跳过全局系表示（对照 C++ 的 continue）
            if lm.representation == FeatRepresentation::Global3D
                || lm.representation == FeatRepresentation::GlobalFullInverseDepth
            {
                continue;
            }
            // 锚在将边缘化的克隆上 → 重锚（对照 C++ 的 assert + 条件）
            assert!(
                marg_timestep <= lm.anchor_clone_timestamp,
                "锚点克隆时刻应不早于边缘化时刻"
            );
            if (lm.anchor_clone_timestamp - marg_timestep).abs() < 1e-9 {
                let new_anchor_timestamp = state.timestamp;
                let new_cam_id = lm.anchor_cam_id as usize;
                // 借用问题：先克隆重锚所需数据，再执行
                let lm2 = state.features_slam.get(&featid).cloned();
                if let Some(mut lm2) = lm2 {
                    perform_anchor_change(state, &mut lm2, new_anchor_timestamp, new_cam_id);
                    state.features_slam.insert(featid, lm2);
                }
            }
        }
    }
}

/// 执行锚点切换（对照 `UpdaterSLAM::perform_anchor_change`）。
///
/// 用旧锚点状态与新锚点状态的表示雅可比构造变换 `Phi`，做
/// `EKFPropagation(phi_order_NEW, phi_order_OLD, Phi, Q=0)` 传播协方差，
/// 然后重设 landmark 的锚点/值/FEJ。
#[allow(clippy::too_many_lines)]
fn perform_anchor_change(
    state: &mut State,
    landmark: &mut Landmark,
    new_anchor_timestamp: f64,
    new_cam_id: usize,
) {
    // 断言锚定表示（对照 C++）
    assert!(
        landmark.representation.is_relative() && landmark.anchor_cam_id != -1,
        "perform_anchor_change 仅用于锚定表示"
    );

    // 旧特征表示（对照 C++ old_feat）
    let old_rep = landmark.representation;
    let old_anchor_cam = landmark.anchor_cam_id as usize;
    let old_anchor_t = landmark.anchor_clone_timestamp;
    let old_feat = firefly_vio_core::feat::Feature {
        featid: landmark.featid,
        to_delete: false,
        timestamps: std::collections::HashMap::new(),
        uvs: std::collections::HashMap::new(),
        uvs_norm: std::collections::HashMap::new(),
        anchor_cam_id: landmark.anchor_cam_id,
        anchor_clone_timestamp: landmark.anchor_clone_timestamp,
        p_FinA: landmark.get_xyz(false),
        p_FinG: Vector3::zeros(),
    };

    // 旧表示雅可比（对照 C++ get_feature_jacobian_representation(old)）
    let (h_f_old, h_x_old) =
        crate::updater_helper::get_feature_jacobian_representation(state, &old_feat, old_rep);
    let h_x_old: Vec<DMatrix<f64>> = h_x_old.into_iter().map(|(_, _, m)| m).collect();

    // 新锚点（对照 C++ 的 R_GtoNEW/p_NEWinG 段）
    let (r_gto_i_new, p_iin_g_new) = {
        let clone = state
            .clones_imu
            .iter()
            .find(|(t, _)| t.total_cmp(&new_anchor_timestamp).is_eq())
            .expect("新锚点克隆必须存在");
        (clone.1.rot(), clone.1.pos())
    };
    let calib_new = state
        .calib_imu_to_cam
        .get(&new_cam_id)
        .expect("新锚点相机必须有外参");
    let r_ito_c_new = calib_new.rot();
    let p_iin_c_new = calib_new.pos();
    let r_gto_new = r_ito_c_new * r_gto_i_new;
    let p_newin_g = p_iin_g_new - r_gto_new.transpose() * p_iin_c_new;

    // 旧锚点（对照 C++ 的 R_GtoOLD/p_OLDinG 段）
    let (r_gto_i_old, p_iin_g_old) = {
        let clone = state
            .clones_imu
            .iter()
            .find(|(t, _)| t.total_cmp(&old_anchor_t).is_eq())
            .expect("旧锚点克隆必须存在");
        (clone.1.rot(), clone.1.pos())
    };
    let calib_old = state
        .calib_imu_to_cam
        .get(&old_anchor_cam)
        .expect("旧锚点相机必须有外参");
    let r_ito_c_old = calib_old.rot();
    let p_iin_c_old = calib_old.pos();
    let r_gto_old = r_ito_c_old * r_gto_i_old;
    let p_oldin_g = p_iin_g_old - r_gto_old.transpose() * p_iin_c_old;

    // 新锚点系下的位置（对照 C++ p_OLDinNEW/new_feat.p_FinA）
    let r_old_to_new = r_gto_new * r_gto_old.transpose();
    let p_oldin_new = r_gto_new * (p_oldin_g - p_newin_g);
    let new_p_fin_a = r_old_to_new * landmark.get_xyz(false) + p_oldin_new;

    // FEJ 版本（对照 C++ 的 *_fej 段）
    let (r_gto_i_new_fej, p_iin_g_new_fej) = {
        let clone = state
            .clones_imu
            .iter()
            .find(|(t, _)| t.total_cmp(&new_anchor_timestamp).is_eq())
            .expect("新锚点克隆必须存在");
        (clone.1.rot_fej(), clone.1.pos_fej())
    };
    let r_gto_new_fej = r_ito_c_new * r_gto_i_new_fej;
    let p_newin_g_fej = p_iin_g_new_fej - r_gto_new_fej.transpose() * p_iin_c_new;
    let (r_gto_i_old_fej, p_iin_g_old_fej) = {
        let clone = state
            .clones_imu
            .iter()
            .find(|(t, _)| t.total_cmp(&old_anchor_t).is_eq())
            .expect("旧锚点克隆必须存在");
        (clone.1.rot_fej(), clone.1.pos_fej())
    };
    let r_gto_old_fej = r_ito_c_old * r_gto_i_old_fej;
    let p_oldin_g_fej = p_iin_g_old_fej - r_gto_old_fej.transpose() * p_iin_c_old;
    let r_old_to_new_fej = r_gto_new_fej * r_gto_old_fej.transpose();
    let p_oldin_new_fej = r_gto_new_fej * (p_oldin_g_fej - p_newin_g_fej);
    let new_p_fin_a_fej = r_old_to_new_fej * landmark.get_xyz(true) + p_oldin_new_fej;

    // 新表示雅可比（对照 C++ get_feature_jacobian_representation(new)）
    let new_feat = firefly_vio_core::feat::Feature {
        featid: landmark.featid,
        to_delete: false,
        timestamps: std::collections::HashMap::new(),
        uvs: std::collections::HashMap::new(),
        uvs_norm: std::collections::HashMap::new(),
        anchor_cam_id: new_cam_id as i32,
        anchor_clone_timestamp: new_anchor_timestamp,
        p_FinA: new_p_fin_a,
        p_FinG: Vector3::zeros(),
    };
    let (h_f_new, h_x_new) =
        crate::updater_helper::get_feature_jacobian_representation(state, &new_feat, old_rep);
    let h_x_new: Vec<DMatrix<f64>> = h_x_new.into_iter().map(|(_, _, m)| m).collect();

    // Phi 顺序（对照 C++ phi_order_NEW/phi_order_OLD/Phi_id_map）
    // NEW = [landmark]；OLD = 旧锚点状态 + 新锚点状态 + landmark
    let phi_order_new = vec![(landmark.id(), landmark.size())];
    let mut phi_order_old: Vec<(i32, usize)> = Vec::new();
    let mut phi_id_map: std::collections::HashMap<(i32, usize), usize> =
        std::collections::HashMap::new();
    let mut current_it = 0usize;
    // 旧锚点状态块（对照 C++ x_order_old 循环）
    let old_state_blocks: Vec<(i32, usize)> = h_x_old_blocks(state, &old_feat);
    let new_state_blocks: Vec<(i32, usize)> = h_x_new_blocks(state, &new_feat);
    for var in old_state_blocks.iter().chain(new_state_blocks.iter()) {
        if !phi_id_map.contains_key(var) {
            phi_id_map.insert(*var, current_it);
            phi_order_old.push(*var);
            current_it += var.1;
        }
    }
    phi_id_map.insert((landmark.id(), landmark.size()), current_it);
    phi_order_old.push((landmark.id(), landmark.size()));
    current_it += landmark.size();

    // Phi 矩阵（对照 C++：pf_new_error = Hfnew⁻¹·(Hfold·pf_olderror +
    // Hxold·x_olderror − Hxnew·x_newerror)）
    let phisize = if old_rep == FeatRepresentation::AnchoredInverseDepthSingle {
        1
    } else {
        3
    };
    let mut phi_mat = DMatrix::<f64>::zeros(phisize, current_it);
    let h_f_new_inv = if phisize == 1 {
        // 1 维：H_f_new 是 3×1，用伪逆（对照 C++ 的 squaredNorm 版）
        let n = h_f_new.norm_squared();
        let mut inv = DMatrix::<f64>::zeros(1, 3);
        if n > 1e-12 {
            inv.row_mut(0).copy_from(&(h_f_new.transpose() / n).row(0));
        }
        inv
    } else {
        h_f_new.clone().try_inverse().expect("新表示 H_f 应可逆")
    };

    // 旧锚点状态块（对照 C++ H_x_old[i]）
    for (i, var) in old_state_blocks.iter().enumerate() {
        let col = phi_id_map[var];
        let blk = &h_f_new_inv * &h_x_old[i];
        // 拷贝到 phi_mat
        for r in 0..phisize {
            for c in 0..var.1 {
                phi_mat[(r, col + c)] += blk[(r, c)];
            }
        }
    }
    // 旧特征（对照 C++ Phi.block(...) = Hfnew_inv·Hfold）
    {
        let col = phi_id_map[&(landmark.id(), landmark.size())];
        let blk = &h_f_new_inv * &h_f_old;
        for r in 0..phisize {
            for c in 0..landmark.size() {
                phi_mat[(r, col + c)] += blk[(r, c)];
            }
        }
    }
    // 新锚点状态块（对照 C++ −Hfnew_inv·Hx_new[i]）
    for (i, var) in new_state_blocks.iter().enumerate() {
        let col = phi_id_map[var];
        let blk = &h_f_new_inv * &h_x_new[i];
        for r in 0..phisize {
            for c in 0..var.1 {
                phi_mat[(r, col + c)] -= blk[(r, c)];
            }
        }
    }

    // 协方差传播（对照 C++ EKFPropagation(state, phi_order_NEW, phi_order_OLD, Phi, Q=0)）
    let q_zero = DMatrix::<f64>::zeros(phisize, phisize);
    crate::state_helper::ekf_propagation(state, &phi_order_new, &phi_order_old, &phi_mat, &q_zero);

    // 设置新特征（对照 C++ 末尾）
    landmark.anchor_cam_id = new_cam_id as i32;
    landmark.anchor_clone_timestamp = new_anchor_timestamp;
    landmark.set_from_xyz(&new_p_fin_a, false);
    landmark.set_from_xyz(&new_p_fin_a_fej, true);
    landmark.has_had_anchor_change = true;
}

/// 旧/新锚点状态块列表（`get_feature_jacobian_representation` 的 `x_order`）。
fn h_x_old_blocks(state: &State, feat: &firefly_vio_core::feat::Feature) -> Vec<(i32, usize)> {
    let mut out = Vec::new();
    let anchor = state
        .clones_imu
        .iter()
        .find(|(t, _)| t.total_cmp(&feat.anchor_clone_timestamp).is_eq())
        .map(|(_, c)| c)
        .expect("锚点克隆必须存在");
    out.push((anchor.id(), 6));
    if state.options.do_calib_camera_pose {
        let calib = state
            .calib_imu_to_cam
            .get(&(feat.anchor_cam_id as usize))
            .expect("锚点相机必须有外参");
        out.push((calib.id(), 6));
    }
    out
}

fn h_x_new_blocks(state: &State, feat: &firefly_vio_core::feat::Feature) -> Vec<(i32, usize)> {
    h_x_old_blocks(state, feat)
}

/// 组装各相机克隆位姿表（对照 `UpdaterSLAM`/`UpdaterMSCKF` 的 `clones_cam`）。
pub(crate) fn build_clones_cam(state: &State) -> std::collections::HashMap<usize, CloneMap> {
    let mut clones_cam: std::collections::HashMap<usize, CloneMap> =
        std::collections::HashMap::new();
    for (cam_id, calib) in &state.calib_imu_to_cam {
        let r_ito_c = calib.rot();
        let p_iin_c = calib.pos();
        let mut cam_clones: CloneMap = Vec::with_capacity(state.clones_imu.len());
        for (t, clone) in &state.clones_imu {
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
    clones_cam
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::{StateOptions, VioManagerOptions};
    use firefly_vio_core::cam::CamRadtan;
    use firefly_vio_core::sensor::ImuData;
    use nalgebra::{Vector2, Vector3};
    use std::collections::HashMap;
    use std::sync::Arc;

    /// 构造带两个移动克隆 + 相机模型的状态，特征 (0.5,0.2,3.0) 在两时刻被观测。
    ///
    /// - t=1：IMU 在原点（相机也近似原点，忽略外参）
    /// - t=2：IMU 平移 (0.5,0,0) → 特征在相机 2 下的归一化坐标变化，形成视差
    fn slam_scene() -> (State, Feature) {
        let opts = StateOptions::default();
        let mut st = State::new(opts);
        let cam = CamRadtan::new(640, 480, &[600.0, 600.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0]);
        st.cameras.insert(0, Arc::new(cam));

        // 两个克隆：t=1 原点静止；t=2 沿 +x 平移 0.5m
        // 注意：FEJ（默认开启）下克隆线性化点取首估计，故每次 set_value 后
        // 必须同步 set_fej（对照 VioManager::initialize_with_gt 的语义），
        // 否则两个克隆的 FEJ 位姿相同 → H_f 秩亏 → 初始化失败。
        st.timestamp = 1.0;
        st.imu.set_value(
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        st.imu.set_fej(
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        crate::state_helper::augment_clone(&mut st, &Vector3::zeros());
        st.timestamp = 2.0;
        st.imu.set_value(
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::new(0.5, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        st.imu.set_fej(
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::new(0.5, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        crate::state_helper::augment_clone(&mut st, &Vector3::zeros());

        // 特征全局位置（= 锚点系，因克隆 1 为原点单位位姿）
        let p_g = Vector3::new(0.5, 0.2, 3.0);
        // 在克隆 1（原点）与克隆 2（平移 0.5,0,0）下的归一化投影
        let uv1 = Vector2::new(p_g.x / p_g.z, p_g.y / p_g.z);
        let p2 = p_g - Vector3::new(0.5, 0.0, 0.0);
        let uv2 = Vector2::new(p2.x / p2.z, p2.y / p2.z);
        // 像素坐标 = 内参投影（fx=600, cx=320, cy=240，无畸变）
        let to_pix = |uv: Vector2<f64>| Vector2::new(600.0 * uv.x + 320.0, 600.0 * uv.y + 240.0);

        let feat = Feature {
            featid: 1,
            to_delete: false,
            timestamps: HashMap::from([(0usize, vec![1.0f64, 2.0])]),
            uvs: HashMap::from([(0usize, vec![to_pix(uv1).cast(), to_pix(uv2).cast()])]),
            uvs_norm: HashMap::from([(0usize, vec![uv1.cast(), uv2.cast()])]),
            anchor_cam_id: -1,
            anchor_clone_timestamp: 0.0,
            p_FinA: Vector3::zeros(),
            p_FinG: Vector3::zeros(),
        };
        (st, feat)
    }

    #[test]
    fn delayed_init_adds_landmark_and_grows_covariance() {
        let (mut st, feat) = slam_scene();
        let cov_before = st.cov.nrows();
        assert_eq!(cov_before, 27); // 15 + 6 + 6
        let mut feats = vec![feat];
        let mut updater = UpdaterSlam::new(
            UpdaterOptions::default(),
            FeatRepresentation::Global3D,
            TriangulationOptions::default(),
        );
        updater.delayed_init(&mut st, &mut feats);
        // 成功 → 特征保留（标记删除），landmark 入库
        assert_eq!(feats.len(), 1);
        assert!(feats[0].to_delete);
        assert!(st.features_slam.contains_key(&1));
        let lm = &st.features_slam[&1];
        assert_eq!(lm.representation, FeatRepresentation::Global3D);
        assert_eq!(lm.id(), cov_before as i32);
        assert_eq!(lm.size(), 3);
        // 协方差增广 3 维
        assert_eq!(st.cov.nrows(), cov_before + 3);
        // 特征位置应接近真值 (0.5, 0.2, 3.0)
        let p = lm.get_xyz(false);
        assert!((p - Vector3::new(0.5, 0.2, 3.0)).norm() < 0.05, "p = {p}");
    }

    #[test]
    fn delayed_init_single_inverse_depth() {
        // SINGLE 表示：landmark 1 维（只估深度），方位锁定在首次观测
        let (mut st, feat) = slam_scene();
        let cov_before = st.cov.nrows();
        let mut feats = vec![feat];
        let mut updater = UpdaterSlam::new(
            UpdaterOptions::default(),
            FeatRepresentation::AnchoredInverseDepthSingle,
            TriangulationOptions::default(),
        );
        updater.delayed_init(&mut st, &mut feats);
        assert_eq!(feats.len(), 1, "SINGLE 特征应初始化成功");
        assert!(st.features_slam.contains_key(&1));
        let lm = &st.features_slam[&1];
        assert_eq!(lm.size(), 1, "SINGLE landmark 应为 1 维");
        assert_eq!(lm.id(), cov_before as i32);
        assert_eq!(st.cov.nrows(), cov_before + 1, "协方差只增广 1 维");
        // 深度应接近真值 3.0
        let p = lm.get_xyz(false);
        assert!((p.z - 3.0).abs() < 0.1, "p = {p}");
        // 方位已锁定
        assert!(lm.uv_norm_zero.norm() > 0.5);
    }

    #[test]
    fn slam_update_single_inverse_depth_keeps_landmark() {
        let (mut st, feat) = slam_scene();
        let mut updater = UpdaterSlam::new(
            UpdaterOptions::default(),
            FeatRepresentation::AnchoredInverseDepthSingle,
            TriangulationOptions::default(),
        );
        let mut feats = vec![feat];
        updater.delayed_init(&mut st, &mut feats);
        assert!(st.features_slam.contains_key(&1));
        let cov_init = st.cov.nrows();
        // 更新：t=2 测量
        let p_init = st.features_slam[&1].get_xyz(false);
        let p2 = p_init - Vector3::new(0.5, 0.0, 0.0);
        let uv2 = Vector2::new(p2.x / p2.z, p2.y / p2.z);
        let to_pix = |uv: Vector2<f64>| Vector2::new(600.0 * uv.x + 320.0, 600.0 * uv.y + 240.0);
        let upd_feat = Feature {
            featid: 1,
            to_delete: false,
            timestamps: HashMap::from([(0usize, vec![2.0f64])]),
            uvs: HashMap::from([(0usize, vec![to_pix(uv2).cast()])]),
            uvs_norm: HashMap::from([(0usize, vec![uv2.cast()])]),
            anchor_cam_id: -1,
            anchor_clone_timestamp: 0.0,
            p_FinA: Vector3::zeros(),
            p_FinG: Vector3::zeros(),
        };
        let mut update_vec = vec![upd_feat];
        updater.update(&mut st, &mut update_vec);
        assert!(st.features_slam.contains_key(&1));
        assert_eq!(st.cov.nrows(), cov_init);
    }

    #[test]
    fn delayed_init_rejects_degenerate() {
        let (mut st, mut feat) = slam_scene();
        // 破坏测量：两个时刻同位置（零视差）→ 三角化失败 → 删除
        feat.timestamps = HashMap::from([(0usize, vec![1.0f64, 2.0])]);
        feat.uvs = HashMap::from([(0usize, vec![Vector2::new(0.1f32, 0.04f32); 2])]);
        feat.uvs_norm = HashMap::from([(0usize, vec![Vector2::new(0.1, 0.04); 2])]);
        let mut feats = vec![feat];
        let mut updater = UpdaterSlam::new(
            UpdaterOptions::default(),
            FeatRepresentation::Global3D,
            TriangulationOptions::default(),
        );
        updater.delayed_init(&mut st, &mut feats);
        assert!(feats.is_empty());
        assert!(st.features_slam.is_empty());
        assert_eq!(st.cov.nrows(), 27);
    }

    #[test]
    fn slam_update_keeps_landmark_consistent() {
        let (mut st, feat) = slam_scene();
        let mut updater = UpdaterSlam::new(
            UpdaterOptions::default(),
            FeatRepresentation::Global3D,
            TriangulationOptions::default(),
        );
        // 先延迟初始化
        let mut feats = vec![feat];
        updater.delayed_init(&mut st, &mut feats);
        assert!(st.features_slam.contains_key(&1));
        let cov_init = st.cov.nrows();

        // 更新：用 t=2 的测量（构造一个新的、只有最新时刻测量的特征）
        let p_init = st.features_slam[&1].get_xyz(false);
        let p2 = p_init - Vector3::new(0.5, 0.0, 0.0);
        let uv2 = Vector2::new(p2.x / p2.z, p2.y / p2.z);
        let to_pix = |uv: Vector2<f64>| Vector2::new(600.0 * uv.x + 320.0, 600.0 * uv.y + 240.0);
        let upd_feat = Feature {
            featid: 1,
            to_delete: false,
            timestamps: HashMap::from([(0usize, vec![2.0f64])]),
            uvs: HashMap::from([(0usize, vec![to_pix(uv2).cast()])]),
            uvs_norm: HashMap::from([(0usize, vec![uv2.cast()])]),
            anchor_cam_id: -1,
            anchor_clone_timestamp: 0.0,
            p_FinA: Vector3::zeros(),
            p_FinG: Vector3::zeros(),
        };
        let mut update_vec = vec![upd_feat];
        updater.update(&mut st, &mut update_vec);
        // 更新后特征仍存在（不应被边缘化/删除）；协方差维度不变
        assert!(st.features_slam.contains_key(&1));
        assert_eq!(st.cov.nrows(), cov_init);
        // 更新不应破坏特征位置（无噪声残差 → 位置基本不变）
        let p_after = st.features_slam[&1].get_xyz(false);
        assert!((p_after - p_init).norm() < 0.1, "p_after = {p_after}");
    }

    #[test]
    fn change_anchors_relocates_anchored_landmark() {
        // 3 个克隆 + 锚在最早克隆上的锚定 SLAM 特征
        let (mut st, feat) = slam_scene();
        // 再加一个克隆（t=3，平移 1.0），使 clones=3
        st.timestamp = 3.0;
        st.imu.set_value(
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        st.imu.set_fej(
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        crate::state_helper::augment_clone(&mut st, &Vector3::zeros());
        // 强制窗口超限：把 max_clone_size 调小（2 < 3）触发边缘化
        st.options.max_clone_size = 2;

        // 用 MSCKF 逆深度表示初始化（锚定 → 锚在最早克隆 t=1）
        let mut updater = UpdaterSlam::new(
            UpdaterOptions::default(),
            FeatRepresentation::AnchoredMsckfInverseDepth,
            TriangulationOptions::default(),
        );
        let mut feats = vec![feat];
        updater.delayed_init(&mut st, &mut feats);
        assert!(st.features_slam.contains_key(&1));
        let cov_before = st.cov.clone();

        // 手动把锚设到将边缘化的最老克隆 t=1（模拟锚恰在 marg 时刻，
        // 对照 C++：三角化锚点通常在最后测量时刻，此处直接构造触发条件）
        {
            let lm = st.features_slam.get_mut(&1).unwrap();
            lm.anchor_clone_timestamp = 1.0;
            // 重设锚点系位置（世界系 (0.5,0.2,3.0)，t=1 相机在原点）
            lm.set_from_xyz(&Vector3::new(0.5, 0.2, 3.0), false);
            lm.set_from_xyz(&Vector3::new(0.5, 0.2, 3.0), true);
        }

        // change_anchors：锚 t=1 是 marg 时刻 → 重锚到最新 t=3
        updater.change_anchors(&mut st);
        let lm = &st.features_slam[&1];
        assert!(
            (lm.anchor_clone_timestamp - 3.0).abs() < 1e-9,
            "锚应切到最新克隆 t=3，got {}",
            lm.anchor_clone_timestamp
        );
        assert!(lm.has_had_anchor_change);
        // 协方差维度不变
        assert_eq!(st.cov.nrows(), cov_before.nrows());
        // 特征位置（锚点系）仍接近真值（世界系 (0.5,0.2,3.0) → t=3 锚点系）
        let p = lm.get_xyz(false);
        // t=3 相机在 (1,0,0)，特征世界 (0.5,0.2,3.0) → 锚点系 = (-0.5,0.2,3.0)
        assert!((p - Vector3::new(-0.5, 0.2, 3.0)).norm() < 0.1, "p = {p}");
    }

    #[test]
    fn change_anchors_skips_global_landmarks() {
        // GLOBAL_3D 特征不应被重锚（change_anchors 应 no-op）
        let (mut st, feat) = slam_scene();
        st.options.max_clone_size = 2;
        let mut updater = UpdaterSlam::new(
            UpdaterOptions::default(),
            FeatRepresentation::Global3D,
            TriangulationOptions::default(),
        );
        let mut feats = vec![feat];
        updater.delayed_init(&mut st, &mut feats);
        assert!(st.features_slam.contains_key(&1));
        let lm = st.features_slam[&1].clone();
        let cov_before = st.cov.clone();
        updater.change_anchors(&mut st);
        assert_eq!(st.cov, cov_before);
        assert_eq!(st.features_slam[&1].representation, lm.representation);
        let _ = lm;
    }

    #[test]
    fn slam_update_with_unknown_feature_panics() {
        let (mut st, feat) = slam_scene();
        let mut updater = UpdaterSlam::new(
            UpdaterOptions::default(),
            FeatRepresentation::Global3D,
            TriangulationOptions::default(),
        );
        let mut feats = vec![feat];
        // 未初始化就直接 update → panic（对照 C++ assert）
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            updater.update(&mut st, &mut feats);
        }));
        assert!(result.is_err());
    }

    #[test]
    fn vio_manager_slam_options_flow_through() {
        // VioManagerOptions 的 slam 选项应传导到 VioManager（构造不 panic）
        let params = VioManagerOptions {
            slam_options: UpdaterOptions {
                sigma_pix: 1.5,
                ..UpdaterOptions::default()
            },
            ..VioManagerOptions::default()
        };
        let cameras = std::collections::BTreeMap::new();
        let tracker = firefly_vio_core::track::TrackKlt::new(
            HashMap::new(),
            200,
            0,
            false,
            firefly_vio_core::track::HistogramMethod::None,
            10,
            5,
            5,
            15,
        );
        let mgr = crate::vio_manager::VioManager::new(params, cameras, tracker);
        assert!((mgr.updater_slam.options.sigma_pix - 1.5).abs() < 1e-12);
        let _ = ImuData {
            timestamp: 0.0,
            wm: Vector3::zeros(),
            am: Vector3::zeros(),
        };
    }
}
