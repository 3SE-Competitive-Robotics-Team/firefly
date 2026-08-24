//! 更新辅助（对照 `OpenVINS` `ov_msckf/update/UpdaterHelper.cpp/.h`）。
//!
//! - [`get_feature_jacobian_full`]：特征测量残差 + 对状态/特征的全雅可比；
//! - [`nullspace_project_inplace`]：Givens 旋转把特征雅可比归零（Golub
//!   《Matrix Computations》5.2.4），消除特征位置未知量；
//! - [`measurement_compress_inplace`]：测量压缩（同样的 Givens 上三角化）。
//!
//! 特征表示仅实现 `GLOBAL_3D`（`StateOptions` 默认）；逆深度/锚定表示
//! 标注 TODO（SLAM 移植时补充）。

// 雅可比组装中的单字符符号（m/n 为行列、a/b 为消零元素）对照 Eigen/Golub
// 源码约定；线性代数代码保留原符号更可审计。
#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::too_many_lines
)]

use firefly_vio_core::feat::Feature;
use firefly_vio_types::quat_ops::skew_x;
use firefly_vio_types::var::Variable;
use nalgebra::linalg::givens::GivensRotation;
use nalgebra::{DMatrix, DVector, Matrix3, Vector2};

use crate::state::State;

/// 特征雅可比组装结果（对照 `get_feature_jacobian_full` 的输出）。
#[derive(Debug, Clone)]
pub struct FeatureJacobian {
    /// 对特征位置的雅可比（`2m` × 3；`GLOBAL_3D`）。
    pub h_f: DMatrix<f64>,
    /// 对状态变量的雅可比（`2m` × `total_hx`，列序对应 `x_order`）。
    pub h_x: DMatrix<f64>,
    /// 残差（2m）。
    pub res: DVector<f64>,
    /// `h_x` 列序对应的变量 `(id, size)` 列表。
    pub x_order: Vec<(i32, usize)>,
}

/// 组装一个特征的测量残差与雅可比（对照
/// `UpdaterHelper::get_feature_jacobian_full`）。
///
/// 支持 `GLOBAL_3D` 与 `ANCHORED_MSCKF_INVERSE_DEPTH`（其余表示标注 TODO）。
/// 残差：`z = uv_meas − distort(project(p_FinG))`；链式雅可比：
/// `dz/dx = dz/dzn · dzn/dpfc · dpfc/dx`（投影 → 畸变 → 状态）。
///
/// 锚定表示：`p_FinG` 由锚点克隆/外参与 `p_FinA` 计算，锚点克隆进入
/// `x_order`，`H_f = dz_dpfg · dpfg_dlambda`（`dpfg_dlambda` 为对逆深度
/// 参数的雅可比），锚点状态的雅可比经 `dpfg_dx` 链入。
///
/// FEJ：开启时克隆旋转/平移取首估计值（`Rot_fej`/`pos_fej`），特征位置
/// 用当前三角化值（MSCKF 特征每次更新重新三角化）。
///
/// # Panics
/// 特征测量对应的克隆或相机标定缺失（调用方组装错误）。
#[must_use]
pub fn get_feature_jacobian_full(
    state: &State,
    feature: &Feature,
    representation: crate::options::FeatRepresentation,
) -> FeatureJacobian {
    // 总测量数与 H_x 列布局（对照 C++ 的 map_hx/x_order 组装）
    let total_meas: usize = feature.timestamps.values().map(Vec::len).sum();
    let mut x_order: Vec<(i32, usize)> = Vec::new();
    let mut map_hx: std::collections::HashMap<(i32, usize), usize> =
        std::collections::HashMap::new();

    let add_var = |id: i32,
                   size: usize,
                   x_order: &mut Vec<(i32, usize)>,
                   map_hx: &mut std::collections::HashMap<(i32, usize), usize>| {
        let key = (id, size);
        map_hx.entry(key).or_insert_with(|| {
            // 列偏移 = 此前所有变量的尺寸之和（对照 C++ 的 total_hx 累计）；
            // 误用变量序号会使多变量 H_x 的列互相覆盖
            let col = x_order.iter().map(|(_, s)| s).sum::<usize>();
            x_order.push(key);
            col
        });
    };

    // 锚点克隆/外参（锚定表示先注册，对照 C++ 的 is_relative 分支）
    let anchor_clone = if representation.is_relative() {
        let (t, c) = state
            .clones_imu
            .iter()
            .find(|(ct, _)| ct.total_cmp(&feature.anchor_clone_timestamp).is_eq())
            .expect("锚定表示的锚点克隆必须存在");
        if state.options.do_calib_camera_pose {
            let calib = state
                .calib_imu_to_cam
                .get(&(feature.anchor_cam_id as usize))
                .expect("锚点相机必须有外参");
            add_var(calib.id(), 6, &mut x_order, &mut map_hx);
        }
        add_var(c.id(), 6, &mut x_order, &mut map_hx);
        Some((*t, c))
    } else {
        None
    };

    for (cam_id, times) in &feature.timestamps {
        // 相机外参/内参（标定开启时进 H_x）
        if state.options.do_calib_camera_pose {
            let calib = state
                .calib_imu_to_cam
                .get(cam_id)
                .expect("特征相机必须有外参标定");
            add_var(calib.id(), 6, &mut x_order, &mut map_hx);
        }
        if state.options.do_calib_camera_intrinsics {
            let intrin = state
                .cam_intrinsics
                .get(cam_id)
                .expect("特征相机必须有内参标定");
            add_var(intrin.id(), 8, &mut x_order, &mut map_hx);
        }
        // 该相机所有测量时刻的克隆
        for t in times {
            let clone = state
                .clones_imu
                .iter()
                .find(|(ct, _)| ct.total_cmp(t).is_eq())
                .map(|(_, c)| c)
                .expect("特征测量时刻必须存在对应克隆");
            add_var(clone.id(), 6, &mut x_order, &mut map_hx);
        }
    }

    // 特征位置（全局系）：锚定表示由锚点计算（对照 C++ 的 p_FinG 段）
    let (p_fin_g, dpfg_dlambda, dpfg_dx) = if representation.is_relative() {
        let (_, anchor) = anchor_clone.expect("锚点克隆已注册");
        let calib = state
            .calib_imu_to_cam
            .get(&(feature.anchor_cam_id as usize))
            .expect("锚点相机必须有外参");
        let r_ito_c_a = calib.rot();
        let p_iin_c_a = calib.pos();
        let r_gto_i_a = anchor.rot();
        let p_iin_g_a = anchor.pos();
        let r_c_to_g = r_gto_i_a.transpose() * r_ito_c_a.transpose();
        let p_fin_g = r_c_to_g * (feature.p_FinA - p_iin_c_a) + p_iin_g_a;

        // dpfg_dx：对锚点克隆/外参（对照 C++ 的 H_anc/H_calib；所有锚定表示共享）
        let mut dpfg_dx = vec![(anchor.id(), 6, {
            let mut h_anc = DMatrix::<f64>::zeros(3, 6);
            h_anc.view_mut((0, 0), (3, 3)).copy_from(
                &(-r_gto_i_a.transpose()
                    * skew_x(&(r_ito_c_a.transpose() * (feature.p_FinA - p_iin_c_a)))),
            );
            h_anc.view_mut((0, 3), (3, 3)).fill_diagonal(1.0);
            h_anc
        })];
        if state.options.do_calib_camera_pose {
            let mut h_calib = DMatrix::<f64>::zeros(3, 6);
            h_calib
                .view_mut((0, 0), (3, 3))
                .copy_from(&(-r_c_to_g * skew_x(&(feature.p_FinA - p_iin_c_a))));
            h_calib.view_mut((0, 3), (3, 3)).copy_from(&(-r_c_to_g));
            let calib = state
                .calib_imu_to_cam
                .get(&(feature.anchor_cam_id as usize))
                .expect("锚点相机必须有外参");
            dpfg_dx.push((calib.id(), 6, h_calib));
        }

        // dpfg_dlambda：按表示取 p_FinA→p_FinG 的链式雅可比（对照 C++
        // `get_feature_jacobian_representation` 的 H_f 构造）
        let p = feature.p_FinA;
        // 各分支统一为 DMatrix（3×3 或 Single 的 3×1，与 h_f 列数一致）
        let dpfg_dlambda: DMatrix<f64> = match representation {
            // ANCHORED_3D：p_FinA 即参数（对照 C++ H_f = R_CtoG）
            crate::options::FeatRepresentation::Anchored3D => {
                let mut out = DMatrix::<f64>::zeros(3, 3);
                out.copy_from(&r_c_to_g);
                out
            }
            // 锚定全逆深度（θ,φ,ρ；对照 C++ d_pfinA_dpinv 的 sin/cos 版）
            crate::options::FeatRepresentation::AnchoredFullInverseDepth => {
                let rho = 1.0 / p.norm();
                let phi = (rho * p.z).acos();
                let theta = p.y.atan2(p.x);
                let (sin_th, cos_th) = theta.sin_cos();
                let (sin_phi, cos_phi) = phi.sin_cos();
                let mut d = Matrix3::zeros();
                d[(0, 0)] = -(1.0 / rho) * sin_th * sin_phi;
                d[(0, 1)] = (1.0 / rho) * cos_th * cos_phi;
                d[(0, 2)] = -(1.0 / (rho * rho)) * cos_th * sin_phi;
                d[(1, 0)] = (1.0 / rho) * cos_th * sin_phi;
                d[(1, 1)] = (1.0 / rho) * sin_th * cos_phi;
                d[(1, 2)] = -(1.0 / (rho * rho)) * sin_th * sin_phi;
                d[(2, 1)] = -(1.0 / rho) * sin_phi;
                d[(2, 2)] = -(1.0 / (rho * rho)) * cos_phi;
                let mut out = DMatrix::<f64>::zeros(3, 3);
                out.copy_from(&(r_c_to_g * d));
                out
            }
            // MSCKF 逆深度（α,β,ρ；对照 C++ d_pfinA_dpinv）
            crate::options::FeatRepresentation::AnchoredMsckfInverseDepth => {
                let alpha = p.x / p.z;
                let beta = p.y / p.z;
                let rho = 1.0 / p.z;
                let mut d = Matrix3::zeros();
                d[(0, 0)] = 1.0 / rho;
                d[(0, 2)] = -(1.0 / (rho * rho)) * alpha;
                d[(1, 1)] = 1.0 / rho;
                d[(1, 2)] = -(1.0 / (rho * rho)) * beta;
                d[(2, 2)] = -(1.0 / (rho * rho));
                let mut out = DMatrix::<f64>::zeros(3, 3);
                out.copy_from(&(r_c_to_g * d));
                out
            }
            // 单逆深度（ρ；对照 C++ d_pfinA_drho = −(1/ρ²)·bearing，3×1）
            crate::options::FeatRepresentation::AnchoredInverseDepthSingle => {
                let rho = 1.0 / p.z;
                let bearing = rho * p;
                let d = -(1.0 / (rho * rho)) * bearing;
                let mut out = DMatrix::<f64>::zeros(3, 1);
                out.column_mut(0).copy_from(&(r_c_to_g * d));
                out
            }
            other => {
                log::warn!("特征表示 {other:?} 未实现，回退 GLOBAL_3D");
                let mut out = DMatrix::<f64>::zeros(3, 3);
                out.fill_with_identity();
                out
            }
        };
        (p_fin_g, dpfg_dlambda, dpfg_dx)
    } else {
        let mut d = DMatrix::<f64>::zeros(3, 3);
        d.fill_with_identity();
        (feature.p_FinG, d, Vec::new())
    };

    // 残差与雅可比（对照 C++ 的测量循环）
    let jacobsize =
        if representation == crate::options::FeatRepresentation::AnchoredInverseDepthSingle {
            1
        } else {
            3
        };
    let mut res = DVector::zeros(2 * total_meas);
    let mut h_f = DMatrix::zeros(2 * total_meas, jacobsize);
    let hx_cols: usize = x_order.iter().map(|(_, s)| s).sum();
    let mut h_x = DMatrix::zeros(2 * total_meas, hx_cols);

    let mut c = 0usize;
    for (cam_id, times) in &feature.timestamps {
        let calib = state
            .calib_imu_to_cam
            .get(cam_id)
            .expect("特征相机必须有外参");
        let cam = state
            .cameras
            .get(cam_id)
            .expect("特征相机必须有畸变模型对象");
        let r_ito_c = calib.rot();
        let p_iin_c = calib.pos();

        for (m, t) in times.iter().enumerate() {
            let clone = state
                .clones_imu
                .iter()
                .find(|(ct, _)| ct.total_cmp(t).is_eq())
                .map(|(_, c)| c)
                .expect("特征测量时刻必须存在对应克隆");
            // 残差与投影用**当前**位姿（对照 C++：res 在 FEJ 覆盖段之前用
            // `clone_Ii->Rot()/pos()` 当前值计算）；FEJ 只影响雅可比线性化
            // 点——不得用 fej 算残差，否则克隆 fej 随更新变陈旧，EKF 会把
            // 状态系统性往 fej 方向拖。
            let r_gto_ii = clone.rot();
            let p_iin_g = clone.pos();

            // 投影到相机系并归一化
            let p_fin_ii = r_gto_ii * (p_fin_g - p_iin_g);
            let p_fin_ci = r_ito_c * p_fin_ii + p_iin_c;
            let uv_norm = Vector2::new(p_fin_ci.x / p_fin_ci.z, p_fin_ci.y / p_fin_ci.z);

            // 残差统一在**像素**空间：uvs 是原始像素；`distort_d`/`distort_f`
            // 本身就把归一化坐标映射回原始像素（`CamRadtan::distort_f` 末尾
            // `pixel_from_norm(x1,y1)`，即 `fx·x+cx`），故 `uv_dist` 已是像素、
            // 直接与 `uv_m` 相减，不得再做投影换算（会双重缩放）。
            let uv_dist = cam.distort_d(uv_norm.cast());
            let uv_pred = uv_dist;

            // 残差：测量 − 预测
            let uv_m = feature.uvs[cam_id][m];
            let r2 = Vector2::new(f64::from(uv_m.x), f64::from(uv_m.y)) - uv_pred;
            res.rows_range_mut(2 * c..2 * c + 2).copy_from(&r2);

            // 雅可比链（对照 C++：dz_dzn/dzn_dpfc/dpfc_dpfg/dpfc_dclone 在
            // FEJ 覆盖段之后计算，即 do_fej 时用 `Rot_fej()/pos_fej()` 与
            // `p_FinG_fej`（GLOBAL_3D 的 p_FinG_fej = 当前三角化 p_FinG））。
            let (r_gto_ii_j, p_iin_g_j, p_fin_g_j) = if state.options.do_fej {
                (clone.rot_fej(), clone.pos_fej(), p_fin_g)
            } else {
                (r_gto_ii, p_iin_g, p_fin_g)
            };
            let p_fin_ii_j = r_gto_ii_j * (p_fin_g_j - p_iin_g_j);
            let p_fin_ci_j = r_ito_c * p_fin_ii_j + p_iin_c;
            let uv_norm_j = Vector2::new(p_fin_ci_j.x / p_fin_ci_j.z, p_fin_ci_j.y / p_fin_ci_j.z);
            let (dz_dzn, _) = cam.compute_distort_jacobian(uv_norm_j.cast());
            // dzn/dpfc：2×3（对 p_FinCi 的 x/y/z 求导）
            let mut dzn_dpfc = DMatrix::zeros(2, 3);
            let z2 = p_fin_ci_j.z * p_fin_ci_j.z;
            dzn_dpfc[(0, 0)] = 1.0 / p_fin_ci_j.z;
            dzn_dpfc[(0, 2)] = -p_fin_ci_j.x / z2;
            dzn_dpfc[(1, 1)] = 1.0 / p_fin_ci_j.z;
            dzn_dpfc[(1, 2)] = -p_fin_ci_j.y / z2;
            // dz_dpfc：d(像素残差)/d(p_FinCi)。dz_dzn=∂(像素)/∂(归一化) 已含
            // fx/fy/cx/cy（distort_f 输出像素），无需再乘 pix_scale。
            let dz_dpfc = dz_dzn * dzn_dpfc;

            let dpfc_dpfg = r_ito_c * r_gto_ii_j;
            // dpfc/dclone：3×6（对克隆的旋转/平移 6 维误差状态）
            let mut dpfc_dclone = DMatrix::<f64>::zeros(3, 6);
            dpfc_dclone
                .view_mut((0, 0), (3, 3))
                .copy_from(&(r_ito_c * skew_x(&p_fin_ii_j)));
            dpfc_dclone
                .view_mut((0, 3), (3, 3))
                .copy_from(&(-dpfc_dpfg));

            let dz_dpfg = &dz_dpfc * dpfc_dpfg;

            // 特征雅可比：dz_dpfg · dpfg_dlambda
            h_f.view_mut((2 * c, 0), (2, jacobsize))
                .copy_from(&(dz_dpfg * &dpfg_dlambda));

            // 克隆雅可比
            let col = map_hx[&(clone.id(), 6)];
            let hx_block = &dz_dpfc * dpfc_dclone;
            h_x.view_mut((2 * c, col), (2, 6)).copy_from(&hx_block);

            // 锚定表示：dpfg_dx 链入（对照 C++ 的 dpfg_dx_order 循环）
            for (var_id, var_size, dpfg_dx_i) in &dpfg_dx {
                let col = map_hx[&(*var_id, *var_size)];
                let block = dz_dpfg * dpfg_dx_i;
                let mut cur = h_x.view((2 * c, col), (2, *var_size)).into_owned();
                cur += block;
                h_x.view_mut((2 * c, col), (2, *var_size)).copy_from(&cur);
            }

            // 相机外参雅可比（标定开启时）
            if state.options.do_calib_camera_pose {
                let mut dpfc_dcalib = Matrix3::zeros();
                dpfc_dcalib
                    .view_mut((0, 0), (3, 3))
                    .copy_from(&skew_x(&(p_fin_ci - p_iin_c)));
                dpfc_dcalib
                    .view_mut((0, 3), (3, 3))
                    .copy_from(&Matrix3::identity());
                let col = map_hx[&(calib.id(), 6)];
                let mut cur = h_x.view((2 * c, col), (2, 6)).into_owned();
                cur += &dz_dpfc * dpfc_dcalib;
                h_x.view_mut((2 * c, col), (2, 6)).copy_from(&cur);
            }

            c += 1;
        }
    }

    FeatureJacobian {
        h_f,
        h_x,
        res,
        x_order,
    }
}

/// Givens 旋转：把 `h` 的第 `n` 列在行 `m-1, m` 处消零（对照
/// `UpdaterHelper::nullspace_project_inplace` 的 Givens 循环）。
///
/// 同时左乘 `h_x` 与 `res` 的对应两行。`pub(crate)` 供 `StateHelper::initialize`
/// 复用（其特征 QR 采用同一套行旋转）。
pub(crate) fn givens_zero(
    h: &mut DMatrix<f64>,
    h_x: &mut DMatrix<f64>,
    res: &mut DVector<f64>,
    m: usize,
    n: usize,
) {
    let (a, b) = (h[(m - 1, n)], h[(m, n)]);
    // 消零目标元已为零：恒等旋转，跳过（Eigen makeGivens + applyOnTheLeft
    // (adjoint) 的语义，由 nalgebra GivensRotation 等价提供）
    let Some((g, _)) = GivensRotation::cancel_y(&Vector2::new(a, b)) else {
        return;
    };
    let cols = h.ncols();
    g.rotate(&mut h.view_range_mut(m - 1..m + 1, n..cols));
    g.rotate(&mut h_x.rows_range_mut(m - 1..m + 1));
    g.rotate(&mut res.rows_range_mut(m - 1..m + 1));
}

/// 零空间投影：Givens 上三角化 `H_f` 并截取下部行（对照
/// `UpdaterHelper::nullspace_project_inplace`）。
///
/// 作用后 `H_f` 的前 `cols` 行变为上三角（`(m, n)` 处 `m > n` 为零），
/// `H_x`/`res` 保留下部 `rows − cols` 行——特征位置未知量被消除。
pub fn nullspace_project_inplace(
    h_f: &mut DMatrix<f64>,
    h_x: &mut DMatrix<f64>,
    res: &mut DVector<f64>,
) {
    for n in 0..h_f.ncols() {
        for m in (n + 1..h_f.nrows()).rev() {
            givens_zero(h_f, h_x, res, m, n);
        }
    }
    // 截取下部（对照 C++ 的 block 截取：`H_x.block(H_f.cols(),0,rows-cols,cols)`）
    let keep = h_f.nrows() - h_f.ncols();
    let start = h_f.ncols();
    *h_x = h_x.rows_range(start..start + keep).into_owned();
    *res = res.rows_range(start..start + keep).into_owned();
    debug_assert_eq!(h_x.nrows(), res.len());
}

/// 表示雅可比结果：`H_f` + 锚点状态块 `(id, size, 3×6)` 列表。
pub type ReprJacobian = (DMatrix<f64>, Vec<(i32, usize, DMatrix<f64>)>);

/// 特征表示雅可比（对照 `UpdaterHelper::get_feature_jacobian_representation`）。
///
/// 返回 `p_FinG`（全局系位置）对特征表示参数的雅可比 `H_f`，以及锚点
/// 克隆/外参状态块（每块 3×6）与对应 `(id, size)` 顺序 `x_order`。
/// `GLOBAL_3D` 时 `H_f` 为单位阵且无状态块（`change_anchors` 只处理锚定表示）。
///
/// FEJ：开启时锚点旋转/平移取首估计、`p_FinA` 取"最佳全局位置"重投影
/// （对照 C++ 的 `p_FinG_best` 段）。
///
/// # Panics
/// 锚定表示但锚点克隆/外参缺失（调用方组装错误）。
#[must_use]
pub fn get_feature_jacobian_representation(
    state: &State,
    feature: &Feature,
    representation: crate::options::FeatRepresentation,
) -> ReprJacobian {
    // GLOBAL_3D：H_f = I，无状态块（对照 C++ 直接返回）
    if representation == crate::options::FeatRepresentation::Global3D {
        let mut h_f = DMatrix::<f64>::zeros(3, 3);
        h_f.fill_with_identity();
        return (h_f, Vec::new());
    }

    // 锚定表示：锚点克隆 + 外参（对照 C++ 的 H_anc/H_calib 构造）
    let (anchor_t, anchor) = state
        .clones_imu
        .iter()
        .find(|(ct, _)| ct.total_cmp(&feature.anchor_clone_timestamp).is_eq())
        .expect("锚定表示的锚点克隆必须存在");
    let calib = state
        .calib_imu_to_cam
        .get(&(feature.anchor_cam_id as usize))
        .expect("锚点相机必须有外参");

    // 当前/FEJ 线性化点（对照 C++ 的 R_ItoC/p_IinC 固定、R_GtoI/p_IinG 与
    // p_FinA 按 do_fej 处理）
    let r_ito_c = calib.rot();
    let p_iin_c = calib.pos();
    let r_gto_i_cur = anchor.rot();
    let p_iin_g_cur = anchor.pos();
    let (r_gto_i, _p_iin_g, p_fin_a) = if state.options.do_fej {
        // "最佳"全局位置（对照 C++ p_FinG_best：用当前锚点值计算）
        let p_fin_g_best =
            r_gto_i_cur.transpose() * r_ito_c.transpose() * (feature.p_FinA - p_iin_c)
                + p_iin_g_cur;
        // 再变换到 FEJ 锚点系（对照 C++ 覆盖段）
        let r_fej = anchor.rot_fej();
        let p_fej = anchor.pos_fej();
        let p_fin_a_fej = (r_fej.transpose() * r_ito_c.transpose()).transpose()
            * (p_fin_g_best - p_fej)
            + p_iin_c;
        (r_fej, p_fej, p_fin_a_fej)
    } else {
        (r_gto_i_cur, p_iin_g_cur, feature.p_FinA)
    };
    let r_c_to_g = r_gto_i.transpose() * r_ito_c.transpose();

    // 锚点克隆块（对照 C++ H_anc：3×6）
    let mut h_anc = DMatrix::<f64>::zeros(3, 6);
    h_anc
        .view_mut((0, 0), (3, 3))
        .copy_from(&(-r_gto_i.transpose() * skew_x(&(r_ito_c.transpose() * (p_fin_a - p_iin_c)))));
    h_anc.view_mut((0, 3), (3, 3)).fill_diagonal(1.0);
    let mut h_x = vec![(anchor.id(), 6, h_anc)];
    let mut x_order = vec![(anchor_t, 6usize)];
    let _ = &mut x_order;
    // 外参块（标定开启时；对照 C++ H_calib）
    if state.options.do_calib_camera_pose {
        let mut h_calib = DMatrix::<f64>::zeros(3, 6);
        h_calib
            .view_mut((0, 0), (3, 3))
            .copy_from(&(-r_c_to_g * skew_x(&(p_fin_a - p_iin_c))));
        h_calib.view_mut((0, 3), (3, 3)).copy_from(&(-r_c_to_g));
        h_x.push((calib.id(), 6, h_calib));
    }

    // H_f：p_FinA → p_FinG 的链式雅可比（对照 C++ 各表示分支）
    let p = p_fin_a;
    let h_f = match representation {
        crate::options::FeatRepresentation::Anchored3D => {
            let mut out = DMatrix::<f64>::zeros(3, 3);
            out.copy_from(&r_c_to_g);
            out
        }
        crate::options::FeatRepresentation::AnchoredFullInverseDepth => {
            let rho = 1.0 / p.norm();
            let phi = (rho * p.z).acos();
            let theta = p.y.atan2(p.x);
            let (sin_th, cos_th) = theta.sin_cos();
            let (sin_phi, cos_phi) = phi.sin_cos();
            let mut d = Matrix3::zeros();
            d[(0, 0)] = -(1.0 / rho) * sin_th * sin_phi;
            d[(0, 1)] = (1.0 / rho) * cos_th * cos_phi;
            d[(0, 2)] = -(1.0 / (rho * rho)) * cos_th * sin_phi;
            d[(1, 0)] = (1.0 / rho) * cos_th * sin_phi;
            d[(1, 1)] = (1.0 / rho) * sin_th * cos_phi;
            d[(1, 2)] = -(1.0 / (rho * rho)) * sin_th * sin_phi;
            d[(2, 1)] = -(1.0 / rho) * sin_phi;
            d[(2, 2)] = -(1.0 / (rho * rho)) * cos_phi;
            let mut out = DMatrix::<f64>::zeros(3, 3);
            out.copy_from(&(r_c_to_g * d));
            out
        }
        crate::options::FeatRepresentation::AnchoredMsckfInverseDepth => {
            let alpha = p.x / p.z;
            let beta = p.y / p.z;
            let rho = 1.0 / p.z;
            let mut d = Matrix3::zeros();
            d[(0, 0)] = 1.0 / rho;
            d[(0, 2)] = -(1.0 / (rho * rho)) * alpha;
            d[(1, 1)] = 1.0 / rho;
            d[(1, 2)] = -(1.0 / (rho * rho)) * beta;
            d[(2, 2)] = -(1.0 / (rho * rho));
            let mut out = DMatrix::<f64>::zeros(3, 3);
            out.copy_from(&(r_c_to_g * d));
            out
        }
        crate::options::FeatRepresentation::AnchoredInverseDepthSingle => {
            let rho = 1.0 / p.z;
            let bearing = rho * p;
            let d = -(1.0 / (rho * rho)) * bearing;
            let mut out = DMatrix::<f64>::zeros(3, 1);
            out.column_mut(0).copy_from(&(r_c_to_g * d));
            out
        }
        other => {
            log::warn!("特征表示 {other:?} 未实现，回退 GLOBAL_3D");
            let mut out = DMatrix::<f64>::zeros(3, 3);
            out.fill_with_identity();
            out
        }
    };
    (h_f, h_x)
}

/// 测量压缩：Givens 上三角化 `H_x` 并截取前 `min(rows, cols)` 行（对照
/// `UpdaterHelper::measurement_compress_inplace`）。
pub fn measurement_compress_inplace(h_x: &mut DMatrix<f64>, res: &mut DVector<f64>) {
    if h_x.nrows() <= h_x.ncols() {
        return;
    }
    for n in 0..h_x.ncols() {
        for m in (n + 1..h_x.nrows()).rev() {
            let (a, b) = (h_x[(m - 1, n)], h_x[(m, n)]);
            let Some((g, _)) = GivensRotation::cancel_y(&Vector2::new(a, b)) else {
                continue;
            };
            let cols = h_x.ncols();
            g.rotate(&mut h_x.view_range_mut(m - 1..m + 1, n..cols));
            g.rotate(&mut res.rows_range_mut(m - 1..m + 1));
        }
    }
    let keep = h_x.nrows().min(h_x.ncols());
    *h_x = h_x.rows_range(0..keep).into_owned();
    *res = res.rows_range(0..keep).into_owned();
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn givens_zeroes_target() {
        // 随机 2 维向量：旋转后第二分量应为零
        let mut h = DMatrix::from_row_slice(2, 3, &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let mut h_x = DMatrix::from_row_slice(2, 2, &[1.0, 0.0, 0.0, 1.0]);
        let mut res = DVector::from_vec(vec![1.0, 2.0]);
        givens_zero(&mut h, &mut h_x, &mut res, 1, 0);
        assert!(h[(1, 0)].abs() < 1e-12, "h[1,0] = {}", h[(1, 0)]);
        // 行范数保持（正交旋转）：第 0 列 [1, 4] → 范数 √17
        let n1 = (h[(0, 0)].powi(2) + h[(1, 0)].powi(2)).sqrt();
        assert!((n1 - 17.0_f64.sqrt()).abs() < 1e-9);
    }

    #[test]
    fn nullspace_projects_and_truncates() {
        // H_f: 5×3（2 维零空间），H_x: 5×4，res: 5
        let mut h_f = DMatrix::from_fn(5, 3, |i, j| (i * 3 + j) as f64 + 1.0);
        let mut h_x = DMatrix::from_fn(5, 4, |i, j| (i * 4 + j) as f64 * 0.5);
        let mut res = DVector::from_fn(5, |i, _| i as f64);
        nullspace_project_inplace(&mut h_f, &mut h_x, &mut res);
        assert_eq!(h_x.nrows(), 2);
        assert_eq!(res.len(), 2);
        // H_f 上三角化：验证 m > n 处为零
        for n in 0..3 {
            for m in n + 1..5 {
                assert!(h_f[(m, n)].abs() < 1e-9, "h_f[{m},{n}] = {}", h_f[(m, n)]);
            }
        }
    }

    #[test]
    fn compress_reduces_rows() {
        let mut h_x = DMatrix::from_fn(6, 2, |i, j| (i * 2 + j) as f64);
        let mut res = DVector::from_fn(6, |i, _| i as f64);
        measurement_compress_inplace(&mut h_x, &mut res);
        assert_eq!(h_x.nrows(), 2);
        assert_eq!(res.len(), 2);
        // 上三角：h_x[1,0] = 0
        assert!(h_x[(1, 0)].abs() < 1e-9);
    }

    /// 构造带一个克隆 + 相机模型的 State（测试用）。
    fn state_with_clone_and_camera() -> (crate::state::State, f64) {
        use crate::options::StateOptions;
        use firefly_vio_core::cam::CamRadtan;
        use firefly_vio_core::feat::Feature;
        use std::collections::HashMap;
        use std::sync::Arc;

        let opts = StateOptions::default();
        let mut st = crate::state::State::new(opts);
        st.timestamp = 1.0;
        st.imu.set_value(
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        crate::state_helper::augment_clone(&mut st, &Vector3::zeros());
        // 相机模型：单位内参（无畸变）
        let cam = CamRadtan::new(640, 480, &[600.0, 600.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0]);
        st.cameras.insert(0, Arc::new(cam));
        // 特征在锚点系 (0.5, 0.2, 3.0)，观测于 t=1.0
        let mut feat = Feature {
            featid: 1,
            to_delete: false,
            timestamps: HashMap::from([(0usize, vec![1.0f64])]),
            uvs: HashMap::from([(0usize, vec![nalgebra::Vector2::new(0.1f32, 0.04f32)])]),
            uvs_norm: HashMap::new(),
            anchor_cam_id: 0,
            anchor_clone_timestamp: 1.0,
            p_FinA: Vector3::new(0.5, 0.2, 3.0),
            p_FinG: Vector3::new(0.5, 0.2, 3.0),
        };
        let _ = &mut feat;
        (st, 1.0)
    }

    #[test]
    fn anchored_msckf_inverse_depth_matches_global_3d_residual() {
        use crate::options::FeatRepresentation;
        use firefly_vio_core::feat::Feature;
        use std::collections::HashMap;

        let (st, _t) = state_with_clone_and_camera();
        // 克隆位姿 = 单位（imu 在原点静止）
        let mut feat = Feature {
            featid: 1,
            to_delete: false,
            timestamps: HashMap::from([(0usize, vec![1.0f64])]),
            uvs: HashMap::from([(0usize, vec![nalgebra::Vector2::new(0.1f32, 0.04f32)])]),
            uvs_norm: HashMap::new(),
            anchor_cam_id: 0,
            anchor_clone_timestamp: 1.0,
            p_FinA: Vector3::new(0.5, 0.2, 3.0),
            p_FinG: Vector3::new(0.5, 0.2, 3.0),
        };
        // 特征全局位置与锚点系一致（单位位姿）
        feat.p_FinG = feat.p_FinA;

        // GLOBAL_3D 雅可比
        let jac_g = get_feature_jacobian_full(&st, &feat, FeatRepresentation::Global3D);
        assert_eq!(jac_g.h_f.nrows(), 2);
        assert_eq!(jac_g.h_f.ncols(), 3);

        // MSCKF 逆深度雅可比（锚定）
        let jac_i =
            get_feature_jacobian_full(&st, &feat, FeatRepresentation::AnchoredMsckfInverseDepth);
        assert_eq!(jac_i.h_f.nrows(), 2);
        assert_eq!(jac_i.h_f.ncols(), 3);
        // 残差一致（p_FinG 相同）
        assert!((jac_g.res - jac_i.res).norm() < 1e-12);
        // 锚定表示 x_order 含锚点克隆（id=15）
        assert!(jac_i.x_order.iter().any(|(id, _)| *id == 15));
    }
}
