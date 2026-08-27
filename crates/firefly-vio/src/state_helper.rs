//! 状态辅助（对照 `OpenVINS` `ov_msckf/state/StateHelper.cpp/.h`）。
//!
//! 所有函数以 `State` 为第一参数（C++ 的静态方法风格）：
//! - [`ekf_propagation`]：协方差传播 `P ← ΦPΦᵀ + Q`（按变量块组装）；
//! - [`ekf_update`]：EKF 更新（`K = PHᵀ(HPHᵀ+R)⁻¹`，Cholesky 求逆）；
//! - [`augment_clone`] / [`marginalize_old_clone`]：滑动窗口克隆增广与边缘化；
//! - [`marginalize`]：通用变量边缘化（切块 + id 重排）。

use firefly_vio_types::var::Variable;
use nalgebra::{DMatrix, DVector, Vector3};

use crate::landmark::Landmark;
use crate::state::State;

/// 协方差传播 `P ← ΦPΦᵀ + Q`（对照 `StateHelper::EKFPropagation`）。
///
/// `order_new`/`order_old` 为传播前后各变量的 `(id, size)` 列表（本实现中
/// 两者维度一致：IMU + 标定，不含克隆）；`phi` 为 `(new × old)` 状态转移，
/// `q` 为传播噪声（`new × new`）。
///
/// # Panics
/// 若 `phi`/`q` 维度与 `order` 不符（debug 断言），或传播后协方差对角出现
/// 负值（非半正定）。
pub fn ekf_propagation(
    state: &mut State,
    order_new: &[(i32, usize)],
    order_old: &[(i32, usize)],
    phi: &DMatrix<f64>,
    q: &DMatrix<f64>,
) {
    let size_new: usize = order_new.iter().map(|(_, s)| s).sum();
    let size_old: usize = order_old.iter().map(|(_, s)| s).sum();
    debug_assert_eq!(phi.nrows(), size_new);
    debug_assert_eq!(phi.ncols(), size_old);
    debug_assert_eq!(q.nrows(), size_new);
    debug_assert_eq!(q.ncols(), size_new);

    // Phi 中各 old 变量的起始列（对照 C++ 的 Phi_id）
    let phi_id: Vec<usize> = order_old
        .iter()
        .scan(0, |acc, (_, s)| {
            let cur = *acc;
            *acc += s;
            Some(cur)
        })
        .collect();

    // Cov_PhiT = P[:, old] · Φᵀ（对照 C++ 的 Cov_PhiT）
    let mut cov_phi_t = DMatrix::zeros(state.cov.nrows(), size_new);
    for (i, (var_id, var_size)) in order_old.iter().enumerate() {
        let id = *var_id as usize;
        let sub = state.cov.view((0, id), (state.cov.nrows(), *var_size));
        let phi_sub = phi.view((0, phi_id[i]), (size_new, *var_size));
        cov_phi_t += sub * phi_sub.transpose();
    }

    // Phi_Cov_PhiT = Q + Φ · Cov_PhiT 的 old 块部分（对照 C++）
    let mut phi_cov_phi_t = q.clone();
    for (i, (var_id, var_size)) in order_old.iter().enumerate() {
        let id = *var_id as usize;
        let phi_sub = phi.view((0, phi_id[i]), (size_new, *var_size));
        let cov_phi_sub = cov_phi_t.view((id, 0), (*var_size, size_new));
        phi_cov_phi_t += phi_sub * cov_phi_sub;
    }

    // 写回协方差（对照 C++ 的三块写入）
    let start_id = order_new[0].0 as usize;
    let total = state.cov.nrows();
    state
        .cov
        .view_mut((start_id, 0), (size_new, total))
        .copy_from(&cov_phi_t.transpose());
    state
        .cov
        .view_mut((0, start_id), (total, size_new))
        .copy_from(&cov_phi_t);
    state
        .cov
        .view_mut((start_id, start_id), (size_new, size_new))
        .copy_from(&phi_cov_phi_t);

    // 负对角检查（对照 C++：非半正定则报错退出）
    for i in 0..state.cov.nrows() {
        assert!(
            state.cov[(i, i)] >= 0.0,
            "EKFPropagation: 对角第 {i} 项为负 ({:.2e})，协方差非半正定",
            state.cov[(i, i)]
        );
    }
}

/// EKF 更新（对照 `StateHelper::EKFUpdate`）。
///
/// `h_order` 为测量涉及变量的 `(id, size)` 列表，`h`/`res`/`r` 为雅可比、
/// 残差与测量噪声（`r` 为方阵）。
///
/// # Panics
/// 若 `S = H·P·Hᵀ + R` 不正定（无法 Cholesky），或更新后协方差对角为负。
pub fn ekf_update(
    state: &mut State,
    h_order: &[(i32, usize)],
    h: &DMatrix<f64>,
    res: &DVector<f64>,
    r: &DMatrix<f64>,
) {
    debug_assert_eq!(h.nrows(), res.len());
    debug_assert_eq!(h.nrows(), r.nrows());
    debug_assert_eq!(h.nrows(), r.ncols());

    // H 中各测量变量的起始列（对照 C++ 的 H_id）
    let h_id: Vec<usize> = h_order
        .iter()
        .scan(0, |acc, (_, s)| {
            let cur = *acc;
            *acc += s;
            Some(cur)
        })
        .collect();

    // M_a = P · Hᵀ（按全状态变量遍历；对照 C++ 的 M_i 循环）
    let mut m_a = DMatrix::zeros(state.cov.nrows(), res.len());
    for (var_id, var_size) in state.variable_order() {
        let id = var_id as usize;
        let mut m_i = DMatrix::zeros(var_size, res.len());
        for (i, (meas_id, meas_size)) in h_order.iter().enumerate() {
            let mid = *meas_id as usize;
            let cov_sub = state.cov.view((id, mid), (var_size, *meas_size));
            let h_sub = h.view((0, h_id[i]), (h.nrows(), *meas_size));
            m_i += cov_sub * h_sub.transpose();
        }
        m_a.view_mut((id, 0), (var_size, res.len())).copy_from(&m_i);
    }

    // P_small（测量变量子协方差）→ S = H·P_small·Hᵀ + R → Cholesky 求逆
    let p_small = get_marginal_covariance(state, h_order);
    let s = h * p_small * h.transpose() + r;
    let s_inv = s
        .clone()
        .cholesky()
        .unwrap_or_else(|| panic!("EKFUpdate: S 矩阵不正定，无法 Cholesky 分解"))
        .inverse();

    let k = &m_a * s_inv;
    // Cov ← Cov − K·M_aᵀ + 对称化（对照 C++ 的上三角技巧）
    state.cov -= &k * m_a.transpose();
    state.cov = 0.5 * (&state.cov + state.cov.transpose());

    for i in 0..state.cov.nrows() {
        assert!(
            state.cov[(i, i)] >= 0.0,
            "EKFUpdate: 对角第 {i} 项为负 ({:.2e})，协方差非半正定",
            state.cov[(i, i)]
        );
    }

    // dx = K·res，逐变量 boxplus 更新（对照 C++ 末尾循环）
    let dx = &k * res;
    state.update_all(&dx);
}

/// 设置初始协方差块（对照 `StateHelper::set_initial_covariance`）。
///
/// 假定 `covariance` 与 `order` 按同序排列，且与现有协方差的其余块不相关
/// （块对角假设，通常用于初始化时刻）。
pub fn set_initial_covariance(
    state: &mut State,
    covariance: &DMatrix<f64>,
    order: &[(i32, usize)],
) {
    let mut i_index = 0usize;
    for (i_id, i_size) in order {
        let mut k_index = 0usize;
        for (k_id, k_size) in order {
            let sub = covariance
                .view((i_index, k_index), (*i_size, *k_size))
                .into_owned();
            state
                .cov
                .view_mut((*i_id as usize, *k_id as usize), (*i_size, *k_size))
                .copy_from(&sub);
            k_index += k_size;
        }
        i_index += i_size;
    }
    state.cov = 0.5 * (&state.cov + state.cov.transpose());
}

/// 抽取测量变量的子协方差（对照 `StateHelper::get_marginal_covariance`）。
#[must_use]
pub fn get_marginal_covariance(state: &State, small_variables: &[(i32, usize)]) -> DMatrix<f64> {
    let cov_size: usize = small_variables.iter().map(|(_, s)| s).sum();
    let mut small_cov = DMatrix::zeros(cov_size, cov_size);
    let mut i_index = 0usize;
    for (i_id, i_size) in small_variables {
        let mut k_index = 0usize;
        for (k_id, k_size) in small_variables {
            let sub = state
                .cov
                .view((*i_id as usize, *k_id as usize), (*i_size, *k_size))
                .into_owned();
            small_cov
                .view_mut((i_index, k_index), (*i_size, *k_size))
                .copy_from(&sub);
            k_index += k_size;
        }
        i_index += i_size;
    }
    small_cov
}

/// 全协方差副本（对照 `StateHelper::get_full_covariance`）。
#[must_use]
pub fn get_full_covariance(state: &State) -> DMatrix<f64> {
    state.cov.clone()
}

/// 边缘化一个变量（对照 `StateHelper::marginalize`）。
///
/// 从协方差中删除 `(marg_id, marg_size)` 对应的行/列，并把其后所有变量
/// id 前移。**注意**：本函数不删除 `clones_imu` 中的条目——调用方
/// （`marginalize_old_clone`）负责。
///
/// # Panics
/// 若 `marg_id + marg_size` 超出协方差维度。
pub fn marginalize(state: &mut State, marg_id: i32, marg_size: usize) {
    let marg_id = marg_id as usize;
    let total = state.cov.nrows();
    assert!(marg_id + marg_size <= total, "marginalize: 越界");

    let x2_size = total - marg_id - marg_size;
    let mut cov_new = DMatrix::zeros(total - marg_size, total - marg_size);
    // P(x1,x1)
    cov_new
        .view_mut((0, 0), (marg_id, marg_id))
        .copy_from(&state.cov.view((0, 0), (marg_id, marg_id)));
    // P(x1,x2)
    cov_new
        .view_mut((0, marg_id), (marg_id, x2_size))
        .copy_from(&state.cov.view((0, marg_id + marg_size), (marg_id, x2_size)));
    // P(x2,x1) = P(x1,x2)ᵀ
    let p12 = cov_new.view((0, marg_id), (marg_id, x2_size)).into_owned();
    cov_new
        .view_mut((marg_id, 0), (x2_size, marg_id))
        .copy_from(&p12.transpose());
    // P(x2,x2)
    cov_new
        .view_mut((marg_id, marg_id), (x2_size, x2_size))
        .copy_from(&state.cov.view(
            (marg_id + marg_size, marg_id + marg_size),
            (x2_size, x2_size),
        ));

    state.cov = cov_new;
    state.renumber_after(marg_id as i32, marg_size);
}

/// 克隆当前 IMU 位姿到协方差末尾并加入滑动窗口（对照
/// `StateHelper::augment_clone`）。
///
/// `last_w` 为当前时刻角速度（时间偏移标定用；未标定时忽略）。
///
/// # Panics
/// 若当前 `timestamp` 与已有克隆重复，或时间偏移标定开启但状态缺少
/// `calib_dt_cam_to_imu` 变量。
pub fn augment_clone(state: &mut State, last_w: &Vector3<f64>) {
    let ts = state.timestamp;
    assert!(
        !state
            .clones_imu
            .iter()
            .any(|(t, _)| t.total_cmp(&ts).is_eq()),
        "augment_clone: 与已有克隆时间戳重复 ({ts})"
    );

    // 克隆 IMU 位姿（值 + FEJ 复制；对照 Type::clone）
    let mut pose = state.imu.pose().clone();
    let new_loc = state.cov.nrows();
    pose.set_local_id(new_loc as i32);

    // 协方差增广：复制 imu pose 的块（对照 C++ clone() 的拷贝段）
    let old_loc = state.imu.pose().id() as usize;
    let old_size = state.cov.nrows();
    let total = old_size + 6;
    let mut cov = DMatrix::zeros(total, total);
    cov.view_mut((0, 0), (old_size, old_size))
        .copy_from(&state.cov);
    cov.view_mut((new_loc, new_loc), (6, 6))
        .copy_from(&state.cov.view((old_loc, old_loc), (6, 6)));
    cov.view_mut((0, new_loc), (old_size, 6))
        .copy_from(&state.cov.view((0, old_loc), (old_size, 6)));
    cov.view_mut((new_loc, 0), (6, old_size))
        .copy_from(&state.cov.view((old_loc, 0), (6, old_size)));
    state.cov = cov;

    // 时间偏移标定：克隆块对 dt 的雅可比增广（对照 C++ 的 dnc_dt 外积）
    if state.options.do_calib_camera_timeoffset {
        let mut dnc_dt = DVector::zeros(6);
        dnc_dt.rows_range_mut(0..3).copy_from(last_w);
        dnc_dt.rows_range_mut(3..6).copy_from(&state.imu.vel());
        let dt_id = state
            .calib_dt_cam_to_imu
            .as_ref()
            .expect("time offset 标定开启时必须有 dt 变量")
            .id() as usize;
        // P[:, pose_id:pose_id+6] += P[:, dt_id] · dnc_dtᵀ
        let col = state.cov.column(dt_id).into_owned();
        let mut block_col = state.cov.view((0, new_loc), (old_size + 6, 6)).into_owned();
        block_col += &col * dnc_dt.transpose();
        state
            .cov
            .view_mut((0, new_loc), (old_size + 6, 6))
            .copy_from(&block_col);
        // P[pose_id:pose_id+6, :] += dnc_dt · P[dt_id, :]
        let row = state.cov.row(dt_id).into_owned();
        let mut block_row = state.cov.view((new_loc, 0), (6, old_size + 6)).into_owned();
        block_row += &dnc_dt * row;
        state
            .cov
            .view_mut((new_loc, 0), (6, old_size + 6))
            .copy_from(&block_row);
    }

    state.clones_imu.push_back((ts, pose));
}

/// 初始化一个新变量（对照 `StateHelper::initialize` 的增广数学）。
///
/// 用于 SLAM 特征延迟初始化：给定测量对已有状态的雅可比 `h_r`、对新
/// 特征的雅可比 `h_l`、噪声 `r` 与残差 `res`，增广特征到协方差末尾并更新
/// 其初值（`x_order` 为 `h_r` 对应的状态 `(id, size)` 列表）。
///
/// # Panics
/// `h_r`/`h_l`/`r`/`res` 维度不符、或 `H_L` 不可逆。
// 形参由跨端 API 契约逐一定死，与 C++ `StateHelper::initialize` 签名一一对应。
#[allow(clippy::too_many_arguments)]
pub fn initialize_feature(
    state: &mut State,
    mut landmark: Landmark,
    h_order: &[(i32, usize)],
    h_r: &mut DMatrix<f64>,
    h_l: &mut DMatrix<f64>,
    r: &DMatrix<f64>,
    res: &mut DVector<f64>,
    chi2_multipler: f64,
) -> bool {
    let new_var_size = landmark.size();
    debug_assert_eq!(h_l.ncols(), new_var_size);
    debug_assert_eq!(h_r.nrows(), h_l.nrows());
    debug_assert_eq!(h_r.nrows(), res.len());
    debug_assert_eq!(h_r.nrows(), r.nrows());

    //==========================================================
    // Givens QR 分离：把 H_L 上三角化（对照 C++ initialize 的 Givens 段）
    // 顶部 new_var_size 行成为可逆的特征初始化系统，底部行将全部为零
    // （对照 C++：applyOnTheLeft 对 H_L 的第 n.. 列、res 与 H_R 全列）
    //==========================================================
    for n in 0..h_l.ncols() {
        for m in ((n + 1)..h_l.nrows()).rev() {
            crate::updater_helper::givens_zero(h_l, h_r, res, m, n);
        }
    }

    // 分离成初始化部分与更新部分（对照 C++ 的 block 截取）
    let n = new_var_size;
    let hx_init = h_r.rows_range(0..n).into_owned();
    let h_f_top = h_l.rows_range(0..n).columns_range(0..n).into_owned();
    let res_init = res.rows_range(0..n).into_owned();
    let r_init = r.view((0, 0), (n, n)).into_owned();
    let h_up = h_r.rows_range(n..).into_owned();
    let res_up = res.rows_range(n..).into_owned();
    let r_up = r.view((n, n), (r.nrows() - n, r.ncols() - n)).into_owned();

    //==========================================================
    // Mahalanobis 距离检验（只对更新系统；对照 C++ initialize 的 S/chi2）
    //==========================================================
    let p_up = get_marginal_covariance(state, h_order);
    debug_assert_eq!(h_up.ncols(), p_up.ncols());
    let s = &h_up * p_up * h_up.transpose() + &r_up;
    let chi2 = match s.clone().cholesky() {
        Some(chol) => res_up.dot(&chol.solve(&res_up)),
        None => f64::INFINITY,
    };
    let chi2_check = crate::updater::chi2_95(res.len());
    if chi2 > chi2_multipler * chi2_check {
        return false;
    }

    //==========================================================
    // initialize_invertible（对照 C++：M_a / M / P_LL / 增广 / 更新值）
    //==========================================================
    let mut m_a = DMatrix::zeros(state.cov.nrows(), res_init.len());
    for (var_id, var_size) in state.variable_order() {
        let id = var_id as usize;
        let mut m_i = DMatrix::zeros(var_size, res_init.len());
        for (i, (meas_id, meas_size)) in h_order.iter().enumerate() {
            let mid = *meas_id as usize;
            let cov_sub = state.cov.view((id, mid), (var_size, *meas_size));
            let h_sub = hx_init.view((0, hx_col(i, h_order)), (hx_init.nrows(), *meas_size));
            m_i += cov_sub * h_sub.transpose();
            let _ = meas_id;
        }
        m_a.view_mut((id, 0), (var_size, res_init.len()))
            .copy_from(&m_i);
    }

    let p_small = get_marginal_covariance(state, h_order);
    let m = &hx_init * p_small * hx_init.transpose() + &r_init;
    let h_l_inv = h_f_top.clone().try_inverse().expect("H_L 应可逆");

    let p_ll = &h_l_inv * m * h_l_inv.transpose();

    // 增广协方差（对照 C++ 的 conservativeResizeLike + 三块写入）
    let old_size = state.cov.nrows();
    let total = old_size + new_var_size;
    let mut cov = DMatrix::zeros(total, total);
    cov.view_mut((0, 0), (old_size, old_size))
        .copy_from(&state.cov);
    let cross = -&m_a * h_l_inv.transpose();
    cov.view_mut((0, old_size), (old_size, new_var_size))
        .copy_from(&cross);
    cov.view_mut((old_size, 0), (new_var_size, old_size))
        .copy_from(&cross.transpose());
    cov.view_mut((old_size, old_size), (new_var_size, new_var_size))
        .copy_from(&p_ll);
    state.cov = cov;

    // 更新特征初值 + 设置 id + 登记进状态变量（对照 C++ 末尾三句）
    landmark.update(&(&h_l_inv * &res_init));
    landmark.set_local_id(old_size as i32);
    let featid = landmark.featid;
    state.features_slam.insert(featid, landmark);

    //==========================================================
    // 用更新系统做 EKF 更新（对照 C++：if(Hup.rows()>0) EKFUpdate）
    //==========================================================
    if h_up.nrows() > 0 {
        ekf_update(state, h_order, &h_up, &res_up, &r_up);
    }
    true
}

/// 边缘化所有标记 `should_marg` 的 SLAM 特征（对照
/// `StateHelper::marginalize_slam`）。
///
/// 无 aruco：所有 `should_marg` 特征均移除。从 `features_slam` 移除后调用
/// [`marginalize`] 删除其协方差块并重排其后变量 id。
#[allow(clippy::implicit_hasher)] // 键为 usize，默认 hasher 足够
pub fn marginalize_slam(state: &mut State) {
    let marg: Vec<usize> = state
        .features_slam
        .iter()
        .filter(|(_, l)| l.should_marg)
        .map(|(id, _)| *id)
        .collect();
    for featid in marg {
        if let Some(mut lm) = state.features_slam.remove(&featid) {
            let id = lm.id();
            let size = lm.size();
            lm.set_local_id(-1);
            marginalize(state, id, size);
        }
    }
}

/// 计算 `x_order` 第 `i` 个变量的起始列（`H_R` 的列布局辅助）。
fn hx_col(i: usize, x_order: &[(i32, usize)]) -> usize {
    x_order.iter().take(i).map(|(_, s)| s).sum()
}

/// 边缘化最老克隆（对照 `StateHelper::marginalize_old_clone`）。
///
/// # Panics
/// 若克隆数超过上限但窗口为空（无克隆可边缘化）。
pub fn marginalize_old_clone(state: &mut State) {
    if state.clones_imu.len() > state.options.max_clone_size {
        let marg_time = state.marg_timestep();
        assert!(marg_time >= 0.0, "marginalize_old_clone: 无克隆可边缘化");
        let (_, clone) = state.clones_imu.front().expect("克隆应存在").clone();
        let (marg_id, marg_size) = (clone.id(), clone.size());
        marginalize(state, marg_id, marg_size);
        state.clones_imu.pop_front();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::StateOptions;
    use nalgebra::Vector4;

    #[test]
    fn augment_clone_grows_covariance() {
        let mut s = State::new(StateOptions::default());
        s.timestamp = 1.0;
        s.imu.set_value(
            Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(0.1, 0.2, 0.3),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        augment_clone(&mut s, &Vector3::new(0.0, 0.0, 0.5));
        assert_eq!(s.cov.nrows(), 21);
        assert!(s.clones_imu.iter().any(|(t, _)| t.total_cmp(&1.0).is_eq()));
        let clone = &s.clones_imu[0].1;
        assert_eq!(clone.id(), 15);
        assert_eq!(clone.pos(), Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(clone.quat(), Vector4::new(0.0, 0.0, 0.0, 1.0));
        // 克隆块 = imu pose 块（本测试协方差为 1e-6 单位阵）
        assert!((s.cov[(15, 15)] - 1e-6).abs() < 1e-12);
    }

    #[test]
    fn augment_clone_with_time_offset_calibration() {
        let opts = StateOptions {
            do_calib_camera_timeoffset: true,
            ..StateOptions::default()
        };
        let mut s = State::new(opts);
        // 16 维（15 + dt）
        assert_eq!(s.cov.nrows(), 16);
        s.timestamp = 1.0;
        s.imu.set_value(
            Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::zeros(),
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let w = Vector3::new(0.1, 0.2, 0.3);
        augment_clone(&mut s, &w);
        // 克隆列 += P[:, dt]·dnc_dtᵀ：dt 与 imu 不相关（对角协方差）→ 无变化
        assert_eq!(s.cov.nrows(), 22);
        let clone = &s.clones_imu[0].1;
        assert_eq!(clone.id(), 16);
        // C++ 语义：两步外积增广（先列块 += P[:,dt]·dnc_dtᵀ，再行块 += dnc_dt·P[dt,:]，
        // 第二步使用更新后的 dt 行）→ (16,16) = 1e-6 + dnc[0]·(1e-4·dnc[0]) = 2e-6
        assert!((s.cov[(16, 16)] - 2e-6).abs() < 1e-15);
        // 交叉项 P[dt, pose] = dnc_dt[0]·P[dt,dt] = 0.1·1e-4
        assert!((s.cov[(15, 16)] - 1e-5).abs() < 1e-15);
    }

    #[test]
    fn marginalize_removes_block_and_renumbers() {
        let mut s = State::new(StateOptions::default());
        s.timestamp = 1.0;
        augment_clone(&mut s, &Vector3::zeros());
        s.timestamp = 2.0;
        augment_clone(&mut s, &Vector3::zeros());
        // 两个克隆 id = 15, 21；协方差 27×27
        assert_eq!(s.cov.nrows(), 27);
        assert_eq!(s.clones_imu[0].1.id(), 15);
        assert_eq!(s.clones_imu[1].1.id(), 21);

        // 边缘化最老克隆（id 15, size 6）
        marginalize(&mut s, 15, 6);
        assert_eq!(s.cov.nrows(), 21);
        // 剩余克隆 id 前移 6 → 15
        assert_eq!(s.clones_imu[0].1.id(), 15);
    }

    #[test]
    fn marginalize_old_clone_respects_window() {
        let opts = StateOptions {
            max_clone_size: 2,
            ..StateOptions::default()
        };
        let mut s = State::new(opts);
        for t in 1..=3 {
            s.timestamp = f64::from(t);
            augment_clone(&mut s, &Vector3::zeros());
        }
        // 3 个克隆 > 上限 2 → 边缘化最老
        marginalize_old_clone(&mut s);
        assert_eq!(s.clones_imu.len(), 2);
        assert!(!s.clones_imu.iter().any(|(t, _)| t.total_cmp(&1.0).is_eq()));
        assert_eq!(s.cov.nrows(), 15 + 2 * 6);
    }

    #[test]
    fn ekf_update_scalar_system() {
        // 标量系统：x ~ N(0,1)，测量 z = x + v（R=1），res=1
        // 后验均值 dx = K·res = 0.5；后验方差 = 1 − 1/2 = 0.5
        let mut s = State::new(StateOptions::default());
        s.cov = DMatrix::identity(15, 15); // 只关心第 0 维
        let h = DMatrix::from_row_slice(1, 1, &[1.0]);
        let res = DVector::from_element(1, 1.0);
        let r = DMatrix::from_element(1, 1, 1.0);
        let order = [(0i32, 1usize)];
        ekf_update(&mut s, &order, &h, &res, &r);
        assert!((s.cov[(0, 0)] - 0.5).abs() < 1e-9, "cov={}", s.cov[(0, 0)]);
        // imu 第 0 个误差状态（四元数 x 分量）更新：dx[0]=0.5 →
        // dq = quatnorm([0.25,0,0,1]) → q[0] = 0.25/√(1+0.0625) = 0.242535...
        let q = s.imu.quat();
        let expected = 0.25 / (1.0 + 0.0625_f64).sqrt();
        assert!((q[0] - expected).abs() < 1e-9, "q[0]={}", q[0]);
        // 无关维度协方差不变
        assert!((s.cov[(1, 1)] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn ekf_propagation_phi_identity() {
        // Φ = I, Q = 0 → 协方差不变
        let mut s = State::new(StateOptions::default());
        s.cov = DMatrix::from_diagonal(&DVector::from_vec(
            (0..15).map(|i| 1.0 + f64::from(i) * 0.1).collect(),
        ));
        let order: Vec<(i32, usize)> = vec![(0, 15)];
        let phi = DMatrix::identity(15, 15);
        let q = DMatrix::zeros(15, 15);
        let before = s.cov.clone();
        ekf_propagation(&mut s, &order, &order, &phi, &q);
        assert!((s.cov - before).norm() < 1e-12);
    }

    #[test]
    fn get_marginal_covariance_extracts_block() {
        let mut s = State::new(StateOptions::default());
        s.cov[(0, 0)] = 2.0;
        s.cov[(7, 7)] = 3.0;
        let order = [(0i32, 3usize), (6i32, 3usize)];
        let small = get_marginal_covariance(&s, &order);
        assert_eq!(small.nrows(), 6);
        assert!((small[(0, 0)] - 2.0).abs() < 1e-12);
        assert!((small[(4, 4)] - 3.0).abs() < 1e-12);
    }
}
