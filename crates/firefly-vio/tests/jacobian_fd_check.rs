//! 数值验证 MSCKF 测量雅可比（有限差分对照解析 `H_x`）。
//!
//! 目的：排查"含零偏发散"疑似翻译缺陷——H 的符号/约定与
//! `Variable::update` boxplus 约定不一致时，小残差下修正≈0 被掩盖，
//! 大残差（有零偏）下错误修正被放大成发散。
//!
//! 方法：`do_fej=false` 时 res 与 H 都用当前值——对每个状态分量做中心
//! 差分（扰动走 `Variable::update` 同款 boxplus），数值导数必须与解析
//! 雅可比逐项吻合。

use std::sync::Arc;

use firefly_vio::options::{FeatRepresentation, StateOptions};
use firefly_vio::state::State;
use firefly_vio_core::cam::{CamRadtan, SharedCamera};
use firefly_vio_core::feat::Feature;
use firefly_vio_types::quat_ops::rot_2_quat;
use firefly_vio_types::var::{PoseJpl, Variable};
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

fn r_ito_c() -> Matrix3<f64> {
    Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0)
}

/// 单相机 + 3 个克隆位姿的状态；特征在正前方 5m。
fn build_state() -> (State, Feature) {
    let intrinsics = [
        168.606_993_943_65,
        168.606_993_943_65,
        160.0,
        120.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ];
    let cam0: SharedCamera = Arc::new(CamRadtan::new(320, 240, &intrinsics));
    let opts = StateOptions {
        do_fej: false,
        num_cameras: 1,
        ..StateOptions::default()
    };
    let mut st = State::new(opts);
    st.cameras.insert(0usize, cam0);

    // 外参：与 apps/vio 一致（p_IinC = R_ItoC·(−t_cam_body)）
    let r = r_ito_c();
    let q = rot_2_quat(&r);
    let p_left_in_c = r * Vector3::new(0.0, 0.025, 0.0);
    let c0 = st.calib_imu_to_cam.get_mut(&0usize).unwrap();
    c0.set_value(q, p_left_in_c);
    c0.set_fej(q, p_left_in_c);

    // IMU 初始：t=1，pos=(1,0,1)，恒速 (1,0,0)，单位姿态
    st.timestamp = 1.0;
    st.imu.set_value(
        nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::zeros(),
        Vector3::zeros(),
    );
    st.imu.set_fej(
        nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::zeros(),
        Vector3::zeros(),
    );

    // 增广克隆 t=1、t=2、t=3（协方差块拷贝同 StateHelper::clone）
    for t in [1.0f64, 2.0, 3.0] {
        let old = st.cov.nrows();
        let mut pose = PoseJpl::default();
        pose.set_local_id(old as i32);
        pose.set_value(
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::new(t, 0.0, 1.0),
        );
        pose.set_fej(
            nalgebra::Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::new(t, 0.0, 1.0),
        );
        let mut nc = DMatrix::zeros(old + 6, old + 6);
        nc.view_mut((0, 0), (old, old)).copy_from(&st.cov);
        nc.view_mut((old, old), (6, 6))
            .copy_from(&st.cov.view((0, 0), (6, 6)));
        nc.view_mut((0, old), (old, 6))
            .copy_from(&st.cov.view((0, 0), (old, 6)));
        nc.view_mut((old, 0), (6, old))
            .copy_from(&st.cov.view((0, 0), (6, old)));
        st.cov = nc;
        st.clones_imu.push_back((t, pose));
    }

    // 特征：正前方，被 3 个克隆 × 左目观测
    let p_fg = Vector3::new(8.0, -0.9, 1.2);
    let mut feat = Feature {
        featid: 1,
        p_FinG: p_fg,
        ..Feature::default()
    };
    let lever_body = Vector3::new(0.0, -0.025, 0.0); // 左目
    for t in [1.0f64, 2.0, 3.0] {
        let p_body = p_fg - Vector3::new(t, 0.0, 1.0);
        let pc = r * (p_body - lever_body);
        assert!(pc.z > 0.5);
        let u = (168.606_993_943_65 * pc.x / pc.z + 160.0) as f32;
        let v = (168.606_993_943_65 * pc.y / pc.z + 120.0) as f32;
        feat.timestamps.entry(0usize).or_default().push(t);
        feat.uvs
            .entry(0usize)
            .or_default()
            .push(nalgebra::Vector2::new(u, v));
        feat.uvs_norm
            .entry(0usize)
            .or_default()
            .push(nalgebra::Vector2::new(
                (pc.x / pc.z) as f32,
                (pc.y / pc.z) as f32,
            ));
    }
    (st, feat)
}

/// 按实现同款公式计算残差（单相机，行序=时间升序）。
fn residual(st: &State, feat: &Feature) -> DVector<f64> {
    let calib = st.calib_imu_to_cam.get(&0usize).unwrap();
    let cam = st.cameras.get(&0usize).unwrap();
    let r_ito_c = calib.rot();
    let p_iin_c = calib.pos();
    let times = &feat.timestamps[&0usize];
    let mut res = DVector::zeros(2 * times.len());
    for (m, t) in times.iter().enumerate() {
        let clone = &st
            .clones_imu
            .iter()
            .find(|(ct, _)| ct.total_cmp(t).is_eq())
            .unwrap()
            .1;
        let p_fin_ii = clone.rot() * (feat.p_FinG - clone.pos());
        let p_in_cam = r_ito_c * p_fin_ii + p_iin_c;
        let uv_norm = nalgebra::Vector2::new(p_in_cam.x / p_in_cam.z, p_in_cam.y / p_in_cam.z);
        let uv_dist = cam.distort_d(uv_norm.cast());
        let uv_m = feat.uvs[&0usize][m];
        res[2 * m] = f64::from(uv_m.x) - uv_dist.x;
        res[2 * m + 1] = f64::from(uv_m.y) - uv_dist.y;
    }
    res
}

#[test]
fn msckf_jacobian_matches_finite_difference() {
    firefly_observability::init();
    let (st, feat) = build_state();

    let fj = firefly_vio::updater_helper::get_feature_jacobian_full(
        &st,
        &feat,
        FeatRepresentation::Global3D,
    );
    assert_eq!(fj.res.len(), 6, "应有 3 克隆 × 2 行");
    assert_eq!(
        fj.x_order.len(),
        3,
        "应为 克隆×3（do_calib_camera_pose 默认关闭）"
    );

    // Sanity：测试残差函数与实现基线残差必须一致
    let my_res = residual(&st, &feat);
    let base_diff = (&my_res - &fj.res).abs().max();
    println!("基线残差最大差 = {base_diff:.3e}");
    println!("实现 res  = {}", fj.res.transpose());
    println!("测试 res  = {}", my_res.transpose());
    assert!(base_diff < 1e-9, "测试残差函数与实现不一致!");

    let eps = 1e-2_f64;
    let mut worst = 0.0f64; // 最大绝对差
    let mut worst_rel = 0.0f64; // 最大相对差
    let mut worst_at = String::new();

    for (vi, &(id, size)) in fj.x_order.iter().enumerate() {
        for comp in 0..size {
            // +ε / −ε：按序直接扰动第 vi 个克隆（Global3D 的 x_order 只含克隆）
            let mut dxt = DVector::<f64>::zeros(size);
            dxt[comp] = eps;
            let mut stp = st.clone();
            stp.clones_imu.get_mut(vi).unwrap().1.update(&dxt);
            let rp = residual(&stp, &feat);
            let mut dxm = DVector::<f64>::zeros(size);
            dxm[comp] = -eps;
            let mut stm = st.clone();
            stm.clones_imu.get_mut(vi).unwrap().1.update(&dxm);
            let rm = residual(&stm, &feat);

            // 残差约定 res = 测量 − 预测，故 H = −∂res/∂x
            let fd = -(rp - rm) / (2.0 * eps);

            // 对照解析块列
            let col0: usize = fj.x_order.iter().take(vi).map(|(_, s)| s).sum();
            println!("--- 变量块 #{vi} (id={id}, size={size}) comp={comp} ---");
            let mut block_has_mismatch = false;
            let mut detail = Vec::new();
            for r in 0..fd.len() {
                let a = fj.h_x[(r, col0 + comp)];
                let f = fd[r];
                let diff = (a - f).abs();
                let rel = if a.abs().max(f.abs()) > 1e-8 {
                    diff / a.abs().max(f.abs())
                } else {
                    0.0
                };
                if rel > 0.05 && diff > 1e-4 {
                    block_has_mismatch = true;
                    detail.push(format!(
                        "    row{r} u/v分量{} 解析={a:+.4e} 数值={f:+.4e}",
                        r % 2
                    ));
                }
                let denom = a.abs().max(f.abs()).max(1e-6);
                let rel = diff / denom;
                if rel > worst_rel {
                    worst_rel = rel;
                    worst = diff;
                    worst_at = format!(
                        "var#{vi}(id={id}) comp={comp} row={r} 解析={a:.5e} 数值={:.5e}",
                        fd[r]
                    );
                }
            }
            if block_has_mismatch {
                println!("  comp={comp}:");
                for d in &detail {
                    println!("{d}");
                }
            }
        }
    }

    println!("最大 |解析−有限差分| = {worst:.3e}（相对 {worst_rel:.2e}）@ {worst_at}");
    // 容差：f32 测量量化 + 中心差分噪声（相对 <1%）
    assert!(
        worst_rel < 0.01,
        "MSCKF 测量雅可比与有限差分不一致: {worst_at}"
    );
}
