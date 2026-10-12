//! 数值验证 MSCKF 测量雅可比（有限差分对照解析 `H_x`）。
//!
//! `do_fej=false` 的解析雅可比通过 boxplus 中心差分验证；
//! FEJ 的锚定表示通过同锚点自观测不变性验证。

mod support;

use firefly_vio::options::{FeatRepresentation, StateOptions};
use firefly_vio::state::State;
use firefly_vio_core::cam::{CamRadtan, SharedCamera};
use firefly_vio_types::var::Variable;
use nalgebra::{DVector, Vector3};
use std::sync::Arc;
use support::{build_state, build_state_with_options, residual};

#[test]
fn msckf_jacobian_matches_finite_difference() {
    let (state, feature) = build_state();
    let jac = firefly_vio::updater_helper::get_feature_jacobian_full(
        &state,
        &feature,
        FeatRepresentation::Global3D,
    );
    assert_eq!(jac.res.len(), 6);
    assert_eq!(jac.x_order.len(), 3);
    assert!((residual(&state, &feature) - &jac.res).amax() < 1e-9);
    let eps = 1e-2;
    let mut column = 0;
    for (clone_index, &(id, size)) in jac.x_order.iter().enumerate() {
        for component in 0..size {
            let mut dx = DVector::zeros(size);
            dx[component] = eps;
            let mut plus = state.clone();
            plus.clones_imu[clone_index].1.update(&dx);
            let mut minus = state.clone();
            minus.clones_imu[clone_index].1.update(&(-dx));
            let fd = -(residual(&plus, &feature) - residual(&minus, &feature)) / (2. * eps);
            for row in 0..fd.len() {
                let analytic = jac.h_x[(row, column + component)];
                let relative =
                    (analytic - fd[row]).abs() / analytic.abs().max(fd[row].abs()).max(1e-6);
                // f32 投影量化与差分截断误差；与测量噪声、飞行阈值无关。
                assert!(
                    relative < 0.01,
                    "clone={id}, component={component}, row={row}: analytic={analytic}, fd={}",
                    fd[row]
                );
            }
        }
        column += size;
    }
}

/// SLAM `H_f`（landmark 列）FD 验证：扰动 landmark 位置分量，残差中心差分
/// 对照解析 `h_f`。MSCKF 的 `H_x` 已由 `msckf_jacobian_matches_finite_difference`
/// 验证；SLAM 更新保留 `H_f`，其符号和尺度必须与像素残差一致。
#[test]
fn slam_h_f_matches_finite_difference() {
    use firefly_vio::landmark::Landmark;
    let (st, feat0) = build_state();

    // 特征作为 SLAM landmark（Global3D）加入状态
    let mut st = st;
    let mut lm = Landmark::new(FeatRepresentation::Global3D, feat0.featid);
    lm.set_from_xyz(&feat0.p_FinG, false);
    lm.set_from_xyz(&feat0.p_FinG, true);
    st.features_slam.insert(feat0.featid, lm);
    // landmark id 从 15（imu 后）开始分配（克隆 3 个 6 维 → 15+18=33）
    st.features_slam
        .get_mut(&feat0.featid)
        .unwrap()
        .set_local_id(33);

    // SLAM 路径：feat.p_FinG 从 landmark 取
    let feat = feat0.clone();
    let jac = firefly_vio::updater_helper::get_feature_jacobian_full(
        &st,
        &feat,
        FeatRepresentation::Global3D,
    );
    // x_order 不含 landmark（jac 只返回 H_f，landmark 列由 UpdaterSLAM
    // 手动追加）——本测试只验证 h_f 与 FD 的一致性。

    let eps = 1e-3_f64;
    let mut worst_rel = 0.0f64;
    for comp in 0..3 {
        let mut stp = st.clone();
        let mut lmp = stp.features_slam.get_mut(&feat0.featid).unwrap().clone();
        let mut p = lmp.get_xyz(false);
        p[comp] += eps;
        lmp.set_from_xyz(&p, false);
        stp.features_slam.insert(feat0.featid, lmp);
        let mut feat_plus = feat.clone();
        feat_plus.p_FinG = stp.features_slam[&feat0.featid].get_xyz(false);
        let rp = residual(&stp, &feat_plus);

        let mut stm = st.clone();
        let mut lmm = stm.features_slam.get_mut(&feat0.featid).unwrap().clone();
        let mut p = lmm.get_xyz(false);
        p[comp] -= eps;
        lmm.set_from_xyz(&p, false);
        stm.features_slam.insert(feat0.featid, lmm);
        let mut feat_minus = feat.clone();
        feat_minus.p_FinG = stm.features_slam[&feat0.featid].get_xyz(false);
        let rm = residual(&stm, &feat_minus);

        let fd = -(rp - rm) / (2.0 * eps);
        for r in 0..fd.len() {
            let a = jac.h_f[(r, comp)];
            let f = fd[r];
            let diff = (a - f).abs();
            let rel = diff / a.abs().max(f.abs()).max(1e-6);
            worst_rel = worst_rel.max(rel);
        }
    }
    assert!(
        worst_rel < 0.05,
        "SLAM H_f 与 FD 失配（最差相对误差 {worst_rel:.2e}）——landmark 列符号/尺度错误"
    );
}

#[test]
fn camera_calibration_jacobians_match_finite_difference() {
    use firefly_vio::updater_helper::get_feature_jacobian_full;
    use firefly_vio_core::cam::CamEqui;
    for fisheye in [false, true] {
        let (mut state, feat) = build_state_with_options(StateOptions {
            do_fej: false,
            do_calib_camera_pose: true,
            do_calib_camera_intrinsics: true,
            ..StateOptions::default()
        });
        let intrinsics = [168.6, 170.2, 160.0, 120.0, 0.01, -0.003, 0.001, -0.001];
        let camera: SharedCamera = if fisheye {
            Arc::new(CamEqui::new(320, 240, &intrinsics))
        } else {
            Arc::new(CamRadtan::new(320, 240, &intrinsics))
        };
        state.cameras.insert(0, camera);
        state
            .cam_intrinsics
            .get_mut(&0)
            .unwrap()
            .set_value(DVector::from_column_slice(&intrinsics));
        let jac = get_feature_jacobian_full(&state, &feat, FeatRepresentation::Global3D);
        let ids = [
            state.calib_imu_to_cam[&0].id(),
            state.cam_intrinsics[&0].id(),
        ];
        let mut column = 0;
        for &(id, size) in &jac.x_order {
            if ids.contains(&id) {
                for component in 0..size {
                    let mut dx = DVector::zeros(state.cov.nrows());
                    let eps = 1e-3;
                    dx[id as usize + component] = eps;
                    let mut plus = state.clone();
                    plus.update_all(&dx);
                    let mut minus = state.clone();
                    minus.update_all(&(-dx));
                    let fd = -(residual(&plus, &feat) - residual(&minus, &feat)) / (2.0 * eps);
                    let error = (&fd - jac.h_x.column(column + component)).amax();
                    assert!(
                        error < 0.02,
                        "fisheye={fisheye}, id={id}, component={component}, error={error}"
                    );
                }
            }
            column += size;
        }
    }
}

#[test]
fn anchored_self_observation_is_independent_of_anchor_pose() {
    use firefly_vio::updater_helper::get_feature_jacobian_full;
    for representation in [
        FeatRepresentation::Anchored3D,
        FeatRepresentation::AnchoredFullInverseDepth,
        FeatRepresentation::AnchoredMsckfInverseDepth,
        FeatRepresentation::AnchoredInverseDepthSingle,
    ] {
        for fej in [false, true] {
            let (mut state, mut feat) = build_state_with_options(StateOptions {
                do_fej: fej,
                do_calib_camera_pose: true,
                ..StateOptions::default()
            });
            state.clones_imu[0].1.update(&DVector::from_column_slice(&[
                0.15, -0.2, 0.1, 0.3, -0.1, 0.2,
            ]));
            let anchor = &state.clones_imu[0].1;
            let calib = &state.calib_imu_to_cam[&0];
            feat.anchor_cam_id = 0;
            feat.anchor_clone_timestamp = 1.0;
            feat.p_FinA = calib.rot() * anchor.rot() * (feat.p_FinG - anchor.pos()) + calib.pos();
            feat.timestamps.get_mut(&0).unwrap().truncate(1);
            feat.uvs.get_mut(&0).unwrap().truncate(1);
            feat.uvs_norm.get_mut(&0).unwrap().truncate(1);
            let jac = get_feature_jacobian_full(&state, &feat, representation);
            assert!(
                jac.h_x.norm() < 1e-9,
                "{representation:?}, fej={fej}, H={}",
                jac.h_x
            );
            assert!(jac.h_f.iter().all(|x| x.is_finite()));
        }
    }
}

#[test]
fn fej_extrinsics_use_frozen_geometry_and_current_residual() {
    use firefly_vio::updater_helper::get_feature_jacobian_full;
    use firefly_vio_types::quat_ops::skew_x;
    let (mut state, mut feat) = build_state_with_options(StateOptions {
        do_fej: true,
        do_calib_camera_pose: true,
        ..StateOptions::default()
    });
    feat.p_FinG_fej = feat.p_FinG + Vector3::new(0.2, -0.1, 0.15);
    for (_, clone) in &mut state.clones_imu {
        clone.update(&DVector::from_column_slice(&[
            0.08, -0.04, 0.02, 0.1, 0.2, -0.1,
        ]));
    }
    let jac = get_feature_jacobian_full(&state, &feat, FeatRepresentation::Global3D);
    assert!((&jac.res - residual(&state, &feat)).amax() < 1e-12);
    let calib = &state.calib_imu_to_cam[&0];
    let col: usize = jac
        .x_order
        .iter()
        .take_while(|(id, _)| *id != calib.id())
        .map(|(_, n)| n)
        .sum();
    let mut frozen = state.clone();
    for (_, clone) in &mut frozen.clones_imu {
        clone.set_value(clone.quat_fej(), clone.pos_fej());
    }
    let project = |s: &State| {
        let c = &s.calib_imu_to_cam[&0];
        DVector::from_iterator(
            6,
            s.clones_imu.iter().flat_map(|(_, clone)| {
                let p = c.rot() * clone.rot() * (feat.p_FinG_fej - clone.pos()) + c.pos();
                [
                    168.606_993_943_65 * p.x / p.z + 160.0,
                    168.606_993_943_65 * p.y / p.z + 120.0,
                ]
            }),
        )
    };
    for (m, (_, clone)) in frozen.clones_imu.iter().enumerate() {
        let p = calib.rot() * clone.rot() * (feat.p_FinG_fej - clone.pos()) + calib.pos();
        let projection = nalgebra::Matrix2x3::new(
            168.606_993_943_65 / p.z,
            0.0,
            -168.606_993_943_65 * p.x / p.z.powi(2),
            0.0,
            168.606_993_943_65 / p.z,
            -168.606_993_943_65 * p.y / p.z.powi(2),
        );
        let expected = projection * skew_x(&(p - calib.pos()));
        assert!((jac.h_x.view((2 * m, col), (2, 3)) - expected).amax() < 1e-10);
    }
    // FEJ 的差分必须固定克隆和路标在线性化点；残差仍在当前状态计算。
    for component in 0..6 {
        let mut dx = DVector::zeros(6);
        let eps = 1e-5;
        dx[component] = eps;
        let mut plus = frozen.clone();
        plus.calib_imu_to_cam.get_mut(&0).unwrap().update(&dx);
        let mut minus = frozen.clone();
        minus.calib_imu_to_cam.get_mut(&0).unwrap().update(&(-dx));
        let fd = (project(&plus) - project(&minus)) / (2.0 * eps);
        assert!((&fd - jac.h_x.column(col + component)).amax() < 1e-6);
    }
}
