//! 测量雅可比测试共用的相机、克隆和像素残差夹具。

use std::sync::Arc;

use firefly_vio::options::StateOptions;
use firefly_vio::state::State;
use firefly_vio_core::cam::{CamRadtan, SharedCamera};
use firefly_vio_core::feat::Feature;
use firefly_vio_types::quat_ops::rot_2_quat;
use firefly_vio_types::var::{PoseJpl, Variable};
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

fn r_ito_c() -> Matrix3<f64> {
    firefly_base::rig::rot_ito_c()
}

/// 单相机 + 3 个克隆位姿的状态；特征在正前方 5m。
pub fn build_state() -> (State, Feature) {
    build_state_with_options(StateOptions {
        do_fej: false,
        ..StateOptions::default()
    })
}

pub fn build_state_with_options(opts: StateOptions) -> (State, Feature) {
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
    let mut st = State::new(opts);
    st.cam_intrinsics
        .get_mut(&0)
        .unwrap()
        .set_value(DVector::from_column_slice(&intrinsics));
    st.cameras.insert(0usize, cam0);

    // 外参：与 apps/vio 一致（p_IinC = R_ItoC·(−t_cam_body)）
    let r = r_ito_c();
    let q = rot_2_quat(&r);
    let p_left_in_c = firefly_base::rig::p_i_in_c(firefly_base::FrameId::LEFT_CAMERA);
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
    let lever_body = firefly_base::rig::position_in_body(firefly_base::FrameId::LEFT_CAMERA);
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
pub fn residual(st: &State, feat: &Feature) -> DVector<f64> {
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
