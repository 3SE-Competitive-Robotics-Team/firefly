//! 传感器标定常量（真值，对照 `firefly-map::DepthCamera::mujoco_default` 与
//! `firefly-mujoco/scene.py` 的 `cam_left`）。
//!
//! 左目与深度相机同朝向（下倾 20°），仅差 2.5cm 横向基线；`PnP` 解的是左目
//! 位姿，经 [`cam_pose_to_body`] 转到机体系（VIO/融合状态系）再参与融合。

use nalgebra::{Isometry3, Matrix3, Matrix4, Vector3};

/// 像素焦距（`fx=fy`，`120/tan(70.88°/2)`，与离线建库同公式）。
pub const MUJOCO_FOCAL: f64 = 168.606_993_943_649_97;
/// 左目在机体系的位置（米，`scene.py cam_left pos`）。
pub const LEFT_POS_IN_BODY: [f64; 3] = [0.0, -0.025, 0.0];

/// 相机 → 机体旋转（列 = 相机轴在机体系坐标，与 `DepthCamera` 一致）。
#[must_use]
pub fn rot_cam_to_body() -> Matrix3<f64> {
    Matrix3::new(
        0.0, 0.3420, -0.9397, //
        -1.0, 0.0, 0.0, //
        0.0, 0.9397, 0.3420,
    )
}

/// 左目位姿 → 机体位姿（`T_global_body = T_global_cam · T_body_cam`，
/// 其中 `T_body_cam` 为 body→cam，由 cam→body 外参求逆得到）。
#[must_use]
pub fn cam_pose_to_body(t_cam: &Matrix4<f64>) -> Matrix4<f64> {
    let r = rot_cam_to_body();
    let p = Vector3::new(
        LEFT_POS_IN_BODY[0],
        LEFT_POS_IN_BODY[1],
        LEFT_POS_IN_BODY[2],
    );
    let t_cb = Isometry3::from_parts(
        nalgebra::Translation3::from(p),
        nalgebra::UnitQuaternion::from_matrix(&r),
    )
    .to_homogeneous();
    let t_bc = t_cb.try_inverse().unwrap_or(Matrix4::identity());
    t_cam * t_bc
}

/// 机体位姿 → 左目位姿（建库反投影/`PnP` 初值用：`T_global_cam = T_global_body · T_cam_body`，
/// 直接右乘——必须右乘：求逆版整体错位）。
#[must_use]
pub fn body_pose_to_cam(t_body: &Matrix4<f64>) -> Matrix4<f64> {
    let t_cb = {
        let r = rot_cam_to_body();
        let p = Vector3::new(
            LEFT_POS_IN_BODY[0],
            LEFT_POS_IN_BODY[1],
            LEFT_POS_IN_BODY[2],
        );
        Isometry3::from_parts(
            nalgebra::Translation3::from(p),
            nalgebra::UnitQuaternion::from_matrix(&r),
        )
        .to_homogeneous()
    };
    t_body * t_cb
}

/// 深度验证：`rot_cam_to_body` 列向量即相机轴（与 `DepthCamera` 文档一致）。
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cam_axes_match_depth_camera() {
        let r = rot_cam_to_body();
        let x = r.column(0);
        assert!((x.x).abs() < 1e-12 && (x.y + 1.0).abs() < 1e-12 && x.z.abs() < 1e-12);
    }

    #[test]
    fn body_cam_extrinsics_match_mujoco_truth() {
        use nalgebra::Matrix4;
        // 外参真值断言（`scene.py cam_left`）：`T_body @ T_cb` 必须等于下式，
        // 求逆版整体错位——`body_pose_to_cam` 必须直接右乘。
        let t_body = Isometry3::from_parts(
            nalgebra::Translation3::new(1.0, 4.0, 1.0),
            nalgebra::UnitQuaternion::identity(),
        )
        .to_homogeneous();
        let t_wcam_expected = Matrix4::new(
            0.0, 0.3420, -0.9397, 1.0, //
            -1.0, 0.0, 0.0, 3.975, //
            0.0, 0.9397, 0.3420, 1.0, //
            0.0, 0.0, 0.0, 1.0,
        );
        let back = body_pose_to_cam(&t_body);
        assert!((back - t_wcam_expected).norm() < 1e-3);
    }

    #[test]
    fn focal_matches_formula() {
        let f = 120.0 / (70.88_f64 / 2.0).to_radians().tan();
        assert!((f - MUJOCO_FOCAL).abs() < 1e-9);
    }
}
