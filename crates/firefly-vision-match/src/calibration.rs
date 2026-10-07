//! 传感器标定常量（真值，对照 `firefly-map::DepthCamera::mujoco_default` 与
//! `apps/render/src/rig.rs` 的左目安装位置）。
//!
//! 左目与深度相机同朝向（下倾 20°），仅差 2.5cm 横向基线；`PnP` 解的是左目
//! 位姿，经 [`cam_pose_to_body`] 转到机体系（VIO/融合状态系）再参与融合。

use firefly_base::{FrameId, RigidTransform};
use nalgebra::{Matrix3, Matrix4};

/// 像素焦距（`fx=fy`，`120/tan(70.88°/2)`，与离线建库同公式）。
pub const MUJOCO_FOCAL: f64 = 168.606_993_943_649_97;
/// 左目在机体系的位置（米，`render::rig::LEFT_OFFSET`：机头最前方）。
pub const LEFT_POS_IN_BODY: [f64; 3] = [0.06, -0.025, 0.0];

/// 相机 → 机体旋转（列 = 相机轴在机体系坐标，与 `DepthCamera` 一致）。
#[must_use]
pub fn rot_cam_to_body() -> Matrix3<f64> {
    Matrix3::new(
        0.0, 0.3420, -0.9397, //
        -1.0, 0.0, 0.0, //
        0.0, 0.9397, 0.3420,
    )
}

/// 标定外参 `body←left_camera`；相机轴为右、上、后（光轴 -Z）。
/// # Panics
/// 编译时标定常量不是有限刚体变换。
#[must_use]
pub fn body_from_left_camera() -> RigidTransform {
    let rotation = nalgebra::UnitQuaternion::from_matrix(&rot_cam_to_body());
    let q = rotation.quaternion();
    RigidTransform::from_parts(
        FrameId::BODY,
        FrameId::LEFT_CAMERA,
        LEFT_POS_IN_BODY,
        [q.i, q.j, q.k, q.w],
    )
    .expect("finite unit camera calibration")
}

/// `T_map_body = T_map_camera T_body_camera⁻¹`。
/// # Panics
/// 输入不是有效 SE(3) 位姿。
#[must_use]
pub fn cam_pose_to_body(t_cam: &Matrix4<f64>) -> Matrix4<f64> {
    RigidTransform::from_matrix(FrameId::MAP, FrameId::LEFT_CAMERA, t_cam)
        .expect("valid camera pose")
        .compose(&body_from_left_camera().inverse())
        .expect("map-camera-body chain")
        .matrix()
}

/// `T_map_camera = T_map_body T_body_camera`。
/// # Panics
/// 输入不是有效 SE(3) 位姿。
#[must_use]
pub fn body_pose_to_cam(t_body: &Matrix4<f64>) -> Matrix4<f64> {
    RigidTransform::from_matrix(FrameId::MAP, FrameId::BODY, t_body)
        .expect("valid body pose")
        .compose(&body_from_left_camera())
        .expect("map-body-camera chain")
        .matrix()
}

/// 左目←前置深度相机，两者具有相同安装朝向（仅差 2.5cm 横向基线）。
/// # Panics
/// 编译时标定常量不是有限刚体变换。
#[must_use]
pub fn left_from_depth() -> RigidTransform {
    let body_left = body_from_left_camera();
    let q = body_left.isometry().rotation.quaternion();
    let body_depth = RigidTransform::from_parts(
        FrameId::BODY,
        FrameId::DEPTH_CAMERA,
        [0.06, 0.0, 0.0],
        [q.i, q.j, q.k, q.w],
    )
    .expect("depth camera calibration");
    body_left
        .inverse()
        .compose(&body_depth)
        .expect("left-body-depth chain")
}

/// RMUC 图像分辨率与针孔内参。
#[must_use]
pub fn pinhole() -> crate::depth::Pinhole {
    crate::depth::Pinhole {
        width: 320,
        height: 240,
        focal: MUJOCO_FOCAL,
        cx: 160.,
        cy: 120.,
    }
}

/// 深度验证：`rot_cam_to_body` 列向量即相机轴（与 `DepthCamera` 文档一致）。
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;

    #[test]
    fn cam_axes_match_depth_camera() {
        let r = rot_cam_to_body();
        let x = r.column(0);
        assert!((x.x).abs() < 1e-12 && (x.y + 1.0).abs() < 1e-12 && x.z.abs() < 1e-12);
    }

    #[test]
    fn body_cam_extrinsics_match_mujoco_truth() {
        use nalgebra::Matrix4;
        // 外参真值断言（`render::rig::LEFT_OFFSET`）：`T_body @ T_cb` 必须等于下式，
        // 求逆版整体错位——`body_pose_to_cam` 必须直接右乘。
        let t_body = Isometry3::from_parts(
            nalgebra::Translation3::new(1.0, 4.0, 1.0),
            nalgebra::UnitQuaternion::identity(),
        )
        .to_homogeneous();
        let t_wcam_expected = Matrix4::new(
            0.0, 0.3420, -0.9397, 1.06, //
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
