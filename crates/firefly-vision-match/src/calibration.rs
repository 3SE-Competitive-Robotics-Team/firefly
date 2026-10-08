//! 传感器标定常量（真值，对照 `firefly-map::DepthCamera::mujoco_default` 与
//! `apps/render/src/rig.rs` 的左目安装位置）。
//!
//! 左目与深度相机同朝向（下倾 20°），仅差 2.5cm 横向基线；`PnP` 解的是左目
//! 位姿，经 [`cam_pose_to_body`] 转到机体系（VIO/融合状态系）再参与融合。

use firefly_base::{FrameId, RigidTransform};
use nalgebra::{Matrix3, Matrix4};

/// 像素焦距（`fx=fy`）：`(H/2)/tan(FOV_Y/2)`，由 [`firefly_base::rig::FOV_Y_DEG`]
/// 派生，不再单独维护字面量。
#[must_use]
pub fn focal() -> f64 {
    // 图像高 240 px（与 render/发布一致）。
    (240.0 / 2.0) / (firefly_base::rig::FOV_Y_DEG.to_radians() / 2.0).tan()
}

/// 左目在机体系的位置（米）；来源 [`firefly_base::rig::LEFT_IN_BODY`]。
pub const LEFT_POS_IN_BODY: [f64; 3] = firefly_base::rig::LEFT_IN_BODY;

/// 相机 → 机体旋转（列 = 相机轴在机体系）；来源 [`firefly_base::rig`]。
#[must_use]
pub fn rot_cam_to_body() -> Matrix3<f64> {
    firefly_base::rig::cam_axes_in_body()
}

/// 标定外参 `body←left_camera`；来源 [`firefly_base::rig`]。
///
/// # Panics
/// 编译时标定常量不是有限刚体变换。
#[must_use]
pub fn body_from_left_camera() -> RigidTransform {
    firefly_base::rig::body_from_camera(FrameId::LEFT_CAMERA)
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

/// 左目←前置深度相机，两者同朝向（仅差 2.5cm 横向基线）；来源 [`firefly_base::rig`]。
///
/// # Panics
/// 编译时标定常量不是有限刚体变换。
#[must_use]
pub fn left_from_depth() -> RigidTransform {
    firefly_base::rig::body_from_camera(FrameId::LEFT_CAMERA)
        .inverse()
        .compose(&firefly_base::rig::body_from_camera(FrameId::DEPTH_CAMERA))
        .expect("left-body-depth chain")
}

/// RMUC 图像分辨率与针孔内参。
#[must_use]
pub fn pinhole() -> crate::depth::Pinhole {
    crate::depth::Pinhole {
        width: 320,
        height: 240,
        focal: focal(),
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
        // 钉住派生值：FOV_Y=70.88°、H=240 时 focal ≈ 168.60699394364997，
        // 与离线建库/发布一致；改动 `rig::FOV_Y_DEG` 会在此处暴露。
        assert!((focal() - 168.606_993_943_649_97).abs() < 1e-9);
    }
}
