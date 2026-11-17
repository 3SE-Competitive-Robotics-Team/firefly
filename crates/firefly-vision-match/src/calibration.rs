//! 传感器标定常量（真值，来源为 `firefly_base::rig` 的相机安装参数）。
//!
//! 左目与深度相机同朝向；`PnP` 解的是左目
//! 位姿，经 [`cam_pose_to_body`] 转到机体系（VIO/融合状态系）再参与融合。

use firefly_base::{FrameId, RigidTransform};
use nalgebra::{Matrix3, Matrix4};

/// 像素焦距（`fx=fy`），由 [`firefly_base::rig::FOV_Y_DEG`] 和图像高度推导，单位 px。
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

/// 左目←前置深度相机；来源 [`firefly_base::rig`]。
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
    let [cx, cy] = firefly_base::rig::principal_point(320, 240);
    crate::depth::Pinhole {
        width: 320,
        height: 240,
        focal: focal(),
        cx,
        cy,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Isometry3;

    #[test]
    fn body_cam_extrinsics_match_mujoco_truth() {
        use nalgebra::Matrix4;
        // 机体无旋转时，世界相机位姿由标称下倾外参与机体平移直接组成。
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
        assert!((focal() - 168.606_993_943_649_97).abs() < 1e-9);
        let camera = pinhole();
        assert!((camera.cx - 159.5).abs() < 1e-12 && (camera.cy - 119.5).abs() < 1e-12);
    }
}
