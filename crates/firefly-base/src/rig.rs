//! RMUC 传感器 rig 几何的**唯一来源**：机体 ↔ 左目 ↔ 右目 ↔ 深度相机。
//!
//! `render`（成像）、`firefly-map`（深度射线）、`firefly-vision-match`（库图定位）
//! 与 `apps/vio`（估计器外参）全部从这里派生，不得再各自复制数值。
//!
//! # 约定
//! - 机体 `BODY`：`+x` 前、`+y` 左、`+z` 上（与 `MuJoCo` 一致）。
//! - 相机（本模块 canonical 约定，与 `render`/`firefly-map` 一致）：`+x` 图右、
//!   `+y` 图上、光轴为 `−z`；图右对应机体 `−y`，光轴下倾 [`DOWNTILT_DEG`]。
//! - 视觉/估计器使用的 CV 光学约定（`+x` 右、`+y` 下、`+z` 前）与之相差
//!   [`CV_FROM_RENDER`]（绕 x 轴 180°）。

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use crate::frames::{FrameId, FrameTree, RigidTransform};

/// 相机下倾角（度）。
pub const DOWNTILT_DEG: f64 = 20.0;
/// 垂直视场（度）。
pub const FOV_Y_DEG: f64 = 70.88;
/// 双目基线（米，沿机体 y）。
pub const BASELINE: f64 = 0.05;
/// 双目/深度机位前移量（米，机头最前方）。
pub const CAMERA_FORWARD_M: f64 = 0.06;
/// 左目在机体系位置（米）。
pub const LEFT_IN_BODY: [f64; 3] = [CAMERA_FORWARD_M, -BASELINE / 2.0, 0.0];
/// 右目在机体系位置（米）。
pub const RIGHT_IN_BODY: [f64; 3] = [CAMERA_FORWARD_M, BASELINE / 2.0, 0.0];
/// 深度相机在机体系位置（米，与双目前脸齐平）。
pub const DEPTH_IN_BODY: [f64; 3] = [CAMERA_FORWARD_M, 0.0, 0.0];

/// rig 的三个相机坐标系。
pub const CAMERAS: [FrameId; 3] = [
    FrameId::LEFT_CAMERA,
    FrameId::RIGHT_CAMERA,
    FrameId::DEPTH_CAMERA,
];

/// CV 光学约定 ← 相机约定（绕 x 轴 180°）。
pub const CV_FROM_RENDER: Matrix3<f64> = Matrix3::new(
    1.0, 0.0, 0.0, //
    0.0, -1.0, 0.0, //
    0.0, 0.0, -1.0,
);

/// 相机各轴在机体系（列 = 轴）：图右 `−y`、图上（下倾后）、光轴 `−z`。
#[must_use]
pub fn cam_axes_in_body() -> Matrix3<f64> {
    let tilt = DOWNTILT_DEG.to_radians();
    let x = Vector3::new(0.0, -1.0, 0.0);
    let y = Vector3::new(tilt.sin(), 0.0, tilt.cos());
    let z = x.cross(&y);
    Matrix3::from_columns(&[x, y, z])
}

/// 相机在机体系的位置（米）。
///
/// # Panics
/// `camera` 不是 rig 相机坐标系。
#[must_use]
pub fn position_in_body(camera: FrameId) -> Vector3<f64> {
    let p = if camera == FrameId::LEFT_CAMERA {
        LEFT_IN_BODY
    } else if camera == FrameId::RIGHT_CAMERA {
        RIGHT_IN_BODY
    } else if camera == FrameId::DEPTH_CAMERA {
        DEPTH_IN_BODY
    } else {
        panic!("not a rig camera frame");
    };
    Vector3::from(p)
}

/// `T_body_camera`：相机在机体系的位姿。
///
/// # Panics
/// `camera` 不是 rig 相机坐标系。
#[must_use]
pub fn body_from_camera(camera: FrameId) -> RigidTransform {
    let axes = cam_axes_in_body();
    let uq = UnitQuaternion::from_matrix(&axes);
    let q = uq.quaternion();
    RigidTransform::from_parts(
        FrameId::BODY,
        camera,
        position_in_body(camera).into(),
        [q.i, q.j, q.k, q.w],
    )
    .expect("finite unit rig calibration")
}

/// rig 坐标树（机体系 + 三个相机，同一时刻快照）。
///
/// # Panics
/// rig 边构成环（编译期常量下不可能）。
#[must_use]
pub fn tree() -> FrameTree {
    let mut tree = FrameTree::default();
    for camera in CAMERAS {
        tree.set(body_from_camera(camera))
            .expect("rig tree is acyclic");
    }
    tree
}

/// 估计器用的机体→相机旋转 `R_ItoC`（CV 光学约定）。
#[must_use]
pub fn rot_ito_c() -> Matrix3<f64> {
    CV_FROM_RENDER * cam_axes_in_body().transpose()
}

/// 估计器用的 `p_IinC = −R_ItoC · p_CinI`（IMU 原点在相机系，CV 光学约定）。
///
/// # Panics
/// `camera` 不是 rig 相机坐标系。
#[must_use]
pub fn p_i_in_c(camera: FrameId) -> Vector3<f64> {
    -rot_ito_c() * position_in_body(camera)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baseline_is_lateral_and_symmetric() {
        assert!((RIGHT_IN_BODY[1] - LEFT_IN_BODY[1] - BASELINE).abs() < 1e-12);
        assert!((LEFT_IN_BODY[0] - RIGHT_IN_BODY[0]).abs() < 1e-12);
        assert!((DEPTH_IN_BODY[1]).abs() < 1e-12);
        assert!((DEPTH_IN_BODY[0] - CAMERA_FORWARD_M).abs() < 1e-12);
    }

    #[test]
    fn axes_are_orthonormal_right_handed() {
        let r = cam_axes_in_body();
        assert!((r.transpose() * r - Matrix3::identity()).norm() < 1e-12);
        assert!((r.determinant() - 1.0).abs() < 1e-12);
        // 光轴（−z 列）应指向机头并下倾
        let view = -r.column(2);
        assert!(view.x > 0.9, "光轴须指向前方: {view}");
        assert!((view.z + DOWNTILT_DEG.to_radians().sin()).abs() < 1e-12);
    }

    /// 树查询与直接变换一致，且可反向查。
    #[test]
    fn tree_lookup_matches_direct_transform() {
        let tree = tree();
        for camera in CAMERAS {
            let via_tree = tree.lookup(FrameId::BODY, camera).unwrap();
            let direct = body_from_camera(camera);
            assert!((via_tree.matrix() - direct.matrix()).norm() < 1e-12);
            let back = tree.lookup(camera, FrameId::BODY).unwrap();
            assert!((back.matrix() - direct.inverse().matrix()).norm() < 1e-12);
        }
    }

    /// `p_IinC` 必须能还原出 render 安装位置（这是 10-07 漏改的那一项）。
    #[test]
    fn p_i_in_c_roundtrip_restores_mounting_position() {
        for camera in CAMERAS {
            let p_c_in_i = position_in_body(camera);
            let restored = -rot_ito_c().transpose() * p_i_in_c(camera);
            assert!(
                (restored - p_c_in_i).norm() < 1e-12,
                "{camera:?}: {restored} != {p_c_in_i}"
            );
        }
    }

    #[test]
    fn rot_ito_c_matches_optical_convention() {
        // 图右 = 机体 −y；光轴 = 机体 +x 下倾 20°。
        let r = rot_ito_c();
        let right = r.row(0).transpose();
        assert!((right - Vector3::new(0.0, -1.0, 0.0)).norm() < 1e-12);
        let forward = r.row(2).transpose();
        let tilt = DOWNTILT_DEG.to_radians();
        assert!((forward - Vector3::new(tilt.cos(), 0.0, -tilt.sin())).norm() < 1e-12);
    }

    /// 双目基线在机体系仍落在横向（立体视差来源）。
    #[test]
    fn stereo_baseline_is_lateral_in_body_frame() {
        let l = body_from_camera(FrameId::LEFT_CAMERA).point(nalgebra::Point3::origin());
        let r = body_from_camera(FrameId::RIGHT_CAMERA).point(nalgebra::Point3::origin());
        let delta = r - l;
        assert!(delta.x.abs() < 1e-12 && delta.z.abs() < 1e-12);
        assert!((delta.y.abs() - BASELINE).abs() < 1e-12);
    }
}
