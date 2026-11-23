//! 机体 `BODY`：`+x` 前、`+y` 左、`+z` 上（与 `MuJoCo` 一致）。
//! 相机光学坐标为图右、图下、镜头前方。前视光轴下倾 [`DOWNTILT_DEG`]。

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use crate::frames::{FrameId, RigidTransform};

pub const DOWNTILT_DEG: f64 = 20.0;
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
/// 下视光心后移 3 cm；仿真标称高度与机体原点相同，实机外参以标定为准。
pub const DOWN_IN_BODY: [f64; 3] = [-0.03, 0.0, 0.0];

/// 对称透视投影的主点，使用左上像素中心为 `(0, 0)` 的图像坐标。
/// GPU 帧缓冲原点在像素边界，读取到图像数组后必须减去半个像素。
#[must_use]
pub fn principal_point(width: usize, height: usize) -> [f64; 2] {
    [(width as f64 - 1.0) / 2.0, (height as f64 - 1.0) / 2.0]
}

pub const CAMERAS: [FrameId; 4] = [
    FrameId::LEFT_CAMERA,
    FrameId::RIGHT_CAMERA,
    FrameId::DEPTH_CAMERA,
    FrameId::DOWN_CAMERA,
];

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
    } else if camera == FrameId::DOWN_CAMERA {
        DOWN_IN_BODY
    } else {
        panic!("not a rig camera frame");
    };
    Vector3::from(p)
}

/// 将光学坐标（右、下、前）变换到机体坐标（前、左、上）。
///
/// # Panics
/// `camera` 不是 rig 相机坐标系。
#[must_use]
pub fn body_from_optical(camera: FrameId) -> RigidTransform {
    let axes = if camera == FrameId::DOWN_CAMERA {
        Matrix3::from_columns(&[-Vector3::y(), -Vector3::x(), -Vector3::z()])
    } else {
        let (sin, cos) = DOWNTILT_DEG.to_radians().sin_cos();
        Matrix3::from_columns(&[
            -Vector3::y(),
            Vector3::new(-sin, 0.0, -cos),
            Vector3::new(cos, 0.0, -sin),
        ])
    };
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn optical_axes_match_camera_mounts() {
        let (sin, cos) = DOWNTILT_DEG.to_radians().sin_cos();
        for camera in CAMERAS {
            let mounting = body_from_optical(camera);
            assert_eq!(
                (mounting.target(), mounting.source()),
                (FrameId::BODY, camera)
            );
            let forward = if camera == FrameId::DOWN_CAMERA {
                -Vector3::z()
            } else {
                Vector3::new(cos, 0., -sin)
            };
            assert!((mounting.vector(Vector3::x()) + Vector3::y()).norm() < 1e-12);
            assert!((mounting.vector(Vector3::z()) - forward).norm() < 1e-12);
        }
    }

    #[test]
    fn camera_positions_preserve_lateral_baseline_and_centered_depth() {
        let l = body_from_optical(FrameId::LEFT_CAMERA).point(nalgebra::Point3::origin());
        let r = body_from_optical(FrameId::RIGHT_CAMERA).point(nalgebra::Point3::origin());
        let delta = r - l;
        assert!(delta.x.abs() < 1e-12 && delta.z.abs() < 1e-12);
        assert!((delta.y - BASELINE).abs() < 1e-12);
        assert!(DEPTH_IN_BODY[1].abs() < 1e-12);
        assert!((DEPTH_IN_BODY[0] - CAMERA_FORWARD_M).abs() < 1e-12);
    }
}
