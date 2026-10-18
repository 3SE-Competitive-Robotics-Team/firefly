//! 产品坐标与 PX4 内部坐标的边界转换。
//!
//! 产品：世界 `+Z` 上、机体 FLU（`+X` 前、`+Y` 左、`+Z` 上）。
//! PX4：世界 NED、机体 FRD（`+X` 前、`+Y` 右、`+Z` 下）。
//! 两个坐标系之间是同一个反射 `F = diag(1, -1, -1)`；它是对合的，
//! 因此 `F⁻¹ = F`，位置、速度、加速度、角速度、力矩都用同一套换轴。
//!
//! 姿态按 `q_frd_ned = F · q_flu_world · F⁻¹` 共轭转换；航向角
//! `yaw_ned = -yaw_flu`（绕相反的世界 z 轴）。所有转换只在本模块出现，
//! 移植的 PX4 类保持 NED/FRD。

use nalgebra::{UnitQuaternion, Vector3};

use super::types::{Quatf, quat_canonical};

/// 世界/机体系换轴：`[x, y, z] -> [x, -y, -z]`。
#[must_use]
pub fn swap_yz(v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(v.x, -v.y, -v.z)
}

/// 产品世界系（Z 上）→ PX4 NED。
#[must_use]
pub fn world_to_ned(v: Vector3<f32>) -> Vector3<f32> {
    swap_yz(v)
}

/// PX4 NED → 产品世界系（Z 上）。
#[must_use]
pub fn world_from_ned(v: Vector3<f32>) -> Vector3<f32> {
    swap_yz(v)
}

/// 机体 FLU → 机体 FRD。
#[must_use]
pub fn body_to_frd(v: Vector3<f32>) -> Vector3<f32> {
    swap_yz(v)
}

/// 机体 FRD → 机体 FLU。
#[must_use]
pub fn body_from_frd(v: Vector3<f32>) -> Vector3<f32> {
    swap_yz(v)
}

/// 机体角速度：FLU → FRD。
#[must_use]
pub fn rate_to_frd(w: Vector3<f32>) -> Vector3<f32> {
    swap_yz(w)
}

/// 机体力矩：FLU → FRD。
#[must_use]
pub fn torque_to_frd(t: Vector3<f32>) -> Vector3<f32> {
    swap_yz(t)
}

/// 机体力矩：FRD → FLU。
#[must_use]
pub fn torque_from_frd(t: Vector3<f32>) -> Vector3<f32> {
    swap_yz(t)
}

/// 姿态：产品 body→world（FLU）→ PX4 body→NED（FRD）。
#[must_use]
pub fn attitude_to_frd_ned(q: Quatf) -> Quatf {
    let flip = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::PI);
    quat_canonical(flip * q * flip.inverse())
}

/// 姿态：PX4 body→NED（FRD）→ 产品 body→world（FLU）。
#[must_use]
pub fn attitude_from_frd_ned(q: Quatf) -> Quatf {
    attitude_to_frd_ned(q)
}

/// 航向：产品世界系（绕 +Z 上）→ PX4 NED（绕 +Z 下）。
#[must_use]
pub fn yaw_to_ned(yaw: f32) -> f32 {
    wrap_pi(-yaw)
}

/// 航向：PX4 NED → 产品世界系。
#[must_use]
pub fn yaw_from_ned(yaw: f32) -> f32 {
    wrap_pi(-yaw)
}

/// 包装到 `[-π, π]`。
#[must_use]
pub fn wrap_pi(angle: f32) -> f32 {
    let mut wrapped = angle % std::f32::consts::TAU;
    if wrapped > std::f32::consts::PI {
        wrapped -= std::f32::consts::TAU;
    }
    if wrapped <= -std::f32::consts::PI {
        wrapped += std::f32::consts::TAU;
    }
    wrapped
}

/// 从 FRD→NED 姿态取 NED 航向（机体 `+X` 在水平面内的方向）。
#[must_use]
pub fn yaw_of_frd_ned(q: Quatf) -> f32 {
    let forward = q * Vector3::x();
    forward.y.atan2(forward.x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Quaternion;

    #[test]
    fn frame_flip_is_an_involution() {
        for v in [
            Vector3::new(1., 2., 3.),
            Vector3::new(-0.4, 5., -6.),
            Vector3::zeros(),
        ] {
            assert!((swap_yz(swap_yz(v)) - v).norm() < 1e-7);
            assert!((world_to_ned(v) - body_to_frd(v)).norm() < 1e-7);
        }
    }

    #[test]
    fn attitude_conjugation_keeps_physical_rotation() {
        // 产品系下绕 +Z 偏航 +90°：机体 +X 指向产品世界 +Y（左）。
        let q_flu =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_2);
        let q_frd = attitude_to_frd_ned(q_flu);
        // NED 下同一物理朝向：机体 +X 指向世界 -Y（西）。
        let forward_ned = q_frd * Vector3::x();
        assert!((forward_ned - Vector3::new(0., -1., 0.)).norm() < 1e-6);
        // 圆路径：先转成 NED 再转回来，姿态一致（允许整体符号）。
        let back = attitude_from_frd_ned(q_frd);
        let dot = back.into_inner().coords.dot(&q_flu.into_inner().coords);
        assert!((dot.abs() - 1.0).abs() < 1e-6);
        // 航向角符号。
        assert!((yaw_of_frd_ned(q_frd) - yaw_to_ned(std::f32::consts::FRAC_PI_2)).abs() < 1e-6);
    }

    #[test]
    fn identity_attitude_and_level_yaw_map_to_identity() {
        let q = UnitQuaternion::identity();
        let converted = attitude_to_frd_ned(q);
        let raw: Quaternion<f32> = converted.into_inner();
        assert!(raw.w.abs() > 1.0 - 1e-6);
        assert!((converted * Vector3::x() - Vector3::x()).norm() < 1e-6);
        assert!(yaw_to_ned(0.).abs() < 1e-6);
    }

    #[test]
    fn yaw_wraps_across_pi() {
        assert!((wrap_pi(3. * std::f32::consts::PI).abs() - std::f32::consts::PI).abs() < 1e-6);
        assert!((wrap_pi(-3. * std::f32::consts::PI).abs() - std::f32::consts::PI).abs() < 1e-6);
        assert!((wrap_pi(0.3) - 0.3).abs() < 1e-6);
    }
}
