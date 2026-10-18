//! PX4 倾斜设定点的偏航误差补偿（`AttitudeControlMath`）。
//!
//! 参考实现：PX4-Autopilot 提交 `4dbd2e069a5c30c2e53e47e842095d2576dc38c4`
//! 的 `src/modules/mc_att_control/AttitudeControl/AttitudeControlMath.hpp`（BSD-3-Clause）。
//! 保持 NED 世界系 / FRD 机体系约定。
//!
//! 不移植：`namespace AttitudeControlMath`——Rust 模块本身就是命名空间。
//!
//! 倾斜修正只在世界系中重定向推力轴，不改变期望倾角大小；`q_sp_tilt` 必须是不含偏航的
//! 纯倾斜四元数。反向平行时 PX4 `Quatf(v0, v1)` 取正交轴 180° 旋转，`types` 中的
//! 同名助手会退化为单位四元数，故此处私有补齐。

// 按要求使用 `types` 的 glob 导入。
#![allow(clippy::wildcard_imports)]

use super::types::*;

/// 把纯倾斜设定点按“当前偏航 + 期望偏航”的差异重新对齐。
///
/// 使 `q_sp_yaw * 修正后的倾斜` 与按当前偏航旋转原倾斜设定点得到的世界系推力方向一致。
///
/// - `q_sp_tilt`：纯倾斜设定点（偏航 = 0），原地修正
/// - `q_att`：当前姿态
/// - `q_sp_yaw`：期望偏航的纯偏航四元数
pub fn correct_tilt_setpoint_for_yaw_error(q_sp_tilt: &mut Quatf, q_att: Quatf, q_sp_yaw: Quatf) {
    let z_unit = Vector3f::z();

    // 从当前姿态提取偏航
    let att_z = quat_dcm_z(q_att);
    let q_tilt = quat_from_two_vectors_px4(z_unit, att_z);
    let q_yaw = q_tilt.inverse() * q_att; // 不是欧拉偏航角

    // 求使 q_yaw * q_tilt_new = q_sp_yaw * q_sp_rp_compensated 的倾斜方向
    let q_sp_rp_compensated = q_sp_yaw.inverse() * q_yaw * *q_sp_tilt;

    // 提取修正后的倾斜
    let att_sp_z = quat_dcm_z(q_sp_rp_compensated);
    *q_sp_tilt = quat_from_two_vectors_px4(z_unit, att_sp_z);
}

/// PX4 `matrix::Quaternion(const Vector3 &src, const Vector3 &dst)`：`src` 到 `dst` 的
/// 最短旋转；反向平行时返回绕正交轴的 180° 旋转，而不是单位四元数。
fn quat_from_two_vectors_px4(src: Vector3f, dst: Vector3f) -> Quatf {
    let mut cr = src.cross(&dst);
    let dt = src.dot(&dst);

    let w = if cr.norm() < 1e-5 && dt < 0.0 {
        // 180° 反向：取 |src| 最小分量对应的坐标轴
        let abs = src.abs();
        let axis = if abs[0] < abs[1] {
            if abs[0] < abs[2] {
                Vector3f::x()
            } else {
                Vector3f::z()
            }
        } else if abs[1] < abs[2] {
            Vector3f::y()
        } else {
            Vector3f::z()
        };

        cr = src.cross(&axis);
        0.0
    } else {
        // 常规情况：半程四元数解
        dt + (src.norm_squared() * dst.norm_squared()).sqrt()
    };

    Quatf::from_quaternion(nalgebra::Quaternion::new(w, cr[0], cr[1], cr[2]))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_z_axis_near(q: Quatf, expected: Vector3f, tol: f32) {
        let actual = quat_dcm_z(q);
        assert!(
            (actual - expected).norm() <= tol,
            "actual {actual:?} expected {expected:?}"
        );
    }

    #[test]
    fn zero_yaw_error_leaves_tilt_unchanged() {
        let q_sp_tilt = quat_from_rotation_vector(Vector3f::new(0.0, 0.3, 0.0));
        let mut corrected = q_sp_tilt;

        correct_tilt_setpoint_for_yaw_error(&mut corrected, Quatf::identity(), Quatf::identity());

        assert_z_axis_near(corrected, quat_dcm_z(q_sp_tilt), 1e-6);
    }

    #[test]
    fn yaw_error_compensation_matches_definition() {
        let q_att = quat_from_rotation_vector(Vector3f::new(0.1, -0.2, 0.5));
        let q_sp_yaw = quat_from_rotation_vector(Vector3f::new(0.0, 0.0, 0.8));
        let q_sp_tilt = quat_from_rotation_vector(Vector3f::new(0.2, 0.15, 0.0));

        // 期望:q_sp_yaw * q_tilt_new 与按当前偏航旋转原倾斜的世界系推力方向一致
        let q_yaw_att = {
            let att_z = quat_dcm_z(q_att);
            let q_tilt = quat_from_two_vectors_px4(Vector3f::z(), att_z);
            q_tilt.inverse() * q_att
        };
        let expected = q_sp_yaw.inverse() * q_yaw_att * q_sp_tilt;

        let mut corrected = q_sp_tilt;
        correct_tilt_setpoint_for_yaw_error(&mut corrected, q_att, q_sp_yaw);

        assert_z_axis_near(corrected, quat_dcm_z(expected), 1e-6);
    }

    #[test]
    fn correction_is_finite_for_upside_down_attitude() {
        let upside_down = quat_from_rotation_vector(Vector3f::new(core::f32::consts::PI, 0.0, 0.0));
        let mut corrected = Quatf::identity();

        correct_tilt_setpoint_for_yaw_error(&mut corrected, upside_down, Quatf::identity());

        assert!(corrected.scalar().is_finite());
        assert!(corrected.vector().iter().all(|v| v.is_finite()));
        // 180° 姿态下 PX4 选取绕 +x 的 180° 倾斜，补偿后回到竖直推力方向
        assert_z_axis_near(corrected, Vector3f::z(), 1e-6);
    }
}
