//! PX4 `ControlMath` 的 1:1 移植。
//!
//! 来源：PX4-Autopilot 提交 `4dbd2e069a5c30c2e53e47e842095d2576dc38c4` 的
//! `src/modules/mc_pos_control/PositionControl/ControlMath.hpp` 与
//! `ControlMath.cpp`，BSD-3-Clause 许可。vendor 目录未包含
//! `ControlMathTest.cpp`，本文件的测试按 `.hpp`/`.cpp` 的语义编写。
//!
//! 约定：世界系 NED、机体系 FRD；位置 m，速度 m/s，加速度 m/s²，角度 rad；
//! 推力按最大集体推力归一化（无量纲，机体系 z 分量向上为负）。
//!
//! 不移植：uORB 主题 `vehicle_attitude_setpoint`，改用共享结构
//! [`VehicleAttitudeSetpoint`]。
//! 不移植：`px4_platform_common/defines.h`，其 `PX4_ISFINITE` 由 `f32::is_finite`
//! 等价提供。
//! 不移植：`geo/geo.h`，本文件未使用其中任何符号。

use nalgebra::Matrix3;

// 共享类型统一从 types 引入，与 PX4 的 `using namespace` 对应
#[allow(clippy::wildcard_imports)]
use super::types::*;

/// 将世界系推力向量和偏航角转换为期望姿态。
///
/// `thr_sp` 为世界系 NED 推力向量，按最大集体推力归一化（无量纲）。
/// `yaw_sp` 为期望偏航角（rad）。
/// 输出 `att_sp.q_d` 为期望姿态，`att_sp.thrust_body[2]` 为归一化集体推力。
pub fn thrust_to_attitude(thr_sp: Vector3f, yaw_sp: f32, att_sp: &mut VehicleAttitudeSetpoint) {
    bodyz_to_attitude(-thr_sp, yaw_sp, att_sp);
    att_sp.thrust_body[2] = -thr_sp.norm();
}

/// 限制两个单位向量之间的倾角。
///
/// `body_unit` 为待调整的单位向量（原地修改），`world_unit` 为固定参考单位向量，
/// `max_angle` 为最大夹角（rad）。
pub fn limit_tilt(body_unit: &mut Vector3f, world_unit: Vector3f, max_angle: f32) {
    // 计算倾角
    let dot_product_unit = body_unit.dot(&world_unit);
    let mut angle = dot_product_unit.acos();
    // 限制倾角
    angle = angle.min(max_angle);
    let mut rejection = *body_unit - (world_unit * dot_product_unit);

    // 两向量恰好平行的退化情况
    if rejection.norm_squared() < f32::EPSILON {
        rejection[0] = 1.0;
    }

    *body_unit = world_unit * angle.cos() + rejection.normalize() * angle.sin();
}

/// 由期望机体 z 轴和偏航角生成期望姿态。
///
/// `body_z` 为世界系 NED 下期望机体 z 轴方向（无需归一化）。
/// `yaw_sp` 为期望偏航角（rad）。
pub fn bodyz_to_attitude(mut body_z: Vector3f, yaw_sp: f32, att_sp: &mut VehicleAttitudeSetpoint) {
    // 零向量无方向，取安全的水平姿态
    if body_z.norm_squared() < f32::EPSILON {
        body_z[2] = 1.0;
    }

    body_z.normalize_mut();

    // 期望偏航方向在 XY 平面内旋转 PI/2 后的向量
    let y_c = Vector3f::new(-yaw_sp.sin(), yaw_sp.cos(), 0.0);

    // 与 body_z 正交的期望机体 x 轴
    let mut body_x = y_c.cross(&body_z);

    // 倒飞时保持机头朝向机头方向
    if body_z[2] < 0.0 {
        body_x = -body_x;
    }

    if body_z[2].abs() < 0.000_001 {
        // 期望推力位于 XY 平面，令 x 轴朝下以构造正确矩阵，
        // 此时偏航分量实际不会被使用
        body_x = Vector3f::new(0.0, 0.0, 1.0);
    }

    body_x.normalize_mut();

    // 期望机体 y 轴
    let body_y = body_z.cross(&body_x);

    // 按列填充旋转矩阵
    let r_sp = Matrix3::from_columns(&[body_x, body_y, body_z]);

    att_sp.q_d = Quatf::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(r_sp));
}

/// 返回两个向量之和，但按优先级和最大幅值约束。
///
/// `v0` 在最大可用幅值下优先，`v1` 次之；`max` 为合成向量的最大模长。
#[must_use]
pub fn constrain_xy(v0: Vector2f, v1: Vector2f, max: f32) -> Vector2f {
    if (v0 + v1).norm() <= max {
        // 向量未超过最大幅值
        return v0 + v1;
    }

    if v0.norm() >= max {
        // 优先方向 v0 的幅值已超过最大值
        return v0.normalize() * max;
    }

    if (v1 - v0).norm().abs() < 0.001 {
        // 两向量相等
        return v0.normalize() * max;
    }

    if v0.norm() < 0.001 {
        // 第一个向量为零
        return v1.normalize() * max;
    }

    // 记合成向量 vf、缩放系数 s、v1 的单位向量 u1，满足 vf = v0 + s * u1
    // 且 ||vf|| <= max。对 ||v0 + s * u1||^2 = max^2 解二次方程：
    // s = -u1.dot(v0) + sqrt(u1.dot(v0)^2 - (v0.dot(v0) - max^2))
    let u1 = v1.normalize();
    let m = u1.dot(&v0);
    let c = v0.dot(&v0) - max * max;
    let s = -m + (m * m - c).sqrt();
    v0 + u1 * s
}

/// 用球体与线段求交来平滑两段线之间的拐角。
///
/// `sphere_c` 为球心（m），`sphere_r` 为球半径（m），`line_a`/`line_b` 为线段端点（m），
/// 交点写入 `res`（m）。返回是否与球体相交。
pub fn cross_sphere_line(
    sphere_c: Vector3f,
    sphere_r: f32,
    line_a: Vector3f,
    line_b: Vector3f,
    res: &mut Vector3f,
) -> bool {
    // 把球心投影到归一化线段 AB 上
    let mut ab_norm = line_b - line_a;

    if ab_norm.norm() < 0.01 {
        return true;
    }

    ab_norm.normalize_mut();
    let d = line_a + ab_norm * (sphere_c - line_a).dot(&ab_norm);
    let cd_len = (sphere_c - d).norm();

    if sphere_r > cd_len {
        // 三角形 CDX 中已知 CD 与 CX = R，求 DX
        let dx_len = (sphere_r * sphere_r - cd_len * cd_len).sqrt();

        if (sphere_c - line_b).dot(&ab_norm) > 0.0 {
            // 目标航点已在身后
            *res = line_b;
        } else {
            // 目标在前方
            *res = d + ab_norm * dx_len; // 线段上的 A->B 方向
        }

        true
    } else {
        // 无交点，返回投影点 D
        *res = d;

        // 前一航点仍在前方
        if (sphere_c - line_a).dot(&ab_norm) < 0.0 {
            *res = line_a;
        }

        // 目标航点已在身后
        if (sphere_c - line_b).dot(&ab_norm) > 0.0 {
            *res = line_b;
        }

        false
    }
}

/// 将前馈量叠加到设定值上，NAN 不参与运算。
///
/// `setpoint` 为可能含 NAN 的当前设定值（原地修改），`addition` 为待叠加量或 NAN。
pub fn add_if_not_nan(setpoint: &mut f32, addition: f32) {
    if is_finite(*setpoint) && is_finite(addition) {
        // 均非 NAN，叠加
        *setpoint += addition;
    } else if !is_finite(*setpoint) {
        // 设定值为 NAN，取叠加量
        *setpoint = addition;
    }

    // 叠加量为 NAN 或两者均为 NAN，不处理
}

/// [`add_if_not_nan`] 的逐分量 `Vector3f` 版本。
pub fn add_if_not_nan_vector3f(setpoint: &mut Vector3f, addition: Vector3f) {
    for (sp, add) in setpoint.iter_mut().zip(addition.iter()) {
        add_if_not_nan(sp, *add);
    }
}

/// 将向量中的 NAN 分量写为零。
pub fn set_zero_if_nan_vector3f(vector: &mut Vector3f) {
    // 叠加零向量即可把 NAN 分量覆盖为零
    add_if_not_nan_vector3f(vector, Vector3f::zeros());
}

#[cfg(test)]
mod tests {
    use core::f32::consts::FRAC_PI_2;

    use super::*;

    fn assert_close(actual: f32, expected: f32, eps: f32) {
        assert!(
            (actual - expected).abs() <= eps,
            "actual {actual}, expected {expected}"
        );
    }

    #[test]
    fn constrain_xy_prioritizes_v0() {
        let max = 1.0;

        // v0 方向已超限：结果沿 v0、模长为 max
        let out = constrain_xy(Vector2f::new(2.0, 0.0), Vector2f::new(0.0, 1.0), max);
        assert_close(out.x, 1.0, 1e-6);
        assert_close(out.y, 0.0, 1e-6);

        // 合成超限但 v0 未超限：保留 v0 分量，沿 v1 方向补足
        let out = constrain_xy(Vector2f::new(0.5, 0.0), Vector2f::new(1.0, 0.0), max);
        assert_close(out.norm(), max, 1e-6);
        assert_close(out.y, 0.0, 1e-6);

        // 未超限：直接相加，v0 不被缩放
        let out = constrain_xy(Vector2f::new(0.3, 0.0), Vector2f::new(0.0, 0.3), max);
        assert_close(out.x, 0.3, 1e-6);
        assert_close(out.y, 0.3, 1e-6);

        // v0 为零：沿 v1 方向取 max
        let out = constrain_xy(Vector2f::new(0.0, 0.0), Vector2f::new(1.0, 1.0), max);
        assert_close(out.norm(), max, 1e-6);
        assert_close(out.x, out.y, 1e-6);

        // 默认分支：合成向量模长被约束为 max
        let out = constrain_xy(Vector2f::new(0.5, 0.2), Vector2f::new(0.5, 0.9), max);
        assert_close(out.norm(), max, 1e-5);
        assert!(out.dot(&Vector2f::new(0.5, 0.2)) > 0.0);
    }

    #[test]
    fn limit_tilt_limits_angle() {
        let mut body = Vector3f::new(1.0, 0.0, 0.0);
        limit_tilt(&mut body, Vector3f::z(), 0.5);
        assert_close(body.norm(), 1.0, 1e-6);
        let angle = body.dot(&Vector3f::z()).acos();
        assert_close(angle, 0.5, 1e-5);

        // 平行向量不变
        let mut body = Vector3f::z();
        limit_tilt(&mut body, Vector3f::z(), 0.5);
        assert!((body - Vector3f::z()).norm() < 1e-6);

        // 夹角小于限制时保持原方向
        let mut body = Vector3f::new(0.0, 0.2, 1.0).normalize();
        let original = body;
        limit_tilt(&mut body, Vector3f::z(), 0.5);
        assert!((body - original).norm() < 1e-6);
    }

    #[test]
    fn bodyz_to_attitude_level_and_yaw() {
        let mut att = VehicleAttitudeSetpoint::default();
        bodyz_to_attitude(Vector3f::z(), 0.0, &mut att);
        assert!((att.q_d * Vector3f::z() - Vector3f::z()).norm() < 1e-6);
        assert!((att.q_d * Vector3f::x() - Vector3f::x()).norm() < 1e-6);

        // 偏航 90 度：机体 x 轴指向世界 +y
        let mut att = VehicleAttitudeSetpoint::default();
        bodyz_to_attitude(Vector3f::z(), FRAC_PI_2, &mut att);
        assert!((att.q_d * Vector3f::z() - Vector3f::z()).norm() < 1e-5);
        assert!((att.q_d * Vector3f::x() - Vector3f::y()).norm() < 1e-5);

        // 零向量按水平姿态处理
        let mut att = VehicleAttitudeSetpoint::default();
        bodyz_to_attitude(Vector3f::zeros(), 0.0, &mut att);
        assert!((att.q_d * Vector3f::z() - Vector3f::z()).norm() < 1e-6);
    }

    #[test]
    fn thrust_to_attitude_thrust_sign() {
        // 向上的推力（NED 下 z 为负）：水平姿态、集体推力为负
        let mut att = VehicleAttitudeSetpoint::default();
        thrust_to_attitude(Vector3f::new(0.0, 0.0, -0.5), 0.0, &mut att);
        assert_close(att.thrust_body[2], -0.5, 1e-6);
        assert!((att.q_d * Vector3f::z() - Vector3f::z()).norm() < 1e-5);

        // 向下的推力：机体 z 轴朝下、集体推力仍为负
        let mut att = VehicleAttitudeSetpoint::default();
        thrust_to_attitude(Vector3f::new(0.0, 0.0, 0.5), 0.0, &mut att);
        assert_close(att.thrust_body[2], -0.5, 1e-6);
        assert!((att.q_d * Vector3f::z() - Vector3f::new(0.0, 0.0, -1.0)).norm() < 1e-5);

        // 一般推力向量：机体 z 轴与推力方向相反
        let thr = Vector3f::new(0.1, 0.2, -0.9);
        let mut att = VehicleAttitudeSetpoint::default();
        thrust_to_attitude(thr, 0.0, &mut att);
        assert_close(att.thrust_body[2], -thr.norm(), 1e-6);
        assert!((att.q_d * Vector3f::z() - (-thr).normalize()).norm() < 1e-5);
    }

    #[test]
    fn cross_sphere_line_smooths_corner() {
        let a = Vector3f::new(0.0, 0.0, 0.0);
        let b = Vector3f::new(10.0, 0.0, 0.0);

        // 不相交：返回最近的线段点
        let mut res = Vector3f::zeros();
        let hit = cross_sphere_line(Vector3f::new(5.0, 0.0, 1.0), 0.5, a, b, &mut res);
        assert!(!hit);
        assert!((res - Vector3f::new(5.0, 0.0, 0.0)).norm() < 1e-5);

        // 相交且交点在球心前方
        let mut res = Vector3f::zeros();
        let hit = cross_sphere_line(Vector3f::new(5.0, 0.0, 1.0), 2.0, a, b, &mut res);
        assert!(hit);
        assert!((res.x - (5.0 + 3.0_f32.sqrt())).abs() < 1e-4);
        assert_close(res.y, 0.0, 1e-6);
        assert_close(res.z, 0.0, 1e-6);

        // 目标航点已在身后：返回 line_b
        let mut res = Vector3f::zeros();
        let hit = cross_sphere_line(Vector3f::new(12.0, 0.0, 0.0), 2.0, a, b, &mut res);
        assert!(hit);
        assert!((res - b).norm() < 1e-6);

        // 退化线段：不修改结果
        let mut res = Vector3f::new(1.0, 2.0, 3.0);
        let hit = cross_sphere_line(Vector3f::new(5.0, 0.0, 0.0), 2.0, a, a, &mut res);
        assert!(hit);
        assert!((res - Vector3f::new(1.0, 2.0, 3.0)).norm() < 1e-6);
    }

    #[test]
    fn nan_helpers_match_px4() {
        // 均为有限值：相加
        let mut value = 1.0;
        add_if_not_nan(&mut value, 2.0);
        assert_close(value, 3.0, 1e-6);

        // 设定值为 NAN：取叠加量
        let mut value = f32::NAN;
        add_if_not_nan(&mut value, 2.0);
        assert_close(value, 2.0, 1e-6);

        // 叠加量为 NAN：保持不变
        let mut value = 1.0;
        add_if_not_nan(&mut value, f32::NAN);
        assert_close(value, 1.0, 1e-6);

        let mut vector = Vector3f::new(f32::NAN, 1.0, 2.0);
        add_if_not_nan_vector3f(&mut vector, Vector3f::new(1.0, 1.0, f32::NAN));
        assert_close(vector[0], 1.0, 1e-6);
        assert_close(vector[1], 2.0, 1e-6);
        assert_close(vector[2], 2.0, 1e-6);

        let mut vector = Vector3f::new(f32::NAN, f32::NAN, f32::NAN);
        set_zero_if_nan_vector3f(&mut vector);
        assert_eq!(vector, Vector3f::zeros());
    }
}
