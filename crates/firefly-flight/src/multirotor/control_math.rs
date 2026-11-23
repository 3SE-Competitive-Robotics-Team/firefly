//! 坐标、单位与推力归一化约定见 [`super::types`]。

use nalgebra::Matrix3;

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
    let dot_product_unit = body_unit.dot(&world_unit);
    let mut angle = dot_product_unit.acos();
    angle = angle.min(max_angle);
    let mut rejection = *body_unit - (world_unit * dot_product_unit);

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

    let y_c = Vector3f::new(-yaw_sp.sin(), yaw_sp.cos(), 0.0);

    let mut body_x = y_c.cross(&body_z);

    if body_z[2] < 0.0 {
        body_x = -body_x;
    }

    if body_z[2].abs() < 0.000_001 {
        // 期望推力位于 XY 平面，令 x 轴朝下以构造正确矩阵，
        // 此时偏航分量实际不会被使用
        body_x = Vector3f::new(0.0, 0.0, 1.0);
    }

    body_x.normalize_mut();

    let body_y = body_z.cross(&body_x);

    let r_sp = Matrix3::from_columns(&[body_x, body_y, body_z]);

    att_sp.q_d = Quatf::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(r_sp));
}

/// 返回两个向量之和，但按优先级和最大幅值约束。
///
/// `v0` 在最大可用幅值下优先，`v1` 次之；`max` 为合成向量的最大模长。
#[must_use]
pub fn constrain_xy(v0: Vector2f, v1: Vector2f, max: f32) -> Vector2f {
    if (v0 + v1).norm() <= max {
        return v0 + v1;
    }

    if v0.norm() >= max {
        return v0.normalize() * max;
    }

    if (v1 - v0).norm().abs() < 0.001 {
        return v0.normalize() * max;
    }

    if v0.norm() < 0.001 {
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
            *res = line_b;
        } else {
            *res = d + ab_norm * dx_len; // 线段上的 A->B 方向
        }

        true
    } else {
        *res = d;

        if (sphere_c - line_a).dot(&ab_norm) < 0.0 {
            *res = line_a;
        }

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
        *setpoint += addition;
    } else if !is_finite(*setpoint) {
        *setpoint = addition;
    }
}

/// [`add_if_not_nan`] 的逐分量 `Vector3f` 版本。
pub fn add_if_not_nan_vector3f(setpoint: &mut Vector3f, addition: Vector3f) {
    for (sp, add) in setpoint.iter_mut().zip(addition.iter()) {
        add_if_not_nan(sp, *add);
    }
}

/// 将向量中的 NaN、正无穷和负无穷分量写为零，保留其余分量。
pub fn replace_nonfinite_with_zero(vector: &mut Vector3f) {
    add_if_not_nan_vector3f(vector, Vector3f::zeros());
}

#[cfg(test)]
mod tests {
    use core::f32::consts::FRAC_PI_2;

    use super::*;

    #[test]
    fn nonfinite_replacement_handles_infinities_and_preserves_finite_components() {
        let mut extremes = Vector3f::new(f32::NAN, f32::INFINITY, f32::NEG_INFINITY);
        replace_nonfinite_with_zero(&mut extremes);
        assert_eq!(extremes, Vector3f::zeros());
        let mut mixed = Vector3f::new(-2., f32::INFINITY, 3.5);
        replace_nonfinite_with_zero(&mut mixed);
        assert_eq!(mixed, Vector3f::new(-2., 0., 3.5));
    }

    fn assert_close(actual: f32, expected: f32, eps: f32) {
        assert!(
            (actual - expected).abs() <= eps,
            "actual {actual}, expected {expected}"
        );
    }

    #[test]
    fn constrain_xy_prioritizes_v0() {
        let max = 1.0;

        let out = constrain_xy(Vector2f::new(2.0, 0.0), Vector2f::new(0.0, 1.0), max);
        assert_close(out.x, 1.0, 1e-6);
        assert_close(out.y, 0.0, 1e-6);

        let out = constrain_xy(Vector2f::new(0.5, 0.0), Vector2f::new(1.0, 0.0), max);
        assert_close(out.norm(), max, 1e-6);
        assert_close(out.y, 0.0, 1e-6);

        let out = constrain_xy(Vector2f::new(0.3, 0.0), Vector2f::new(0.0, 0.3), max);
        assert_close(out.x, 0.3, 1e-6);
        assert_close(out.y, 0.3, 1e-6);

        let out = constrain_xy(Vector2f::new(0.0, 0.0), Vector2f::new(1.0, 1.0), max);
        assert_close(out.norm(), max, 1e-6);
        assert_close(out.x, out.y, 1e-6);

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

        let mut body = Vector3f::z();
        limit_tilt(&mut body, Vector3f::z(), 0.5);
        assert!((body - Vector3f::z()).norm() < 1e-6);

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

        let mut att = VehicleAttitudeSetpoint::default();
        bodyz_to_attitude(Vector3f::z(), FRAC_PI_2, &mut att);
        assert!((att.q_d * Vector3f::z() - Vector3f::z()).norm() < 1e-5);
        assert!((att.q_d * Vector3f::x() - Vector3f::y()).norm() < 1e-5);

        let mut att = VehicleAttitudeSetpoint::default();
        bodyz_to_attitude(Vector3f::zeros(), 0.0, &mut att);
        assert!((att.q_d * Vector3f::z() - Vector3f::z()).norm() < 1e-6);
    }

    #[test]
    fn thrust_to_attitude_thrust_sign() {
        let mut att = VehicleAttitudeSetpoint::default();
        thrust_to_attitude(Vector3f::new(0.0, 0.0, -0.5), 0.0, &mut att);
        assert_close(att.thrust_body[2], -0.5, 1e-6);
        assert!((att.q_d * Vector3f::z() - Vector3f::z()).norm() < 1e-5);

        let mut att = VehicleAttitudeSetpoint::default();
        thrust_to_attitude(Vector3f::new(0.0, 0.0, 0.5), 0.0, &mut att);
        assert_close(att.thrust_body[2], -0.5, 1e-6);
        assert!((att.q_d * Vector3f::z() - Vector3f::new(0.0, 0.0, -1.0)).norm() < 1e-5);

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

        let mut res = Vector3f::zeros();
        let hit = cross_sphere_line(Vector3f::new(5.0, 0.0, 1.0), 0.5, a, b, &mut res);
        assert!(!hit);
        assert!((res - Vector3f::new(5.0, 0.0, 0.0)).norm() < 1e-5);

        let mut res = Vector3f::zeros();
        let hit = cross_sphere_line(Vector3f::new(5.0, 0.0, 1.0), 2.0, a, b, &mut res);
        assert!(hit);
        assert!((res.x - (5.0 + 3.0_f32.sqrt())).abs() < 1e-4);
        assert_close(res.y, 0.0, 1e-6);
        assert_close(res.z, 0.0, 1e-6);

        let mut res = Vector3f::zeros();
        let hit = cross_sphere_line(Vector3f::new(12.0, 0.0, 0.0), 2.0, a, b, &mut res);
        assert!(hit);
        assert!((res - b).norm() < 1e-6);

        let mut res = Vector3f::new(1.0, 2.0, 3.0);
        let hit = cross_sphere_line(Vector3f::new(5.0, 0.0, 0.0), 2.0, a, a, &mut res);
        assert!(hit);
        assert!((res - Vector3f::new(1.0, 2.0, 3.0)).norm() < 1e-6);
    }

    #[test]
    fn nan_components_are_treated_as_unset() {
        let mut value = 1.0;
        add_if_not_nan(&mut value, 2.0);
        assert_close(value, 3.0, 1e-6);

        let mut value = f32::NAN;
        add_if_not_nan(&mut value, 2.0);
        assert_close(value, 2.0, 1e-6);

        let mut value = 1.0;
        add_if_not_nan(&mut value, f32::NAN);
        assert_close(value, 1.0, 1e-6);

        let mut vector = Vector3f::new(f32::NAN, 1.0, 2.0);
        add_if_not_nan_vector3f(&mut vector, Vector3f::new(1.0, 1.0, f32::NAN));
        assert_close(vector[0], 1.0, 1e-6);
        assert_close(vector[1], 2.0, 1e-6);
        assert_close(vector[2], 2.0, 1e-6);

        let mut vector = Vector3f::new(f32::NAN, f32::NAN, f32::NAN);
        replace_nonfinite_with_zero(&mut vector);
        assert_eq!(vector, Vector3f::zeros());
    }
}
