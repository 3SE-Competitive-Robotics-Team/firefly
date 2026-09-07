//! 位姿表示转换（`OdomMessage` ↔ 齐次矩阵，各融合消费端共用）。

use firefly_pubsub::odom::OdomMessage;
use nalgebra::{Isometry3, Matrix4, Quaternion, Translation3, UnitQuaternion, Vector3};

/// `OdomMessage` → 全局位姿齐次矩阵。
#[must_use]
pub fn odom_to_matrix(msg: &OdomMessage) -> Matrix4<f64> {
    let t = Vector3::new(msg.position_x, msg.position_y, msg.position_z);
    let q = UnitQuaternion::from_quaternion(Quaternion::new(
        msg.quat_w, msg.quat_x, msg.quat_y, msg.quat_z,
    ));
    Isometry3::from_parts(Translation3::new(t.x, t.y, t.z), q).to_homogeneous()
}

/// 矫正后位姿 → `OdomMessage`（速度经漂移旋转修正，时间戳/初始化位沿用源消息）。
#[must_use]
pub fn matrix_to_odom(
    t_corr: &Matrix4<f64>,
    src: &OdomMessage,
    drift: &Matrix4<f64>,
) -> OdomMessage {
    let p = t_corr.fixed_view::<3, 1>(0, 3).into_owned();
    let r = t_corr.fixed_view::<3, 3>(0, 0).into_owned();
    let quat = UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix(&r));
    let q = quat.quaternion();
    let drift_rot = drift.fixed_view::<3, 3>(0, 0).into_owned();
    let v = Vector3::new(src.velocity_x, src.velocity_y, src.velocity_z);
    let v_corr = drift_rot * v;
    OdomMessage {
        timestamp: src.timestamp,
        position_x: p.x,
        position_y: p.y,
        position_z: p.z,
        velocity_x: v_corr.x,
        velocity_y: v_corr.y,
        velocity_z: v_corr.z,
        quat_x: q.i,
        quat_y: q.j,
        quat_z: q.k,
        quat_w: q.w,
        is_initialized: src.is_initialized,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let msg = OdomMessage {
            timestamp: 1.0,
            position_x: 1.0,
            position_y: 2.0,
            position_z: 3.0,
            velocity_x: 0.1,
            velocity_y: 0.0,
            velocity_z: 0.0,
            quat_x: 0.0,
            quat_y: 0.0,
            quat_z: 0.0,
            quat_w: 1.0,
            is_initialized: true,
        };
        let back = matrix_to_odom(&odom_to_matrix(&msg), &msg, &Matrix4::identity());
        assert!((back.position_x - 1.0).abs() < 1e-9);
        assert!((back.quat_w - 1.0).abs() < 1e-9);
        assert!(back.is_initialized);
    }
}
