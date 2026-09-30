//! 里程计跨坐标转换：位姿复合，物理速度旋转换基。
use firefly_base::{FrameId, RigidTransform};
use firefly_error::{Error, ErrorKind, Result};
use firefly_pubsub::odom::OdomMessage;
use nalgebra::Vector3;

/// 将连续 odom 系状态转换到地图系；时间和初始化标志沿用原始状态。
/// 速度为同一物理速度换基，不包含估计坐标变换跳变的导数。
/// # Errors
/// 位姿、速度无效，或给定变换不是 map←odom。
pub fn corrected_odom(src: &OdomMessage, alignment: &RigidTransform) -> Result<OdomMessage> {
    if alignment.target() != FrameId::MAP || alignment.source() != FrameId::ODOM {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "expected map<-odom alignment",
        ));
    }
    let pose = alignment.compose(&src.body_pose(FrameId::ODOM)?)?;
    let velocity = alignment.vector(Vector3::new(src.velocity_x, src.velocity_y, src.velocity_z));
    if !velocity.iter().all(|v| v.is_finite()) {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "non-finite odometry velocity",
        ));
    }
    let p = pose.isometry().translation.vector;
    let q = pose.isometry().rotation.quaternion();
    Ok(OdomMessage {
        position_x: p.x,
        position_y: p.y,
        position_z: p.z,
        velocity_x: velocity.x,
        velocity_y: velocity.y,
        velocity_z: velocity.z,
        quat_x: q.i,
        quat_y: q.j,
        quat_z: q.k,
        quat_w: q.w,
        ..*src
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn alignment_rotates_velocity_without_translating_it() {
        let h = std::f64::consts::FRAC_PI_4;
        let alignment = RigidTransform::from_parts(
            FrameId::MAP,
            FrameId::ODOM,
            [10., 0., 0.],
            [0., 0., h.sin(), h.cos()],
        )
        .unwrap();
        let source = OdomMessage {
            timestamp: 1.,
            position_x: 2.,
            velocity_x: 1.,
            is_initialized: true,
            ..Default::default()
        };
        let out = corrected_odom(&source, &alignment).unwrap();
        assert!((out.position_x - 10.).abs() < 1e-12 && (out.position_y - 2.).abs() < 1e-12);
        assert!(out.velocity_x.abs() < 1e-12 && (out.velocity_y - 1.).abs() < 1e-12);
        assert!((out.timestamp - source.timestamp).abs() < 1e-12 && out.is_initialized);
    }
}
