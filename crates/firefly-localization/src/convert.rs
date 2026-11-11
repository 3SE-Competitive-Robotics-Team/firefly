//! 里程计跨坐标转换：位姿复合，物理速度旋转换基。
use firefly_base::RigidTransform;
use firefly_error::Result;
use firefly_pubsub::odom::OdomMessage;

/// 将连续 odom 系状态转换到地图系；时间和初始化标志沿用原始状态。
/// 速度为同一物理速度换基，不包含估计坐标变换跳变的导数。
/// # Errors
/// 位姿、速度无效，或给定变换不是 map←odom。
pub fn corrected_odom(src: &OdomMessage, alignment: &RigidTransform) -> Result<OdomMessage> {
    src.in_map(alignment)
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_base::FrameId;
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
