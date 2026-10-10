//! VIO→FC 外援：同一时刻的 odom 系姿态/陀螺零偏边缘分布。
use iceoryx2::prelude::*;

pub const ATTITUDE_AID_TOPIC: &str = "Firefly/VioAttitudeAid";

/// Hamilton body→odom xyzw；右误差 `[δθ,δbg]` 协方差按行展开。
/// session 标识 VIO 进程会话，timestamp 为状态对应的 IMU 时钟秒。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyAttitudeAid")]
pub struct AttitudeAidMessage {
    pub timestamp: f64,
    pub session: u64,
    pub quat_xyzw: [f64; 4],
    pub gyro_bias: [f64; 3],
    pub covariance: [f64; 36],
}

#[cfg(test)]
mod tests {
    #[test]
    fn attitude_aid_has_fixed_interprocess_layout() {
        use super::AttitudeAidMessage;
        assert_eq!(std::mem::size_of::<AttitudeAidMessage>(), 360);
        assert_eq!(std::mem::offset_of!(AttitudeAidMessage, covariance), 72);
    }
}
