//! VIO 后验在 IMU 时钟上的高频预测输出。
use iceoryx2::prelude::*;

pub const ATTITUDE_TOPIC: &str = "Firefly/VioAttitude";

/// Hamilton body→odom xyzw；角速度为标定及去偏后的机体系 rad/s。
/// `timestamp` 是预测的 IMU 时刻，`posterior_time` 是其 VIO 后验起点。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyVioAttitude")]
pub struct AttitudeMessage {
    pub timestamp: f64,
    pub posterior_time: f64,
    pub session: u64,
    pub quat_xyzw: [f64; 4],
    pub angular_velocity: [f64; 3],
    pub gyro_bias: [f64; 3],
}

#[cfg(test)]
mod tests {
    #[test]
    fn attitude_has_fixed_interprocess_layout() {
        use super::AttitudeMessage;
        assert_eq!(std::mem::size_of::<AttitudeMessage>(), 104);
        assert_eq!(std::mem::offset_of!(AttitudeMessage, quat_xyzw), 24);
    }
}
