//! IMU 消息与 imu 话题（对照 `docs/architecture.md` 的 `topic: imu`）。
//!
//! `#[repr(C)]` 定长结构，满足 `ZeroCopySend` 约束。trace 上下文由发布
//! 中间件写入 **User Header**（见 [`crate::trace`]），payload 保持精简。

use iceoryx2::prelude::*;

/// imu 话题名（对照 `docs/architecture.md` 的 `topic: imu`）。
pub const IMU_TOPIC: &str = "Firefly/Imu";

/// IMU 消息。
///
/// 传感器坐标系的角速度（陀螺仪，rad/s）和比力（加速度计，m/s²）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyImuMessage")]
pub struct ImuMessage {
    /// 传感器时钟时间戳（秒）。
    pub timestamp: f64,
    /// 角速度 `w`（rad/s）。
    pub angular_velocity_x: f64,
    pub angular_velocity_y: f64,
    pub angular_velocity_z: f64,
    /// 比力 `a`（m/s²）。
    pub linear_acceleration_x: f64,
    pub linear_acceleration_y: f64,
    pub linear_acceleration_z: f64,
}

impl Default for ImuMessage {
    fn default() -> Self {
        Self {
            timestamp: -1.0,
            angular_velocity_x: 0.0,
            angular_velocity_y: 0.0,
            angular_velocity_z: 0.0,
            linear_acceleration_x: 0.0,
            linear_acceleration_y: 0.0,
            linear_acceleration_z: 0.0,
        }
    }
}

/// IMU 服务容量与接收队列；Python 发布端声明相同服务容量。
/// 接收队列覆盖 100Hz 下约 1s 排空停顿，溢出会丢失测量。
pub const IMU_SUBSCRIPTION: crate::subscriber::SubscriberOptions =
    crate::subscriber::SubscriberOptions {
        buffer_size: 100,
        service_max_buffer_size: Some(128),
        max_publishers: None,
    };

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_message_is_plain_old_data() {
        let m = ImuMessage::default();
        assert_eq!(std::mem::size_of::<ImuMessage>(), 56);
        assert!((m.timestamp + 1.0).abs() < 1e-9);
        assert!(m.linear_acceleration_z.abs() < 1e-12);
    }

    /// 缓冲不变量（编译期）：订阅缓冲 ≤ 服务上限（超限则创建直接失败，见
    /// `Subscriber::with_options`），且覆盖 100Hz 下 1s 排空停顿。
    const _: () = {
        assert!(IMU_SUBSCRIPTION.buffer_size <= IMU_SUBSCRIPTION.service_max_buffer_size.unwrap());
        assert!(IMU_SUBSCRIPTION.buffer_size >= 100);
    };
}
