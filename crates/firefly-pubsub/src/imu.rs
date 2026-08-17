//! IMU 消息与 imu 话题（对照 `docs/architecture.md` 的 `topic: imu`）。
//!
//! `#[repr(C)]` 定长结构，满足 `ZeroCopySend` 约束。trace 上下文由发布
//! 中间件写入 **User Header**（见 [`crate::trace`]），payload 保持精简。

use iceoryx2::prelude::*;

use crate::publish::Publisher;
use crate::subscriber::{Received, Subscriber};
use crate::trace::TraceContext;

/// imu 话题名（对照 `docs/architecture.md` 的 `topic: imu`）。
pub const IMU_TOPIC: &str = "Firefly/Imu";

/// IMU 消息。
///
/// 字段与 `firefly-vio-core` 的 [`ImuData`](firefly_vio_core::sensor::ImuData)
/// 输出对应：角速度（陀螺仪，rad/s）+ 比力（加速度计，m/s²）。
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

/// 收到的 imu 样本。
pub type ReceivedImu = Received<ImuMessage>;

/// imu 发布器（话题 `Firefly/Imu`，泛型核心的命名封装）。
pub struct ImuPublisher(Publisher<ImuMessage>);

impl ImuPublisher {
    /// 打开 imu 话题的发布器。
    ///
    /// # Errors
    /// 见 [`Publisher::with_topic`]。
    pub fn new() -> Result<Self, firefly_error::Error> {
        Self::with_topic(IMU_TOPIC)
    }

    /// 以自定义话题名打开 imu 发布器。
    ///
    /// # Errors
    /// 见 [`Publisher::with_topic`]。
    pub fn with_topic(topic: &str) -> Result<Self, firefly_error::Error> {
        Ok(Self(Publisher::with_topic(topic)?))
    }

    /// 发布一条 imu 消息（trace 上下文自动注入，见 [`Publisher::publish`]）。
    ///
    /// # Errors
    /// 见 [`Publisher::publish`]。
    pub fn publish(&self, msg: ImuMessage) -> Result<TraceContext, firefly_error::Error> {
        self.0.publish(msg)
    }
}

/// imu 订阅器（话题 `Firefly/Imu`，泛型核心的命名封装）。
pub struct ImuSubscriber(Subscriber<ImuMessage>);

impl ImuSubscriber {
    /// 打开 imu 话题的订阅器。
    ///
    /// # Errors
    /// 见 [`Subscriber::with_topic`]。
    pub fn new() -> Result<Self, firefly_error::Error> {
        Self::with_topic(IMU_TOPIC)
    }

    /// 以自定义话题名打开 imu 订阅器。
    ///
    /// # Errors
    /// 见 [`Subscriber::with_topic`]。
    pub fn with_topic(topic: &str) -> Result<Self, firefly_error::Error> {
        Ok(Self(Subscriber::with_topic(topic)?))
    }

    /// 接收一条 imu 消息（见 [`Subscriber::receive`]）。
    ///
    /// # Errors
    /// 见 [`Subscriber::receive`]。
    pub fn receive(&self) -> Result<Option<ReceivedImu>, firefly_error::Error> {
        self.0.receive()
    }
}

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
}
