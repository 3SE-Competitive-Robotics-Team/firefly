//! IMU 消息与 imu 话题（对照 `docs/architecture.md` 的 `topic: imu`）。
//!
//! `#[repr(C)]` 定长结构，满足 `ZeroCopySend` 约束。trace 上下文由发布
//! 中间件写入 **User Header**（见 [`crate::trace`]），payload 保持精简。

use iceoryx2::prelude::*;

use crate::node::IpcNode;
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

/// IMU 订阅缓冲区深度：覆盖约 1s 排空停顿（`VIO` 前端偶发数百 ms 阻塞；
/// 旧值 20 恰等于断流告警阈值 200ms，零余量，溢出即永久丢数）。
/// 服务上限见 `IMU_SERVICE_MAX`（本文件创建服务时显式声明，不依赖机器全局配置）。
const IMU_BUFFER_SIZE: usize = 100;
/// IMU 服务订阅端上限：须 ≥ 订阅缓冲（`iceoryx2` 建服务方定上限，后续
/// `open` 不得超限；`sim` 先起时由 Python 侧以同值创建，见 `main.py`）。
const IMU_SERVICE_MAX: usize = 128;

/// imu 订阅器（话题 `Firefly/Imu`，泛型核心的命名封装）。
pub struct ImuSubscriber(Subscriber<ImuMessage>);

impl ImuSubscriber {
    /// 打开 imu 话题的订阅器（`buffer_size`=20，覆盖 10 帧 IMU 数据）。
    ///
    /// # Errors
    /// 见 [`Subscriber::with_topic_and_buffer`]。
    pub fn new(node: &IpcNode) -> Result<Self, firefly_error::Error> {
        Self::with_topic(node, IMU_TOPIC)
    }

    /// 以自定义话题名打开 imu 订阅器（服务上限 `IMU_SERVICE_MAX`，订阅缓冲
    /// `IMU_BUFFER_SIZE`；先创建方定上限，`sim`/`vio` 任一先起皆一致）。
    ///
    /// # Errors
    /// 服务或订阅器创建失败（IPC 资源不可用等）。
    pub fn with_topic(node: &IpcNode, topic: &str) -> Result<Self, firefly_error::Error> {
        let service = node
            .service_builder(&topic.try_into().map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::InvalidArgument,
                    format!("非法话题名 `{topic}`: {e:?}"),
                )
            })?)
            .publish_subscribe::<ImuMessage>()
            .user_header::<TraceContext>()
            .subscriber_max_buffer_size(IMU_SERVICE_MAX)
            .open_or_create()
            .map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("打开/创建话题 `{topic}` 失败: {e:?}"),
                )
            })?;
        let subscriber = service
            .subscriber_builder()
            .buffer_size(IMU_BUFFER_SIZE)
            .create()
            .map_err(|e| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    format!("创建订阅器失败: {e:?}"),
                )
            })?;
        Ok(Self(Subscriber::from_inner(subscriber)))
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

    /// 缓冲不变量（编译期）：订阅缓冲 ≤ 服务上限（超限则创建直接失败，见
    /// `with_topic`），且覆盖 100Hz 下 1s 排空停顿。
    const _: () = {
        assert!(IMU_BUFFER_SIZE <= IMU_SERVICE_MAX);
        assert!(IMU_BUFFER_SIZE >= 100);
    };
}
