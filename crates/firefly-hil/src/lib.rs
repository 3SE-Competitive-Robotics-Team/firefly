//! USB HIL 链路：主机与板端之间的版本化帧协议、分片与 `FunctionFS` 端点描述。
//!
//! 单个厂商接口包含 OUT 下行（传感器与指令）和 IN 上行（控制与遥测）。主机侧用 `nusb` 访问 USB
//! 设备，板端用 Linux `FunctionFS` 暴露功能接口；两侧共用本 crate 的帧编解码，
//! 算法进程只看到既有 IPC 话题，不感知 USB。
//!
//! 时间约定：传感器与状态消息的时间戳统一为仿真时间轴；跨机新鲜度只在
//! 接收端用本机单调时钟判断，禁止比较两台机器的本地时钟。

/// 厂商自定义接口类。
pub const INTERFACE_CLASS: u8 = 0xFF;
/// HIL 接口子类。
pub const HIL_SUBCLASS: u8 = 1;

pub mod frame;
pub mod ingress;
pub mod protocol;
pub mod topics;

#[cfg(target_os = "linux")]
pub mod gadget;

pub use frame::{FrameReader, FrameWriter};
pub use protocol::{Header, Kind, Link, Message, Stats};
