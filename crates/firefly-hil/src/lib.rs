//! USB HIL 帧与 `FunctionFS` 接口；线上编码、消息方向与时间域见 [`protocol`]。

/// 厂商自定义接口类。
pub const INTERFACE_CLASS: u8 = 0xFF;
pub const HIL_SUBCLASS: u8 = 1;

pub mod frame;
pub mod ingress;
pub mod protocol;
pub mod topics;

#[cfg(target_os = "linux")]
pub mod gadget;

pub use frame::{FrameReader, FrameWriter};
pub use protocol::{Header, Kind, Link, Message, Stats};
