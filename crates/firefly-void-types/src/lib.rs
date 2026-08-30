//! DIVO 基础类型：状态流形、传感器数据结构与标定配置。
//!
//! 技术蓝本为 FAST-LIVO2（`~/Projects/fast_livo2/`）：
//! - 状态流形 [`state::State`]（19 维，SO(3)×R¹⁶）与 boxplus/boxminus；
//! - 传感器帧结构 [`sensor::ImuSample`] / [`sensor::CameraFrame`] / [`sensor::DepthFrame`]；
//! - 外参与标定配置 [`extrinsics::ExtrinsicsConfig`]；
//! - 视觉/标定基础类型 [`visual::GrayImage`] / [`visual::Intrinsics`] / [`visual::VisualState`]。
//!
//! 本 crate 只含纯数学与数据结构，不涉及估计与 IO。

pub mod extrinsics;
pub mod sensor;
pub mod so3;
pub mod state;
pub mod visual;
