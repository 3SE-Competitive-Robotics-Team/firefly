//! MSCKF 编排层（对照 `OpenVINS` `ov_msckf`）。
//!
//! `State`/`StateHelper` 管理滑动窗口与协方差，`UpdaterMSCKF` 做视觉更新，
//! `VioManager` 编排 IMU/相机输入。由 firefly-vio 移植成员实现。

pub mod landmark;
pub mod options;
pub mod state;
pub mod state_helper;
pub mod updater;
pub mod updater_helper;
pub mod updater_slam;
pub mod updater_zero_velocity;
pub mod vio_manager;
