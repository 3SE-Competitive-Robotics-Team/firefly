//! 初始化器（对照 `OpenVINS` `ov_init`）。
//!
//! - [`InertialInitializer`]：编排静态/动态初始化（对照 `InertialInitializer`）；
//! - [`StaticInitializer`]：静态初始化（静止/急动检测，对照 `StaticInitializer`）；
//! - [`DynamicInitializer`]：动态初始化（线性系统 + 高斯牛顿 MLE，对照
//!   `DynamicInitializer`，Ceres 由自研 GN 优化器替代）；
//! - [`CpiV1`]：连续预积分（对照 `ov_core::CpiV1`）；
//! - [`InitializerHelper`]：插值/读数选择/董氏系数。
//!
//! 结果以纯数据形式返回（[`InitResult`]），由 `VioManager` 组装进 `State`。

pub mod cpi_v1;
pub mod dynamic_init;
pub mod helper;
pub mod inertial_init;
pub mod options;
pub mod static_init;

use firefly_vio_types::var::PoseJpl;
use nalgebra::{DMatrix, Vector3};

/// 初始化结果（对照 C++ `initialize` 的 out 参数：
/// `timestamp`/`covariance`/`order`/`t_imu`/`_clones_IMU`/`_features_SLAM`）。
///
/// `order` 为协方差块对应的 `(id, size)` 列表（本实现中仅含 IMU 15 维）。
#[derive(Debug, Clone)]
pub struct InitResult {
    /// 初始化完成时刻（相机时钟系；对照 C++ 的 `timestamp`）。
    pub timestamp: f64,
    /// IMU 状态协方差（15×15；静态为固定先验，动态为 MLE 恢复并膨胀）。
    pub covariance: DMatrix<f64>,
    /// 协方差块顺序（对照 C++ 的 `order`，通常为 `[(imu_id, 15)]`）。
    pub order: Vec<(i32, usize)>,
    /// IMU 状态 `[q_GtoI(4), p_IinG(3), v_IinG(3), bg(3), ba(3)]`（16 维）。
    pub imu_state: [f64; 16],
    /// 动态初始化恢复的 IMU 克隆（时刻 → 位姿；对照 `_clones_IMU`）。
    pub clones_imu: Vec<(f64, PoseJpl)>,
    /// 动态初始化恢复的 SLAM 特征（featid → 全局 3D 位置；对照
    /// `_features_SLAM`，表示恒为 `GLOBAL_3D`）。
    pub features_slam: Vec<(usize, Vector3<f64>)>,
}
