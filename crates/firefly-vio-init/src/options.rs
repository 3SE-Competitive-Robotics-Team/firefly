//! 初始化器选项（对照 `OpenVINS` `ov_init/src/init/InertialInitializerOptions.h`）。
//!
//! 只移植 struct + 默认值；YAML 解析属于 apps 层职责（与 `firefly-vio` 的
//! 选项模块同一约定）。

use firefly_vio_core::cam::SharedCamera;
use nalgebra::{SVector, Vector3};
use std::collections::BTreeMap;

/// 初始化器选项（对照 `InertialInitializerOptions`，默认值与其成员初始化一致）。
// 与 C++ 结构 1:1 移植（大量布尔/数值开关），拆分反而破坏对照可审计性。
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone)]
pub struct InitOptions {
    /// 初始化窗口时长（秒；对照 `init_window_time`）。
    pub init_window_time: f64,
    /// 加速度方差阈值，超过视为移动（对照 `init_imu_thresh`）。
    pub init_imu_thresh: f64,
    /// 平均视差阈值，超过视为移动（对照 `init_max_disparity`）。
    pub init_max_disparity: f64,
    /// 初始化需要的最少特征数（对照 `init_max_features`）。
    pub init_max_features: usize,
    /// 是否使用动态初始化器（对照 `init_dyn_use`）。
    pub init_dyn_use: bool,
    /// 是否在 MLE 中优化标定（对照 `init_dyn_mle_opt_calib`）。
    pub init_dyn_mle_opt_calib: bool,
    /// MLE 最大迭代次数（对照 `init_dyn_mle_max_iter`）。
    pub init_dyn_mle_max_iter: usize,
    /// MLE 优化最大时间（秒；对照 `init_dyn_mle_max_time`）。
    pub init_dyn_mle_max_time: f64,
    /// 动态初始化使用的位姿数（对照 `init_dyn_num_pose`）。
    pub init_dyn_num_pose: usize,
    /// 动态初始化最少旋转量（度；对照 `init_dyn_min_deg`）。
    pub init_dyn_min_deg: f64,
    /// 协方差膨胀：姿态（对照 `init_dyn_inflation_orientation`）。
    pub init_dyn_inflation_orientation: f64,
    /// 协方差膨胀：速度（对照 `init_dyn_inflation_velocity`）。
    pub init_dyn_inflation_velocity: f64,
    /// 协方差膨胀：陀螺偏置（对照 `init_dyn_inflation_bias_gyro`）。
    pub init_dyn_inflation_bias_gyro: f64,
    /// 协方差膨胀：加速度计偏置（对照 `init_dyn_inflation_bias_accel`）。
    pub init_dyn_inflation_bias_accel: f64,
    /// 协方差恢复的最小倒数条件数（对照 `init_dyn_min_rec_cond`）。
    pub init_dyn_min_rec_cond: f64,
    /// 动态初始化陀螺偏置初值（对照 `init_dyn_bias_g`）。
    pub init_dyn_bias_g: Vector3<f64>,
    /// 动态初始化加速度计偏置初值（对照 `init_dyn_bias_a`）。
    pub init_dyn_bias_a: Vector3<f64>,

    /// 陀螺白噪声密度（对照 `sigma_w`）。
    pub sigma_w: f64,
    /// 陀螺随机游走（对照 `sigma_wb`）。
    pub sigma_wb: f64,
    /// 加速度计白噪声密度（对照 `sigma_a`）。
    pub sigma_a: f64,
    /// 加速度计随机游走（对照 `sigma_ab`）。
    pub sigma_ab: f64,
    /// 像素噪声 sigma（对照 `sigma_pix`）。
    pub sigma_pix: f64,

    /// 重力加速度大小（对照 `gravity_mag`）。
    pub gravity_mag: f64,
    /// 是否使用双目（对照 `use_stereo`）。
    pub use_stereo: bool,
    /// 相机-IMU 时间偏移（对照 `calib_camimu_dt`）。
    pub calib_camimu_dt: f64,

    /// 相机内参（id → 畸变模型；对照 `camera_intrinsics`）。
    pub camera_intrinsics: BTreeMap<usize, SharedCamera>,
    /// 相机外参 `[q_ItoC(4); p_IinC(3)]`（对照 `camera_extrinsics`）。
    pub camera_extrinsics: BTreeMap<usize, SVector<f64, 7>>,
}

impl Default for InitOptions {
    fn default() -> Self {
        Self {
            init_window_time: 1.0,
            init_imu_thresh: 1.0,
            init_max_disparity: 1.0,
            init_max_features: 50,
            init_dyn_use: false,
            init_dyn_mle_opt_calib: false,
            init_dyn_mle_max_iter: 20,
            init_dyn_mle_max_time: 5.0,
            init_dyn_num_pose: 5,
            init_dyn_min_deg: 45.0,
            init_dyn_inflation_orientation: 10.0,
            init_dyn_inflation_velocity: 10.0,
            init_dyn_inflation_bias_gyro: 100.0,
            init_dyn_inflation_bias_accel: 100.0,
            init_dyn_min_rec_cond: 1e-15,
            init_dyn_bias_g: Vector3::zeros(),
            init_dyn_bias_a: Vector3::zeros(),
            sigma_w: 1.6968e-4,
            sigma_wb: 1.9393e-5,
            sigma_a: 2.0000e-3,
            sigma_ab: 3.0000e-3,
            sigma_pix: 1.0,
            gravity_mag: 9.81,
            use_stereo: true,
            calib_camimu_dt: 0.0,
            camera_intrinsics: BTreeMap::new(),
            camera_extrinsics: BTreeMap::new(),
        }
    }
}
