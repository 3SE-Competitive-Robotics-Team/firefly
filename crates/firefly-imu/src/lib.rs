//! FC 姿态领域：Hamilton body→odom，右误差 `[δθ, δbg]`，SI 单位。
//! 原始 IMU 同时供 VIO 消费；此库不拥有线程、IPC 或控制器。

mod eskf;
mod stream;

pub use eskf::{AttitudeState, Gate, Matrix6, gravity_jacobian, right_jacobian, transition};
use firefly_error::{Error, ErrorKind, Result};
use serde::Deserialize;
pub use stream::{Aid, Estimator, Sample};

/// 噪声密度为连续时间标准差；方差按实际测量间隔离散化。
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(default)]
pub struct Options {
    /// rad/s/√Hz，MuJoCo 100Hz 下每样本 0.002 rad/s。
    pub gyro_density: f64,
    /// rad/s²/√Hz，保守零偏随机游走模型；需由部署 IMU 标定。
    pub bias_walk: f64,
    /// m/s²/√Hz，MuJoCo 100Hz 下每样本 0.02 m/s²。
    pub accel_density: f64,
    /// m/s²，静止加计模型余量（安装振动与未估计的加计零偏）。
    pub accel_model_std: f64,
    /// m/s²，场景重力模长。
    pub gravity: f64,
    /// χ²(3) 的 99% 分位数；使用未归一化三轴加计创新。
    pub accel_nis_limit: f64,
    /// s，连续静止窗口。
    pub stationary_seconds: f64,
    /// rad/s，静止窗口每轴陀螺标准差上限。
    pub stationary_gyro_std: f64,
    /// m/s²，静止窗口每轴加计标准差上限。
    pub stationary_accel_std: f64,
    /// rad/s，静止窗口陀螺均值模长上限。
    pub stationary_gyro_max: f64,
    /// m/s²，静止加计模长相对 g 的最大偏差。
    pub stationary_accel_error: f64,
    /// s，最大可积分间隔；超限锁存不连续，必须重启估计器。
    pub max_dt: f64,
    /// Hz，角速度反馈一阶低通截止频率，按实际 dt 更新。
    pub rate_cutoff_hz: f64,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            gyro_density: 2e-4,
            bias_walk: 2e-5,
            accel_density: 2e-3,
            accel_model_std: 0.1,
            gravity: 9.81,
            accel_nis_limit: 11.344_866_730_144_373,
            stationary_seconds: 2.0,
            stationary_gyro_std: 0.005,
            stationary_accel_std: 0.1,
            stationary_gyro_max: 0.05,
            stationary_accel_error: 0.3,
            max_dt: 0.05,
            rate_cutoff_hz: 30.0,
        }
    }
}
impl Options {
    /// # Errors
    /// 所有数值必须有限且为正。
    pub fn validate(&self) -> Result<()> {
        if [
            self.gyro_density,
            self.bias_walk,
            self.accel_density,
            self.accel_model_std,
            self.gravity,
            self.accel_nis_limit,
            self.stationary_seconds,
            self.stationary_gyro_std,
            self.stationary_accel_std,
            self.stationary_gyro_max,
            self.stationary_accel_error,
            self.max_dt,
            self.rate_cutoff_hz,
        ]
        .iter()
        .any(|x| !x.is_finite() || *x <= 0.)
        {
            return Err(invalid("IMU options must be finite and positive"));
        }
        Ok(())
    }
}
fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message)
}
