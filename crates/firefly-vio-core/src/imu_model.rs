//! IMU 标定模型（对照 `OpenVINS` `ov_msckf/state/State.h` 的 `Dm`/`Tg`）。

use nalgebra::{Matrix3, SVector, Vector3, Vector6};

/// IMU 固有误差模型（对照 `StateOptions::ImuModel`）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ImuModel {
    /// kalibr 标定模型：尺度/错切按行主序填充。
    Kalibr,
    /// rpng 模型：尺度/错切按列主序填充。
    Rpng,
}

/// 尺度/错切矩阵 `D`（对照 `State::Dm`）。
///
/// 输入为 6 维向量（尺度 3 + 错切 3），两种模型布局不同：
/// - `Kalibr`：`[[v0,0,0],[v1,v3,0],[v2,v4,v5]]`
/// - `Rpng`：`[[v0,v1,v3],[0,v2,v4],[0,0,v5]]`
#[must_use]
pub fn dm(model: ImuModel, vec: &Vector6<f64>) -> Matrix3<f64> {
    match model {
        ImuModel::Kalibr => Matrix3::new(
            vec[0], 0.0, 0.0, vec[1], vec[3], 0.0, vec[2], vec[4], vec[5],
        ),
        ImuModel::Rpng => Matrix3::new(
            vec[0], vec[1], vec[3], 0.0, vec[2], vec[4], 0.0, 0.0, vec[5],
        ),
    }
}

/// 重力敏感矩阵 `Tg`（对照 `State::Tg`，列主序填充 3×3）。
#[must_use]
pub fn tg(vec: &SVector<f64, 9>) -> Matrix3<f64> {
    Matrix3::new(
        vec[0], vec[3], vec[6], vec[1], vec[4], vec[7], vec[2], vec[5], vec[8],
    )
}

/// IMU 标定参数（对照 `State` 的标定变量：偏置 + 旋转 + 尺度 + 重力敏感）。
#[derive(Debug, Clone)]
pub struct ImuCalibration {
    pub bias_a: Vector3<f64>,
    pub bias_g: Vector3<f64>,
    pub r_acc_to_imu: Matrix3<f64>,
    pub r_gyro_to_imu: Matrix3<f64>,
    pub da: Matrix3<f64>,
    pub dw: Matrix3<f64>,
    pub tg: Matrix3<f64>,
}

/// 传播时使用的校正后测量（对照 `Propagator::predict_and_compute` 的测量校正段）。
///
/// 加速度先减偏置，经 `R_ACCtoIMU · Da` 校正；角速度减偏置与重力敏感项
/// `Tg·a`，经 `R_GYROtoIMU · Dw` 校正。
#[derive(Debug, Clone, Copy)]
pub struct CorrectedImu {
    pub wm: Vector3<f64>,
    pub am: Vector3<f64>,
}

impl CorrectedImu {
    /// 校正原始 IMU 测量：加速度 `R_ACCtoIMU·Da·(am−ba)`，
    /// 角速度 `R_GYROtoIMU·Dw·(wm−bg−Tg·a)`。
    #[must_use]
    pub fn correct(raw: &ImuData, calib: &ImuCalibration) -> Self {
        let a_hat = calib.r_acc_to_imu * calib.da * (raw.am - calib.bias_a);
        let w_hat = calib.r_gyro_to_imu * calib.dw * (raw.wm - calib.bias_g - calib.tg * a_hat);
        Self {
            wm: w_hat,
            am: a_hat,
        }
    }
}

use crate::sensor::ImuData;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kalibr_dm_layout() {
        let v = Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let d = dm(ImuModel::Kalibr, &v);
        assert_eq!(d, Matrix3::new(1.0, 0.0, 0.0, 2.0, 4.0, 0.0, 3.0, 5.0, 6.0));
    }

    #[test]
    fn rpng_dm_layout() {
        let v = Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let d = dm(ImuModel::Rpng, &v);
        assert_eq!(d, Matrix3::new(1.0, 2.0, 4.0, 0.0, 3.0, 5.0, 0.0, 0.0, 6.0));
    }

    #[test]
    fn tg_column_major() {
        let v = nalgebra::vector![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let t = tg(&v);
        assert_eq!(t, Matrix3::new(1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0));
    }

    #[test]
    fn correct_applies_calibration() {
        let raw = ImuData {
            timestamp: 0.0,
            wm: Vector3::new(0.1, 0.2, 0.3),
            am: Vector3::new(1.0, 2.0, 3.0),
        };
        let calib = ImuCalibration {
            bias_a: Vector3::new(0.01, 0.02, 0.03),
            bias_g: Vector3::new(0.001, 0.002, 0.003),
            r_acc_to_imu: Matrix3::identity(),
            r_gyro_to_imu: Matrix3::identity(),
            da: Matrix3::identity(),
            dw: Matrix3::identity(),
            tg: Matrix3::zeros(),
        };
        let c = CorrectedImu::correct(&raw, &calib);
        assert_eq!(c.am, raw.am - calib.bias_a);
        assert_eq!(c.wm, raw.wm - calib.bias_g);
    }
}
