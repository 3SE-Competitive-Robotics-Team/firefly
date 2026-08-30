//! 外参与标定配置（论文 Table I 与 DESIGN.md §3 的传感器适配）。
//!
//! 仿真设定：深度相机与左目共面（MuJoCo 同刚体），外参近似单位阵，
//! 参数保留可配；相机内参为 pinhole-radtan 模型（对照
//! `firefly-vio-core::cam::CamRadtan` 的接口风格，独立实现于 void 命名空间）。

use nalgebra::{Matrix3, Rotation3, Vector3};

/// 深度相机 → IMU 外参 `^I T_L`（平移单位 m，旋转为机体系旋转矩阵）。
#[derive(Debug, Clone, Copy)]
pub struct ExtrinsicsConfig {
    /// `^I T_L`：深度相机到 IMU 的平移（在 IMU 系下表示）。
    pub depth_to_imu_trans: Vector3<f64>,
    /// `^I T_L`：深度相机到 IMU 的旋转。
    pub depth_to_imu_rot: Rotation3<f64>,
    /// `^C T_I`：IMU 到相机（左目）的旋转。
    pub imu_to_cam_rot: Rotation3<f64>,
    /// 相机内参 `[f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2]`（radtan 顺序）。
    pub cam_intrinsics: [f64; 8],
    /// 相机图像宽度（像素）。
    pub cam_width: usize,
    /// 相机图像高度（像素）。
    pub cam_height: usize,
}

impl Default for ExtrinsicsConfig {
    /// 默认外参：深度相机与左目共面（单位阵），内参 320×240 基准。
    fn default() -> Self {
        Self {
            depth_to_imu_trans: Vector3::zeros(),
            depth_to_imu_rot: Rotation3::identity(),
            imu_to_cam_rot: Rotation3::identity(),
            cam_intrinsics: [300.0, 300.0, 160.0, 120.0, 0.0, 0.0, 0.0, 0.0],
            cam_width: 320,
            cam_height: 240,
        }
    }
}

impl ExtrinsicsConfig {
    /// 把深度相机系点变换到 IMU 系（`p_I = R_I←L · p_L + t_I←L`）。
    #[must_use]
    pub fn depth_to_imu(&self, p: Vector3<f64>) -> Vector3<f64> {
        self.depth_to_imu_rot * p + self.depth_to_imu_trans
    }

    /// 把 IMU 系点变换到相机（左目）系（`p_C = R_C←I · p_I`）。
    #[must_use]
    pub fn imu_to_cam(&self, p: Vector3<f64>) -> Vector3<f64> {
        self.imu_to_cam_rot * p
    }

    /// 相机内参矩阵 `K = [[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]]`。
    #[must_use]
    pub fn camera_matrix(&self) -> Matrix3<f64> {
        let k = self.cam_intrinsics;
        Matrix3::new(k[0], 0.0, k[2], 0.0, k[1], k[3], 0.0, 0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn default_is_identity_with_center_intrinsics() {
        let c = ExtrinsicsConfig::default();
        // 共面设定：深度点直接等于 IMU 系点
        let p = Vector3::new(1.0, -2.0, 3.0);
        assert!((c.depth_to_imu(p) - p).norm() < 1e-12);
        // 单位内参矩阵：主点居中、焦距 300
        let k = c.camera_matrix();
        assert!((k[(0, 0)] - 300.0).abs() < 1e-12);
        assert!((k[(1, 1)] - 300.0).abs() < 1e-12);
        assert!((k[(0, 2)] - 160.0).abs() < 1e-12);
        assert!((k[(1, 2)] - 120.0).abs() < 1e-12);
    }
}
