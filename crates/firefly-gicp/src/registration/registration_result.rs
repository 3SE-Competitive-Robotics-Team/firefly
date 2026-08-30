//! 配准结果（对照 `registration/registration_result.hpp`）。

use nalgebra::{Matrix4, Matrix6, Vector6};

/// 配准结果（对照 `RegistrationResult`）。
#[derive(Clone, Debug)]
pub struct RegistrationResult {
    /// 估计变换 `T_target_source`。
    pub t_target_source: Matrix4<f64>,
    /// 迭代次数。
    pub iterations: usize,
    /// 是否收敛。
    pub converged: bool,
    /// 内点数。
    pub num_inliers: usize,
    /// 末次线性化的信息矩阵。
    pub h: Matrix6<f64>,
    /// 末次线性化的信息向量。
    pub b: Vector6<f64>,
    /// 末次误差。
    pub error: f64,
}

impl RegistrationResult {
    /// 以初值构造。
    pub fn new(t_init: Matrix4<f64>) -> Self {
        Self {
            t_target_source: t_init,
            iterations: 0,
            converged: false,
            num_inliers: 0,
            h: Matrix6::zeros(),
            b: Vector6::zeros(),
            error: 0.0,
        }
    }
}

impl Default for RegistrationResult {
    fn default() -> Self {
        Self::new(Matrix4::identity())
    }
}
