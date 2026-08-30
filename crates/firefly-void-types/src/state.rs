//! 19 维状态流形 `M ≜ SO(3)×R¹⁶` 与 boxplus/boxminus（论文 (1)-(3) 式）。
//!
//! 状态构成 `x = [^G R_I, ^G p_I, ^G v_I, b_g, b_a, ^G g, τ]`，
//! 旋转用右手系 `Rotation3`（Hamilton 约定，`R = R_cur · Exp(δω)` 右乘扰动）。
//!
//! boxplus/boxminus 定义对照官方 `common_lib.h:167`（`operator+`）与
//! `common_lib.h:194`（`operator-`）：
//! - 旋转分量：`⊞` 用右乘 `Exp`，`⊟` 用 `Log(R_bᵀ R_a)`；
//! - 其余 16 维欧氏分量：`⊞`/`⊟` 即向量加减。

use nalgebra::{Rotation3, Vector3};

use crate::so3::{exp, log};

/// 状态维数（`Dim(SO(3))=3` + 16，对照 `common_lib.h:30` `DIM_STATE`）。
pub const DIM_STATE: usize = 19;

/// 19×19 状态协方差矩阵（单位 m·rad/s·m/s 混合，随分量语义）。
pub type StateCovariance = nalgebra::SMatrix<f64, DIM_STATE, DIM_STATE>;

/// 19 维误差状态向量（`x_a ⊟ x_b` 的取值空间，见 [`State::boxminus`]）。
pub type ErrorState = nalgebra::SVector<f64, DIM_STATE>;

/// 状态流形上的一个点。
///
/// 字段顺序与官方 `StatesGroup`（`common_lib.h:126`）一致：
/// `rot_end, pos_end, vel_end, bias_g, bias_a, gravity, inv_expo_time`。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    /// `^G R_I`：IMU 姿态（世界系 → 机体系旋转矩阵）。
    pub rot: Rotation3<f64>,
    /// `^G p_I`：IMU 位置（世界系，单位 m）。
    pub pos: Vector3<f64>,
    /// `^G v_I`：IMU 速度（世界系，单位 m/s）。
    pub vel: Vector3<f64>,
    /// `b_g`：陀螺仪零偏（机体系，单位 rad/s）。
    pub bias_g: Vector3<f64>,
    /// `b_a`：加速度计零偏（机体系，单位 m/s²）。
    pub bias_a: Vector3<f64>,
    /// `^G g`：重力向量（世界系，单位 m/s²）。
    pub gravity: Vector3<f64>,
    /// `τ`：逆曝光时间（相对首帧，无单位，`τ = 1/t_exposure`）。
    pub inv_expo_time: f64,
    /// 状态协方差 `P`。
    pub cov: StateCovariance,
}

impl Default for State {
    /// 默认状态：姿态单位阵、其余为零、τ=1；
    /// 协方差初始化为官方 `StatesGroup` 构造（`common_lib.h:137`）：
    /// 全对角 `0.01`，速度方差 `1e-5`，bias/重力块对角 `1e-5`。
    fn default() -> Self {
        let mut cov = StateCovariance::identity() * 0.01;
        cov[(6, 6)] = 0.00001;
        for i in 10..19 {
            cov[(i, i)] = 0.00001;
        }
        Self {
            rot: Rotation3::identity(),
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
            bias_g: Vector3::zeros(),
            bias_a: Vector3::zeros(),
            gravity: Vector3::zeros(),
            inv_expo_time: 1.0,
            cov,
        }
    }
}

impl State {
    /// boxplus：`x ⊞ δx`，旋转右乘扰动，其余分量欧氏相加（对照 `common_lib.h:167`）。
    ///
    /// 保持旋转与协方差不变（协方差不随状态更新改变，由 EKF 递推）。
    #[must_use]
    pub fn boxplus(&self, delta: &ErrorState) -> Self {
        let rot_delta = Vector3::new(delta[0], delta[1], delta[2]);
        Self {
            rot: self.rot * Rotation3::from_matrix_unchecked(exp(rot_delta)),
            pos: self.pos + delta.fixed_rows::<3>(3),
            vel: self.vel + delta.fixed_rows::<3>(7),
            bias_g: self.bias_g + delta.fixed_rows::<3>(10),
            bias_a: self.bias_a + delta.fixed_rows::<3>(13),
            gravity: self.gravity + delta.fixed_rows::<3>(16),
            inv_expo_time: self.inv_expo_time + delta[6],
            cov: self.cov,
        }
    }

    /// boxminus：`x_a ⊟ x_b`，旋转差用 `Log(R_bᵀ R_a)`（对照 `common_lib.h:194`）。
    #[must_use]
    pub fn boxminus(&self, other: &Self) -> ErrorState {
        let rotd = other.rot.inverse() * self.rot;
        let mut delta = ErrorState::zeros();
        delta.fixed_rows_mut::<3>(0).copy_from(&log(rotd.matrix()));
        delta
            .fixed_rows_mut::<3>(3)
            .copy_from(&(self.pos - other.pos));
        delta[6] = self.inv_expo_time - other.inv_expo_time;
        delta
            .fixed_rows_mut::<3>(7)
            .copy_from(&(self.vel - other.vel));
        delta
            .fixed_rows_mut::<3>(10)
            .copy_from(&(self.bias_g - other.bias_g));
        delta
            .fixed_rows_mut::<3>(13)
            .copy_from(&(self.bias_a - other.bias_a));
        delta
            .fixed_rows_mut::<3>(16)
            .copy_from(&(self.gravity - other.gravity));
        delta
    }

    /// 状态转移 `x ← x ⊞ (dt·f(x,u,0))`（论文 (1)(2) 式离散化）。
    ///
    /// `f` 对照论文 (2) 式与官方 `prop_imu_once`（`LIVMapper.cpp:556`）：
    /// 姿态用角速度的零阶保持（`Exp(ω_avr·dt)`），位置用当前速度与
    /// 世界系加速度的梯形/矩形积分，bias 与重力保持，τ 为随机游走。
    ///
    /// 返回增量 `dt·f(x,u,0)`，供调用方做协方差传播的线性化参考。
    #[must_use]
    pub fn predict(&self, omega: Vector3<f64>, acc: Vector3<f64>, dt: f64) -> ErrorState {
        let omega_unbiased = omega - self.bias_g;
        let acc_unbiased = acc - self.bias_a;
        let acc_world = self.rot * acc_unbiased + self.gravity;
        let mut delta = ErrorState::zeros();
        delta
            .fixed_rows_mut::<3>(0)
            .copy_from(&(omega_unbiased * dt));
        delta
            .fixed_rows_mut::<3>(3)
            .copy_from(&(self.vel * dt + 0.5 * acc_world * dt * dt));
        delta.fixed_rows_mut::<3>(7).copy_from(&(acc_world * dt));
        delta
    }

    /// boxplus 原地版本：`self ← self ⊞ δx`（对照 `common_lib.h:182` `operator+=`）。
    pub fn boxplus_assign(&mut self, delta: &ErrorState) {
        *self = self.boxplus(delta);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Matrix3;

    fn assert_vec_close(a: Vector3<f64>, b: Vector3<f64>, eps: f64) {
        assert!((a - b).norm() < eps, "{a} != {b}");
    }

    fn assert_mat_close(a: &Matrix3<f64>, b: &Matrix3<f64>, eps: f64) {
        for i in 0..3 {
            for j in 0..3 {
                assert!((a[(i, j)] - b[(i, j)]).abs() < eps, "{a} != {b}");
            }
        }
    }

    #[test]
    fn boxplus_rotation_right_multiplication() {
        // 右乘扰动：⊞ 后 rot = rot · Exp(δω)
        let x = State {
            rot: Rotation3::from_axis_angle(&Vector3::z_axis(), 0.5),
            ..State::default()
        };
        let dw = Vector3::new(0.0, 0.0, 0.3);
        let mut delta = ErrorState::zeros();
        delta.fixed_rows_mut::<3>(0).copy_from(&dw);
        let y = x.boxplus(&delta);
        let expect = x.rot * Rotation3::from_matrix_unchecked(exp(dw));
        assert_mat_close(y.rot.matrix(), expect.matrix(), 1e-12);
    }

    #[test]
    fn boxplus_boxminus_inverse() {
        // 互逆性：x ⊞ δx ⊟ x == δx（对任意旋转扰动）
        for w in [
            Vector3::new(0.1, -0.2, 0.3),
            Vector3::new(1.0, 0.5, -0.8),
            Vector3::new(1.4, 0.1, -1.2),
        ] {
            let x = State::default();
            let mut delta = ErrorState::zeros();
            delta.fixed_rows_mut::<3>(0).copy_from(&w);
            delta
                .fixed_rows_mut::<3>(3)
                .copy_from(&Vector3::new(1.0, -2.0, 0.5));
            delta[6] = 0.2;
            delta
                .fixed_rows_mut::<3>(7)
                .copy_from(&Vector3::new(0.3, 0.1, -0.4));
            delta
                .fixed_rows_mut::<3>(10)
                .copy_from(&Vector3::new(0.01, -0.02, 0.03));
            delta
                .fixed_rows_mut::<3>(13)
                .copy_from(&Vector3::new(-0.1, 0.2, -0.3));
            delta
                .fixed_rows_mut::<3>(16)
                .copy_from(&Vector3::new(0.0, 0.0, -9.8));
            let y = x.boxplus(&delta);
            let back = y.boxminus(&x);
            let err = (back - delta).norm();
            assert!(err < 1e-9, "roundtrip err={err}");
        }
    }

    #[test]
    fn boxminus_translation_components() {
        // 欧氏分量直接相减
        let a = State {
            pos: Vector3::new(1.0, 2.0, 3.0),
            vel: Vector3::new(4.0, 5.0, 6.0),
            ..State::default()
        };
        let b = State {
            pos: Vector3::new(0.5, 1.0, 2.0),
            vel: Vector3::new(1.0, 1.0, 1.0),
            ..State::default()
        };
        let d = a.boxminus(&b);
        assert_vec_close(
            d.fixed_rows::<3>(3).into_owned(),
            Vector3::new(0.5, 1.0, 1.0),
            1e-12,
        );
        assert_vec_close(
            d.fixed_rows::<3>(7).into_owned(),
            Vector3::new(3.0, 4.0, 5.0),
            1e-12,
        );
    }

    #[test]
    fn default_covariance_blocks() {
        // 对照 common_lib.h:137 的初始化
        let x = State::default();
        assert!((x.cov[(0, 0)] - 0.01).abs() < 1e-15);
        assert!((x.cov[(5, 5)] - 0.01).abs() < 1e-15);
        assert!((x.cov[(6, 6)] - 0.00001).abs() < 1e-15);
        assert!((x.cov[(10, 10)] - 0.00001).abs() < 1e-15);
        assert!((x.cov[(18, 18)] - 0.00001).abs() < 1e-15);
    }

    #[test]
    fn predict_constant_velocity_flat() {
        // 恒速直线（无旋转、零加速度）：位置增量 = v·dt，速度不变
        let x = State {
            vel: Vector3::new(1.0, 0.0, 0.0),
            gravity: Vector3::new(0.0, 0.0, 0.0),
            ..State::default()
        };
        let dt = 0.1;
        let delta = x.predict(Vector3::zeros(), Vector3::zeros(), dt);
        assert_vec_close(
            delta.fixed_rows::<3>(3).into_owned(),
            Vector3::new(0.1, 0.0, 0.0),
            1e-12,
        );
        assert_vec_close(
            delta.fixed_rows::<3>(7).into_owned(),
            Vector3::zeros(),
            1e-12,
        );
        assert_vec_close(
            delta.fixed_rows::<3>(0).into_owned(),
            Vector3::zeros(),
            1e-12,
        );
    }
}
