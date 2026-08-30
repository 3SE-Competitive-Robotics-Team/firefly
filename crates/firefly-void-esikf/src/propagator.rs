//! IMU 前向传播与扫描内后向传播（对照 `src/IMU_Processing.cpp` 的
//! `UndistortPcl` 与论文第 IV-C 节）。
//!
//! 前向传播逐 IMU 输入推进状态与 19×19 协方差，F/Q 阵离散化与官方
//! `IMU_Processing.cpp:385-401` 一致；后向传播沿扫描内 IMU 位姿序列
//! 反向积分，用于深度点云运动补偿（官方 backward propagation）。

use firefly_void_types::so3::{exp_dt, skew};
use firefly_void_types::state::{State, StateCovariance};
use nalgebra::{Matrix3, Rotation3, Vector3};

/// 传播噪声参数（对照 `ImuProcess` 构造，`IMU_Processing.cpp:19-23`）。
///
/// 各量为对角协方差标量；`cov_bias_*` 驱动 bias 随机游走。
#[derive(Debug, Clone, Copy)]
pub struct PropagationNoise {
    /// 陀螺仪测量噪声方差 `cov_gyr`（(rad/s)²）。
    pub gyr: f64,
    /// 加速度计测量噪声方差 `cov_acc`（(m/s²)²）。
    pub acc: f64,
    /// 陀螺零偏随机游走方差 `cov_bias_gyr`（(rad/s²)²）。
    pub bias_gyr: f64,
    /// 加速度零偏随机游走方差 `cov_bias_acc`（(m/s²·s)²）。
    pub bias_acc: f64,
    /// 逆曝光时间随机游走方差 `cov_inv_expo`（1/s²）。
    pub inv_expo: f64,
}

impl Default for PropagationNoise {
    fn default() -> Self {
        Self {
            gyr: 0.1,
            acc: 0.1,
            bias_gyr: 0.1,
            bias_acc: 0.1,
            inv_expo: 0.2,
        }
    }
}

/// 传播器：前向传播 + 后向传播（无内部缓冲，缓冲由调用方管理）。
#[derive(Debug, Clone, Copy)]
pub struct Propagator {
    noise: PropagationNoise,
    /// `^G g` 估计开关（对照 `disable_gravity_est`，关闭时 `F_x` 不置重力块）。
    gravity_est: bool,
    /// `b_a`/`b_g` 估计开关（对照 `disable_bias_est`）。
    bias_est: bool,
    /// τ 估计开关（对照 `disable_exposure_est`，关闭时 Q 不置 τ 块）。
    exposure_est: bool,
}

impl Default for Propagator {
    fn default() -> Self {
        Self::new()
    }
}

impl Propagator {
    /// 构造：默认开启重力/bias/曝光估计（对照官方默认配置）。
    #[must_use]
    pub fn new() -> Self {
        Self {
            noise: PropagationNoise::default(),
            gravity_est: true,
            bias_est: true,
            exposure_est: true,
        }
    }

    /// 设置传播噪声参数。
    #[must_use]
    pub fn with_noise(mut self, noise: PropagationNoise) -> Self {
        self.noise = noise;
        self
    }

    /// 关闭重力估计（对照 `disable_gravity_est`）。
    pub fn disable_gravity_est(&mut self) {
        self.gravity_est = false;
    }

    /// 关闭 bias 估计（对照 `disable_bias_est`）。
    pub fn disable_bias_est(&mut self) {
        self.bias_est = false;
    }

    /// 关闭曝光时间估计（对照 `disable_exposure_est`）。
    pub fn disable_exposure_est(&mut self) {
        self.exposure_est = false;
    }

    /// 前向传播：按一个 IMU 步长推进状态与协方差。
    ///
    /// 输入 `omega`/`acc` 为两帧 IMU 的均值（梯形积分，对照
    /// `IMU_Processing.cpp:334-341`）；`dt` 为步长（秒）。
    /// 状态更新对照 `prop_imu_once`（`LIVMapper.cpp:556`）：
    /// 姿态 `R ← R·Exp(ω_avr·dt)`，加速度 `a_w = R·a_avr + ^G g`，
    /// 位置 `p += v·dt + ½·a_w·dt²`，速度 `v += a_w·dt`。
    #[fastrace::trace]
    pub fn propagate(
        &self,
        state: &mut State,
        omega_avr: Vector3<f64>,
        acc_avr: Vector3<f64>,
        dt: f64,
    ) {
        let omega_unbiased = omega_avr - state.bias_g;
        let acc_unbiased = acc_avr - state.bias_a;

        // 协方差传播：F_x 离散化对照 IMU_Processing.cpp:385-391，
        // Q 对照 IMU_Processing.cpp:395-399
        let f_x = self.discretized_f(state.rot.matrix(), omega_unbiased, acc_unbiased, dt);
        let cov_w = self.process_noise(state.rot.matrix(), dt);
        state.cov = f_x * state.cov * f_x.transpose() + cov_w;

        // 状态传播（对照 prop_imu_once）
        let rot_f = exp_dt(omega_unbiased, dt);
        state.rot *= Rotation3::from_matrix_unchecked(rot_f);
        let acc_world = state.rot * acc_unbiased + state.gravity;
        state.pos += state.vel * dt + 0.5 * acc_world * dt * dt;
        state.vel += acc_world * dt;
    }

    /// 后向传播：从 `end` 到 `start` 反向积分（用于深度点云运动补偿）。
    ///
    /// 官方 backward propagation（`IMU_Processing.cpp:494-539`）逐 IMU 位姿
    /// 反向推进；本函数给出纯 IMU 反向积分：给定时刻 `t`（位于区间内）的
    /// 位姿 `(R_t, p_t)`，由 `end` 状态反向求解。
    ///
    /// 对照公式（常加速度假设，与 forward 的矩形积分互逆）：
    /// `R_i = R_end·Exp(−ω_avr·Δt)`，`p_i = p_end − v_end·Δt + ½·a·Δt²`，
    /// `Δt = t_end − t`。
    ///
    /// 返回该时刻的姿态与位置。
    #[fastrace::trace]
    #[must_use]
    pub fn backward(
        &self,
        end: &State,
        omega_avr: Vector3<f64>,
        acc_avr: Vector3<f64>,
        t_end: f64,
        t: f64,
    ) -> (Matrix3<f64>, Vector3<f64>) {
        let dt = t_end - t;
        let omega_unbiased = omega_avr - end.bias_g;
        let acc_unbiased = acc_avr - end.bias_a;
        let acc_world = end.rot * acc_unbiased + end.gravity;
        let r_i = end.rot.matrix() * exp_dt(omega_unbiased, -dt);
        let p_i = end.pos - end.vel * dt + 0.5 * acc_world * dt * dt;
        (r_i, p_i)
    }

    /// 离散化 `F_x` 矩阵（19×19，对照 `IMU_Processing.cpp:385-391`）。
    ///
    /// 状态排序 `[R, p, τ, v, b_g, b_a, g]`（与 `common_lib.h:167` boxplus 一致）：
    /// - `F_x[0:3,0:3] = Exp(−ω·dt)`（旋转误差衰减）；
    /// - `F_x[0:3,10:13] = −I·dt`（`b_g` 对旋转误差）；
    /// - `F_x[3:6,7:10] = I·dt`（`v` 对位置误差）；
    /// - `F_x[7:10,0:3] = −R·⌊a×⌋·dt`（姿态误差对速度）；
    /// - `F_x[7:10,13:16] = −R·dt`（`b_a` 对速度）；
    /// - `F_x[7:10,16:19] = I·dt`（重力对速度，可配）。
    #[must_use]
    pub fn discretized_f(
        &self,
        rot: &Matrix3<f64>,
        omega_unbiased: Vector3<f64>,
        acc_unbiased: Vector3<f64>,
        dt: f64,
    ) -> StateCovariance {
        let mut f_x = StateCovariance::identity();
        f_x.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&exp_dt(omega_unbiased, -dt));
        if self.bias_est {
            f_x.fixed_view_mut::<3, 3>(0, 10).fill(0.0);
            for i in 0..3 {
                f_x[(i, 10 + i)] = -dt;
            }
        }
        f_x.fixed_view_mut::<3, 3>(3, 7).fill(0.0);
        for i in 0..3 {
            f_x[(3 + i, 7 + i)] = dt;
        }
        f_x.fixed_view_mut::<3, 3>(7, 0)
            .copy_from(&(-rot * skew(&acc_unbiased) * dt));
        if self.bias_est {
            f_x.fixed_view_mut::<3, 3>(7, 13).copy_from(&(-rot * dt));
        }
        if self.gravity_est {
            f_x.fixed_view_mut::<3, 3>(7, 16).fill(0.0);
            for i in 0..3 {
                f_x[(7 + i, 16 + i)] = dt;
            }
        }
        f_x
    }

    /// 过程噪声 Q 矩阵（19×19，对照 `IMU_Processing.cpp:395-399`）。
    ///
    /// - 陀螺/加速度零偏随机游走对角 `cov_gyr`/`cov_bias_*` 乘 `dt²`；
    /// - 加速度计噪声经 `R` 旋转到世界系（`R·diag(cov_acc)·Rᵀ·dt²`）；
    /// - τ 随机游走 `cov_inv_expo·dt²`（可配）。
    #[must_use]
    pub fn process_noise(&self, rot: &Matrix3<f64>, dt: f64) -> StateCovariance {
        let mut q = StateCovariance::zeros();
        for i in 0..3 {
            q[(i, i)] = self.noise.gyr * dt * dt;
            q[(10 + i, 10 + i)] = self.noise.bias_gyr * dt * dt;
            q[(13 + i, 13 + i)] = self.noise.bias_acc * dt * dt;
        }
        let acc_cov = self.noise.acc * dt * dt;
        q.fixed_view_mut::<3, 3>(7, 7)
            .copy_from(&(rot * (acc_cov * Matrix3::identity()) * rot.transpose()));
        if self.exposure_est {
            q[(6, 6)] = self.noise.inv_expo * dt * dt;
        }
        q
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_void_types::state::State;
    use nalgebra::{Rotation3, Vector3};

    fn assert_close(a: f64, b: f64, eps: f64) {
        assert!((a - b).abs() < eps, "{a} != {b}");
    }

    #[test]
    fn propagate_constant_angular_velocity_analytic() {
        // 恒定角速度 ω=(0,0,0.5)、零加速度：
        // R(t) = R0·Exp(ω·t)，p 沿世界系积分
        let p = Propagator::new();
        let mut s = State {
            vel: Vector3::new(1.0, 0.0, 0.0),
            ..State::default()
        };
        let omega = Vector3::new(0.0, 0.0, 0.5);
        let acc = Vector3::new(0.0, 0.0, 0.0);
        let dt = 0.02;
        let n_steps = 25;
        for _ in 0..n_steps {
            p.propagate(&mut s, omega, acc, dt);
        }
        let t = f64::from(n_steps) * dt;
        let r_expect = Rotation3::from_axis_angle(&Vector3::z_axis(), 0.5 * t);
        for i in 0..3 {
            for j in 0..3 {
                assert_close(s.rot.matrix()[(i, j)], r_expect.matrix()[(i, j)], 1e-9);
            }
        }
        // 位置 = v·t（世界系恒速），旋转不贡献加速度
        assert_close(s.pos[0], t, 1e-9);
        assert_close(s.pos[1], 0.0, 1e-9);
        assert_close(s.vel[0], 1.0, 1e-9);
    }

    #[test]
    fn propagate_constant_acceleration_analytic() {
        // 恒定世界系加速度 a_w = (0,0,1.0)：p(t) = ½·a·t²，v(t) = a·t
        let p = Propagator::new();
        let mut s = State::default();
        let omega = Vector3::zeros();
        let acc = Vector3::new(0.0, 0.0, 1.0);
        let dt = 0.01;
        let n_steps = 50;
        for _ in 0..n_steps {
            p.propagate(&mut s, omega, acc, dt);
        }
        let t = f64::from(n_steps) * dt;
        assert_close(s.vel[2], t, 1e-6);
        assert_close(s.pos[2], 0.5 * t * t, 1e-6);
    }

    #[test]
    fn propagate_covariance_symmetric_positive() {
        // 传播后协方差保持对称正定（对角随 Q 增长）
        let p = Propagator::new();
        let mut s = State::default();
        p.propagate(
            &mut s,
            Vector3::new(0.1, -0.2, 0.3),
            Vector3::new(0.0, 0.0, 9.8),
            0.02,
        );
        assert!((s.cov - s.cov.transpose()).norm() < 1e-12);
        for i in 0..19 {
            assert!(s.cov[(i, i)] > 0.0, "diag[{i}] not positive");
        }
        // 旋转协方差被过程噪声抬升
        assert!(s.cov[(0, 0)] > 0.01);
    }

    #[test]
    fn backward_matches_forward() {
        // 后向传播应与前向互逆（同一常加速度假设下）
        let p = Propagator::new();
        let mut s = State::default();
        let omega = Vector3::new(0.0, 0.0, 0.3);
        let acc = Vector3::new(0.0, 0.0, 2.0);
        let dt = 0.02;
        p.propagate(&mut s, omega, acc, dt);
        let (r_back, p_back) = p.backward(&s, omega, acc, dt, 0.0);
        assert_close(p_back[0], 0.0, 1e-9);
        assert_close(p_back[2], 0.0, 1e-6);
        // 旋转反向：R(0) ≈ I
        for i in 0..3 {
            for j in 0..3 {
                assert_close(r_back[(i, j)], if i == j { 1.0 } else { 0.0 }, 1e-9);
            }
        }
    }

    #[test]
    fn discretized_f_block_structure() {
        let p = Propagator::new();
        let rot = Matrix3::identity();
        let f = p.discretized_f(&rot, Vector3::zeros(), Vector3::new(0.0, 0.0, 9.8), 0.1);
        // 位置←速度块 = I·dt
        assert_close(f[(3, 7)], 0.1, 1e-12);
        // 速度←重力块 = I·dt
        assert_close(f[(7, 16)], 0.1, 1e-12);
        // 旋转←b_g 块 = −I·dt
        assert_close(f[(0, 10)], -0.1, 1e-12);
        // 速度←b_a 块 = −I·dt
        assert_close(f[(7, 13)], -0.1, 1e-12);
        // 其余块为零
        assert_close(f[(0, 3)], 0.0, 1e-12);
    }

    #[test]
    fn disable_switches_zero_blocks() {
        let mut p = Propagator::new();
        p.disable_gravity_est();
        p.disable_bias_est();
        p.disable_exposure_est();
        let rot = Matrix3::identity();
        let f = p.discretized_f(&rot, Vector3::zeros(), Vector3::new(0.0, 0.0, 9.8), 0.1);
        assert_close(f[(7, 16)], 0.0, 1e-12);
        assert_close(f[(0, 10)], 0.0, 1e-12);
        assert_close(f[(7, 13)], 0.0, 1e-12);
        let q = p.process_noise(&rot, 0.1);
        assert_close(q[(6, 6)], 0.0, 1e-12);
    }
}
