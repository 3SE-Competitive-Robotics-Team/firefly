//! PX4 三轴角速率 PID 控制器（`RateControl`）。
//!
//! 参考实现：PX4-Autopilot 提交 `4dbd2e069a5c30c2e53e47e842095d2576dc38c4`
//! 的 `src/lib/rate_control/rate_control.hpp` 与 `src/lib/rate_control/rate_control.cpp`
//! （BSD-3-Clause）。保持 PX4 的 NED 世界系 / FRD 机体系约定；力矩指令按各轴最大力矩归一化。
//!
//! 不移植：`rate_ctrl_status_s` 的 uORB 话题定义与发布——[`RateControl::get_rate_control_status`]
//! 把三个积分项写入调用方提供的 [`RateCtrlStatus`]。
//! 不移植：`RateControl() = default` / `~RateControl()`——Rust 用 [`Default`] 零初始化并由编译器析构。
//! 不移植：`using namespace matrix`——改用 `nalgebra` 类型。

// 字段名与 C++ 保持一致（含 `_` 前缀），并按要求使用 `types` 的 glob 导入。
#![allow(clippy::used_underscore_binding, clippy::wildcard_imports)]

use super::types::*;

/// 三轴角速率 PID 控制器。
///
/// 输出为归一化力矩指令 `[-1, 1]`（PX4 不在此处限幅，饱和由控制分配反馈）。
#[derive(Clone, Copy, Debug, Default)]
pub struct RateControl {
    /// 比例增益 [归一化力矩/(rad/s)]。
    _gain_p: Vector3f,
    /// 积分增益 [归一化力矩/rad]。
    _gain_i: Vector3f,
    /// 微分增益 [归一化力矩/(rad/s²)]。
    _gain_d: Vector3f,
    /// 积分项绝对值上限 [归一化力矩]。
    _lim_int: Vector3f,
    /// 角速率到力矩的直接前馈增益 [归一化力矩/(rad/s)]。
    _gain_ff: Vector3f,

    /// 积分项状态 [归一化力矩]。
    _rate_int: Vector3f,

    /// 控制分配反馈的负向饱和标志，轴序 roll/pitch/yaw。
    _control_allocator_saturation_negative: [bool; 3],
    /// 控制分配反馈的正向饱和标志，轴序 roll/pitch/yaw。
    _control_allocator_saturation_positive: [bool; 3],
}

impl RateControl {
    /// 设置 PID 增益。
    ///
    /// - `p`：比例增益 [归一化力矩/(rad/s)]
    /// - `i`：积分增益 [归一化力矩/rad]
    /// - `d`：微分增益 [归一化力矩/(rad/s²)]
    pub fn set_pid_gains(&mut self, p: Vector3f, i: Vector3f, d: Vector3f) {
        self._gain_p = p;
        self._gain_i = i;
        self._gain_d = d;
    }

    /// 设置所有轴的积分项上限 [归一化力矩]。
    pub fn set_integrator_limit(&mut self, integrator_limit: Vector3f) {
        self._lim_int = integrator_limit;
    }

    /// 设置角速率到力矩的直接前馈增益 [归一化力矩/(rad/s)]。
    pub fn set_feed_forward_gain(&mut self, ff: Vector3f) {
        self._gain_ff = ff;
    }

    /// 设置控制分配反馈的饱和状态（轴序 roll/pitch/yaw）。
    pub fn set_saturation_status(
        &mut self,
        saturation_positive: [bool; 3],
        saturation_negative: [bool; 3],
    ) {
        self._control_allocator_saturation_positive = saturation_positive;
        self._control_allocator_saturation_negative = saturation_negative;
    }

    /// 设置单轴正向饱和标志。
    ///
    /// `axis`：0 roll，1 pitch，2 yaw；越界忽略。
    pub fn set_positive_saturation_flag(&mut self, axis: usize, is_saturated: bool) {
        if axis < 3 {
            self._control_allocator_saturation_positive[axis] = is_saturated;
        }
    }

    /// 设置单轴负向饱和标志。
    ///
    /// `axis`：0 roll，1 pitch，2 yaw；越界忽略。
    pub fn set_negative_saturation_flag(&mut self, axis: usize, is_saturated: bool) {
        if axis < 3 {
            self._control_allocator_saturation_negative[axis] = is_saturated;
        }
    }

    /// 执行一个控制周期。
    ///
    /// - `rate` [rad/s]：当前机体角速率估计
    /// - `rate_sp` [rad/s]：期望机体角速率
    /// - `angular_accel` [rad/s²]：机体角加速度
    /// - `dt` [s]：控制周期
    /// - `landed`：着陆时冻结积分
    ///
    /// 返回归一化力矩指令 [−1, 1]。
    #[must_use]
    pub fn update(
        &mut self,
        rate: Vector3f,
        rate_sp: Vector3f,
        angular_accel: Vector3f,
        dt: f32,
        landed: bool,
    ) -> Vector3f {
        // 角速率误差
        let mut rate_error = rate_sp - rate;

        // 带前馈的 PID：积分项使用本周期积分前的值
        let torque = self._gain_p.emult(rate_error) + self._rate_int
            - self._gain_d.emult(angular_accel)
            + self._gain_ff.emult(rate_sp);

        // 未着陆时才积分
        if !landed {
            self.update_integral(&mut rate_error, dt);
        }

        torque
    }

    /// 积分项清零。
    pub fn reset_integral(&mut self) {
        self._rate_int = Vector3f::zeros();
    }

    /// 单轴积分项清零。
    ///
    /// `axis`：0 roll，1 pitch，2 yaw；越界忽略。对应 C++ 的 `resetIntegral(size_t)` 重载。
    pub fn reset_integral_axis(&mut self, axis: usize) {
        if axis < 3 {
            self._rate_int[axis] = 0.0;
        }
    }

    /// 读取积分项状态，供日志/诊断使用。
    pub fn get_rate_control_status(&self, rate_ctrl_status: &mut RateCtrlStatus) {
        rate_ctrl_status.rollspeed_integ = self._rate_int[0];
        rate_ctrl_status.pitchspeed_integ = self._rate_int[1];
        rate_ctrl_status.yawspeed_integ = self._rate_int[2];
    }

    /// 一阶积分更新，含抗饱和与随误差衰减的 `i_factor`。
    fn update_integral(&mut self, rate_error: &mut Vector3f, dt: f32) {
        for i in 0..3 {
            // 正向饱和时只允许负误差继续积分
            if self._control_allocator_saturation_positive[i] {
                rate_error[i] = rate_error[i].min(0.0);
            }

            // 负向饱和时只允许正误差继续积分
            if self._control_allocator_saturation_negative[i] {
                rate_error[i] = rate_error[i].max(0.0);
            }

            // 误差越大积分增益越小：400 °/s 误差内几乎无影响，200 °/s 时降到 75%
            let mut i_factor = rate_error[i] / radians(400.0);
            i_factor = (1.0 - i_factor * i_factor).max(0.0);

            // 一阶（矩形）积分
            let rate_i = self._rate_int[i] + i_factor * self._gain_i[i] * rate_error[i] * dt;

            // 非有限值或越界时不写回
            if is_finite(rate_i) {
                self._rate_int[i] = constrain(rate_i, -self._lim_int[i], self._lim_int[i]);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 逐分量断言，`tol` 与量纲一致。
    fn assert_near(actual: Vector3f, expected: Vector3f, tol: f32) {
        for i in 0..3 {
            assert!(
                (actual[i] - expected[i]).abs() <= tol,
                "axis {i}: actual {} expected {}",
                actual[i],
                expected[i]
            );
        }
    }

    fn status(rc: &RateControl) -> RateCtrlStatus {
        let mut status = RateCtrlStatus::default();
        rc.get_rate_control_status(&mut status);
        status
    }

    #[test]
    fn all_zero_case() {
        let mut rate_control = RateControl::default();
        let torque = rate_control.update(
            Vector3f::zeros(),
            Vector3f::zeros(),
            Vector3f::zeros(),
            0.0,
            false,
        );
        assert_near(torque, Vector3f::zeros(), 1e-12);
    }

    #[test]
    fn proportional_derivative_and_feed_forward_terms() {
        let mut rate_control = RateControl::default();
        rate_control.set_pid_gains(
            Vector3f::new(0.15, 0.2, 0.25),
            Vector3f::zeros(),
            Vector3f::new(0.01, 0.02, 0.03),
        );
        rate_control.set_feed_forward_gain(Vector3f::new(0.5, 0.6, 0.7));

        // landed = true 隔离积分项；输出 = P·e − D·ω̇ + FF·ω_sp
        let torque = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(0.1, 0.2, -0.3),
            Vector3f::new(1.0, 2.0, 3.0),
            0.0025,
            true,
        );
        assert_near(torque, Vector3f::new(0.055, 0.12, -0.375), 1e-6);
    }

    #[test]
    fn integrator_is_clamped_to_limit() {
        let mut rate_control = RateControl::default();
        rate_control.set_pid_gains(
            Vector3f::zeros(),
            Vector3f::new(10.0, 10.0, 10.0),
            Vector3f::zeros(),
        );
        rate_control.set_integrator_limit(Vector3f::new(0.05, 0.05, 0.05));

        for _ in 0..3 {
            let _ = rate_control.update(
                Vector3f::zeros(),
                Vector3f::new(1.0, 1.0, 1.0),
                Vector3f::zeros(),
                0.01,
                false,
            );
        }

        let state = status(&rate_control);
        assert!((state.rollspeed_integ - 0.05).abs() < 1e-7);
        assert!((state.pitchspeed_integ - 0.05).abs() < 1e-7);
        assert!((state.yawspeed_integ - 0.05).abs() < 1e-7);

        // 下一步力矩使用限幅后的积分项
        let torque = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(1.0, 1.0, 1.0),
            Vector3f::zeros(),
            0.01,
            false,
        );
        assert_near(torque, Vector3f::new(0.05, 0.05, 0.05), 1e-7);
    }

    #[test]
    fn saturation_blocks_integration_on_axis() {
        let mut rate_control = RateControl::default();
        rate_control.set_pid_gains(
            Vector3f::zeros(),
            Vector3f::new(10.0, 10.0, 10.0),
            Vector3f::zeros(),
        );
        rate_control.set_integrator_limit(Vector3f::new(1.0, 1.0, 1.0));
        rate_control.set_positive_saturation_flag(0, true);

        let _ = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(1.0, 1.0, 1.0),
            Vector3f::zeros(),
            0.01,
            false,
        );
        let state = status(&rate_control);
        assert!(state.rollspeed_integ.abs() < 1e-9);
        assert!(state.pitchspeed_integ > 0.0);
        assert!(state.yawspeed_integ > 0.0);

        // 负向饱和对称地阻断负误差
        let mut rate_control = RateControl::default();
        rate_control.set_pid_gains(
            Vector3f::zeros(),
            Vector3f::new(10.0, 10.0, 10.0),
            Vector3f::zeros(),
        );
        rate_control.set_integrator_limit(Vector3f::new(1.0, 1.0, 1.0));
        rate_control.set_negative_saturation_flag(1, true);
        let _ = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(1.0, -1.0, -1.0),
            Vector3f::zeros(),
            0.01,
            false,
        );
        let state = status(&rate_control);
        assert!(state.pitchspeed_integ.abs() < 1e-9);
        assert!(state.rollspeed_integ > 0.0);
        assert!(state.yawspeed_integ < 0.0);
    }

    #[test]
    fn saturation_status_sets_all_flags() {
        let mut rate_control = RateControl::default();
        rate_control.set_pid_gains(
            Vector3f::zeros(),
            Vector3f::new(1.0, 1.0, 1.0),
            Vector3f::zeros(),
        );
        rate_control.set_integrator_limit(Vector3f::new(1.0, 1.0, 1.0));
        rate_control.set_saturation_status([true, false, true], [false, true, false]);

        let _ = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(1.0, 1.0, 1.0),
            Vector3f::zeros(),
            0.01,
            false,
        );
        let state = status(&rate_control);
        assert!(state.rollspeed_integ.abs() < 1e-9);
        assert!(state.pitchspeed_integ > 0.0);
        assert!(state.yawspeed_integ.abs() < 1e-9);
    }

    #[test]
    fn i_factor_reduces_integral_with_rate_error() {
        let mut rate_control = RateControl::default();
        rate_control.set_pid_gains(
            Vector3f::zeros(),
            Vector3f::new(1.0, 1.0, 1.0),
            Vector3f::zeros(),
        );
        rate_control.set_integrator_limit(Vector3f::new(10.0, 10.0, 10.0));

        // 100 °/s：i_factor = 1 − (100/400)² = 0.9375
        let error_100 = radians(100.0);
        let _ = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(error_100, error_100, error_100),
            Vector3f::zeros(),
            1.0,
            false,
        );
        let state = status(&rate_control);
        assert!((state.rollspeed_integ - 0.9375 * error_100).abs() < 1e-6);

        // 200 °/s：i_factor = 0.75
        rate_control.reset_integral();
        let error_200 = radians(200.0);
        let _ = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(error_200, error_200, error_200),
            Vector3f::zeros(),
            1.0,
            false,
        );
        let state = status(&rate_control);
        assert!((state.rollspeed_integ - 0.75 * error_200).abs() < 1e-6);

        // 400 °/s：i_factor = 0，不再积分
        rate_control.reset_integral();
        let error_400 = radians(400.0);
        let _ = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(error_400, error_400, error_400),
            Vector3f::zeros(),
            1.0,
            false,
        );
        let state = status(&rate_control);
        assert!(state.rollspeed_integ.abs() < 1e-9);
    }

    #[test]
    fn landed_does_not_integrate() {
        let mut rate_control = RateControl::default();
        rate_control.set_pid_gains(
            Vector3f::zeros(),
            Vector3f::new(1.0, 1.0, 1.0),
            Vector3f::zeros(),
        );
        rate_control.set_integrator_limit(Vector3f::new(1.0, 1.0, 1.0));

        let _ = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(1.0, 1.0, 1.0),
            Vector3f::zeros(),
            0.01,
            true,
        );
        let state = status(&rate_control);
        assert!(state.rollspeed_integ.abs() < 1e-12);

        let _ = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(1.0, 1.0, 1.0),
            Vector3f::zeros(),
            0.01,
            false,
        );
        let state = status(&rate_control);
        assert!(state.rollspeed_integ > 0.0);
    }

    #[test]
    fn nonfinite_integral_step_is_dropped() {
        let mut rate_control = RateControl::default();
        rate_control.set_pid_gains(
            Vector3f::zeros(),
            Vector3f::new(1.0, 1.0, 1.0),
            Vector3f::zeros(),
        );
        rate_control.set_integrator_limit(Vector3f::new(1.0, 1.0, 1.0));

        // dt = ∞ 使积分步长非有限，PX4 保留上一周期的积分值
        let _ = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(1.0, 1.0, 1.0),
            Vector3f::zeros(),
            f32::INFINITY,
            false,
        );
        let state = status(&rate_control);
        assert!(state.rollspeed_integ.abs() < 1e-12);
    }

    #[test]
    fn reset_integral_clears_all_or_single_axis() {
        let mut rate_control = RateControl::default();
        rate_control.set_pid_gains(
            Vector3f::zeros(),
            Vector3f::new(1.0, 1.0, 1.0),
            Vector3f::zeros(),
        );
        rate_control.set_integrator_limit(Vector3f::new(1.0, 1.0, 1.0));
        let _ = rate_control.update(
            Vector3f::zeros(),
            Vector3f::new(0.1, 0.2, 0.3),
            Vector3f::zeros(),
            0.01,
            false,
        );

        rate_control.reset_integral_axis(1);
        let state = status(&rate_control);
        assert!(state.rollspeed_integ > 0.0);
        assert!(state.pitchspeed_integ.abs() < 1e-12);
        assert!(state.yawspeed_integ > 0.0);

        rate_control.reset_integral();
        let state = status(&rate_control);
        assert!(state.rollspeed_integ.abs() < 1e-12);
        assert!(state.pitchspeed_integ.abs() < 1e-12);
        assert!(state.yawspeed_integ.abs() < 1e-12);
    }
}
