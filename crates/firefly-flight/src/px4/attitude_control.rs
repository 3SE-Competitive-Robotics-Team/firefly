//! PX4 四元数姿态控制器（`AttitudeControl`）。
//!
//! 参考实现：PX4-Autopilot 提交 `4dbd2e069a5c30c2e53e47e842095d2576dc38c4`
//! 的 `src/modules/mc_att_control/AttitudeControl/AttitudeControl.hpp` 与
//! `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`（BSD-3-Clause）。
//!
//! 结构：二阶临界阻尼参考模型（ZOH 精确离散）→ 参考姿态的 P 控制 → 角速率前馈。
//! 偏航权重、逐轴速率限幅、前馈增益与限幅、`omega_command` 的偏航解耦均与 PX4 一致。
//! 保持 NED 世界系 / FRD 机体系约定；输入/输出均为机体角速率。
//!
//! 不移植：`AttitudeControl()` / `~AttitudeControl()`——Rust 用 [`AttitudeControl::new`] / [`Default`] 构造、自动析构。
//! 不移植：`setAttitudeSetpoint(..., const float dt = -1.f)` 的默认参数——Rust 无默认参数，
//! 调用方显式传 `-1.0` 表示把参考模型重置到当前设定点。
//! 不移植：文件内联的 `qmul` / `qinv` / `qzaxis` 包装——直接用 `nalgebra` 的 `*`、
//! `inverse()` 与 [`quat_dcm_z`]。
//!
//! 辅助函数：[`quat_canonical_px4`] 与 [`quat_from_two_vectors_px4`] 分别复刻 PX4
//! `Quatf::canonical()`（按首个非零分量定符号）与 `Quatf(v0, v1)`（反向平行时取正交轴
//! 180° 旋转）。`types::quat_canonical` / `types::quat_from_two_vectors` 在 180° 边界上
//! 分别退化为按标量符号/单位四元数，因此在本模块内私有补齐。

// 字段名与 C++ 保持一致（含 `_` 前缀），并按要求使用 `types` 的 glob 导入。
// `UnitQuaternion` 没有 `MulAssign`，四元数更新沿用 PX4 的 `q = q * dq` 赋值式。
#![allow(
    clippy::assign_op_pattern,
    clippy::used_underscore_binding,
    clippy::wildcard_imports
)]

use nalgebra::Quaternion;

use super::types::*;

/// 四元数姿态控制器。
#[derive(Clone, Copy, Debug)]
pub struct AttitudeControl {
    /// 姿态 P 增益 [rad/s per rad]，偏航轴已补偿 `_yaw_w`。
    _proportional_gain: Vector3f,
    /// 输出角速率上限 [rad/s]。
    _rate_limit: Vector3f,
    /// 偏航权重 [0, 1]，压低偏航相对 roll/pitch 的优先级。
    _yaw_w: f32,

    /// 参考模型跟踪的参考姿态。
    _q_ref: Quatf,
    /// 误差驱动的修正角速率（二阶状态）[rad/s]。
    _omega_correction: Vector3f,
    /// 解析前馈角速率 [rad/s]，不受前馈限幅。
    _omega_command: Vector3f,
    /// 参考模型是否已初始化。
    _ref_initialized: bool,

    /// 参考模型自然频率 [rad/s]。
    _omega_n: f32,
    /// 参考模型刚度系数 [rad/s²]，与 `_omega_n` 同步。
    _kq: f32,

    /// 误差驱动前馈的增益 [0, 1]。
    _ff_gain: f32,
    /// 误差驱动前馈的逐轴限幅 [rad/s]，0 表示不限幅。
    _ff_max: f32,
}

impl Default for AttitudeControl {
    fn default() -> Self {
        Self::new()
    }
}

impl AttitudeControl {
    /// 构造使用 PX4 默认参数的姿态控制器。
    ///
    /// 默认：增益与速率上限为零、偏航权重 0、参考模型频率 50 rad/s、
    /// 前馈增益 1、前馈限幅 0（不限幅）、参考姿态为单位四元数。
    #[must_use]
    pub fn new() -> Self {
        Self {
            _proportional_gain: Vector3f::zeros(),
            _rate_limit: Vector3f::zeros(),
            _yaw_w: 0.0,
            _q_ref: Quatf::identity(),
            _omega_correction: Vector3f::zeros(),
            _omega_command: Vector3f::zeros(),
            _ref_initialized: false,
            _omega_n: 50.0,
            _kq: 50.0 * 50.0,
            _ff_gain: 1.0,
            _ff_max: 0.0,
        }
    }

    /// 设置姿态 P 增益与偏航权重。
    ///
    /// - `proportional_gain`：各轴增益 [rad/s per rad]
    /// - `yaw_weight`：偏航权重，内部限幅到 [0, 1]；非零时把偏航增益按 `1/yaw_weight`
    ///   预补偿，使 `update` 中的偏航缩放后的实际增益保持不变。
    pub fn set_proportional_gain(&mut self, proportional_gain: Vector3f, yaw_weight: f32) {
        self._proportional_gain = proportional_gain;
        self._yaw_w = constrain(yaw_weight, 0.0, 1.0);

        // 补偿偏航权重对输出的缩放
        if self._yaw_w > 1e-4 {
            self._proportional_gain[2] /= self._yaw_w;
        }
    }

    /// 设置参考模型的自然频率 [rad/s]，下限 0.1 rad/s。
    pub fn set_ref_model_frequency(&mut self, omega_n: f32) {
        self._omega_n = omega_n.max(0.1);
        self._kq = self._omega_n * self._omega_n;
    }

    /// 设置误差驱动前馈的增益，限幅到 [0, 1]。
    pub fn set_feed_forward_gain(&mut self, gain: f32) {
        self._ff_gain = constrain(gain, 0.0, 1.0);
    }

    /// 设置误差驱动前馈的逐轴限幅 [rad/s]，负值按 0 处理（0 = 不限幅）。
    pub fn set_feed_forward_limit(&mut self, limit: f32) {
        self._ff_max = limit.max(0.0);
    }

    /// 设置输出角速率上限 [rad/s]。
    pub fn set_rate_limit(&mut self, rate_limit: Vector3f) {
        self._rate_limit = rate_limit;
    }

    /// 设置新的姿态设定点。
    ///
    /// - `qd`：期望姿态
    /// - `yawspeed_setpoint` [rad/s]：世界系偏航角速率前馈
    /// - `dt` [s]：距上一个设定点的时间；`dt <= 0` 或首次调用时把参考姿态直接对齐到 `qd`
    pub fn set_attitude_setpoint(&mut self, qd: Quatf, yawspeed_setpoint: f32, dt: f32) {
        let qd_normalized = quat_normalize(qd);

        if self._ref_initialized && dt > 0.0 {
            self.propagate_reference_model(qd_normalized, yawspeed_setpoint, dt);
        } else {
            // 首次调用（或 dt 越界）：参考姿态直接对齐设定点
            self._q_ref = qd_normalized;
            self._omega_correction = Vector3f::zeros();
            self._omega_command = Vector3f::zeros();
            self._ref_initialized = true;
        }
    }

    /// 用世界系增量旋转修正参考姿态。
    ///
    /// `_omega_correction` 与 `_omega_command` 位于参考机体系，世界系重标定下不变，故不修正。
    pub fn adapt_attitude_setpoint(&mut self, q_delta: Quatf) {
        self._q_ref = quat_normalize(q_delta * self._q_ref);
    }

    /// 执行一个控制周期。
    ///
    /// - `q`：当前姿态估计（机体→世界）
    ///
    /// 返回机体系角速率设定点 [rad/s]，供 [`super::rate_control::RateControl`] 跟踪。
    #[must_use]
    pub fn update(&self, q: Quatf) -> Vector3f {
        // P 控制器始终跟踪参考模型姿态
        let mut qd = self._q_ref;

        // 忽略偏航的降阶期望姿态，优先保证 roll/pitch
        let e_z = quat_dcm_z(q);
        let e_z_d = quat_dcm_z(qd);
        let mut qd_red = quat_from_two_vectors_px4(e_z, e_z_d);

        if quat_imag(qd_red)[0].abs() > (1.0 - 1e-5) || quat_imag(qd_red)[1].abs() > (1.0 - 1e-5) {
            // 机体推力方向与期望完全相反：直接用完整期望姿态
            qd_red = qd;
        } else {
            // 世界系中的倾斜误差四元数右乘到当前姿态
            qd_red = qd_red * q;
        }

        // qd = qd_red * qd_dyaw，其中 qd_dyaw = (cos(Δψ/2), 0, 0, sin(Δψ/2))
        let qd_dyaw = quat_canonical_px4(qd_red.inverse() * qd);
        // acos/asin 的定义域保护
        let qd_dyaw_w = constrain(qd_dyaw.scalar(), -1.0, 1.0);
        let qd_dyaw_z = constrain(qd_dyaw.vector()[2], -1.0, 1.0);

        // 偏航角按权重缩放后重组期望姿态
        qd = qd_red
            * Quatf::from_quaternion(Quaternion::new(
                (self._yaw_w * qd_dyaw_w.acos()).cos(),
                0.0,
                0.0,
                (self._yaw_w * qd_dyaw_z.asin()).sin(),
            ));

        // 四元数姿态控制律：qe 为 q 到 qd 的旋转
        let qe = quat_canonical_px4(q.inverse() * qd);

        // 用 sin(α/2)·旋转轴 作为姿态误差，并处理四元数反极歧义
        let eq = 2.0 * quat_imag(qe);

        // 计算角速率设定点
        let mut rate_setpoint = eq.emult(self._proportional_gain);

        // 把参考系角速率映射到当前机体系
        let q_rel = q.inverse() * self._q_ref;

        // 解析角速率指令（手动/自动偏航）是设定点而非模型预测：以单位增益前馈，
        // 不受前馈增益与限幅影响。
        rate_setpoint += quat_rotate(q_rel, self._omega_command);

        // 增益缩放、限幅约束误差驱动的模型预判（增益 0 时为零）
        let mut omega_ff = self._ff_gain * quat_rotate(q_rel, self._omega_correction);

        if self._ff_max > 0.0 {
            for i in 0..3 {
                omega_ff[i] = constrain(omega_ff[i], -self._ff_max, self._ff_max);
            }
        }

        rate_setpoint += omega_ff;

        // 输出角速率限幅
        for i in 0..3 {
            rate_setpoint[i] =
                constrain(rate_setpoint[i], -self._rate_limit[i], self._rate_limit[i]);
        }

        rate_setpoint
    }

    /// 参考模型当前跟踪的姿态。
    ///
    /// C++ 返回 `const Quatf&`；`Quatf` 在 Rust 中是 `Copy`，按值返回。
    #[must_use]
    pub fn get_reference_attitude(&self) -> Quatf {
        self._q_ref
    }

    /// 二阶参考模型前进一步（ZOH 精确离散）。
    ///
    /// 状态矩阵 `A = [[0, -1], [_kq, -2*_omega_n]]` 在 `- _omega_n` 处有二重特征值，
    /// 其指数展开因幂零项截断，得到闭式 `exp(A·dt) = e^{-_omega_n·dt}·[(1+_omega_n·dt)I + dt·A]`，
    /// 对任意 `dt > 0` 无条件稳定。
    ///
    /// - `qd`：归一化期望姿态
    /// - `yawspeed_setpoint` [rad/s]：世界系偏航角速率前馈
    /// - `dt` [s]：距上一个设定点的时间（> 0）
    fn propagate_reference_model(&mut self, qd: Quatf, yawspeed_setpoint: f32, dt: f32) {
        // 切空间输入：解析偏航角速率转换到 q_ref 机体系；误差向量由 q_ref 到 qd
        let q_ref_inv = self._q_ref.inverse();
        let yaw_axis_body = quat_dcm_z(q_ref_inv); // 世界偏航轴在 q_ref 机体系中的表示
        let omega_command = if is_finite(yawspeed_setpoint) {
            yaw_axis_body * yawspeed_setpoint
        } else {
            Vector3f::zeros()
        };

        let q_err = quat_canonical_px4(q_ref_inv * qd);
        let e = 2.0 * quat_imag(q_err);

        // exp(A·dt) 的分量
        let w_dt = self._omega_n * dt;
        let emt = (-w_dt).exp();
        let a = (1.0 + w_dt) * emt;
        let b = dt * emt;
        let gamma = self._kq * dt * emt;
        let delta = (1.0 - w_dt) * emt;

        // 在切空间传播误差驱动修正（二阶状态）；delta_phi 为 [0, dt] 内角速率积分
        let delta_phi = (1.0 - a) * e + b * self._omega_correction + omega_command * dt;
        self._omega_correction = gamma * e + delta * self._omega_correction;

        // 偏航角速率指令：设定航向跟随实测偏航，误差驱动速率前馈会形成正反馈，
        // 因此偏航轴只保留解析指令 omega_command。
        if is_finite(yawspeed_setpoint) && yawspeed_setpoint.abs() > f32::EPSILON {
            self._omega_correction -= self._omega_correction.dot(&yaw_axis_body) * yaw_axis_body;
        }

        // 解析参考角速率单独保存，供 update 免除前馈限幅
        self._omega_command = omega_command;

        self._q_ref = quat_normalize(self._q_ref * quat_from_rotation_vector(delta_phi));
    }
}

/// PX4 `Quatf::canonical()`：用首个非零分量的符号选定四元数表示。
fn quat_canonical_px4(q: Quatf) -> Quatf {
    for component in [q.scalar(), q.vector()[0], q.vector()[1], q.vector()[2]] {
        if component.abs() > f32::EPSILON {
            return if component < 0.0 {
                Quatf::new_unchecked(-q.into_inner())
            } else {
                q
            };
        }
    }

    q
}

/// PX4 `matrix::Quaternion(const Vector3 &src, const Vector3 &dst)`：
/// `src` 到 `dst` 的最短旋转。
///
/// 反向平行（`|src × dst| < 1e-5` 且点积为负）时，PX4 选 `|src|` 最小分量对应的坐标轴，
/// 再与 `src` 叉乘得到 180° 旋转轴，使 `update` 的“推力完全反向”分支可被识别。
fn quat_from_two_vectors_px4(src: Vector3f, dst: Vector3f) -> Quatf {
    let mut cr = src.cross(&dst);
    let dt = src.dot(&dst);

    let w = if cr.norm() < 1e-5 && dt < 0.0 {
        // 180° 反向：按 |src| 的最小分量选一个与 src 不平行的轴
        let abs = src.abs();
        let axis = if abs[0] < abs[1] {
            if abs[0] < abs[2] {
                Vector3f::x()
            } else {
                Vector3f::z()
            }
        } else if abs[1] < abs[2] {
            Vector3f::y()
        } else {
            Vector3f::z()
        };

        cr = src.cross(&axis);
        0.0
    } else {
        // 常规情况：半程四元数解
        dt + (src.norm_squared() * dst.norm_squared()).sqrt()
    };

    Quatf::from_quaternion(Quaternion::new(w, cr[0], cr[1], cr[2]))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 参考实现的设定点频率 [Hz] 对应的周期 [s]。
    const K_DT: f32 = 0.004;
    /// 参考模型收敛所需的仿真步数。
    const K_SETTLE_STEPS: usize = 500;

    fn assert_near(actual: f32, expected: f32, tol: f32) {
        assert!(
            (actual - expected).abs() <= tol,
            "actual {actual} expected {expected}"
        );
    }

    fn feedforward_controller() -> AttitudeControl {
        let mut attitude_control = AttitudeControl::default();
        attitude_control.set_proportional_gain(Vector3f::new(6.5, 6.5, 2.8), 0.4);
        attitude_control.set_rate_limit(Vector3f::new(10.0, 10.0, 10.0));
        attitude_control.set_ref_model_frequency(10.0);
        attitude_control
    }

    /// 以恒定机体角速率推进设定点，直到参考模型稳定。
    /// 首步传 `dt < 0`，与首次设定点重置参考模型的行为一致。
    fn ramp_setpoint(
        attitude_control: &mut AttitudeControl,
        body_rate: Vector3f,
        yawspeed_sp: f32,
        steps: usize,
    ) -> Quatf {
        let mut q_d = Quatf::identity();

        for i in 0..steps {
            q_d = q_d * quat_from_rotation_vector(body_rate * K_DT);
            let dt = if i == 0 { -1.0 } else { K_DT };
            attitude_control.set_attitude_setpoint(q_d, yawspeed_sp, dt);
        }

        q_d
    }

    #[test]
    fn all_zero_case() {
        let attitude_control = AttitudeControl::default();
        let rate_setpoint = attitude_control.update(Quatf::identity());
        assert_near(rate_setpoint.norm(), 0.0, 1e-12);
    }

    #[test]
    fn yaw_weight_scaling() {
        // 纯偏航设定点，偏航权重 0.4
        let yaw_gain = 2.8_f32;
        let yaw_sp = 0.1_f32;
        let pure_yaw_attitude = quat_from_rotation_vector(Vector3f::new(0.0, 0.0, yaw_sp));

        let mut attitude_control = AttitudeControl::default();
        attitude_control.set_proportional_gain(Vector3f::new(6.5, 6.5, yaw_gain), 0.4);
        attitude_control.set_rate_limit(Vector3f::new(1000.0, 1000.0, 1000.0));
        attitude_control.set_attitude_setpoint(pure_yaw_attitude, 0.0, -1.0);

        let rate_setpoint = attitude_control.update(Quatf::identity());

        // roll/pitch 无输出
        assert_near(rate_setpoint[0], 0.0, 1e-6);
        assert_near(rate_setpoint[1], 0.0, 1e-6);
        // 偏航误差 × 增益（已按 yaw_weight 预补偿）
        assert_near(rate_setpoint[2], yaw_sp * yaw_gain, 1e-4);

        // 偏航权重为 0：无输出且无 NaN
        attitude_control.set_proportional_gain(Vector3f::new(6.5, 6.5, yaw_gain), 0.0);
        let rate_setpoint = attitude_control.update(Quatf::identity());
        assert_near(rate_setpoint.norm(), 0.0, 1e-9);
    }

    #[test]
    fn rate_limit_clamps_each_axis() {
        let mut attitude_control = AttitudeControl::default();
        attitude_control.set_proportional_gain(Vector3f::new(100.0, 100.0, 100.0), 1.0);
        attitude_control.set_rate_limit(Vector3f::new(0.2, 0.3, 0.4));

        let qd = quat_from_rotation_vector(Vector3f::new(0.5, 0.0, 0.0));
        attitude_control.set_attitude_setpoint(qd, 0.0, -1.0);

        let rate_setpoint = attitude_control.update(Quatf::identity());
        assert_near(rate_setpoint[0], 0.2, 1e-6);
        assert_near(rate_setpoint[1], 0.0, 1e-5);
        assert_near(rate_setpoint[2], 0.0, 1e-5);
    }

    #[test]
    fn attitude_error_sign_matches_known_rotation() {
        for angle in [0.2_f32, -0.2] {
            let mut attitude_control = AttitudeControl::default();
            attitude_control.set_proportional_gain(Vector3f::new(1.0, 1.0, 1.0), 1.0);
            attitude_control.set_rate_limit(Vector3f::new(10.0, 10.0, 10.0));

            let qd = quat_from_rotation_vector(Vector3f::new(angle, 0.0, 0.0));
            attitude_control.set_attitude_setpoint(qd, 0.0, -1.0);

            let rate_setpoint = attitude_control.update(Quatf::identity());
            // 误差四元数虚部为 +x（或 -x），输出滚转速率与旋转角同号
            assert_near(rate_setpoint[0], 2.0 * (angle / 2.0).sin(), 1e-5);
            assert_near(rate_setpoint[1], 0.0, 1e-6);
            assert_near(rate_setpoint[2], 0.0, 1e-6);
        }
    }

    /// 移植 `AttitudeControlConvergenceTest`：状态按速率设定点积分，误差应逐步减小并收敛。
    ///
    /// C++ 断言末态四元数与目标逐分量相等；跨实现浮点运算顺序不同，改为按姿态夹角断言。
    #[test]
    fn converges_to_goal_for_all_quaternion_pairs() {
        let inputs = [
            Quatf::identity(),
            Quatf::from_quaternion(Quaternion::new(0.0_f32, 1.0, 0.0, 0.0)),
            Quatf::from_quaternion(Quaternion::new(0.0, 0.0, 1.0, 0.0)),
            Quatf::from_quaternion(Quaternion::new(0.0, 0.0, 0.0, 1.0)),
            Quatf::from_quaternion(Quaternion::new(0.698, 0.024, -0.681, -0.220)),
            Quatf::from_quaternion(Quaternion::new(-0.820, -0.313, 0.225, -0.423)),
            Quatf::from_quaternion(Quaternion::new(0.599, -0.172, 0.755, -0.204)),
            Quatf::from_quaternion(Quaternion::new(0.216, -0.662, 0.290, -0.656)),
        ];

        for state0 in inputs {
            for goal in inputs {
                let mut attitude_control = AttitudeControl::default();
                attitude_control.set_proportional_gain(Vector3f::new(0.5, 0.6, 0.3), 0.4);
                attitude_control.set_rate_limit(Vector3f::new(100.0, 100.0, 100.0));
                attitude_control.set_attitude_setpoint(goal, 0.0, -1.0);

                let mut state = state0;
                let mut rate_setpoint = Vector3f::new(1000.0, 1000.0, 1000.0);
                let mut converged_at = 0;

                for step in (1..=100).rev() {
                    let rate_setpoint_new = attitude_control.update(state);
                    // 用速率设定点直接旋转仿真状态
                    state = state * quat_from_rotation_vector(rate_setpoint_new);
                    // 间歇性反极表示，验证不会反向绕远
                    state = Quatf::new_unchecked(-state.into_inner());

                    // 误差与输出应逐步减小
                    if rate_setpoint_new.norm() >= rate_setpoint.norm() {
                        converged_at = step;
                        break;
                    }

                    rate_setpoint = rate_setpoint_new;
                }

                assert!(converged_at > 0, "did not converge in 100 steps");
                let error = (state.inverse() * goal).angle();
                assert!(error < 1e-3, "final error {error} rad");
            }
        }
    }

    #[test]
    fn constant_setpoint_gives_no_feedforward() {
        let mut attitude_control = feedforward_controller();
        let q_d = quat_from_rotation_vector(Vector3f::new(0.1, 0.0, 0.0));

        for i in 0..K_SETTLE_STEPS {
            let dt = if i == 0 { -1.0 } else { K_DT };
            attitude_control.set_attitude_setpoint(q_d, 0.0, dt);
        }

        // 车辆已在设定点：无位置误差，模型也不产生速率输出
        let rate_setpoint = attitude_control.update(q_d);
        assert_near(rate_setpoint.norm(), 0.0, 1e-3);
    }

    #[test]
    fn roll_ramp_produces_roll_feedforward() {
        let mut attitude_control = feedforward_controller();
        let omega = 0.5_f32;
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(omega, 0.0, 0.0),
            0.0,
            K_SETTLE_STEPS,
        );

        // 在参考姿态处评估：P 项为零，只剩前馈
        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        assert_near(rate_setpoint[0], omega, 1e-3);
        assert_near(rate_setpoint[1], 0.0, 1e-3);
        assert_near(rate_setpoint[2], 0.0, 1e-3);
    }

    #[test]
    fn pitch_ramp_produces_pitch_feedforward() {
        let mut attitude_control = feedforward_controller();
        let omega = 0.5_f32;
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(0.0, omega, 0.0),
            0.0,
            K_SETTLE_STEPS,
        );

        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        assert_near(rate_setpoint[0], 0.0, 1e-3);
        assert_near(rate_setpoint[1], omega, 1e-3);
        assert_near(rate_setpoint[2], 0.0, 1e-3);
    }

    #[test]
    fn high_rate_ramp_still_exact() {
        let mut attitude_control = feedforward_controller();
        let omega = core::f32::consts::FRAC_PI_2; // ~90 °/s
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(0.0, omega, 0.0),
            0.0,
            K_SETTLE_STEPS,
        );

        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        assert_near(rate_setpoint[0], 0.0, 1e-3);
        assert_near(rate_setpoint[1], omega, 1e-3);
        assert_near(rate_setpoint[2], 0.0, 1e-3);
    }

    #[test]
    fn yaw_ramp_only_analytical_feedforward_contributes() {
        let mut attitude_control = feedforward_controller();
        let omega = 0.5_f32;
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(0.0, 0.0, omega),
            omega,
            K_SETTLE_STEPS,
        );

        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        assert_near(rate_setpoint[0], 0.0, 1e-3);
        assert_near(rate_setpoint[1], 0.0, 1e-3);
        assert_near(rate_setpoint[2], omega, 1e-3);
    }

    #[test]
    fn tilted_yaw_does_not_double_count() {
        // 机体固定倾斜，绕世界 z 轴恒速偏航：ω_body = (-sin(tilt)·ψ̇, 0, cos(tilt)·ψ̇)
        let tilt = 0.5_f32;
        let yaw_rate = 0.5_f32;
        let q_pitch = quat_from_rotation_vector(Vector3f::new(0.0, tilt, 0.0));
        let mut attitude_control = feedforward_controller();

        for i in 0..K_SETTLE_STEPS {
            let q_yaw =
                quat_from_rotation_vector(Vector3f::new(0.0, 0.0, yaw_rate * K_DT * i as f32));
            let q_d = q_yaw * q_pitch;
            let dt = if i == 0 { -1.0 } else { K_DT };
            attitude_control.set_attitude_setpoint(q_d, yaw_rate, dt);
        }

        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        assert_near(rate_setpoint[0], -tilt.sin() * yaw_rate, 1e-3);
        assert_near(rate_setpoint[1], 0.0, 1e-3);
        assert_near(rate_setpoint[2], tilt.cos() * yaw_rate, 1e-3);
    }

    #[test]
    fn feed_forward_disabled_suppresses_contribution() {
        let mut attitude_control = feedforward_controller();
        let omega = 0.5_f32;
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(omega, 0.0, 0.0),
            0.0,
            K_SETTLE_STEPS,
        );

        // 增益 0：P 项为零时输出为零
        attitude_control.set_feed_forward_gain(0.0);
        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        assert_near(rate_setpoint.norm(), 0.0, 1e-3);

        // 恢复增益：参考模型状态保留，前馈立即恢复
        attitude_control.set_feed_forward_gain(1.0);
        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        assert_near(rate_setpoint[0], omega, 1e-3);
        assert_near(rate_setpoint[1], 0.0, 1e-3);
        assert_near(rate_setpoint[2], 0.0, 1e-3);
    }

    #[test]
    fn unlocked_yaw_does_not_feed_back_slaved_heading() {
        // 手动偏航速率：设定航向跟随实测偏航快速推进，解析指令只有很小的滤波尾
        let ramp_rate = 0.8_f32;
        let commanded = 0.05_f32;
        let mut attitude_control = feedforward_controller();
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(0.0, 0.0, ramp_rate),
            commanded,
            K_SETTLE_STEPS,
        );

        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        // 偏航前馈只跟踪解析指令，不锁存被测偏航
        assert_near(rate_setpoint[2], commanded, 1e-3);
        assert!(rate_setpoint[2].abs() < 0.2);
        assert_near(rate_setpoint[0], 0.0, 1e-3);
        assert_near(rate_setpoint[1], 0.0, 1e-3);
    }

    #[test]
    fn locked_yaw_ramp_still_feeds_forward() {
        // 真实偏航角目标、无解析角速率：参考模型必须对航向求导
        let omega = 0.5_f32;
        let mut attitude_control = feedforward_controller();
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(0.0, 0.0, omega),
            0.0,
            K_SETTLE_STEPS,
        );

        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        assert_near(rate_setpoint[2], omega, 1e-3);
        assert_near(rate_setpoint[0], 0.0, 1e-3);
        assert_near(rate_setpoint[1], 0.0, 1e-3);
    }

    #[test]
    fn tilted_unlocked_yaw_uses_commanded_rate_only() {
        let tilt = 0.5_f32;
        let slaved_rate = 0.8_f32;
        let commanded = 0.05_f32;
        let q_pitch = quat_from_rotation_vector(Vector3f::new(0.0, tilt, 0.0));
        let mut attitude_control = feedforward_controller();

        for i in 0..K_SETTLE_STEPS {
            let q_yaw =
                quat_from_rotation_vector(Vector3f::new(0.0, 0.0, slaved_rate * K_DT * i as f32));
            let q_d = q_yaw * q_pitch;
            let dt = if i == 0 { -1.0 } else { K_DT };
            attitude_control.set_attitude_setpoint(q_d, commanded, dt);
        }

        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        // 前馈是解析世界系偏航角速率在机体系的投影，不是被跟随的航向斜坡
        assert_near(rate_setpoint[0], -tilt.sin() * commanded, 1e-3);
        assert_near(rate_setpoint[1], 0.0, 1e-3);
        assert_near(rate_setpoint[2], tilt.cos() * commanded, 1e-3);
    }

    #[test]
    fn commanded_yaw_rate_fed_forward_when_ff_disabled() {
        let commanded = 0.05_f32;
        let mut attitude_control = feedforward_controller();
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(0.0, 0.0, 0.8),
            commanded,
            K_SETTLE_STEPS,
        );

        // 解析偏航速率绕过参考模型，以单位增益前馈
        attitude_control.set_feed_forward_gain(0.0);
        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        assert_near(rate_setpoint[2], commanded, 1e-3);
        assert_near(rate_setpoint[0], 0.0, 1e-3);
        assert_near(rate_setpoint[1], 0.0, 1e-3);
    }

    #[test]
    fn fractional_gain_does_not_weaken_commanded_yaw() {
        let commanded = 0.05_f32;
        let mut attitude_control = feedforward_controller();
        attitude_control.set_feed_forward_gain(0.1);
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(0.0, 0.0, 0.8),
            commanded,
            K_SETTLE_STEPS,
        );

        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        // 指令偏航不受前馈增益缩放
        assert_near(rate_setpoint[2], commanded, 1e-3);
        assert_near(rate_setpoint[0], 0.0, 1e-3);
        assert_near(rate_setpoint[1], 0.0, 1e-3);
    }

    #[test]
    fn fractional_gain_scales_anticipation() {
        let omega = 0.5_f32;
        let mut attitude_control = feedforward_controller();
        attitude_control.set_feed_forward_gain(0.1);
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(omega, 0.0, 0.0),
            0.0,
            K_SETTLE_STEPS,
        );

        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        // 误差驱动预判按增益缩放，与解析指令不同
        assert_near(rate_setpoint[0], 0.1 * omega, 1e-3);
        assert_near(rate_setpoint[1], 0.0, 1e-3);
        assert_near(rate_setpoint[2], 0.0, 1e-3);
    }

    #[test]
    fn feed_forward_limit_caps_anticipation_only() {
        let omega = 0.5_f32;
        let mut attitude_control = feedforward_controller();
        attitude_control.set_feed_forward_limit(0.2);
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(omega, 0.0, 0.0),
            0.0,
            K_SETTLE_STEPS,
        );

        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        assert_near(rate_setpoint[0], 0.2, 1e-3);

        // 解析指令不受前馈限幅影响
        let mut attitude_control = feedforward_controller();
        attitude_control.set_feed_forward_limit(0.2);
        ramp_setpoint(
            &mut attitude_control,
            Vector3f::new(0.0, 0.0, 0.8),
            0.5,
            K_SETTLE_STEPS,
        );
        let rate_setpoint = attitude_control.update(attitude_control.get_reference_attitude());
        assert_near(rate_setpoint[2], 0.5, 1e-3);
    }

    #[test]
    fn reference_model_converges_to_constant_setpoint() {
        let mut attitude_control = feedforward_controller();
        let q_d = quat_from_rotation_vector(Vector3f::new(-0.3, 0.2, 0.4));
        attitude_control.set_attitude_setpoint(q_d, 0.0, -1.0);

        // 常值设定点：参考模型无误差、保持不动
        for _ in 0..K_SETTLE_STEPS {
            attitude_control.set_attitude_setpoint(q_d, 0.0, K_DT);
        }

        assert!((attitude_control.get_reference_attitude().inverse() * q_d).angle() < 1e-6);
    }

    #[test]
    fn adapt_attitude_setpoint_applies_world_delta() {
        let mut attitude_control = AttitudeControl::default();
        let q_d = quat_from_rotation_vector(Vector3f::new(0.1, 0.0, 0.0));
        attitude_control.set_attitude_setpoint(q_d, 0.0, -1.0);

        let q_delta = quat_from_rotation_vector(Vector3f::new(0.0, 0.0, 0.2));
        attitude_control.adapt_attitude_setpoint(q_delta);

        let expected = q_delta * q_d;
        let error = (attitude_control.get_reference_attitude().inverse() * expected).angle();
        assert!(error < 1e-6);
    }
}
