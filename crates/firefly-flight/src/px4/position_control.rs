//! PX4 `PositionControl` 的 1:1 移植：位置 P 控制 + 速度 PID 控制。
//!
//! 来源：PX4-Autopilot 提交 `4dbd2e069a5c30c2e53e47e842095d2576dc38c4` 的
//! `src/modules/mc_pos_control/PositionControl/PositionControl.hpp`、
//! `PositionControl.cpp` 与 `PositionControlTest.cpp`，BSD-3-Clause 许可。
//!
//! 约定：世界系 NED、机体系 FRD；位置 m，速度 m/s，加速度 m/s²，角度 rad，
//! 角速度 rad/s；推力按最大集体推力归一化（无量纲，机体系 z 分量向上为负）。
//! 设定值中的 NAN 表示该分量未设定。
//!
//! `types.rs` 未提供 `vehicle_local_position_setpoint_s` 的对应结构，因此本文件
//! 定义专用结构 [`LocalPositionSetpoint`]，字段与原消息的控制相关字段一一对应。
//!
//! 不移植：uORB 主题 `trajectory_setpoint`、`vehicle_attitude_setpoint`、
//! `vehicle_local_position_setpoint`，改用 [`TrajectorySetpoint`]、
//! [`VehicleAttitudeSetpoint`] 与 [`LocalPositionSetpoint`]。
//! 不移植：参数与 YAML 配置，增益和限幅通过 setter 注入。
//! 不移植：模块调度与采样周期，由调用方按固定 `dt` 驱动 [`PositionControl::update`]。
//! 不移植：日志与 uORB 调试消息，控制律不产生日志。
//! 不移植：land detector，落地上锁不属于控制律。
//! 不移植：`empty_trajectory_setpoint` 的 timestamp 字段，本 crate 的消息结构不含时间戳。

use super::control_math::{
    add_if_not_nan_vector3f, constrain_xy, limit_tilt, set_zero_if_nan_vector3f, thrust_to_attitude,
};
// 共享类型统一从 types 引入，与 PX4 的 `using namespace` 对应
#[allow(clippy::wildcard_imports)]
use super::types::*;

// PX4 在 `PositionControl.hpp` 中定义 `PositionControlStates`；本 crate 统一放在
// `types.rs`，此处重导出以保持 PX4 的模块边界。
pub use super::types::PositionControlStates;

/// PX4 `vehicle_local_position_setpoint_s` 中由位置控制器填充的字段。
///
/// 位置 m，速度 m/s，加速度 m/s²（NED），推力按最大集体推力归一化；
/// 偏航 rad，偏航角速度 rad/s。
#[derive(Clone, Copy, Debug, Default)]
pub struct LocalPositionSetpoint {
    /// 期望位置（m，NED）。
    pub position: Vector3f,
    /// 期望速度（m/s，NED）。
    pub velocity: Vector3f,
    /// 期望加速度（m/s²，NED）。
    pub acceleration: Vector3f,
    /// 期望推力（归一化，NED）。
    pub thrust: Vector3f,
    /// 期望偏航角（rad）。
    pub yaw: f32,
    /// 期望偏航角速度（rad/s）。
    pub yawspeed: f32,
}

/// 多旋翼核心位置控制器。
///
/// 位置环为 P 控制，速度环为 PID 控制；输入为载具位置/速度/偏航、
/// 期望位置/速度/加速度/推力/偏航/偏航角速度以及比全局限幅更严格的约束，
/// 输出为推力向量与偏航设定值。
///
/// 若某分量设定值为 NAN，则视为未设定：有位置与速度设定值时速度设定值作为前馈，
/// 且 P 控制器输出的速度分量优先于前馈分量。
pub struct PositionControl {
    // 增益
    /// 位置控制比例增益（1/s）。
    gain_pos_p: Vector3f,
    /// 速度控制比例增益（1/s）。
    gain_vel_p: Vector3f,
    /// 速度控制积分增益（1/s²）。
    gain_vel_i: Vector3f,
    /// 速度控制微分增益（无量纲）。
    gain_vel_d: Vector3f,

    // 限幅
    /// 水平速度限幅（m/s）。
    lim_vel_horizontal: f32,
    /// 向上速度限幅（m/s）。
    lim_vel_up: f32,
    /// 向下速度限幅（m/s）。
    lim_vel_down: f32,
    /// 最小集体推力（归一化，取值使推力向量 z 分量为负，例如 0.1）。
    lim_thr_min: f32,
    /// 最大集体推力（归一化，例如 0.9）。
    lim_thr_max: f32,
    /// 优先分配垂直推力时为水平控制保留的推力裕度（归一化）。
    lim_thr_xy_margin: f32,
    /// 输出姿态相对水平面的最大倾角（rad）。
    lim_tilt: f32,

    /// 悬停推力（归一化），限制在 `[HOVER_THRUST_MIN, HOVER_THRUST_MAX]`。
    hover_thrust: f32,
    /// 是否解耦水平与垂直加速度（忽略垂直加速度设定值对倾角的影响）。
    decouple_horizontal_and_vertical_acceleration: bool,

    // 状态
    /// 当前载具位置（m，NED）。
    pos: Vector3f,
    /// 当前载具速度（m/s，NED）。
    vel: Vector3f,
    /// 速度导数（m/s²，NED），代替加速度估计。
    vel_dot: Vector3f,
    /// 速度控制积分项（m/s²）。
    vel_int: Vector3f,
    /// 当前航向（rad）。
    yaw: f32,

    // 设定值
    /// 期望位置（m，NED）。
    pos_sp: Vector3f,
    /// 期望速度（m/s，NED）。
    vel_sp: Vector3f,
    /// 期望加速度（m/s²，NED）。
    acc_sp: Vector3f,
    /// 期望推力（归一化，NED）。
    thr_sp: Vector3f,
    /// 期望航向（rad）。
    yaw_sp: f32,
    /// 期望偏航角速度（rad/s）。
    yawspeed_sp: f32,
}

impl Default for PositionControl {
    fn default() -> Self {
        Self::new()
    }
}

impl PositionControl {
    /// 悬停推力配置/估计的下限（归一化）。
    const HOVER_THRUST_MIN: f32 = 0.1;
    /// 悬停推力配置/估计的上限（归一化）。
    const HOVER_THRUST_MAX: f32 = 0.9;

    /// PX4 `PositionControl::empty_trajectory_setpoint`。
    ///
    /// 所有设定值均为 NAN（不受控），本 crate 的消息结构不含时间戳。
    pub const EMPTY_TRAJECTORY_SETPOINT: TrajectorySetpoint = TrajectorySetpoint::EMPTY;

    /// 创建零初始状态的控制器（对应 C++ 的默认构造函数）。
    #[must_use]
    pub fn new() -> Self {
        Self {
            gain_pos_p: Vector3f::zeros(),
            gain_vel_p: Vector3f::zeros(),
            gain_vel_i: Vector3f::zeros(),
            gain_vel_d: Vector3f::zeros(),
            lim_vel_horizontal: 0.0,
            lim_vel_up: 0.0,
            lim_vel_down: 0.0,
            lim_thr_min: 0.0,
            lim_thr_max: 0.0,
            lim_thr_xy_margin: 0.0,
            lim_tilt: 0.0,
            hover_thrust: 0.0,
            decouple_horizontal_and_vertical_acceleration: true,
            pos: Vector3f::zeros(),
            vel: Vector3f::zeros(),
            vel_dot: Vector3f::zeros(),
            vel_int: Vector3f::zeros(),
            yaw: 0.0,
            pos_sp: Vector3f::zeros(),
            vel_sp: Vector3f::zeros(),
            acc_sp: Vector3f::zeros(),
            thr_sp: Vector3f::zeros(),
            yaw_sp: 0.0,
            yawspeed_sp: 0.0,
        }
    }

    /// 设置位置控制比例增益（1/s，x/y/z 三轴）。
    pub fn set_position_gains(&mut self, p: Vector3f) {
        self.gain_pos_p = p;
    }

    /// 设置速度控制 P/I/D 增益（x/y/z 三轴）。
    pub fn set_velocity_gains(&mut self, p: Vector3f, i: Vector3f, d: Vector3f) {
        self.gain_vel_p = p;
        self.gain_vel_i = i;
        self.gain_vel_d = d;
    }

    /// 设置前馈与位置控制使用的速度限幅。
    ///
    /// `vel_horizontal` 为水平速度限幅（m/s），`vel_up` 为向上限幅（m/s），
    /// `vel_down` 为向下限幅（m/s）。
    pub fn set_velocity_limits(&mut self, vel_horizontal: f32, vel_up: f32, vel_down: f32) {
        self.lim_vel_horizontal = vel_horizontal;
        self.lim_vel_up = vel_up;
        self.lim_vel_down = vel_down;
    }

    /// 设置控制器可输出的最小/最大归一化集体推力 `[0, 1]`。
    ///
    /// 最小推力会抬到 `1e-3`，以保证总能由推力向量推断姿态。
    pub fn set_thrust_limits(&mut self, min: f32, max: f32) {
        // 保证推力向量始终有足够长度来推断姿态
        self.lim_thr_min = min.max(10e-4);
        self.lim_thr_max = max;
    }

    /// 设置优先分配垂直推力时保留给水平控制的推力裕度（归一化）。
    pub fn set_horizontal_thrust_margin(&mut self, margin: f32) {
        self.lim_thr_xy_margin = margin;
    }

    /// 设置输出姿态允许的最大倾角（rad，相对水平姿态）。
    pub fn set_tilt_limit(&mut self, tilt: f32) {
        self.lim_tilt = tilt;
    }

    /// 设置悬停推力（归一化），并限制在 `[HOVER_THRUST_MIN, HOVER_THRUST_MAX]`。
    pub fn set_hover_thrust(&mut self, hover_thrust: f32) {
        self.hover_thrust = constrain(hover_thrust, Self::HOVER_THRUST_MIN, Self::HOVER_THRUST_MAX);
    }

    /// 更新悬停推力，同时调整积分项以避免输出突变。
    ///
    /// 由 `T = a_sp * Th / g - Th` 可推导：为在替换悬停推力 Th 为 Th' 后保持推力不变，
    /// 需给积分项加上 `a_sp' - a_sp`，其中 `a_sp' = (a_sp - g) * Th / Th' + g`。
    pub fn update_hover_thrust(&mut self, hover_thrust_new: f32) {
        let previous_hover_thrust = self.hover_thrust;
        self.set_hover_thrust(hover_thrust_new);

        if is_finite(self.acc_sp[2]) {
            self.vel_int[2] += (self.acc_sp[2] - CONSTANTS_ONE_G) * previous_hover_thrust
                / self.hover_thrust
                + CONSTANTS_ONE_G
                - self.acc_sp[2];
        }
    }

    /// 传入当前载具状态（位置 m、速度 m/s、加速度 m/s²、偏航 rad）。
    pub fn set_state(&mut self, states: &PositionControlStates) {
        self.pos = states.position;
        self.vel = states.velocity;
        self.yaw = states.yaw;
        self.vel_dot = states.acceleration;
    }

    /// 传入期望设定值，包含前馈项。
    ///
    /// NAN 表示无前馈；若不存在更高阶设定值，则该状态不受控。
    pub fn set_input_setpoint(&mut self, setpoint: &TrajectorySetpoint) {
        self.pos_sp = setpoint.position;
        self.vel_sp = setpoint.velocity;
        self.acc_sp = setpoint.acceleration;
        self.yaw_sp = setpoint.yaw;
        self.yawspeed_sp = setpoint.yawspeed;
    }

    /// 执行位置 P 控制与速度 PID 控制，更新推力、偏航与偏航角速度设定值。
    ///
    /// `dt` 为距上次迭代的时间（s）。返回 true 表示输出设定值可执行。
    pub fn update(&mut self, dt: f32) -> bool {
        let valid = self.input_valid();

        if valid {
            self.position_control();
            self.velocity_control(dt);

            self.yawspeed_sp = if is_finite(self.yawspeed_sp) {
                self.yawspeed_sp
            } else {
                0.0
            };
            // TODO: 更合适的方式是显式禁用偏航控制
            self.yaw_sp = if is_finite(self.yaw_sp) {
                self.yaw_sp
            } else {
                self.yaw
            };
        }

        // 必须存在有效的输出加速度与推力设定值，否则说明某处出错
        valid && self.acc_sp.is_all_finite() && self.thr_sp.is_all_finite()
    }

    /// 垂直通道诊断快照（NED）：`[vel_sp_z, vel_z, acc_sp_z, thr_sp_z]`。
    #[must_use]
    pub fn debug_vertical(&self) -> [f32; 4] {
        [self.vel_sp[2], self.vel[2], self.acc_sp[2], self.thr_sp[2]]
    }

    /// 将积分项清零。
    pub fn reset_integral(&mut self) {
        self.vel_int = Vector3f::zeros();
    }

    /// 将水平积分项清零。
    pub fn reset_integral_xy(&mut self) {
        self.vel_int[0] = 0.0;
        self.vel_int[1] = 0.0;
    }

    /// 若为 true，则计算倾角设定值时假设垂直加速度为零。
    pub fn decouple_horizontal_and_vectical_acceleration(&mut self, val: bool) {
        self.decouple_horizontal_and_vertical_acceleration = val;
    }

    /// 返回控制器输出的局部位置设定值，含 PID 输出与前馈。
    #[must_use]
    pub fn get_local_position_setpoint(&self) -> LocalPositionSetpoint {
        LocalPositionSetpoint {
            position: self.pos_sp,
            velocity: self.vel_sp,
            acceleration: self.acc_sp,
            thrust: self.thr_sp,
            yaw: self.yaw_sp,
            yawspeed: self.yawspeed_sp,
        }
    }

    /// 返回由位置/速度控制结果生成的期望姿态设定值。
    #[must_use]
    pub fn get_attitude_setpoint(&self) -> VehicleAttitudeSetpoint {
        let mut attitude_setpoint = VehicleAttitudeSetpoint::default();
        thrust_to_attitude(self.thr_sp, self.yaw_sp, &mut attitude_setpoint);
        attitude_setpoint.yaw_sp_move_rate = self.yawspeed_sp;
        attitude_setpoint
    }

    /// 检查输入设定值与所需状态估计是否有效。
    fn input_valid(&self) -> bool {
        let mut valid = true;

        // x、y、z 每轴至少需要一个设定值
        for i in 0..3 {
            valid = valid
                && (is_finite(self.pos_sp[i])
                    || is_finite(self.vel_sp[i])
                    || is_finite(self.acc_sp[i]));
        }

        // x 与 y 的设定值必须成对出现
        valid = valid && (is_finite(self.pos_sp[0]) == is_finite(self.pos_sp[1]));
        valid = valid && (is_finite(self.vel_sp[0]) == is_finite(self.vel_sp[1]));
        valid = valid && (is_finite(self.acc_sp[0]) == is_finite(self.acc_sp[1]));

        // 每个受控状态都需要有效的状态估计
        for i in 0..3 {
            if is_finite(self.pos_sp[i]) {
                valid = valid && is_finite(self.pos[i]);
            }

            if is_finite(self.vel_sp[i]) {
                valid = valid && is_finite(self.vel[i]) && is_finite(self.vel_dot[i]);
            }
        }

        valid
    }

    /// 位置比例控制。
    fn position_control(&mut self) {
        // P 位置控制器
        let mut vel_sp_position = (self.pos_sp - self.pos).component_mul(&self.gain_pos_p);
        // 位置设定值或状态为 NAN 时，该分量不影响速度设定值
        add_if_not_nan_vector3f(&mut self.vel_sp, vel_sp_position);
        // 约束前先清除 NAN，保证后续引用安全
        set_zero_if_nan_vector3f(&mut vel_sp_position);

        // 水平速度约束：优先满足期望位置方向的速度分量，其次才是前馈项
        let vel_sp_xy = constrain_xy(
            Vector2f::new(vel_sp_position[0], vel_sp_position[1]),
            Vector2f::new(
                self.vel_sp[0] - vel_sp_position[0],
                self.vel_sp[1] - vel_sp_position[1],
            ),
            self.lim_vel_horizontal,
        );
        self.vel_sp[0] = vel_sp_xy[0];
        self.vel_sp[1] = vel_sp_xy[1];
        // z 方向速度约束
        self.vel_sp[2] = constrain(self.vel_sp[2], -self.lim_vel_up, self.lim_vel_down);
    }

    /// 速度 PID 控制，并处理垂直抗积分饱和、水平裕度饱和与跟踪抗饱和。
    fn velocity_control(&mut self, dt: f32) {
        // 约束垂直速度积分项
        self.vel_int[2] = constrain(self.vel_int[2], -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

        // PID 速度控制
        let mut vel_error = self.vel_sp - self.vel;
        let acc_sp_velocity = vel_error.component_mul(&self.gain_vel_p) + self.vel_int
            - self.vel_dot.component_mul(&self.gain_vel_d);

        // 设定值或对应状态为 NAN 时不产生控制输入
        add_if_not_nan_vector3f(&mut self.acc_sp, acc_sp_velocity);

        self.acceleration_control();

        // 垂直方向抗积分饱和
        if (self.thr_sp[2] >= -self.lim_thr_min && vel_error[2] >= 0.0)
            || (self.thr_sp[2] <= -self.lim_thr_max && vel_error[2] <= 0.0)
        {
            vel_error[2] = 0.0;
        }

        // 优先垂直控制，同时为水平控制保留裕度
        let thrust_sp_xy = Vector2f::new(self.thr_sp[0], self.thr_sp[1]);
        let thrust_sp_xy_norm = thrust_sp_xy.norm();
        let thrust_max_squared = sq(self.lim_thr_max);

        // 在保留水平裕度的前提下，计算剩余可用垂直推力
        let allocated_horizontal_thrust = thrust_sp_xy_norm.min(self.lim_thr_xy_margin);
        let thrust_z_max_squared = thrust_max_squared - sq(allocated_horizontal_thrust);

        // 饱和最大垂直推力
        self.thr_sp[2] = self.thr_sp[2].max(-thrust_z_max_squared.sqrt());

        // 垂直控制优先后，计算剩余可用水平推力
        let thrust_max_xy_squared = thrust_max_squared - sq(self.thr_sp[2]);
        let mut thrust_max_xy = 0.0;

        if thrust_max_xy_squared > 0.0 {
            thrust_max_xy = thrust_max_xy_squared.sqrt();
        }

        // 饱和水平推力
        if thrust_sp_xy_norm > thrust_max_xy {
            let thrust_sp_xy_scaled = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
            self.thr_sp[0] = thrust_sp_xy_scaled[0];
            self.thr_sp[1] = thrust_sp_xy_scaled[1];
        }

        // 水平方向使用跟踪抗饱和（ARW）：饱和期间用积分项把输出拉回非饱和
        // 参见 Anti-Reset Windup for PID controllers, L. Rundqwist, 1990
        let acc_sp_xy_produced =
            Vector2f::new(self.thr_sp[0], self.thr_sp[1]) * (CONSTANTS_ONE_G / self.hover_thrust);

        // 由于饱和与实际垂直推力（独立计算）的影响，实际产生的加速度可能大于或小于期望值；
        // 仅在信号饱和时运行 ARW 回路
        if self.acc_sp.fixed_rows::<2>(0).norm_squared() > acc_sp_xy_produced.norm_squared() {
            let arw_gain = 2.0 / self.gain_vel_p[0];
            let acc_sp_xy = Vector2f::new(self.acc_sp[0], self.acc_sp[1]);

            let vel_error_xy = Vector2f::new(vel_error[0], vel_error[1])
                - (acc_sp_xy - acc_sp_xy_produced) * arw_gain;
            vel_error[0] = vel_error_xy[0];
            vel_error[1] = vel_error_xy[1];
        }

        // 保证积分项不出现 NAN
        set_zero_if_nan_vector3f(&mut vel_error);
        // 更新速度控制积分项
        self.vel_int += vel_error.component_mul(&self.gain_vel_i) * dt;
    }

    /// 加速度设定值处理：生成机体 z 轴、限制倾角并换算集体推力。
    fn acceleration_control(&mut self) {
        // 姿态生成时垂直方向按标准重力加速度处理
        let mut z_specific_force = -CONSTANTS_ONE_G;

        if !self.decouple_horizontal_and_vertical_acceleration {
            // 纳入垂直加速度设定值以改善水平加速度跟踪
            z_specific_force += self.acc_sp[2];
        }

        let mut body_z =
            Vector3f::new(-self.acc_sp[0], -self.acc_sp[1], -z_specific_force).normalize();
        limit_tilt(&mut body_z, Vector3f::z(), self.lim_tilt);
        // 按悬停推力对应标准重力，把加速度换算为 NED 推力
        let thrust_ned_z =
            self.acc_sp[2] * (self.hover_thrust / CONSTANTS_ONE_G) - self.hover_thrust;
        // 把推力投影到规划出的机体姿态上
        let cos_ned_body = Vector3f::z().dot(&body_z);
        let collective_thrust = (thrust_ned_z / cos_ned_body).min(-self.lim_thr_min);
        self.thr_sp = body_z * collective_thrust;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAXIMUM_THRUST: f32 = 0.9;
    const HORIZONTAL_THRUST_MARGIN: f32 = 0.3;
    const DT: f32 = 0.1;

    fn assert_close(actual: f32, expected: f32, eps: f32) {
        assert!(
            (actual - expected).abs() <= eps,
            "actual {actual}, expected {expected}"
        );
    }

    /// PX4 `Eulerf` 的 roll/pitch/yaw 提取（rad）。
    fn euler(q: Quatf) -> (f32, f32, f32) {
        let rotation = q.to_rotation_matrix();
        let m = rotation.matrix();
        let phi = m[(2, 1)].atan2(m[(2, 2)]);
        let theta = -m[(2, 0)].clamp(-1.0, 1.0).asin();
        let psi = m[(1, 0)].atan2(m[(0, 0)]);
        (phi, theta, psi)
    }

    /// 对应 `PositionControlBasicTest` 夹具。
    struct Fixture {
        control: PositionControl,
        input: TrajectorySetpoint,
        output: LocalPositionSetpoint,
        attitude: VehicleAttitudeSetpoint,
    }

    impl Fixture {
        fn new() -> Self {
            let mut control = PositionControl::new();
            control.set_position_gains(Vector3f::new(1.0, 1.0, 1.0));
            control.set_velocity_gains(
                Vector3f::new(20.0, 20.0, 20.0),
                Vector3f::new(20.0, 20.0, 20.0),
                Vector3f::new(20.0, 20.0, 20.0),
            );
            control.set_velocity_limits(1.0, 1.0, 1.0);
            control.set_thrust_limits(0.1, MAXIMUM_THRUST);
            control.set_horizontal_thrust_margin(HORIZONTAL_THRUST_MARGIN);
            control.set_tilt_limit(1.0);
            control.set_hover_thrust(0.5);

            Self {
                control,
                input: PositionControl::EMPTY_TRAJECTORY_SETPOINT,
                output: LocalPositionSetpoint::default(),
                attitude: VehicleAttitudeSetpoint::default(),
            }
        }

        fn run(&mut self) -> bool {
            self.control.set_input_setpoint(&self.input);
            let ret = self.control.update(DT);
            self.output = self.control.get_local_position_setpoint();
            self.attitude = self.control.get_attitude_setpoint();
            ret
        }
    }

    #[test]
    fn empty_setpoint() {
        let control = PositionControl::new();
        let output = control.get_local_position_setpoint();
        assert_eq!(output.position, Vector3f::zeros());
        assert_eq!(output.velocity, Vector3f::zeros());
        assert_eq!(output.acceleration, Vector3f::zeros());
        assert_eq!(output.thrust, Vector3f::zeros());
        assert_close(output.yaw, 0.0, 1e-6);
        assert_close(output.yawspeed, 0.0, 1e-6);

        let attitude = control.get_attitude_setpoint();
        assert_eq!(attitude.q_d, Quatf::identity());
        assert_eq!(attitude.thrust_body, Vector3f::zeros());
        assert_close(attitude.yaw_sp_move_rate, 0.0, 1e-6);
    }

    #[test]
    fn hover_setpoint_has_zero_horizontal_thrust_and_hover_thrust() {
        let mut fixture = Fixture::new();
        fixture.input.velocity = Vector3f::zeros();

        assert!(fixture.run());
        assert_close(fixture.output.thrust[0], 0.0, 1e-6);
        assert_close(fixture.output.thrust[1], 0.0, 1e-6);
        assert_close(fixture.output.thrust[2], -0.5, 1e-6);
        assert_close(fixture.attitude.thrust_body[2], -0.5, 1e-6);
        assert!((fixture.attitude.q_d * Vector3f::z() - Vector3f::z()).norm() < 1e-5);
    }

    #[test]
    fn position_step_gives_velocity_in_position_direction() {
        let mut fixture = Fixture::new();
        fixture.input.position = Vector3f::new(1.0, 0.0, 0.0);

        assert!(fixture.run());
        // 位置阶跃产生同向速度设定值，并被水平速度限幅约束
        assert!(fixture.output.velocity[0] > 0.0);
        assert!(fixture.output.velocity[0] <= 1.0 + 1e-6);
        assert_close(fixture.output.velocity[1], 0.0, 1e-6);
        assert!(fixture.output.thrust[0] > 0.0);
    }

    #[test]
    fn position_direction() {
        let mut fixture = Fixture::new();
        fixture.input.position = Vector3f::new(0.1, 0.1, -0.1);

        assert!(fixture.run());
        let thrust = fixture.output.thrust;
        assert!(thrust[0] > 0.0);
        assert!(thrust[1] > 0.0);
        assert!(thrust[2] < 0.0);

        let body_z = fixture.attitude.q_d * Vector3f::z();
        assert!(body_z[0] < 0.0);
        assert!(body_z[1] < 0.0);
        assert!(body_z[2] > 0.0);
    }

    #[test]
    fn velocity_direction() {
        let mut fixture = Fixture::new();
        fixture.input.velocity = Vector3f::new(0.1, 0.1, -0.1);

        assert!(fixture.run());
        let thrust = fixture.output.thrust;
        assert!(thrust[0] > 0.0);
        assert!(thrust[1] > 0.0);
        assert!(thrust[2] < 0.0);

        let body_z = fixture.attitude.q_d * Vector3f::z();
        assert!(body_z[0] < 0.0);
        assert!(body_z[1] < 0.0);
        assert!(body_z[2] > 0.0);
    }

    #[test]
    fn tilt_limit() {
        let mut fixture = Fixture::new();
        fixture.input.position = Vector3f::new(10.0, 10.0, 0.0);

        assert!(fixture.run());
        let mut body_z = fixture.attitude.q_d * Vector3f::z();
        let mut angle = body_z.dot(&Vector3f::z()).acos();
        assert!(angle > 0.0);
        assert!(angle <= 1.0);

        fixture.control.set_tilt_limit(0.5);
        assert!(fixture.run());
        body_z = fixture.attitude.q_d * Vector3f::z();
        angle = body_z.dot(&Vector3f::z()).acos();
        assert!(angle > 0.0);
        assert!(angle <= 0.500_01);

        fixture.control.set_tilt_limit(1.0); // 恢复原值
    }

    #[test]
    fn velocity_limit() {
        let mut fixture = Fixture::new();
        fixture.input.position = Vector3f::new(10.0, 10.0, -10.0);

        assert!(fixture.run());
        let velocity_xy = Vector2f::new(fixture.output.velocity[0], fixture.output.velocity[1]);
        assert!(velocity_xy.norm() <= 1.0);
        assert!(fixture.output.velocity[2].abs() <= 1.0);
    }

    #[test]
    fn position_control_max_thrust_limit() {
        let mut fixture = Fixture::new();
        // 让控制器同时进入垂直与水平饱和
        fixture.input.position = Vector3f::new(10.0, 10.0, -10.0);

        assert!(fixture.run());
        let thrust = fixture.output.thrust;

        // 推力向量模长受最大推力限制
        assert_close(thrust.norm(), MAXIMUM_THRUST, 1e-4);

        // 水平推力受裕度限制
        assert_close(thrust[0], HORIZONTAL_THRUST_MARGIN / 2.0_f32.sqrt(), 1e-5);
        assert_close(thrust[1], HORIZONTAL_THRUST_MARGIN / 2.0_f32.sqrt(), 1e-5);
        assert_close(
            thrust[2],
            -(MAXIMUM_THRUST * MAXIMUM_THRUST
                - HORIZONTAL_THRUST_MARGIN * HORIZONTAL_THRUST_MARGIN)
                .sqrt(),
            1e-5,
        );

        // 集体推力受最大推力限制
        assert_close(fixture.attitude.thrust_body[0], 0.0, 1e-6);
        assert_close(fixture.attitude.thrust_body[1], 0.0, 1e-6);
        assert_close(fixture.attitude.thrust_body[2], -MAXIMUM_THRUST, 1e-5);

        // 水平裕度产生的倾角等于 水平裕度 / 最大推力
        let (phi, _, _) = euler(fixture.attitude.q_d);
        let expected = ((HORIZONTAL_THRUST_MARGIN / 2.0_f32.sqrt()) / MAXIMUM_THRUST).asin();
        assert_close(phi, expected, 1e-5);
    }

    #[test]
    fn position_control_min_thrust_limit() {
        let mut fixture = Fixture::new();
        fixture.input.position = Vector3f::new(10.0, 0.0, 10.0);

        assert!(fixture.run());
        assert_close(fixture.output.thrust.norm(), 0.1, 1e-6);
        assert_close(fixture.attitude.thrust_body[2], -0.1, 1e-6);

        let (phi, theta, _) = euler(fixture.attitude.q_d);
        assert_close(phi, 0.0, 1e-6);
        assert_close(theta, -1.0, 1e-6);
    }

    #[test]
    fn failsafe_input() {
        let mut fixture = Fixture::new();
        fixture.input.acceleration[0] = 0.0;
        fixture.input.acceleration[1] = 0.0;
        fixture.input.velocity[2] = 0.1;

        assert!(fixture.run());
        assert_close(fixture.attitude.thrust_body[0], 0.0, 1e-6);
        assert_close(fixture.attitude.thrust_body[1], 0.0, 1e-6);
        assert!(fixture.output.thrust[2] < -0.1);
        assert!(fixture.output.thrust[2] > -0.5);
        assert!(fixture.attitude.thrust_body[2] > -0.5);
        assert!(fixture.attitude.thrust_body[2] <= -0.1);
    }

    #[test]
    fn idle_thrust_input() {
        let mut fixture = Fixture::new();
        // 很大的向下加速度，确保不产生推力
        fixture.input.acceleration = Vector3f::new(0.0, 0.0, 100.0);

        assert!(fixture.run());
        assert_close(fixture.output.thrust[0], 0.0, 1e-6);
        assert_close(fixture.output.thrust[1], 0.0, 1e-6);
        assert_close(fixture.output.thrust[2], -0.1, 1e-6); // 最小推力
    }

    #[test]
    fn input_combinations_position() {
        let mut fixture = Fixture::new();
        fixture.input.position = Vector3f::new(0.1, 0.2, 0.3);

        assert!(fixture.run());
        assert_close(fixture.output.position[0], 0.1, 1e-6);
        assert_close(fixture.output.position[1], 0.2, 1e-6);
        assert_close(fixture.output.position[2], 0.3, 1e-6);
        assert!(fixture.output.velocity.iter().all(|v| v.is_finite()));
        assert!(fixture.output.thrust.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn input_combinations_position_velocity() {
        let mut fixture = Fixture::new();
        fixture.input.velocity[0] = 0.1;
        fixture.input.velocity[1] = 0.2;
        fixture.input.position[2] = 0.3; // 高度

        assert!(fixture.run());
        assert!(fixture.output.position[0].is_nan());
        assert!(fixture.output.position[1].is_nan());
        assert_close(fixture.output.position[2], 0.3, 1e-6);
        assert_close(fixture.output.velocity[0], 0.1, 1e-6);
        assert_close(fixture.output.velocity[1], 0.2, 1e-6);
        assert!(fixture.output.velocity[2].is_finite());
        assert!(fixture.output.thrust.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn setpoint_validity_simple() {
        let mut fixture = Fixture::new();
        assert!(!fixture.run());

        fixture.input.position[0] = 0.1;
        assert!(!fixture.run());

        fixture.input.position[1] = 0.2;
        assert!(!fixture.run());

        fixture.input.acceleration[2] = 0.3;
        assert!(fixture.run());
    }

    fn set_input_component(input: &mut TrajectorySetpoint, index: usize, value: f32) {
        match index {
            0 => input.position[0] = value,
            1 => input.velocity[0] = value,
            2 => input.acceleration[0] = value,
            3 => input.position[1] = value,
            4 => input.velocity[1] = value,
            5 => input.acceleration[1] = value,
            6 => input.position[2] = value,
            7 => input.velocity[2] = value,
            8 => input.acceleration[2] = value,
            _ => {}
        }
    }

    #[test]
    #[allow(clippy::similar_names)] // 与 PX4 测试中的 has_x/y/z_setpoint 命名保持一致
    fn setpoint_validity_all_combinations() {
        // 遍历所有“已设定/未设定（NAN）”组合，检查接受/拒绝是否符合规则
        let mut fixture = Fixture::new();

        for combination in 0..512_u32 {
            fixture.input = PositionControl::EMPTY_TRAJECTORY_SETPOINT;

            for j in 0..9_usize {
                if combination & (1 << j) != 0 {
                    // 取任意有限值，部分取值会明显触及限幅以覆盖边界组合
                    let value = combination as f32 / (j as f32 + 1.0);
                    set_input_component(&mut fixture.input, j, value);
                }
            }

            // 每轴至少一个设定值
            let has_x_setpoint = (combination & 7) != 0;
            let has_y_setpoint = ((combination >> 3) & 7) != 0;
            let has_z_setpoint = ((combination >> 6) & 7) != 0;
            // x/y 设定值必须成对出现
            let has_xy_pairs = (combination & 7) == ((combination >> 3) & 7);
            let expected = has_x_setpoint && has_y_setpoint && has_z_setpoint && has_xy_pairs;

            assert_eq!(fixture.run(), expected, "combination {combination}");
        }
    }

    #[test]
    fn position_setpoint_invalid_state() {
        let mut fixture = Fixture::new();
        fixture.input.position = Vector3f::new(0.1, 0.2, 0.3);

        let mut states = PositionControlStates::default();

        // 所有状态有效
        fixture.control.set_state(&states);
        assert!(fixture.run());

        // 位置无效
        states.position[0] = f32::NAN;
        fixture.control.set_state(&states);
        assert!(!fixture.run());

        // 位置与速度均无效
        states.velocity[0] = f32::NAN;
        fixture.control.set_state(&states);
        assert!(!fixture.run());

        // 速度无效
        states.position[0] = 0.0;
        fixture.control.set_state(&states);
        assert!(!fixture.run());

        // 速度导数无效
        states.velocity[0] = 0.0;
        states.acceleration[1] = f32::NAN;
        fixture.control.set_state(&states);
        assert!(!fixture.run());
    }

    #[test]
    fn velocity_setpoint_invalid_state() {
        let mut fixture = Fixture::new();
        fixture.input.velocity = Vector3f::new(0.1, 0.2, 0.3);

        let mut states = PositionControlStates::default();

        // 所有状态有效
        fixture.control.set_state(&states);
        assert!(fixture.run());

        // 位置无效：速度控制仍然有效
        states.position[0] = f32::NAN;
        fixture.control.set_state(&states);
        assert!(fixture.run());

        // 位置与速度均无效
        states.velocity[0] = f32::NAN;
        fixture.control.set_state(&states);
        assert!(!fixture.run());

        // 速度无效
        states.position[0] = 0.0;
        fixture.control.set_state(&states);
        assert!(!fixture.run());

        // 速度导数无效
        states.velocity[0] = 0.0;
        states.acceleration[1] = f32::NAN;
        fixture.control.set_state(&states);
        assert!(!fixture.run());
    }

    #[test]
    fn update_hover_thrust() {
        // 给定悬停推力且速度变化为零
        let hover_thrust = 0.6;
        let mut fixture = Fixture::new();
        fixture.control.set_hover_thrust(hover_thrust);
        fixture.input.velocity = Vector3f::zeros();

        assert!(fixture.run());
        assert_close(fixture.output.thrust[2], -hover_thrust, 1e-6);

        // 通过 update_hover_thrust 改变悬停推力后，积分项被调整以保持输出连续
        let hover_thrust_new = 0.7;
        fixture.control.update_hover_thrust(hover_thrust_new);
        assert!(fixture.run());
        assert_close(fixture.output.thrust[2], -hover_thrust, 1e-6);
    }

    #[test]
    fn velocity_integrator_is_clamped_to_gravity() {
        let mut fixture = Fixture::new();
        fixture.input.velocity = Vector3f::zeros();
        assert!(fixture.run());

        fixture.control.vel_int[2] = 100.0;
        assert!(fixture.run());
        assert_close(fixture.control.vel_int[2], CONSTANTS_ONE_G, 1e-4);

        fixture.control.vel_int[2] = -100.0;
        assert!(fixture.run());
        assert_close(fixture.control.vel_int[2], -CONSTANTS_ONE_G, 1e-4);
    }

    #[test]
    fn integrator_windup_with_invalid_setpoint() {
        // 用包含部分有效值的无效设定值运行
        let mut fixture = Fixture::new();
        fixture.input.position[0] = 0.1;
        fixture.input.position[1] = 0.2;
        // z 轴设定值保持 NAN
        assert!(!fixture.run());

        // 再用有效设定值运行
        fixture.input = PositionControl::EMPTY_TRAJECTORY_SETPOINT;
        fixture.input.velocity = Vector3f::zeros();
        assert!(fixture.run());

        // 积分项没有累积出意外偏差
        let (phi, theta, _) = euler(fixture.attitude.q_d);
        assert_close(phi, 0.0, 1e-6);
        assert_close(theta, 0.0, 1e-6);
    }
}
