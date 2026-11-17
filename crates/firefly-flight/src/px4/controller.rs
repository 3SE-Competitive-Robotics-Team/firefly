//! 将产品 Z-up/FLU 状态接到 PX4 NED/FRD 控制链。
//! 分配器的归一化执行器输出由机体推力上限换算为牛顿；核心约定见 [`super::types`]。

use nalgebra::Vector3;

use super::allocation::{
    Airmode, ControlAllocationSequentialDesaturation, effectiveness_from_rotors,
};
use super::attitude_control::AttitudeControl;
use super::frames::{
    attitude_to_frd_ned, body_to_frd, rate_to_frd, world_to_ned, yaw_of_frd_ned, yaw_to_ned,
};
use super::position_control::PositionControl;
use super::rate_control::RateControl;
use super::types::{
    AXIS_PITCH, AXIS_ROLL, AXIS_YAW, ActuatorVector, ControlVector, PositionControlStates, Quatf,
    TrajectorySetpoint, VehicleAttitudeSetpoint, VehicleRatesSetpoint, VehicleTorqueSetpoint,
    constrain, radians,
};
use crate::params::Px4Params;
use crate::{Airframe, G, PositionSetpoint, QuadParams, QuadState};

pub struct Controller {
    params: Px4Params,
    rate: RateControl,
    attitude: AttitudeControl,
    position: PositionControl,
    allocation: ControlAllocationSequentialDesaturation,
    /// 上次装配的机体签名；变化时重建效能矩阵与悬停推力。
    airframe_signature: Option<u64>,
    /// 位置控制器输出的期望姿态（FRD→NED）。
    pub attitude_setpoint: VehicleAttitudeSetpoint,
    /// 姿态控制器输出的角速度设定值（FRD，rad/s）。
    pub rates_setpoint: VehicleRatesSetpoint,
    /// 角速度控制器输出的归一化力矩（FRD）。
    pub torque_setpoint: VehicleTorqueSetpoint,
    /// 分配器实际达到的控制量（`[roll, pitch, yaw, thrust_z]`）。
    pub allocated_control: ControlVector,
    /// 归一化悬停推力（质量与推力上限的函数）。
    pub hover_thrust: f32,
    pub saturated: bool,
    /// 控制器内部认为位置设定值无效（未输出新姿态）。
    pub input_invalid: bool,
}

impl Controller {
    /// Ground-contact thrust gate: minimum plus 30% of the minimum-to-hover range.
    /// A valid previous control output is required; hover alone never confirms contact.
    #[must_use]
    pub fn landing_thrust_low(&self) -> bool {
        let minimum = self.params.position.thrust_min;
        let threshold = minimum + 0.3 * (self.hover_thrust - minimum);
        self.airframe_signature.is_some()
            && !self.input_invalid
            && self.allocated_control[3].abs() <= threshold
    }

    #[must_use]
    pub fn new(params: Px4Params) -> Self {
        let mut rate = RateControl::default();
        rate.set_pid_gains(
            Vector3::from(params.rate.k).component_mul(&Vector3::from(params.rate.p)),
            Vector3::from(params.rate.k).component_mul(&Vector3::from(params.rate.i)),
            Vector3::from(params.rate.k).component_mul(&Vector3::from(params.rate.d)),
        );
        rate.set_integrator_limit(Vector3::from(params.rate.int_lim));
        rate.set_feed_forward_gain(Vector3::from(params.rate.ff));

        let mut attitude = AttitudeControl::new();
        attitude
            .set_proportional_gain(Vector3::from(params.attitude.p), params.attitude.yaw_weight);
        attitude.set_ref_model_frequency(params.attitude.ref_model_freq);
        attitude.set_feed_forward_gain(params.attitude.ff_gain);
        attitude.set_feed_forward_limit(params.attitude.ff_limit);
        attitude.set_rate_limit(Vector3::from(params.rate.max_deg_s).map(radians));

        let mut position = PositionControl::new();
        position.set_position_gains(Vector3::from(params.position.p));
        position.set_velocity_gains(
            Vector3::from(params.position.vel_p),
            Vector3::from(params.position.vel_i),
            Vector3::from(params.position.vel_d),
        );
        position.set_velocity_limits(
            params.position.vel_max_horizontal,
            params.position.vel_max_up,
            params.position.vel_max_down,
        );
        position.set_tilt_limit(radians(params.position.tilt_max_deg));
        position.set_thrust_limits(params.position.thrust_min, params.position.thrust_max);
        position.set_horizontal_thrust_margin(params.position.horizontal_thrust_margin);

        let allocation = ControlAllocationSequentialDesaturation::new();

        Self {
            params,
            rate,
            attitude,
            position,
            allocation,
            airframe_signature: None,
            attitude_setpoint: VehicleAttitudeSetpoint::default(),
            rates_setpoint: VehicleRatesSetpoint::default(),
            torque_setpoint: VehicleTorqueSetpoint::default(),
            allocated_control: ControlVector::zeros(),
            hover_thrust: 0.5,
            saturated: false,
            input_invalid: false,
        }
    }

    /// 注入/更新机体参数。仅在内容变化时重建效能矩阵，避免每 tick 求伪逆。
    pub fn set_airframe(&mut self, quad: &QuadParams, airframe: &Airframe) {
        let signature = signature_of(quad, airframe);
        if self.airframe_signature == Some(signature) {
            return;
        }
        self.airframe_signature = Some(signature);

        // PX4 FRD：位置 y/z 取反，旋向取反（+1 = FRD +z 反扭矩）。
        let positions: Vec<[f32; 3]> = airframe
            .rotors
            .iter()
            .map(|r| {
                let p = body_to_frd(Vector3::from(r.position));
                [p.x, p.y, p.z]
            })
            .collect();
        let spins: Vec<f32> = airframe.rotors.iter().map(|r| -r.spin).collect();
        let effectiveness =
            effectiveness_from_rotors(&positions, &spins, airframe.torque_coefficient);

        self.allocation.set_effectiveness_matrix(
            effectiveness,
            ActuatorVector::zeros(),
            ActuatorVector::zeros(),
            true,
        );
        self.allocation.set_actuator_min(ActuatorVector::zeros());
        self.allocation
            .set_actuator_max(ActuatorVector::repeat(1.0));

        let hover = quad.mass * G / airframe.max_thrust();
        self.hover_thrust = constrain(self.params.hover_thrust.unwrap_or(hover), 0.1, 0.9);
        self.position.set_hover_thrust(self.hover_thrust);
    }

    /// 位置模式：位置/速度/加速度/偏航参考 → 4 电机推力（N）。
    ///
    /// `landed` 表示触地候选：冻结角速度积分、清零水平推力并优先分配集合推力。
    /// 位置设定值无效时保留上一次有效姿态与推力；内环输出无效时返回 `None`。
    pub fn update(
        &mut self,
        state: &QuadState,
        setpoint: &PositionSetpoint,
        quad: &QuadParams,
        airframe: &Airframe,
        dt: f32,
        landed: bool,
    ) -> Option<[f32; 4]> {
        self.set_airframe(quad, airframe);

        let q_frd_ned = attitude_to_frd_ned(state.attitude);
        self.position.set_state(&PositionControlStates {
            position: world_to_ned(state.position),
            velocity: world_to_ned(state.velocity),
            // PX4 的 D 项使用估计加速度；产品反馈没有独立加速度，取零（默认 D 增益很小）。
            acceleration: Vector3::zeros(),
            yaw: yaw_of_frd_ned(q_frd_ned),
        });

        self.position.set_input_setpoint(&TrajectorySetpoint {
            position: world_to_ned(setpoint.position),
            velocity: world_to_ned(setpoint.velocity),
            acceleration: world_to_ned(setpoint.acceleration),
            jerk: Vector3::repeat(f32::NAN),
            yaw: yaw_to_ned(setpoint.yaw),
            yawspeed: -setpoint.yaw_rate,
        });

        if !self.position.update(dt) {
            // 瞬时无效设定值不得切断推力：保留上一次有效姿态/推力，继续跑内环。
            self.input_invalid = true;
            return self.run_inner_loop(state, airframe, dt, landed, false);
        }
        self.input_invalid = false;

        self.attitude_setpoint = self.position.attitude_setpoint();
        if landed {
            let local = self.position.local_position_setpoint();
            self.position.reset_integral_xy();
            self.attitude_setpoint = VehicleAttitudeSetpoint {
                q_d: nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), local.yaw),
                thrust_body: Vector3::new(0., 0., local.thrust.z),
                yaw_sp_move_rate: 0.,
            };
        }
        self.run_inner_loop(state, airframe, dt, landed, false)
    }

    /// 手动姿态模式：给定产品系期望姿态、归一化集合推力与偏航角速度，走
    /// 姿态 → 角速度 → 分配三层（`apps/quad` 的第三人称飞行）。
    ///
    /// `attitude_sp` 为产品 body→world（FLU）；`thrust` 为 `[0,1]` 归一化集合推力；
    /// `yaw_rate` 绕产品世界 `+Z`（rad/s）。返回 4 电机推力（N）。
    #[allow(clippy::too_many_arguments)] // 三组输入 + 机体/时间，拆结构只会增加搬运
    pub fn attitude_mode(
        &mut self,
        state: &QuadState,
        attitude_sp: Quatf,
        thrust: f32,
        yaw_rate: f32,
        quad: &QuadParams,
        airframe: &Airframe,
        dt: f32,
    ) -> [f32; 4] {
        self.set_airframe(quad, airframe);
        self.attitude_setpoint = VehicleAttitudeSetpoint {
            q_d: attitude_to_frd_ned(attitude_sp),
            thrust_body: Vector3::new(0.0, 0.0, -constrain(thrust, 0.0, 1.0)),
            yaw_sp_move_rate: -yaw_rate,
        };
        self.run_inner_loop(state, airframe, dt, false, false)
            .unwrap_or([0.0; 4])
    }

    /// 姿态支持：无位置反馈时保持机头指向 `yaw`（产品世界系）与悬停推力。
    ///
    /// 只走姿态/角速度/分配三层，不使用位置反馈；不保证保高或定点。
    pub fn support(
        &mut self,
        state: &QuadState,
        yaw: f32,
        quad: &QuadParams,
        airframe: &Airframe,
        dt: f32,
    ) -> Option<[f32; 4]> {
        self.set_airframe(quad, airframe);

        let yaw_ned = yaw_to_ned(yaw);
        self.attitude_setpoint = VehicleAttitudeSetpoint {
            q_d: nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw_ned),
            thrust_body: Vector3::new(0.0, 0.0, -self.hover_thrust),
            yaw_sp_move_rate: 0.0,
        };
        // 降级模式不连续跟踪参考模型：每个 tick 把参考姿态吸附到当前设定值，
        // 避免从上一模式的航向系平滑过渡时产生大角速度指令。
        self.run_inner_loop(state, airframe, dt, false, true)
    }

    /// 姿态 → 角速度 → 力矩 → 分配，四个电机推力（N）。
    fn run_inner_loop(
        &mut self,
        state: &QuadState,
        airframe: &Airframe,
        dt: f32,
        landed: bool,
        snap_attitude_reference: bool,
    ) -> Option<[f32; 4]> {
        let q_frd_ned = attitude_to_frd_ned(state.attitude);
        self.attitude.update_reference(
            self.attitude_setpoint.q_d,
            self.attitude_setpoint.yaw_sp_move_rate,
            if snap_attitude_reference { -1.0 } else { dt },
        );
        let rates = self.attitude.update(q_frd_ned);
        self.rates_setpoint = VehicleRatesSetpoint {
            roll: rates.x,
            pitch: rates.y,
            yaw: rates.z,
            thrust_body: self.attitude_setpoint.thrust_body,
        };

        let rate_frd = rate_to_frd(state.ang_vel);
        // 角加速度反馈缺省为零；PX4 用 `vehicle_angular_velocity.xyz_derivative`。
        let torque = self
            .rate
            .update(rate_frd, rates, Vector3::zeros(), dt, landed);
        self.torque_setpoint = VehicleTorqueSetpoint { xyz: torque };

        let control = ControlVector::from_column_slice(&[
            torque.x,
            torque.y,
            torque.z,
            self.attitude_setpoint.thrust_body.z,
        ]);
        self.allocation.set_airmode(if landed {
            Airmode::Disabled
        } else {
            match self.params.allocation.airmode {
                1 => Airmode::RollPitch,
                2 => Airmode::RollPitchYaw,
                _ => Airmode::Disabled,
            }
        });
        self.allocation.set_control_setpoint(control);
        self.allocation.allocate();
        let actuator_sp = self
            .allocation
            .clip_actuator_setpoint(self.allocation.actuator_setpoint());
        self.allocation.set_actuator_setpoint(actuator_sp);
        self.allocated_control = self.allocation.allocated_control();

        // 下一 tick 的抗饱和反馈：未分配到的力矩（PX4 `control_allocator_status`）。
        let unallocated = control - self.allocated_control;
        let positive = [
            unallocated[AXIS_ROLL] > f32::EPSILON,
            unallocated[AXIS_PITCH] > f32::EPSILON,
            unallocated[AXIS_YAW] > f32::EPSILON,
        ];
        let negative = [
            unallocated[AXIS_ROLL] < -f32::EPSILON,
            unallocated[AXIS_PITCH] < -f32::EPSILON,
            unallocated[AXIS_YAW] < -f32::EPSILON,
        ];
        self.rate.set_saturation_status(positive, negative);
        self.saturated = unallocated.norm() > 1e-4;

        let max = airframe.max_thrust_per_motor;
        let motors = std::array::from_fn(|i| actuator_sp[i].clamp(0.0, 1.0) * max);
        motors.iter().all(|m| m.is_finite()).then_some(motors)
    }

    pub fn reset(&mut self) {
        self.rate.reset_integral();
        self.position.reset_integral();
    }

    #[must_use]
    pub fn hover_thrust(&self) -> f32 {
        self.hover_thrust
    }

    /// 位置控制垂直通道诊断快照（NED）：`[vel_sp_z, vel_z, acc_sp_z, thr_sp_z]`。
    #[must_use]
    pub fn debug_vertical(&self) -> [f32; 4] {
        self.position.debug_vertical()
    }
}

fn signature_of(quad: &QuadParams, airframe: &Airframe) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    let mut mix = |bits: u32| {
        hash ^= u64::from(bits);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    };
    for value in [
        quad.mass,
        quad.inertia[0],
        quad.inertia[1],
        quad.inertia[2],
        quad.linear_drag,
        quad.angular_drag,
        airframe.max_thrust_per_motor,
        airframe.torque_coefficient,
    ] {
        mix(value.to_bits());
    }
    for rotor in &airframe.rotors {
        for value in rotor.position {
            mix(value.to_bits());
        }
        mix(rotor.spin.to_bits());
    }
    hash
}
