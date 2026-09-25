//! 飞控：角度模式（飞行手柄）与位置模式（飞控外环），共用姿态内环与推力限幅。
//!
//! 推力恒沿机体 `+Z`：两个模式都只决定"推力大小 + 期望姿态"，水平加速因此只能来自
//! 倾斜，姿态动力学决定响应快慢——这是四旋翼所有动态耦合的来源。

use glam::{Mat3, Quat, Vec3};

use crate::{
    G,
    params::{ControlParams, QuadParams},
    state::{QuadState, Wrench},
};

/// 角度模式指令（−1..1 归一化输入，飞行手柄）。
#[derive(Clone, Copy, Debug, Default)]
pub struct AngleCommand {
    /// 俯仰（+1 = 前倾）。
    pub pitch: f32,
    /// 横滚（+1 = 右倾）。
    pub roll: f32,
    /// 偏航角速度（+1 = 右偏，绕机体 z）。
    pub yaw: f32,
    /// 升降速度（+1 = 上升）。
    pub throttle: f32,
}

/// 位置模式参考（世界系）。
#[derive(Clone, Copy, Debug, Default)]
pub struct PositionSetpoint {
    /// 期望位置（m）。
    pub position: Vec3,
    /// 期望速度（m/s）。
    pub velocity: Vec3,
    /// 期望航向（rad，世界 Z；水平面内机体 `+X` 的方向）。
    pub yaw: f32,
}

/// 角度模式：`cmd` 的期望倾斜/偏航角速度 + 升降速度 → 世界系力/力矩。
#[must_use]
pub fn angle_mode(
    state: &QuadState,
    cmd: &AngleCommand,
    params: &QuadParams,
    ctl: &ControlParams,
) -> Wrench {
    // 期望姿态：当前航向下叠加期望倾斜。偏航不写进姿态——它是机体 z 的角速度，
    // 随倾斜一起转（真机绕机体轴偏航），故用姿态内环的角速度前馈。
    let tilt = ctl.tilt_max_deg.to_radians();
    let yaw = state.yaw();
    let q_des = Quat::from_rotation_z(yaw)
        * Quat::from_rotation_y(cmd.pitch * tilt)
        * Quat::from_rotation_x(cmd.roll * tilt);
    let torque = attitude_torque(state, q_des, -cmd.yaw * ctl.yaw_rate_max, params, ctl);

    // 垂直速度控制 + 倾斜补偿：推力沿机体 z，倾斜时须补 1/cosθ 才维持高度。
    let up_z = (state.attitude * Vec3::Z).z.clamp(0.3, 1.0);
    let vz_des = cmd.throttle * ctl.climb_rate_max;
    let a_vert = G + ctl.vz_kp * (vz_des - state.velocity.z);
    let thrust = (params.mass * a_vert / up_z).clamp(0.0, params.max_thrust);

    Wrench {
        force: state.attitude * (Vec3::Z * thrust) + drag_force(state, params),
        torque,
    }
}

/// 位置模式：位置/速度/偏航参考 → 期望推力矢量与姿态 → 姿态内环（含倾角与推力限幅）。
#[must_use]
pub fn position_mode(
    state: &QuadState,
    setpoint: &PositionSetpoint,
    params: &QuadParams,
    ctl: &ControlParams,
) -> Wrench {
    // 位置/速度 PD → 期望加速度；加上重力补偿得到所需外力（世界系）。
    let a_des = ctl.pos_kp * (setpoint.position - state.position)
        + ctl.vel_kp * (setpoint.velocity - state.velocity);
    let f_des = (a_des + Vec3::Z * G) * params.mass;

    // 可用的只有机体 z 方向的推力：大小取所需合力，方向取期望力方向（限幅在倾角锥内）。
    // 实际力始终沿**当前**机体 z，方向差由姿态内环转过来——倾斜-平移耦合即在此。
    let thrust = f_des.length().clamp(0.0, params.max_thrust);
    let q_des = attitude_from_thrust_dir(thrust_direction(f_des, ctl.tilt_max_deg), setpoint.yaw);
    let torque = attitude_torque(state, q_des, 0.0, params, ctl);

    Wrench {
        force: state.attitude * (Vec3::Z * thrust) + drag_force(state, params),
        torque,
    }
}

/// 期望推力方向：期望力方向，超出 `tilt_max` 锥时压回锥面（俯仰/横滚限幅）。
fn thrust_direction(f_des: Vec3, tilt_max_deg: f32) -> Vec3 {
    let dir = f_des.try_normalize().unwrap_or(Vec3::Z);
    let max_tilt = tilt_max_deg.to_radians();
    if dir.z.clamp(-1.0, 1.0) >= max_tilt.cos() {
        return dir;
    }
    let axis = Vec3::Z.cross(dir).try_normalize().unwrap_or(Vec3::X);
    Quat::from_axis_angle(axis, max_tilt) * Vec3::Z
}

/// 由推力方向（机体 z）+ 期望航向（机体 x 的水平投影）构造期望姿态。
fn attitude_from_thrust_dir(b3: Vec3, yaw: f32) -> Quat {
    let b1_ref = Vec3::new(yaw.cos(), yaw.sin(), 0.0);
    let b2 = b3.cross(b1_ref).try_normalize().unwrap_or(Vec3::Y);
    let b1 = b2.cross(b3);
    Quat::from_mat3(&Mat3::from_cols(b1, b2, b3))
}

/// 姿态内环（两个模式共用）：姿态误差 → 机体角速度指令 → 角加速度 → 世界系力矩。
///
/// `yaw_rate_ff`：机体 z 角速度前馈（角度模式的偏航摇杆）。角阻尼按
/// `I·(-angular_drag·ω)` 施加，与惯量无关。
fn attitude_torque(
    state: &QuadState,
    q_des: Quat,
    yaw_rate_ff: f32,
    params: &QuadParams,
    ctl: &ControlParams,
) -> Vec3 {
    // `q` 与 `-q` 表示同一旋转：`w<0` 时 `to_scaled_axis` 返回接近 2π 的大角度
    //（航向在 ±π 回绕时触发），须折叠到最近路径，否则会绕远路整圈。
    let mut q_err = state.attitude.inverse() * q_des;
    if q_err.w < 0.0 {
        q_err = -q_err;
    }
    let rate_des = q_err.to_scaled_axis() * ctl.attitude_kp + Vec3::Z * yaw_rate_ff;
    let ang_accel = (rate_des - state.ang_vel) * ctl.rate_kp;
    let torque_body =
        Vec3::from(params.inertia) * (ang_accel - state.ang_vel * params.angular_drag);
    state.attitude * torque_body
}

/// 平移阻尼（世界系力，`-k·v`）。
fn drag_force(state: &QuadState, params: &QuadParams) -> Vec3 {
    -state.velocity * params.linear_drag
}
