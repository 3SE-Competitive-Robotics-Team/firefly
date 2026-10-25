//! PX4 `VelocitySmoothing`：jerk 受限的 1D 速度轨迹（`src/lib/motion_planning/VelocitySmoothing.{hpp,cpp}`）。
//!
//! 三个轴各用一个实例，`time_synchronization` 把多个轴的总时长对齐成直线。
//! 与 PX4 的差异只有语言层：`math::sign/constrain` → 本地辅助函数，字段名保留。
//!
//! 单位：位置 m、速度 m/s、加速度 m/s²、jerk m/s³、时间 s。

use super::types::constrain;

/// 轨迹在某时刻的 jerk/加速度/速度/位置。
#[derive(Clone, Copy, Debug, Default)]
pub struct Trajectory {
    /// jerk（m/s³）。
    pub j: f32,
    /// 加速度（m/s²）。
    pub a: f32,
    /// 速度（m/s）。
    pub v: f32,
    /// 位置（m）。
    pub x: f32,
}

/// PX4 `VelocitySmoothing`。
#[derive(Clone, Copy, Debug)]
pub struct VelocitySmoothing {
    /// 速度设定值（经 `max_vel` 限幅），m/s。
    vel_sp: f32,
    /// 最大 jerk（m/s³）。
    max_jerk: f32,
    /// 最大加速度（m/s²）。
    max_accel: f32,
    /// 最大速度（m/s）。
    max_vel: f32,
    /// 当前轨迹状态。
    state: Trajectory,
    /// 运动方向（+1 / -1 / 0）。
    direction: i32,
    /// 本段轨迹的初始状态。
    state_init: Trajectory,
    /// 加速段时长（s）。
    t1: f32,
    /// 匀速段时长（s）。
    t2: f32,
    /// 减速段时长（s）。
    t3: f32,
    /// 段内局部时间（s）。
    local_time: f32,
}

impl Default for VelocitySmoothing {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
}

impl VelocitySmoothing {
    /// 以初始加速度/速度/位置构造。
    #[must_use]
    pub fn new(initial_accel: f32, initial_vel: f32, initial_pos: f32) -> Self {
        let mut value = Self {
            vel_sp: 0.0,
            max_jerk: 0.0,
            max_accel: 0.0,
            max_vel: 0.0,
            state: Trajectory::default(),
            direction: 0,
            state_init: Trajectory::default(),
            t1: 0.0,
            t2: 0.0,
            t3: 0.0,
            local_time: 0.0,
        };
        value.reset(initial_accel, initial_vel, initial_pos);
        value
    }

    /// 重置当前状态；不清空约束。
    pub fn reset(&mut self, accel: f32, vel: f32, pos: f32) {
        self.state = Trajectory {
            j: 0.0,
            a: accel,
            v: vel,
            x: pos,
        };
        self.state_init = self.state;
    }

    /// 计算下一段轨迹的 T1/T2/T3；每周期在 `update_traj` 之前/之后调用，见 [`Self::update`]。
    pub fn update_durations(&mut self, vel_setpoint: f32) {
        self.vel_sp = constrain(vel_setpoint, -self.max_vel, self.max_vel);
        self.local_time = 0.0;
        self.state_init = self.state;
        self.direction = self.compute_direction();
        self.update_durations_minimize_total_time();
    }

    /// 推进轨迹 `dt`；`time_stretch` 可放慢/快进。
    pub fn update_traj(&mut self, dt: f32, time_stretch: f32) {
        self.local_time += dt * time_stretch;
        let mut t_remain = self.local_time;

        let t1 = t_remain.min(self.t1);
        self.state = evaluate_poly(
            self.max_jerk,
            self.state_init.a,
            self.state_init.v,
            self.state_init.x,
            t1,
            self.direction,
        );
        t_remain -= t1;

        if t_remain > 0.0 {
            let t2 = t_remain.min(self.t2);
            self.state = evaluate_poly(0.0, self.state.a, self.state.v, self.state.x, t2, 0);
            t_remain -= t2;
        }

        if t_remain > 0.0 {
            let t3 = t_remain.min(self.t3);
            self.state = evaluate_poly(
                self.max_jerk,
                self.state.a,
                self.state.v,
                self.state.x,
                t3,
                -self.direction,
            );
            t_remain -= t3;
        }

        if t_remain > 0.0 {
            self.state = evaluate_poly(0.0, 0.0, self.state.v, self.state.x, t_remain, 0);
        }
    }

    /// 一步更新（PX4 `ManualVelocitySmoothingZ::update` 的顺序：先推进、再算下一段时长）。
    pub fn update(&mut self, dt: f32, vel_setpoint: f32) {
        self.update_traj(dt, 1.0);
        self.update_durations(vel_setpoint);
    }

    /// 把多条轨迹的总时长对齐到最长者，保证同步轴走直线。
    pub fn time_synchronization(trajectories: &mut [Self]) {
        let mut desired_time = 0.0f32;
        let mut longest = 0usize;
        for (index, trajectory) in trajectories.iter().enumerate() {
            let total = trajectory.total_time();
            if total > desired_time {
                desired_time = total;
                longest = index;
            }
        }
        if desired_time > f32::EPSILON {
            for (index, trajectory) in trajectories.iter_mut().enumerate() {
                if index != longest && trajectory.total_time() < desired_time {
                    trajectory.update_durations_given_total_time(desired_time);
                }
            }
        }
    }

    /// 最大 jerk（m/s³）。
    #[must_use]
    pub fn max_jerk(&self) -> f32 {
        self.max_jerk
    }
    /// 设置最大 jerk（m/s³）。
    pub fn set_max_jerk(&mut self, max_jerk: f32) {
        self.max_jerk = max_jerk;
    }
    /// 最大加速度（m/s²）。
    #[must_use]
    pub fn max_accel(&self) -> f32 {
        self.max_accel
    }
    /// 设置最大加速度（m/s²）。
    pub fn set_max_accel(&mut self, max_accel: f32) {
        self.max_accel = max_accel;
    }
    /// 最大速度（m/s）。
    #[must_use]
    pub fn max_vel(&self) -> f32 {
        self.max_vel
    }
    /// 设置最大速度（m/s）。
    pub fn set_max_vel(&mut self, max_vel: f32) {
        self.max_vel = max_vel;
    }

    /// 当前 jerk（m/s³）。
    #[must_use]
    pub fn current_jerk(&self) -> f32 {
        self.state.j
    }
    /// 设置当前加速度（m/s²）。
    pub fn set_current_acceleration(&mut self, accel: f32) {
        self.state.a = accel;
        self.state_init.a = accel;
    }
    /// 当前加速度（m/s²）。
    #[must_use]
    pub fn current_acceleration(&self) -> f32 {
        self.state.a
    }
    /// 设置当前速度（m/s）。
    pub fn set_current_velocity(&mut self, vel: f32) {
        self.state.v = vel;
        self.state_init.v = vel;
    }
    /// 当前速度（m/s）。
    #[must_use]
    pub fn current_velocity(&self) -> f32 {
        self.state.v
    }
    /// 设置当前位置（m）。
    pub fn set_current_position(&mut self, pos: f32) {
        self.state.x = pos;
        self.state_init.x = pos;
    }
    /// 当前位置（m）。
    #[must_use]
    pub fn current_position(&self) -> f32 {
        self.state.x
    }

    /// T1（s）。
    #[must_use]
    pub fn t1(&self) -> f32 {
        self.t1
    }
    /// T2（s）。
    #[must_use]
    pub fn t2(&self) -> f32 {
        self.t2
    }
    /// T3（s）。
    #[must_use]
    pub fn t3(&self) -> f32 {
        self.t3
    }
    /// 总时长（s）。
    #[must_use]
    pub fn total_time(&self) -> f32 {
        self.t1 + self.t2 + self.t3
    }

    fn compute_direction(&self) -> i32 {
        let vel_zero_acc = self.compute_vel_at_zero_acc();
        let mut direction = sign(self.vel_sp - vel_zero_acc);
        if direction == 0 {
            direction = sign(self.state.a);
        }
        direction
    }

    fn compute_vel_at_zero_acc(&self) -> f32 {
        let mut vel_zero_acc = self.state.v;
        if self.state.a.abs() > f32::EPSILON {
            let jerk_zero_acc = -(sign(self.state.a) as f32) * self.max_jerk;
            let t_zero_acc = -self.state.a / jerk_zero_acc;
            vel_zero_acc = self.state.v
                + self.state.a * t_zero_acc
                + 0.5 * jerk_zero_acc * t_zero_acc * t_zero_acc;
        }
        vel_zero_acc
    }

    fn update_durations_minimize_total_time(&mut self) {
        let jerk_max_t1 = self.direction as f32 * self.max_jerk;
        let delta_v = self.vel_sp - self.state.v;
        if jerk_max_t1.abs() > f32::EPSILON {
            self.t1 = compute_t1(self.state.a, delta_v, jerk_max_t1, self.max_accel);
            self.t3 = compute_t3(self.t1, self.state.a, jerk_max_t1);
            self.t2 = compute_t2(self.t1, self.t3, self.state.a, delta_v, jerk_max_t1);
        } else {
            self.t1 = 0.0;
            self.t2 = 0.0;
            self.t3 = 0.0;
        }
    }

    fn update_durations_given_total_time(&mut self, t123: f32) {
        let jerk_max_t1 = self.direction as f32 * self.max_jerk;
        let delta_v = self.vel_sp - self.state.v;
        if jerk_max_t1.abs() > f32::EPSILON {
            self.t1 = compute_t1_with_total_time(
                t123,
                self.state.a,
                delta_v,
                jerk_max_t1,
                self.max_accel,
            );
            self.t3 = compute_t3(self.t1, self.state.a, jerk_max_t1);
            self.t2 = compute_t2_with_total_time(t123, self.t1, self.t3);
        } else {
            self.t1 = 0.0;
            self.t2 = 0.0;
            self.t3 = 0.0;
        }
    }
}

fn sign(value: f32) -> i32 {
    if value > 0.0 {
        1
    } else if value < 0.0 {
        -1
    } else {
        0
    }
}

// 以下四个是无状态的纯函数（只用参数），从 PX4 的成员函数拆出到模块作用域：
// 它们不读 `self`，留在 `impl` 里只会让类型职责变模糊。

fn compute_t1(a0: f32, v3: f32, jerk_max: f32, accel_max: f32) -> f32 {
    let delta = 2.0 * a0 * a0 + 4.0 * jerk_max * v3;
    if delta < 0.0 {
        return 0.0;
    }
    let sqrt_delta = delta.sqrt();
    let t1_plus = (-a0 + 0.5 * sqrt_delta) / jerk_max;
    let t1_minus = (-a0 - 0.5 * sqrt_delta) / jerk_max;
    let t3_plus = a0 / jerk_max + t1_plus;
    let t3_minus = a0 / jerk_max + t1_minus;
    let t1 = if t1_plus >= 0.0 && t3_plus >= 0.0 {
        t1_plus
    } else if t1_minus >= 0.0 && t3_minus >= 0.0 {
        t1_minus
    } else {
        0.0
    };
    saturate_t1_for_accel(a0, jerk_max, t1, accel_max).max(0.0)
}
fn compute_t1_with_total_time(t123: f32, a0: f32, v3: f32, jerk_max: f32, accel_max: f32) -> f32 {
    let a = -jerk_max;
    let b = jerk_max * t123 - a0;
    let delta = t123 * t123 * jerk_max * jerk_max + 2.0 * t123 * a0 * jerk_max
        - a0 * a0
        - 4.0 * jerk_max * v3;
    if delta < 0.0 {
        return 0.0;
    }
    let sqrt_delta = delta.sqrt();
    let denominator_inv = 1.0 / (2.0 * a);
    let t1_plus = ((-b + sqrt_delta) * denominator_inv).max(0.0);
    let t1_minus = ((-b - sqrt_delta) * denominator_inv).max(0.0);
    let t3_plus = a0 / jerk_max + t1_plus;
    let t3_minus = a0 / jerk_max + t1_minus;
    let t1 = if (t1_plus >= 0.0 && t3_plus >= 0.0) && ((t1_plus + t3_plus) <= t123) {
        t1_plus
    } else if (t1_minus >= 0.0 && t3_minus >= 0.0) && ((t1_minus + t3_minus) <= t123) {
        t1_minus
    } else {
        0.0
    };
    saturate_t1_for_accel(a0, jerk_max, t1, accel_max)
}
fn evaluate_poly(jerk: f32, a0: f32, v0: f32, x0: f32, t: f32, direction: i32) -> Trajectory {
    let jt = direction as f32 * jerk;
    let t2 = t * t;
    let t3 = t2 * t;
    Trajectory {
        j: jt,
        a: a0 + jt * t,
        v: v0 + a0 * t + 0.5 * jt * t2,
        x: x0 + v0 * t + 0.5 * a0 * t2 + 1.0 / 6.0 * jt * t3,
    }
}

fn saturate_t1_for_accel(a0: f32, jerk_max: f32, t1: f32, accel_max: f32) -> f32 {
    let accel_t1 = a0 + jerk_max * t1;
    let mut t1_new = t1;
    if accel_t1 > accel_max {
        t1_new = (accel_max - a0) / jerk_max;
    } else if accel_t1 < -accel_max {
        t1_new = (-accel_max - a0) / jerk_max;
    }
    t1_new
}

fn compute_t2(t1: f32, t3: f32, a0: f32, v3: f32, jerk_max: f32) -> f32 {
    let mut t2 = 0.0;
    let den = a0 + jerk_max * t1;
    if den.abs() > f32::EPSILON {
        t2 = (-0.5 * t1 * t1 * jerk_max - t1 * t3 * jerk_max - t1 * a0 + 0.5 * t3 * t3 * jerk_max
            - t3 * a0
            + v3)
            / den;
    }
    t2.max(0.0)
}

fn compute_t2_with_total_time(t123: f32, t1: f32, t3: f32) -> f32 {
    (t123 - t1 - t3).max(0.0)
}

fn compute_t3(t1: f32, a0: f32, jerk_max: f32) -> f32 {
    (a0 / jerk_max + t1).max(0.0)
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // PX4 测试用精确零值断言轨迹静止
mod tests {
    use super::*;

    #[test]
    fn all_zero_case() {
        let mut trajectory = VelocitySmoothing::default();
        trajectory.update_durations(0.0);
        trajectory.update_traj(0.0, 1.0);
        assert_eq!(trajectory.total_time(), 0.0);
        assert_eq!(trajectory.current_jerk(), 0.0);
        assert_eq!(trajectory.current_acceleration(), 0.0);
        assert_eq!(trajectory.current_velocity(), 0.0);
        assert_eq!(trajectory.current_position(), 0.0);
    }

    #[test]
    fn division_by_zero_is_safe() {
        let mut trajectory = VelocitySmoothing::default();
        trajectory.set_max_jerk(0.0);
        trajectory.set_max_accel(0.0);
        trajectory.set_max_vel(1.0);
        trajectory.update_durations(1.0);
        trajectory.update_traj(1.0, 1.0);
        assert_eq!(trajectory.total_time(), 0.0);
        assert_eq!(trajectory.current_velocity(), 0.0);

        let mut trajectory = VelocitySmoothing::new(1.0, 0.0, 0.0);
        trajectory.set_max_jerk(0.0);
        trajectory.set_max_accel(0.0);
        trajectory.set_max_vel(1.0);
        trajectory.update_durations(1.0);
        trajectory.update_traj(1.0, 1.0);
        assert_eq!(trajectory.total_time(), 0.0);
        assert_eq!(trajectory.current_acceleration(), 0.0);
    }

    #[test]
    fn constant_setpoint_reaches_target_with_zero_acceleration() {
        let mut trajectory = VelocitySmoothing::default();
        trajectory.set_max_jerk(55.2);
        trajectory.set_max_accel(6.0);
        trajectory.set_max_vel(6.0);
        let setpoint = -3.0;
        let dt = 0.01;
        trajectory.update_traj(0.0, 1.0);
        trajectory.update_durations(setpoint);
        let steps = (trajectory.total_time() / dt).ceil() as usize;
        for _ in 0..steps {
            trajectory.update_traj(dt, 1.0);
            assert!(trajectory.current_jerk().abs() <= trajectory.max_jerk() + 1e-4);
            assert!(
                trajectory.current_acceleration().abs() <= trajectory.max_accel() + 1e-4,
                "accel {}",
                trajectory.current_acceleration()
            );
            assert!(
                trajectory.current_velocity().abs() <= trajectory.max_vel() + 1e-4,
                "vel {}",
                trajectory.current_velocity()
            );
            trajectory.update_durations(setpoint);
        }
        assert!((trajectory.current_velocity() - setpoint).abs() < 0.01);
        assert!(trajectory.current_acceleration().abs() < 1e-4);
    }

    #[test]
    fn time_synchronization_equalizes_total_time() {
        let mut trajectories = [
            VelocitySmoothing::new(0.22, 2.47, 0.0),
            VelocitySmoothing::new(0.0, -5.59e-6, 0.0),
            VelocitySmoothing::new(0.22, 2.47, 0.0),
        ];
        for trajectory in &mut trajectories {
            trajectory.set_max_jerk(55.2);
            trajectory.set_max_accel(6.0);
            trajectory.set_max_vel(6.0);
        }
        for (trajectory, setpoint) in trajectories.iter_mut().zip([-3.0, 1.0, 0.0]) {
            trajectory.update_traj(0.0, 1.0);
            trajectory.update_durations(setpoint);
        }
        VelocitySmoothing::time_synchronization(&mut trajectories[..2]);
        assert!((trajectories[0].total_time() - trajectories[1].total_time()).abs() < 1e-4);
    }

    #[test]
    fn time_synchronization_same_delta_has_same_phases() {
        let mut trajectories = [
            VelocitySmoothing::new(0.0, 0.5, 0.0),
            VelocitySmoothing::new(0.0, -0.2, 0.0),
            VelocitySmoothing::new(0.0, 0.0, 0.0),
        ];
        for trajectory in &mut trajectories {
            trajectory.set_max_jerk(55.2);
            trajectory.set_max_accel(6.0);
            trajectory.set_max_vel(6.0);
        }
        for (trajectory, setpoint) in trajectories.iter_mut().zip([0.8, 0.1, 0.0]) {
            trajectory.update_durations(setpoint);
        }
        VelocitySmoothing::time_synchronization(&mut trajectories);
        assert!((trajectories[0].total_time() - trajectories[1].total_time()).abs() < 1e-6);
        assert!((trajectories[0].t1() - trajectories[1].t1()).abs() < 1e-6);
        assert!((trajectories[0].t2() - trajectories[1].t2()).abs() < 1e-6);
        assert!((trajectories[0].t3() - trajectories[1].t3()).abs() < 1e-6);
        // 零速度/零设定值的轴在 PX4 源码里 `jerk_max_T1 == 0`，三段时长为零；
        // `timeSynchronization` 不会给它分配常速段，因此这里只断言同步轴（XY）相位一致。
        let _ = &trajectories[2];
    }
}
