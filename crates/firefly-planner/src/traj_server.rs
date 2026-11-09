//! 轨迹执行输出：接收完整轨迹与心跳，独立采样并计算航向。
use firefly_trajectory::Trajectory;
use nalgebra::Vector3;
const TIME_FORWARD: f64 = 1.0;
const YAW_DOT_MAX: f64 = 2.0 * std::f64::consts::PI;
const YAW_DOT_DOT_MAX: f64 = 5.0 * std::f64::consts::PI;
/// 参考状态指令（闭环 PD 跟踪目标）。
#[derive(Debug, Clone, Copy)]
pub struct Reference {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    /// 参考加速度（m/s²，地图系）：位置环前馈项。
    pub acceleration: Vector3<f64>,
    /// 参考偏航角（rad，地图系 x 轴为零向，包装在 [-π,π]；对照官方
    /// `traj_server::calculate_yaw` 的限幅输出）。
    pub yaw: f64,
    /// 参考偏航角速度（rad/s，两级限幅后的实际变化率；非官方发布值——
    /// 官方 `position_cmd.yaw_dot` 实际携带的是未限幅期望航向，此处按字段
    /// 语义取限幅后的角速度）。
    pub yaw_dot: f64,
}

#[derive(Clone)]
pub struct TrajectoryCommand {
    pub trajectory: Trajectory,
    pub start_time: f64,
    pub traj_id: u32,
}

pub struct TrajectoryServer {
    trajectory: Option<TrajectoryCommand>,
    received: bool,
    heartbeat: Option<f64>,
    timeout: f64,
    yaw_hold: bool,
    pub(crate) last_yaw: f64,
    pub(crate) last_yaw_dot: f64,
    pub(crate) last_yaw_time: Option<f64>,
    last_position: Vector3<f64>,
}
impl TrajectoryServer {
    #[must_use]
    pub fn new(yaw_hold: bool, timeout: f64) -> Self {
        Self {
            trajectory: None,
            received: false,
            heartbeat: None,
            timeout,
            yaw_hold,
            last_yaw: 0.0,
            last_yaw_dot: 0.0,
            last_yaw_time: None,
            last_position: Vector3::zeros(),
        }
    }
    pub fn receive(&mut self, command: TrajectoryCommand) {
        self.trajectory = Some(command);
        self.received = true;
    }
    pub fn heartbeat(&mut self, now: f64) {
        self.heartbeat = Some(now);
    }
    pub fn invalidate(&mut self) {
        self.trajectory = None;
        self.received = false;
    }
    #[must_use]
    pub const fn yaw_state(&self) -> (f64, f64) {
        (self.last_yaw, self.last_yaw_dot)
    }
    /// 返回本拍的发布顺序：超时保持在前，正常轨迹指令在后。
    /// 上游超时分支不提前返回，本拍继续采样，下一拍停止。
    #[must_use]
    pub fn sample(&mut self, now: f64, heartbeat_now: f64) -> [Option<Reference>; 2] {
        let Some(heartbeat) = self.heartbeat else {
            return [None, None];
        };
        if !self.received {
            return [None, None];
        }
        let Some(command) = self.trajectory.as_ref() else {
            return [None, None];
        };
        let hold = if heartbeat_now - heartbeat > self.timeout {
            self.received = false;
            Some(Reference {
                position: self.last_position,
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
                yaw: self.last_yaw,
                yaw_dot: 0.0,
            })
        } else {
            None
        };
        let elapsed = now - command.start_time;
        if elapsed < 0.0 || elapsed >= command.trajectory.duration() {
            return [hold, None];
        }
        let duration = command.trajectory.duration();
        let reference = {
            let state = command.trajectory.eval(elapsed);
            let target = command
                .trajectory
                .eval((elapsed + TIME_FORWARD).min(duration))
                .position;
            let (yaw, yaw_dot) = self.update_yaw(state.position, target, now);
            self.last_position = state.position;
            Reference {
                position: state.position,
                velocity: state.velocity,
                acceleration: state.acceleration,
                yaw,
                yaw_dot,
            }
        };
        [hold, Some(reference)]
    }
    pub(crate) fn update_yaw(
        &mut self,
        pos: Vector3<f64>,
        target: Vector3<f64>,
        now: f64,
    ) -> (f64, f64) {
        if self.yaw_hold {
            // 本项目扩展：锁定起飞朝向横飞，不跟速度方向（官方无此模式）。
            self.last_yaw_time = Some(now);
            self.last_yaw_dot = 0.0;
            return (self.last_yaw, 0.0);
        }
        let Some(t_last) = self.last_yaw_time else {
            self.last_yaw_time = Some(now);
            return (self.last_yaw, self.last_yaw_dot);
        };
        let dt = now - t_last;
        if dt <= 1e-9 {
            // 官方依赖 ros 定时器保证 dt > 0；此处显式防护除零
            return (self.last_yaw, self.last_yaw_dot);
        }
        self.last_yaw_time = Some(now);

        let dir = target - pos;
        let yaw_temp = if dir.norm() > 0.1 {
            dir.y.atan2(dir.x)
        } else {
            self.last_yaw
        };

        let mut d_yaw = yaw_temp - self.last_yaw;
        if d_yaw >= std::f64::consts::PI {
            d_yaw -= 2.0 * std::f64::consts::PI;
        }
        if d_yaw <= -std::f64::consts::PI {
            d_yaw += 2.0 * std::f64::consts::PI;
        }

        let rate_cap = if d_yaw >= 0.0 {
            YAW_DOT_MAX
        } else {
            -YAW_DOT_MAX
        };
        let accel_cap = if d_yaw >= 0.0 {
            YAW_DOT_DOT_MAX
        } else {
            -YAW_DOT_DOT_MAX
        };
        let d_yaw_max = if (self.last_yaw_dot + dt * accel_cap).abs() <= rate_cap.abs() {
            // 加速段可达：匀加速位移剖面
            self.last_yaw_dot * dt + 0.5 * accel_cap * dt * dt
        } else {
            // 本帧内先匀加速到限值再匀速：梯形面积
            let t1 = (rate_cap - self.last_yaw_dot) / accel_cap;
            ((dt - t1) + dt) * (rate_cap - self.last_yaw_dot) / 2.0
        };
        if d_yaw.abs() > d_yaw_max.abs() {
            d_yaw = d_yaw_max;
        }
        let yaw_dot = d_yaw / dt;
        let mut yaw = self.last_yaw + d_yaw;
        if yaw > std::f64::consts::PI {
            yaw -= 2.0 * std::f64::consts::PI;
        }
        if yaw < -std::f64::consts::PI {
            yaw += 2.0 * std::f64::consts::PI;
        }
        self.last_yaw = yaw;
        self.last_yaw_dot = yaw_dot;
        (yaw, yaw_dot)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
    fn command() -> TrajectoryCommand {
        let start = Endpoint {
            position: Vector3::new(1.0, 2.0, 3.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(5.0, 2.0, 3.0),
            ..start
        };
        let trajectory = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[], &[4.0])
            .unwrap()
            .solve()
            .unwrap();
        TrajectoryCommand {
            trajectory,
            start_time: 1.0,
            traj_id: 1,
        }
    }
    #[test]
    fn heartbeat_without_trajectory_never_publishes() {
        let mut server = TrajectoryServer::new(false, 0.5);
        assert!(server.sample(1.0, 0.0).iter().all(Option::is_none));
        server.heartbeat(0.0);
        assert!(server.sample(1.1, 0.1).iter().all(Option::is_none));
        assert!(server.sample(2.0, 1.0).iter().all(Option::is_none));
    }

    #[test]
    fn heartbeat_timeout_invalidates_trajectory_until_another_is_received() {
        let mut server = TrajectoryServer::new(false, 0.5);
        server.receive(command());
        assert!(server.sample(1.0, 0.0)[1].is_none());
        server.heartbeat(0.0);
        let tracked = server.sample(2.0, 0.2)[1].unwrap();
        assert!(tracked.velocity.norm() > 0.0);
        let expired = server.sample(2.1, 0.51);
        let stopped = expired[0].unwrap();
        let continued = expired[1].unwrap();
        assert!(continued.velocity.norm() > 0.);
        assert!((continued.position - command().trajectory.eval(1.1).position).norm() < 1e-12);
        assert_eq!(stopped.position, tracked.position);
        assert_eq!(stopped.velocity, Vector3::zeros());
        assert_eq!(stopped.acceleration, Vector3::zeros());
        server.heartbeat(0.52);
        assert!(server.sample(2.2, 0.52)[1].is_none());
        server.receive(command());
        assert!(server.sample(2.2, 0.52)[1].is_some());
    }
    #[test]
    fn complete_snapshot_can_be_sampled_without_running_planning() {
        let command = command();
        let mut server = TrajectoryServer::new(false, 0.5);
        server.receive(command.clone());
        server.heartbeat(0.0);
        for i in 0..40 {
            let now = 1.0 + f64::from(i) * 0.01;
            let reference = server.sample(now, f64::from(i) * 0.01)[1].unwrap();
            let state = command.trajectory.eval(now - command.start_time);
            assert!((reference.position - state.position).norm() < 1e-12);
            assert!((reference.velocity - state.velocity).norm() < 1e-12);
        }
    }
    #[test]
    fn future_start_and_finished_trajectory_are_silent_like_upstream() {
        let mut server = TrajectoryServer::new(false, 0.5);
        server.receive(command());
        server.heartbeat(0.0);
        assert!(server.sample(0.99, 0.0)[1].is_none());
        let tracked = server.sample(2.0, 0.1)[1].unwrap();
        assert!(server.sample(5.0, 0.2).iter().all(Option::is_none));
        assert!(server.sample(6.0, 0.3).iter().all(Option::is_none));
        let expired = server.sample(6.1, 0.6);
        assert_eq!(expired[0].unwrap().position, tracked.position);
        assert!(expired[1].is_none());
        assert!(server.sample(6.2, 0.7).iter().all(Option::is_none));
    }
}
