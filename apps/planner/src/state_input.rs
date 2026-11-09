//! 局部里程计输入：校验、墙钟存活检测。
//!
//! 对照 EGO-Planner-v2 的 `have_odom_` 门控；状态源由上游估计服务封装。
//! 不依赖观测算法或地图匹配成功频率。
//! 失联锁存须重启进程解除，轨迹参考不能充当状态观测。

use std::time::{Duration, Instant};

use firefly_planner::State;
use firefly_pubsub::odom::OdomMessage;
use nalgebra::{Point3, UnitQuaternion, Vector3};

/// 100Hz 里程计允许 500ms 墙钟中断；消息重复不能延长这个期限。
const MAX_AGE: Duration = Duration::from_millis(500);

#[derive(Clone, Copy)]
pub struct Snapshot {
    pub timestamp: f64,
    pub state: State,
    pub orientation: UnitQuaternion<f64>,
}

impl Snapshot {
    fn from_message(m: &OdomMessage) -> Option<Self> {
        let values = [
            m.timestamp,
            m.position_x,
            m.position_y,
            m.position_z,
            m.velocity_x,
            m.velocity_y,
            m.velocity_z,
            m.quat_x,
            m.quat_y,
            m.quat_z,
            m.quat_w,
        ];
        if !m.is_initialized || m.timestamp < 0.0 || !values.iter().all(|v| v.is_finite()) {
            return None;
        }
        let pose = m.body_pose(firefly_base::FrameId::ODOM).ok()?;
        Some(Self {
            timestamp: m.timestamp,
            state: State {
                position: Point3::new(m.position_x, m.position_y, m.position_z),
                velocity: Vector3::new(m.velocity_x, m.velocity_y, m.velocity_z),
                acceleration: Vector3::zeros(),
            },
            orientation: pose.isometry().rotation,
        })
    }
}

#[derive(Clone, Default)]
pub struct StateInput {
    latest: Option<Snapshot>,
    last_receive: Option<Instant>,
    stopped: bool,
    odom: Option<OdomMessage>,
}

impl StateInput {
    pub fn replace_snapshot(&mut self, snapshot: Self) {
        *self = snapshot;
    }
    pub fn stop(&mut self) {
        self.stopped = true;
    }
    pub fn odometry(&self) -> Option<OdomMessage> {
        self.odom
    }

    /// 有效状态建立后，失联或无效状态锁存停止；接收新包不能掩盖已发生的超时。
    pub fn observe(&mut self, message: &OdomMessage, now: Instant) -> bool {
        self.check_timeout(now);
        if self.stopped {
            return false;
        }
        let Some(snapshot) = Snapshot::from_message(message) else {
            if self.last_receive.is_some() && !self.stopped {
                log::warn!(
                    "收到无效局部里程计（is_initialized={} t={:.3}），锁存停止规划",
                    message.is_initialized,
                    message.timestamp
                );
            }
            self.stopped = self.last_receive.is_some();
            return false;
        };
        if self
            .latest
            .as_ref()
            .is_some_and(|last| snapshot.timestamp <= last.timestamp)
        {
            return false;
        }
        self.last_receive = Some(now);
        self.latest = Some(snapshot);
        self.odom = Some(*message);
        true
    }

    fn check_timeout(&mut self, now: Instant) {
        if self
            .last_receive
            .is_some_and(|last| now.duration_since(last) >= MAX_AGE)
        {
            if !self.stopped {
                log::warn!(
                    "局部里程计墙钟中断 ≥{}ms，锁存停止规划",
                    MAX_AGE.as_millis()
                );
            }
            self.stopped = true;
        }
    }

    pub fn current(&mut self, now: Instant) -> Option<Snapshot> {
        self.check_timeout(now);
        if self.stopped { None } else { self.latest }
    }

    pub const fn stopped(&self) -> bool {
        self.stopped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn message(timestamp: f64) -> OdomMessage {
        OdomMessage {
            timestamp,
            is_initialized: true,
            ..OdomMessage::default()
        }
    }

    #[test]
    fn waits_for_initialized_finite_state() {
        let mut input = StateInput::default();
        let now = Instant::now();
        assert!(input.current(now).is_none());
        assert!(!input.observe(&OdomMessage::default(), now));
        let mut m = message(1.0);
        m.position_x = f64::NAN;
        assert!(!input.observe(&m, now));
        m = message(1.0);
        m.quat_w = 0.0;
        assert!(!input.observe(&m, now));
        assert!(!input.stopped());
        assert!(input.observe(&message(1.0), now));
        assert!(input.current(now).is_some());
    }

    #[test]
    fn repeated_or_reordered_messages_do_not_refresh_wall_clock() {
        let mut input = StateInput::default();
        let now = Instant::now();
        assert!(input.observe(&message(10.0), now));
        assert!(!input.observe(&message(10.0), now + Duration::from_millis(400)));
        assert!(!input.observe(&message(9.0), now + Duration::from_millis(450)));
        assert!(input.current(now + MAX_AGE).is_none());
        assert!(input.stopped());
        assert!(!input.observe(&message(11.0), now + MAX_AGE));
    }

    #[test]
    fn resumed_stream_cannot_hide_gap_between_ticks() {
        let mut input = StateInput::default();
        let now = Instant::now();
        input.observe(&message(1.0), now);
        assert!(!input.observe(&message(2.0), now + Duration::from_secs(1)));
        assert!(input.current(now + Duration::from_secs(1)).is_none());
    }

    #[test]
    fn slow_planning_does_not_expire_continuously_received_state() {
        let now = Instant::now();
        let mut receiver = StateInput::default();
        receiver.observe(&message(1.), now);
        let mut planner = receiver.clone();
        for i in 1..=100 {
            assert!(receiver.observe(
                &message(1. + f64::from(i) / 100.),
                now + Duration::from_millis(i as u64 * 10),
            ));
        }
        planner.replace_snapshot(receiver.clone());
        assert_eq!(
            planner
                .current(now + Duration::from_secs(1))
                .unwrap()
                .timestamp,
            2.
        );
        receiver.current(now + Duration::from_secs(2));
        planner.replace_snapshot(receiver);
        assert!(planner.current(now + Duration::from_secs(2)).is_none());
        assert!(planner.stopped());
    }

    #[test]
    fn estimator_reset_stops_an_active_stream() {
        let mut input = StateInput::default();
        let now = Instant::now();
        input.observe(&message(1.0), now);
        assert!(!input.observe(&OdomMessage::default(), now));
        assert!(input.current(now).is_none());
        assert!(input.stopped());
    }
}
