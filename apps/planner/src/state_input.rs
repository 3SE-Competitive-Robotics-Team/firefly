//! 局部里程计输入：校验、墙钟存活检测。
//!
//! 对照 EGO-Planner-v2 的 `have_odom_` 门控；状态源由上游估计服务封装。
//! 只使用真实里程计观测推进健康状态；失联锁存通过重启进程解除。

use std::time::{Duration, Instant};

use firefly_planner::State;
use firefly_pubsub::odom::OdomMessage;
use nalgebra::{Point3, UnitQuaternion, Vector3};

/// 100Hz 里程计的墙钟存活期为 500ms，由严格递增的有效测量刷新。
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
    phase: InputPhase,
}

#[derive(Clone, Default)]
enum InputPhase {
    #[default]
    Waiting,
    Active {
        snapshot: Snapshot,
        received: Instant,
        odom: OdomMessage,
    },
    Stopped {
        odom: Option<OdomMessage>,
    },
}

impl StateInput {
    pub fn replace_snapshot(&mut self, snapshot: Self) {
        *self = snapshot;
    }
    pub fn stop(&mut self) {
        self.phase = InputPhase::Stopped {
            odom: self.odometry(),
        };
    }
    pub fn odometry(&self) -> Option<OdomMessage> {
        match &self.phase {
            InputPhase::Waiting => None,
            InputPhase::Active { odom, .. } => Some(*odom),
            InputPhase::Stopped { odom } => *odom,
        }
    }

    /// 接收前先检查墙钟存活期；有效状态的失联或无效观测进入停止锁存。
    pub fn observe(&mut self, message: &OdomMessage, now: Instant) -> bool {
        self.check_timeout(now);
        if self.stopped() {
            return false;
        }
        let Some(snapshot) = Snapshot::from_message(message) else {
            if matches!(self.phase, InputPhase::Active { .. }) {
                log::warn!(
                    "收到无效局部里程计（is_initialized={} t={:.3}），锁存停止规划",
                    message.is_initialized,
                    message.timestamp
                );
                self.stop();
            }
            return false;
        };
        if matches!(&self.phase, InputPhase::Active { snapshot: last, .. } if snapshot.timestamp <= last.timestamp)
        {
            return false;
        }
        self.phase = InputPhase::Active {
            snapshot,
            received: now,
            odom: *message,
        };
        true
    }

    fn check_timeout(&mut self, now: Instant) {
        if matches!(&self.phase, InputPhase::Active { received, .. } if now.duration_since(*received) >= MAX_AGE)
        {
            log::warn!(
                "局部里程计墙钟中断 ≥{}ms，锁存停止规划",
                MAX_AGE.as_millis()
            );
            self.stop();
        }
    }

    pub fn current(&mut self, now: Instant) -> Option<Snapshot> {
        self.check_timeout(now);
        match self.phase {
            InputPhase::Active { snapshot, .. } => Some(snapshot),
            _ => None,
        }
    }

    pub const fn stopped(&self) -> bool {
        matches!(self.phase, InputPhase::Stopped { .. })
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
