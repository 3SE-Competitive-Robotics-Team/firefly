//! 地图系里程计输入：校验、墙钟存活检测与深度帧位姿插值。
//!
//! 对照 EGO-Planner-v2 的 `have_odom_` 门控；状态源由上游估计服务封装。
//! 不依赖观测算法或地图匹配成功频率。
//! 失联锁存须重启进程解除，轨迹参考不能充当状态观测。

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use firefly_planner::State;
use firefly_pubsub::navigation::NavigationMessage;
use firefly_pubsub::odom::OdomMessage;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

/// 100Hz 里程计允许 500ms 墙钟中断；消息重复不能延长这个期限。
const MAX_AGE: Duration = Duration::from_millis(500);
/// 深度插值历史长度（秒）；仅使用同一状态源，不外推。
const HISTORY_SECONDS: f64 = 2.0;
/// 插值两端最大间隔（秒），覆盖规划进程 10Hz 接收节拍。
const MAX_INTERPOLATION_GAP: f64 = 0.2;

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
        let pose = m.body_pose(firefly_base::FrameId::MAP).ok()?;
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

    fn pose(self) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::from(self.state.position.coords),
            self.orientation,
        )
    }
}

#[derive(Clone, Default)]
pub struct StateInput {
    history: VecDeque<Snapshot>,
    last_receive: Option<Instant>,
    stopped: bool,
    navigation: Option<NavigationMessage>,
    revised: bool,
}

impl StateInput {
    /// 接收快照保留规划线程尚未处理的坐标版本变更。
    pub fn replace_snapshot(&mut self, mut snapshot: Self) {
        snapshot.revised |= self.revised;
        *self = snapshot;
    }

    pub fn stop(&mut self) {
        self.stopped = true;
    }

    /// 版本与位姿原子接收；跨修正版本禁止深度插值。
    pub fn observe_navigation(&mut self, message: &NavigationMessage, now: Instant) -> bool {
        self.check_timeout(now);
        if message.alignment().is_err() {
            self.stopped |= self.last_receive.is_some();
            return false;
        }
        if let Some(old) = self.navigation {
            if message.odom.timestamp <= old.odom.timestamp || message.revision < old.revision {
                return false;
            }
            if message.revision == old.revision
                && (message.alignment().unwrap().matrix() - old.alignment().unwrap().matrix())
                    .norm()
                    > 1e-9
            {
                self.stopped = true;
                return false;
            }
        }
        if !self.observe(&message.odom, now) {
            return false;
        }
        if self
            .navigation
            .is_some_and(|old| old.revision != message.revision)
        {
            let latest = self.history.pop_back().unwrap();
            self.history.clear();
            self.history.push_back(latest);
            self.revised = true;
        }
        self.navigation = Some(*message);
        true
    }

    pub const fn has_revision_change(&self) -> bool {
        self.revised
    }

    pub fn take_revision_change(&mut self) -> bool {
        std::mem::take(&mut self.revised)
    }

    pub fn navigation(&self) -> Option<NavigationMessage> {
        self.navigation
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
                    "收到无效地图系里程计（is_initialized={} t={:.3}），锁存停止规划",
                    message.is_initialized,
                    message.timestamp
                );
            }
            self.stopped = self.last_receive.is_some();
            return false;
        };
        if self
            .history
            .back()
            .is_some_and(|last| snapshot.timestamp <= last.timestamp)
        {
            return false;
        }
        self.last_receive = Some(now);
        self.history.push_back(snapshot);
        while self.history.len() > 256
            || self
                .history
                .front()
                .is_some_and(|first| snapshot.timestamp - first.timestamp > HISTORY_SECONDS)
        {
            self.history.pop_front();
        }
        true
    }

    fn check_timeout(&mut self, now: Instant) {
        if self
            .last_receive
            .is_some_and(|last| now.duration_since(last) >= MAX_AGE)
        {
            if !self.stopped {
                log::warn!(
                    "地图系里程计墙钟中断 ≥{}ms，锁存停止规划",
                    MAX_AGE.as_millis()
                );
            }
            self.stopped = true;
        }
    }

    pub fn current(&mut self, now: Instant) -> Option<Snapshot> {
        self.check_timeout(now);
        if self.stopped {
            None
        } else {
            self.history.back().copied()
        }
    }

    pub const fn stopped(&self) -> bool {
        self.stopped
    }

    /// 只在有效历史的时间覆盖范围内插值；过大采样空洞与未来帧均不外推。
    pub fn pose_at(&self, timestamp: f64) -> Option<Isometry3<f64>> {
        if self.stopped || !timestamp.is_finite() {
            return None;
        }
        let first = self.history.front()?;
        if timestamp < first.timestamp || timestamp > self.history.back()?.timestamp {
            return None;
        }
        let mut previous = first;
        for next in &self.history {
            if timestamp.total_cmp(&next.timestamp).is_eq() {
                return Some(next.pose());
            }
            if timestamp < next.timestamp {
                let interval = next.timestamp - previous.timestamp;
                if interval > MAX_INTERPOLATION_GAP {
                    return None;
                }
                let weight = (timestamp - previous.timestamp) / interval;
                let position = previous
                    .state
                    .position
                    .coords
                    .lerp(&next.state.position.coords, weight);
                let orientation = previous.orientation.slerp(&next.orientation, weight);
                return Some(Isometry3::from_parts(
                    Translation3::from(position),
                    orientation,
                ));
            }
            previous = next;
        }
        None
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

    fn navigation(t: f64, revision: u64, shift: f64) -> NavigationMessage {
        let alignment = firefly_base::RigidTransform::from_parts(
            firefly_base::FrameId::MAP,
            firefly_base::FrameId::ODOM,
            [shift, 0., 0.],
            [0., 0., 0., 1.],
        )
        .unwrap();
        NavigationMessage::new(message(t), &alignment, revision)
    }

    #[test]
    fn revision_discards_interpolation_across_loop_correction() {
        let mut input = StateInput::default();
        let now = Instant::now();
        assert!(input.observe_navigation(&navigation(1., 0, 0.), now));
        assert!(input.observe_navigation(&navigation(1.1, 0, 0.), now));
        assert!(input.pose_at(1.05).is_some());
        assert!(input.observe_navigation(&navigation(1.2, 1, 2.), now));
        assert!(input.take_revision_change());
        assert!(input.pose_at(1.15).is_none());
        assert!(input.pose_at(1.2).is_some());
        assert!(!input.observe_navigation(&navigation(1.3, 0, 0.), now));
        assert!(!input.take_revision_change());
    }

    #[test]
    fn unversioned_transform_change_stops_planning() {
        let mut input = StateInput::default();
        let now = Instant::now();
        input.observe_navigation(&navigation(1., 0, 0.), now);
        assert!(!input.observe_navigation(&navigation(1.1, 0, 1.), now));
        assert!(input.stopped());
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
        receiver.observe_navigation(&navigation(1., 0, 0.), now);
        let mut planner = receiver.clone();
        for i in 1..=100 {
            assert!(receiver.observe_navigation(
                &navigation(1. + f64::from(i) / 100., 0, 0.),
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
    fn repeated_snapshots_preserve_unhandled_coordinate_revision() {
        let now = Instant::now();
        let mut receiver = StateInput::default();
        receiver.observe_navigation(&navigation(1., 0, 0.), now);
        let mut planner = receiver.clone();
        receiver.observe_navigation(&navigation(1.1, 1, 2.), now);
        planner.replace_snapshot(receiver.clone());
        receiver.take_revision_change();
        planner.replace_snapshot(receiver.clone());
        assert!(planner.take_revision_change());
        planner.replace_snapshot(receiver);
        assert!(!planner.take_revision_change());
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

    #[test]
    fn depth_pose_rejects_sampling_gaps_even_when_reception_is_fresh() {
        let mut input = StateInput::default();
        let now = Instant::now();
        input.observe(&message(1.0), now);
        input.observe(&message(1.3), now + Duration::from_millis(100));
        assert!(input.current(now + Duration::from_millis(100)).is_some());
        assert!(input.pose_at(1.15).is_none());
        assert!(input.pose_at(1.3).is_some());
    }

    #[test]
    fn depth_pose_is_interpolated_without_extrapolation() {
        let mut input = StateInput::default();
        let now = Instant::now();
        input.observe(&message(1.0), now);
        let mut end = message(1.1);
        end.position_x = 2.0;
        end.quat_z = (std::f64::consts::FRAC_PI_4).sin();
        end.quat_w = (std::f64::consts::FRAC_PI_4).cos();
        input.observe(&end, now + Duration::from_millis(100));
        let pose = input.pose_at(1.05).unwrap();
        assert!((pose.translation.x - 1.0).abs() < 1e-12);
        assert!((pose.rotation.angle() - std::f64::consts::FRAC_PI_4).abs() < 1e-12);
        assert!(input.pose_at(0.99).is_none());
        assert!(input.pose_at(1.11).is_none());
        assert!(input.pose_at(f64::NAN).is_none());
        assert!(
            input
                .current(now + MAX_AGE + Duration::from_millis(100))
                .is_none()
        );
        assert!(input.pose_at(1.05).is_none());
    }
}
