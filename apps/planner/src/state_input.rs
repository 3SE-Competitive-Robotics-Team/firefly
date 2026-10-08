//! 地图系里程计输入：校验、墙钟存活检测与深度帧位姿插值。
//!
//! 对照 EGO-Planner-v2 的 `have_odom_` 门控和失联后禁用自动恢复的约束。
//! 失联锁存须重启进程解除，轨迹参考不能充当状态观测。

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use firefly_planner::State;
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

#[derive(Default)]
pub struct StateInput {
    history: VecDeque<Snapshot>,
    last_receive: Option<Instant>,
    stopped: bool,
}

impl StateInput {
    /// 有效状态建立后，失联或无效状态锁存停止；接收新包不能掩盖已发生的超时。
    pub fn observe(&mut self, message: &OdomMessage, now: Instant) -> bool {
        self.check_timeout(now);
        if self.stopped {
            return false;
        }
        let Some(snapshot) = Snapshot::from_message(message) else {
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

/// 视觉地图质量门控；获得可用状态后，退化或中断锁存停止。
#[derive(Default)]
pub struct QualityInput {
    latest: Option<(firefly_pubsub::odom::LocalizationStatus, Instant)>,
    ready_once: bool,
    stopped: bool,
    /// 累计收到的状态条数（区分「没收到」与「收到但未就绪」）。
    observed: usize,
}
impl QualityInput {
    pub fn observe(&mut self, status: firefly_pubsub::odom::LocalizationStatus, now: Instant) {
        if !status.timestamp.is_finite() || status.timestamp < 0. {
            return;
        }
        if self
            .latest
            .is_some_and(|(previous, _)| status.timestamp <= previous.timestamp)
        {
            return;
        }
        if self.latest.is_none() {
            log::info!(
                "首个 LocalizationStatus：tracking_ready={} visual_t={:.3} disagreement={:.3}",
                status.tracking_ready,
                status.visual_timestamp,
                status.position_disagreement
            );
        }
        self.observed += 1;
        self.stopped |= self.ready_once
            && (!status.tracking_ready
                || self
                    .latest
                    .is_some_and(|(_, received)| now.duration_since(received) >= MAX_AGE));
        self.ready_once |= status.tracking_ready;
        self.latest = Some((status, now));
    }
    pub fn ready(&mut self, timestamp: f64, now: Instant) -> bool {
        let current = self.latest.is_some_and(|(s, received)| {
            s.tracking_ready
                && now.duration_since(received) < MAX_AGE
                && (timestamp - s.timestamp).abs() <= 0.2
        });
        self.stopped |= self.ready_once && !current;
        current && !self.stopped
    }
    pub fn stopped(&self) -> bool {
        self.stopped
    }
    /// 是否收到过任何状态（诊断用）。
    #[must_use]
    pub fn has_received(&self) -> bool {
        self.observed > 0
    }
    /// 最新状态的原始 `tracking_ready` 标志（诊断用）。
    #[must_use]
    pub fn latest_tracking_ready(&self) -> bool {
        self.latest.is_some_and(|(s, _)| s.tracking_ready)
    }
}

#[cfg(test)]
mod quality_tests {
    use super::*;
    use firefly_pubsub::odom::LocalizationStatus;
    #[test]
    fn degradation_latches_and_duplicates_do_not_refresh() {
        let now = Instant::now();
        let status = LocalizationStatus {
            timestamp: 1.,
            visual_timestamp: 1.,
            position_disagreement: 0.1,
            tracking_ready: true,
        };
        let mut gate = QualityInput::default();
        assert!(!gate.ready(1., now));
        gate.observe(status, now);
        assert!(gate.ready(1., now));
        gate.observe(status, now + Duration::from_millis(400));
        assert!(!gate.ready(1., now + Duration::from_millis(501)));
        gate.observe(
            LocalizationStatus {
                timestamp: 2.,
                ..status
            },
            now + Duration::from_millis(502),
        );
        assert!(!gate.ready(2., now + Duration::from_millis(502)));
        let mut gate = QualityInput::default();
        gate.observe(status, now);
        gate.observe(
            LocalizationStatus {
                timestamp: 1.1,
                tracking_ready: false,
                ..status
            },
            now,
        );
        assert!(gate.stopped());
    }
}
