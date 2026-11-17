//! 有时限的按需资源请求；接收端以本地单调时钟过期，SIL/HIL 使用同一契约。
use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use crate::{
    node::IpcNode,
    subscriber::{Subscriber, SubscriberOptions},
};
use iceoryx2::prelude::*;

pub const ACTIVITY_TOPIC: &str = "Firefly/Activity";
pub const DOWN_CAMERA: u32 = 1;
pub const LANDING: u32 = 2;
pub const MAX_OWNERS: usize = 8;

/// 每个任务拥有独立的 `owner` 并周期续租；`mask=0` 或 `lease_ms=0` 释放自己的请求。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, ZeroCopySend)]
#[type_name("FireflyActivityMessage")]
pub struct ActivityMessage {
    /// 测量时间域中的阶段时刻，仅用于诊断。
    pub timestamp: f64,
    pub owner: u64,
    pub sequence: u64,
    pub mask: u32,
    /// 激活下视时只允许 15 或 30 FPS；`LANDING` 同时请求 `DOWN_CAMERA`。
    pub rate_hz: u32,
    /// 0 释放；1..=2000 毫秒的本地到达时间租期。
    pub lease_ms: u32,
}

impl ActivityMessage {
    #[must_use]
    pub fn valid(&self) -> bool {
        self.timestamp.is_finite()
            && self.owner != 0
            && self.sequence != 0
            && self.mask & !(DOWN_CAMERA | LANDING) == 0
            && self.lease_ms <= 2000
            && (self.mask == 0 || self.lease_ms == 0 || matches!(self.rate_hz, 15 | 30))
    }
}

/// 采集时间域的节拍；容纳 1ms 测量时钟量化，保留相位并跳过停顿后的积压拍。
#[derive(Default)]
pub struct FrameCadence {
    last: Option<f64>,
    next: f64,
    rate: u32,
}

impl FrameCadence {
    pub fn admit(&mut self, timestamp: f64, rate: u32) -> bool {
        if rate == 0 || !timestamp.is_finite() {
            return false;
        }
        let period = 1. / f64::from(rate);
        if self.rate != rate {
            self.next = self.last.map_or(timestamp, |last| last + period);
            self.rate = rate;
        }
        let quantum = 0.001 + 1e-9;
        if timestamp + quantum < self.next
            || self
                .last
                .is_some_and(|last| timestamp <= last || timestamp - last + quantum < period)
        {
            return false;
        }
        self.last = Some(timestamp);
        self.next += period;
        if self.next <= timestamp {
            self.next = timestamp + period;
        }
        true
    }
}

struct Request {
    message: ActivityMessage,
    expires: Instant,
}

/// 互不撤销的有界任务租约；重复投递不续命，过期和释放均进入 Suspended。
#[derive(Default)]
pub struct Activity {
    requests: BTreeMap<u64, Request>,
}

impl Activity {
    /// 接受新序号；无效请求、重复请求或拥有者预算耗尽返回 false。
    pub fn observe(&mut self, message: ActivityMessage, now: Instant) -> bool {
        if !message.valid()
            || self
                .requests
                .get(&message.owner)
                .is_some_and(|old| message.sequence <= old.message.sequence)
        {
            return false;
        }
        if !self.requests.contains_key(&message.owner) && self.requests.len() >= MAX_OWNERS {
            self.requests.retain(|_, request| request.expires > now);
            if self.requests.len() >= MAX_OWNERS {
                return false;
            }
        }
        self.requests.insert(
            message.owner,
            Request {
                message,
                expires: now + Duration::from_millis(u64::from(message.lease_ms)),
            },
        );
        true
    }

    #[must_use]
    pub fn rate(&self, resource: u32, now: Instant) -> Option<u32> {
        self.requests
            .values()
            .filter(|request| {
                let mask = request.message.mask;
                request.expires > now
                    && (mask & resource != 0 || resource == DOWN_CAMERA && mask & LANDING != 0)
            })
            .map(|request| request.message.rate_hz)
            .max()
    }
}

/// 常驻的轻量控制订阅端；工作资源由调用者的顶层状态机持有。
pub struct ActivityReceiver {
    subscriber: Subscriber<ActivityMessage>,
    activity: Activity,
}

impl ActivityReceiver {
    /// # Errors
    /// IPC 服务或订阅端创建失败。
    pub fn open(node: &IpcNode, topic: &str) -> firefly_error::Result<Self> {
        Ok(Self {
            subscriber: Subscriber::with_options(
                node,
                topic,
                SubscriberOptions {
                    buffer_size: 8,
                    service_max_buffer_size: Some(8),
                    max_publishers: Some(MAX_OWNERS),
                },
            )?,
            activity: Activity::default(),
        })
    }

    /// 排空控制流后返回当前资源需求；过期不依赖新消息。
    /// # Errors
    /// IPC 接收失败。
    pub fn poll(&mut self, resource: u32) -> firefly_error::Result<Option<u32>> {
        let now = Instant::now();
        while let Some(message) = self.subscriber.receive()? {
            self.activity.observe(*message, now);
        }
        Ok(self.activity.rate(resource, now))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn request(owner: u64, sequence: u64, mask: u32) -> ActivityMessage {
        ActivityMessage {
            owner,
            sequence,
            mask,
            rate_hz: 15,
            lease_ms: 1000,
            ..Default::default()
        }
    }
    #[test]
    fn lease_expiry_duplicates_and_independent_owners() {
        let now = Instant::now();
        let mut activity = Activity::default();
        assert_eq!(activity.rate(DOWN_CAMERA, now), None);
        assert!(activity.observe(request(1, 1, LANDING), now));
        assert_eq!(activity.rate(DOWN_CAMERA, now), Some(15));
        assert!(!activity.observe(request(1, 1, LANDING), now + Duration::from_millis(800)));
        assert_eq!(activity.rate(LANDING, now + Duration::from_secs(1)), None);
        assert!(activity.observe(request(1, 2, LANDING), now));
        let mut second = request(2, 1, DOWN_CAMERA);
        second.rate_hz = 30;
        assert!(activity.observe(second, now));
        assert!(activity.observe(request(1, 3, 0), now));
        assert_eq!(activity.rate(LANDING, now), None);
        assert_eq!(activity.rate(DOWN_CAMERA, now), Some(30));
        assert!(!activity.observe(request(1, 2, LANDING), now));
    }
    #[test]
    fn invalid_rates_masks_and_owner_budget_are_bounded() {
        let now = Instant::now();
        let mut activity = Activity::default();
        for rate in [0, 10, 60] {
            let mut message = request(1, 1, LANDING);
            message.rate_hz = rate;
            assert!(!activity.observe(message, now));
        }
        assert!(!activity.observe(request(1, 1, 4), now));
        for owner in 1..=MAX_OWNERS as u64 {
            assert!(activity.observe(request(owner, 1, LANDING), now));
        }
        assert!(!activity.observe(request(99, 1, LANDING), now));
        assert!(activity.observe(request(99, 1, LANDING), now + Duration::from_secs(2)));
    }
    #[test]
    fn cadence_caps_30_hz_source_at_15_without_catchup() {
        let mut cadence = FrameCadence::default();
        assert_eq!(
            (0..300)
                .filter(|i| cadence.admit(f64::from(*i) / 30., 15))
                .count(),
            150
        );
        assert!(!cadence.admit(0., 15));
        assert!(cadence.admit(100., 15));
        assert!(!cadence.admit(100., 15));
    }

    #[test]
    fn millisecond_capture_clock_preserves_15_and_30_fps() {
        for rate in [15, 30] {
            let mut cadence = FrameCadence::default();
            let count = (0..600)
                .filter(|i| {
                    let timestamp = (f64::from(*i) / 30. * 1000.).round() / 1000.;
                    cadence.admit(timestamp, rate)
                })
                .count();
            assert_eq!(count, rate as usize * 20);
        }
        let mut cadence = FrameCadence::default();
        let count = (0..6000)
            .filter(|i| cadence.admit(f64::from(*i) / 300., 15))
            .count();
        assert!(count <= 300, "fast sources obey the same long-term rate");
    }
}
