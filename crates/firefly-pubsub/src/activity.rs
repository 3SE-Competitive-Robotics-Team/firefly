//! 有时限的推理请求；接收端以本地单调时钟过期，SIL/HIL 使用同一契约。
use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use crate::{
    node::IpcNode,
    subscriber::{Subscriber, SubscriberOptions},
};
use iceoryx2::prelude::*;

pub const ACTIVITY_TOPIC: &str = "Firefly/Activity";
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
    /// 0 释放；1..=2000 毫秒的本地到达时间租期。
    pub lease_ms: u32,
}

impl ActivityMessage {
    #[must_use]
    pub fn valid(&self) -> bool {
        self.timestamp.is_finite()
            && self.owner != 0
            && self.sequence != 0
            && self.mask & !LANDING == 0
            && self.lease_ms <= 2000
    }
}

struct Request {
    message: ActivityMessage,
    expires: Instant,
}

/// 互不撤销的有界推理任务租约；重复投递不续命，过期和释放撤销推理需求。
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
    pub fn requested(&self, resource: u32, now: Instant) -> bool {
        self.requests
            .values()
            .any(|request| request.expires > now && request.message.mask & resource != 0)
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

    /// 排空控制流后返回当前推理需求；过期不依赖新消息。
    /// # Errors
    /// IPC 接收失败。
    pub fn poll(&mut self, resource: u32) -> firefly_error::Result<bool> {
        let now = Instant::now();
        while let Some(message) = self.subscriber.receive()? {
            self.activity.observe(*message, now);
        }
        Ok(self.activity.requested(resource, now))
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
            lease_ms: 1000,
            ..Default::default()
        }
    }
    #[test]
    fn lease_expiry_duplicates_and_independent_owners() {
        let now = Instant::now();
        let mut activity = Activity::default();
        assert!(!activity.requested(LANDING, now));
        assert!(activity.observe(request(1, 1, LANDING), now));
        assert!(activity.requested(LANDING, now));
        assert!(!activity.observe(request(1, 1, LANDING), now + Duration::from_millis(800)));
        assert!(!activity.requested(LANDING, now + Duration::from_secs(1)));
        assert!(activity.observe(request(1, 2, LANDING), now));
        assert!(activity.observe(request(2, 1, LANDING), now));
        assert!(activity.observe(request(1, 3, 0), now));
        assert!(activity.requested(LANDING, now));
        assert!(activity.observe(request(2, 2, 0), now));
        assert!(!activity.requested(LANDING, now));
        assert!(!activity.observe(request(1, 2, LANDING), now));
    }
    #[test]
    fn invalid_masks_and_owner_budget_are_bounded() {
        let now = Instant::now();
        let mut activity = Activity::default();
        assert!(!activity.observe(request(1, 1, 1), now));
        for owner in 1..=MAX_OWNERS as u64 {
            assert!(activity.observe(request(owner, 1, LANDING), now));
        }
        assert!(!activity.observe(request(99, 1, LANDING), now));
        assert!(activity.observe(request(99, 1, LANDING), now + Duration::from_secs(2)));
    }
}
