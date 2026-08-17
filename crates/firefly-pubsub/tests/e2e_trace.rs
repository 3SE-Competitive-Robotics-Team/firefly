//! 端到端验证：trace 上下文跨 iceoryx2 共享内存边界续接。
//!
//! 单进程内同时创建发布端与订阅端（iceoryx2 支持同进程 pub/sub），
//! 验证：
//! - 发布端 User Header 携带当前 fastrace 活动 span 的上下文与双时间戳；
//! - 订阅端收到的上下文与发布端完全一致（`trace_id`/`span_id`/`sampled`）；
//! - 订阅端可用 `TraceContext::continue_span` 续接为子 span
//!   （fastrace 官方跨进程模式 `Span::root(name, ctx)`）。
//!
//! 需要 `firefly-observability`（dev-dep）启用 fastrace `enable` 特性，
//! 否则 `SpanContext::current_local_parent()` 恒为 `None`。

use fastrace::prelude::*;
use firefly_observability::init as init_observability;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::OdomPublisher;
use firefly_pubsub::subscriber::OdomSubscriber;

const TOPIC: &str = "Firefly/Test/E2eTrace";

#[test]
fn trace_context_crosses_pubsub_boundary() {
    init_observability();

    let publisher = OdomPublisher::with_topic(TOPIC).expect("创建发布端");
    let subscriber = OdomSubscriber::with_topic(TOPIC).expect("创建订阅端");

    // 发布端：root span 下发布，header 应携带当前活动 span 上下文
    let (trace_id, span_id) = {
        let root = Span::root("e2e-publisher", SpanContext::random());
        let _guard = root.set_local_parent();
        let active = SpanContext::current_local_parent().expect("root span 内应能取到上下文");

        let sent = publisher
            .publish(OdomMessage {
                timestamp: 1.5,
                ..OdomMessage::default()
            })
            .expect("发布");

        assert!(sent.is_traced());
        assert!(sent.sampled());
        assert_eq!(
            sent.trace_id(),
            active.trace_id.0,
            "header 的 trace_id 应为活动 span 的"
        );
        assert_eq!(
            sent.span_id, active.span_id.0,
            "header 的 span_id 应为活动 span 的"
        );
        assert!(sent.send_ts_secs > 0, "墙钟发送时间已采集");
        (active.trace_id.0, active.span_id.0)
    };

    // 订阅端：收到同一条消息，上下文一致
    let sample = subscriber
        .receive()
        .expect("接收")
        .expect("发布后应有样本可达");
    let payload: OdomMessage = *sample;
    assert!(
        (payload.timestamp - 1.5).abs() < 1e-9,
        "payload 完整穿过共享内存"
    );
    let ctx = sample.user_header();
    assert!(ctx.is_traced());
    assert_eq!(ctx.trace_id(), trace_id, "订阅端 trace_id 与发布端一致");
    assert_eq!(ctx.span_id, span_id, "订阅端 span_id 与发布端一致");
    assert!(ctx.sampled());
    assert!(ctx.send_ts_secs > 0);

    // 订阅端续接：以收到的上下文为父启动 span（跨进程 trace 续接）
    let continued = ctx.continue_span("e2e-subscriber");
    assert!(
        continued.is_some(),
        "有 trace 上下文时续接 span 必须返回 Some"
    );

    firefly_observability::flush();
}
