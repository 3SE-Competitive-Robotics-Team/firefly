//! IPC 用户头携带 W3C `(trace_id, span_id, sampled)`，订阅端以发送 span 为父续接。
//! 墙钟用于可读时间；单调时钟用于同机端到端延迟，不受墙钟跳变影响。
//! 两端须以相同的 [`TraceContext`] POD 用户头创建服务。
//! 规范：[W3C Trace Context](https://www.w3.org/TR/trace-context/)。

use std::borrow::Cow;

use fastrace::prelude::*;
use iceoryx2::prelude::*;
use iceoryx2_bb_posix::clock::{ClockType, Time};

/// [`Default`] 产生全零占位（`version == 0`），不携带有效发送时间。
/// [`Self::timestamps_only`] 采集发送时间并填写当前版本，但不携带 span。
/// `version == 1` 表示结构已填充；`trace_id == 0` 表示发布端不在任何 fastrace
/// span 内（此时仅时间戳有效，[`TraceContext::is_traced`] 为 `false`）。
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, ZeroCopySend)]
#[type_name("FireflyTraceContext")]
pub struct TraceContext {
    pub version: u8,
    /// 标志位：bit0 = sampled（W3C Trace Context）。
    pub flags: u8,
    /// 保留字段（未来扩展），必须为零。
    pub reserved: [u8; 2],
    /// trace-id 高 64 位（W3C 128-bit 拆分，避免 16 字节对齐）。
    pub trace_id_hi: u64,
    pub trace_id_lo: u64,
    /// 当前 span-id（订阅端以其为 parent 续接）。
    pub span_id: u64,
    /// 墙钟发送时间（unix 秒，与 jiff [`Timestamp`](jiff::Timestamp) 互转）。
    pub send_ts_secs: i64,
    /// 墙钟发送时间（亚秒纳秒）。
    pub send_ts_nanos: u32,
    /// `CLOCK_MONOTONIC` 发送时间（自启动纳秒，跨进程可比）。
    pub send_ts_mono_ns: u64,
}

impl TraceContext {
    pub const VERSION: u8 = 1;
    pub const FLAG_SAMPLED: u8 = 0b0000_0001;

    /// 从 fastrace 活动 span 上下文构建（发布端注入），同时采集双时间戳。
    #[must_use]
    pub fn from_span_context(sc: SpanContext) -> Self {
        Self::capture(
            (sc.trace_id.0 >> 64) as u64,
            sc.trace_id.0 as u64,
            sc.span_id.0,
            if sc.sampled { Self::FLAG_SAMPLED } else { 0 },
        )
    }

    /// 仅采集时间戳（发布端不在任何 span 内时使用，无 trace 上下文）。
    #[must_use]
    pub fn timestamps_only() -> Self {
        Self::capture(0, 0, 0, 0)
    }

    #[must_use]
    pub fn is_traced(&self) -> bool {
        self.version == Self::VERSION && self.trace_id() != 0
    }

    #[must_use]
    pub fn trace_id(&self) -> u128 {
        (u128::from(self.trace_id_hi) << 64) | u128::from(self.trace_id_lo)
    }

    #[must_use]
    pub fn sampled(&self) -> bool {
        self.flags & Self::FLAG_SAMPLED != 0
    }

    /// 转回 fastrace `SpanContext`（订阅端续接：作为新 span 的父上下文）。
    #[must_use]
    pub fn span_context(&self) -> Option<SpanContext> {
        self.is_traced().then(|| {
            SpanContext::new(TraceId(self.trace_id()), SpanId(self.span_id)).sampled(self.sampled())
        })
    }

    /// 以本上下文为父，在本进程内启动入口 span。
    ///
    /// 返回 `None` 表示无 trace 上下文（[`TraceContext::is_traced`] 为 `false`）；
    /// 未启用 fastrace `enable` 时返回 noop span（不采集）。
    #[must_use]
    pub fn continue_span(&self, name: impl Into<Cow<'static, str>>) -> Option<Span> {
        self.span_context().map(|sc| Span::root(name, sc))
    }

    /// 墙钟发送时间（jiff [`Timestamp`](jiff::Timestamp)，UTC）。
    #[must_use]
    pub fn send_timestamp(&self) -> jiff::Timestamp {
        let nanos = i128::from(self.send_ts_secs) * 1_000_000_000 + i128::from(self.send_ts_nanos);
        jiff::Timestamp::from_nanosecond(nanos).unwrap_or(jiff::Timestamp::UNIX_EPOCH)
    }

    /// 单调时间不可用时填零。
    fn capture(trace_id_hi: u64, trace_id_lo: u64, span_id: u64, flags: u8) -> Self {
        let real = jiff::Timestamp::now();
        let mono_ns = Time::now_with_clock(ClockType::Monotonic)
            .map_or(0, |t| t.as_duration().as_nanos() as u64);
        Self {
            version: Self::VERSION,
            flags,
            reserved: [0; 2],
            trace_id_hi,
            trace_id_lo,
            span_id,
            send_ts_secs: real.as_second(),
            send_ts_nanos: real.subsec_nanosecond() as u32,
            send_ts_mono_ns: mono_ns,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_zero_copy_send<T: ZeroCopySend>() {}
    #[test]
    fn trace_context_is_plain_old_data() {
        assert_zero_copy_send::<TraceContext>();
        assert_eq!(std::mem::size_of::<TraceContext>(), 56);
        let e = TraceContext::default();
        assert_eq!(e.version, 0);
        assert_eq!(e.trace_id(), 0);
        assert_eq!(e.send_ts_secs, 0);
        assert_eq!(e.send_ts_mono_ns, 0);
        assert!(e.span_context().is_none());
    }

    #[test]
    fn span_context_round_trips() {
        let sc = SpanContext::new(
            TraceId(0x1122_3344_5566_7788_aabb_ccdd_eeff_0011),
            SpanId(0x0102_0304_0506_0708),
        )
        .sampled(true);
        let ctx = TraceContext::from_span_context(sc);
        assert!(ctx.is_traced());
        assert!(ctx.sampled());
        assert_eq!(ctx.trace_id(), sc.trace_id.0);
        let back = ctx
            .span_context()
            .expect("traced context must convert back");
        assert_eq!(back.trace_id, sc.trace_id);
        assert_eq!(back.span_id, sc.span_id);
        assert_eq!(back.sampled, sc.sampled);
        assert!(ctx.send_ts_secs > 0);
        if Time::now_with_clock(ClockType::Monotonic).is_ok() {
            assert!(ctx.send_ts_mono_ns > 0);
        } else {
            assert_eq!(ctx.send_ts_mono_ns, 0);
        }
    }

    #[test]
    fn unsampled_flag_preserved() {
        let sc = SpanContext::new(TraceId(1), SpanId(2)).sampled(false);
        let ctx = TraceContext::from_span_context(sc);
        assert!(ctx.is_traced());
        assert!(!ctx.sampled());
        assert!(!ctx.span_context().unwrap().sampled);
    }

    #[test]
    fn timestamps_only_has_no_trace_context() {
        let ctx = TraceContext::timestamps_only();
        assert_eq!(ctx.version, TraceContext::VERSION);
        assert!(!ctx.is_traced());
        assert!(ctx.span_context().is_none());
        assert!(ctx.send_ts_secs > 0);
    }

    #[test]
    fn send_timestamp_converts_to_jiff() {
        let ctx = TraceContext::timestamps_only();
        let ts = ctx.send_timestamp();
        assert_eq!(ts.as_second(), ctx.send_ts_secs);
        assert_eq!(
            i64::from(ts.subsec_nanosecond()),
            i64::from(ctx.send_ts_nanos)
        );
    }
}
