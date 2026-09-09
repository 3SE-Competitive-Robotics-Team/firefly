//! 结构化日志消息（iceoryx2 zero-copy）：各进程经 `Firefly/Log` 话题向
//! `firefly-viz` 聚合，统一写 rerun `TextLog`（持久可检索）——替代散落的
//! 终端 stderr 文本行。
//!
//! 设计约束：
//! - `#[repr(C)]` 定长结构，满足 `ZeroCopySend`（自包含、无堆指针、
//!   统一内存布局、`'static`、无 `Drop`），与 [`crate::viz`] 同惯例；
//! - `trace_id_hi/lo` 随消息携带（订阅端无需读 User Header 即可关联 span
//!   树，对照 [`crate::trace::TraceContext`]）；
//! - `sim_time < 0` 表示发布端尚无 sim 时钟（如 ORT 模型加载期），聚合端
//!   回落墙钟时间轴写入，仍可检索；
//! - 文本按 UTF-8 字节截断（[`LogMessage::set_text`] 保护字符边界，
//!   与 [`crate::viz::VizMessage::set_entity`] 同手法）。

use iceoryx2::prelude::*;

/// 统一日志话题（各进程发布端 → `firefly-viz` 聚合端）。
pub const LOG_TOPIC: &str = "Firefly/Log";

/// 日志级别（`log::Level` 的数值镜像，聚合端映射到 rerun `TextLogLevel`）。
pub mod level {
    /// 错误。
    pub const ERROR: u8 = 1;
    /// 警告。
    pub const WARN: u8 = 2;
    /// 信息。
    pub const INFO: u8 = 3;
    /// 调试。
    pub const DEBUG: u8 = 4;
    /// 追踪。
    pub const TRACE: u8 = 5;
}

/// 进程标签字节上限（64，如 `vio`/`planner`/`gicp`/`aliked`/`lightglue`）。
pub const TAG_MAX: usize = 64;
/// 日志文本字节上限（512，超长截断）。
pub const TEXT_MAX: usize = 512;

/// 结构化日志消息（扁平定长：级别 + 进程标签 + 文本 + 双时间戳 + trace 链）。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyLogMessage")]
pub struct LogMessage {
    /// 级别（[`level`] 常量）。
    pub log_level: u8,
    /// 进程标签有效字节数。
    pub tag_len: u8,
    /// 保留字段（对齐），必须为零。
    pub reserved: [u8; 2],
    /// 文本有效字节数。
    pub text_len: u32,
    /// 进程标签（UTF-8，`[u8; 64]` 定长 + `len`）。
    pub tag: [u8; TAG_MAX],
    /// 日志文本（UTF-8，`[u8; 512]` 定长 + `len`）。
    pub text: [u8; TEXT_MAX],
    /// 仿真时钟时间戳（秒，`sim_time` 时间轴；`< 0` 表示尚无 sim 时钟，
    /// 聚合端回落墙钟时间轴）。
    pub sim_time: f64,
    /// 墙钟发送时间（unix 秒）。
    pub wall_secs: i64,
    /// 墙钟发送时间（亚秒纳秒）。
    pub wall_nanos: u32,
    /// trace-id 高 64 位（W3C 128-bit 拆分；0 = 无 span 上下文）。
    pub trace_id_hi: u64,
    /// trace-id 低 64 位。
    pub trace_id_lo: u64,
}

impl Default for LogMessage {
    fn default() -> Self {
        Self {
            log_level: level::INFO,
            tag_len: 0,
            reserved: [0; 2],
            text_len: 0,
            tag: [0u8; TAG_MAX],
            text: [0u8; TEXT_MAX],
            sim_time: -1.0,
            wall_secs: 0,
            wall_nanos: 0,
            trace_id_hi: 0,
            trace_id_lo: 0,
        }
    }
}

impl LogMessage {
    /// 设置进程标签（UTF-8，超长截断；内部以字符边界保护不切碎多字节序列）。
    pub fn set_tag(&mut self, tag: &str) {
        let bytes = tag.as_bytes();
        let mut n = bytes.len().min(TAG_MAX);
        while n > 0 && (bytes[n - 1] & 0xC0) == 0x80 {
            n -= 1;
        }
        if n > 0 && (bytes[n - 1] & 0xC0) == 0xC0 {
            n -= 1;
        }
        self.tag[..n].copy_from_slice(&bytes[..n]);
        self.tag_len = n as u8;
    }

    /// 设置日志文本（UTF-8，超长截断；内部以字符边界保护不切碎多字节序列）。
    pub fn set_text(&mut self, text: &str) {
        let bytes = text.as_bytes();
        let mut n = bytes.len().min(TEXT_MAX);
        while n > 0 && (bytes[n - 1] & 0xC0) == 0x80 {
            n -= 1;
        }
        if n > 0 && (bytes[n - 1] & 0xC0) == 0xC0 {
            n -= 1;
        }
        self.text[..n].copy_from_slice(&bytes[..n]);
        self.text_len = n as u32;
    }

    /// 进程标签（按 `tag_len` 截断，非法 UTF-8 以替换符呈现）。
    #[must_use]
    pub fn tag_str(&self) -> &str {
        let n = (self.tag_len as usize).min(TAG_MAX);
        std::str::from_utf8(&self.tag[..n]).unwrap_or("\u{FFFD}")
    }

    /// 日志文本（按 `text_len` 截断，非法 UTF-8 以替换符呈现）。
    #[must_use]
    pub fn text_str(&self) -> &str {
        let n = (self.text_len as usize).min(TEXT_MAX);
        std::str::from_utf8(&self.text[..n]).unwrap_or("\u{FFFD}")
    }

    /// 是否携带有效 trace 上下文。
    #[must_use]
    pub fn is_traced(&self) -> bool {
        self.trace_id_hi != 0 || self.trace_id_lo != 0
    }
}

/// 同话题发布端上限（7 进程 + 工具链余量；服务首次创建时生效，
/// 见 [`crate::publish::Publisher::with_topic_buffer_publishers`]。
/// 注：`max_publishers` 只在服务创建时生效——`firefly-viz` 聚合端先启动
/// 预创建（`_precreate_log_service`，上限 10），Rust 发布端只 `open`
/// 不创建；顺序反了（发布端先建，默认上限 2）即
/// `DoesNotSupportRequestedAmountOfPublishers` 降级纯 stderr，
/// 见 `firefly-viz/main.py` 的 runbook 注释）。
pub const LOG_MAX_PUBLISHERS: usize = 10;

/// 日志发布器（话题 [`LOG_TOPIC`]，泛型核心的命名封装）。
pub struct LogPublisher(pub(crate) crate::publish::Publisher<LogMessage>);

impl LogPublisher {
    /// 打开统一日志话题的发布器（`max_publishers` = [`LOG_MAX_PUBLISHERS`]，
    /// 7 进程同话题发布；先启动的进程创建服务，后续 open 必须 ≤ 该值）。
    ///
    /// # Errors
    /// 见 [`crate::publish::Publisher::with_topic`]。
    pub fn new(node: &crate::node::IpcNode) -> Result<Self, firefly_error::Error> {
        Ok(Self(
            crate::publish::Publisher::with_topic_buffer_publishers(
                node,
                LOG_TOPIC,
                None,
                Some(LOG_MAX_PUBLISHERS),
            )?,
        ))
    }

    /// 发布一条日志消息（trace 上下文由调用方随消息携带，见
    /// [`firefly_observability::IpcAppend`]）。
    ///
    /// # Errors
    /// 见 [`crate::publish::Publisher::publish`]。
    pub fn publish(
        &self,
        msg: LogMessage,
    ) -> Result<crate::trace::TraceContext, firefly_error::Error> {
        self.0.publish(msg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 编译期断言：满足 iceoryx2 零拷贝约束。
    fn assert_zero_copy_send<T: ZeroCopySend>() {}

    #[test]
    fn log_message_is_plain_old_data() {
        assert_zero_copy_send::<LogMessage>();
        // 定长布局（8 字节对齐）：1(level)+1(tag_len)+2(reserved)+4(text_len)+
        // 64(tag)+512(text)+8(sim)+8(wall_secs)+4(wall_nanos)+4(pad)+8+8(trace) = 624
        assert_eq!(std::mem::size_of::<LogMessage>(), 624);
        let m = LogMessage::default();
        assert_eq!(m.log_level, level::INFO);
        assert!(m.sim_time < 0.0);
        assert!(!m.is_traced());
    }

    #[test]
    fn tag_text_round_trip() {
        let mut m = LogMessage::default();
        m.set_tag("vio");
        m.set_text("VIO ready");
        assert_eq!(m.tag_str(), "vio");
        assert_eq!(m.text_str(), "VIO ready");
        // 多字节截断不断裂：TEXT_MAX-1 个 ASCII + 1 个 3 字节字符 → 退到 ASCII 前缀
        m.set_text(&format!("{}{}", "a".repeat(TEXT_MAX - 1), "绪"));
        assert!(m.text_str().is_ascii());
    }

    #[test]
    fn text_truncates_at_char_boundary() {
        let mut m = LogMessage::default();
        // 511 个 ASCII + 1 个 3 字节字符 = 514 字节：退过续字节后停在多字节
        // 首字节再退一位 → 纯 ASCII 前缀（与 VizMessage::set_entity 同语义）
        let s = format!("{}{}", "a".repeat(TEXT_MAX - 1), "中");
        m.set_text(&s);
        assert!((m.text_len as usize) <= TEXT_MAX);
        assert!(m.text_str().is_ascii());
        assert_eq!(m.text_str(), "a".repeat(TEXT_MAX - 1));
    }
}
