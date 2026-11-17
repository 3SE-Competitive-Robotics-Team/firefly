use iceoryx2::prelude::*;

pub const LOG_TOPIC: &str = "Firefly/Log";
/// 突发事件与 trace 诊断的订阅队列上限；mimir 使用同值。
pub const LOG_BUFFER_SIZE: usize = 256;

/// 日志级别（`log::Level` 的数值镜像，聚合端映射到 rerun `TextLogLevel`）。
pub mod level {
    pub const ERROR: u8 = 1;
    pub const WARN: u8 = 2;
    pub const INFO: u8 = 3;
    pub const DEBUG: u8 = 4;
    pub const TRACE: u8 = 5;
}

pub const TAG_MAX: usize = 64;
pub const TEXT_MAX: usize = 512;

#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyLogMessage")]
pub struct LogMessage {
    pub log_level: u8,
    pub tag_len: u8,
    /// 保留字段（对齐），必须为零。
    pub reserved: [u8; 2],
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
    pub trace_id_lo: u64,
    /// Per-publisher send sequence, including failed sends.
    pub sequence: u64,
    /// Cumulative records lost before IPC, including queue and publish failures.
    pub dropped_records: u64,
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
            sequence: 0,
            dropped_records: 0,
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

    #[must_use]
    pub fn is_traced(&self) -> bool {
        self.trace_id_hi != 0 || self.trace_id_lo != 0
    }
}

/// 同话题发布端上限（7 进程 + 工具链余量），服务首次创建时生效，
/// 见 [`crate::publish::Publisher::with_topic_buffer_publishers`]。
/// 发布端与聚合端都使用相同容量；先启动聚合端才能记录启动阶段日志。
pub const LOG_MAX_PUBLISHERS: usize = 10;

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_zero_copy_send<T: ZeroCopySend>() {}

    #[test]
    fn log_message_is_plain_old_data() {
        assert_zero_copy_send::<LogMessage>();
        assert_eq!(std::mem::size_of::<LogMessage>(), 640);
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
    }

    #[test]
    fn text_truncates_at_char_boundary() {
        let mut m = LogMessage::default();
        let s = format!("{}{}", "a".repeat(TEXT_MAX - 1), "中");
        m.set_text(&s);
        assert!((m.text_len as usize) <= TEXT_MAX);
        assert!(m.text_str().is_ascii());
        assert_eq!(m.text_str(), "a".repeat(TEXT_MAX - 1));
    }
}
