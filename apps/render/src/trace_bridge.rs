//! tracing → log 转发：`Bevy` 内部日志走 `tracing`，`LogPlugin` 禁用后
//! 不再安装 subscriber，所有 `Bevy` 侧日志会被静默丢弃（窗口/渲染器/
//! 退出原因全不可见）。本转发器是唯一的全局 tracing dispatcher，把事件
//! 原样（级别/目标/调用点/字段）送进 `log` 门面，统一由 `logforth` 输出。
//!
//! 注意方向别反：`tracing-log` 的 `LogTracer` 是 log→tracing（与
//! `logforth` 抢占全局 logger，不能用），这里需要的是 tracing→log。

use std::sync::atomic::{AtomicU64, Ordering};

use bevy::log::tracing::{
    Event, Id, Level, Metadata, Subscriber,
    dispatcher::{Dispatch, set_global_default},
    field::{Field, Visit},
    span::{Attributes, Record},
};

/// 事件字段收集（`message` 作正文，其余 `k=v` 附加）。
#[derive(Default)]
struct Fields {
    /// 正文。
    message: Option<String>,
    /// 附加字段。
    rest: Vec<String>,
}

impl Fields {
    /// 格式化正文（无 `message` 字段时以字段列表充当）。
    fn format(self) -> String {
        match self.message {
            Some(message) if self.rest.is_empty() => message,
            Some(message) => format!("{message} {}", self.rest.join(" ")),
            None => self.rest.join(" "),
        }
    }
}

/// tracing 事件访问器（标量直写，其余 `Debug` 兜底）。
struct FieldVisitor {
    /// 收集器。
    fields: Fields,
}

impl Visit for FieldVisitor {
    fn record_str(&mut self, field: &Field, value: &str) {
        if field.name() == "message" {
            self.fields.message = Some(value.to_owned());
        } else {
            self.fields.rest.push(format!("{}={value}", field.name()));
        }
    }

    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        self.fields.rest.push(format!("{}={value:?}", field.name()));
    }

    fn record_i64(&mut self, field: &Field, value: i64) {
        self.fields.rest.push(format!("{}={value}", field.name()));
    }

    fn record_u64(&mut self, field: &Field, value: u64) {
        self.fields.rest.push(format!("{}={value}", field.name()));
    }

    fn record_bool(&mut self, field: &Field, value: bool) {
        self.fields.rest.push(format!("{}={value}", field.name()));
    }

    fn record_f64(&mut self, field: &Field, value: f64) {
        self.fields.rest.push(format!("{}={value}", field.name()));
    }
}

/// 转发 subscriber（span 只分配 id 不建树：`log` 门面无 span 概念，
/// 崩溃诊断只需要事件本身；`DEBUG` 以上才求值，`TRACE` 永不启用）。
struct ForwardToLog {
    /// span id 分配器。
    next_id: AtomicU64,
}

impl Subscriber for ForwardToLog {
    fn enabled(&self, metadata: &Metadata<'_>) -> bool {
        *metadata.level() <= Level::DEBUG
    }

    fn new_span(&self, _span: &Attributes<'_>) -> Id {
        Id::from_u64(self.next_id.fetch_add(1, Ordering::Relaxed).max(1))
    }

    fn record(&self, _span: &Id, _values: &Record<'_>) {}

    fn record_follows_from(&self, _span: &Id, _follows: &Id) {}

    fn event(&self, event: &Event<'_>) {
        let metadata = event.metadata();
        let level = match *metadata.level() {
            Level::TRACE => log::Level::Trace,
            Level::DEBUG => log::Level::Debug,
            Level::INFO => log::Level::Info,
            Level::WARN => log::Level::Warn,
            Level::ERROR => log::Level::Error,
        };
        // 官方桥（`logforth-bridge-log`）同款纪律：先问门面过不过滤，
        // 不过直接丢，避免字段格式化的无用开销。
        if !log::logger().enabled(
            &log::Metadata::builder()
                .level(level)
                .target(metadata.target())
                .build(),
        ) {
            return;
        }
        let mut visitor = FieldVisitor {
            fields: Fields::default(),
        };
        event.record(&mut visitor);
        let message = visitor.fields.format();
        // `format_args!` 临时值活不过分号：build 与 log 必须同语句完成。
        log::logger().log(
            &log::Record::builder()
                .args(format_args!("{message}"))
                .level(level)
                .target(metadata.target())
                .module_path(metadata.module_path())
                .file(metadata.file())
                .line(metadata.line())
                .build(),
        );
    }

    fn enter(&self, _span: &Id) {}

    fn exit(&self, _span: &Id) {}
}

/// 安装全局转发（`main` 最早调用，晚于任何 tracing dispatcher 抢占者即失败）。
///
/// # Errors
/// 全局 dispatcher 已被占用（返回占用错误描述）。
pub fn init() -> Result<(), String> {
    set_global_default(Dispatch::new(ForwardToLog {
        next_id: AtomicU64::new(1),
    }))
    .map_err(|e| format!("{e:?}"))
}
