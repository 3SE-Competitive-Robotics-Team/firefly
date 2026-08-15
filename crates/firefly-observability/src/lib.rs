//! 日志与追踪基础设施。
//!
//! 基于 logforth + fastrace 的最佳实践（fastrace/examples/log-integration）：
//! - 日志记录携带当前 span 信息（`FastraceDiagnostic`）
//! - 日志同时作为 span 事件（`FastraceEvent`），在 trace 中可见
//! - 日志级别由 `RUST_LOG` 控制，未设置时仅输出 error
//!
//! 用法：
//! ```
//! firefly_observability::init();
//! // ...
//! firefly_observability::flush();
//! ```

use logforth::filter::rustlog::RustLogFilterBuilder;

pub fn init() {
    logforth::starter_log::builder()
        .dispatch(|d| {
            d.filter(RustLogFilterBuilder::from_default_env().build())
                .diagnostic(logforth::diagnostic::FastraceDiagnostic::default())
                .append(logforth::append::Stderr::default())
        })
        .dispatch(|d| d.append(logforth::append::FastraceEvent::default()))
        .apply();

    fastrace::set_reporter(
        fastrace::collector::ConsoleReporter,
        fastrace::collector::Config::default(),
    );
}

pub fn flush() {
    fastrace::flush();
}
