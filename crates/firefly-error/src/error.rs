//! 扁平错误结构：机器可读的 kind/status + 人类可读的上下文。

use core::fmt;

use crate::{ErrorKind, ErrorStatus};

#[derive(Debug, Clone, Copy)]
struct Location {
    file: &'static str,
    line: u32,
    column: u32,
}

pub struct Error {
    kind: ErrorKind,
    status: ErrorStatus,
    message: String,
    operation: Option<&'static str>,
    location: Location,
    context: Vec<(&'static str, String)>,
    source: Option<Box<dyn std::error::Error + Send + Sync>>,
}

impl Error {
    #[track_caller]
    pub fn new(kind: ErrorKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            status: ErrorStatus::Permanent,
            message: message.into(),
            operation: None,
            location: location(),
            context: Vec::new(),
            source: None,
        }
    }

    #[track_caller]
    pub fn temporary(kind: ErrorKind, message: impl Into<String>) -> Self {
        Self::new(kind, message).with_status(ErrorStatus::Temporary)
    }

    #[must_use]
    pub const fn kind(&self) -> ErrorKind {
        self.kind
    }

    #[must_use]
    pub const fn status(&self) -> ErrorStatus {
        self.status
    }

    #[must_use]
    pub const fn is_retryable(&self) -> bool {
        self.status.is_retryable()
    }

    #[must_use]
    pub fn message(&self) -> &str {
        &self.message
    }

    #[must_use]
    pub fn operation(&self) -> Option<&'static str> {
        self.operation
    }

    pub fn context(&self) -> impl Iterator<Item = (&'static str, &str)> {
        self.context.iter().map(|(k, v)| (*k, v.as_str()))
    }

    #[must_use]
    pub fn with_status(mut self, status: ErrorStatus) -> Self {
        self.status = status;
        self
    }

    #[must_use]
    pub fn with_operation(mut self, operation: &'static str) -> Self {
        self.operation = Some(operation);
        self
    }

    #[must_use]
    pub fn with_context(mut self, key: &'static str, value: impl core::fmt::Display) -> Self {
        self.context.push((key, value.to_string()));
        self
    }

    #[must_use]
    pub fn with_source(mut self, source: impl std::error::Error + Send + Sync + 'static) -> Self {
        self.source = Some(Box::new(source));
        self
    }
}

pub type Result<T> = core::result::Result<T, Error>;

impl fmt::Debug for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Error")
            .field("kind", &self.kind)
            .field("status", &self.status)
            .field("message", &self.message)
            .field("operation", &self.operation)
            .field("location", &self.location)
            .field("context", &self.context)
            .field("source", &self.source)
            .finish()
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{}] {}", self.kind, self.message)?;
        if let Some(op) = self.operation {
            write!(f, " ({op})")?;
        }
        write!(
            f,
            ", at {}:{}:{}",
            self.location.file, self.location.line, self.location.column
        )?;
        for (key, value) in &self.context {
            write!(f, "\n  {key}: {value}")?;
        }
        if let Some(source) = &self.source {
            write!(f, "\n  caused by: {source}")?;
        }
        Ok(())
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        self.source
            .as_deref()
            .map(|s| s as &(dyn std::error::Error + 'static))
    }
}

impl From<std::io::Error> for Error {
    fn from(value: std::io::Error) -> Self {
        Self::new(ErrorKind::Internal, "io error").with_source(value)
    }
}

#[track_caller]
fn location() -> Location {
    let loc = core::panic::Location::caller();
    Location {
        file: loc.file(),
        line: loc.line(),
        column: loc.column(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kind_and_status() {
        let e = Error::temporary(ErrorKind::Timeout, "planning timed out");
        assert_eq!(e.kind(), ErrorKind::Timeout);
        assert_eq!(e.status(), ErrorStatus::Temporary);
        assert!(e.is_retryable());
    }

    #[test]
    fn error_parts_and_display() {
        // 永久错误不可重试
        let e = Error::new(ErrorKind::InvalidArgument, "bad input");
        assert!(!e.is_retryable());
        // 上下文保留
        let e = Error::new(ErrorKind::OutOfRange, "index out of bounds")
            .with_context("piece", 3)
            .with_context("dim", "x");
        let ctx: Vec<_> = e.context().collect();
        assert_eq!(ctx.len(), 2);
        assert_eq!(ctx[0], ("piece", "3"));
        assert_eq!(ctx[1], ("dim", "x"));
        // 位置信息
        let e = Error::new(ErrorKind::Internal, "boom");
        let text = e.to_string();
        assert!(text.contains("error.rs"), "missing file: {text}");
        assert!(text.contains(':'), "missing line:col: {text}");
        // display 含全部部分
        let e = Error::new(ErrorKind::Convergence, "lbfgs failed")
            .with_operation("optimize")
            .with_context("iterations", 100);
        let text = e.to_string();
        assert!(
            text.contains("[convergence] lbfgs failed (optimize)"),
            "{text}"
        );
        assert!(text.contains("iterations: 100"), "{text}");
        // source 链
        let inner = std::io::Error::new(std::io::ErrorKind::NotFound, "no such file");
        let e = Error::new(ErrorKind::NotFound, "map file missing").with_source(inner);
        assert!(e.to_string().contains("caused by: no such file"));
        assert!(std::error::Error::source(&e).is_some());
        // Result 别名
        let f = |ok: bool| -> Result<i32> {
            if ok {
                Ok(1)
            } else {
                Err(Error::new(ErrorKind::Internal, "nope"))
            }
        };
        assert_eq!(f(true).unwrap(), 1);
        assert!(f(false).is_err());
    }
}
