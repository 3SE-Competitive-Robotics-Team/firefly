use core::fmt;

use crate::{ErrorKind, ErrorStatus};

#[derive(Debug, Clone, Copy)]
struct Location {
    file: &'static str,
    line: u32,
    column: u32,
}

/// A classified failure with its origin, operation, diagnostic context and source.
#[derive(Clone)]
pub struct Error {
    kind: ErrorKind,
    status: ErrorStatus,
    message: String,
    operation: Option<&'static str>,
    location: Location,
    context: Vec<(&'static str, String)>,
    source: Option<std::sync::Arc<dyn std::error::Error + Send + Sync>>,
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

    /// Attach the underlying failure at a boundary that knows the caller's recovery options.
    #[track_caller]
    pub fn caused_by(
        kind: ErrorKind,
        message: impl Into<String>,
        source: impl std::error::Error + Send + Sync + 'static,
    ) -> Self {
        Self::new(kind, message).with_source(source)
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

    /// 设置当前操作；上层边界再次设置时替换该字段，不写入通用上下文。
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
        self.source = Some(std::sync::Arc::new(source));
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

impl From<std::fs::TryLockError> for Error {
    #[track_caller]
    fn from(error: std::fs::TryLockError) -> Self {
        match error {
            std::fs::TryLockError::WouldBlock => {
                Self::caused_by(ErrorKind::Unavailable, "file is already locked", error)
            }
            std::fs::TryLockError::Error(source) => Self::from(source),
        }
    }
}

impl From<std::io::Error> for Error {
    #[track_caller]
    fn from(value: std::io::Error) -> Self {
        use std::io::ErrorKind as Io;
        let value = match value.downcast::<Self>() {
            Ok(original) => return original,
            Err(value) => value,
        };
        let kind = match value.kind() {
            Io::NotFound => ErrorKind::NotFound,
            Io::PermissionDenied => ErrorKind::PermissionDenied,
            Io::InvalidInput => ErrorKind::InvalidArgument,
            Io::InvalidData | Io::UnexpectedEof => ErrorKind::InvalidData,
            Io::TimedOut => ErrorKind::Timeout,
            Io::Unsupported => ErrorKind::Unsupported,
            Io::OutOfMemory | Io::StorageFull => ErrorKind::ResourceExhausted,
            Io::WouldBlock | Io::ResourceBusy | Io::ConnectionRefused => ErrorKind::Unavailable,
            _ => ErrorKind::Io,
        };
        Self::caused_by(kind, "I/O failed", value)
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
    fn io_boundary_preserves_the_cause_and_path_without_promising_a_retry() {
        use crate::IoResultExt;
        use std::error::Error as _;
        use std::io::ErrorKind as Io;
        for (input, expected) in [
            (Io::NotFound, ErrorKind::NotFound),
            (Io::InvalidData, ErrorKind::InvalidData),
            (Io::PermissionDenied, ErrorKind::PermissionDenied),
            (Io::TimedOut, ErrorKind::Timeout),
            (Io::BrokenPipe, ErrorKind::Io),
        ] {
            let result: std::io::Result<()> = Err(input.into());
            let error = result
                .at_path("read config", std::path::Path::new("camera.toml"))
                .unwrap_err();
            assert_eq!(error.kind(), expected);
            assert!(!error.is_retryable());
            assert_eq!(error.operation(), Some("read config"));
            assert_eq!(error.context().next(), Some(("path", "camera.toml")));
            assert_eq!(
                error
                    .source()
                    .unwrap()
                    .downcast_ref::<std::io::Error>()
                    .unwrap()
                    .kind(),
                input
            );
            assert_eq!(error.location.file, file!());
        }
    }

    #[test]
    fn io_adapter_preserves_classification_and_context() {
        let busy = Error::temporary(ErrorKind::ResourceExhausted, "sample loan")
            .with_context("topic", "Test");
        let original = busy.to_string();
        let recovered = Error::from(std::io::Error::other(busy));
        assert_eq!(recovered.kind(), ErrorKind::ResourceExhausted);
        assert!(recovered.is_retryable());
        assert_eq!(recovered.to_string(), original);
    }

    #[test]
    fn kind_and_status() {
        let e = Error::temporary(ErrorKind::Timeout, "planning timed out");
        assert_eq!(e.kind(), ErrorKind::Timeout);
        assert_eq!(e.status(), ErrorStatus::Temporary);
        assert!(e.is_retryable());
    }

    #[test]
    fn error_parts_and_display() {
        let e = Error::new(ErrorKind::InvalidArgument, "bad input");
        assert!(!e.is_retryable());
        let e = Error::new(ErrorKind::OutOfRange, "index out of bounds")
            .with_context("piece", 3)
            .with_context("dim", "x");
        let ctx: Vec<_> = e.context().collect();
        assert_eq!(ctx.len(), 2);
        assert_eq!(ctx[0], ("piece", "3"));
        assert_eq!(ctx[1], ("dim", "x"));
        let e = Error::new(ErrorKind::Internal, "boom");
        let text = e.to_string();
        assert!(text.contains("error.rs"), "missing file: {text}");
        assert!(text.contains(':'), "missing line:col: {text}");
        let e = Error::new(ErrorKind::Convergence, "lbfgs failed")
            .with_operation("optimize")
            .with_context("iterations", 100);
        let text = e.to_string();
        assert!(
            text.contains("[convergence] lbfgs failed (optimize)"),
            "{text}"
        );
        assert!(text.contains("iterations: 100"), "{text}");
        assert_eq!(e.operation(), Some("optimize"));
        assert_eq!(e.context().collect::<Vec<_>>(), vec![("iterations", "100")]);
        let e = e.with_operation("plan");
        assert_eq!(e.operation(), Some("plan"));
        let inner = std::io::Error::new(std::io::ErrorKind::NotFound, "no such file");
        let e = Error::new(ErrorKind::NotFound, "map file missing").with_source(inner);
        assert!(e.to_string().contains("caused by: no such file"));
        assert!(std::error::Error::source(&e).is_some());
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
