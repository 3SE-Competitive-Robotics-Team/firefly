//! 错误分类：按调用者可以采取的动作划分。

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorKind {
    NotFound,
    InvalidArgument,
    OutOfRange,
    Unsupported,
    ResourceExhausted,
    Timeout,
    Convergence,
    Internal,
}

impl ErrorKind {
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::NotFound => "not found",
            Self::InvalidArgument => "invalid argument",
            Self::OutOfRange => "out of range",
            Self::Unsupported => "unsupported",
            Self::ResourceExhausted => "resource exhausted",
            Self::Timeout => "timeout",
            Self::Convergence => "convergence",
            Self::Internal => "internal",
        }
    }
}

impl core::fmt::Display for ErrorKind {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}
