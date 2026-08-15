//! 错误状态：显式的重试语义。

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorStatus {
    Permanent,
    Temporary,
    Persistent,
}

impl ErrorStatus {
    #[must_use]
    pub const fn is_retryable(self) -> bool {
        matches!(self, Self::Temporary | Self::Persistent)
    }
}

impl core::fmt::Display for ErrorStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Permanent => f.write_str("permanent"),
            Self::Temporary => f.write_str("temporary"),
            Self::Persistent => f.write_str("persistent"),
        }
    }
}
