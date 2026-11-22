//! Shared error classification, diagnostic context and process reporting.

mod error;
mod kind;
mod status;

pub use error::{Error, Result};
pub use kind::ErrorKind;
pub use status::ErrorStatus;

/// Print one terminal diagnostic on stderr and return the process exit code.
pub fn report(app: &str, result: Result<()>) -> std::process::ExitCode {
    report_with(result, |error| eprintln!("{app}: {error}"))
}

/// Report a failure through the application's active logger or protocol.
pub fn report_with(result: Result<()>, emit: impl FnOnce(&Error)) -> std::process::ExitCode {
    match result {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(error) => {
            emit(&error);
            std::process::ExitCode::FAILURE
        }
    }
}

/// Add the operation and the affected path to a filesystem error.
pub trait IoResultExt<T> {
    /// # Errors
    /// Returns the classified I/O error with its original source.
    fn at_path(self, operation: &'static str, path: &std::path::Path) -> Result<T>;
}

impl<T> IoResultExt<T> for std::io::Result<T> {
    #[track_caller]
    fn at_path(self, operation: &'static str, path: &std::path::Path) -> Result<T> {
        match self {
            Ok(value) => Ok(value),
            Err(source) => Err(Error::from(source)
                .with_operation(operation)
                .with_context("path", path.display())),
        }
    }
}
