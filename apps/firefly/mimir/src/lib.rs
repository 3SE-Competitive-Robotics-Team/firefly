mod archive;
pub mod config;
mod frames;
pub mod ingress;
mod schema;
mod segments;
mod status;

use firefly_error::{Error, ErrorKind};

fn failure(
    operation: &'static str,
    error: impl std::error::Error + Send + Sync + 'static,
) -> Error {
    Error::caused_by(ErrorKind::Internal, operation, error)
}

fn invalid(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("decode observation")
}
