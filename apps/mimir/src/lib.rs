//! Native IPC recording with bounded ingress and synchronous, columnar RRD writes.
pub mod archive;
pub mod config;
mod frames;
pub mod ingress;
pub mod schema;
mod status;

use firefly_error::{Error, ErrorKind};

fn failure(operation: &'static str, error: impl std::fmt::Display) -> Error {
    Error::new(ErrorKind::Internal, error.to_string()).with_operation(operation)
}

fn invalid(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("decode observation")
}
