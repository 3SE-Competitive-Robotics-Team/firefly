//! 错误设计内核。
//!
//! 错误按"调用者能做什么"分类，而非按来源分类。
//! 参考 `FastLabs`: Stop Forwarding Errors, Start Designing Them。

mod error;
mod kind;
mod status;

pub use error::{Error, Result};
pub use kind::ErrorKind;
pub use status::ErrorStatus;
