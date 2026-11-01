// SPDX-License-Identifier: BSD-3-Clause
//! The optional `sdk` feature links the system RealSense SDK through pkg-config.
//! Unsafe code is confined to this crate; consumers receive owned frame data.
#[cfg(feature = "sdk")]
mod sdk;
#[cfg(feature = "sdk")]
pub use sdk::*;
