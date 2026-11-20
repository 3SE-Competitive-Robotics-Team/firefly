//! Service capacities shared by native mission requests and their HIL relay.

use crate::{
    node::IpcNode,
    publish::Publisher,
    subscriber::{Subscriber, SubscriberOptions},
};
use iceoryx2::prelude::ZeroCopySend;
use std::fmt::Debug;

/// The daemon, the board HIL relay and an operator's ffctl can publish concurrently.
pub const REQUEST_MAX_PUBLISHERS: usize = 3;

pub trait MissionRequest: Debug + ZeroCopySend + Sized + 'static {
    const TOPIC: &'static str;
    const BUFFER_SIZE: usize;

    /// # Errors
    /// Returns an error if the IPC service or publisher cannot be opened.
    fn publisher(node: &IpcNode) -> firefly_error::Result<Publisher<Self>> {
        Publisher::with_topic_buffer_publishers(
            node,
            Self::TOPIC,
            Some(Self::BUFFER_SIZE),
            Some(REQUEST_MAX_PUBLISHERS),
        )
    }

    /// FC and planner consume the latest request. The relay can queue a burst.
    ///
    /// # Errors
    /// Returns an error if the IPC service or subscriber cannot be opened.
    fn subscriber(node: &IpcNode) -> firefly_error::Result<Subscriber<Self>> {
        Subscriber::with_options(
            node,
            Self::TOPIC,
            SubscriberOptions {
                buffer_size: 1,
                service_max_buffer_size: Some(Self::BUFFER_SIZE),
                max_publishers: Some(REQUEST_MAX_PUBLISHERS),
            },
        )
    }
}
