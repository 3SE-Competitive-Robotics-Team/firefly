use super::{DOWN_HEIGHT, DOWN_WIDTH, DepthSlot, DownSlot, GraySlot, IMAGE_HEIGHT, IMAGE_WIDTH};
use crate::{node::IpcNode, publish::Publisher, trace::TraceContext};
use firefly_error::{Error, ErrorKind, Result};
use std::mem::MaybeUninit;

// The private writer view and the reader share the same C wire layout. Keeping the
// slot private makes publishing a partially initialized image impossible through this API.
macro_rules! image_publisher {
    ($publisher:ident, $slot:ident, $pixel:ty, $w:ident, $h:ident) => {
        impl Default for $slot {
            #[expect(
                clippy::large_stack_arrays,
                reason = "fixed IPC layout with uninitialized pixel storage"
            )]
            fn default() -> Self {
                Self {
                    timestamp: 0.,
                    sensor_id: 0,
                    width: $w as u32,
                    height: $h as u32,
                    data: [MaybeUninit::uninit(); $w * $h],
                }
            }
        }
        /// Publishes complete row-major images directly into shared memory.
        pub struct $publisher(Publisher<$slot>);
        impl $publisher {
            /// # Errors
            /// Returns IPC service or publisher creation errors.
            pub fn open(node: &IpcNode, topic: &str, buffer: Option<usize>) -> Result<Self> {
                Ok(Self(Publisher::with_topic_and_buffer(node, topic, buffer)?))
            }
            /// # Errors
            /// A short or oversized image is discarded. Propagates IPC errors.
            pub fn publish_pixels(
                &self,
                timestamp: f64,
                sensor_id: i32,
                pixels: impl IntoIterator<Item = $pixel>,
            ) -> Result<TraceContext> {
                self.0.try_publish_with(|sample| {
                    sample.timestamp = timestamp;
                    sample.sensor_id = sensor_id;
                    let mut iter = pixels.into_iter();
                    for slot in &mut sample.data {
                        slot.write(iter.next().ok_or_else(|| {
                            Error::new(ErrorKind::InvalidData, "incomplete camera image")
                        })?);
                    }
                    if iter.next().is_some() {
                        return Err(Error::new(ErrorKind::InvalidData, "oversized camera image"));
                    }
                    Ok(())
                })
            }
        }
    };
}
image_publisher!(GrayImagePublisher, GraySlot, u8, IMAGE_WIDTH, IMAGE_HEIGHT);
image_publisher!(DownImagePublisher, DownSlot, u8, DOWN_WIDTH, DOWN_HEIGHT);
image_publisher!(
    DepthImagePublisher,
    DepthSlot,
    f32,
    IMAGE_WIDTH,
    IMAGE_HEIGHT
);
