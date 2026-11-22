use iceoryx2::prelude::*;

pub const STICKS_TOPIC: &str = "Firefly/Sticks";

pub const STICKS_BUFFER_SIZE: usize = 2;

/// Matrix primary channels in microseconds: roll, pitch, yaw, throttle.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, ZeroCopySend)]
#[type_name("FireflySticksMessage")]
pub struct SticksMessage {
    pub state_time: f64,
    pub channels: [u16; 4],
}

impl SticksMessage {
    #[must_use]
    pub fn valid(self) -> bool {
        self.state_time.is_finite()
            && self.state_time >= 0.0
            && self.channels.iter().all(|c| (1000..=2000).contains(c))
    }
}
