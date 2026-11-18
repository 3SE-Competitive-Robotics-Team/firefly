use iceoryx2::prelude::*;

pub const FC_STATUS_TOPIC: &str = "Firefly/FcStatus";
pub const FC_STATUS_BUFFER_SIZE: usize = 16;

/// FC 状态快照，使用 IMU 测量时钟和 `FlightState::code` 编码。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, ZeroCopySend)]
#[type_name("FireflyFcStatus")]
pub struct FcStatusMessage {
    pub timestamp: f64,
    pub sequence: u64,
    pub mode: u8,
}
