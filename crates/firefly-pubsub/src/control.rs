use iceoryx2::prelude::*;

pub const CONTROL_TOPIC: &str = "Firefly/Control";

/// 指令服务容量与订阅缓冲深度；Python 侧使用同值以兼容服务创建顺序。
pub const CONTROL_BUFFER_SIZE: usize = 32;

#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyControlMessage")]
pub struct ControlMessage {
    /// 飞控本次控制所用的状态采样时刻（传感器时钟，秒）。
    pub state_time: f64,
    /// 4 电机推力（N，顺序与机体旋翼编号一致；已由飞控侧限幅）。
    pub thrust: [f64; 4],
    /// 飞控 tick 计数（自增；连续性/丢包自检）。
    pub tick: u64,
}

impl Default for ControlMessage {
    fn default() -> Self {
        Self {
            state_time: -1.0,
            thrust: [0.0; 4],
            tick: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_message_is_plain_old_data() {
        let m = ControlMessage::default();
        assert_eq!(std::mem::size_of::<ControlMessage>(), 48);
        assert!(m.thrust.iter().all(|t| t.abs() < 1e-12), "{:?}", m.thrust);
        assert!((m.state_time + 1.0).abs() < 1e-9);
    }
}
