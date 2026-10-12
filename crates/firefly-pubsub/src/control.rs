//! 飞控指令消息（`Firefly/Control`）：飞控 → 被控对象，1kHz，被控对象取最新样本。
//!
//! 指令是**4 个电机的推力**（N），不是世界系 wrench：被控对象把 4 个推力写进旋翼
//! 执行器，力的合成（力随机体 z、反扭矩随机体 z 按旋向）由被控对象的引擎按机体几何
//! 完成——`MuJoCo` 侧是 MJCF 里 site 上的 gear（对照 `mujoco_menagerie/skydio_x2`），
//! 自积分侧是 `firefly-flight::Airframe::realize`。换真机只把这里换成归一化电机指令
//! + 推力曲线。
//!
//! `state_time` 是飞控计算本指令所用的状态采样时刻：被控对象据此判断指令是否陈旧
//! （自己另给最新样本的年龄），`tick` 供连续性/丢包自检。

use iceoryx2::prelude::*;

/// 飞控指令话题（飞控 → 被控对象）。
pub const CONTROL_TOPIC: &str = "Firefly/Control";

/// 指令服务容量与订阅缓冲区深度（Python 侧同值）：32 条（1kHz 发布、被控对象 200Hz 排空——留 ~32ms 抖动余量）。
pub const CONTROL_BUFFER_SIZE: usize = 32;

/// 飞控指令：4 电机推力 + 所用状态时刻 + tick 计数。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyControlMessage")]
pub struct ControlMessage {
    /// 飞控本次控制所用的状态采样时刻（仿真秒；被控对象据此判陈旧）。
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
        // 8（state_time）+ 32（4 电机推力）+ 8（tick）
        assert_eq!(std::mem::size_of::<ControlMessage>(), 48);
        assert!(m.thrust.iter().all(|t| t.abs() < 1e-12), "{:?}", m.thrust);
        assert!((m.state_time + 1.0).abs() < 1e-9);
    }
}
