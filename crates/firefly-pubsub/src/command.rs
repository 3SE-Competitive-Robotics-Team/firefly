//! 地面站指令消息（外部工具 → 飞控，事件语义，非周期流）。
//!
//! 指令语义 = **"谁在控制"**（对照 `MAVLink` `COMMAND_LONG` 的解锁/模式类子集）：
//! 目标点与轨迹属规划侧（`Firefly/Goal` → planner → `Firefly/Reference`），飞控只在
//! `Track` 下消费参考流。飞控订阅本话题，按 [`CommandMessage::sequence`] 去重——
//! 一次性 CLI 为打通信道会重复投递同一条指令（见 `firefly-goal` 的连接竞态说明），
//! 去重保证只执行一次。
//!
//! `#[repr(C)]` 定长零拷贝，与 Python 侧 `firefly_mujoco.messages.CommandMessage`
//! 布局/类型名严格一致。

use iceoryx2::prelude::*;

/// 指令话题（外部工具 → 飞控）。
pub const COMMAND_TOPIC: &str = "Firefly/Command";

/// 指令种类（线上编码，`Rust`/Python 双端同值；对应 `firefly_flight::Command` 的变体）。
pub mod kind {
    /// 解锁（`Command::Arm`）。
    pub const ARM: u8 = 1;
    /// 上锁（`Command::Disarm`）。
    pub const DISARM: u8 = 2;
    /// 自动起飞到 `altitude`（相对起飞点，m；`Command::Takeoff`）。
    pub const TAKEOFF: u8 = 3;
    /// 位置保持（`Command::Hold`）。
    pub const HOLD: u8 = 4;
    /// 跟踪外部参考流（`Command::Track`）。
    pub const TRACK: u8 = 5;
    /// 自动降落（`Command::Land`）。
    pub const LAND: u8 = 6;
}

/// 指令种类名（日志用；未知编码返回 `None`）。
///
/// 编码到 `firefly_flight::Command` 的映射在飞控进程（`apps/fc`）——本 crate 只承载
/// 线上表示，不认识状态机语义。
#[must_use]
pub fn kind_name(kind: u8) -> Option<&'static str> {
    match kind {
        kind::ARM => Some("ARM"),
        kind::DISARM => Some("DISARM"),
        kind::TAKEOFF => Some("TAKEOFF"),
        kind::HOLD => Some("HOLD"),
        kind::TRACK => Some("TRACK"),
        kind::LAND => Some("LAND"),
        _ => None,
    }
}

/// 地面站指令：种类 + 参数 + 去重序号。
#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
#[type_name("FireflyCommandMessage")]
pub struct CommandMessage {
    /// 发布时刻（墙钟秒，仅诊断用；飞控以自身 clk 判新旧）。
    pub timestamp: f64,
    /// 指令种类（`kind` 模块的 `*` 编码）。
    pub kind: u8,
    /// 指令参数：`kind::TAKEOFF` 用（相对起飞点高度，m），其余种类忽略。
    pub altitude: f64,
    /// 指令序号（发布端保证严格递增，如墙钟纳秒）：飞控只执行序号更大的指令，
    /// 重复投递的同一条指令因此只生效一次。
    pub sequence: u64,
}

impl Default for CommandMessage {
    fn default() -> Self {
        Self {
            timestamp: -1.0,
            kind: kind::ARM,
            altitude: 0.0,
            sequence: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_message_is_plain_old_data() {
        let m = CommandMessage::default();
        // 8（timestamp）+ 1（kind）+ 7（对齐填充）+ 8（altitude）+ 8（sequence）
        assert_eq!(std::mem::size_of::<CommandMessage>(), 32);
        assert!(m.altitude.abs() < 1e-12);
        assert_eq!(kind_name(m.kind), Some("ARM"));
    }

    #[test]
    fn kind_names_are_unique_and_unknown_is_none() {
        let named = [
            kind::ARM,
            kind::DISARM,
            kind::TAKEOFF,
            kind::HOLD,
            kind::TRACK,
            kind::LAND,
        ]
        .map(kind_name);
        assert!(named.iter().all(Option::is_some), "{named:?}");
        assert_eq!(kind_name(0), None);
        assert_eq!(kind_name(7), None);
    }
}
