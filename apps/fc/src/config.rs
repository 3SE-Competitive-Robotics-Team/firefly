//! `configs/fc.toml`：飞控进程参数（缺键回落 `firefly-flight` 默认值）。
//!
//! 只承载**飞控侧**参数（控制环频率、增益）：质量/惯量/旋翼几何/电机推力上限
//! 是被控对象属性，由被控对象经 `Firefly/Airframe` 发布，见 `main.rs`。

use firefly_flight::ControlParams;
use serde::Deserialize;

/// 顶层配置。
#[derive(Deserialize, Clone, Debug)]
pub struct FcConfig {
    /// 控制环频率（Hz）。指令按此频率发布，被控对象取最新样本。
    #[serde(default = "d_rate_hz")]
    pub rate_hz: f64,
    /// 飞控参数（姿态/位置增益、倾角与角速度限幅）。
    #[serde(default)]
    pub control: ControlParams,
}

fn d_rate_hz() -> f64 {
    1000.0
}

/// 读配置；缺文件/解析失败即报错退出。
#[must_use]
pub fn load(path: &str) -> FcConfig {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| {
        log::error!("读取 {path} 失败：{e}");
        std::process::exit(1);
    });
    toml::from_str(&text).unwrap_or_else(|e| {
        log::error!("解析 {path} 失败：{e}");
        std::process::exit(1);
    })
}
