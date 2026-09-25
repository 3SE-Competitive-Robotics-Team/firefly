//! `configs/quad.toml`：飞行 demo 的场地/起点 + `firefly-flight` 的机体、机体布局与飞控参数。
//!
//! 参数类型定义在 `firefly-flight`（四旋翼模型与飞控的唯一实现），缺键回落
//! 那边的默认值（219g 级微型机）；本文件只写需要按 demo 调整的键。

use bevy::prelude::*;
use firefly_flight::{Airframe, ControlParams, QuadParams};
use serde::Deserialize;

/// 顶层配置。
#[derive(Resource, Deserialize, Clone, Debug)]
pub struct QuadConfig {
    /// 场地 glb（相对 `models/`）。
    pub field: String,
    /// 初始位置（米，世界系）。
    pub start: [f32; 3],
    /// 刚体参数（质量/惯量/气动阻尼）。
    #[serde(default)]
    pub drone: QuadParams,
    /// 机体布局（旋翼几何/旋向/电机推力上限）。
    #[serde(default)]
    pub airframe: Airframe,
    /// 飞控参数。
    #[serde(default)]
    pub control: ControlParams,
}

/// 读配置；缺文件/解析失败即报错退出。
#[must_use]
pub fn load(path: &str) -> QuadConfig {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| {
        log::error!("读取 {path} 失败：{e}");
        std::process::exit(1);
    });
    toml::from_str(&text).unwrap_or_else(|e| {
        log::error!("解析 {path} 失败：{e}");
        std::process::exit(1);
    })
}
