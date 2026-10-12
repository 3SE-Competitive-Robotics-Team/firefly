//! `configs/quad.toml`：飞行 demo 的场地/起点 + `firefly-flight` 的机体、机体布局与飞控参数。
//!
//! 参数类型定义在 `firefly-flight`（四旋翼模型与飞控的唯一实现），缺键回落
//! 对应类型的默认值。

use bevy::prelude::*;
use firefly_flight::{Airframe, ControlParams, QuadParams};
use serde::Deserialize;

/// 顶层配置。
#[derive(Resource, Deserialize, Clone, Debug)]
#[serde(default)]
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

impl Default for QuadConfig {
    fn default() -> Self {
        Self {
            field: "rmuc2026/field.glb".into(),
            start: [0.0, -2.0, 1.2],
            drone: QuadParams::default(),
            airframe: Airframe::default(),
            control: ControlParams::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shipped_config_preserves_control_overrides() {
        let config: QuadConfig =
            toml::from_str(include_str!("../../../configs/quad.toml")).unwrap();
        let expected = ControlParams {
            attitude_kp: 9.0,
            rate_kp: 6.0,
            ..ControlParams::default()
        };
        assert_eq!(config.field, "rmuc2026/field.glb");
        assert_eq!(config.start, [0.0, -2.0, 1.2]);
        assert_eq!(format!("{:?}", config.control), format!("{expected:?}"));
    }
}
