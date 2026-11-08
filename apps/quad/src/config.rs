//! `configs/quad.toml`：第三人称飞行的场地/起点 + 机体、机体布局与 PX4 控制参数。
//!
//! 参数类型定义在 `firefly-flight`（四旋翼模型与飞控的唯一实现），缺键回落
//! 对应类型的默认值。

use bevy::prelude::*;
use firefly_flight::{Airframe, Px4Params, QuadParams};
use serde::Deserialize;

/// 顶层配置。
#[derive(Resource, Deserialize, Clone, Debug)]
#[serde(default, deny_unknown_fields)]
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
    /// PX4 控制参数（角速度/姿态/位置环、分配 airmode）。
    #[serde(default)]
    pub control: Px4Params,
    /// 手柄到姿态/推力的映射参数。
    #[serde(default)]
    pub manual: ManualParams,
}

/// 手柄映射：倾角上限复用 `control`，升降与偏航速率在此配置。
#[derive(Deserialize, Clone, Copy, Debug)]
#[serde(default, deny_unknown_fields)]
pub struct ManualParams {
    /// 满油门升降速度（m/s）。
    pub climb_rate_max: f32,
    /// 升降速度误差 → 垂直加速度（1/s）。
    pub vz_kp: f32,
    /// 满舵偏航角速度（rad/s，绕产品世界 +Z）。
    pub yaw_rate_max: f32,
}

impl QuadConfig {
    pub fn validate(&self) -> firefly_error::Result<()> {
        self.control.validate()
    }
}

impl Default for ManualParams {
    fn default() -> Self {
        Self {
            climb_rate_max: 3.0,
            vz_kp: 4.0,
            yaw_rate_max: 3.0,
        }
    }
}

impl Default for QuadConfig {
    fn default() -> Self {
        Self {
            field: "rmuc2027/v1.0.0/field.glb".into(),
            start: [0.0, -2.0, 1.2],
            drone: QuadParams::default(),
            airframe: Airframe::default(),
            control: Px4Params::default(),
            manual: ManualParams::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_unknown_fields_and_invalid_shared_controls() {
        for source in [
            "feild = 'field.glb'",
            "[manual]\nvz_kpp = 4",
            "[drone]\nmas = 1",
            "[airframe]\nrotorrs = []",
            "[control]\npositon = {}",
            "[control.rate]\npp = [1, 1, 1]",
        ] {
            assert!(toml::from_str::<QuadConfig>(source).is_err(), "{source}");
        }
        for source in [
            "[control.allocation]\nairmode = 255",
            "[control.rate]\np = [nan, 1, 1]",
        ] {
            assert!(
                toml::from_str::<QuadConfig>(source)
                    .unwrap()
                    .validate()
                    .is_err(),
                "{source}"
            );
        }
        QuadConfig::default().validate().unwrap();
    }

    #[test]
    fn shipped_config_preserves_control_overrides() {
        let config: QuadConfig =
            toml::from_str(include_str!("../../../configs/quad.toml")).unwrap();
        assert_eq!(config.field, "rmuc2027/v1.0.0/field.glb");
        for (actual, expected) in config.start.iter().zip([0.0, -2.0, 1.2]) {
            assert!((actual - expected).abs() < 1e-6);
        }
        // 240Hz 定步长使用对应的角速度环与姿态环增益。
        assert!((config.control.rate.p[0] - 0.02).abs() < 1e-6);
        assert!((config.control.attitude.p[0] - 9.0).abs() < 1e-6);
        assert!((config.manual.climb_rate_max - 3.0).abs() < 1e-6);
        assert!((config.manual.vz_kp - 4.0).abs() < 1e-6);
        assert!((config.manual.yaw_rate_max - 3.0).abs() < 1e-6);
    }
}
