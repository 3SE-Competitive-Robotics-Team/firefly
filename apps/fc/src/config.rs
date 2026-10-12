//! `configs/fc.toml`：飞控进程参数（缺键回落 `firefly-flight` 默认值）。
//!
//! 只承载**飞控侧**参数（控制环频率、增益）：质量/惯量/旋翼几何/电机推力上限
//! 是被控对象属性，由被控对象经 `Firefly/Airframe` 发布，见 `main.rs`。

use firefly_flight::{ControlParams, FsmParams};
use serde::Deserialize;

/// 顶层配置。
#[derive(Deserialize, Clone, Debug)]
pub struct FcConfig {
    /// 独立姿态估计器的噪声、静止窗口与创新门限。
    #[serde(default)]
    pub imu: firefly_imu::Options,
    /// 控制环频率（Hz）。指令按此频率发布，被控对象取最新样本。
    #[serde(default = "d_rate_hz")]
    pub rate_hz: f64,
    /// 飞控参数（姿态/位置增益、倾角与角速度限幅）。
    #[serde(default)]
    pub control: ControlParams,
    /// 飞行状态机参数（起降高度/速率、参考失联超时、落地判据）。
    #[serde(default)]
    pub fsm: FsmParams,
}

fn d_rate_hz() -> f64 {
    1000.0
}

impl FcConfig {
    pub fn validate(&self) -> firefly_error::Result<()> {
        if !self.rate_hz.is_finite() || self.rate_hz <= 0. {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "rate_hz must be finite and positive",
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shipped_config_preserves_control_overrides() {
        let config: FcConfig = toml::from_str(include_str!("../../../configs/fc.toml")).unwrap();
        let defaults: FcConfig = toml::from_str("").unwrap();
        let expected = ControlParams {
            pos_kp: 2.2,
            vel_kp: 2.4,
            ..ControlParams::default()
        };
        assert!((config.rate_hz - defaults.rate_hz).abs() < f64::EPSILON);
        assert_eq!(format!("{:?}", config.control), format!("{expected:?}"));
    }
}
