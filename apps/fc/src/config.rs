//! `configs/fc.toml`：飞控进程参数（缺键回落 `firefly-flight` 默认值）。
//!
//! 只承载**飞控侧**参数（控制环频率、PX4 控制增益）：质量/惯量/旋翼几何/电机推力上限
//! 是被控对象属性，由被控对象经 `Firefly/Airframe` 发布，见 `main.rs`。

use firefly_flight::{FsmParams, Px4Params};
use serde::Deserialize;

/// 顶层配置。
#[derive(Deserialize, Clone, Debug)]
#[serde(deny_unknown_fields)]
pub struct FcConfig {
    /// 独立备用姿态估计器的噪声、静止窗口与创新门限。
    #[serde(default)]
    pub imu: firefly_imu::Options,
    /// 控制环频率（Hz）。指令按此频率发布，被控对象取最新样本。
    #[serde(default = "d_rate_hz")]
    pub rate_hz: f64,
    /// PX4 控制核心参数（角速度/姿态/位置环、分配 airmode）。
    #[serde(default)]
    pub control: Px4Params,
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
        self.imu.validate()?;
        self.control.validate()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shipped_config_parses_and_keeps_product_envelope() {
        let config: FcConfig = toml::from_str(include_str!("../../../configs/fc.toml")).unwrap();
        assert!((config.rate_hz - 1000.0).abs() < f64::EPSILON);
        assert_eq!(config.control.allocation.airmode, 1);
        assert!((config.control.position.tilt_max_deg - 32.0).abs() < 1e-6);
        assert!((config.control.position.vel_max_horizontal - 3.0).abs() < 1e-6);
        assert!((config.control.position.p[0] - 0.95).abs() < 1e-6);
        // 角速度/姿态环未覆盖，使用 PX4 默认值。
        assert!((config.control.rate.p[0] - 0.15).abs() < 1e-6);
        assert!((config.control.attitude.p[0] - 4.0).abs() < 1e-6);
        assert!((config.control.position.vel_i[0] - 0.4).abs() < 1e-6);
        config.validate().unwrap();
    }

    #[test]
    fn rejects_unknown_fields_and_invalid_control_values() {
        for source in [
            "rate_hzz = 1000",
            "[control]\npositon = {}",
            "[control.rate]\npp = [1, 1, 1]",
            "[control.attitude]\nyaw_weigth = 0.4",
            "[control.position]\nvel_pp = [1, 1, 1]",
            "[control.allocation]\nairmod = 1",
            "[fsm]\nreference_timout = 1",
            "[imu]\ngyro_densitty = 0.1",
        ] {
            assert!(toml::from_str::<FcConfig>(source).is_err(), "{source}");
        }
        for source in [
            "[control.allocation]\nairmode = 255",
            "[control.rate]\np = [nan, 1, 1]",
            "[control.attitude]\nyaw_weight = 2",
            "[imu]\nmax_dt = nan",
        ] {
            assert!(
                toml::from_str::<FcConfig>(source)
                    .unwrap()
                    .validate()
                    .is_err(),
                "{source}"
            );
        }
        let zero_limits: FcConfig = toml::from_str(
            "[control.rate]\nmax_deg_s = [0, 0, 0]\n[control.position]\nvel_max_horizontal = 0\nvel_max_up = 0\nvel_max_down = 0\n[control.attitude]\nref_model_freq = 0",
        ).unwrap();
        zero_limits.validate().unwrap();
    }
}
