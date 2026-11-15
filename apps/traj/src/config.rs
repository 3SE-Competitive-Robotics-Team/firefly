//! traj 的心跳失联门限，单位为秒。

use firefly_error::{Error, ErrorKind, Result};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Options {
    /// 失联后完成已检查的停车尾段，再停止采样。
    pub heartbeat_timeout: f64,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            heartbeat_timeout: firefly_planner::traj_server::DEFAULT_HEARTBEAT_TIMEOUT,
        }
    }
}

impl Options {
    pub fn validate(&self) -> Result<()> {
        if !self.heartbeat_timeout.is_finite() || self.heartbeat_timeout <= 0. {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "heartbeat_timeout must be finite and positive",
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shipped_config_and_override_are_valid() {
        let options: Options = toml::from_str(include_str!("../../../configs/traj.toml")).unwrap();
        options.validate().unwrap();
        assert_eq!(options.heartbeat_timeout, 0.5);
        let options: Options = toml::from_str("heartbeat_timeout = 0.8").unwrap();
        options.validate().unwrap();
        assert_eq!(options.heartbeat_timeout, 0.8);
    }

    #[test]
    fn rejects_invalid_timeouts_and_unknown_fields() {
        for value in ["nan", "inf", "-inf", "0", "-1"] {
            let options: Options = toml::from_str(&format!("heartbeat_timeout = {value}")).unwrap();
            assert!(options.validate().is_err(), "{value}");
        }
        assert!(toml::from_str::<Options>("heartbeat_timout = 1").is_err());
    }
}
