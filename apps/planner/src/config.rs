//! `configs/planner.toml` 加载：缺键回落默认值，文件缺失/解析失败报错。

use std::path::Path;

use firefly_error::{Error, ErrorKind, Result};
use firefly_planner::{ManagerOptions, PlannerConfig};
use serde::Deserialize;

/// 顶层：平铺键 = [`PlannerConfig`]，`[manager]` = [`ManagerOptions`]。
#[derive(Debug, Deserialize, Default)]
#[serde(default)]
pub struct PlannerToml {
    #[serde(flatten)]
    pub config: PlannerConfig,
    pub manager: ManagerOptions,
}

impl PlannerToml {
    /// 从 TOML 文件加载。
    ///
    /// # Errors
    ///
    /// 文件不可读（`NotFound`）或 TOML 解析失败。
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let raw = std::fs::read_to_string(path)
            .map_err(|e| Error::new(ErrorKind::NotFound, "config file not found").with_source(e))?;
        toml::from_str(&raw)
            .map_err(|e| Error::new(ErrorKind::InvalidArgument, "invalid config").with_source(e))
    }
}

#[cfg(test)]
mod tests {
    use super::PlannerToml;
    use firefly_planner::manager::{DEFAULT_ARRIVE_DIST, DEFAULT_REPLAN_THRESH};

    /// 随仓库发布的配置文件必须可解析且为完整部署值。
    #[test]
    fn shipped_config_parses() {
        let cfg = PlannerToml::load(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../configs/planner.toml"
        ))
        .expect("shipped configs/planner.toml must parse");
        assert!((cfg.config.max_velocity - 1.5).abs() < 1e-12);
        assert!((cfg.config.weight_obstacle - 10_000.0).abs() < 1e-9);
        assert!((cfg.manager.replan_thresh - DEFAULT_REPLAN_THRESH).abs() < 1e-12);
    }

    /// 缺键回落默认值（最小化配置合法）。
    #[test]
    fn partial_file_falls_back_to_defaults() {
        let cfg: PlannerToml = toml::from_str("max_velocity = 2.0").unwrap();
        assert!((cfg.config.max_velocity - 2.0).abs() < 1e-12);
        assert!((cfg.config.piece_length - 1.5).abs() < 1e-12);
        assert!((cfg.manager.arrive_dist - DEFAULT_ARRIVE_DIST).abs() < 1e-12);
    }
}
