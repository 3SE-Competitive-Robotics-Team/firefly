//! `configs/planner.toml` 的参数定义；缺失字段使用对应类型的默认值。

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
    pub mapping: MappingOptions,
}

#[cfg(test)]
mod tests {
    use super::PlannerToml;
    use firefly_planner::manager::DEFAULT_REPLAN_THRESH;

    /// 随仓库发布的配置文件必须可解析且为完整部署值。
    #[test]
    fn shipped_config_parses() {
        let cfg = firefly_config::load::<PlannerToml>(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../configs/planner.toml"
        ))
        .expect("shipped configs/planner.toml must parse");
        assert!(cfg.config.yaw_hold);
        assert!((cfg.config.obstacle_clearance - 0.4).abs() < 1e-12);
        assert!((cfg.config.max_velocity - 1.5).abs() < 1e-12);
        assert!((cfg.config.weight_obstacle - 10_000.0).abs() < 1e-9);
        assert_eq!(cfg.config.virtual_ground, Some(-0.505));
        assert_eq!(cfg.config.virtual_ceiling, Some(2.195));
        assert!((cfg.manager.replan_thresh - DEFAULT_REPLAN_THRESH).abs() < 1e-12);
    }

    /// 缺键回落默认值（最小化配置合法）。
    #[test]
    fn partial_file_falls_back_to_defaults() {
        let cfg: PlannerToml = toml::from_str("max_velocity = 2.0").unwrap();
        assert!((cfg.config.max_velocity - 2.0).abs() < 1e-12);
        assert!((cfg.config.piece_length - 1.5).abs() < 1e-12);
        // 缺键 = 不启用虚拟墙（官方 enable_virtual_wall 默认 false）
        assert_eq!(cfg.config.virtual_ground, None);
        assert_eq!(cfg.config.virtual_ceiling, None);
    }
}

/// 在线滚动地图；米，范围至少覆盖默认深度 8m 与局部规划视野。
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MappingOptions {
    pub resolution: f64,
    pub range: [f64; 3],
    /// 移出局部窗口的观测体素上限；耗尽时停止规划，零禁用历史保留。
    pub history_capacity: usize,
}
impl Default for MappingOptions {
    fn default() -> Self {
        Self {
            resolution: 0.1,
            range: [8., 8., 4.],
            history_capacity: 500_000,
        }
    }
}
impl MappingOptions {
    pub fn build(&self, clearance: f64) -> Result<firefly_map::GridMap> {
        if !self.resolution.is_finite()
            || self.resolution <= 0.
            || self.range.iter().any(|v| !v.is_finite() || *v <= 0.)
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "invalid online map dimensions",
            ));
        }
        let mut map = firefly_map::GridMapBuilder::new(self.resolution, [1; 3])
            .with_local_update_range(self.range)
            .with_obstacles_inflation(clearance)
            .with_observation_history(self.history_capacity)
            .build()?;
        map.move_ring_buffer(nalgebra::Vector3::zeros());
        Ok(map)
    }
}
