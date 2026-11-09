//! `configs/planner.toml` 的参数定义；缺失字段使用对应类型的默认值。

use firefly_error::{Error, ErrorKind, Result};
use firefly_planner::{ManagerOptions, PlannerConfig};
use serde::Deserialize;

/// 顶层：平铺键 = [`PlannerConfig`]，`[manager]` = [`ManagerOptions`]。
#[derive(Debug, Default)]
pub struct PlannerToml {
    pub config: PlannerConfig,
    pub manager: ManagerOptions,
    pub mapping: MappingOptions,
}

impl<'de> Deserialize<'de> for PlannerToml {
    fn deserialize<D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> std::result::Result<Self, D::Error> {
        let mut table = toml::Table::deserialize(deserializer)?;
        let manager = table
            .remove("manager")
            .map(toml::Value::try_into)
            .transpose()
            .map_err(serde::de::Error::custom)?
            .unwrap_or_default();
        let mapping = table
            .remove("mapping")
            .map(toml::Value::try_into)
            .transpose()
            .map_err(serde::de::Error::custom)?
            .unwrap_or_default();
        let config = toml::Value::Table(table)
            .try_into()
            .map_err(serde::de::Error::custom)?;
        Ok(Self {
            config,
            manager,
            mapping,
        })
    }
}

impl PlannerToml {
    pub fn validate(&self) -> Result<()> {
        if !self.config.heartbeat_timeout.is_finite() || self.config.heartbeat_timeout <= 0. {
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

    #[test]
    fn rejects_unknown_fields_without_changing_flattened_layout() {
        for source in [
            "max_velocty = 2.0",
            "[manager]\nplanning_horizonn = 5",
            "[mapping]\nresoluton = 0.1",
            "[mapping.depth]\nmax_pose_gapp = 0.1",
        ] {
            assert!(toml::from_str::<PlannerToml>(source).is_err(), "{source}");
        }
        let cfg: PlannerToml =
            toml::from_str("max_velocity = 2\n[manager]\nplanning_horizon = 5").unwrap();
        assert!((cfg.config.max_velocity - 2.).abs() < f64::EPSILON);
        assert!((cfg.manager.planning_horizon - 5.).abs() < f64::EPSILON);
    }

    #[test]
    fn rejects_non_finite_and_non_positive_heartbeat_timeout() {
        for value in ["nan", "inf", "-inf", "0", "-1"] {
            let cfg: PlannerToml = toml::from_str(&format!("heartbeat_timeout = {value}")).unwrap();
            assert!(cfg.validate().is_err(), "{value}");
        }
        PlannerToml::default().validate().unwrap();
    }
}

/// 在线滚动地图；米，范围至少覆盖默认深度 8m 与局部规划视野。
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MappingOptions {
    pub resolution: f64,
    pub range: [f64; 3],
    pub depth: DepthOptions,
}
impl Default for MappingOptions {
    fn default() -> Self {
        Self {
            resolution: 0.1,
            range: [8., 8., 4.],
            depth: DepthOptions::default(),
        }
    }
}

/// Deployment calibration and launch overrides. Image filtering/probabilities use upstream
/// `advanced_param.xml` defaults; the numerical crate itself keeps the ROS parameter defaults.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DepthOptions {
    /// CV pinhole fx, fy, cx, cy for the axial depth image.
    pub intrinsics: [f64; 4],
    pub translation: [f64; 3],
    /// body <- depth optical camera, Hamilton xyzw.
    pub rotation: [f64; 4],
    pub use_depth_filter: bool,
    pub depth_filter_mindist: f64,
    pub depth_filter_margin: i32,
    pub k_depth_scaling_factor: f64,
    pub skip_pixel: i32,
    pub p_hit: f64,
    pub p_miss: f64,
    pub p_min: f64,
    pub p_max: f64,
    pub p_occ: f64,
    pub fading_time: f64,
    /// Maximum timestamp gap between the two odometry samples used for interpolation.
    pub max_pose_gap: f64,
    /// Extra fusion extent around the planning grid; coverage is still checked at runtime.
    pub window_margin: f64,
}

impl Default for DepthOptions {
    fn default() -> Self {
        use firefly_base::{FrameId, rig};
        use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_WIDTH};
        let rotation =
            nalgebra::UnitQuaternion::from_matrix(&(rig::cam_axes_in_body() * rig::CV_FROM_RENDER));
        let q = rotation.quaternion();
        let focal = IMAGE_HEIGHT as f64 * 0.5 / (rig::FOV_Y_DEG.to_radians() * 0.5).tan();
        let center = rig::principal_point(IMAGE_WIDTH, IMAGE_HEIGHT);
        Self {
            intrinsics: [focal, focal, center[0], center[1]],
            translation: rig::position_in_body(FrameId::DEPTH_CAMERA).into(),
            rotation: [q.i, q.j, q.k, q.w],
            use_depth_filter: true,
            depth_filter_mindist: 0.2,
            depth_filter_margin: 2,
            k_depth_scaling_factor: 1000.,
            skip_pixel: 2,
            p_hit: 0.65,
            p_miss: 0.35,
            p_min: 0.12,
            p_max: 0.90,
            p_occ: 0.80,
            fading_time: 1000.,
            max_pose_gap: 0.05,
            window_margin: 1.,
        }
    }
}

impl DepthOptions {
    pub fn extrinsic(&self) -> Result<nalgebra::Isometry3<f64>> {
        Ok(*firefly_base::RigidTransform::from_parts(
            firefly_base::FrameId::BODY,
            firefly_base::FrameId::DEPTH_CAMERA,
            self.translation,
            self.rotation,
        )?
        .isometry())
    }

    pub fn parameters(
        &self,
        grid: &firefly_map::GridMap,
        clearance: f64,
    ) -> Result<firefly_ego_map::MappingParameters> {
        let extrinsic = self.extrinsic()?;
        if !self.max_pose_gap.is_finite()
            || self.max_pose_gap <= 0.
            || self.max_pose_gap > super::MAP_TIMEOUT
            || !self.window_margin.is_finite()
            || self.window_margin <= 0.
            || self.depth_filter_margin < 0
            || self.depth_filter_margin as usize * 2 >= firefly_pubsub::camera::IMAGE_HEIGHT
            || !self.depth_filter_mindist.is_finite()
            || self.depth_filter_mindist <= 0.
            || !self.k_depth_scaling_factor.is_finite()
            || self.k_depth_scaling_factor <= 0.
            || self.k_depth_scaling_factor > f64::from(f32::MAX)
            || self.skip_pixel <= 0
            || self.skip_pixel as usize >= firefly_pubsub::camera::IMAGE_HEIGHT
            || !clearance.is_finite()
            || clearance < 0.
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "invalid depth mapping options",
            ));
        }
        let [fx, fy, cx, cy] = self.intrinsics;
        let padding = self.window_margin + extrinsic.translation.vector.norm();
        Ok(firefly_ego_map::MappingParameters {
            resolution: grid.resolution(),
            local_update_range: (grid.size() * 0.5 + nalgebra::Vector3::repeat(padding)).into(),
            obstacles_inflation: clearance,
            fx,
            fy,
            cx,
            cy,
            odom_depth_timeout: super::MAP_TIMEOUT,
            use_depth_filter: self.use_depth_filter,
            depth_filter_mindist: self.depth_filter_mindist,
            depth_filter_margin: self.depth_filter_margin,
            k_depth_scaling_factor: self.k_depth_scaling_factor,
            skip_pixel: self.skip_pixel,
            p_hit: self.p_hit,
            p_miss: self.p_miss,
            p_min: self.p_min,
            p_max: self.p_max,
            p_occ: self.p_occ,
            fading_time: self.fading_time,
            ..Default::default()
        })
    }
}
impl MappingOptions {
    pub fn build(&self, clearance: f64) -> Result<firefly_map::GridMap> {
        if !self.resolution.is_finite()
            || self.resolution <= 0.
            || self.range.iter().any(|v| !v.is_finite() || *v <= 0.)
            || !clearance.is_finite()
            || clearance < 0.
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "invalid online map dimensions",
            ));
        }
        let mut map = firefly_map::GridMapBuilder::new(self.resolution, [1; 3])
            .with_local_update_range(self.range)
            .with_obstacles_inflation(clearance)
            .build()?;
        map.move_ring_buffer(nalgebra::Vector3::zeros());
        Ok(map)
    }
}
