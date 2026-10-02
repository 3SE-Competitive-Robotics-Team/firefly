//! `configs/localization.toml` 加载（纯数据 `Options`，缺键回落 `Default`）。

use std::path::Path;

use firefly_error::{Error, ErrorKind, Result};
use serde::Deserialize;

use crate::filter::FusionOptions;

/// 地图启动先验与视觉融合配置。
#[derive(Debug, Clone, Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct LocalizationConfig {
    /// 固定启动先验 map←odom；属于部署配置，不来自在线真值。
    pub origin: InitialAlignment,
    /// 融合参数。
    pub fusion: FusionOptions,
}

/// 已知启动坐标：局部 VIO 原点在地图中的位置与航向。
#[derive(Debug, Clone, Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct InitialAlignment {
    /// 米，地图系。
    pub position: [f64; 3],
    /// 弧度，map←odom 绕 +Z 的旋转。
    pub yaw: f64,
}
impl InitialAlignment {
    /// 构造固定启动先验。
    /// # Errors
    /// 参数非有限。
    pub fn transform(&self) -> Result<firefly_base::RigidTransform> {
        let half = self.yaw * 0.5;
        firefly_base::RigidTransform::from_parts(
            firefly_base::FrameId::MAP,
            firefly_base::FrameId::ODOM,
            self.position,
            [0., 0., half.sin(), half.cos()],
        )
    }
}

impl LocalizationConfig {
    /// 从 TOML 文件加载。
    ///
    /// # Errors
    ///
    /// 文件不可读或解析失败。
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let raw = std::fs::read_to_string(path.as_ref())
            .map_err(|e| Error::new(ErrorKind::NotFound, "config file not found").with_source(e))?;
        toml::from_str(&raw)
            .map_err(|e| Error::new(ErrorKind::InvalidArgument, "invalid config").with_source(e))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_origin_and_reset_preserve_known_alignment() {
        let cfg: LocalizationConfig =
            toml::from_str("[origin]\nposition = [-13.0, 0.0, 0.405]\nyaw = 1.5707963267948966")
                .unwrap();
        let alignment = cfg.origin.transform().unwrap();
        let mut fusion = crate::FusionFilter::with_default();
        fusion.set_alignment(alignment).unwrap();
        fusion.reset();
        let p = fusion.corrected_pose(&nalgebra::Matrix4::identity());
        assert!((p[(0, 3)] + 13.0).abs() < 1e-12);
        assert!((p[(2, 3)] - 0.405).abs() < 1e-12);
        assert!((p[(1, 0)] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn misplaced_fusion_options_are_rejected() {
        assert!(
            toml::from_str::<LocalizationConfig>("[fusion.visual]\nr_floor_pos = 0.1").is_err()
        );
    }

    #[test]
    fn defaults_roundtrip() {
        let cfg = LocalizationConfig::default();
        assert!((cfg.fusion.min_inlier_ratio - 0.3).abs() < 1e-12);
    }

    #[test]
    fn partial_toml_falls_back() {
        let cfg: LocalizationConfig = toml::from_str("fusion.min_inlier_ratio = 0.4").unwrap();
        assert!((cfg.fusion.min_inlier_ratio - 0.4).abs() < 1e-12);
    }

    /// 部分子表缺键回落紧画像（保守：缺配置不断言零值，不静默放行）。
    #[test]
    fn partial_subtable_falls_back_to_tight() {
        let cfg: LocalizationConfig =
            toml::from_str("[fusion.visual]\nmax_innovation_trans = 20.0").unwrap();
        assert!((cfg.fusion.visual.max_innovation_trans - 20.0).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_innovation_rot - 5.0_f64.to_radians()).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_correction_trans - 0.5).abs() < 1e-12);
    }

    #[test]
    fn shipped_config_parses() {
        let cfg = LocalizationConfig::load(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../configs/localization.toml"
        ))
        .expect("configs/localization.toml must parse");
        assert!((cfg.fusion.min_inlier_ratio - 0.3).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_innovation_trans - 20.0).abs() < 1e-12);
        assert!((cfg.fusion.process_noise_pos_per_m - 0.04).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_correction_trans - 20.0).abs() < 1e-12);
        assert!((cfg.fusion.p_min_pos - 0.25).abs() < 1e-12);
    }
}
