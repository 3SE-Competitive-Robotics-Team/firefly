//! `configs/gicp.toml` 加载（纯数据 `Options`，缺键回落 `Default`）。

use std::path::Path;

use firefly_error::{Error, ErrorKind, Result};
use serde::Deserialize;

use crate::filter::FusionOptions;
use crate::reloc::RelocOptions;

/// 顶层：`[reloc]` + `[fusion]`。
#[derive(Debug, Clone, Deserialize, Default)]
#[serde(default)]
pub struct LocalizationConfig {
    /// 重定位参数。
    pub reloc: RelocOptions,
    /// 融合参数。
    pub fusion: FusionOptions,
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
    fn defaults_roundtrip() {
        let cfg = LocalizationConfig::default();
        assert!((cfg.reloc.downsampling_resolution - 0.2).abs() < 1e-12);
        assert!((cfg.reloc.max_correspondence_distance - 2.0).abs() < 1e-12);
        assert!((cfg.fusion.min_inlier_ratio - 0.3).abs() < 1e-12);
    }

    #[test]
    fn partial_toml_falls_back() {
        let cfg: LocalizationConfig = toml::from_str("fusion.min_inlier_ratio = 0.4").unwrap();
        assert!((cfg.fusion.min_inlier_ratio - 0.4).abs() < 1e-12);
        assert!((cfg.reloc.downsampling_resolution - 0.2).abs() < 1e-12);
    }

    /// 部分子表缺键回落紧画像（保守：缺配置不断言零值，不静默放行）。
    #[test]
    fn partial_subtable_falls_back_to_tight() {
        let cfg: LocalizationConfig =
            toml::from_str("[fusion.visual]\nmax_innovation_trans = 20.0").unwrap();
        assert!((cfg.fusion.visual.max_innovation_trans - 20.0).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_innovation_rot - 5.0_f64.to_radians()).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_correction_trans - 0.5).abs() < 1e-12);
        assert!((cfg.fusion.gicp.max_innovation_trans - 0.3).abs() < 1e-12);
    }

    #[test]
    fn shipped_config_parses() {
        let cfg = LocalizationConfig::load(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../configs/gicp.toml"
        ))
        .expect("configs/gicp.toml must parse");
        assert!((cfg.reloc.downsampling_resolution - 0.1).abs() < 1e-12);
        assert!((cfg.fusion.min_inlier_ratio - 0.3).abs() < 1e-12);
        // 双画像接线：几何紧门 / 视觉松门（VINS 尺度）
        assert!((cfg.fusion.gicp.max_innovation_trans - 0.3).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_innovation_trans - 20.0).abs() < 1e-12);
        assert!((cfg.fusion.process_noise_pos_per_m - 0.04).abs() < 1e-12);
        assert!((cfg.fusion.gicp.max_correction_trans - 0.5).abs() < 1e-12);
        assert!((cfg.fusion.visual.max_correction_trans - 20.0).abs() < 1e-12);
    }
}
