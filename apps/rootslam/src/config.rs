//! Estimator settings shared by IPC and recorded sensor sources.
use firefly_basalt::utils::vio_config::VioConfig;
use firefly_error::{Error, ErrorKind, Result};
use std::path::PathBuf;

#[derive(Default, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Options {
    pub vio_config: Option<PathBuf>,
    /// Detection cell width in pixels; overrides the Basalt archive when present.
    pub optical_flow_detection_grid_size: Option<i32>,
    pub stats_directory: Option<PathBuf>,
    pub map: bool,
    /// 回环检测开关；关闭后只保留局部建图与关键帧共视图。
    pub loops: bool,
    /// Let localization exclusively publish corrected odometry and navigation.
    pub external_localization: bool,
}

impl Options {
    /// # Errors
    /// Rejects a detection grid that does not fit every calibrated camera.
    pub fn validate_model(&self, model: &crate::input::SensorModel) -> Result<()> {
        let config = initial_vio_config(self)?;
        if config.optical_flow_detection_grid_size <= 0
            || model
                .calibration
                .resolution
                .iter()
                .any(|r| config.optical_flow_detection_grid_size > r.x.min(r.y))
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "Detection grid does not fit the sensor calibration",
            )
            .with_operation("rootslam.config.validate"));
        }
        Ok(())
    }
    /// # Errors
    /// Rejects nonpositive detection cells before loading a sensor source.
    pub fn validate(&self) -> Result<()> {
        if self
            .optical_flow_detection_grid_size
            .is_some_and(|size| size <= 0)
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "optical_flow_detection_grid_size must be positive",
            ));
        }
        Ok(())
    }
}

/// Builds the Basalt VIO configuration with bias and yaw initialization priors.
///
/// The upstream archive replaces every field on load and skips the yaw prior, so the yaw
/// information is applied after the archive is read.
#[expect(
    clippy::field_reassign_with_default,
    reason = "The yaw prior must be applied after the archive load replaces every field"
)]
pub(crate) fn initial_vio_config(options: &Options) -> Result<VioConfig> {
    options.validate()?;
    let mut vio = VioConfig::default();
    // Basalt stores precision: invert the squared standard deviations.
    vio.vio_init_ba_weight = 1.0 / 0.1_f64.powi(2);
    vio.vio_init_bg_weight = 1.0 / 0.01_f64.powi(2);
    if let Some(path) = &options.vio_config {
        vio.load(path)?;
    }
    if let Some(size) = options.optical_flow_detection_grid_size {
        vio.optical_flow_detection_grid_size = size;
    }
    // Keep yaw information independent of the translation prior.
    vio.vio_init_yaw_weight = Some(1e2);
    Ok(vio)
}
