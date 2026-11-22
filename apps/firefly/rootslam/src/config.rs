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

pub(crate) fn live_vio_config(options: &Options) -> Result<VioConfig> {
    let mut vio = initial_vio_config(options)?;
    if options.vio_config.is_none() {
        // The controller keeps its goals in the odometry frame chosen at startup.
        // Use Basalt's first-pose precision for that frame's yaw as well as its origin.
        vio.vio_init_yaw_weight = None;
        // A horizontal accelerometer bias of 0.1 m/s² tilts the measured gravity
        // direction by about 0.01 rad. Carry that measurement into the first solve.
        vio.vio_init_tilt_weight = 1.0 / 0.01_f64.powi(2);
        // The live flight starts at rest on the pad (velocity sigma 0.01 m/s).
        vio.vio_init_velocity_weight = 1.0 / 0.01_f64.powi(2);
        // Pattern 50 spans 10.5 pixels at each level. Use the full and
        // half-resolution images to measure disparity on this 320×240 rig.
        vio.optical_flow_pattern = 50;
        vio.optical_flow_levels = 1;
        vio.optical_flow_rematch_stereo = true;
    }
    Ok(vio)
}
