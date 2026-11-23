use bevy::prelude::*;
use firefly_error::{Error, ErrorKind, Result};
use serde::{Deserialize, Serialize};

/// 场景三方向光缺省照度（lux）。
pub const DEFAULT_DIRECTIONAL: [f32; 3] = firefly_render::lighting::ILLUMINANCE;
pub const DEFAULT_SENSOR_EV100: f32 = 11.0;
/// 环境光贴图缺省颜色/强度（sRGB 0~1；强度缩放后单位 cd/m²）。
pub const DEFAULT_ENV_TOP: [f32; 3] = [0.62, 0.68, 0.78];
pub const DEFAULT_ENV_MID: [f32; 3] = [0.35, 0.38, 0.44];
pub const DEFAULT_ENV_BOTTOM: [f32; 3] = [0.09, 0.09, 0.10];
pub const DEFAULT_ENV_INTENSITY: f32 = 1500.0;
pub const DEFAULT_CAPTURE_HZ: f64 = 10.0;

#[derive(Deserialize, Serialize, Clone, Copy, Debug)]
#[serde(deny_unknown_fields)]
pub struct LightConfig {
    /// 场景三方向光照度（lux）。
    #[serde(default = "default_directional")]
    pub directional: [f32; 3],
}

fn default_directional() -> [f32; 3] {
    DEFAULT_DIRECTIONAL
}

impl Default for LightConfig {
    fn default() -> Self {
        Self {
            directional: DEFAULT_DIRECTIONAL,
        }
    }
}

#[derive(Deserialize, Serialize, Clone, Copy, Debug)]
#[serde(deny_unknown_fields)]
pub struct ViewConfig {
    /// 传感器相机曝光 EV100（越大越暗）。
    #[serde(default = "default_sensor_ev100")]
    pub sensor_ev100: f32,
}

fn default_sensor_ev100() -> f32 {
    DEFAULT_SENSOR_EV100
}

impl Default for ViewConfig {
    fn default() -> Self {
        Self {
            sensor_ev100: DEFAULT_SENSOR_EV100,
        }
    }
}

#[derive(Deserialize, Serialize, Clone, Copy, Debug)]
#[serde(deny_unknown_fields)]
pub struct EnvConfig {
    /// 顶色（sRGB 0~1）。
    #[serde(default = "default_env_top")]
    pub top: [f32; 3],
    /// 地平线色（sRGB 0~1）。
    #[serde(default = "default_env_mid")]
    pub mid: [f32; 3],
    /// 底色（sRGB 0~1）。
    #[serde(default = "default_env_bottom")]
    pub bottom: [f32; 3],
    /// 强度缩放（cd/m²）。
    #[serde(default = "default_env_intensity")]
    pub intensity: f32,
}

fn default_env_top() -> [f32; 3] {
    DEFAULT_ENV_TOP
}

fn default_env_mid() -> [f32; 3] {
    DEFAULT_ENV_MID
}

fn default_env_bottom() -> [f32; 3] {
    DEFAULT_ENV_BOTTOM
}

fn default_env_intensity() -> f32 {
    DEFAULT_ENV_INTENSITY
}

impl Default for EnvConfig {
    fn default() -> Self {
        Self {
            top: DEFAULT_ENV_TOP,
            mid: DEFAULT_ENV_MID,
            bottom: DEFAULT_ENV_BOTTOM,
            intensity: DEFAULT_ENV_INTENSITY,
        }
    }
}

#[derive(Deserialize, Serialize, Clone, Copy, Debug)]
#[serde(deny_unknown_fields)]
pub struct CaptureConfig {
    /// 采集频率（Hz）；上限受 Bevy 120Hz 调度约束。
    #[serde(default = "default_capture_hz")]
    pub hz: f64,
    /// 连续下视采集频率，仅允许 15/30 FPS，实际频率受输入位姿与总采集节拍限制。
    #[serde(default = "default_down_hz")]
    pub down_hz: u32,
}

const fn default_down_hz() -> u32 {
    15
}

fn default_capture_hz() -> f64 {
    DEFAULT_CAPTURE_HZ
}

impl Default for CaptureConfig {
    fn default() -> Self {
        Self {
            hz: DEFAULT_CAPTURE_HZ,
            down_hz: default_down_hz(),
        }
    }
}

impl CaptureConfig {
    /// 采集周期（秒，sim 时间轴）。
    #[must_use]
    pub fn period_seconds(self) -> f64 {
        1.0 / self.hz
    }

    /// # Errors
    /// 频率非有限、非正或超过 120Hz 调度上限。
    pub fn validate(self) -> Result<()> {
        if !self.hz.is_finite() || self.hz <= 0.0 || self.hz > 120.0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                format!("capture.hz 必须在 (0, 120] Hz 内，当前 {}", self.hz),
            )
            .with_operation("render config"));
        }
        if !matches!(self.down_hz, 15 | 30) {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "capture.down_hz 必须为 15 或 30 FPS",
            )
            .with_operation("render config"));
        }
        Ok(())
    }
}

#[derive(Resource, Deserialize, Clone, Debug, Default)]
#[serde(deny_unknown_fields)]
pub struct RenderConfig {
    #[serde(default)]
    pub light: LightConfig,
    #[serde(default)]
    pub view: ViewConfig,
    #[serde(default)]
    pub capture: CaptureConfig,
    #[serde(default)]
    pub env: EnvConfig,
}

/// 离线资产兼容契约；算法版本与生产二进制另外记录为 provenance。
#[derive(Serialize)]
pub struct AssetContract {
    schema: u32,
    depth_labels: &'static str,
    width: usize,
    height: usize,
    principal_point: [f64; 2],
    fov_y_deg: f32,
    downtilt_deg: f32,
    left_offset: [f32; 3],
    depth_offset: [f32; 3],
    near: f32,
    sensor_ev100: f32,
    light: LightConfig,
    env: EnvConfig,
}

impl RenderConfig {
    /// # Errors
    /// 采集频率越界。
    pub fn validate(&self) -> Result<()> {
        self.capture.validate()
    }

    pub fn asset_contract(&self) -> AssetContract {
        AssetContract {
            schema: 1,
            depth_labels: "ideal_geometry_z",
            width: firefly_pubsub::camera::IMAGE_WIDTH,
            height: firefly_pubsub::camera::IMAGE_HEIGHT,
            principal_point: firefly_base::rig::principal_point(
                firefly_pubsub::camera::IMAGE_WIDTH,
                firefly_pubsub::camera::IMAGE_HEIGHT,
            ),
            fov_y_deg: crate::rig::FOV_Y_DEG,
            downtilt_deg: crate::rig::DOWNTILT_DEG,
            left_offset: firefly_base::rig::LEFT_IN_BODY.map(|m| m as f32),
            depth_offset: firefly_base::rig::DEPTH_IN_BODY.map(|m| m as f32),
            near: crate::rig::SENSOR_NEAR,
            sensor_ev100: self.view.sensor_ev100,
            light: self.light,
            env: self.env,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn continuous_down_capture_accepts_only_15_or_30_fps() {
        for down_hz in [15, 30] {
            assert!(
                CaptureConfig {
                    down_hz,
                    ..Default::default()
                }
                .validate()
                .is_ok()
            );
        }
        for down_hz in [0, 10, 60] {
            assert!(
                CaptureConfig {
                    down_hz,
                    ..Default::default()
                }
                .validate()
                .is_err()
            );
        }
    }
    #[test]
    fn rejects_unknown_fields_at_each_config_level() {
        for source in [
            "ligth = {}",
            "[light]\ndirectionall = [1, 2, 3]",
            "[view]\nsensor_ev10 = 12",
            "[capture]\nhzz = 60",
            "[env]\nintensitty = 1500",
            "[depth_noise]\nenabld = false",
        ] {
            assert!(toml::from_str::<RenderConfig>(source).is_err(), "{source}");
        }
    }
    #[test]
    fn asset_contract_tracks_sensor_exposure() {
        let mut config = RenderConfig::default();
        let original = toml::to_string(&config.asset_contract()).unwrap();
        config.view.sensor_ev100 += 1.;
        assert_ne!(original, toml::to_string(&config.asset_contract()).unwrap());
    }
}
