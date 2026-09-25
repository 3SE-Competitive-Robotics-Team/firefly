//! `apps/render` 视觉配置（`configs/render.toml`）：场景光照的覆盖值与各相机曝光。
//! 场景光照本体（唯一一份、所有相机共用）在 [`firefly_render::lighting`]。

use bevy::prelude::*;
use serde::Deserialize;

/// 配置路径（编译期绝对路径，与运行 `CWD` 无关）。
const RENDER_CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../configs/render.toml");

/// 场景三方向光缺省照度（lux）。
pub const DEFAULT_DIRECTIONAL: [f32; 3] = firefly_render::lighting::ILLUMINANCE;
/// 传感器相机曝光缺省（EV100）。
pub const DEFAULT_SENSOR_EV100: f32 = 11.0;
/// 环境光贴图缺省颜色/强度（sRGB 0~1；强度缩放后单位 cd/m²）。
pub const DEFAULT_ENV_TOP: [f32; 3] = [0.62, 0.68, 0.78];
pub const DEFAULT_ENV_MID: [f32; 3] = [0.35, 0.38, 0.44];
pub const DEFAULT_ENV_BOTTOM: [f32; 3] = [0.09, 0.09, 0.10];
pub const DEFAULT_ENV_INTENSITY: f32 = 1500.0;

/// 场景照明配置。
#[derive(Deserialize, Clone, Copy, Debug)]
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

/// 相机曝光配置（传感器 rig 与主视角各一份，主视角缺省同值）。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct ViewConfig {
    /// 传感器相机曝光 EV100（越大越暗）。
    #[serde(default = "default_sensor_ev100")]
    pub sensor_ev100: f32,
    /// 主视角曝光 EV100（缺省 = `sensor_ev100`）。
    #[serde(default)]
    pub ev100: Option<f32>,
}

fn default_sensor_ev100() -> f32 {
    DEFAULT_SENSOR_EV100
}

impl ViewConfig {
    /// 主视角曝光（未显式配置时与传感器一致）。
    #[must_use]
    pub fn viewer_ev100(&self) -> f32 {
        self.ev100.unwrap_or(self.sensor_ev100)
    }
}

impl Default for ViewConfig {
    fn default() -> Self {
        Self {
            sensor_ev100: DEFAULT_SENSOR_EV100,
            ev100: None,
        }
    }
}

/// 场景环境光贴图（IBL）配置。
///
/// 半球渐变"天空"（`EnvironmentMapLight::hemispherical_gradient`）：近黑高光面
/// 只靠方向光会「黑底 + 一点高光」，IBL 同时给暗部补光（漫反射）与柔和反射
/// （镜面），是黑亮面读得出形体的关键。颜色为 sRGB 0~1，`intensity` 缩放后
/// 单位 cd/m²。
#[derive(Deserialize, Clone, Copy, Debug)]
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

/// `configs/render.toml` 顶层：场景光照的覆盖值与相机曝光。
#[derive(Resource, Deserialize, Clone, Copy, Debug, Default)]
pub struct RenderConfig {
    /// 场景照明。
    #[serde(default)]
    pub light: LightConfig,
    /// 相机曝光。
    #[serde(default)]
    pub view: ViewConfig,
    /// 场景环境光贴图。
    #[serde(default)]
    pub env: EnvConfig,
}

/// 读 `configs/render.toml`；缺文件/解析失败即报错退出。
#[must_use]
pub fn load() -> RenderConfig {
    let text = std::fs::read_to_string(RENDER_CONFIG).unwrap_or_else(|e| {
        log::error!("读取 configs/render.toml 失败：{e}");
        std::process::exit(1);
    });
    toml::from_str(&text).unwrap_or_else(|e| {
        log::error!("解析 configs/render.toml 失败：{e}");
        std::process::exit(1);
    })
}
