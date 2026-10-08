//! `apps/render` 视觉配置（`configs/render.toml`）：场景光照、相机曝光与深度退化。
//! 场景光照本体（唯一一份、所有相机共用）在 [`firefly_render::lighting`]。

use bevy::prelude::*;
use serde::{Deserialize, Serialize};

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
#[derive(Deserialize, Serialize, Clone, Copy, Debug)]
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
#[derive(Deserialize, Serialize, Clone, Copy, Debug)]
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
#[derive(Deserialize, Serialize, Clone, Copy, Debug)]
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

/// 体素显示配置。
#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct VoxelsConfig {
    /// 静态场地 `.ffmap` 路径（相对仓库根；缺文件则静态层不显示）。
    pub field_map: String,
}

impl Default for VoxelsConfig {
    fn default() -> Self {
        Self {
            field_map: "apps/planner/maps/rmuc2026.ffmap".to_owned(),
        }
    }
}

/// `configs/render.toml` 顶层：场景光照、相机曝光与深度退化。
#[derive(Resource, Deserialize, Clone, Debug, Default)]
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
    /// 深度传感器退化模型。
    #[serde(default)]
    pub depth_noise: crate::depth_noise::DepthNoiseOptions,
    /// 体素显示（静态场地 + 实时感知）。
    #[serde(default)]
    pub voxels: VoxelsConfig,
}

/// 读 `configs/render.toml`；缺文件/解析失败即报错退出。
#[must_use]
pub fn load() -> RenderConfig {
    let text = std::fs::read_to_string(RENDER_CONFIG).unwrap_or_else(|e| {
        log::error!("读取 configs/render.toml 失败：{e}");
        std::process::exit(1);
    });
    let config: RenderConfig = toml::from_str(&text).unwrap_or_else(|e| {
        log::error!("解析 configs/render.toml 失败：{e}");
        std::process::exit(1);
    });
    config.depth_noise.validate().unwrap_or_else(|e| {
        log::error!("深度噪声配置无效：{e}");
        std::process::exit(1);
    });
    config
}

/// 离线资产兼容契约；算法版本与生产二进制另外记录为 provenance。
#[derive(Serialize)]
pub struct AssetContract {
    schema: u32,
    depth_labels: &'static str,
    width: usize,
    height: usize,
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
    pub fn asset_contract(&self) -> AssetContract {
        AssetContract {
            schema: 1,
            depth_labels: "ideal_geometry_z",
            width: firefly_pubsub::camera::IMAGE_WIDTH,
            height: firefly_pubsub::camera::IMAGE_HEIGHT,
            fov_y_deg: crate::rig::FOV_Y_DEG,
            downtilt_deg: crate::rig::DOWNTILT_DEG,
            left_offset: crate::rig::LEFT_OFFSET.to_array(),
            depth_offset: crate::rig::DEPTH_OFFSET.to_array(),
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
    fn asset_contract_excludes_online_noise_and_viewer_but_tracks_sensor_exposure() {
        let mut config = RenderConfig::default();
        let original = toml::to_string(&config.asset_contract()).unwrap();
        config.depth_noise.temporal_fraction = 0.5;
        config.view.ev100 = Some(14.);
        assert_eq!(original, toml::to_string(&config.asset_contract()).unwrap());
        config.view.sensor_ev100 += 1.;
        assert_ne!(original, toml::to_string(&config.asset_contract()).unwrap());
    }
}
