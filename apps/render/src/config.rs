//! `apps/render` 视觉配置（`configs/render.toml`）：只管光照，与场景无关联。
//!
//! 光照是目视调参项——场景切换不动它，改亮度不必重编译。方向固定（三盏，比值
//! 对照 `MuJoCo` diffuse 0.7:0.3:0.22），此处只调照度/环境光。

use bevy::prelude::*;
use serde::Deserialize;

/// 配置路径（编译期绝对路径，与运行 `CWD` 无关）。
const RENDER_CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../configs/render.toml");

/// 三方向光缺省照度（lux；直射日光量级，方向比值同 `MuJoCo`）。
pub const DEFAULT_DIRECTIONAL: [f32; 3] = [32000.0, 14000.0, 10000.0];
/// 全局环境光缺省（cd/m²，`Bevy` 缺省 80）。
pub const DEFAULT_AMBIENT: f32 = 30.0;
/// 相机曝光缺省（EV100；越大越暗。缺省贴合正午阳光，与三方向光照度同量级）。
pub const DEFAULT_EV100: f32 = 15.0;
/// 主视角 bloom 强度缺省（0 关闭）。
pub const DEFAULT_BLOOM: f32 = 0.08;

/// 光照配置。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct LightConfig {
    /// 全局环境光（cd/m²）。
    #[serde(default = "default_ambient")]
    pub ambient: f32,
    /// 三方向光照度（lux）。
    #[serde(default = "default_directional")]
    pub directional: [f32; 3],
}

fn default_ambient() -> f32 {
    DEFAULT_AMBIENT
}

fn default_directional() -> [f32; 3] {
    DEFAULT_DIRECTIONAL
}

impl Default for LightConfig {
    fn default() -> Self {
        Self {
            ambient: DEFAULT_AMBIENT,
            directional: DEFAULT_DIRECTIONAL,
        }
    }
}

/// 相机/后处理配置。
///
/// 曝光必须与光照量级匹配：灯光按 lux 锚定日光时 `ev100≈15`，否则整幅惨白
///（缺省 `Exposure::BLENDER = 9.7` 是室内量级）。同一曝光作用于传感器与主视角，
/// 保证发布图像与目视一致。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct ViewConfig {
    /// 相机曝光 EV100（越大越暗）。
    #[serde(default = "default_ev100")]
    pub ev100: f32,
    /// 主视角 bloom 强度（0 关闭；传感器相机不加）。
    #[serde(default = "default_bloom")]
    pub bloom_intensity: f32,
}

fn default_ev100() -> f32 {
    DEFAULT_EV100
}

fn default_bloom() -> f32 {
    DEFAULT_BLOOM
}

impl Default for ViewConfig {
    fn default() -> Self {
        Self {
            ev100: DEFAULT_EV100,
            bloom_intensity: DEFAULT_BLOOM,
        }
    }
}

/// `configs/render.toml` 顶层。
#[derive(Resource, Deserialize, Clone, Copy, Debug, Default)]
pub struct RenderConfig {
    /// 光照。
    #[serde(default)]
    pub light: LightConfig,
    /// 相机/后处理。
    #[serde(default)]
    pub view: ViewConfig,
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
