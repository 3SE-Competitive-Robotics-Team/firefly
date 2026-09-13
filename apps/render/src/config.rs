//! `apps/render` 视觉配置（`configs/render.toml`）：只管光照，与场景无关联。
//!
//! 光照是目视调参项——场景切换不动它，改亮度不必重编译。方向固定（三盏，比值
//! 对照 `MuJoCo` diffuse 0.7:0.3:0.22），此处只调照度/环境光。

use bevy::prelude::*;
use serde::Deserialize;

/// 配置路径（编译期绝对路径，与运行 `CWD` 无关）。
const RENDER_CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../configs/render.toml");

/// 三方向光缺省照度（lux；直射日光量级，方向比值同 `MuJoCo`）。
pub const DEFAULT_DIRECTIONAL: [f32; 3] = [40000.0, 17000.0, 13000.0];
/// 全局环境光缺省（cd/m²，`Bevy` 缺省 80）。
pub const DEFAULT_AMBIENT: f32 = 50.0;

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

/// `configs/render.toml` 顶层。
#[derive(Resource, Deserialize, Clone, Copy, Debug, Default)]
pub struct RenderConfig {
    /// 光照。
    #[serde(default)]
    pub light: LightConfig,
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
