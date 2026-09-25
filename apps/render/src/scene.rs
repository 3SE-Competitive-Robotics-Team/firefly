//! 场景注册表：`configs/scene.toml` 的 `scene` → `models/<dir>/` 视觉资产 + 出生点。
//!
//! 统一接入约定（见 `packages/firefly-cad/README.md`）：`Bevy` asset root 恒为
//! [`firefly_render::scene::MODELS_DIR`]，视觉恒为 `models/<dir>/<visual>`；换场地改
//! `configs/scene.toml` 一行 + 注册表一项，不改加载代码。未注册名称回退
//! [`DEFAULT_SCENE`] 并告警。
//!
//! 世界观配置与 `sim` / `viz` 共用同一份 `configs/scene.toml`（单一来源防漂移）。

use bevy::prelude::*;
use serde::Deserialize;

/// 世界观配置（与 `sim` / `viz` 共用）。
const SCENE_CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../configs/scene.toml");

/// 一个场地的接入描述。
#[derive(Resource, Clone, Copy, Debug)]
pub struct SceneSpec {
    /// `models/` 下的目录名（即注册键，与 `configs/scene.toml` 的 `scene` 取值一致）。
    pub dir: &'static str,
    /// 目录内视觉 glb 文件名。
    pub visual: &'static str,
    /// 机体起点（米，`MuJoCo` 系；真值到达前的初始摆位，对照 `configs/sim.toml`）。
    pub start: [f32; 3],
}

impl SceneSpec {
    /// 相对 [`MODELS_DIR`] 的 asset 路径（对照 `AssetPlugin.file_path`）。
    #[must_use]
    pub fn asset_path(&self) -> String {
        format!("{}/{}", self.dir, self.visual)
    }
}

/// 缺省场景（`configs/scene.toml` 缺 `scene` 键时回落）。
pub const DEFAULT_SCENE: &str = "rmuc2026";

/// 已注册场地（视觉 glb + 起点；碰撞描述在 `MuJoCo` 侧注册表）。
const SCENES: &[SceneSpec] = &[
    SceneSpec {
        dir: "rmuc2026",
        visual: "field.glb",
        start: [2.0, 0.0, 1.0],
    },
    SceneSpec {
        dir: "warehouse",
        visual: "structure.glb",
        start: [2.0, 0.0, 1.0],
    },
];

/// `configs/scene.toml` 的反序列化形态。
#[derive(Deserialize)]
struct SceneConfig {
    #[serde(default = "default_scene_name")]
    scene: String,
}

fn default_scene_name() -> String {
    DEFAULT_SCENE.to_owned()
}

/// 读 `configs/scene.toml` 的 `scene`；缺文件/解析失败即报错退出。
fn load_scene_name() -> String {
    let text = std::fs::read_to_string(SCENE_CONFIG).unwrap_or_else(|e| {
        log::error!("读取 configs/scene.toml 失败：{e}");
        std::process::exit(1);
    });
    toml::from_str::<SceneConfig>(&text)
        .unwrap_or_else(|e| {
            log::error!("解析 configs/scene.toml 失败：{e}");
            std::process::exit(1);
        })
        .scene
}

/// 按 `configs/scene.toml` 的 `scene` 选场景；未注册回退 [`DEFAULT_SCENE`]。
#[must_use]
pub fn selected() -> SceneSpec {
    let name = load_scene_name();
    SCENES
        .iter()
        .copied()
        .find(|s| s.dir == name)
        .unwrap_or_else(|| {
            log::warn!("未知场景 {name}，回退 {DEFAULT_SCENE}");
            *SCENES
                .iter()
                .find(|s| s.dir == DEFAULT_SCENE)
                .expect("缺省场景必须在注册表")
        })
}
