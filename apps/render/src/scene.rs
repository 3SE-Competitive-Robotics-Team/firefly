//! RMUC 场景资产（赛季 + 场地版本）与初始摆位；配置与 sim / viz 共用。

use bevy::prelude::*;
use serde::Deserialize;

/// 世界观配置（与 `sim` / `viz` 共用）。
const SCENE_CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../configs/scene.toml");

/// 一个已注册的场地版本。
#[derive(Resource, Clone, Copy, Debug)]
pub struct SceneSpec {
    /// 配置取值，`<赛季>@<版本>`。
    pub name: &'static str,
    /// `models/` 下的版本目录。
    pub dir: &'static str,
    /// 目录内视觉 glb 文件名。
    pub visual: &'static str,
    /// 机体起点（米，`MuJoCo` 系；真值到达前的初始摆位）。
    pub start: [f32; 3],
}

impl SceneSpec {
    /// 相对 [`firefly_render::scene::MODELS_DIR`] 的 asset 路径（对照 `AssetPlugin.file_path`）。
    #[must_use]
    pub fn asset_path(&self) -> String {
        format!("{}/{}", self.dir, self.visual)
    }
}

/// 默认场景；必须与 `configs/scene.toml` 的缺省一致。
pub const DEFAULT_SCENE: &str = "rmuc2027@1.0.0";

/// 已接入的场地版本。新增版本：目录 `models/<赛季>/v<版本>/` 加一行。
const SCENES: &[SceneSpec] = &[
    SceneSpec {
        name: "rmuc2026@2.0.0",
        dir: "rmuc2026/v2.0.0",
        visual: "field.glb",
        start: [-13.0, 0.0, 0.405],
    },
    SceneSpec {
        name: "rmuc2027@1.0.0",
        dir: "rmuc2027/v1.0.0",
        visual: "field.glb",
        start: [-13.0, 0.0, 0.405],
    },
];

fn resolve(name: &str) -> Option<SceneSpec> {
    SCENES.iter().find(|spec| spec.name == name).copied()
}

/// `configs/scene.toml` 的反序列化形态。
#[derive(Deserialize)]
struct SceneConfig {
    #[serde(default = "default_scene")]
    scene: String,
}

fn default_scene() -> String {
    DEFAULT_SCENE.into()
}

/// 读取场景；缺配置使用默认场景，读取/解析失败报错退出。
fn load_scene_name() -> String {
    firefly_config::load::<SceneConfig>(SCENE_CONFIG)
        .unwrap_or_else(|e| {
            log::error!("加载 configs/scene.toml 失败：{e}");
            std::process::exit(1);
        })
        .scene
}

/// 解析场景；未注册的版本或缺失视觉资产必须报错退出。
#[must_use]
pub fn selected() -> SceneSpec {
    let name = load_scene_name();
    let Some(spec) = resolve(&name) else {
        let supported: Vec<_> = SCENES.iter().map(|s| s.name).collect();
        log::error!("未注册的场景 {name}；可用：{}", supported.join(", "));
        std::process::exit(1);
    };
    let path = std::path::Path::new(firefly_render::scene::MODELS_DIR).join(spec.asset_path());
    if !path.is_file() {
        log::error!("场景视觉资产缺失：{}", path.display());
        std::process::exit(1);
    }
    spec
}
