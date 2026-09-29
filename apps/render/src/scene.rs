//! RMUC2026 场景资产与初始摆位；配置与 sim / viz 共用。

use bevy::prelude::*;
use serde::Deserialize;

/// 世界观配置（与 `sim` / `viz` 共用）。
const SCENE_CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../configs/scene.toml");

/// RMUC 视觉资产与初始摆位。
#[derive(Resource, Clone, Copy, Debug)]
pub struct SceneSpec {
    /// `models/` 下的目录名（与 `configs/scene.toml` 的 `scene` 取值一致）。
    pub dir: &'static str,
    /// 目录内视觉 glb 文件名。
    pub visual: &'static str,
    /// 机体起点（米，`MuJoCo` 系；真值到达前的初始摆位，对照 `configs/sim.toml`）。
    pub start: [f32; 3],
}

impl SceneSpec {
    /// 相对 [`firefly_render::scene::MODELS_DIR`] 的 asset 路径（对照 `AssetPlugin.file_path`）。
    #[must_use]
    pub fn asset_path(&self) -> String {
        format!("{}/{}", self.dir, self.visual)
    }
}

/// 唯一支持的场景。
pub const DEFAULT_SCENE: &str = "rmuc2026";

const RMUC: SceneSpec = SceneSpec {
    dir: "rmuc2026",
    visual: "field.glb",
    start: [-13.0, 0.0, 0.405],
};

/// `configs/scene.toml` 的反序列化形态。
#[derive(Deserialize)]
struct SceneConfig {
    scene: String,
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

/// 加载 RMUC；不支持的场景或缺失视觉资产必须报错退出。
#[must_use]
pub fn selected() -> SceneSpec {
    let name = load_scene_name();
    if name != DEFAULT_SCENE {
        log::error!("仅支持场景 {DEFAULT_SCENE}，收到 {name}");
        std::process::exit(1);
    }
    let path = std::path::Path::new(firefly_render::scene::MODELS_DIR).join(RMUC.asset_path());
    if !path.is_file() {
        log::error!("RMUC 视觉资产缺失：{}", path.display());
        std::process::exit(1);
    }
    RMUC
}
