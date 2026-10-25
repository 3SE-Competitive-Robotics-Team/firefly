//! 场景资产加载：`models/` 资产根 + 视觉 glTF。

use bevy::gltf::GltfAssetLabel;
use bevy::prelude::*;

/// 场景资产根：按安装布局解析，与启动目录无关（两个可视化应用共用）。
#[must_use]
pub fn models_dir() -> std::path::PathBuf {
    firefly_config::paths::root().join("models")
}

/// 加载场景视觉 glTF（路径相对 [`models_dir`]）。
pub fn spawn_scene(commands: &mut Commands, assets: &AssetServer, visual: &str) {
    commands.spawn(WorldAssetRoot(
        assets.load(GltfAssetLabel::Scene(0).from_asset(visual.to_owned())),
    ));
}
