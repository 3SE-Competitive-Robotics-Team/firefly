//! 场景资产加载：`models/` 资产根 + 视觉 glTF。

use bevy::gltf::GltfAssetLabel;
use bevy::prelude::*;

/// 场景资产根（编译期绝对路径，与运行 `CWD` 无关；两个可视化应用共用）。
pub const MODELS_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../models");

/// 加载场景视觉 glTF（路径相对 [`MODELS_DIR`]）。
pub fn spawn_scene(commands: &mut Commands, assets: &AssetServer, visual: &str) {
    commands.spawn(WorldAssetRoot(
        assets.load(GltfAssetLabel::Scene(0).from_asset(visual.to_owned())),
    ));
}
