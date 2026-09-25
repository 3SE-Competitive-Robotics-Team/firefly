//! 机体可视化：`drone.glb` 模型 + 前向标记。

use std::f32::consts::FRAC_PI_2;

use bevy::gltf::GltfAssetLabel;
use bevy::prelude::*;

/// 机体模型路径（相对 `models/`）。
pub const DRONE_MODEL: &str = "drone/drone.glb";
/// 前向标记相对机体原点的前移（米）。
pub const MARKER_FORWARD: f32 = 0.10;

/// 机体模型挂到根实体的子节点：`glTF`（Y-up、机头 `+Z`）→ 世界 Z-up、机头 `+X`，
/// 外加绿色前向标记（模型朝向存疑时一眼可辨）。
pub fn spawn_drone_visual(
    parent: &mut ChildSpawnerCommands,
    assets: &AssetServer,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let model_rot = Quat::from_rotation_z(FRAC_PI_2) * Quat::from_rotation_x(FRAC_PI_2);
    parent.spawn((
        WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset(DRONE_MODEL))),
        Transform::from_rotation(model_rot),
    ));
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.03, 0.012, 0.012))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.1, 0.9, 0.1),
            emissive: LinearRgba::new(0.1, 3.0, 0.1, 1.0),
            ..default()
        })),
        Transform::from_xyz(MARKER_FORWARD, 0.0, 0.0),
    ));
}
