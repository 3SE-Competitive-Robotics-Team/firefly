//! 微无人机第三人称飞行 demo（独立，不接 IPC / VIO）。
//!
//! 250g 级四旋翼 6-DOF 动力学 + 角度模式 `WASD` 控制，在 `configs/quad.toml`
//! 指定的场地里飞。第三人称追踪相机。这是后续「在真实动态下调试 VIO/感知」的
//! 可玩基座。
//!
//! 运行：`cargo run -p quad`

mod camera;
mod config;
mod quad;

use std::f32::consts::FRAC_PI_2;

use bevy::asset::AssetPlugin;
use bevy::camera::Exposure;
use bevy::gltf::GltfAssetLabel;
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;
use bevy::text::FontSize;
use bevy::window::{Window, WindowPlugin};

use config::QuadConfig;
use quad::{Quad, QuadInput};

/// 场地资产根（编译期绝对路径，与运行 `CWD` 无关）。
const MODELS_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../models");
/// 配置（编译期绝对路径）。
const CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../configs/quad.toml");
/// 全局环境光（cd/m²；与 `apps/render` 同量级）。
const AMBIENT: f32 = 30.0;
/// 相机曝光 EV100（与三方向光 lux 量级匹配）。
const EV100: f32 = 15.0;

fn main() {
    let cfg = config::load(CONFIG);
    log::info!("quad：场地 {}，起点 {:?}", cfg.field, cfg.start);
    App::new()
        .add_plugins(
            DefaultPlugins
                .set(AssetPlugin {
                    file_path: MODELS_DIR.to_owned(),
                    ..default()
                })
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        resolution: (1280, 720).into(),
                        title: "firefly quad".to_owned(),
                        ..default()
                    }),
                    ..default()
                }),
        )
        .insert_resource(cfg)
        .insert_resource(QuadInput::default())
        // 动力学定步长 240Hz（显式积分稳定）。
        .insert_resource(Time::<Fixed>::from_hz(240.0))
        .add_systems(Startup, (setup_scene, setup_drone, setup_hud))
        .add_systems(FixedUpdate, quad::dynamics)
        .add_systems(Update, (quad::read_input, camera::chase, update_hud))
        .run();
}

/// 场地 + 光照 + 主相机。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn setup_scene(mut commands: Commands, cfg: Res<QuadConfig>, assets: Res<AssetServer>) {
    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: AMBIENT,
        ..default()
    });
    for (dir, lux) in [
        (Vec3::new(-0.3, -0.25, -0.92), 32000.0),
        (Vec3::new(-0.15, 0.6, -0.78), 14000.0),
        (Vec3::new(0.75, 0.1, -0.65), 10000.0),
    ] {
        commands.spawn((
            DirectionalLight {
                illuminance: lux,
                ..default()
            },
            Transform::from_rotation(Quat::from_rotation_arc(Vec3::NEG_Z, dir.normalize())),
        ));
    }
    commands.spawn(WorldAssetRoot(
        assets.load(GltfAssetLabel::Scene(0).from_asset(cfg.field.clone())),
    ));
    commands.spawn((
        Camera3d::default(),
        Exposure { ev100: EV100 },
        Bloom {
            intensity: 0.08,
            ..default()
        },
        Transform::from_xyz(-2.0, 0.0, 2.0).looking_at(Vec3::ZERO, Vec3::Z),
    ));
}

/// 机体根实体 + 模型子节点 + 前向标记。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn setup_drone(
    mut commands: Commands,
    cfg: Res<QuadConfig>,
    assets: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // 模型：glTF Y-up、机头 +Z → 世界 Z-up、机头 +X。
    let model_rot = Quat::from_rotation_z(FRAC_PI_2) * Quat::from_rotation_x(FRAC_PI_2);
    commands
        .spawn((
            Quad::default(),
            Transform::from_translation(Vec3::from(cfg.start)),
            Visibility::default(),
        ))
        .with_children(|body| {
            body.spawn((
                WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset("drone/drone.glb"))),
                Transform::from_rotation(model_rot),
            ));
            // 前向标记（绿块：机头 +X），模型朝向存疑时一眼可辨。
            body.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.03, 0.012, 0.012))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: Color::srgb(0.1, 0.9, 0.1),
                    emissive: LinearRgba::new(0.1, 3.0, 0.1, 1.0),
                    ..default()
                })),
                Transform::from_xyz(cfg.drone.arm + 0.04, 0.0, 0.0),
            ));
        });
}

/// HUD 文本（`update_hud` 更新）。
#[derive(Component)]
struct Hud;

/// 左上角提示。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn setup_hud(mut commands: Commands) {
    commands.spawn((
        Text::new("WASD tilt   Space/Shift up-down   Q/E yaw   R reset"),
        TextFont {
            font_size: FontSize::Px(16.0),
            ..default()
        },
        TextColor(Color::srgb(0.92, 0.92, 0.92)),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(12.0),
            top: Val::Px(10.0),
            ..default()
        },
        Hud,
    ));
}

/// HUD：速度 / 高度（仅变化时写，避免每帧触发 UI 变更检测）。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn update_hud(drone: Single<(&Quad, &Transform)>, mut text: Single<&mut Text, With<Hud>>) {
    let (quad, transform) = drone.into_inner();
    let want = format!(
        "speed {:.1} m/s   alt {:.1} m\nWASD tilt   Space/Shift up-down   Q/E yaw   R reset",
        quad.velocity.length(),
        transform.translation.z
    );
    if text.as_str() != want {
        text.0.clear();
        text.0.push_str(&want);
    }
}
