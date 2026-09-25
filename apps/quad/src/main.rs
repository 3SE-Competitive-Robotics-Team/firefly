//! 微无人机第三人称飞行 demo（独立，不接 IPC / VIO）。
//!
//! 四旋翼模型与飞控（角度模式）由 `firefly-flight` 提供——与闭环评估共用同一实现；
//! 本 app 只做输入/相机/场景接线，在 `configs/quad.toml` 指定的场地里飞。
//! 第三人称追踪相机与场景/光照/机体可视化复用 `firefly-render`（与 `apps/render`
//! 同一套基建）。这是后续「在真实动态下调试 VIO/感知」的可玩基座。
//!
//! 运行：`cargo run --release -p quad`

mod config;
mod quad;

use bevy::asset::AssetPlugin;
use bevy::prelude::*;
use bevy::text::FontSize;
use bevy::window::{Window, WindowPlugin};

use config::QuadConfig;
use firefly_render::camera::{FollowCamera, FollowTarget, follow_camera};
use firefly_render::lighting::{ILLUMINANCE, spawn_scene_lighting, viewer_camera};
use firefly_render::scene::{MODELS_DIR, spawn_scene};
use quad::{Quad, QuadInput};

/// 配置（编译期绝对路径）。
const CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../configs/quad.toml");

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
        .add_systems(Update, (quad::read_input, follow_camera, update_hud))
        .run();
}

/// 场地 + 光照 + 主相机。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn setup_scene(mut commands: Commands, cfg: Res<QuadConfig>, assets: Res<AssetServer>) {
    spawn_scene_lighting(&mut commands, ILLUMINANCE);
    spawn_scene(&mut commands, &assets, &cfg.field);
    commands.spawn((
        Camera3d::default(),
        viewer_camera(),
        FollowCamera,
        Transform::from_xyz(-2.0, 0.0, 2.0).looking_at(Vec3::ZERO, Vec3::Z),
    ));
}

/// 机体根实体 + 模型子节点（模型/标记由 `firefly-render` 统一挂）。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn setup_drone(
    mut commands: Commands,
    cfg: Res<QuadConfig>,
    assets: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn((
            Quad::default(),
            FollowTarget,
            Transform::from_translation(Vec3::from(cfg.start)),
            Visibility::default(),
        ))
        .with_children(|body| {
            firefly_render::drone::spawn_drone_visual(body, &assets, &mut meshes, &mut materials);
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
        quad.state.velocity.length(),
        transform.translation.z
    );
    if text.as_str() != want {
        text.0.clear();
        text.0.push_str(&want);
    }
}
