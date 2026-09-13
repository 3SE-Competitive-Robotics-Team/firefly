//! `Bevy` 视觉渲染进程：位姿订阅 + 双目/深度渲染 + 发布 + 调试显示。
//!
//! 分工：物理（`MuJoCo` 步进、`IMU`/真值发布、`PD` 控制）仍在 `firefly-sim`；
//! 本进程是无状态渲染 worker——订阅 `Firefly/GroundTruth` 摆传感器 rig，
//! 离屏渲染左/右 `RGB` 与深度预通道，回读后发布 `Firefly/CameraLeft`、
//! `Firefly/CameraRight`、`Firefly/Depth` 并通知 `Firefly/CameraPair`。
//! `VIO`/`planner` 零改动（话题、布局、`trace` 头与 `MuJoCo` 时代一致）。
//!
//! 日志后端：`Bevy` 自带 `LogPlugin` 禁用，改用 `firefly-observability`
//!（`logforth` + `fastrace`，跨进程 trace 续接与 `rrd` 日志聚合的载体）。
//!
//! 运行（仓库根，先起 `sim` 再起本进程）：
//! ```sh
//! uv run firefly-sim -- --no-camera
//! cargo run -p render
//! ```
//!
//! `structure.glb` 由同目录 `structure.obj` 转换而来（`models/` 不进
//! git，缺失时重转）：
//! `uv run --group dev python -c "import trimesh;
//! trimesh.load('models/warehouse/structure.obj',
//! force='scene').export('models/warehouse/structure.glb')"`。

mod capture;
mod config;
mod link;
mod rig;
mod scene;
mod sensors;
mod trace_bridge;
mod ui;

use bevy::asset::AssetPlugin;
use bevy::log::LogPlugin;
use bevy::prelude::*;
use bevy::window::{Window, WindowPlugin};

use capture::{CaptureHub, CapturePlugin};
use config::RenderConfig;
use link::{PendingFrames, drain_captures, open_ports, poll_pose};
use rig::{PoseState, follow_main, spawn_rig};
use scene::SceneSpec;
use ui::setup_panel;

/// 无人机起点由场景注册表提供（`models/<dir>` 与起点见 [`scene`]；对照
/// `configs/sim.toml`）。asset root 恒为 `models/`，视觉恒为
/// `models/<dir>/<visual>`——换场地只加注册行 + 目录。
fn main() {
    firefly_observability::init();
    // `Bevy` 内部日志走 `tracing`（`LogPlugin` 已禁用，不再安装 subscriber）：
    // 经 `trace_bridge` 转发到 `log` 门面，统一由 `logforth` 输出，否则
    // `Bevy` 侧所有日志（窗口/渲染器/退出原因）静默丢失，崩溃时无现场。
    // 必须在任何 dispatcher 抢占者之前安装，失败即报错不停服。
    if let Err(e) = trace_bridge::init() {
        log::warn!("tracing 转发安装失败（Bevy 侧日志将静默）: {e}");
    }
    let spec = scene::selected();
    let render_config = config::load();
    log::info!(
        "场景 {}：asset root {}，视觉 {}，起点 {:?}",
        spec.dir,
        scene::MODELS_DIR,
        spec.asset_path(),
        spec.start
    );
    let ports = open_ports().unwrap_or_else(|e| {
        log::error!("IPC 端口打开失败：{e:?}");
        std::process::exit(1);
    });
    App::new()
        .add_plugins(
            DefaultPlugins
                .build()
                .disable::<LogPlugin>()
                .set(AssetPlugin {
                    file_path: scene::MODELS_DIR.to_owned(),
                    ..default()
                })
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        resolution: (1280, 720).into(),
                        title: "firefly render".to_owned(),
                        ..default()
                    }),
                    ..default()
                }),
        )
        .insert_resource(spec)
        .insert_resource(render_config)
        .insert_resource(GlobalAmbientLight {
            color: Color::WHITE,
            brightness: render_config.light.ambient,
            ..default()
        })
        .insert_resource(PoseState {
            pos: Vec3::from(spec.start),
            ..default()
        })
        .insert_resource(CaptureHub::default())
        .insert_resource(PendingFrames::default())
        .insert_non_send(ports)
        .add_plugins(CapturePlugin)
        .add_systems(Startup, (setup_scene, spawn_rig, setup_panel))
        .add_systems(PreUpdate, poll_pose)
        .add_systems(Update, follow_main)
        .add_systems(Last, (drain_captures, flush_on_exit))
        .run();
}

/// 场景装配：注册表选中场景的视觉 glb + 环境光 + 三方向光（照度正比于
/// MJCF diffuse）+ 无人机占位体。红蓝灯饰自发光由 glb 的 emissive 材质承载
/// （Blender 阶段写入），Bevy 侧配 bloom 出光感。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn setup_scene(
    mut commands: Commands,
    assets: Res<AssetServer>,
    scene: Res<SceneSpec>,
    render_config: Res<RenderConfig>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(WorldAssetRoot(assets.load(
        bevy::gltf::GltfAssetLabel::Scene(0).from_asset(scene.asset_path()),
    )));

    // 照度锚定直射日光（`DirectionalLight` 文档：直射日光 32000~100000 lux；
    // `MuJoCo` 的 diffuse 无物理单位，只取三灯比值 0.7:0.3:0.22）。方向固定，
    // 照度由 `configs/render.toml` 的 `light.directional` 调。
    // 地面灰度均值须落在 120~170/255（对照 `MuJoCo` 实测 140~160），
    // 否则 VIO 无特征可跟——改照度后以发布图像均值人工比对。
    for (index, dir) in [
        Vec3::new(-0.3, -0.25, -0.92),
        Vec3::new(-0.15, 0.6, -0.78),
        Vec3::new(0.75, 0.1, -0.65),
    ]
    .iter()
    .enumerate()
    {
        commands.spawn((
            DirectionalLight {
                illuminance: render_config.light.directional[index],
                shadow_maps_enabled: index == 0,
                ..default()
            },
            Transform::from_rotation(Quat::from_rotation_arc(Vec3::NEG_Z, dir.normalize())),
        ));
    }

    let start = Vec3::from(scene.start);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.3, 0.3, 0.08))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.7, 0.2),
            ..default()
        })),
        Transform::from_translation(start),
    ));
    log::info!("场景 {} render ready", scene.dir);
}

/// 退出时刷 trace（`Ctrl-C` → `AppExit`，端口随后按 `Drop` 纪律释放）。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn flush_on_exit(exits: MessageReader<AppExit>) {
    if !exits.is_empty() {
        firefly_observability::flush();
    }
}
