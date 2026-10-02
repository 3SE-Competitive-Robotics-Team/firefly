//! RMUC 视觉进程：订阅仿真位姿，渲染双目与深度，发布 IPC 并显示调试面板。
//!
//! sim 负责 `MuJoCo` 物理和 IMU；本进程从 `Firefly/GroundTruth` 摆放传感器，
//! 发布 `Firefly/CameraLeft`、`Firefly/CameraRight`、`Firefly/Depth`，
//! 通过 `Firefly/CameraPair` 唤醒订阅端。真值只用于生成仿真测量。
//! 视觉资产必须存在于 `models/rmuc2026/field.glb`。
//!
//! 传感器相机按 10Hz 窗口激活，逐像素处理在工作线程完成，IPC 端口由主线程持有。
//! 日志经 `firefly-observability` 汇入统一 rrd 记录。
//! `--offline` 使用独立 `Firefly/Offline/*` 数据话题供离线库图渲染。

mod capture;
mod config;
mod depth_noise;
mod freecam;
mod link;
mod process;
mod rig;
mod scene;
mod sensors;
mod trace_bridge;
mod ui;
mod viewer_pose;

use bevy::asset::AssetPlugin;
use bevy::camera::visibility::RenderLayers;
use bevy::log::LogPlugin;
use bevy::prelude::*;
use bevy::window::{Window, WindowPlugin};
use bevy::winit::WinitSettings;

use capture::{CaptureHub, CapturePlugin};
use config::RenderConfig;
use firefly_render::camera::{FollowTarget, follow_camera};
use firefly_render::drone::spawn_drone_visual;
use firefly_render::lighting::spawn_scene_lighting;
use firefly_render::scene::{MODELS_DIR, spawn_scene};
use freecam::{FreeCam, freecam_move, toggle_freecam, update_mode_label};
use link::{
    CapturePipeline, CaptureStats, PendingFrames, SensorCapture, drain_captures, open_ports,
    poll_pose, publish_processed,
};
use rig::{DRONE_LAYER, PoseState, spawn_rig};
use scene::SceneSpec;
use ui::setup_panel;

/// 以 RMUC 资产和初始摆位启动 Bevy；仿真位姿到达后更新机体与传感器。
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
    let args: Vec<_> = std::env::args().collect();
    if let Some(index) = args.iter().position(|arg| arg == "--export-asset-contract") {
        let outcome = args
            .get(index + 1)
            .ok_or("missing asset contract path".to_owned())
            .and_then(|path| {
                toml::to_string_pretty(&render_config.asset_contract())
                    .map_err(|error| error.to_string())
                    .and_then(|text| std::fs::write(path, text).map_err(|error| error.to_string()))
            });
        if let Err(error) = outcome {
            log::error!("资产契约导出失败: {error}");
            std::process::exit(1);
        }
        return;
    }

    log::info!(
        "场景 {}：asset root {}，视觉 {}，起点 {:?}",
        spec.dir,
        MODELS_DIR,
        spec.asset_path(),
        spec.start
    );
    let offline = std::env::args().any(|arg| arg == "--offline");
    let ports = open_ports(offline).unwrap_or_else(|e| {
        log::error!("IPC 端口打开失败：{e:?}");
        std::process::exit(1);
    });
    let log_ipc = firefly_observability::init_ipc(&ports.node, "render");
    App::new()
        .add_plugins(
            DefaultPlugins
                .build()
                .disable::<LogPlugin>()
                .set(AssetPlugin {
                    file_path: MODELS_DIR.to_owned(),
                    ..default()
                })
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        resolution: (1280, 720).into(),
                        title: "firefly render".to_owned(),
                        // 传感器节拍不得等待桌面合成器的垂直同步。
                        present_mode: bevy::window::PresentMode::AutoNoVsync,
                        ..default()
                    }),
                    ..default()
                }),
        )
        .insert_resource(spec)
        .insert_resource(render_config)
        // 传感器渲染是计算链路的一环，不是可挂起的桌面窗口：失焦时也必须持续
        // 出图（Bevy 缺省失焦降到 1Hz，VIO 会拿到断流的图像而失稳）。
        .insert_resource(WinitSettings::continuous())
        .insert_resource(PoseState {
            pos: Vec3::from(spec.start),
            ..default()
        })
        .insert_resource(CaptureHub::default())
        .insert_resource(PendingFrames::default())
        .insert_resource(SensorCapture::default())
        .insert_resource(CapturePipeline::spawn(render_config.depth_noise, offline))
        .insert_resource(CaptureStats::default())
        .insert_resource(FreeCam::default())
        .insert_resource(viewer_pose::ViewerPose::default())
        .insert_non_send(ports)
        .insert_non_send(log_ipc)
        .add_plugins(CapturePlugin)
        .add_systems(Startup, (setup_scene, spawn_rig, setup_panel))
        .add_systems(PreUpdate, poll_pose)
        .add_systems(
            Update,
            (
                follow_camera.run_if(freecam_off).after(follow_drone),
                follow_drone,
                tag_drone_layers,
                log_render_rate,
                stop_on_signal,
            ),
        )
        .add_systems(
            Update,
            (toggle_freecam, freecam_move, update_mode_label).chain(),
        )
        .add_systems(
            Last,
            (drain_captures, publish_processed, pump_logs, flush_on_exit).chain(),
        )
        .run();
}

#[allow(clippy::needless_pass_by_value)]
fn pump_logs(logs: NonSend<firefly_observability::LogIpc>) {
    firefly_observability::pump_log_ipc(&logs);
}

/// 场景装配：选中场景的视觉 glb + 场景光照（唯一一份，见
/// `firefly_render::lighting`）+ 机体占位体；自发光由 glb 的 emissive 材质承载。
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
    spawn_scene(&mut commands, &assets, &scene.asset_path());

    // 场景光照：唯一一份，所有相机共用（约束见 `firefly_render::lighting`）。
    spawn_scene_lighting(&mut commands, render_config.light.directional);

    let start = Vec3::from(scene.start);

    // 机体可视化（`firefly-render` 统一挂 `drone.glb` + 前向标记）：`follow_drone`
    // 每帧贴真值位姿。只进主视角（`DRONE_LAYER`）：传感器相机位于机体内，机体模型
    // 必须对它们不可见，否则自遮挡传感器图（glTF 子实体层由 `tag_drone_layers` 补）。
    commands
        .spawn((
            DroneVisual,
            FollowTarget,
            RenderLayers::layer(DRONE_LAYER),
            Transform::from_translation(start),
            Visibility::default(),
        ))
        .with_children(|body| {
            spawn_drone_visual(body, &assets, &mut meshes, &mut materials);
        });

    log::info!("场景 {} render ready", scene.dir);
}

/// 机体可视化根；仅用于主视图的延迟位姿插值。
#[derive(Component)]
struct DroneVisual;

/// 机体显示在相邻真值样本间插值；显示延迟不进入传感器 rig。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn follow_drone(
    pose: Res<viewer_pose::ViewerPose>,
    time: Res<Time>,
    mut drone: Query<&mut Transform, With<DroneVisual>>,
) {
    if let Some(display) = pose.sample(time.elapsed_secs_f64()) {
        for mut transform in &mut drone {
            *transform = display;
        }
    }
}

/// 机体模型（`drone.glb`）子树打到 `DRONE_LAYER`：glTF 场景子实体不继承父层，
/// 缺一步它们会留在默认层被传感器相机看到。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn tag_drone_layers(
    roots: Query<Entity, With<DroneVisual>>,
    children: Query<&Children>,
    tagged: Query<(), With<RenderLayers>>,
    mut commands: Commands,
) {
    for root in &roots {
        let mut stack = vec![root];
        while let Some(entity) = stack.pop() {
            if tagged.get(entity).is_err() {
                commands
                    .entity(entity)
                    .insert(RenderLayers::layer(DRONE_LAYER));
            }
            if let Ok(kids) = children.get(entity) {
                stack.extend(kids.iter());
            }
        }
    }
}

/// 自由浏览模式下第三人称追踪让位给 `freecam::freecam_move`。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn freecam_off(free: Res<FreeCam>) -> bool {
    !free.enabled
}

/// 出图节奏诊断：每 [`RATE_LOG_PERIOD`] 秒报一次应用帧率与**真实供图速率**。
/// 供图目标 10Hz（传感器节拍）；帧率是供给侧余量，供图才是 VIO 拿到的输入。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn log_render_rate(time: Res<Time>, mut acc: Local<(f32, u32)>, mut stats: ResMut<CaptureStats>) {
    acc.0 += time.delta_secs();
    acc.1 += 1;
    if acc.0 >= RATE_LOG_PERIOD {
        let frames = f64::from(acc.1) / f64::from(acc.0);
        let captures = f64::from(stats.published) / f64::from(acc.0);
        log::info!(
            "render 帧率 {frames:.1} Hz，供图 {captures:.1} Hz（本周期 {} 拍）",
            stats.published
        );
        *acc = (0.0, 0);
        stats.published = 0;
    }
}

/// 出图节奏诊断周期（秒）。
const RATE_LOG_PERIOD: f32 = 5.0;

/// 退出时刷 trace（`Ctrl-C` → `AppExit`，端口随后按 `Drop` 纪律释放）。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn flush_on_exit(exits: MessageReader<AppExit>) {
    if !exits.is_empty() {
        firefly_observability::flush();
    }
}

/// 将 IPC 节点的终止请求交给 Bevy 正常退出，释放发布端口与 GPU 资源。
#[allow(clippy::needless_pass_by_value)]
fn stop_on_signal(ports: NonSend<link::IpcPorts>, mut exits: MessageWriter<AppExit>) {
    if ports.node.wait(std::time::Duration::ZERO).is_err() {
        exits.write(AppExit::Success);
    }
}
