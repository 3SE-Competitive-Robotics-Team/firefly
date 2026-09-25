//! `Bevy` 视觉渲染进程：位姿订阅 + 双目/深度渲染 + 发布 + 调试显示。
//!
//! 分工：物理（`MuJoCo` 步进、`IMU`/真值发布、`PD` 控制）仍在 `firefly-sim`；
//! 本进程是无状态渲染 worker——订阅 `Firefly/GroundTruth` 摆传感器 rig，
//! 离屏渲染左/右 `RGB` 与深度预通道，回读后发布 `Firefly/CameraLeft`、
//! `Firefly/CameraRight`、`Firefly/Depth` 并通知 `Firefly/CameraPair`。
//! `VIO`/`planner` 零改动（话题、布局、`trace` 头与 `MuJoCo` 时代一致）。
//!
//! 出图链路按需驱动：传感器相机只在固定 10Hz 出图窗口激活渲染（非窗口帧不空转）；
//! 回读后的逐像素处理（灰度/深度/噪声/调试显示）在工作线程，主线程只做发布
//!（iceoryx2 端口主线程独占）——供图不再被逐像素计算或帧率抖动拖住。
//!
//! 日志后端：`Bevy` 自带 `LogPlugin` 禁用，改用 `firefly-observability`
//!（`logforth` + `fastrace`，跨进程 trace 续接与 `rrd` 日志聚合的载体）。
//!
//! 运行（仓库根，先起 `sim` 再起本进程）：
//! ```sh
//! uv run firefly-sim -- --no-camera
//! cargo run --release -p render
//! ```
//!
//! `structure.glb` 由同目录 `structure.obj` 转换而来（`models/` 不进
//! git，缺失时重转）：
//! `uv run --group dev python -c "import trimesh;
//! trimesh.load('models/warehouse/structure.obj',
//! force='scene').export('models/warehouse/structure.glb')"`。

mod capture;
mod config;
mod freecam;
mod link;
mod process;
mod rig;
mod scene;
mod sensors;
mod trace_bridge;
mod ui;

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
use firefly_render::lighting::{DIRECTIONS, spawn_viewer_lighting};
use firefly_render::scene::{MODELS_DIR, spawn_scene};
use freecam::{FreeCam, freecam_move, toggle_freecam, update_mode_label};
use link::{
    CapturePipeline, CaptureStats, PendingFrames, SensorCapture, drain_captures, open_ports,
    poll_pose, publish_processed,
};
use rig::{DRONE_LAYER, PoseState, SENSOR_LIGHT_LAYER, VIEWER_LIGHT_LAYER, spawn_rig};
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
        MODELS_DIR,
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
                    file_path: MODELS_DIR.to_owned(),
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
        .insert_resource(CapturePipeline::spawn())
        .insert_resource(CaptureStats::default())
        .insert_resource(FreeCam::default())
        .insert_non_send(ports)
        .add_plugins(CapturePlugin)
        .add_systems(Startup, (setup_scene, spawn_rig, setup_panel))
        .add_systems(PreUpdate, poll_pose)
        .add_systems(
            Update,
            (
                follow_camera.run_if(freecam_off),
                follow_drone,
                tag_drone_layers,
                log_render_rate,
            ),
        )
        .add_systems(
            Update,
            (toggle_freecam, freecam_move, update_mode_label).chain(),
        )
        .add_systems(
            Last,
            (drain_captures, publish_processed, flush_on_exit).chain(),
        )
        .run();
}

/// 场景装配：注册表选中场景的视觉 glb、传感器相机照明、主视角共享照明与无人机
/// 占位体。红蓝灯饰自发光由 glb 的 emissive 材质承载（Blender 阶段写入），
/// 主视角配 bloom 出光感（bloom 随共享效果）。
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

    // 传感器相机专用方向光（不投影）：照度锚定直射日光量级（`DirectionalLight`
    // 文档：直射日光 32000~100000 lux），地面灰度均值须落在 120~170/255（对照
    // `MuJoCo` 实测 140~160），否则 VIO 无特征可跟——改照度后以发布图像均值人工
    // 比对。方向固定，照度由 `configs/render.toml` 的 `light.directional` 调；
    // 单独成层，主视角不吃这一套（互不叠加）。
    for (index, dir) in DIRECTIONS.iter().enumerate() {
        commands.spawn((
            DirectionalLight {
                illuminance: render_config.light.directional[index],
                shadow_maps_enabled: false,
                ..default()
            },
            RenderLayers::layer(SENSOR_LIGHT_LAYER),
            Transform::from_rotation(Quat::from_rotation_arc(Vec3::NEG_Z, dir.normalize())),
        ));
    }

    // 主视角照明：与 `apps/quad` 同一套（环境光 + 三方向光 + 曝光 + bloom），
    // 收敛在 `firefly-render::lighting`，两边不再各自定制。灯光收进
    // `VIEWER_LIGHT_LAYER`，与传感器灯光分开、不互相叠加。
    spawn_viewer_lighting(&mut commands, &RenderLayers::layer(VIEWER_LIGHT_LAYER));

    // 顶棚灯阵：rows × cols 盏点光源均匀铺在场地上方，营造场馆照明。点光源
    // 不投阴影——一盏点光阴影 = 6 面 cubemap，数十盏会直接压垮渲染。只作用于
    // 传感器相机（主视角观感由共享效果决定）。
    let grid = &render_config.light.grid;
    let color = Color::srgb(grid.color[0], grid.color[1], grid.color[2]);
    for row in 0..grid.rows {
        for col in 0..grid.cols {
            commands.spawn((
                PointLight {
                    color,
                    intensity: grid.intensity,
                    range: grid.range,
                    ..default()
                },
                RenderLayers::layer(SENSOR_LIGHT_LAYER),
                Transform::from_xyz(
                    grid_axis(col, grid.cols, grid.span[0]),
                    grid_axis(row, grid.rows, grid.span[1]),
                    grid.height,
                ),
            ));
        }
    }
    log::info!(
        "顶棚灯阵 {}×{} = {} 盏（高 {:.1}m，单灯 {:.0}lm，半径 {:.1}m）",
        grid.rows,
        grid.cols,
        grid.rows * grid.cols,
        grid.height,
        grid.intensity,
        grid.range
    );

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

/// 机体可视化根（`follow_drone` 每帧贴真值位姿；对照 `apps/quad` 的机体根）。
#[derive(Component)]
struct DroneVisual;

/// 机体模型贴到真值位姿（机体→世界，Hamilton；与传感器相机同源 `PoseState`）。
// 系统参数按值传递（`SystemParam` 契约）。
#[allow(clippy::needless_pass_by_value)]
fn follow_drone(pose: Res<PoseState>, mut drone: Query<&mut Transform, With<DroneVisual>>) {
    if !pose.has_pose {
        return;
    }
    for mut transform in &mut drone {
        transform.translation = pose.pos;
        transform.rotation = pose.quat;
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

/// 灯阵单轴布点（中心为原点，`count` 盏均布在 `span` 上；单盏居中）。
fn grid_axis(index: u32, count: u32, span: f32) -> f32 {
    if count <= 1 {
        0.0
    } else {
        -span / 2.0 + span * index as f32 / (count - 1) as f32
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
