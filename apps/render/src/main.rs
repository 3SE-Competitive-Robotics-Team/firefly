//! RMUC 视觉进程：订阅仿真位姿，渲染双目与深度，发布 IPC；调试展示由独立 firefly-viz 承担。
//!
//! sim 负责 `MuJoCo` 物理和 IMU；本进程从 `Firefly/GroundTruth` 摆放传感器，
//! 发布 `Firefly/CameraLeft`、`Firefly/CameraRight`、`Firefly/Depth`，
//! 通过 `Firefly/CameraPair` 唤醒订阅端。真值只用于生成仿真测量。
//! 视觉资产必须存在于 `models/rmuc2026/v2.0.0/field.glb`。
//!
//! 传感器相机按 30Hz 窗口激活，逐像素处理与传感器发布在工作线程完成。
//! 日志经 `firefly-observability` 汇入统一 rrd 记录。
//! `--offline` 使用独立 `Firefly/Offline/*` 数据话题供离线库图渲染。

mod buffer;
mod capture;
mod config;
mod depth_noise;
mod grayscale;
mod link;
mod process;
mod rig;
mod scene;
mod sensors;
mod trace_bridge;

use bevy::app::ScheduleRunnerPlugin;
use bevy::asset::AssetPlugin;
use bevy::log::LogPlugin;
use bevy::prelude::*;
use bevy::window::{ExitCondition, WindowPlugin};
use bevy::winit::WinitPlugin;

use capture::{CaptureHub, CapturePlugin};
use config::RenderConfig;
use firefly_render::lighting::spawn_scene_lighting;
use firefly_render::scene::{MODELS_DIR, spawn_scene};
use link::{
    CapturePipeline, CaptureStats, PendingFrames, SensorCapture, drain_captures, open_ports,
    poll_pose, poll_processor,
};
use rig::{PoseState, spawn_rig};
use scene::SceneSpec;

/// 以 RMUC 资产和初始摆位启动 Bevy；仿真位姿到达后更新机体与传感器。
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    let export_path = args
        .iter()
        .position(|arg| arg == "--export-asset-contract")
        .map(|index| args.get(index + 1).ok_or("missing asset contract path"))
        .transpose()?;
    let offline = args.iter().any(|arg| arg == "--offline");
    let render_config = firefly_config::startup::<RenderConfig>(
        config::RENDER_CONFIG,
        || {
            firefly_observability::init();
            if let Err(e) = trace_bridge::init() {
                log::warn!("tracing 转发安装失败（Bevy 侧日志将静默）: {e}");
            }
        },
        |config| config.depth_noise.validate(),
    )?;
    let spec = scene::selected();
    if let Some(path) = export_path {
        std::fs::write(
            path,
            toml::to_string_pretty(&render_config.asset_contract())?,
        )?;
        return Ok(());
    }

    log::info!(
        "场景 {}：asset root {}，视觉 {}，起点 {:?}",
        spec.dir,
        MODELS_DIR,
        spec.asset_path(),
        spec.start
    );
    let ports = open_ports(offline).unwrap_or_else(|e| {
        log::error!("IPC 端口打开失败：{e:?}");
        std::process::exit(1);
    });
    let log_ipc = firefly_observability::init_ipc(&ports.node, "render");
    build_app(spec, &render_config, ports, log_ipc, offline)
        .unwrap_or_else(|error| {
            log::error!("render 初始化失败: {error}");
            std::process::exit(1);
        })
        .run();
    Ok(())
}

/// 组装 Bevy 应用（插件、资源与系统注册）；`main` 只做参数、日志与 IPC 准备。
fn build_app(
    spec: SceneSpec,
    render_config: &RenderConfig,
    ports: link::IpcPorts,
    log_ipc: firefly_observability::LogIpc,
    offline: bool,
) -> Result<App, firefly_error::Error> {
    let mut app = App::new();
    app.add_plugins(
        DefaultPlugins
            .build()
            .disable::<LogPlugin>()
            .set(AssetPlugin {
                file_path: MODELS_DIR.to_owned(),
                ..default()
            })
            .disable::<WinitPlugin>()
            .set(WindowPlugin {
                primary_window: None,
                exit_condition: ExitCondition::DontExit,
                ..default()
            }),
    )
    .add_plugins(ScheduleRunnerPlugin::run_loop(
        std::time::Duration::from_secs_f64(1.0 / 120.0),
    ))
    .insert_resource(spec)
    .insert_resource(render_config.clone())
    .insert_resource(PoseState {
        pos: Vec3::from(spec.start),
        ..default()
    })
    .insert_resource(CaptureHub::default())
    .insert_resource(PendingFrames::default())
    .insert_resource(SensorCapture::default())
    .insert_resource(CapturePipeline::spawn(render_config.depth_noise, offline)?)
    .insert_resource(CaptureStats::default())
    .insert_non_send(ports)
    .insert_non_send(log_ipc)
    .add_plugins(CapturePlugin)
    .add_plugins(grayscale::GrayscalePlugin)
    .add_systems(Startup, (setup_scene, spawn_rig))
    .add_systems(PreUpdate, poll_pose)
    .add_systems(Update, (log_render_rate, stop_on_signal))
    .add_systems(
        Last,
        (drain_captures, poll_processor, pump_logs, flush_on_exit).chain(),
    );
    Ok(app)
}

#[allow(clippy::needless_pass_by_value)]
fn pump_logs(logs: NonSend<firefly_observability::LogIpc>) {
    firefly_observability::pump_log_ipc(&logs);
}

/// 传感器场景只包含真实几何与共享光照。
#[allow(clippy::needless_pass_by_value)]
fn setup_scene(
    mut commands: Commands,
    assets: Res<AssetServer>,
    scene: Res<SceneSpec>,
    render_config: Res<RenderConfig>,
) {
    spawn_scene(&mut commands, &assets, &scene.asset_path());
    spawn_scene_lighting(&mut commands, render_config.light.directional);
    log::info!("场景 {} render ready", scene.dir);
}

/// 出图节奏诊断：每 [`RATE_LOG_PERIOD`] 秒报一次应用帧率与**真实供图速率**。
/// 供图目标 30Hz（传感器节拍）；帧率是供给侧余量，供图才是 VIO 拿到的输入。
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
